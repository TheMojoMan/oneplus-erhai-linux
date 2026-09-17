// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022, Linaro Limited

#include <dt-bindings/sound/qcom,q6afe.h>
#include <dt-bindings/sound/qcom,q6dsp-lpass-ports.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <linux/soundwire/sdw.h>
#include <sound/jack.h>
#include <linux/input-event-codes.h>
#include "qdsp6/q6afe.h"
#include "common.h"
#include "sdw.h"

struct sc8280xp_snd_data {
	bool stream_prepared[AFE_PORT_MAX];
	struct snd_soc_card *card;
	struct snd_soc_jack jack;
	struct snd_soc_jack dp_jack[8];
	bool jack_setup;
};

/*
 * The AW88261 amplifiers on the secondary TDM bus need a running LPAIF bit
 * clock (8 slots x 32 bit @ 48 kHz) before their PLL-lock check, and the DSP
 * does not vote for it on its own here. Vote for it from the AP side through
 * q6prm when that backend is initialised.
 */
#define SEC_TDM_IBIT_RATE	12288000

static void sc8280xp_vote_sec_tdm_bclk(struct device *dev)
{
	struct of_phandle_args clkspec;
	struct device_node *np;
	struct clk *clk;
	int ret;

	np = of_find_compatible_node(NULL, NULL, "qcom,q6prm-lpass-clocks");
	if (!np) {
		dev_err(dev, "q6prm-lpass-clocks node not found\n");
		return;
	}

	clkspec.np = np;
	clkspec.args_count = 2;
	clkspec.args[0] = LPASS_CLK_ID_SEC_TDM_IBIT;
	clkspec.args[1] = LPASS_CLK_ATTRIBUTE_COUPLE_NO;
	clk = of_clk_get_from_provider(&clkspec);
	of_node_put(np);
	if (IS_ERR(clk)) {
		dev_err(dev, "failed to get SEC_TDM_IBIT: %ld\n", PTR_ERR(clk));
		return;
	}

	ret = clk_set_rate(clk, SEC_TDM_IBIT_RATE);
	if (ret)
		dev_err(dev, "failed to set SEC_TDM_IBIT rate: %d\n", ret);

	ret = clk_prepare_enable(clk);
	if (ret)
		dev_err(dev, "failed to enable SEC_TDM_IBIT: %d\n", ret);
}

static int sc8280xp_snd_init(struct snd_soc_pcm_runtime *rtd)
{
	struct sc8280xp_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_jack *dp_jack  = NULL;
	int dp_pcm_id = 0;

	switch (cpu_dai->id) {
	case PRIMARY_MI2S_RX...QUATERNARY_MI2S_TX:
	case QUINARY_MI2S_RX...QUINARY_MI2S_TX:
		snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_BP_FP);
		if (cpu_dai->id == SECONDARY_MI2S_RX)
			sc8280xp_vote_sec_tdm_bclk(card->dev);
		break;
	case WSA_CODEC_DMA_RX_0:
	case WSA_CODEC_DMA_RX_1:
		/*
		 * Set limit of -3 dB on Digital Volume and 0 dB on PA Volume
		 * to reduce the risk of speaker damage until we have active
		 * speaker protection in place.
		 */
		snd_soc_limit_volume(card, "WSA_RX0 Digital Volume", 81);
		snd_soc_limit_volume(card, "WSA_RX1 Digital Volume", 81);
		snd_soc_limit_volume(card, "SpkrLeft PA Volume", 17);
		snd_soc_limit_volume(card, "SpkrRight PA Volume", 17);
		break;
	case DISPLAY_PORT_RX_0:
		/* DISPLAY_PORT dai ids are not contiguous */
		dp_pcm_id = 0;
		dp_jack = &data->dp_jack[dp_pcm_id];
		break;
	case DISPLAY_PORT_RX_1 ... DISPLAY_PORT_RX_7:
		dp_pcm_id = cpu_dai->id - DISPLAY_PORT_RX_1 + 1;
		dp_jack = &data->dp_jack[dp_pcm_id];
		break;
	default:
		break;
	}

	if (dp_jack)
		return qcom_snd_dp_jack_setup(rtd, dp_jack, dp_pcm_id);

	return qcom_snd_wcd_jack_setup(rtd, &data->jack, &data->jack_setup);
}

static int sc8280xp_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	rate->min = rate->max = 48000;
	snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S16_LE);
	channels->min = 2;
	channels->max = 2;
	switch (cpu_dai->id) {
	case TX_CODEC_DMA_TX_0:
	case TX_CODEC_DMA_TX_1:
	case TX_CODEC_DMA_TX_2:
	case TX_CODEC_DMA_TX_3:
		channels->min = 1;
		break;
	default:
		break;
	}


	return 0;
}

/*
 * For SoundWire capture backends the ADSP codec_dma interface must be told
 * which LPASS intf channel carries each logical PCM channel. The codec
 * get_channel_map() returns master_channel_map[] indexed by master port
 * with a bitmask of active channels per port; for codec_dma each active
 * master port feeds the intf channel with the same number, so the BE
 * channel map is the ascending list of ports that have active channels
 * (e.g. wcd939x ADC1->port2, ADC4->port3 => map {2,3}, matching the
 * OxygenOS ground truth "CODEC_DMA-LPAIF_RXTX-TX-3 Channel Map {02,03}").
 *
 * Must run in prepare(): the q6apmbedai cpu dai's hw_params resets the map
 * to the sequential default, and its prepare() sends the media format to
 * the ADSP — snd_soc_link_prepare (here) runs between the two.
 */
static int sc8280xp_snd_set_cpu_tx_channel_map(struct snd_soc_pcm_runtime *rtd,
					       struct snd_soc_dai *cpu_dai)
{
	unsigned int tx_ch[SDW_MAX_PORTS], rx_ch[SDW_MAX_PORTS];
	unsigned int ch_map[SDW_MAX_PORTS];
	struct snd_soc_dai *codec_dai;
	unsigned int tx_num = 0, rx_num;
	int ret, i, j, n = 0;

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		ret = snd_soc_dai_get_channel_map(codec_dai, &tx_num, tx_ch,
						  &rx_num, rx_ch);
		if (ret == -ENOTSUPP)
			continue;
		if (ret) {
			dev_err(rtd->dev, "Failed to get codec chan map %s\n",
				codec_dai->name);
			return ret;
		}
		break;
	}

	for (j = 0; j < tx_num; j++) {
		if (tx_ch[j])
			ch_map[n++] = j;
	}

	if (!n)
		return 0;

	/* TEMP DEBUG (mic bringup, +b22) */
	dev_info(rtd->dev, "MICDBG machine: cpu_dai %s compacted map n=%u: %u %u %u %u\n",
		 cpu_dai->name, n,
		 n > 0 ? ch_map[0] : 0, n > 1 ? ch_map[1] : 0,
		 n > 2 ? ch_map[2] : 0, n > 3 ? ch_map[3] : 0);

	ret = snd_soc_dai_set_channel_map(cpu_dai, n, ch_map, 0, NULL);
	if (ret)
		dev_err(rtd->dev, "Failed to set cpu channel map %s: %d\n",
			cpu_dai->name, ret);

	return ret;
}

static int sc8280xp_snd_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct sc8280xp_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	int ret = 0;

	switch (cpu_dai->id) {
	case VA_CODEC_DMA_TX_0:
	case VA_CODEC_DMA_TX_1:
	case VA_CODEC_DMA_TX_2:
	case TX_CODEC_DMA_TX_0:
	case TX_CODEC_DMA_TX_1:
	case TX_CODEC_DMA_TX_2:
	case TX_CODEC_DMA_TX_3:
	case TX_CODEC_DMA_TX_4:
	case TX_CODEC_DMA_TX_5:
		ret = sc8280xp_snd_set_cpu_tx_channel_map(rtd, cpu_dai);
		break;
	default:
		break;
	}

	if (ret)
		return ret;

	return qcom_snd_sdw_prepare(substream, &data->stream_prepared[cpu_dai->id]);
}

static int sc8280xp_snd_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct sc8280xp_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);

	return qcom_snd_sdw_hw_free(substream, &data->stream_prepared[cpu_dai->id]);
}

static const struct snd_soc_ops sc8280xp_be_ops = {
	.startup = qcom_snd_sdw_startup,
	.shutdown = qcom_snd_sdw_shutdown,
	.hw_free = sc8280xp_snd_hw_free,
	.prepare = sc8280xp_snd_prepare,
};

static void sc8280xp_add_be_ops(struct snd_soc_card *card)
{
	struct snd_soc_dai_link *link;
	int i;

	for_each_card_prelinks(card, i, link) {
		if (link->no_pcm == 1) {
			link->init = sc8280xp_snd_init;
			link->be_hw_params_fixup = sc8280xp_be_hw_params_fixup;
			link->ops = &sc8280xp_be_ops;
		}
	}
}

static int sc8280xp_platform_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card;
	struct sc8280xp_snd_data *data;
	struct device *dev = &pdev->dev;
	int ret;

	card = devm_kzalloc(dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;
	card->owner = THIS_MODULE;
	/* Allocate the private data */
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	card->dev = dev;
	dev_set_drvdata(dev, card);
	snd_soc_card_set_drvdata(card, data);
	ret = qcom_snd_parse_of(card);
	if (ret)
		return ret;

	card->driver_name = of_device_get_match_data(dev);
	sc8280xp_add_be_ops(card);
	return devm_snd_soc_register_card(dev, card);
}

static const struct of_device_id snd_sc8280xp_dt_match[] = {
	{.compatible = "qcom,eliza-sndcard", "eliza"},
	{.compatible = "qcom,kaanapali-sndcard", "kaanapali"},
	{.compatible = "qcom,qcm6490-idp-sndcard", "qcm6490"},
	{.compatible = "qcom,qcs615-sndcard", "qcs615"},
	{.compatible = "qcom,qcs6490-rb3gen2-sndcard", "qcs6490"},
	{.compatible = "qcom,qcs8275-sndcard", "qcs8300"},
	{.compatible = "qcom,qcs9075-sndcard", "sa8775p"},
	{.compatible = "qcom,qcs9100-sndcard", "sa8775p"},
	{.compatible = "qcom,sc8280xp-sndcard", "sc8280xp"},
	{.compatible = "qcom,sm8450-sndcard", "sm8450"},
	{.compatible = "qcom,sm8550-sndcard", "sm8550"},
	{.compatible = "qcom,sm8650-sndcard", "sm8650"},
	{.compatible = "qcom,sm8750-sndcard", "sm8750"},
	{}
};

MODULE_DEVICE_TABLE(of, snd_sc8280xp_dt_match);

static struct platform_driver snd_sc8280xp_driver = {
	.probe  = sc8280xp_platform_probe,
	.driver = {
		.name = "snd-sc8280xp",
		.of_match_table = snd_sc8280xp_dt_match,
	},
};
module_platform_driver(snd_sc8280xp_driver);
MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org");
MODULE_DESCRIPTION("SC8280XP ASoC Machine Driver");
MODULE_LICENSE("GPL");
