// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2018-2023, Linaro Limited.
// Copyright (c) 2018, The Linux Foundation. All rights reserved.

#include <dt-bindings/sound/qcom,lpass.h>
#include <dt-bindings/sound/qcom,q6afe.h>
#include <linux/module.h>
#include <sound/soc.h>
#include "sdw.h"

/*
 * erhai mic (2026-09-18, +b61): the TX_CODEC_DMA_TX_3 intf channel map is
 * owned by the machine driver -- sc8280xp_snd_set_cpu_tx_channel_map()
 * compacts the codec's get_channel_map() master_channel_map[] into the
 * ascending list of master ports that carry active channels.  For our
 * topology the codec reports map[1..4] = 0 54 59 0 (ADC1 -> master port 2,
 * ADC4 -> master port 3), so the machine driver programs {2,3} -- exactly
 * Android's recording-time ground truth
 * "CODEC_DMA-LPAIF_RXTX-TX-3 Channel Map = {0x02,0x03}".
 *
 * The b48/b49/b59 blind override that used to live here ({2,3} then a
 * speculative flip to {3,4}) is GONE on purpose: it ran 3 us AFTER the
 * machine driver and silently clobbered the correct derived value with a
 * guessed one.  Do NOT reintroduce a hardcoded lane list here -- if the map
 * is wrong, fix the codec's master_channel_map[] / get_channel_map().
 */

static bool qcom_snd_is_sdw_dai(int id)
{
	switch (id) {
	case WSA_CODEC_DMA_RX_0:
	case WSA_CODEC_DMA_TX_0:
	case WSA_CODEC_DMA_RX_1:
	case WSA_CODEC_DMA_TX_1:
	case WSA_CODEC_DMA_TX_2:
	case RX_CODEC_DMA_RX_0:
	case TX_CODEC_DMA_TX_0:
	case RX_CODEC_DMA_RX_1:
	case TX_CODEC_DMA_TX_1:
	case RX_CODEC_DMA_RX_2:
	case TX_CODEC_DMA_TX_2:
	case RX_CODEC_DMA_RX_3:
	case TX_CODEC_DMA_TX_3:
	case RX_CODEC_DMA_RX_4:
	case TX_CODEC_DMA_TX_4:
	case RX_CODEC_DMA_RX_5:
	case TX_CODEC_DMA_TX_5:
	case RX_CODEC_DMA_RX_6:
	case RX_CODEC_DMA_RX_7:
	case SLIMBUS_0_RX...SLIMBUS_6_TX:
		return true;
	default:
		break;
	}

	/* DSP Bypass usecase, cpu dai index overlaps with DSP dai ids,
	 * DO NOT MERGE into top switch case */
	switch (id) {
	case LPASS_CDC_DMA_TX3:
	case LPASS_CDC_DMA_RX0:
		return true;
	default:
		break;
	}

	return false;
}

/**
 * qcom_snd_sdw_startup() - Helper to start Soundwire stream for SoC audio card
 * @substream: The PCM substream from audio, as passed to snd_soc_ops->startup()
 *
 * Helper for the SoC audio card (snd_soc_ops->startup()) to allocate and set
 * Soundwire stream runtime to each codec DAI.
 *
 * The shutdown() callback should call sdw_release_stream() on the same
 * sdw_stream_runtime.
 *
 * Return: 0 or errno
 */
int qcom_snd_sdw_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	u32 rx_ch[SDW_MAX_PORTS], tx_ch[SDW_MAX_PORTS];
	struct sdw_stream_runtime *sruntime;
	struct snd_soc_dai *codec_dai;
	u32 rx_ch_cnt = 0, tx_ch_cnt = 0;
	int ret, i, j;

	if (!qcom_snd_is_sdw_dai(cpu_dai->id))
		return 0;

	sruntime = sdw_alloc_stream(cpu_dai->name, SDW_STREAM_PCM);
	if (!sruntime)
		return -ENOMEM;

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		ret = snd_soc_dai_set_stream(codec_dai, sruntime,
					     substream->stream);
		if (ret < 0 && ret != -ENOTSUPP) {
			dev_err(rtd->dev, "Failed to set sdw stream on %s\n", codec_dai->name);
			goto err_set_stream;
		} else if (ret == -ENOTSUPP) {
			/* Ignore unsupported */
			continue;
		}

		ret = snd_soc_dai_get_channel_map(codec_dai, &tx_ch_cnt, tx_ch,
						  &rx_ch_cnt, rx_ch);
		if (ret != 0 && ret != -ENOTSUPP) {
			dev_err(rtd->dev, "Failed to get codec chan map %s\n", codec_dai->name);
			goto err_set_stream;
		} else if (ret == -ENOTSUPP) {
			/* Ignore unsupported */
			continue;
		}
	}

	switch (cpu_dai->id) {
	case RX_CODEC_DMA_RX_0:
	case TX_CODEC_DMA_TX_3:
		if (tx_ch_cnt || rx_ch_cnt) {
			for_each_rtd_codec_dais(rtd, j, codec_dai) {
				ret = snd_soc_dai_set_channel_map(codec_dai,
								  tx_ch_cnt, tx_ch,
								  rx_ch_cnt, rx_ch);
				if (ret != 0 && ret != -ENOTSUPP)
					goto err_set_stream;
			}
		}
	}

	/*
	 * NOTE: the TX_CODEC_DMA_TX_3 intf channel map is deliberately NOT
	 * touched here any more.  sc8280xp_snd_set_cpu_tx_channel_map() (the
	 * machine driver's .prepare) programs it from the codec's reported
	 * port map and produces Android's {2,3}; the old b48/b49/b59 override
	 * that lived at this spot ran AFTER it and clobbered the correct
	 * value with a hardcoded guess.
	 */

	return 0;

err_set_stream:
	sdw_release_stream(sruntime);

	return ret;
}
EXPORT_SYMBOL_GPL(qcom_snd_sdw_startup);

int qcom_snd_sdw_prepare(struct snd_pcm_substream *substream,
			 bool *stream_prepared)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct sdw_stream_runtime *sruntime;
	int ret;


	if (!qcom_snd_is_sdw_dai(cpu_dai->id))
		return 0;

	sruntime = qcom_snd_sdw_get_stream(substream);
	if (!sruntime)
		return 0;

	if (*stream_prepared)
		return 0;

	/*
	 * erhai mic (b61): the TX_CODEC_DMA_TX_3 channel-map override that
	 * lived here (b49's re-assert, later flipped to {3,4} by b59) has been
	 * REMOVED.  It was made obsolete by sc8280xp_snd_set_cpu_tx_channel_map()
	 * -- the machine driver's .prepare runs before this function and derives
	 * the map from the codec's real port map, yielding Android's {2,3}.
	 * The hardcoded override contradicted it and won (it ran later), which
	 * is why the ADSP was fed the wrong intf channels.  Leave this out.
	 */

	ret = sdw_prepare_stream(sruntime);
	if (ret)
		return ret;

	/**
	 * NOTE: there is a strict hw requirement about the ordering of port
	 * enables and actual WSA881x PA enable. PA enable should only happen
	 * after soundwire ports are enabled if not DC on the line is
	 * accumulated resulting in Click/Pop Noise
	 * PA enable/mute are handled as part of codec DAPM and digital mute.
	 */

	ret = sdw_enable_stream(sruntime);
	if (ret) {
		sdw_deprepare_stream(sruntime);
		return ret;
	}
	*stream_prepared  = true;

	return ret;
}
EXPORT_SYMBOL_GPL(qcom_snd_sdw_prepare);

struct sdw_stream_runtime *qcom_snd_sdw_get_stream(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *codec_dai;
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct sdw_stream_runtime *sruntime;
	int i;

	if (!qcom_snd_is_sdw_dai(cpu_dai->id))
		return NULL;

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		sruntime = snd_soc_dai_get_stream(codec_dai, substream->stream);
		if (sruntime != ERR_PTR(-ENOTSUPP))
			return sruntime;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(qcom_snd_sdw_get_stream);

void qcom_snd_sdw_shutdown(struct snd_pcm_substream *substream)
{
	struct sdw_stream_runtime *sruntime = qcom_snd_sdw_get_stream(substream);

	sdw_release_stream(sruntime);
}
EXPORT_SYMBOL_GPL(qcom_snd_sdw_shutdown);

int qcom_snd_sdw_hw_free(struct snd_pcm_substream *substream, bool *stream_prepared)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct sdw_stream_runtime *sruntime;

	if (!qcom_snd_is_sdw_dai(cpu_dai->id))
		return 0;

	sruntime = qcom_snd_sdw_get_stream(substream);
	if (sruntime && *stream_prepared) {
		sdw_disable_stream(sruntime);
		sdw_deprepare_stream(sruntime);
		*stream_prepared = false;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(qcom_snd_sdw_hw_free);
MODULE_DESCRIPTION("Qualcomm ASoC SoundWire helper functions");
MODULE_LICENSE("GPL");
