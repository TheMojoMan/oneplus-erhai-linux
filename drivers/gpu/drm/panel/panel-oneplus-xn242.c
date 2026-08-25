// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 TheMojoMan <themojoman@gmx.net>

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>
#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>

struct xn242 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi[2];
	const struct panel_desc *desc;
	struct drm_dsc_config dsc;
	struct regulator_bulk_data supplies[3];
	struct gpio_desc *reset_gpio;
	u8 display_maker;
	bool enabled;
};

static inline struct xn242 *
to_xn242(struct drm_panel *panel)
{
	return container_of(panel, struct xn242, panel);
}

static void xn242_apply_dsc_mode(struct xn242 *ctx);

struct panel_desc {
	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;

	const struct mipi_dsi_device_info dsi_info;
	int (*init_sequence)(struct xn242 *ctx);
};

static void xn242_reset(struct xn242 *ctx)
{
	/* Android sequence: HIGH(10ms) → LOW(20ms) → HIGH(120ms) */
	gpiod_set_value_cansleep(ctx->reset_gpio, 0); /* deassert → pin HIGH */
	msleep(10);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1); /* assert   → pin LOW  */
	msleep(20);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0); /* deassert → pin HIGH */
	msleep(120);
}

static int erhai_init_sequence(struct xn242 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx;
	int i;

	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
		if (!ctx->dsi[i])
			continue;

		ctx->dsi[i]->mode_flags |= MIPI_DSI_MODE_LPM;
		dsi_ctx.dsi = ctx->dsi[i];
		dsi_ctx.accum_err = 0;

		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x20);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x07, 0x46);

		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xe0);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbb, 0x00);

		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x24);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x98, 0x80);

		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x23);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x89, 0xa4);

		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xf0);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe6, 0x02);

		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x27);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd0, 0x31);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd1, 0x05);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xde, 0x40);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xdf, 0x02);

		/* Disable the TCON's auto VBP/VFP generation (page 0x25, reg 0x05=0)
		 * so the explicit porch values programmed below via 0x3B actually
		 * take effect. The mainline NT36532E (Xiaomi sheng) driver does this
		 * with the comment "Disable auto VBP VFP, must set them in 0x3b" — on
		 * an auto-timing TCON our 0x3B porch bytes are silently ignored and
		 * the panel hard-locks to its factory-derived vertical timing (the
		 * vbp=230 resonance seen in the v190 sweep). NT3653x shares this
		 * command map with NT36532E. */
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x25);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x05, 0x00);

		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
		/* 0x3B: use Android's value (0x03E8=1000). This is NOT the chunk size;
		 * it's a panel-internal timing parameter. Android sends 1000 for 120Hz/3392px.
		 * Byte layout (from nt36532e/sheng): [const, (VBP+VSYNC)_lo, VFP, ?, ?,
		 * (VBP+VSYNC)_hi[1:0], ...]. Our 0xE8=232 = VBP(230)+VSYNC(2), 0x1A=26 = VFP. */
		{
			u8 cmd3b[] = {
				0x03, 0xe8,     /* 0x03E8 = 1000 = Android's value for 120Hz mode */
				0x1a, 0x04, 0x04, 0x00, 0x80, 0x36, 0x36
			};
			mipi_dsi_dcs_write(ctx->dsi[i], 0x3b, cmd3b, sizeof(cmd3b));
		}
		/* v207: add 0xB2=0x91 (Framerate ctrl) before 0xB3=0x40 (Framerate
		 * ctrl2). The mainline NT36532E (sheng) sends BOTH for 120/60Hz; erhai
		 * only ever sent 0xB3. Testing whether the missing 0xB2 framerate
		 * register matters for this NT3653x panel. */
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb2, 0x91);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb3, 0x40);

		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x51, 0x07, 0xff);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x53, 0x2c);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x55, 0x00);

		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x23);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x60);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x07, 0x20);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x08, 0x01);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x09, 0x5a);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x11, 0x05);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x12, 0xa0);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x13, 0x01);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x15, 0xea);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x16, 0x17);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0a, 0x8e);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0b, 0x8e);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0c, 0x8e);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0d, 0x00);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x19, 0x00);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1a, 0x04);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1b, 0x08);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1c, 0x0c);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1d, 0x10);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1e, 0x14);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1f, 0x18);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x20, 0x1c);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x21, 0x20);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x22, 0x24);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x23, 0x28);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x24, 0x2c);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x25, 0x30);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x26, 0x34);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x27, 0x38);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x28, 0x3c);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2a, 0x20);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2b, 0x20);

		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x30, 0xff);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x31, 0xfd);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x32, 0xfc);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x33, 0xfa);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x34, 0xf8);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x35, 0xf6);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x36, 0xf4);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x37, 0xf2);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x38, 0xf0);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x39, 0xee);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3a, 0xec);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3b, 0xea);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3d, 0xe9);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3f, 0xe8);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x40, 0xe7);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x41, 0xe6);

		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x45, 0xff);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x46, 0xf9);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x47, 0xf6);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x48, 0xf2);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x49, 0xf0);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4a, 0xec);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4b, 0xe8);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4c, 0xe4);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4d, 0xe0);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4e, 0xde);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4f, 0xd9);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x50, 0xd6);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x51, 0xd4);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x52, 0xc2);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x53, 0xd0);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x54, 0xcd);

		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x58, 0xff);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x59, 0xf6);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5a, 0xf0);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5b, 0xeb);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5c, 0xe8);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5d, 0xe5);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5e, 0xe3);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5f, 0xe0);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x60, 0xde);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x61, 0xda);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x62, 0xd7);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x63, 0xd4);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x64, 0xd2);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x65, 0xd0);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x66, 0xcc);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x67, 0xc8);

		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6e, 0xb3);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0xd0);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x70, 0x00);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x71, 0xb3);

		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x90, 0x03);
		/* byte 10 = 0x38 (= 568 pclks per INTF, Android calibration).
		 * Pairs with first_line_bpg_offset=13 in the DSI host, which
		 * cascades RC params (initial_dec_delay=716,
		 * scale_increment_interval=568) to match the panel decoder. */
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x91, 0x89, 0xa8, 0x00, 0x14, 0xd2, 0x00, 0x00, 0x00, 0x02, 0x38, 0x00, 0x0b, 0x05, 0x7a, 0x03, 0x37);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x92, 0x10, 0xe0);

		/* exit_sleep (0x11) and display_on (0x29) used to live here, but
		 * that fires them BEFORE PPS+compression_mode are sent in enable()
		 * → xn242_apply_dsc_mode. Moved to the end of apply_dsc_mode so
		 * the order is: init DCS → PPS → compression_mode → 0x11 → 0x29. */

		if (dsi_ctx.accum_err)
			return dsi_ctx.accum_err;
	}

	msleep(120);

	return 0;
}

/*
 * Android per-link timing (120Hz): HFP=50 HBP=64 HPW=32 VFP=26 VBP=230 VPW=2
 * Combined (dual-DSI): HFP=100 HBP=128 HPW=64 → htotal=3684
 * Pixel clock: 3684 * 5316 * 60 / 1000 = 1175048 kHz
 */
static const struct drm_display_mode xn242_mode_60 = {
	.clock = 1175048,
	.hdisplay = 3392,
	.hsync_start = 3392 + 100,
	.hsync_end = 3392 + 100 + 64,
	.htotal = 3684,
	.vdisplay = 2400,
	.vsync_start = 2400 + 2684,
	.vsync_end = 2400 + 2684 + 2,
	.vtotal = 5316,
	.width_mm = 274,
	.height_mm = 193,
	.type = DRM_MODE_TYPE_DRIVER,
};

/*
 * Android per-link timing (120Hz): HFP=50 HBP=64 HPW=32 VFP=26 VBP=230 VPW=2
 * Combined (dual-DSI): HFP=100 HBP=128 HPW=64 → htotal=3684
 * Pixel clock: 3684 * 2658 * 120 / 1000 = 1175048 kHz
 */
static const struct drm_display_mode xn242_mode_120 = {
	.clock = 1175048,
	.hdisplay = 3392,
	.hsync_start = 3392 + 100,
	.hsync_end = 3392 + 100 + 64,
	.htotal = 3684,
	.vdisplay = 2400,
	.vsync_start = 2400 + 26,
	.vsync_end = 2400 + 26 + 2,
	.vtotal = 2658,
	.width_mm = 274,
	.height_mm = 193,
	.type = DRM_MODE_TYPE_DRIVER,
};

/*
 * SINGLE-LINK wall-bisect mode (2026-05-24). Drives ONE DSI link only (left
 * 1696 columns), selected by the erhai-v193 DTS (panel has one output port,
 * mdss_dsi0 not bonded, mdss_dsi1 disabled). The panel's per-link decoder sees
 * EXACTLY the stream one bonded link gets today (1696 wide, 2 slices of 848,
 * chunk 848), but with the second link and inter-link sync removed entirely.
 * Purpose: if the left half renders all 2400 lines correctly here, the ~800px
 * wall is a dual-link sync/bonding artifact; if it still walls at ~800, the
 * wall is genuinely per-link decode/pacing. Per-link h-timing = the 120Hz
 * combined porches halved across one link (hfp=50, hsa=32, hbp=64 → htotal
 * 1842); v-timing identical to 120Hz; clock = 1842*2658*120/1000 = 587524.
 */
static const struct drm_display_mode xn242_mode_single_120 = {
	.clock = 587524,
	.hdisplay = 1696,
	.hsync_start = 1696 + 50,
	.hsync_end = 1696 + 50 + 32,
	.htotal = 1696 + 50 + 32 + 64,	/* 1842 */
	.vdisplay = 2400,
	.vsync_start = 2400 + 26,
	.vsync_end = 2400 + 26 + 2,
	.vtotal = 2658,
	.width_mm = 137,
	.height_mm = 193,
	.type = DRM_MODE_TYPE_DRIVER,
};

/*
 * The panel's top rate, 143.98Hz, is NOT advertised: switching to it is not yet
 * root-caused. The panel takes the timing, but the DSI hosts then throw a
 * continuous FIFO error storm (dsi_err_worker status=4) on both links, the image
 * fades to stripes/half-black and the in-cell touch controller (synced to panel
 * timing) dies until another mode set. Everything else in the OnePlus dfps list
 * (30/48/50/60/90/120) is a pure porch reprogram at one constant clock and works.
 *
 * The timing matches the vendor's byte-for-byte, so restoring the mode is only a
 * matter of re-adding it here and in xn242_rate_table[]:
 *
 *   .clock = 1261188,		    pins the derived pclk to 120's 227099396, so the
 *				    switch stays porch-only at a constant PLL; the
 *				    naive label 1261347 pulls the PLL ~173 kHz and
 *				    loses DSC lock on its own
 *   .hdisplay = 3392, .hsync_start = 3392 + 68, .hsync_end = 3392 + 68 + 32,
 *   .htotal = 3532,		    HFP=68 HSA=32 HBP=40 combined (34/16/20 per link)
 *   .vdisplay = 2400, .vsync_start = 2400 + 26, .vsync_end = 2400 + 26 + 2,
 *   .vtotal = 2480,
 *
 * That yields per-link ha_start=36, ha_end=602, i.e. REG_DSI_ACTIVE_H 0x025a0024
 * and REG_DSI_TOTAL 0x09af027b -- identical to the vendor's 144Hz register dump.
 * Unlike the other rates it also changes the horizontal porches (htotal=3532),
 * because 2400 active lines do not fit a 144Hz frame at htotal=3684.
 */

/*
 * Additional refresh rates from the OnePlus panel dtsi dfps table
 * (qcom,dsi-supported-dfps-list / *-hfp/hbp/hpw/vbp/vfp/vpw-list). The panel runs
 * ONE constant DSI bit clock (1.3625 GHz, qcom,mdss-dsi-panel-clockrate) for every
 * rate — only the porches differ ("dfps_immediate_porch_mode_both_hv_porch"). The
 * .clock here is the natural htotal*vtotal*fps for each mode (so userspace shows the
 * right Hz); the seamless-dfps path never reprograms the PLL, so all rates share the
 * one byte clock and switch by porch reprogram alone. Per-link porches are doubled
 * into the combined dual-DSI horizontal timing (HFP=2*hfp, HPW=2*hpw, HBP=2*hbp).
 *
 * 30/48/50/60/90/120 are ONE constant-clock domain: clock=1175048, htotal=3684
 * (hsync_start +100, hsync_end +164), constant vbp=230 / vsync_width=2, and only
 * vfp (vtotal) varies -- so switching between ANY of these six is a pure porch
 * reprogram, no clock change (the panel's native dfps); they all derive the same
 * pixel clock 227099396 / byte clock 170324547.
 *
 * The panel's 144Hz rate is not advertised; see the comment above the rate table
 * for why, and for the timing needed to bring it back.
 */

/* 30Hz: htotal=3684, vfp=8000 vbp=230 -> vtotal=10632 */
static const struct drm_display_mode xn242_mode_30 = {
	.clock = 1175048,
	.hdisplay = 3392,
	.hsync_start = 3392 + 100,
	.hsync_end = 3392 + 100 + 64,
	.htotal = 3684,
	.vdisplay = 2400,
	.vsync_start = 2400 + 8000,
	.vsync_end = 2400 + 8000 + 2,
	.vtotal = 10632,
	.width_mm = 274,
	.height_mm = 193,
	.type = DRM_MODE_TYPE_DRIVER,
};

/* 48Hz: htotal=3684, vfp=4013 vbp=230 -> vtotal=6645 (same clock as 120, porch-only) */
static const struct drm_display_mode xn242_mode_48 = {
	.clock = 1175048,
	.hdisplay = 3392,
	.hsync_start = 3392 + 100,
	.hsync_end = 3392 + 100 + 64,
	.htotal = 3684,
	.vdisplay = 2400,
	.vsync_start = 2400 + 4013,
	.vsync_end = 2400 + 4013 + 2,
	.vtotal = 6645,
	.width_mm = 274,
	.height_mm = 193,
	.type = DRM_MODE_TYPE_DRIVER,
};

/* 50Hz: htotal=3684, vfp=3747 vbp=230 -> vtotal=6379 (same clock as 120, porch-only) */
static const struct drm_display_mode xn242_mode_50 = {
	.clock = 1175048,
	.hdisplay = 3392,
	.hsync_start = 3392 + 100,
	.hsync_end = 3392 + 100 + 64,
	.htotal = 3684,
	.vdisplay = 2400,
	.vsync_start = 2400 + 3747,
	.vsync_end = 2400 + 3747 + 2,
	.vtotal = 6379,
	.width_mm = 274,
	.height_mm = 193,
	.type = DRM_MODE_TYPE_DRIVER,
};

/* 90Hz: htotal=3684, vfp=912 vbp=230 -> vtotal=3544 (same clock as 120, porch-only) */
static const struct drm_display_mode xn242_mode_90 = {
	.clock = 1175048,
	.hdisplay = 3392,
	.hsync_start = 3392 + 100,
	.hsync_end = 3392 + 100 + 64,
	.htotal = 3684,
	.vdisplay = 2400,
	.vsync_start = 2400 + 912,
	.vsync_end = 2400 + 912 + 2,
	.vtotal = 3544,
	.width_mm = 274,
	.height_mm = 193,
	.type = DRM_MODE_TYPE_DRIVER,
};


static const struct panel_desc erhai_desc = {
	.dsi_info = {
		.type = "erhai",
		.channel = 0,
		.node = NULL,
	},
	.format = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS,
	.init_sequence = erhai_init_sequence,
};

static int xn242_off(struct xn242 *ctx)
{
	/* v207: issue off-commands on dsi[0] (the primary/trigger link under the
	 * mainline DSI_1 cmd-sync convention restored in v206). Previously this
	 * used dsi[1]; with mainline routing a transfer issued only on DSI_1 is
	 * short-circuited (the real dual transfer happens on DSI_0), so the
	 * display_off/enter_sleep never reached the panel and the last frame
	 * stayed lit until power-off. dsi[0] transmits + mirrors to dsi[1]. */
	struct mipi_dsi_device *dsi = ctx->dsi[0];
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	msleep(20);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	msleep(120);

	return dsi_ctx.accum_err;
}

static int xn242_prepare(struct drm_panel *panel)
{
	struct xn242 *ctx = to_xn242(panel);
	struct mipi_dsi_device *dsi = ctx->dsi[0];
	struct device *dev = &dsi->dev;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators: %d\n", ret);
		return ret;
	}

	msleep(120);

	xn242_reset(ctx);

	msleep(120);

	ret = ctx->desc->init_sequence(ctx);
	if (ret < 0) {
		regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
		dev_err(panel->dev, "failed to initialize panel: %d\n", ret);
		goto fail;
	}

	/*
	 * Do NOT send PPS or compression_mode here. prepare() runs before
	 * dsi_timing_setup() (due to prepare_prev_first=true), so ctx->dsc
	 * has pic_width=0 and no RC params yet. Sending an invalid PPS here
	 * would leave the panel DSC decoder in a broken state.
	 * Both PPS and compression_mode are sent in enable() after
	 * dsi_timing_setup() has populated ctx->dsc with valid parameters.
	 */

	msleep(120);

	return 0;

fail:
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	return ret;
}

static int xn242_enable(struct drm_panel *panel)
{
	struct xn242 *ctx = to_xn242(panel);

	/* By enable() time dsi_timing_setup() has run, so ctx->dsc is valid. */
	ctx->enabled = true;
	xn242_apply_dsc_mode(ctx);

	msleep(28);

	return 0;
}

static int xn242_disable(struct drm_panel *panel)
{
	struct xn242 *ctx = to_xn242(panel);
	struct device *dev = &ctx->dsi[0]->dev;
	int ret;

	ctx->enabled = false;
	ret = xn242_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	return ret;
}

static int xn242_unprepare(struct drm_panel *panel)
{
	struct xn242 *ctx = to_xn242(panel);

	ctx->dsi[0]->mode_flags &= ~MIPI_DSI_MODE_LPM;
	if (ctx->dsi[1])
		ctx->dsi[1]->mode_flags &= ~MIPI_DSI_MODE_LPM;

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);

	return 0;
}

/*
 * The OnePlus dfps rate set, minus its 144Hz entry (see above). These six share
 * one constant DSI bit clock and switch seamlessly between each other by porch
 * reprogram alone (see the seamless-dfps path in msm_atomic.c / dpu_encoder.c /
 * dsi_host.c). 120 is the boot (PREFERRED) mode.
 */
static const struct {
	const struct drm_display_mode *mode;
	int hz;
} xn242_rate_table[] = {
	{ &xn242_mode_30,  30  },
	{ &xn242_mode_48,  48  },
	{ &xn242_mode_50,  50  },
	{ &xn242_mode_60,  60  },
	{ &xn242_mode_90,  90  },
	{ &xn242_mode_120, 120 },
};

static int xn242_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	unsigned int count = 0;
	struct drm_display_mode *mode;
	struct xn242 *ctx = to_xn242(panel);
	int i;

	/* When the DT wires only one DSI link (dsi[1]==NULL), advertise only the
	 * 1696-wide single-link mode so userspace selects it. */
	if (!ctx->dsi[1]) {
		mode = drm_mode_duplicate(connector->dev, &xn242_mode_single_120);
		if (mode) {
			mode->type |= DRM_MODE_TYPE_PREFERRED;
			drm_mode_set_name(mode);
			drm_mode_probed_add(connector, mode);
			count++;
		}
		return count;
	}

	for (i = 0; i < ARRAY_SIZE(xn242_rate_table); i++) {
		mode = drm_mode_duplicate(connector->dev, xn242_rate_table[i].mode);
		if (!mode)
			continue;

		if (xn242_rate_table[i].hz == 120)
			mode->type |= DRM_MODE_TYPE_PREFERRED;
		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
		count++;
	}

	return count;
}

/* Send the panel-side DSC bring-up commands (0x3B porch, PPS,
 * compression_mode, exit_sleep, display_on). Called from enable() once
 * dsi_timing_setup() has populated ctx->dsc with valid RC parameters. */
static void xn242_apply_dsc_mode(struct xn242 *ctx)
{
	struct drm_dsc_config dsc_tmp;
	int i;

	if (!ctx->enabled)
		return;

	/* Per-link DSC config for PPS generation. ctx->dsc already carries the
	 * RC parameters from dsi_populate_dsc_params; PPS is generated from a
	 * copy so the per-link fixups below don't leak back into the host's
	 * config. */
	dsc_tmp = ctx->dsc;

	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
		struct drm_dsc_config pps_cfg;
		struct drm_dsc_picture_parameter_set pps;

		if (!ctx->dsi[i])
			continue;

		ctx->dsi[i]->mode_flags |= MIPI_DSI_MODE_LPM;

		/* 0x3B: Android's value 0x03E8=1000. Not the chunk size — panel-internal timing. */
		{
			u8 cmd3b[] = {
				0x03, 0xe8,     /* 1000 = Android's value for 120Hz mode */
				0x1a, 0x04, 0x04, 0x00, 0x80, 0x36, 0x36
			};
			/* Disable auto VBP/VFP (page 0x25 reg 0x05=0) so the 0x3B porch
			 * values below take effect — see nt36532e/sheng. Without this the
			 * panel ignores 0x3B and uses its own factory vertical timing. */
			mipi_dsi_dcs_write(ctx->dsi[i], 0xff, (u8[]){0x25}, 1);
			mipi_dsi_dcs_write(ctx->dsi[i], 0xfb, (u8[]){0x01}, 1);
			mipi_dsi_dcs_write(ctx->dsi[i], 0x05, (u8[]){0x00}, 1);
			mipi_dsi_dcs_write(ctx->dsi[i], 0xff, (u8[]){0x10}, 1);
			mipi_dsi_dcs_write(ctx->dsi[i], 0xfb, (u8[]){0x01}, 1);
			mipi_dsi_dcs_write(ctx->dsi[i], 0x3b, cmd3b, sizeof(cmd3b));
		}

		/* PPS: per-DSI-link parameters. pic_width=1696 (SPLIT_PANEL halves
		 * the 3392 frame across the two links), 2 slices of 848 per link,
		 * chunk_size the computed 848 (agrees with the panel's 0x91). */
		pps_cfg = dsc_tmp;
		pps_cfg.pic_width = 1696;
		pps_cfg.slice_count = 2;
		drm_dsc_pps_payload_pack(&pps, &pps_cfg);
		mipi_dsi_picture_parameter_set(ctx->dsi[i], &pps);
		msleep(10);

		mipi_dsi_compression_mode(ctx->dsi[i], true);
		msleep(10);

		/* Spec order: exit_sleep AFTER PPS+compression so the decoder is
		 * configured before pixels stream; then display_on. */
		mipi_dsi_dcs_exit_sleep_mode(ctx->dsi[i]);
		msleep(120);

		mipi_dsi_dcs_set_display_on(ctx->dsi[i]);
	}
}

static const struct drm_panel_funcs xn242_panel_funcs = {
	.prepare = xn242_prepare,
	.enable = xn242_enable,
	.disable = xn242_disable,
	.unprepare = xn242_unprepare,
	.get_modes = xn242_get_modes,
};

/* OTP QP limits for DSC RC ranges 10-14 (see xn242_probe). */
static const struct {
	u8 range_min_qp;
	u8 range_max_qp;
} xn242_otp_rc_range[] = {
	{ 5, 10 }, { 5, 11 }, { 5, 11 }, { 9, 12 }, { 12, 13 },
};

static int xn242_probe(struct mipi_dsi_device *dsi)
{
	struct mipi_dsi_host *dsi_sec_host;
	struct xn242 *ctx;
	struct device *dev = &dsi->dev;
	struct device_node *dsi_sec;
	int ret, i;

	ctx = devm_drm_panel_alloc(dev, struct xn242, panel, &xn242_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->supplies[0].supply = "vddio";
	ctx->supplies[1].supply = "dvddbuck";
	ctx->supplies[2].supply = "dvddldo";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->supplies),
				      ctx->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->desc = of_device_get_match_data(dev);
	if (!ctx->desc)
		return -ENODEV;

	dsi_sec = of_graph_get_remote_node(dsi->dev.of_node, 1, -1);

	if (dsi_sec) {
		const struct mipi_dsi_device_info info = { "xn242", 0,
							   dsi_sec };

		dev_notice(dev, "Using Dual-DSI with OF node `%s`\n", dsi_sec->name);

		dsi_sec_host = of_find_mipi_dsi_host_by_node(dsi_sec);
		of_node_put(dsi_sec);
		if (!dsi_sec_host) {
			return dev_err_probe(dev, -EPROBE_DEFER,
					     "Cannot get secondary DSI host\n");
		}

		ctx->dsi[1] =
			devm_mipi_dsi_device_register_full(dev, dsi_sec_host, &info);
		if (IS_ERR(ctx->dsi[1])) {
			return dev_err_probe(dev, PTR_ERR(ctx->dsi[1]),
					     "Cannot get secondary DSI node\n");
		}

		mipi_dsi_set_drvdata(ctx->dsi[1], ctx);
	} else {
		dev_notice(dev, "Using Single-DSI\n");
	}

	ctx->dsi[0] = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->panel.prepare_prev_first = true;

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	drm_panel_add(&ctx->panel);

	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 2;		/* DSC 1.2 */

	/*
	 * Panel-wide DSC geometry. This describes what each DSI-link decoder
	 * sees AFTER the dual-DSI/multiplex split:
	 *   - 3392-wide full frame is split into 2 DSI links (SPLIT_PANEL, 1696/link)
	 *   - Each link's pair of DSC engines multiplex 2 slices of 848 (MULTIPLEX)
	 *   - drm_dsc_compute_rc_parameters with slice_width=848 → chunk_size=848,
	 *     matching panel OTP RC parameters (calibrated for 848-px slices).
	 * The PPS we send the panel uses per-link values (pic_width=1696,
	 * slice_count=2) — see xn242_apply_dsc_mode.
	 */
	/* Single-link bisect (dsi[1]==NULL, from erhai-v193 DTS): one DSI link
	 * decodes a 1696-wide frame (2 slices of 848). pic_width=1696 so the DSC
	 * engine encodes the full single-link line (no SPLIT_PANEL halving). */
	ctx->dsc.pic_width = ctx->dsi[1] ? 3392 : 1696;
	ctx->dsc.pic_height = 2400;

	ctx->dsc.slice_height = 20;		/* per the panel's vendor dtsi */
	ctx->dsc.slice_count = 2;
	ctx->dsc.slice_width = 848; /* 1696 / 2 */

	ctx->dsc.bits_per_component = 8;
	ctx->dsc.bits_per_pixel = 128; /* 8 bpp */
	ctx->dsc.block_pred_enable = true;
	ctx->dsc.convert_rgb = true;
	ctx->dsc.line_buf_depth = 9;

	/* Pre-set chunk_size=848 for prepare()-time 0x3B command. */
	ctx->dsc.slice_chunk_size = 848; /* = slice_width for 8bpp */

	/*
	 * OTP-calibrated RC values, captured from Android's DCE register dump.
	 * The generic drm_dsc PRE_SCR table differs and the panel decodes
	 * against its own calibration, so hand these to the DSI host, which
	 * re-applies them after drm_dsc_setup_rc_params():
	 *   first_line_bpg_offset 13 (table says 12) — cascades into
	 *     nfl_bpg_offset, slice_bpg_offset, initial_dec_delay and
	 *     scale_increment_interval.
	 *   ranges 10-14 QP limits (DSC_RC_MIN_QP_2/MAX_QP_2); the table caps
	 *     max_qp higher (11/12/13/13/15) than the panel decodes.
	 */
	ctx->dsc.first_line_bpg_offset = 13;
	for (i = 0; i < ARRAY_SIZE(xn242_otp_rc_range); i++) {
		ctx->dsc.rc_range_params[10 + i].range_min_qp =
			xn242_otp_rc_range[i].range_min_qp;
		ctx->dsc.rc_range_params[10 + i].range_max_qp =
			xn242_otp_rc_range[i].range_max_qp;
	}

	/* The rest of the RC parameters are computed by dsi_populate_dsc_params
	 * at mode-set time. No need to call drm_dsc_compute_rc_parameters here. */

	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
		if (!ctx->dsi[i])
			continue;

		ctx->dsi[i]->dsc = &ctx->dsc;

		ctx->dsi[i]->lanes = 4;
		ctx->dsi[i]->format = MIPI_DSI_FMT_RGB888;
		ctx->dsi[i]->mode_flags = MIPI_DSI_MODE_VIDEO |
					  MIPI_DSI_MODE_DSC_ALL_SLICES_IN_PKT;

		ret = devm_mipi_dsi_attach(dev, ctx->dsi[i]);
		if (ret < 0) {
			drm_panel_remove(&ctx->panel);
			return dev_err_probe(dev, ret,
					     "Failed to attach to DSI%d\n", i);
		}
	}

	return 0;
}

static void xn242_remove(struct mipi_dsi_device *dsi)
{
	struct xn242 *ctx = mipi_dsi_get_drvdata(dsi);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id xn242_of_match[] = {
	{
		.compatible = "oneplus,erhai-xn242",
		.data = &erhai_desc,
	},
	{},
};
MODULE_DEVICE_TABLE(of, xn242_of_match);

static struct mipi_dsi_driver xn242_driver = {
	.probe = xn242_probe,
	.remove = xn242_remove,
	.driver = {
		.name = "panel-oneplus-xn242",
		.of_match_table = xn242_of_match,
	},
};
module_mipi_dsi_driver(xn242_driver);

MODULE_AUTHOR("TheMojoMan <themojoman@gmx.net>");
MODULE_DESCRIPTION("DRM panel driver for the XN242 panel");
MODULE_LICENSE("GPL");
