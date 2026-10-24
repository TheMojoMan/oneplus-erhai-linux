// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 TheMojoMan <themojoman@gmx.net>

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/kstrtox.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/sysfs.h>

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
	/* erhai panel control surface (diagnostic) — see xn_* below */
	unsigned int init_ok;
	unsigned int init_err;
	int last_err;
	u8 last_read[16];
	u8 last_read_len;
};

static inline struct xn242 *
to_xn242(struct drm_panel *panel)
{
	return container_of(panel, struct xn242, panel);
}

/*
 * erhai: the NT36536 is an in-cell controller powered by these panel supplies;
 * it must be put to sleep BEFORE the rails drop on display-off and re-initialized
 * once they are back (see drivers/input/touchscreen/nt36523/nt36xxx.c).
 */
#if IS_ENABLED(CONFIG_TOUCHSCREEN_NT36523_SPI)
extern void nvt_ts_panel_power_off(void);
extern void nvt_ts_panel_power_on(void);
#endif

static void xn242_apply_dsc_mode(struct xn242 *ctx);

/* ==========================================================================
 * erhai PANEL CONTROL SURFACE (diagnostic, 2026-10-23)
 *
 * Every number, byte blob and sleep of the XN242 bring-up is a tunable that is
 * visible and writable at run time through
 *
 *	/sys/module/panel_oneplus_xn242/parameters/<name>
 *
 * (and settable at boot from the kernel command line as
 * `panel_oneplus_xn242.<name>=<value>`).  Every default reproduces the driver
 * as it behaved before this surface existed, so a default boot is unchanged.
 *
 * Verbs, in the same directory: `reinit` (write soft|local|status),
 * `dcs` (raw DCS: `w <reg> [bytes]`, `w0/w1 <reg> [bytes]`, `r <reg>`),
 * `dump` (read: live state + every value) and `log` (read: what was done).
 *
 * Writes are logged but only take effect at the NEXT bring-up: `reinit`,
 * a mode set by the compositor, or a session restart.  `skip` (or `-`/`-1`)
 * means "do not send that write at all".
 * ======================================================================== */

#define XN_VAL_MAX	48
#define XN_LOG_LEN	4096
#define XN_SKIP		0xffffffffU

/* Sequence blocks, selected by the `seq_mask` bitfield. */
#define XN_SEQ_HEAD	0x01	/* pages 20/e0/24/23/f0/27 one-shots */
#define XN_SEQ_P25	0x02	/* page 0x25 reg 0x05 (auto VBP/VFP) */
#define XN_SEQ_3B	0x04	/* page 0x10: 0x3b blob, 0xb2, 0xb3 */
#define XN_SEQ_5153	0x08	/* page 0x10: 0x51, 0x53, 0x55 */
#define XN_SEQ_P23	0x10	/* page 0x23 (the long gamma/OD block) */
#define XN_SEQ_9192	0x20	/* page 0x10: 0x90, 0x91, 0x92 */
#define XN_SEQ_ON	0x40	/* exit_sleep(0x11) + display_on(0x29) */

#define XN_TUNABLES(X) \
	X(reset_high_ms,	"10", 0) \
	X(reset_low_ms,		"20", 0) \
	X(reset_wait_ms,	"120", 0) \
	X(reset_post_ms,	"120", 0) \
	X(rail_settle_ms,	"120", 0) \
	X(init_settle_ms,	"120", 0) \
	X(pre_enable_ms,	"28", 0) \
	X(post_enable_ms,	"0", 0) \
	X(display_off_ms,	"20", 0) \
	X(sleep_in_ms,		"120", 0) \
	X(exit_sleep_ms,	"120", 0) \
	X(pps_gap_ms,		"10", 0) \
	X(comp_gap_ms,		"10", 0) \
	X(seq_mask,		"127", 0) \
	X(page25_reg05,		"0", 0) \
	X(porch_3b,		"03 e8 1a 04 04 00 80 36 36", 1) \
	X(reg_3b_23,		"234", 0) \
	X(reg_b2,		"145", 0) \
	X(reg_b3,		"64", 0) \
	X(reg_51,		"2047", 0) \
	X(reg_53,		"44", 0) \
	X(reg_53b,		"208", 0) \
	X(reg_55,		"0", 0) \
	X(reg_90,		"3", 0) \
	X(reg_91,		"89 a8 00 14 d2 00 00 00 02 38 00 0b 05 7a 03 37", 1) \
	X(reg_92,		"10 e0", 1) \
	X(lanes,		"4", 0) \
	X(dsc_enable,		"1", 0) \
	X(dsc_bpp,		"128", 0) \
	X(dsc_slice_count,	"2", 0) \
	X(dsc_slice_width,	"848", 0) \
	X(dsc_slice_height,	"20", 0) \
	X(dsc_pic_width_link,	"1696", 0) \
	X(dsc_first_line_bpg,	"13", 0) \
	X(dsc_line_buf_depth,	"9", 0) \
	X(dsc_convert_rgb,	"1", 0) \
	X(dsc_block_pred,	"1", 0) \
	X(dsc_otp_rc,		"1", 0) \
	X(only_hz,		"0", 0)

enum xn_idx {
#define XN_ID(n, d, s) XN_##n,
	XN_TUNABLES(XN_ID)
#undef XN_ID
	XN_IDX_MAX
};

struct xn_tun {
	const char *name;
	const char *def;
	char val[XN_VAL_MAX];
	u32 num;
	u8 is_str;
};

static struct xn_tun xn_tuns[] = {
#define XN_ROW(n, d, s) { .name = #n, .def = d, .val = d, .is_str = (s) },
	XN_TUNABLES(XN_ROW)
#undef XN_ROW
};

static DEFINE_MUTEX(xn_lock);
static struct xn242 *xn_ctx;
static unsigned int xn_cycles;
static char xn_msg[192] = "(no action yet)";
static char xn_log[XN_LOG_LEN];
static size_t xn_log_used;

static void xn_log_add(const char *fmt, ...) __printf(1, 2);
static void xn_log_add(const char *fmt, ...)
{
	va_list ap;
	char line[160];
	int n;

	va_start(ap, fmt);
	n = vscnprintf(line, sizeof(line), fmt, ap);
	va_end(ap);
	if (n <= 0)
		return;

	if (xn_log_used + n + 2 > sizeof(xn_log)) {
		size_t half = xn_log_used / 2;
		char *nl = memchr(xn_log + half, '\n', xn_log_used - half);
		size_t cut = nl ? (size_t)(nl + 1 - xn_log) : xn_log_used;

		memmove(xn_log, xn_log + cut, xn_log_used - cut);
		xn_log_used -= cut;
	}

	memcpy(xn_log + xn_log_used, line, n);
	xn_log_used += n;
	xn_log[xn_log_used++] = '\n';
	xn_log[xn_log_used] = 0;
}

static u32 xn_val(enum xn_idx i)
{
	return xn_tuns[i].num;
}

static const char *xn_txt(enum xn_idx i)
{
	return xn_tuns[i].val;
}

static bool xn_skip(enum xn_idx i)
{
	return xn_tuns[i].num == XN_SKIP;
}

static bool xn_yes(enum xn_idx i)
{
	return xn_tuns[i].num != 0 && xn_tuns[i].num != XN_SKIP;
}

static bool xn_seq(u32 bit)
{
	return (xn_val(XN_seq_mask) & bit) != 0;
}

static int xn_hexval(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}

/*
 * Parse "aa bb cc" / "aabbcc" / "0xaa,0xbb" / "aa:bb".  Returns the number of
 * bytes, -1 for a skip word/an empty string, or a negative errno.
 */
static int xn_parse_bytes(const char *s, u8 *out, int max)
{
	int n = 0;

	while (*s) {
		int hi, lo;

		while (*s == ' ' || *s == '\t' || *s == ',' || *s == ':')
			s++;
		if (!*s)
			break;

		if (*s == '-') {
			const char *p = s + 1;

			while (*p == ' ' || *p == '\t')
				p++;
			if (!*p || *p == '1')
				return -1;
			return -EINVAL;
		}

		hi = xn_hexval(*s++);
		if (hi < 0)
			return -EINVAL;
		lo = xn_hexval(*s);
		if (lo < 0)
			lo = hi;	/* one hex digit is enough */
		else
			s++;

		if (n >= max)
			return -E2BIG;
		out[n++] = (hi << 4) | lo;
	}

	return n ? n : -1;
}

/* Byte list of a blob tunable; 0 means "skipped". */
static int xn_blob(enum xn_idx i, u8 *out, int max)
{
	int n = xn_parse_bytes(xn_txt(i), out, max);

	return n < 0 ? 0 : n;
}

static bool xn_is_skip_word(const char *s)
{
	return !strcmp(s, "skip") || !strcmp(s, "-") || !strcmp(s, "-1");
}

static int xn_tun_store(struct xn_tun *t, const char *val)
{
	if (t->is_str) {
		u8 bytes[24];
		int n = xn_parse_bytes(val, bytes, sizeof(bytes));

		if (n != -1 && n < 0)
			return n;
		if (strlen(val) >= XN_VAL_MAX)
			return -EINVAL;
		strscpy(t->val, val, XN_VAL_MAX);
		t->num = (n < 0) ? XN_SKIP : (u32)n;
		xn_log_add("set %s = \"%s\"%s", t->name, t->val,
			   n < 0 ? "  [SKIPPED]" : "");
		return 0;
	}

	if (xn_is_skip_word(val)) {
		strscpy(t->val, "skip", XN_VAL_MAX);
		t->num = XN_SKIP;
		xn_log_add("set %s = skip", t->name);
		return 0;
	}

	{
		unsigned int v;
		int ret = kstrtouint(val, 0, &v);

		if (ret)
			return ret;
		if (v > 0xffff)
			return -EINVAL;
		t->num = v;
		snprintf(t->val, XN_VAL_MAX, "%u", v);
		xn_log_add("set %s = %u", t->name, v);
	}
	return 0;
}

static void xn_tun_reparse(struct xn_tun *t)
{
	if (t->is_str) {
		u8 bytes[24];
		int n = xn_parse_bytes(t->val, bytes, sizeof(bytes));

		t->num = (n < 0) ? XN_SKIP : (u32)n;
	} else {
		unsigned int v;

		if (!kstrtouint(t->val, 0, &v))
			t->num = v;
	}
}

/*
 * Parse the table once the command line has been consumed: a boot argument
 * such as panel_oneplus_xn242.page25_reg05=1 lands before any driver probe.
 */
static int __init xn_params_init(void)
{
	int i;

	for (i = 0; i < (int)XN_IDX_MAX; i++)
		xn_tun_reparse(&xn_tuns[i]);

	return 0;
}
core_initcall(xn_params_init);

static int xn_tun_param_set(const char *val, const struct kernel_param *kp)
{
	return xn_tun_store(kp->arg, val);
}

static int xn_tun_param_get(char *buf, const struct kernel_param *kp)
{
	struct xn_tun *t = kp->arg;

	return sysfs_emit(buf, "%s\n", t->val);
}

static const struct kernel_param_ops xn_tun_ops = {
	.set = xn_tun_param_set,
	.get = xn_tun_param_get,
};

#define XN_REG(n, d, s) module_param_cb(n, &xn_tun_ops, &xn_tuns[XN_##n], 0644);
XN_TUNABLES(XN_REG)
#undef XN_REG

struct panel_desc {
	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;

	const struct mipi_dsi_device_info dsi_info;
	int (*init_sequence)(struct xn242 *ctx);
};

static void xn242_reset(struct xn242 *ctx)
{
	/*
	 * Android's vendor DT carries <1 10>, <0 5>, <1 10> — i.e. 10 ms HIGH,
	 * 5 ms LOW, 10 ms HIGH — while this port has always used 10/20/120.
	 * All three are tunables now (reset_high_ms / reset_low_ms /
	 * reset_wait_ms); the defaults are the values the port shipped with.
	 */
	gpiod_set_value_cansleep(ctx->reset_gpio, 0); /* deassert → pin HIGH */
	msleep(xn_val(XN_reset_high_ms));
	gpiod_set_value_cansleep(ctx->reset_gpio, 1); /* assert   → pin LOW  */
	msleep(xn_val(XN_reset_low_ms));
	gpiod_set_value_cansleep(ctx->reset_gpio, 0); /* deassert → pin HIGH */
	msleep(xn_val(XN_reset_wait_ms));
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

		/* pages 20/e0/24/23/f0/27 one-shots  (seq_mask & 0x01) */
		if (!xn_seq(XN_SEQ_HEAD))
			goto head_done;
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
head_done:

		/* Page 0x25 reg 0x05: 0 = disable the TCON's own VBP/VFP generation
		 * so the explicit porch values programmed below via 0x3B actually
		 * take effect (the mainline NT36532E "sheng" driver does this), 1
		 * leaves the panel on its factory-derived vertical timing.  The
		 * erhai vendor sequence does NOT write page 0x25 at all — this is
		 * an addition of this port, hence tunable (page25_reg05; `skip` =
		 * do not send).  (seq_mask & 0x02) */
		if (xn_seq(XN_SEQ_P25) && !xn_skip(XN_page25_reg05)) {
			mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x25);
			mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
			mipi_dsi_dcs_write_var_seq_multi(&dsi_ctx, 0x05,
					(u8)(xn_val(XN_page25_reg05) ? 1 : 0));
		}

		/* Page 0x10: the 0x3B porch blob + the framerate registers
		 * (seq_mask & 0x04) */
		if (xn_seq(XN_SEQ_3B)) {
			mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
			mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
			/* 0x3B: panel-internal timing.  Byte layout (from
			 * nt36532e/sheng): [const, (VBP+VSYNC)_lo, VFP, ?, ?,
			 * (VBP+VSYNC)_hi[1:0], ...]; 0xE8=232 = VBP(230)+VSYNC(2),
			 * 0x1A=26 = VFP.  Android sends exactly this at 120 Hz.
			 * porch_3b; `-` skips the write. */
			{
				u8 b[16];
				int n = xn_blob(XN_porch_3b, b, sizeof(b));

				if (n)
					mipi_dsi_dcs_write(ctx->dsi[i], 0x3b, b, n);
			}
			/* 0xB2 = framerate control.  The erhai vendor sequence does
			 * NOT send it (v207 borrowed it from the NT36532E
			 * reference) — reg_b2=skip removes it again. */
			if (!xn_skip(XN_reg_b2))
				mipi_dsi_dcs_write_var_seq_multi(&dsi_ctx, 0xb2,
						(u8)xn_val(XN_reg_b2));
			if (!xn_skip(XN_reg_b3))
				mipi_dsi_dcs_write_var_seq_multi(&dsi_ctx, 0xb3,
						(u8)xn_val(XN_reg_b3));
		}

		/* Page 0x10: brightness/control bytes  (seq_mask & 0x08) */
		if (xn_seq(XN_SEQ_5153)) {
			mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
			mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
			if (!xn_skip(XN_reg_51)) {
				u32 v = xn_val(XN_reg_51);

				mipi_dsi_dcs_write_var_seq_multi(&dsi_ctx, 0x51,
						(u8)(v >> 8), (u8)v);
			}
			if (!xn_skip(XN_reg_53))
				mipi_dsi_dcs_write_var_seq_multi(&dsi_ctx, 0x53,
						(u8)xn_val(XN_reg_53));
			if (!xn_skip(XN_reg_55))
				mipi_dsi_dcs_write_var_seq_multi(&dsi_ctx, 0x55,
						(u8)xn_val(XN_reg_55));
		}

		/* page 0x23: the long gamma/OD block  (seq_mask & 0x10) */
		if (!xn_seq(XN_SEQ_P23))
			goto p23_done;
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
		mipi_dsi_dcs_write_var_seq_multi(&dsi_ctx, 0x3b,
				(u8)xn_val(XN_reg_3b_23));
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
		mipi_dsi_dcs_write_var_seq_multi(&dsi_ctx, 0x53,
				(u8)xn_val(XN_reg_53b));
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
p23_done:

		/* Page 0x10: the 0x90/0x91/0x92 timing blob (seq_mask & 0x20).
		 * 0x91 is Android's per-INTF pclk calibration; byte 10 = 0x38
		 * (= 568 pclks) pairs with first_line_bpg_offset=13 in the DSI
		 * host, which cascades the RC params (initial_dec_delay=716,
		 * scale_increment_interval=568) to the panel decoder. */
		if (xn_seq(XN_SEQ_9192)) {
			mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
			mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
			if (!xn_skip(XN_reg_90))
				mipi_dsi_dcs_write_var_seq_multi(&dsi_ctx, 0x90,
						(u8)xn_val(XN_reg_90));
			{
				u8 b[24];
				int n = xn_blob(XN_reg_91, b, sizeof(b));

				if (n)
					mipi_dsi_dcs_write(ctx->dsi[i], 0x91,
							   b, n);
			}
			{
				u8 b[24];
				int n = xn_blob(XN_reg_92, b, sizeof(b));

				if (n)
					mipi_dsi_dcs_write(ctx->dsi[i], 0x92,
							   b, n);
			}
		}

		/* exit_sleep (0x11) and display_on (0x29) used to live here, but
		 * that fires them BEFORE PPS+compression_mode are sent in enable()
		 * → xn242_apply_dsc_mode. Moved to the end of apply_dsc_mode so
		 * the order is: init DCS → PPS → compression_mode → 0x11 → 0x29. */

		if (dsi_ctx.accum_err)
			return dsi_ctx.accum_err;
	}

	msleep(xn_val(XN_init_settle_ms));

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
	/*
	 * Matches the vendor tree and the working SM8750 port (yzddmr6's
	 * panel-novatek-nt36536.c): the clock lane stays in continuous HS.
	 * This DSC video panel drops receiver lock if the clock lane returns
	 * to LP during blanking.  NOTE: the flags that actually reach the DSI
	 * host are the ones xn242_probe() writes before devm_mipi_dsi_attach()
	 * (MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_DSC_ALL_SLICES_IN_PKT, no
	 * NON_CONTINUOUS); this member is kept consistent with them so the
	 * descriptor never suggests otherwise.
	 */
	.mode_flags = MIPI_DSI_MODE_VIDEO,
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
	msleep(xn_val(XN_display_off_ms));
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	msleep(xn_val(XN_sleep_in_ms));

	xn_log_add("panel off: accum_err=%d", dsi_ctx.accum_err);

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

	msleep(xn_val(XN_rail_settle_ms));

	xn242_reset(ctx);

	msleep(xn_val(XN_reset_post_ms));

	ret = ctx->desc->init_sequence(ctx);
	if (ret < 0) {
		regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
		ctx->init_err++;
		ctx->last_err = ret;
		xn_log_add("prepare: init_sequence failed ret=%d", ret);
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

	msleep(xn_val(XN_init_settle_ms));

	ctx->init_ok++;
	ctx->last_err = 0;
	xn_log_add("prepare: init_sequence ok");

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

	msleep(xn_val(XN_pre_enable_ms));

#if IS_ENABLED(CONFIG_TOUCHSCREEN_NT36523_SPI)
	/* erhai: rails are back (prepare() ran) — re-init the in-cell touch. */
	nvt_ts_panel_power_on();
#endif

	if (xn_val(XN_post_enable_ms))
		msleep(xn_val(XN_post_enable_ms));

	return 0;
}

static int xn242_disable(struct drm_panel *panel)
{
	struct xn242 *ctx = to_xn242(panel);
	struct device *dev = &ctx->dsi[0]->dev;
	int ret;

#if IS_ENABLED(CONFIG_TOUCHSCREEN_NT36523_SPI)
	/* erhai: in-cell touch shares these supplies — sleep it BEFORE they drop. */
	nvt_ts_panel_power_off();
#endif

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
		u32 want = xn_val(XN_only_hz);

		if (want && xn242_rate_table[i].hz != want)
			continue;

		mode = drm_mode_duplicate(connector->dev, xn242_rate_table[i].mode);
		if (!mode)
			continue;

		if (want ? true : xn242_rate_table[i].hz == 120)
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

	/* Per-link geometry (tunables; defaults = the values this port shipped). */
	dsc_tmp.pic_width = xn_val(XN_dsc_pic_width_link);
	dsc_tmp.slice_count = xn_val(XN_dsc_slice_count);

	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
		struct drm_dsc_config pps_cfg;
		struct drm_dsc_picture_parameter_set pps;

		if (!ctx->dsi[i])
			continue;

		ctx->dsi[i]->mode_flags |= MIPI_DSI_MODE_LPM;

		/* 0x3B (the panel-internal porch/timing blob) and the page-0x25
		 * auto-VBP/VFP switch — both sequence-gated and both skippable. */
		if (xn_seq(XN_SEQ_P25) && !xn_skip(XN_page25_reg05)) {
			/* Disable auto VBP/VFP (page 0x25 reg 0x05=0) so the 0x3B
			 * porch values below take effect — see nt36532e/sheng.
			 * Without this the panel ignores 0x3B and uses its own
			 * factory vertical timing. */
			mipi_dsi_dcs_write(ctx->dsi[i], 0xff, (u8[]){0x25}, 1);
			mipi_dsi_dcs_write(ctx->dsi[i], 0xfb, (u8[]){0x01}, 1);
			mipi_dsi_dcs_write(ctx->dsi[i], 0x05,
					   (u8[]){ xn_val(XN_page25_reg05) ? 0x01 : 0x00 }, 1);
		}
		if (xn_seq(XN_SEQ_3B)) {
			u8 b[16];
			int n;

			mipi_dsi_dcs_write(ctx->dsi[i], 0xff, (u8[]){0x10}, 1);
			mipi_dsi_dcs_write(ctx->dsi[i], 0xfb, (u8[]){0x01}, 1);
			n = xn_blob(XN_porch_3b, b, sizeof(b));
			if (n)
				mipi_dsi_dcs_write(ctx->dsi[i], 0x3b, b, n);
		}

		if (!xn_yes(XN_dsc_enable))
			continue;

		/* PPS: per-DSI-link parameters. pic_width=1696 (SPLIT_PANEL halves
		 * the 3392 frame across the two links), 2 slices of 848 per link,
		 * chunk_size the computed 848 (agrees with the panel's 0x91). */
		pps_cfg = dsc_tmp;
		drm_dsc_pps_payload_pack(&pps, &pps_cfg);
		mipi_dsi_picture_parameter_set(ctx->dsi[i], &pps);
		msleep(xn_val(XN_pps_gap_ms));

		mipi_dsi_compression_mode(ctx->dsi[i], true);
		msleep(xn_val(XN_comp_gap_ms));

		if (!xn_seq(XN_SEQ_ON))
			continue;

		/* Spec order: exit_sleep AFTER PPS+compression so the decoder is
		 * configured before pixels stream; then display_on. */
		mipi_dsi_dcs_exit_sleep_mode(ctx->dsi[i]);
		msleep(xn_val(XN_exit_sleep_ms));

		mipi_dsi_dcs_set_display_on(ctx->dsi[i]);
	}

	xn_log_add("dsc apply: enable=%d pic_w=%u slices=%u bpp=%u",
		   xn_yes(XN_dsc_enable), xn_val(XN_dsc_pic_width_link),
		   xn_val(XN_dsc_slice_count), xn_val(XN_dsc_bpp));
}

/* Attach/detach the DSC config on the DSI devices (the host reprograms its
 * DSC at the next mode set) and push the geometry tunables into ctx->dsc. */
static void xn_apply_dsc_config(struct xn242 *ctx)
{
	struct drm_dsc_config *dsc = &ctx->dsc;
	int i;

	dsc->bits_per_pixel = xn_val(XN_dsc_bpp);
	dsc->slice_width = xn_val(XN_dsc_slice_width);
	dsc->slice_height = xn_val(XN_dsc_slice_height);
	dsc->slice_count = xn_val(XN_dsc_slice_count);
	dsc->line_buf_depth = xn_val(XN_dsc_line_buf_depth);
	dsc->convert_rgb = xn_yes(XN_dsc_convert_rgb);
	dsc->block_pred_enable = xn_yes(XN_dsc_block_pred);
	dsc->first_line_bpg_offset = xn_val(XN_dsc_first_line_bpg);
	dsc->slice_chunk_size = dsc->slice_width;	/* = slice_width at 8bpp */

	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
		if (!ctx->dsi[i])
			continue;
		ctx->dsi[i]->lanes = xn_val(XN_lanes);
		if (xn_yes(XN_dsc_enable))
			ctx->dsi[i]->dsc = dsc;
		else
			ctx->dsi[i]->dsc = NULL;
	}

	xn_log_add("dsc config: %s bpp=%u slice=%ux%u count=%u lanes=%u",
		   xn_yes(XN_dsc_enable) ? "on" : "OFF", dsc->bits_per_pixel,
		   dsc->slice_width, dsc->slice_height, dsc->slice_count,
		   xn_val(XN_lanes));
}

/* Re-run the panel bring-up with the values that are in force right now.
 *
 *	soft  = re-send the init sequence + PPS + compression + 0x11/0x29,
 *		without touching the rails (fast, panel-side only)
 *	local = the same behind a real rail/reset cycle (touch included)
 *
 * Neither one reprograms the DSI host (that happens at a mode set), which is
 * exactly what makes `local` a cheap per-parameter test: the link keeps
 * streaming and the panel simply re-locks to it.
 */
static int xn_local_cycle(bool rails)
{
	struct xn242 *ctx = xn_ctx;
	struct device *dev;
	int ret = 0;

	if (!ctx)
		return -ENODEV;

	dev = &ctx->dsi[0]->dev;

	mutex_lock(&xn_lock);
	xn_cycles++;
	xn_log_add("reinit %s: start (enabled=%d links=%d)",
		   rails ? "local" : "soft", ctx->enabled,
		   ctx->dsi[1] ? 2 : 1);

	if (rails) {
#if IS_ENABLED(CONFIG_TOUCHSCREEN_NT36523_SPI)
		nvt_ts_panel_power_off();
#endif
		if (ctx->enabled) {
			int off = xn242_off(ctx);

			if (off)
				xn_log_add("reinit: off commands returned %d", off);
		}
		regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		msleep(50);

		ret = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies),
					    ctx->supplies);
		if (ret) {
			dev_err(dev, "erhai panel: reinit, rail enable failed %d\n",
				ret);
			goto out;
		}
		msleep(xn_val(XN_rail_settle_ms));
		xn242_reset(ctx);
		msleep(xn_val(XN_reset_post_ms));
	}

	xn_apply_dsc_config(ctx);

	ret = ctx->desc->init_sequence(ctx);
	if (ret) {
		xn_log_add("reinit: init_sequence failed ret=%d", ret);
		goto out;
	}

	ctx->enabled = true;
	xn242_apply_dsc_mode(ctx);
	msleep(xn_val(XN_pre_enable_ms));

#if IS_ENABLED(CONFIG_TOUCHSCREEN_NT36523_SPI)
	if (rails)
		nvt_ts_panel_power_on();
#endif

	if (xn_val(XN_post_enable_ms))
		msleep(xn_val(XN_post_enable_ms));

out:
	ctx->last_err = ret;
	if (ret)
		ctx->init_err++;
	else
		ctx->init_ok++;

	xn_log_add("reinit %s: done ret=%d cycles=%u", rails ? "local" : "soft",
		   ret, xn_cycles);
	snprintf(xn_msg, sizeof(xn_msg), "reinit %s: ret=%d cycles=%u%s",
		 rails ? "local" : "soft", ret, xn_cycles,
		 rails ? "" : " (sequence only, no rail cycle)");
	mutex_unlock(&xn_lock);

	dev_info(dev, "erhai panel: reinit %s ret=%d cycles=%u\n",
		 rails ? "local" : "soft", ret, xn_cycles);

	return ret;
}

static int xn_msg_get(char *buf, const struct kernel_param *kp)
{
	return sysfs_emit(buf, "%s\n", xn_msg);
}

static int xn_reinit_set(const char *val, const struct kernel_param *kp)
{
	if (!strncmp(val, "local", 5) || !strcmp(val, "1"))
		return xn_local_cycle(true);
	if (!strncmp(val, "soft", 4))
		return xn_local_cycle(false);
	if (!strncmp(val, "status", 6) || !strncmp(val, "none", 4) ||
	    !strcmp(val, "0"))
		return 0;
	return -EINVAL;
}

static const struct kernel_param_ops xn_reinit_ops = {
	.set = xn_reinit_set,
	.get = xn_msg_get,
};

module_param_cb(reinit, &xn_reinit_ops, NULL, 0644);

/* Raw DCS access to the panel:
 *
 *	w <reg> [bytes]		write on BOTH links (w0/w1 = one link)
 *	p <page> <reg> [b..]	page-select (0xff/<page> + 0xfb=0x01), then write
 *	r <reg>			read one byte from link 0
 *
 * `r` reports through the `reinit`/`dcs` value; a failed transfer returns an
 * errno, so `echo r 0x0a > dcs` fails loudly instead of silently lying. */
static int xn_take_hex(const char **pp, unsigned int *out)
{
	const char *p = *pp;
	unsigned int v = 0;
	int digits = 0, d;

	while (*p == ' ' || *p == '\t' || *p == ',')
		p++;
	if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X'))
		p += 2;
	while ((d = xn_hexval(*p)) >= 0) {
		v = (v << 4) | (unsigned int)d;
		digits++;
		p++;
	}
	if (!digits)
		return -EINVAL;
	*pp = p;
	*out = v;
	return 0;
}

static int xn_dcs_set(const char *val, const struct kernel_param *kp)
{
	struct xn242 *ctx = xn_ctx;
	const char *p = val;
	char op[8];
	unsigned int reg = 0, page = 0;
	u8 b[24];
	int len = 0, n = 0, ret = 0, i, lo = 0, hi;

	if (!ctx)
		return -ENODEV;

	while (*p == ' ' || *p == '\t')
		p++;
	while (len < 7 && p[len] && p[len] != ' ' && p[len] != '\t')
		len++;
	memcpy(op, p, len);
	op[len] = 0;
	p += len;
	if (!op[0])
		return -EINVAL;

	if (op[0] == 'r') {
		u8 v[8] = {0};

		if (xn_take_hex(&p, &reg))
			return -EINVAL;

		ret = mipi_dsi_dcs_read(ctx->dsi[0], (u8)reg, v, 1);
		if (ret > 0) {
			ctx->last_read_len = min_t(int, ret,
						   (int)sizeof(ctx->last_read));
			memcpy(ctx->last_read, v, ctx->last_read_len);
			snprintf(xn_msg, sizeof(xn_msg),
				 "r 0x%02x -> ret=%d [%*ph]", reg, ret, ret, v);
		} else {
			snprintf(xn_msg, sizeof(xn_msg),
				 "r 0x%02x -> ret=%d (no answer)", reg, ret);
		}
		xn_log_add("%s", xn_msg);
		return ret > 0 ? 0 : (ret ? ret : -EIO);
	}

	if (op[0] != 'w' && op[0] != 'p')
		return -EINVAL;

	if (op[0] == 'p') {
		if (xn_take_hex(&p, &page))
			return -EINVAL;
	}
	if (xn_take_hex(&p, &reg))
		return -EINVAL;

	n = xn_parse_bytes(p, b, sizeof(b));
	if (n < 0)
		n = 0;			/* no payload = a bare DCS command */

	hi = ARRAY_SIZE(ctx->dsi);
	if (op[1] == '0')
		hi = 1;
	else if (op[1] == '1')
		lo = 1;

	for (i = lo; i < hi; i++) {
		if (!ctx->dsi[i])
			continue;
		ctx->dsi[i]->mode_flags |= MIPI_DSI_MODE_LPM;
		if (op[0] == 'p') {
			mipi_dsi_dcs_write(ctx->dsi[i], 0xff, (u8[]){ (u8)page }, 1);
			mipi_dsi_dcs_write(ctx->dsi[i], 0xfb, (u8[]){ 0x01 }, 1);
		}
		ret = mipi_dsi_dcs_write(ctx->dsi[i], (u8)reg,
					 n ? b : NULL, n);
	}

	if (op[0] == 'p')
		snprintf(xn_msg, sizeof(xn_msg),
			 "p 0x%02x/0x%02x [%*ph] -> ret=%d", page, reg, n, b, ret);
	else
		snprintf(xn_msg, sizeof(xn_msg),
			 "%s 0x%02x [%*ph] -> ret=%d", op, reg, n, b, ret);
	xn_log_add("%s", xn_msg);

	return ret < 0 ? ret : 0;
}

static const struct kernel_param_ops xn_dcs_ops = {
	.set = xn_dcs_set,
	.get = xn_msg_get,
};

module_param_cb(dcs, &xn_dcs_ops, NULL, 0644);

static int xn_dump_show(char *buf, const struct kernel_param *kp)
{
	struct xn242 *ctx = xn_ctx;
	int n = 0, i;

	n += sysfs_emit_at(buf, n, "driver=panel-oneplus-xn242 control-surface=1\n");
	n += sysfs_emit_at(buf, n, "cycles=%u last_action=%s\n", xn_cycles, xn_msg);

	if (!ctx) {
		n += sysfs_emit_at(buf, n, "panel=NOT-PROBED\n");
	} else {
		n += sysfs_emit_at(buf, n,
				   "panel: enabled=%d links=%d init_ok=%u init_err=%u last_err=%d\n",
				   ctx->enabled, ctx->dsi[1] ? 2 : 1,
				   ctx->init_ok, ctx->init_err, ctx->last_err);
		if (ctx->enabled) {
			u8 rddid[3] = {0};
			u8 pm = 0;
			int r1 = mipi_dsi_dcs_read(ctx->dsi[0],
						   MIPI_DCS_GET_DISPLAY_ID,
						   rddid, sizeof(rddid));
			int r2 = mipi_dsi_dcs_read(ctx->dsi[0],
						   MIPI_DCS_GET_POWER_MODE,
						   &pm, 1);

			n += sysfs_emit_at(buf, n,
					   "dcs: rddid ret=%d [%3ph] power_mode ret=%d [0x%02x]\n",
					   r1, rddid, r2, pm);
		} else {
			n += sysfs_emit_at(buf, n,
					   "dcs: panel off, no reads (run reinit)\n");
		}
	}

	n += sysfs_emit_at(buf, n, "modes:");
	for (i = 0; i < ARRAY_SIZE(xn242_rate_table); i++)
		n += sysfs_emit_at(buf, n, " %u", xn242_rate_table[i].hz);
	n += sysfs_emit_at(buf, n, "  (only_hz=%u)\n", xn_val(XN_only_hz));

	for (i = 0; i < (int)XN_IDX_MAX; i++)
		n += sysfs_emit_at(buf, n, "%s=%s\n", xn_tuns[i].name,
				   xn_tuns[i].val);

	return n;
}

static int xn_log_show(char *buf, const struct kernel_param *kp)
{
	if (!xn_log_used)
		return sysfs_emit(buf, "(empty)\n");

	return sysfs_emit(buf, "%s", xn_log);
}

static const struct kernel_param_ops xn_dump_ops = {
	.get = xn_dump_show,
};

static const struct kernel_param_ops xn_log_ops = {
	.get = xn_log_show,
};

module_param_cb(dump, &xn_dump_ops, NULL, 0444);
module_param_cb(log, &xn_log_ops, NULL, 0444);

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
	ctx->dsc.first_line_bpg_offset = xn_val(XN_dsc_first_line_bpg);
	if (xn_yes(XN_dsc_otp_rc)) {
		for (i = 0; i < ARRAY_SIZE(xn242_otp_rc_range); i++) {
			ctx->dsc.rc_range_params[10 + i].range_min_qp =
				xn242_otp_rc_range[i].range_min_qp;
			ctx->dsc.rc_range_params[10 + i].range_max_qp =
				xn242_otp_rc_range[i].range_max_qp;
		}
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

	/*
	 * erhai panel control surface: the DSC attach + geometry and the DSI
	 * lane count come from the tunables (defaults = the values above), so
	 * they can be changed from the terminal and re-applied by `reinit`.
	 */
	xn_ctx = ctx;
	xn_apply_dsc_config(ctx);
	xn_log_add("probe: control surface ready (links=%d, dsc=%s, lanes=%u)",
		   ctx->dsi[1] ? 2 : 1, xn_yes(XN_dsc_enable) ? "on" : "off",
		   xn_val(XN_lanes));

	return 0;
}

static void xn242_remove(struct mipi_dsi_device *dsi)
{
	struct xn242 *ctx = mipi_dsi_get_drvdata(dsi);

	xn_ctx = NULL;
	xn_log_add("remove: panel control detached");
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
