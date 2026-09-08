// SPDX-License-Identifier: GPL-2.0-only
/*
 * OnePlus Pad 3 (erhai) magnetic pogo keyboard driver.
 *
 * The pogo keyboard is a half-duplex one-wire-bus (OWB) UART device on the
 * GENI QUP serial engine at 0x880000 (ttyHS5 on Android), 921600 8N1 raw.
 * The keyboard streams HID-boot-style key reports, media-key reports and a
 * multi-touch touchpad stream autonomously once powered; a periodic sync
 * (heartbeat) packet doubles as the plug-present signal.
 *
 * This is a from-scratch mainline serdev port of the OnePlus vendor driver
 * (oplus device_info pogo_keyboard). The OWB framing/CRC, the keymap and
 * the touchpad decode are ported verbatim; the vendor's msm_geni_serial hooks
 * + kernel-space tty-file I/O are replaced by the serdev RX/TX path, and the
 * power/tx-en/wake lines are plain gpiods.
 *
 * v1 is receive-only (no host TX): power the keyboard, hold the tx-enable
 * buffer in receive, parse the inbound OWB stream and report input. Host TX
 * (caps-lock LED feedback, SN/version query) is cosmetic and can be added
 * later behind the tx-enable gpio toggle.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/serdev.h>
#include <linux/slab.h>

/* ---- OWB protocol constants (owb.h) ---- */
#define OWB_HEAD_SYNC		0x55
#define OWB_TAIL_SYNC		0xAA
#define OWB_START_CODE		0xF1
#define OWB_REPEAT_CODE		0xF2
#define OWB_END_CODE		0xFE
#define OWB_ADDR_KEYBOARD	0xA1
#define OWB_ADDR_PAD		0xA2

#define OWB_CMD_GENERAL_KEY	0x01
#define OWB_CMD_MEDIA_KEY	0x02
#define OWB_CMD_TOUCHPAD	0x03
#define OWB_CMD_MOUSE		0x04
#define OWB_CMD_SYNC_UPLOAD	0x2F	/* power-up (sub 0x01) + heartbeat (sub 0x05) */

/* CRC-16/IBM as used by the keyboard MCU: poly 0x8005, MSB-first, init 0xC596 */
#define OWB_CRC_POLY		0x8005
#define OWB_CRC_INIT		0xC596

#define OWB_BUF_SIZE		256
#define TOUCH_FINGER_MAX	5
#define KEYBOARD_NUM_KEYS	256
#define KEYBOARD_MM_NUM_KEYS	23

/* vendor-private keycodes (pogo_keyboard.h) used by the media-key table */
#define KEY_LOCKSCREEN		0x280
#define KEY_SWITCHLANUAGE	0x282
#define KEY_MICDISABLE		0x283
#define KEY_TOUCHPANELMUTE	0x284
#define KEY_GLOBALSEARCH	0x285
#define KEY_FULLSCREEN		0x286
#define KEY_SPLITSCREEN		0x287
#define KEY_SUPERINTCON		0x289
#define KEY_CUSTOMERAPP1	0x28a
#define KEY_CUSTOMERAPP2	0x28b
#ifndef KEY_AI
#define KEY_AI			0x28c
#endif

struct touch_event {
	int id;
	int is_down;
	int is_left;
	int is_right;
	int x;
	int y;
	int area;
};

struct pogo_kbd {
	struct serdev_device *serdev;
	struct device *dev;

	struct gpio_desc *power_gpio;	/* gpio85 - pogo power enable */
	struct gpio_desc *txen_gpio;	/* gpio83 - uart tx direction enable */
	struct gpio_desc *wake_gpio;	/* gpio96 - plug/wake detect (unused in v1) */
	struct delayed_work poll_work;	/* power-cycle/detect/keepalive poll */
	unsigned long last_rx;		/* jiffies of last valid frame */
	int fail_cycles;		/* consecutive detect cycles with no data */
	bool got_data;

	struct input_dev *kbd;
	struct input_dev *touchpad;
	struct input_dev *switch_dev;
	bool tablet_mode;

	/* detection diagnostics (per poll cycle) */
	u32 rx_frames;		/* completed OWB frames since probe */
	u32 rx_bad;		/* completed frames that failed CRC */
	u32 rx_frames_mark;	/* rx_frames at the last dead poll */
	u32 rx_bad_mark;	/* rx_bad at the last dead poll */

	u32 tp_x_max, tp_y_max, tp_x_res, tp_y_res;
	bool touch_support;
	u16 crc_init;

	/* OWB receive state machine (persists across serdev receive_buf calls) */
	bool recv_start;
	u8 head_sync_cnt;
	u16 idx;
	u8 recv_buf[OWB_BUF_SIZE];

	/* keyboard state (HID boot protocol: 1 ctrl + 1 rsvd + 6 keys) */
	u8 old[8];
	u8 cur[8];
	u16 mm_old[2];
	u16 mm_new[2];

	/* touchpad state */
	u32 touch_down;
	u32 touch_temp;
	int prev_finger_count;
	u8 touchpad_data[5 + TOUCH_FINGER_MAX * 5];
	struct touch_event event;
};

/* HID usage -> Linux keycode, mirrors usbkbd.c / vendor keyboard.c verbatim */
#define unk KEY_UNKNOWN
static const unsigned char pogo_keymap[KEYBOARD_NUM_KEYS] = {
	  0,  0,  0,  0, 30, 48, 46, 32, 18, 33, 34, 35, 23, 36, 37, 38,
	 50, 49, 24, 25, 16, 19, 31, 20, 22, 47, 17, 45, 21, 44,  2,  3,
	  4,  5,  6,  7,  8,  9, 10, 11, 28,  1, 14, 15, 57, 12, 13, 26,
	 27, 43, 43, 39, 40, 41, 51, 52, 53, 58, 59, 60, 61, 62, 63, 64,
	 65, 66, 67, 68, 87, 88, 99, 70,119,110,102,104,111,107,109,106,
	105,108,103, 69, 98, 55, 74, 78, 96, 79, 80, 81, 75, 76, 77, 71,
	 72, 73, 82, 83, 86,127,116,117,183,184,185,186,187,188,189,190,
	191,192,193,194,134,138,130,132,128,129,131,137,133,135,136,113,
	115,114,unk,unk,unk,121,unk, 89, 93,124, 92, 94, 95,unk,unk,unk,
	122,123, 90, 91, 85,unk,unk,unk,unk,unk,unk,unk,111,unk,unk,unk,
	unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,
	unk,unk,unk,unk,unk,unk,179,180,unk,unk,unk,unk,unk,unk,unk,unk,
	unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,
	unk,unk,unk,unk,unk,unk,unk,unk,111,unk,unk,unk,unk,unk,unk,unk,
	 29, 42, 56,125, 97, 54,100,126,164,166,165,163,161,115,114,113,
	150,158,159,128,136,177,178,176,142,152,173,140,unk,unk,unk,unk
};

static const unsigned short mm_keymap[KEYBOARD_MM_NUM_KEYS][2] = {
	{0x70,  KEY_BRIGHTNESSDOWN},
	{0x6f,  KEY_BRIGHTNESSUP},
	{0xe2,  KEY_MUTE},
	{0xea,  KEY_VOLUMEDOWN},
	{0xe9,  KEY_VOLUMEUP},
	{0x224, KEY_ESC},
	{0x38e, KEY_LOCKSCREEN},
	{0x390, KEY_SWITCHLANUAGE},
	{0x391, KEY_MICDISABLE},
	{0x392, KEY_TOUCHPANELMUTE},
	{0x393, KEY_GLOBALSEARCH},
	{0x394, KEY_FULLSCREEN},
	{0x395, KEY_SPLITSCREEN},
	{0x397, KEY_SUPERINTCON},
	{0x398, KEY_CUSTOMERAPP1},
	{0x399, KEY_CUSTOMERAPP2},
	{0xcd,  KEY_PLAYPAUSE},
	{0xb5,  KEY_NEXTSONG},
	{0xb6,  KEY_PREVIOUSSONG},
	{0x244, KEY_APPSELECT},
	{0x2fa, KEY_AI},
	{0x1d0, KEY_FN},
	{unk,   unk},
};

/* ---- CRC-16/IBM (app_compute_crc16 / app_crc16_get) ---- */
static u16 owb_crc16(const u8 *buf, int len, u16 init)
{
	u16 crc = init;
	int i, b;

	for (i = 0; i < len; i++) {
		u8 data = buf[i];

		for (b = 0; b < 8; b++) {
			if ((((crc & 0x8000) >> 8) ^ (data & 0x80)) != 0)
				crc = (crc << 1) ^ OWB_CRC_POLY;
			else
				crc = crc << 1;
			data <<= 1;
		}
	}
	return crc;
}

/* ---- keyboard report (vendor pogo_keyboard_input_report) ---- */
static void pogo_kbd_report_keys(struct pogo_kbd *kb, const u8 *buf)
{
	struct input_dev *dev = kb->kbd;
	int i;

	/* 8 bytes: 1 ctrl byte + 1 reserved + 6 key scancodes */
	memcpy(kb->cur, &buf[1], 8);

	for (i = 0; i < 8; i++)
		input_report_key(dev, pogo_keymap[i + 224], (kb->cur[0] >> i) & 1);

	for (i = 2; i < 8; i++) {
		if (kb->old[i] > 3 &&
		    memscan(kb->cur + 2, kb->old[i], 6) == kb->cur + 8) {
			if (pogo_keymap[kb->old[i]])
				input_report_key(dev, pogo_keymap[kb->old[i]], 0);
		}
		if (kb->cur[i] > 3 &&
		    memscan(kb->old + 2, kb->cur[i], 6) == kb->old + 8) {
			if (pogo_keymap[kb->cur[i]])
				input_report_key(dev, pogo_keymap[kb->cur[i]], 1);
		}
	}

	input_sync(dev);
	memcpy(kb->old, kb->cur, 8);
}

static void *memscan_ex(void *addr, int c, size_t size)
{
	u8 *p = addr;

	while (size) {
		u16 key = *p | ((*(p + 1)) << 8);

		if (key == c)
			return p;
		p += 2;
		size -= 2;
	}
	return p;
}

/* ---- media-key report (vendor pogo_keyboard_mm_input_report) ---- */
static void pogo_kbd_report_mm(struct pogo_kbd *kb, const u8 *buf)
{
	struct input_dev *dev = kb->kbd;
	int i, j;

	memcpy(kb->mm_new, &buf[1], 4);
	for (i = 0; i < 2; i++) {
		u16 kw = kb->mm_old[i];

		if (memscan_ex(kb->mm_new, kw, 4) == (void *)(kb->mm_new + 2) && kw) {
			for (j = 0; j < KEYBOARD_MM_NUM_KEYS; j++)
				if (kw == mm_keymap[j][0])
					break;
			if (j < KEYBOARD_MM_NUM_KEYS)
				input_report_key(dev, mm_keymap[j][1], 0);
		}

		kw = kb->mm_new[i];
		if (memscan_ex(kb->mm_old, kw, 4) == (void *)(kb->mm_old + 2) && kw) {
			for (j = 0; j < KEYBOARD_MM_NUM_KEYS; j++)
				if (kw == mm_keymap[j][0])
					break;
			if (j < KEYBOARD_MM_NUM_KEYS)
				input_report_key(dev, mm_keymap[j][1], 1);
		}
	}

	input_sync(dev);
	memcpy(kb->mm_old, kb->mm_new, 4);
}

/* ---- touchpad report (vendor touchpad_input_report) ---- */
static void reset_tool_buttons(struct input_dev *dev)
{
	input_report_key(dev, BTN_TOOL_FINGER, 0);
	input_report_key(dev, BTN_TOOL_DOUBLETAP, 0);
	input_report_key(dev, BTN_TOOL_TRIPLETAP, 0);
	input_report_key(dev, BTN_TOOL_QUADTAP, 0);
	input_report_key(dev, BTN_TOOL_QUINTTAP, 0);
}

static void report_tool_button(struct input_dev *dev, int fingers)
{
	switch (fingers) {
	case 1: input_report_key(dev, BTN_TOOL_FINGER, 1); break;
	case 2: input_report_key(dev, BTN_TOOL_DOUBLETAP, 1); break;
	case 3: input_report_key(dev, BTN_TOOL_TRIPLETAP, 1); break;
	case 4: input_report_key(dev, BTN_TOOL_QUADTAP, 1); break;
	case 5: input_report_key(dev, BTN_TOOL_QUINTTAP, 1); break;
	default: break;
	}
}

static void pogo_kbd_report_touch(struct pogo_kbd *kb, const u8 *buf)
{
	struct input_dev *dev = kb->touchpad;
	struct touch_event *event = &kb->event;
	struct touch_event temp;
	u8 *data = kb->touchpad_data;
	int i, j, fingers, len, offset, finger_count = 0;
	u8 palm;
	u32 xarea;

	if (!dev)
		return;

	len = buf[0];
	if (len <= 0 || len > sizeof(kb->touchpad_data))
		return;
	fingers = buf[len - 1];
	if (fingers < 0 || fingers > TOUCH_FINGER_MAX)
		return;

	if (!memcmp(data, &buf[1], fingers * 5) &&
	    !memcmp(&data[len - 1], &buf[len], 1))
		return;

	memset(&temp, 0, sizeof(temp));
	memcpy(data, &buf[1], len);
	temp.is_left = data[len - 1] & 0x01;
	temp.is_right = (data[len - 1] >> 1) & 0x01;

	for (j = 0; j < fingers; j++) {
		offset = 5 * j;
		temp.id = (data[offset] >> 4) & 0xf;
		temp.is_down = (data[offset] >> 1) & 0x01;
		palm = data[offset] & 0x03;
		temp.x = data[offset + 1] | data[offset + 2] << 8;
		temp.y = data[offset + 3] | data[offset + 4] << 8;
		xarea = kb->tp_x_max / 3;
		temp.area = (temp.x > xarea && temp.x < 2 * xarea) ? 1 : 0;

		if (temp.id >= TOUCH_FINGER_MAX)
			continue;

		input_mt_slot(dev, temp.id);
		if (temp.is_down) {
			input_mt_report_slot_state(dev, MT_TOOL_FINGER, true);
			input_report_key(dev, BTN_TOUCH, 1);
			kb->touch_down |= BIT(temp.id);
			kb->touch_temp |= BIT(temp.id);
			input_report_abs(dev, ABS_MT_POSITION_X, temp.x);
			input_report_abs(dev, ABS_MT_POSITION_Y, temp.y);
			if (palm == 0x02) {
				input_report_abs(dev, ABS_MT_TOUCH_MINOR, 255);
				input_report_abs(dev, ABS_MT_TOUCH_MAJOR, 255);
			}
			finger_count++;
		} else {
			input_mt_report_slot_state(dev, MT_TOOL_FINGER, false);
			kb->touch_down &= ~BIT(temp.id);
			if (unlikely(kb->touch_down ^ kb->touch_temp)) {
				for (i = 0; i < TOUCH_FINGER_MAX; i++) {
					if (BIT(i) & (kb->touch_down ^ kb->touch_temp)) {
						input_mt_slot(dev, i);
						input_mt_report_slot_state(dev, MT_TOOL_FINGER, false);
					}
				}
			}
			if (!kb->touch_down) {
				for (i = 0; i < TOUCH_FINGER_MAX; i++) {
					input_mt_slot(dev, i);
					input_report_abs(dev, ABS_MT_TOUCH_MINOR, 0);
					input_report_abs(dev, ABS_MT_TOUCH_MAJOR, 0);
					input_mt_report_slot_state(dev, MT_TOOL_FINGER, false);
				}
				input_report_key(dev, BTN_TOUCH, 0);
				reset_tool_buttons(dev);
				kb->touch_temp = 0;
				kb->touch_down = 0;
				kb->prev_finger_count = 0;
				input_sync(dev);
				return;
			}
		}
	}

	if (kb->prev_finger_count != finger_count) {
		reset_tool_buttons(dev);
		kb->prev_finger_count = finger_count;
		report_tool_button(dev, kb->prev_finger_count);
	}

	if (fingers > 0) {
		if (temp.is_left || (event->is_left && !temp.is_left)) {
			event->is_left = temp.is_left;
			input_report_key(dev, BTN_LEFT, temp.is_left);
		}
		if (temp.is_right || (event->is_right && !temp.is_right)) {
			event->is_right = temp.is_right;
			input_report_key(dev, BTN_RIGHT, temp.is_right);
		}
	}
	input_sync(dev);
}

/* ---- OWB frame dispatch (vendor pogo_keyboard_mod_data_process) ---- */
static void pogo_kbd_dispatch(struct pogo_kbd *kb, const u8 *buf, int len)
{
	/* a valid frame arrived: keyboard is streaming and present */
	kb->got_data = true;
	kb->last_rx = jiffies;
	kb->fail_cycles = 0;

	switch (buf[0]) {
	case OWB_CMD_GENERAL_KEY:
		pogo_kbd_report_keys(kb, &buf[1]);
		break;
	case OWB_CMD_MEDIA_KEY:
		pogo_kbd_report_mm(kb, &buf[1]);
		break;
	case OWB_CMD_TOUCHPAD:
		if (kb->touch_support)
			pogo_kbd_report_touch(kb, &buf[1]);
		break;
	case OWB_CMD_SYNC_UPLOAD:
		/* power-up (sub 0x01) / heartbeat (sub 0x05): plug-present beacon */
		if (buf[2] == 0x01)
			dev_info(kb->dev, "keyboard plug-in (power-up sync)\n");
		break;
	default:
		break;
	}
}

/* ---- OWB validity check (vendor pogo_keyboard_data_is_valid) ----
 * recv_buf layout: [F1, src, dst, maincmd, len, data..., crc_hi, crc_lo, FE, AA, AA]
 */
static bool pogo_kbd_frame_valid(struct pogo_kbd *kb, const u8 *buf, int len)
{
	u8 flag;
	u16 crc;

	if (len < 5)
		return false;
	flag = buf[2];
	if (flag != OWB_ADDR_KEYBOARD && flag != OWB_ADDR_PAD)
		return false;
	crc = (buf[len - 5] << 8) | buf[len - 4];
	return crc == owb_crc16(buf, len - 5, kb->crc_init);
}

/* ---- OWB receive state machine (vendor pogo_keyboard_recv) ----
 * Persistent state in @kb so frames may span multiple serdev callbacks.
 */
static void pogo_kbd_recv(struct pogo_kbd *kb, const u8 *buf, size_t len)
{
	size_t i;

	for (i = 0; i < len; i++) {
		u8 rx = buf[i];

		if (rx == OWB_HEAD_SYNC) {
			kb->head_sync_cnt++;
		} else if (rx == OWB_START_CODE || rx == OWB_REPEAT_CODE) {
			if (kb->head_sync_cnt >= 4) {
				kb->idx = 0;
				kb->recv_start = true;
			}
			kb->head_sync_cnt = 0;
		} else {
			kb->head_sync_cnt = 0;
		}

		if (kb->recv_start) {
			if (kb->idx >= OWB_BUF_SIZE) {
				kb->idx = 0;
				kb->recv_start = false;
				continue;
			}
			kb->recv_buf[kb->idx++] = rx;

			if (kb->idx >= 3 &&
			    kb->recv_buf[kb->idx - 1] == OWB_TAIL_SYNC &&
			    kb->recv_buf[kb->idx - 2] == OWB_TAIL_SYNC &&
			    kb->recv_buf[kb->idx - 3] == OWB_END_CODE) {
				kb->recv_start = false;
				if (kb->idx > 6 &&
				    pogo_kbd_frame_valid(kb, kb->recv_buf, kb->idx)) {
					/* strip [F1,src,dst] head + [FE,AA,AA] tail */
					pogo_kbd_dispatch(kb, &kb->recv_buf[3],
							  kb->idx - 6);
				} else {
					kb->rx_bad++;
				}
				kb->rx_frames++;
				kb->idx = 0;
			}
		}
	}
}

static size_t pogo_kbd_receive_buf(struct serdev_device *serdev,
				   const u8 *buf, size_t count)
{
	struct pogo_kbd *kb = serdev_device_get_drvdata(serdev);

	pogo_kbd_recv(kb, buf, count);
	return count;
}

static const struct serdev_device_ops pogo_kbd_serdev_ops = {
	.receive_buf = pogo_kbd_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};

/* ---- input device setup ---- */
static int pogo_kbd_register_kbd(struct pogo_kbd *kb)
{
	struct input_dev *dev;
	int i;

	dev = devm_input_allocate_device(kb->dev);
	if (!dev)
		return -ENOMEM;

	dev->name = "OnePlus Pad 3 Keyboard";
	dev->phys = "pogo-kbd/input0";
	dev->id.bustype = BUS_HOST;
	dev->id.vendor = 0x22d9;
	dev->id.product = 0x3869;
	dev->id.version = 0x0010;

	__set_bit(EV_KEY, dev->evbit);
	__set_bit(EV_LED, dev->evbit);
	__set_bit(LED_CAPSL, dev->ledbit);
	__set_bit(KEY_TOUCHPAD_TOGGLE, dev->keybit);

	for (i = 0; i < KEYBOARD_NUM_KEYS; i++)
		if (pogo_keymap[i] && pogo_keymap[i] != unk)
			__set_bit(pogo_keymap[i], dev->keybit);
	for (i = 0; i < KEYBOARD_MM_NUM_KEYS; i++)
		if (mm_keymap[i][1] && mm_keymap[i][1] != unk)
			__set_bit(mm_keymap[i][1], dev->keybit);

	kb->kbd = dev;
	return input_register_device(dev);
}

static int pogo_kbd_register_touchpad(struct pogo_kbd *kb)
{
	struct input_dev *dev;
	int ret;

	dev = devm_input_allocate_device(kb->dev);
	if (!dev)
		return -ENOMEM;

	dev->name = "OnePlus Pad 3 Touchpad";
	dev->phys = "pogo-kbd/input1";
	dev->id.bustype = BUS_HOST;
	dev->id.vendor = 0x22d9;
	dev->id.product = 0x3869;
	dev->id.version = 0x0010;

	__set_bit(EV_KEY, dev->evbit);
	__set_bit(EV_ABS, dev->evbit);
	__set_bit(BTN_TOUCH, dev->keybit);
	__set_bit(BTN_LEFT, dev->keybit);
	__set_bit(BTN_RIGHT, dev->keybit);
	__set_bit(BTN_TOOL_FINGER, dev->keybit);
	__set_bit(BTN_TOOL_DOUBLETAP, dev->keybit);
	__set_bit(BTN_TOOL_TRIPLETAP, dev->keybit);
	__set_bit(BTN_TOOL_QUADTAP, dev->keybit);
	__set_bit(BTN_TOOL_QUINTTAP, dev->keybit);
	__set_bit(INPUT_PROP_POINTER, dev->propbit);
	__set_bit(INPUT_PROP_BUTTONPAD, dev->propbit);

	input_set_abs_params(dev, ABS_MT_POSITION_X, 0, kb->tp_x_max, 0, 0);
	input_set_abs_params(dev, ABS_MT_POSITION_Y, 0, kb->tp_y_max, 0, 0);
	input_set_abs_params(dev, ABS_MT_TOUCH_MAJOR, 0, 100, 0, 0);
	input_set_abs_params(dev, ABS_MT_TOUCH_MINOR, 0, 100, 0, 0);
	input_abs_set_res(dev, ABS_MT_POSITION_X, kb->tp_x_res);
	input_abs_set_res(dev, ABS_MT_POSITION_Y, kb->tp_y_res);

	ret = input_mt_init_slots(dev, TOUCH_FINGER_MAX, INPUT_MT_POINTER);
	if (ret)
		return ret;

	kb->touchpad = dev;
	return input_register_device(dev);
}

/*
 * SW_TABLET_MODE: mutter/gnome only allow screen auto-rotation while the seat
 * is in touch mode, which follows this switch (no switch present => pointer
 * attached => laptop mode => no rotation). The keyboard is what turns the
 * tablet into a laptop, so report tablet mode whenever the keyboard stream is
 * not alive, and laptop mode while it is attached. This also gates the native
 * Rotation Lock quick toggle.
 */
static int pogo_kbd_register_switch(struct pogo_kbd *kb)
{
	struct input_dev *dev;
	int ret;

	dev = devm_input_allocate_device(kb->dev);
	if (!dev)
		return -ENOMEM;

	dev->name = "OnePlus Pad 3 Tablet Mode Switch";
	dev->phys = "pogo-kbd/input2";
	dev->id.bustype = BUS_HOST;
	dev->id.vendor = 0x22d9;
	dev->id.product = 0x3869;
	dev->id.version = 0x0010;

	__set_bit(EV_SW, dev->evbit);
	__set_bit(SW_TABLET_MODE, dev->swbit);

	/* The keyboard only streams after the detect poll powers it: we start
	 * detached, i.e. in tablet mode. */
	kb->tablet_mode = true;

	kb->switch_dev = dev;
	ret = input_register_device(dev);
	if (ret)
		return ret;

	input_report_switch(dev, SW_TABLET_MODE, 1);
	input_sync(dev);
	return 0;
}

#define POGO_OFF_MS		1000	/* rail-off time to force a keyboard reset */
#define POGO_DEEP_OFF_MS	3000	/* longer rail-off for the periodic deep reset */
#define POGO_DEEP_CYCLES	20	/* deep reset every Nth failed cycle */
#define POGO_POLL_MS		3000	/* keepalive / detect cadence */
#define POGO_RX_TIMEOUT_MS	6000	/* no frame this long => detached */
#define POGO_BACKOFF_CYCLES	8	/* slow down after this many empty cycles */
#define POGO_BACKOFF_MS		5000	/* idle poll cadence (short: snappy attach) */

/*
 * Self-healing power/detect poll (no plug IRQ — gpio96 floats and storms when
 * the keyboard is off).
 *
 * The keyboard only starts streaming after a clean power-up edge; if it was
 * left powered by a prior OS (Android across a warm reboot) it sits idle until
 * the rail is actually cycled. So while no frames are arriving, power-cycle the
 * rail (off long enough to reset, then on) once per poll until the keyboard
 * answers. Once it is streaming, its ~2 s heartbeat keeps last_rx fresh and we
 * just monitor; if it later stops (unplugged), we resume cycling — which makes
 * detach/reattach self-heal. Backs off to a slow poll when nothing is attached.
 *
 * Runs in a workqueue, so the rail-off sleep here is fine.
 */
static void pogo_kbd_poll_work(struct work_struct *work)
{
	struct pogo_kbd *kb = container_of(work, struct pogo_kbd, poll_work.work);
	unsigned long delay = POGO_POLL_MS;
	bool alive = kb->got_data &&
		     time_before(jiffies,
				 kb->last_rx + msecs_to_jiffies(POGO_RX_TIMEOUT_MS));

	/* Keyboard attached => laptop mode, detached => tablet mode. */
	if (kb->tablet_mode != !alive) {
		kb->tablet_mode = !alive;
		dev_info(kb->dev, "%s mode\n", !alive ? "tablet" : "laptop");
	}

	/*
	 * Re-assert the current switch state every poll cycle: compositors
	 * (mutter) only learn switch state from toggle events and never query
	 * it, so a state set before the compositor started would otherwise be
	 * invisible forever. Duplicate reports are harmless.
	 */
	input_report_switch(kb->switch_dev, SW_TABLET_MODE, kb->tablet_mode);
	input_sync(kb->switch_dev);

	if (!alive) {
		if (kb->got_data)
			dev_info(kb->dev, "keyboard stream lost; re-detecting\n");
		kb->got_data = false;

		/*
		 * Diagnostics for the "attached but never recognised" state:
		 * completed OWB frames with bad CRC mean the OWB line is alive
		 * and the keyboard is streaming garbage (rebooting, or noise),
		 * while total frame silence means it is not powered at all.
		 */
		if (kb->rx_frames != kb->rx_frames_mark)
			dev_info(kb->dev,
				 "detect: %u OWB frames since last poll (%u bad CRC)\n",
				 kb->rx_frames - kb->rx_frames_mark,
				 kb->rx_bad - kb->rx_bad_mark);
		kb->rx_frames_mark = kb->rx_frames;
		kb->rx_bad_mark = kb->rx_bad;

		/* drop any partial frame across the rail bounce */
		kb->recv_start = false;
		kb->idx = 0;
		kb->head_sync_cnt = 0;

		/* power-cycle to (re)detect; periodic deep reset for a wedged keyboard */
		gpiod_set_value_cansleep(kb->power_gpio, 0);
		msleep(kb->fail_cycles &&
		       kb->fail_cycles % POGO_DEEP_CYCLES == 0 ?
		       POGO_DEEP_OFF_MS : POGO_OFF_MS);
		gpiod_set_value_cansleep(kb->power_gpio, 1);

		if (kb->fail_cycles++ > POGO_BACKOFF_CYCLES)
			delay = POGO_BACKOFF_MS;	/* nothing attached — relax */
	}

	schedule_delayed_work(&kb->poll_work, msecs_to_jiffies(delay));
}

static int pogo_kbd_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	struct pogo_kbd *kb;
	u32 xy[2];
	int ret;

	kb = devm_kzalloc(dev, sizeof(*kb), GFP_KERNEL);
	if (!kb)
		return -ENOMEM;

	kb->serdev = serdev;
	kb->dev = dev;
	kb->crc_init = OWB_CRC_INIT;
	serdev_device_set_drvdata(serdev, kb);
	INIT_DELAYED_WORK(&kb->poll_work, pogo_kbd_poll_work);

	/* touchpad geometry (touchpad-xy-max / -resolution) */
	if (!of_property_read_u32_array(dev->of_node, "touchpad-xy-max", xy, 2)) {
		kb->touch_support = true;
		kb->tp_x_max = xy[0];
		kb->tp_y_max = xy[1];
		if (of_property_read_u32_array(dev->of_node,
					       "touchpad-xy-resolution", xy, 2) == 0) {
			kb->tp_x_res = xy[0];
			kb->tp_y_res = xy[1];
		}
	}

	/*
	 * power-enable (gpio85): keep the keyboard rail OFF for now. We power
	 * it on only once serdev is open (below), and power-cycle it so the
	 * keyboard re-runs its init and re-sends the power-up sync burst even
	 * if it was left powered by a prior OS (Android) — otherwise it streams
	 * nothing until a manual power cycle.
	 */
	kb->power_gpio = devm_gpiod_get(dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(kb->power_gpio))
		return dev_err_probe(dev, PTR_ERR(kb->power_gpio),
				     "failed to get power gpio\n");

	/* tx-enable buffer (gpio83): low = receive mode (v1 is RX-only) */
	kb->txen_gpio = devm_gpiod_get(dev, "tx-enable", GPIOD_OUT_LOW);
	if (IS_ERR(kb->txen_gpio))
		return dev_err_probe(dev, PTR_ERR(kb->txen_gpio),
				     "failed to get tx-enable gpio\n");

	/*
	 * wake/plug-detect (gpio96): consumed so the DT pin is owned, but NOT
	 * used as an IRQ — it floats and edge-storms when the keyboard is off,
	 * which corrupts detection. Plug/unplug is handled by the keepalive poll.
	 */
	kb->wake_gpio = devm_gpiod_get_optional(dev, "wake", GPIOD_IN);
	if (IS_ERR(kb->wake_gpio))
		return dev_err_probe(dev, PTR_ERR(kb->wake_gpio),
				     "failed to get wake gpio\n");

	ret = pogo_kbd_register_kbd(kb);
	if (ret)
		return dev_err_probe(dev, ret, "register keyboard failed\n");
	if (kb->touch_support) {
		ret = pogo_kbd_register_touchpad(kb);
		if (ret)
			return dev_err_probe(dev, ret, "register touchpad failed\n");
	}
	ret = pogo_kbd_register_switch(kb);
	if (ret)
		return dev_err_probe(dev, ret, "register tablet-mode switch failed\n");

	serdev_device_set_client_ops(serdev, &pogo_kbd_serdev_ops);
	ret = devm_serdev_device_open(dev, serdev);
	if (ret)
		return dev_err_probe(dev, ret, "serdev open failed\n");

	serdev_device_set_baudrate(serdev, 921600);
	serdev_device_set_flow_control(serdev, false);
	serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);

	/*
	 * The rail was forced off at gpiod_get (GPIOD_OUT_LOW). serdev is now
	 * listening; kick off the keepalive/detect poll (it sleeps for the
	 * rail-off time, so it must not run inline here).
	 */
	schedule_delayed_work(&kb->poll_work, msecs_to_jiffies(200));

	dev_info(dev, "OnePlus pogo keyboard ready (921600 8N1, touchpad %ux%u)\n",
		 kb->tp_x_max, kb->tp_y_max);
	return 0;
}

static void pogo_kbd_remove(struct serdev_device *serdev)
{
	struct pogo_kbd *kb = serdev_device_get_drvdata(serdev);

	cancel_delayed_work_sync(&kb->poll_work);
	gpiod_set_value_cansleep(kb->power_gpio, 0);
}

static const struct of_device_id pogo_kbd_of_match[] = {
	{ .compatible = "oneplus,erhai-pogo-keyboard" },
	{ }
};
MODULE_DEVICE_TABLE(of, pogo_kbd_of_match);

static struct serdev_device_driver pogo_kbd_driver = {
	.probe = pogo_kbd_probe,
	.remove = pogo_kbd_remove,
	.driver = {
		.name = "oneplus-pogo-kbd",
		.of_match_table = pogo_kbd_of_match,
	},
};
module_serdev_device_driver(pogo_kbd_driver);

MODULE_DESCRIPTION("OnePlus Pad 3 pogo magnetic keyboard (OWB UART) driver");
MODULE_LICENSE("GPL");
