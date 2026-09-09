#!/usr/bin/env python3
"""
Apply the Fn-key patch to oneplus-pogo-kbd.c (OnePlus Pad 3 pogo keyboard).

- magnifier key (HID scancode 186 / KEY_F23) becomes the Fn modifier
- Fn + media keys -> F1..F12
- anchor-based, no line numbers: aborts without modifying if anything
  doesn't match exactly.
"""
import shutil
import sys

FN = "oneplus-pogo-kbd.c"

try:
    src = open(FN).read()
except FileNotFoundError:
    sys.exit(f"ERROR: {FN} not found in this directory.")

if "pogo_fn_remap" in src:
    sys.exit("Patch seems already applied (pogo_fn_remap found). Aborting.")


def replace_once(text, old, new, what):
    n = text.count(old)
    if n != 1:
        sys.exit(f"ERROR: anchor for '{what}' found {n} times (expected 1).\n"
                 "File NOT modified.")
    print(f"  ok: {what}")
    return text.replace(old, new)


# ---- 1. add fn_held to struct pogo_kbd ------------------------------------
src = replace_once(
    src,
    "\tu16 mm_new[2];\n",
    "\tu16 mm_new[2];\n"
    "\tbool fn_held;\t\t/* magnifier key held => Fn modifier active */\n",
    "struct member fn_held")

# ---- 2. insert Fn defines + remap table + lookup before the CRC helper ----
CRC_ANCHOR = "/* ---- CRC-16/IBM (app_compute_crc16 / app_crc16_get) ---- */"

FN_BLOCK = """\
/*
 * The magnifier key is consumed by the driver and used as the Fn modifier.
 * It arrives on the general-key path as HID scancode 186, which the keymap
 * maps to KEY_F23 (verified by evtest on the real hardware).
 */
#define POGO_FN_HID_SCANCODE\t186

/*
 * Fn + media key -> F1..F12, in physical key order left to right:
 * br-, br+, mic, touchpad-toggle, screenshot, lock,
 * rewind, play/pause, forward, mute, vol-, vol+
 */
static const u16 pogo_fn_remap[][2] = {
\t{0x70,  KEY_F1},\t/* brightness down\t*/
\t{0x6f,  KEY_F2},\t/* brightness up\t*/
\t{0x391, KEY_F3},\t/* microphone/settings\t*/
\t{0x392, KEY_F4},\t/* touchpad on/off\t*/
\t{0x390, KEY_F5},\t/* screen capture\t*/
\t{0x38e, KEY_F6},\t/* lock\t\t\t*/
\t{0xb6,  KEY_F7},\t/* rewind\t\t*/
\t{0xcd,  KEY_F8},\t/* play/pause\t\t*/
\t{0xb5,  KEY_F9},\t/* forward\t\t*/
\t{0xe2,  KEY_F10},\t/* mute\t\t\t*/
\t{0xea,  KEY_F11},\t/* volume down\t\t*/
\t{0xe9,  KEY_F12},\t/* volume up\t\t*/
};

/* resolve a media scancode to a Linux keycode, honouring the Fn modifier.
 * returns 0 if the scancode is unknown. */
static u16 pogo_kbd_mm_lookup(struct pogo_kbd *kb, u16 kw)
{
\tint j;

\tif (kb->fn_held)
\t\tfor (j = 0; j < ARRAY_SIZE(pogo_fn_remap); j++)
\t\t\tif (pogo_fn_remap[j][0] == kw)
\t\t\t\treturn pogo_fn_remap[j][1];

\tfor (j = 0; j < KEYBOARD_MM_NUM_KEYS; j++)
\t\tif (mm_keymap[j][0] == kw)
\t\t\treturn mm_keymap[j][1];

\treturn 0;
}

"""

src = replace_once(src, CRC_ANCHOR, FN_BLOCK + CRC_ANCHOR,
                   "Fn block (defines + table + lookup)")

# ---- 3. intercept the magnifier key in the general-key report -------------
OLD_LOOP = """\
\tfor (i = 2; i < 8; i++) {
\t\tif (kb->old[i] > 3 &&
\t\t    memscan(kb->cur + 2, kb->old[i], 6) == kb->cur + 8) {
\t\t\tif (pogo_keymap[kb->old[i]])
\t\t\t\tinput_report_key(dev, pogo_keymap[kb->old[i]], 0);
\t\t}
\t\tif (kb->cur[i] > 3 &&
\t\t    memscan(kb->old + 2, kb->cur[i], 6) == kb->old + 8) {
\t\t\tif (pogo_keymap[kb->cur[i]])
\t\t\t\tinput_report_key(dev, pogo_keymap[kb->cur[i]], 1);
\t\t}
\t}
"""

NEW_LOOP = """\
\tfor (i = 2; i < 8; i++) {
\t\tif (kb->old[i] > 3 &&
\t\t    memscan(kb->cur + 2, kb->old[i], 6) == kb->cur + 8) {
\t\t\tif (kb->old[i] == POGO_FN_HID_SCANCODE)
\t\t\t\tkb->fn_held = false;\t/* magnifier: Fn modifier */
\t\t\telse if (pogo_keymap[kb->old[i]])
\t\t\t\tinput_report_key(dev, pogo_keymap[kb->old[i]], 0);
\t\t}
\t\tif (kb->cur[i] > 3 &&
\t\t    memscan(kb->old + 2, kb->cur[i], 6) == kb->old + 8) {
\t\t\tif (kb->cur[i] == POGO_FN_HID_SCANCODE)
\t\t\t\tkb->fn_held = true;\t/* magnifier: Fn modifier */
\t\t\telse if (pogo_keymap[kb->cur[i]])
\t\t\t\tinput_report_key(dev, pogo_keymap[kb->cur[i]], 1);
\t\t}
\t}
"""

src = replace_once(src, OLD_LOOP, NEW_LOOP,
                   "pogo_kbd_report_keys Fn interception")

# ---- 4. replace pogo_kbd_report_mm wholesale ------------------------------
MM_START = "/* ---- media-key report (vendor pogo_keyboard_mm_input_report) ---- */"
MM_END = "/* ---- touchpad report (vendor touchpad_input_report) ---- */"

i0 = src.find(MM_START)
i1 = src.find(MM_END)
if i0 < 0 or i1 < 0 or i1 <= i0:
    sys.exit("ERROR: could not locate pogo_kbd_report_mm. File NOT modified.")

NEW_MM = """\
/* ---- media-key report (vendor pogo_keyboard_mm_input_report) ---- */
static void pogo_kbd_report_mm(struct pogo_kbd *kb, const u8 *buf)
{
\tstruct input_dev *dev = kb->kbd;
\tint i;
\tu16 code;

\tmemcpy(kb->mm_new, &buf[1], 4);
\tfor (i = 0; i < 2; i++) {
\t\tu16 kw = kb->mm_old[i];

\t\tif (kw && memscan_ex(kb->mm_new, kw, 4) == (void *)(kb->mm_new + 2)) {
\t\t\tcode = pogo_kbd_mm_lookup(kb, kw);
\t\t\tif (code)
\t\t\t\tinput_report_key(dev, code, 0);
\t\t}

\t\tkw = kb->mm_new[i];
\t\tif (kw && memscan_ex(kb->mm_old, kw, 4) == (void *)(kb->mm_old + 2)) {
\t\t\tcode = pogo_kbd_mm_lookup(kb, kw);
\t\t\tif (code)
\t\t\t\tinput_report_key(dev, code, 1);
\t\t}
\t}

\tinput_sync(dev);
\tmemcpy(kb->mm_old, kb->mm_new, 4);
}

"""

src = src[:i0] + NEW_MM + src[i1:]
print("  ok: pogo_kbd_report_mm replaced")

# ---- 5. declare F1..F12 in the keybit --------------------------------------
src = replace_once(
    src,
    "\tkb->kbd = dev;\n\treturn input_register_device(dev);",
    "\tfor (i = 0; i < ARRAY_SIZE(pogo_fn_remap); i++)\n"
    "\t\t__set_bit(pogo_fn_remap[i][1], dev->keybit);\n\n"
    "\tkb->kbd = dev;\n\treturn input_register_device(dev);",
    "keybit for F1..F12")

# ---- write result ----------------------------------------------------------
shutil.copy2(FN, FN + ".orig")
with open(FN, "w") as f:
    f.write(src)

print(f"\nAll 5 edits applied successfully.")
print(f"Backup written to {FN}.orig")
