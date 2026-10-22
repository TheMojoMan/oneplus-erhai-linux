/* SPDX-License-Identifier: GPL-2.0-only */
// Portions ported to Linux from the OnePlus Android kernel driver
//   vendor/oplus/kernel/charger/wireless_pen/oplus_wireless_pen_glink.c
// and the Android qti_battery_charger.c battery/USB property mapping,
// in
//   github.com/OnePlusOSS/android_kernel_modules_and_devicetree_oneplus_sm8750
//   commit e888579 (OnePlus OPD2415_15.0.1.701(EX01))
// which have license GPL-2.0-only
//   (Copyright (C) 2020-2022 Oplus. All rights reserved. /
//    Copyright (c) 2021-2024 Qualcomm Innovation Center, Inc.)
//
/*
 * erhai_otg_glink.c shared interface: pmic-glink battery_charger driver,
 * exports ADSP property GET/SET helpers so the SC8547 fast-charge driver and
 * the PPS host engine (erhai_pps) can consult and drive the DSP-side charger
 * classification/negotiation (vendor usb_property_id / battery_property_id,
 * e.g. USB_VOOC_FAST_CHG_TYPE = 26, USB_SET_PPS_VOLT = 34) — the Android
 * engine's is_pd_svooc_adapter()/adapter_sid_power() retry policy.
 */
#ifndef _ERHAI_OTG_GLINK_H_
#define _ERHAI_OTG_GLINK_H_

#include <linux/errno.h>
#include <linux/types.h>

#if IS_ENABLED(CONFIG_ERHAI_OTG_GLINK)
int erhai_otg_adsp_usb_prop(u32 property);
/*
 * Read/write one ADSP battery_charger property over pmic_glink.  opcode is
 * the battery_charger target:
 *	BC_USB_PROPERTY_GET(0x32) / BC_USB_PROPERTY_SET(0x33) - USB psy
 *	BC_BATTERY_PROPERTY_GET(0x30) / BC_BATTERY_PROPERTY_SET(0x31) - battery
 * GET returns the property value (>= 0) or a negative errno; SET returns 0 or
 * a negative errno (-ENODEV glink session down, -EINVAL unknown opcode,
 * -EINVAL property rejected by the ADSP in its ret_code).
 */
int erhai_otg_adsp_prop_get(u32 opcode, u32 property);
int erhai_otg_adsp_prop_set(u32 opcode, u32 property, u32 value);
bool erhai_otg_charge_inhibited(void);
#else
static inline int erhai_otg_adsp_usb_prop(u32 property)
{
	return -ENODEV;
}
static inline int erhai_otg_adsp_prop_get(u32 opcode, u32 property)
{
	return -ENODEV;
}
static inline bool erhai_otg_charge_inhibited(void)
{
	return false;
}
static inline int erhai_otg_adsp_prop_set(u32 opcode, u32 property, u32 value)
{
	return -ENODEV;
}
#endif

#endif /* _ERHAI_OTG_GLINK_H_ */
