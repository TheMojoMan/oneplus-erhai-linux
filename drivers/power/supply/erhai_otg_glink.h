/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * erhai_otg_glink.c shared interface: pmic-glink battery_charger driver,
 * exports an ADSP USB-property GET so the SC8547 fast-charge driver can
 * consult the DSP-side charger classification (vendor usb_property_id,
 * e.g. USB_VOOC_FAST_CHG_TYPE = 26) as a second signal — the Android
 * engine's is_pd_svooc_adapter()/adapter_sid_power() retry policy.
 */
#ifndef _ERHAI_OTG_GLINK_H_
#define _ERHAI_OTG_GLINK_H_

#include <linux/errno.h>
#include <linux/types.h>

#if IS_ENABLED(CONFIG_ERHAI_OTG_GLINK)
int erhai_otg_adsp_usb_prop(u32 property);
#else
static inline int erhai_otg_adsp_usb_prop(u32 property)
{
	return -ENODEV;
}
#endif

#endif /* _ERHAI_OTG_GLINK_H_ */