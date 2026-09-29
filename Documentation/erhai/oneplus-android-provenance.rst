.. SPDX-License-Identifier: GPL-2.0-only

=====================================================
Provenance of code ported/imported from OnePlus Android
=====================================================

Parts of this tree were ported or imported into Linux from the OnePlus
OxygenOS (Android) open-source releases.  For each part the source, the
license of that source and the commits that introduced it are listed here;
the ported driver files carry the same statement in their own header and the
commits also carry it as a git note (git log --notes).

Sources
=======

* github.com/OnePlusOSS/android_kernel_oneplus_sm8750 - OnePlus Pad 3
  (erhai, SM8750) Android kernel, branch oneplus/sm8750_v_15.0.1_pad_3;
  ships COPYING = GPL-2.0 WITH Linux-syscall-note (ACK-based tree).
* github.com/OnePlusOSS/android_kernel_modules_and_devicetree_oneplus_sm8750
  commit e888579 (OnePlus OPD2415_15.0.1.701(EX01)).  No top-level LICENSE; per-file headers apply (OPLUS
  modules GPL-2.0-only, WLAN stack ISC, Qualcomm modules/devicetree
  GPL-2.0-only / BSD-3-Clause).

OPLUS wireless-pen charger (CPS8601) - ported
=============================================

* source: vendor/oplus/kernel/charger/wireless_pen/oplus_cps8601.{c,h}, oplus_cps8601_bl.h and the MCU app image in it; github.com/OnePlusOSS/android_kernel_modules_and_devicetree_oneplus_sm8750 commit e888579 (OnePlus OPD2415_15.0.1.701(EX01))
* license: GPL-2.0-only (Copyright (C) 2020-2022 Oplus. All rights reserved.)
* files: ``drivers/power/supply/oplus_cps8601.c`` ``drivers/power/supply/oplus_cps8601_bl.h`` ``drivers/power/supply/cps8601_fw.h``
* commits: 377cb02f55 31f4e6b5e3 5bb2f0d8a6 290cd75512 7df5488eae 5e996bdd70 4a513f2c48 180e2df045

OPLUS voocphy charge pump (SC8547A) - ported
============================================

* source: vendor/oplus/kernel/charger/v2/voocphy/phy/oplus_sc8547a.{c,h}; github.com/OnePlusOSS/android_kernel_modules_and_devicetree_oneplus_sm8750 commit e888579 (OnePlus OPD2415_15.0.1.701(EX01))
* license: GPL-2.0-only (Copyright (C) 2020-2022 Oplus. All rights reserved.)
* files: ``drivers/power/supply/sc8547a.c`` ``drivers/power/supply/sc8547a.h``
* commits: 082c64968c 069d4daf96 dfb1c73eb4 de148aee1b 99bd5a5058 59160d6149 7100c4ee0c 025fbe9305

OPLUS pmic_glink battery/VOOC protocol (otg glink + PPS engine) - ported
========================================================================

* source: vendor/oplus/kernel/charger/wireless_pen/oplus_wireless_pen_glink.c + Android qti_battery_charger.c property/opcode mapping; github.com/OnePlusOSS/android_kernel_modules_and_devicetree_oneplus_sm8750 commit e888579 (OnePlus OPD2415_15.0.1.701(EX01))
* license: GPL-2.0-only (Oplus 2020-2022 / Qualcomm Innovation Center, Inc.)
* files: ``drivers/power/supply/erhai_otg_glink.c`` ``drivers/power/supply/erhai_otg_glink.h`` ``drivers/power/supply/erhai_pps.c``
* commits: 082c64968c a57a14905b 5af2a3befc 3f30f98815 babf229536 8aa1f5570a 61353d9c4b 8cc6a8bb26 389b1c46c3 4ccedd6fd4 21c3106961 67177d7051 dfb1c73eb4 de148aee1b c7e2e40c90 99bd5a5058 3c8ec8324d 59160d6149 8d68035b98

OnePlus pogo keyboard - ported
==============================

* source: OnePlus Android pogo-keyboard driver + kernel_platform/qcom/opensource/devicetree/oplus/oplus_pogo_kb/erhai-oplus-pogopin-kb-*.dtsi; github.com/OnePlusOSS/android_kernel_modules_and_devicetree_oneplus_sm8750 commit e888579 (OnePlus OPD2415_15.0.1.701(EX01))
* license: GPL-2.0-only (release COPYING = GPL-2.0 WITH Linux-syscall-note)
* files: ``drivers/input/keyboard/oneplus-pogo-kbd.c``
* commits: 8717921392 9a06272f9e 00d4242345 b987035d77 e20cc44df3 ec60f007d5 c9ec4696d7 6c52c5d865 740318f09c 2090e150db

SmartSens SC1320CS / SC820CS sensors - sequences ported
=======================================================

* source: vendor/qcom/opensource/camera-kernel + erhai camera devicetree; github.com/OnePlusOSS/android_kernel_modules_and_devicetree_oneplus_sm8750 commit e888579 (OnePlus OPD2415_15.0.1.701(EX01))
* license: GPL-2.0-only (Qualcomm Innovation Center, Inc.); vendor sensor lib not published
* files: ``drivers/media/i2c/sc1320cs.c`` ``drivers/media/i2c/sc820cs.c``
* commits: 5e297ec3da 82da183920

Erhai device tree - values ported
=================================

* source: arch/arm64/boot/dts/qcom/* (github.com/OnePlusOSS/android_kernel_oneplus_sm8750, branch oneplus/sm8750_v_15.0.1_pad_3) + OPLUS overlays (kernel_platform/qcom/opensource/devicetree/oplus/**) in github.com/OnePlusOSS/android_kernel_modules_and_devicetree_oneplus_sm8750 commit e888579 (OnePlus OPD2415_15.0.1.701(EX01))
* license: BSD-3-Clause (Qualcomm DT) / GPL-2.0-only (OPLUS overlays)
* files: ``arch/arm64/boot/dts/qcom/erhai.dts`` ``arch/arm64/boot/dts/qcom/sm8750.dtsi`` ``arch/arm64/boot/dts/qcom/pm8550.dtsi``
* commits: 31233c9898 b255feaff0 fc242a5abd 828a91fde6 98013fb160 3a8ae84bee 62686c3f36 74d477b8b4 6d025bbee5 98c3536c0f 3285cd5934 82da183920 9c9533de9a dc8b5ffa69 54be614582 1f2c418018 0736fcf7ce fcb8b81c53 46be81efd4 c646d876cd 2ae8637478 862681a860 bffee396f2 9ae09588c5 99690e045b d9bfd9d39a 9fba6a808d 97d89dc215 1c196c9d7e 04247c3119 5c83feb1b6 377cb02f55 5e996bdd70

WLAN stack (qcacld-3.0 / fw-api / qca-wifi-host-cmn) - imported
===============================================================

* source: vendor/qcom/opensource/wlan/{qcacld-3.0,fw-api,qca-wifi-host-cmn}; github.com/OnePlusOSS/android_kernel_modules_and_devicetree_oneplus_sm8750 commit e888579 (OnePlus OPD2415_15.0.1.701(EX01))
* license: ISC (Copyright (c) Linux Foundation / Qualcomm Innovation Center, Inc.)
* files: ``drivers/staging/qcacld-3.0`` ``drivers/staging/fw-api`` ``drivers/staging/qca-wifi-host-cmn``
* commits: 5316e40389 e6f91a10b4

CNSS / WCN7880 host stack - imported
====================================

* source: drivers/net/wireless/cnss2, cnss_utils, cnss_prealloc, cnss_genl + the WCN core patch set; github.com/OnePlusOSS/android_kernel_oneplus_sm8750 / github.com/OnePlusOSS/android_kernel_modules_and_devicetree_oneplus_sm8750 commit e888579 (OnePlus OPD2415_15.0.1.701(EX01))
* license: GPL-2.0-only (Qualcomm Innovation Center, Inc.)
* files: ``drivers/net/wireless/cnss2`` ``drivers/net/wireless/cnss_utils`` ``drivers/net/wireless/cnss_prealloc`` ``drivers/net/wireless/cnss_genl``
* commits: adb454b75c d0bf4949d9 b7b555b5d8 53ba8ccd4e 5b343748be 755e144c6e

Qualcomm Android audio-kernel parity (wcd939x / tx-macro / swr / va-macro)
==========================================================================

* source: vendor/qcom/opensource/audio-kernel/** + erhai audio devicetree overlays; github.com/OnePlusOSS/android_kernel_modules_and_devicetree_oneplus_sm8750 commit e888579 (OnePlus OPD2415_15.0.1.701(EX01))
* license: GPL-2.0-only (Qualcomm Innovation Center, Inc.)
* files: ``sound/soc/codecs/wcd939x.c`` ``sound/soc/codecs/wcd939x.h`` ``sound/soc/codecs/lpass-tx-macro.c`` ``sound/soc/codecs/lpass-va-macro.c`` ``sound/soc/qcom/swr.c`` ``sound/soc/qcom/sdw.c`` ``sound/soc/qcom/sc8280xp.c``
* commits: bb4d3986ab beac77a240 ae70a5684e 9c9533de9a bd55e64688 2a530ed040 411b904a52 3fc9aa7cf4 54be614582 c646d876cd 2ae8637478 7edda41eb3 49f893da78 ddb69fe267 902a8b5fd7 d6e26c8b25 615004c76b 5fc213fe52 018f237b09 1718389966 04919ced0c 986b0e4bea a361cc23d4 cb60f2f394 16c2e309cc f590c59813 1e32f18c3e e10448936a 2d75451953

Qualcomm Android display-drivers parity (dual-DSI/DSC/DP)
=========================================================

* source: vendor/qcom/opensource/display-drivers/msm/**; github.com/OnePlusOSS/android_kernel_modules_and_devicetree_oneplus_sm8750 commit e888579 (OnePlus OPD2415_15.0.1.701(EX01))
* license: GPL-2.0-only (Qualcomm Innovation Center, Inc.)
* files: ``drivers/gpu/drm/msm/dpu`` ``drivers/gpu/drm/msm/dp`` ``drivers/gpu/drm/msm/msm_atomic.c`` ``drivers/gpu/drm/msm/msm_drv.c``
* commits: b8a0e020e6 cfdb4d96d2 ebe8982bf8 b7b555b5d8

Not OnePlus-derived (attributed elsewhere)
=========================================

* drivers/input/touchscreen/nt36523/* - Novatek NT36xxx driver
  (Copyright (C) 2010-2018 Novatek, Inc. / 2021 XiaoMi, Inc., GPL-2.0).
* drivers/media/i2c/cn3927.c - CN3927 VCM, ported from CFM880/nabu-camera
  (Rockchip Copyright (C) 2022, GPL-2.0-only).
* drivers/gpu/drm/panel/panel-oneplus-xn242.c and
  drivers/video/backlight/ktz8868.c - written for erhai, sole Copyright (c)
  2026 TheMojoMan (owner decision 2026-07-22: no vendor attribution).

