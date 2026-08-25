/*
 * Copyright (C) 2010 - 2018 Novatek, Inc.
 * Copyright (C) 2021 XiaoMi, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * erhai (OnePlus Pad 3): NT36536 single + cascade memory maps + trim table
 * transplanted verbatim from the OnePlus SM8750 OSS NT36536_noflash vendor
 * driver onto the Xiaomi nabu NT36xxx SPI base. Our XN242 dual-DSI panel uses
 * the CASCADE map (FW header byte[0x20]&0x02 set / ENB_CASC low).
 */

/* NT36536 hw register addresses (per OnePlus nvt_drivers_nt36536_noflash.c) */
#define CHIP_VER_TRIM_ADDR     0x1FB104
#define CHIP_VER_TRIM_OLD_ADDR 0x1F64E

/* hw_crc length scheme */
#define HWCRC_NOSUPPORT		0x00
#define HWCRC_LEN_2Bytes	0x01
#define HWCRC_LEN_3Bytes	0x02

/* auto-copy (cascade SRAM mirror) completion-check mode */
#define CHECK_SPI_DMA_TX_INFO	0x01
#define CHECK_TX_AUTO_COPY_EN	0x02

/* bootloader multi-header support */
#define BLDMHEADER_NOSUPPORT	0x00
#define BLDMHEADER_SUPPORT	0x01

typedef struct nvt_ts_reg {
	uint32_t addr;	/* byte in which address */
	uint8_t mask;	/* in which bits of that byte */
} nvt_ts_reg_t;

struct nvt_ts_mem_map {
	uint32_t EVENT_BUF_ADDR;
	uint32_t RAW_PIPE0_ADDR;
	uint32_t RAW_PIPE1_ADDR;
	uint32_t BASELINE_ADDR;
	uint32_t BASELINE_BTN_ADDR;
	uint32_t DIFF_PIPE0_ADDR;
	uint32_t DIFF_PIPE1_ADDR;
	uint32_t RAW_BTN_PIPE0_ADDR;
	uint32_t RAW_BTN_PIPE1_ADDR;
	uint32_t DIFF_BTN_PIPE0_ADDR;
	uint32_t DIFF_BTN_PIPE1_ADDR;
	uint32_t PEN_2D_BL_TIP_X_ADDR;
	uint32_t PEN_2D_BL_TIP_Y_ADDR;
	uint32_t PEN_2D_BL_RING_X_ADDR;
	uint32_t PEN_2D_BL_RING_Y_ADDR;
	uint32_t PEN_2D_DIFF_TIP_X_ADDR;
	uint32_t PEN_2D_DIFF_TIP_Y_ADDR;
	uint32_t PEN_2D_DIFF_RING_X_ADDR;
	uint32_t PEN_2D_DIFF_RING_Y_ADDR;
	uint32_t PEN_2D_RAW_TIP_X_ADDR;
	uint32_t PEN_2D_RAW_TIP_Y_ADDR;
	uint32_t PEN_2D_RAW_RING_X_ADDR;
	uint32_t PEN_2D_RAW_RING_Y_ADDR;
	uint32_t PEN_1D_DIFF_TIP_X_ADDR;
	uint32_t PEN_1D_DIFF_TIP_Y_ADDR;
	uint32_t PEN_1D_DIFF_RING_X_ADDR;
	uint32_t PEN_1D_DIFF_RING_Y_ADDR;
	uint32_t READ_FLASH_CHECKSUM_ADDR;
	uint32_t RW_FLASH_DATA_ADDR;
	/* single vs cascade selector */
	nvt_ts_reg_t ENB_CASC_REG;
	/* Phase 2 Host Download */
	uint32_t BOOT_RDY_ADDR;
	uint32_t POR_CD_ADDR;
	uint32_t TX_AUTO_COPY_EN;
	uint32_t SPI_DMA_TX_INFO;
	/* BLD CRC */
	uint32_t BLD_LENGTH_ADDR;
	uint32_t ILM_LENGTH_ADDR;
	uint32_t DLM_LENGTH_ADDR;
	uint32_t BLD_DES_ADDR;
	uint32_t ILM_DES_ADDR;
	uint32_t DLM_DES_ADDR;
	uint32_t G_ILM_CHECKSUM_ADDR;
	uint32_t G_DLM_CHECKSUM_ADDR;
	uint32_t R_ILM_CHECKSUM_ADDR;
	uint32_t R_DLM_CHECKSUM_ADDR;
	uint32_t BLD_CRC_EN_ADDR;
	uint32_t DMA_CRC_EN_ADDR;
	uint32_t BLD_ILM_DLM_CRC_ADDR;
	uint32_t DMA_CRC_FLAG_ADDR;
	uint32_t FW_HISTORY_ADDR;
};

struct nvt_ts_hw_info {
	uint8_t carrier_system;
	uint8_t hw_crc;
	uint8_t auto_copy;
	uint8_t bld_multi_header;
	uint32_t chip_ver_trim_addr;
	uint32_t swrst_sif_addr;
	uint32_t bld_spe_pups_addr;
};

static const struct nvt_ts_mem_map NT36536_cascade_memory_map = {
	.EVENT_BUF_ADDR           = 0x130900,
	.RAW_PIPE0_ADDR           = 0x10BAA0,
	.RAW_PIPE1_ADDR           = 0x10BAA0,
	.BASELINE_ADDR            = 0x109FA0,
	.BASELINE_BTN_ADDR        = 0,
	.DIFF_PIPE0_ADDR          = 0x127E70,
	.DIFF_PIPE1_ADDR          = 0x129970,
	.RAW_BTN_PIPE0_ADDR       = 0,
	.RAW_BTN_PIPE1_ADDR       = 0,
	.DIFF_BTN_PIPE0_ADDR      = 0,
	.DIFF_BTN_PIPE1_ADDR      = 0,
	.PEN_2D_BL_TIP_X_ADDR     = 0x110BA0,
	.PEN_2D_BL_TIP_Y_ADDR     = 0x111020,
	.PEN_2D_BL_RING_X_ADDR    = 0x1114A0,
	.PEN_2D_BL_RING_Y_ADDR    = 0x111920,
	.PEN_2D_DIFF_TIP_X_ADDR   = 0x1126A0,
	.PEN_2D_DIFF_TIP_Y_ADDR   = 0x112B20,
	.PEN_2D_DIFF_RING_X_ADDR  = 0x112FA0,
	.PEN_2D_DIFF_RING_Y_ADDR  = 0x113420,
	.PEN_2D_RAW_TIP_X_ADDR    = 0x130B00,
	.PEN_2D_RAW_TIP_Y_ADDR    = 0x130F90,
	.PEN_2D_RAW_RING_X_ADDR   = 0x131420,
	.PEN_2D_RAW_RING_Y_ADDR   = 0x1318B0,
	.PEN_1D_DIFF_TIP_X_ADDR   = 0x131D40,
	.PEN_1D_DIFF_TIP_Y_ADDR   = 0x131DD0,
	.PEN_1D_DIFF_RING_X_ADDR  = 0x131E60,
	.PEN_1D_DIFF_RING_Y_ADDR  = 0x131EF0,
	.ENB_CASC_REG             = {.addr = 0x1FB12C, .mask = 0x01},
	/* Phase 2 Host Download */
	.BOOT_RDY_ADDR            = 0x1FB50D,
	.TX_AUTO_COPY_EN          = 0x1FC825,
	.SPI_DMA_TX_INFO          = 0x1FC814,
	/* BLD CRC */
	.BLD_LENGTH_ADDR          = 0x1FB518,	/* 0x1FB518 ~ 0x1FB51A (3 bytes) */
	.ILM_LENGTH_ADDR          = 0x1FCC34,	/* 0x1FCC34 ~ 0x1FCC36 (3 bytes) */
	.DLM_LENGTH_ADDR          = 0x1FCC44,	/* 0x1FCC44 ~ 0x1FCC46 (3 bytes) */
	.BLD_DES_ADDR             = 0x1FB514,	/* 0x1FB514 ~ 0x1FB516 (3 bytes) */
	.ILM_DES_ADDR             = 0x1FCC30,	/* 0x1FCC30 ~ 0x1FCC32 (3 bytes) */
	.DLM_DES_ADDR             = 0x1FCC40,	/* 0x1FCC40 ~ 0x1FCC42 (3 bytes) */
	.G_ILM_CHECKSUM_ADDR      = 0x1FCC38,	/* 0x1FCC38 ~ 0x1FCC3B (4 bytes) */
	.G_DLM_CHECKSUM_ADDR      = 0x1FCC48,	/* 0x1FCC48 ~ 0x1FCC4B (4 bytes) */
	.R_ILM_CHECKSUM_ADDR      = 0x1FCC7C,	/* 0x1FCC7C ~ 0x1FCC7F (4 bytes) */
	.R_DLM_CHECKSUM_ADDR      = 0x1FCC80,	/* 0x1FCC80 ~ 0x1FCC83 (4 bytes) */
	.DMA_CRC_EN_ADDR          = 0x1FB51B,
	.BLD_ILM_DLM_CRC_ADDR     = 0x1FB532,
	.DMA_CRC_FLAG_ADDR        = 0x1FB534,
};

static const struct nvt_ts_mem_map NT36536_single_memory_map = {
	.EVENT_BUF_ADDR           = 0x130900,
	.RAW_PIPE0_ADDR           = 0x10D748,
	.RAW_PIPE1_ADDR           = 0x10D748,
	.BASELINE_ADDR            = 0x10C348,
	.BASELINE_BTN_ADDR        = 0,
	.DIFF_PIPE0_ADDR          = 0x11DB50,
	.DIFF_PIPE1_ADDR          = 0x11EF50,
	.RAW_BTN_PIPE0_ADDR       = 0,
	.RAW_BTN_PIPE1_ADDR       = 0,
	.DIFF_BTN_PIPE0_ADDR      = 0,
	.DIFF_BTN_PIPE1_ADDR      = 0,
	.PEN_2D_BL_TIP_X_ADDR     = 0x121750,
	.PEN_2D_BL_TIP_Y_ADDR     = 0x121990,
	.PEN_2D_BL_RING_X_ADDR    = 0x121E10,
	.PEN_2D_BL_RING_Y_ADDR    = 0x122050,
	.PEN_2D_DIFF_TIP_X_ADDR   = 0x110B14,
	.PEN_2D_DIFF_TIP_Y_ADDR   = 0x110D54,
	.PEN_2D_DIFF_RING_X_ADDR  = 0x1111D4,
	.PEN_2D_DIFF_RING_Y_ADDR  = 0x111414,
	.PEN_2D_RAW_TIP_X_ADDR    = 0x111AD4,
	.PEN_2D_RAW_TIP_Y_ADDR    = 0x111D14,
	.PEN_2D_RAW_RING_X_ADDR   = 0x112194,
	.PEN_2D_RAW_RING_Y_ADDR   = 0x1123D4,
	.PEN_1D_DIFF_TIP_X_ADDR   = 0x130B00,
	.PEN_1D_DIFF_TIP_Y_ADDR   = 0x130B90,
	.PEN_1D_DIFF_RING_X_ADDR  = 0x130C20,
	.PEN_1D_DIFF_RING_Y_ADDR  = 0x130CB0,
	.ENB_CASC_REG             = {.addr = 0x1FB12C, .mask = 0x01},
	/* Phase 2 Host Download */
	.BOOT_RDY_ADDR            = 0x1FB50D,
	.TX_AUTO_COPY_EN          = 0x1FC825,
	.SPI_DMA_TX_INFO          = 0x1FC814,
	/* BLD CRC */
	.BLD_LENGTH_ADDR          = 0x1FB518,	/* 0x1FB518 ~ 0x1FB51A (3 bytes) */
	.ILM_LENGTH_ADDR          = 0x1FCC34,	/* 0x1FCC34 ~ 0x1FCC36 (3 bytes) */
	.DLM_LENGTH_ADDR          = 0x1FCC44,	/* 0x1FCC44 ~ 0x1FCC46 (3 bytes) */
	.BLD_DES_ADDR             = 0x1FB514,	/* 0x1FB514 ~ 0x1FB516 (3 bytes) */
	.ILM_DES_ADDR             = 0x1FCC30,	/* 0x1FCC30 ~ 0x1FCC32 (3 bytes) */
	.DLM_DES_ADDR             = 0x1FCC40,	/* 0x1FCC40 ~ 0x1FCC42 (3 bytes) */
	.G_ILM_CHECKSUM_ADDR      = 0x1FCC38,	/* 0x1FCC38 ~ 0x1FCC3B (4 bytes) */
	.G_DLM_CHECKSUM_ADDR      = 0x1FCC48,	/* 0x1FCC48 ~ 0x1FCC4B (4 bytes) */
	.R_ILM_CHECKSUM_ADDR      = 0x1FCC7C,	/* 0x1FCC7C ~ 0x1FCC7F (4 bytes) */
	.R_DLM_CHECKSUM_ADDR      = 0x1FCC80,	/* 0x1FCC80 ~ 0x1FCC83 (4 bytes) */
	.DMA_CRC_EN_ADDR          = 0x1FB51B,
	.BLD_ILM_DLM_CRC_ADDR     = 0x1FB532,
	.DMA_CRC_FLAG_ADDR        = 0x1FB534,
};

static const struct nvt_ts_hw_info NT36536_hw_info = {
	.carrier_system     = 2,
	.hw_crc             = HWCRC_LEN_3Bytes,
	.auto_copy          = CHECK_TX_AUTO_COPY_EN,
	.bld_multi_header   = BLDMHEADER_SUPPORT,
	.chip_ver_trim_addr = 0x1FB104,
	.swrst_sif_addr     = 0x1FB43E,
	.bld_spe_pups_addr  = 0x1FB535,
};

#define NVT_ID_BYTE_MAX 6
struct nvt_ts_trim_id_table {
	uint8_t id[NVT_ID_BYTE_MAX];
	uint8_t mask[NVT_ID_BYTE_MAX];
	const struct nvt_ts_mem_map *mmap;
	const struct nvt_ts_mem_map *mmap_casc;
	const struct nvt_ts_hw_info *hwinfo;
};

static const struct nvt_ts_trim_id_table trim_id_table[] = {
	{.id = {0x14, 0xFF, 0xFF, 0x23, 0x65, 0x03}, .mask = {1, 0, 0, 1, 1, 1},
		.mmap = &NT36536_single_memory_map, .mmap_casc = &NT36536_cascade_memory_map,
		.hwinfo = &NT36536_hw_info},
	{.id = {0x15, 0x0E, 0xFF, 0x23, 0x65, 0x03}, .mask = {1, 1, 0, 1, 1, 1},
		.mmap = &NT36536_single_memory_map, .mmap_casc = &NT36536_cascade_memory_map,
		.hwinfo = &NT36536_hw_info},
};
