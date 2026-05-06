/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * FocalTech FT3518 flash upgrade
 *
 * Copyright (c) 2012-2019, FocalTech Systems, Ltd.
 */

#ifndef __FOCALTECH_FLASH_H__
#define __FOCALTECH_FLASH_H__

#include "focaltech_core.h"

#define FTS_CMD_RESET				0x07
#define FTS_CMD_FLASH_TYPE			0x05
#define FTS_CMD_FLASH_MODE			0x09
#define FLASH_MODE_WRITE_FLASH_VALUE		0x0A
#define FLASH_MODE_UPGRADE_VALUE		0x0B

#define FTS_CMD_ERASE_APP			0x61
#define FTS_REASE_APP_DELAY			1350
#define FTS_RETRIES_REASE			50
#define FTS_RETRIES_DELAY_REASE		200

#define FTS_CMD_FLASH_STATUS			0x6A
#define FTS_CMD_FLASH_STATUS_LEN		2
#define FTS_CMD_FLASH_STATUS_ERASE_OK		0xF0AA
#define FTS_CMD_FLASH_STATUS_ECC_OK		0xF055
#define FTS_CMD_FLASH_STATUS_WRITE_OK		0x1000

#define FTS_CMD_ECC_INIT			0x64
#define FTS_CMD_ECC_CAL				0x65
#define FTS_CMD_ECC_CAL_LEN			6
#define FTS_CMD_ECC_READ			0x66
#define FTS_RETRIES_ECC_CAL			10
#define FTS_RETRIES_DELAY_ECC_CAL		50

#define FTS_CMD_DATA_LEN			0xB0
#define FTS_CMD_DATA_LEN_LEN			4
#define FTS_CMD_WRITE				0xBF
#define FTS_CMD_WRITE_LEN			6
#define FTS_RETRIES_WRITE			100
#define FTS_RETRIES_DELAY_WRITE		1

#define FTS_DELAY_READ_ID			20
#define FTS_DELAY_UPGRADE_RESET		80
#define FTS_FLASH_PACKET_LENGTH			32
#define FTS_MAX_LEN_ECC_CALC			0xFFFE
#define FTS_MIN_LEN				0x120
#define FTS_MAX_LEN_FILE			(128 * 1024)

#define FTS_REG_UPGRADE				0xFC
#define FTS_UPGRADE_AA				0xAA
#define FTS_UPGRADE_55				0x55
#define FTS_DELAY_UPGRADE_AA			10
#define FTS_UPGRADE_LOOP			30

#define BYTE_OFF_0(x)	((u8)((x) & 0xFF))
#define BYTE_OFF_8(x)	((u8)(((x) >> 8) & 0xFF))
#define BYTE_OFF_16(x)	((u8)(((x) >> 16) & 0xFF))

#define FTX_MAX_COMPATIBLE_TYPE			4

enum FW_STATUS {
	FTS_RUN_IN_ERROR,
	FTS_RUN_IN_APP,
	FTS_RUN_IN_ROM,
	FTS_RUN_IN_PRAM,
	FTS_RUN_IN_BOOTLOADER,
};

struct fts_upgrade;

struct upgrade_func {
	u64 ctype[FTX_MAX_COMPATIBLE_TYPE];
	u32 fwveroff;
	u32 fwcfgoff;
	u32 appoff;
	bool hid_supported;
	bool pramboot_supported;
	int (*upgrade)(struct fts_upgrade *upg, u8 *buf, u32 len);
};

struct fts_upgrade {
	struct fts_ts_data *ts_data;
	struct upgrade_func *func;
	u8 *fw;
	u32 fw_length;
	bool fw_from_request;
};

int fts_fwupg_reset_in_boot(struct fts_ts_data *ts);
int fts_fwupg_enter_into_boot(struct fts_upgrade *upg);
int fts_fwupg_erase(struct fts_ts_data *ts, u32 delay);
int fts_fwupg_ecc_cal(struct fts_upgrade *upg, u32 saddr, u32 len);
int fts_flash_write_buf(struct fts_upgrade *upg, u32 saddr, u8 *buf,
			u32 len, u32 delay);

/* FT3518 upgrade function */
extern struct upgrade_func upgrade_func_ft5452;

#endif /* __FOCALTECH_FLASH_H__ */
