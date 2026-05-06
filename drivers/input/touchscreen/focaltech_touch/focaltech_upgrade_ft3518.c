// SPDX-License-Identifier: GPL-2.0-only
/*
 * FocalTech FT3518/FT5452-compatible flash upgrade
 *
 * Copyright (c) 2012-2019, Focaltech Ltd.
 */

#include "focaltech_flash.h"

static int fts_ft3518_upgrade(struct fts_upgrade *upg, u8 *buf, u32 len)
{
	struct fts_ts_data *ts = upg->ts_data;
	u8 cmd[4];
	int ecc_host, ecc_tp;
	int ret;

	if (!buf || len < FTS_MIN_LEN || len > FTS_MAX_LEN_FILE) {
		dev_err(ts->dev, "fw buffer invalid, len=%u\n", len);
		return -EINVAL;
	}

	ret = fts_fwupg_enter_into_boot(upg);
	if (ret < 0) {
		dev_err(ts->dev, "enter boot fail\n");
		goto fw_reset;
	}

	cmd[0] = 0x09; /* FTS_CMD_FLASH_MODE */
	cmd[1] = 0x0B; /* FLASH_MODE_UPGRADE_VALUE */
	ret = fts_write(ts, cmd, 2);
	if (ret < 0) {
		dev_err(ts->dev, "upgrade mode cmd fail\n");
		goto fw_reset;
	}

	cmd[0] = 0xB0; /* FTS_CMD_DATA_LEN */
	cmd[1] = BYTE_OFF_16(len);
	cmd[2] = BYTE_OFF_8(len);
	cmd[3] = BYTE_OFF_0(len);
	ret = fts_write(ts, cmd, 4);
	if (ret < 0) {
		dev_err(ts->dev, "data len cmd fail\n");
		goto fw_reset;
	}

	ret = fts_fwupg_erase(ts, FTS_REASE_APP_DELAY);
	if (ret < 0) {
		dev_err(ts->dev, "erase fail\n");
		goto fw_reset;
	}

	ecc_host = fts_flash_write_buf(upg, upg->func->appoff, buf, len, 1);
	if (ecc_host < 0) {
		dev_err(ts->dev, "write flash fail\n");
		goto fw_reset;
	}

	ecc_tp = fts_fwupg_ecc_cal(upg, upg->func->appoff, len);
	if (ecc_tp < 0) {
		dev_err(ts->dev, "ecc cal fail\n");
		goto fw_reset;
	}

	dev_info(ts->dev, "ecc host:%02x tp:%02x\n", ecc_host, ecc_tp);
	if (ecc_host != ecc_tp) {
		dev_err(ts->dev, "ecc check fail\n");
		goto fw_reset;
	}

	dev_info(ts->dev, "upgrade success, resetting\n");
	fts_fwupg_reset_in_boot(ts);
	msleep(200);
	return 0;

fw_reset:
	fts_fwupg_reset_in_boot(ts);
	return -EIO;
}

struct upgrade_func upgrade_func_ft5452 = {
	.ctype = { 0x81 },
	.fwveroff = 0x010E,
	.fwcfgoff = 0x1FFB0,
	.appoff = 0x0000,
	.pramboot_supported = false,
	.hid_supported = true,
	.upgrade = fts_ft3518_upgrade,
};
