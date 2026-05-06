// SPDX-License-Identifier: GPL-2.0-only
/*
 * FocalTech FT3518 flash upgrade helpers
 *
 * Copyright (c) 2012-2019, FocalTech Ltd.
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/vmalloc.h>
#include "focaltech_core.h"
#include "focaltech_flash.h"

static struct fts_upgrade fwupgrade_storage;
static struct fts_upgrade *fwupgrade;

/* Check flash status register matches expected value */
static bool fts_fwupg_check_flash_status(struct fts_ts_data *ts,
					 u16 flash_status, int retries,
					 int retries_delay)
{
	u8 cmd = FTS_CMD_FLASH_STATUS;
	u8 val[FTS_CMD_FLASH_STATUS_LEN];
	u16 read_status;
	int ret;
	int i;

	for (i = 0; i < retries; i++) {
		ret = fts_read(ts, &cmd, 1, val, FTS_CMD_FLASH_STATUS_LEN);
		if (ret >= 0) {
			read_status = ((u16)val[0] << 8) + val[1];
			if (flash_status == read_status)
				return true;
		}
		msleep(retries_delay);
	}
	return false;
}

static int fts_fwupg_get_boot_state(struct fts_upgrade *upg,
				    enum FW_STATUS *fw_sts)
{
	struct fts_ts_data *ts = upg->ts_data;
	u8 cmd[4] = { 0x55, 0xAA, 0, 0 };
	u8 val[2] = { 0 };
	int ret;

	if (upg->func->hid_supported) {
		u8 hidcmd[] = { 0xEB, 0xAA, 0x09 };

		ret = fts_write(ts, hidcmd, 3);
		if (ret < 0)
			return ret;
		msleep(10);
	}

	ret = fts_write(ts, cmd, 2);
	if (ret < 0)
		return ret;

	msleep(12);
	cmd[0] = 0x90;
	ret = fts_read(ts, cmd, 4, val, 2);
	if (ret < 0)
		return ret;

	if (val[0] == ts->chip_id.rom_idh && val[1] == ts->chip_id.rom_idl)
		*fw_sts = FTS_RUN_IN_ROM;
	else if (val[0] == ts->chip_id.pb_idh && val[1] == ts->chip_id.pb_idl)
		*fw_sts = FTS_RUN_IN_PRAM;
	else if (val[0] == ts->chip_id.bl_idh && val[1] == ts->chip_id.bl_idl)
		*fw_sts = FTS_RUN_IN_BOOTLOADER;

	return 0;
}

static int fts_fwupg_reset_to_boot(struct fts_upgrade *upg)
{
	struct fts_ts_data *ts = upg->ts_data;
	int ret;

	ret = fts_write_reg(ts, FTS_REG_UPGRADE, FTS_UPGRADE_AA);
	if (ret < 0)
		return ret;
	msleep(FTS_DELAY_UPGRADE_AA);

	ret = fts_write_reg(ts, FTS_REG_UPGRADE, FTS_UPGRADE_55);
	if (ret < 0)
		return ret;
	msleep(FTS_DELAY_UPGRADE_RESET);

	return 0;
}

int fts_fwupg_reset_in_boot(struct fts_ts_data *ts)
{
	u8 cmd = FTS_CMD_RESET;
	int ret;

	ret = fts_write(ts, &cmd, 1);
	if (ret < 0)
		return ret;

	msleep(FTS_DELAY_UPGRADE_RESET);
	return 0;
}

static bool fts_fwupg_check_state(struct fts_upgrade *upg, enum FW_STATUS rstate)
{
	enum FW_STATUS cstate = FTS_RUN_IN_ERROR;
	int i;
	int ret;

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		ret = fts_fwupg_get_boot_state(upg, &cstate);
		if (cstate == rstate)
			return true;
		msleep(FTS_DELAY_READ_ID);
	}
	return false;
}

int fts_fwupg_enter_into_boot(struct fts_upgrade *upg)
{
	struct fts_ts_data *ts = upg->ts_data;
	bool state;
	int ret;

	ret = fts_wait_tp_to_valid(ts);
	if (ret >= 0) {
		/* FW is running, reset to boot */
		ret = fts_fwupg_reset_to_boot(upg);
		if (ret < 0) {
			dev_err(ts->dev, "reset to boot fail\n");
			return ret;
		}
	}

	if (!upg->func->pramboot_supported) {
		state = fts_fwupg_check_state(upg, FTS_RUN_IN_BOOTLOADER);
		if (!state) {
			dev_err(ts->dev, "fw not in bootloader\n");
			return -EIO;
		}
	}

	return 0;
}

int fts_fwupg_erase(struct fts_ts_data *ts, u32 delay)
{
	u8 cmd = FTS_CMD_ERASE_APP;
	int ret;

	ret = fts_write(ts, &cmd, 1);
	if (ret < 0) {
		dev_err(ts->dev, "erase cmd fail\n");
		return ret;
	}
	msleep(delay);

	if (!fts_fwupg_check_flash_status(ts, FTS_CMD_FLASH_STATUS_ERASE_OK,
					  FTS_RETRIES_REASE,
					  FTS_RETRIES_DELAY_REASE)) {
		dev_err(ts->dev, "erase status check fail\n");
		return -EIO;
	}

	return 0;
}

int fts_flash_write_buf(struct fts_upgrade *upg, u32 saddr, u8 *buf,
			u32 len, u32 delay)
{
	struct fts_ts_data *ts = upg->ts_data;
	u8 packet_buf[FTS_FLASH_PACKET_LENGTH + FTS_CMD_WRITE_LEN];
	u32 packet_num, packet_len, offset, addr, remainder;
	u8 ecc_tmp = 0;
	u8 cmd, val[2];
	u16 read_status, wr_ok;
	u32 i, j;
	int ret;

	if (!buf || !len)
		return -EINVAL;

	packet_num = len / FTS_FLASH_PACKET_LENGTH;
	remainder = len % FTS_FLASH_PACKET_LENGTH;
	if (remainder > 0)
		packet_num++;
	packet_len = FTS_FLASH_PACKET_LENGTH;

	packet_buf[0] = FTS_CMD_WRITE;
	for (i = 0; i < packet_num; i++) {
		offset = i * FTS_FLASH_PACKET_LENGTH;
		addr = saddr + offset;
		packet_buf[1] = BYTE_OFF_16(addr);
		packet_buf[2] = BYTE_OFF_8(addr);
		packet_buf[3] = BYTE_OFF_0(addr);

		if (i == packet_num - 1 && remainder)
			packet_len = remainder;

		packet_buf[4] = BYTE_OFF_8(packet_len);
		packet_buf[5] = BYTE_OFF_0(packet_len);

		for (j = 0; j < packet_len; j++) {
			packet_buf[FTS_CMD_WRITE_LEN + j] = buf[offset + j];
			ecc_tmp ^= packet_buf[FTS_CMD_WRITE_LEN + j];
		}

		ret = fts_write(ts, packet_buf, packet_len + FTS_CMD_WRITE_LEN);
		if (ret < 0) {
			dev_err(ts->dev, "app write fail\n");
			return ret;
		}
		mdelay(delay);

		/* check write status */
		wr_ok = FTS_CMD_FLASH_STATUS_WRITE_OK + addr / packet_len;
		for (j = 0; j < FTS_RETRIES_WRITE; j++) {
			cmd = FTS_CMD_FLASH_STATUS;
			ret = fts_read(ts, &cmd, 1, val, 2);
			if (ret >= 0) {
				read_status = ((u16)val[0] << 8) + val[1];
				if (wr_ok == read_status)
					break;
			}
			mdelay(FTS_RETRIES_DELAY_WRITE);
		}
	}

	return (int)ecc_tmp;
}

int fts_fwupg_ecc_cal(struct fts_upgrade *upg, u32 saddr, u32 len)
{
	struct fts_ts_data *ts = upg->ts_data;
	u8 wbuf[FTS_CMD_ECC_CAL_LEN];
	u8 val[2];
	u32 packet_num, packet_len, offset, addr, remainder;
	int ret;
	u32 i;

	/* checksum init */
	wbuf[0] = FTS_CMD_ECC_INIT;
	ret = fts_write(ts, wbuf, 1);
	if (ret < 0) {
		dev_err(ts->dev, "ecc init cmd fail\n");
		return ret;
	}

	packet_num = len / FTS_MAX_LEN_ECC_CALC;
	remainder = len % FTS_MAX_LEN_ECC_CALC;
	if (remainder)
		packet_num++;

	packet_len = FTS_MAX_LEN_ECC_CALC;
	wbuf[0] = FTS_CMD_ECC_CAL;
	for (i = 0; i < packet_num; i++) {
		offset = FTS_MAX_LEN_ECC_CALC * i;
		addr = saddr + offset;
		wbuf[1] = BYTE_OFF_16(addr);
		wbuf[2] = BYTE_OFF_8(addr);
		wbuf[3] = BYTE_OFF_0(addr);

		if (i == packet_num - 1 && remainder)
			packet_len = remainder;
		wbuf[4] = BYTE_OFF_8(packet_len);
		wbuf[5] = BYTE_OFF_0(packet_len);

		ret = fts_write(ts, wbuf, FTS_CMD_ECC_CAL_LEN);
		if (ret < 0) {
			dev_err(ts->dev, "ecc calc cmd fail\n");
			return ret;
		}
		msleep(packet_len / 256);

		if (!fts_fwupg_check_flash_status(ts,
						  FTS_CMD_FLASH_STATUS_ECC_OK,
						  FTS_RETRIES_ECC_CAL,
						  FTS_RETRIES_DELAY_ECC_CAL)) {
			dev_err(ts->dev, "ecc status check fail\n");
			return -EIO;
		}
	}

	wbuf[0] = FTS_CMD_ECC_READ;
	ret = fts_read(ts, wbuf, 1, val, 1);
	if (ret < 0) {
		dev_err(ts->dev, "ecc read cmd fail\n");
		return ret;
	}

	return (int)val[0];
}

int fts_fwupg_init(struct fts_ts_data *ts_data)
{
	fwupgrade = &fwupgrade_storage;
	fwupgrade->ts_data = ts_data;
	fwupgrade->func = &upgrade_func_ft5452;

	dev_dbg(ts_data->dev, "fw upgrade init done\n");
	return 0;
}

void fts_fwupg_exit(struct fts_ts_data *ts_data)
{
	if (fwupgrade) {
		if (fwupgrade->fw_from_request)
			vfree(fwupgrade->fw);
		fwupgrade = NULL;
	}
}
