/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * FocalTech FT3518 TouchScreen driver
 *
 * Copyright (c) 2012-2019, FocalTech Systems, Ltd.
 * Copyright (c) 2024, Meizu 18X Linux port
 */

#ifndef __FOCALTECH_CORE_H__
#define __FOCALTECH_CORE_H__

#include <linux/types.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/touchscreen.h>
#include <linux/mutex.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <drm/drm_panel.h>

/* Register addresses */
#define FTS_REG_CHIP_ID			0xA3
#define FTS_REG_CHIP_ID2		0x9F
#define FTS_REG_POWER_MODE		0xA5
#define FTS_REG_POWER_MODE_SLEEP	0x03
#define FTS_REG_WORKMODE		0x00
#define FTS_REG_WORKMODE_WORK_VALUE	0x00

/* Touch data constants */
#define FTS_MAX_POINTS			10
#define FTS_ONE_TCH_LEN			6
#define FTS_TOUCH_DATA_LEN		(FTS_MAX_POINTS * FTS_ONE_TCH_LEN + 3)

#define FTS_MAX_ID			0x0A
#define FTS_TOUCH_X_H_POS		3
#define FTS_TOUCH_X_L_POS		4
#define FTS_TOUCH_Y_H_POS		5
#define FTS_TOUCH_Y_L_POS		6
#define FTS_TOUCH_PRE_POS		7
#define FTS_TOUCH_AREA_POS		8
#define FTS_TOUCH_POINT_NUM		2
#define FTS_TOUCH_EVENT_POS		3
#define FTS_TOUCH_ID_POS		5

#define FTS_TOUCH_DOWN			0
#define FTS_TOUCH_UP			1
#define FTS_TOUCH_CONTACT		2
#define EVENT_DOWN(flag) \
	((flag) == FTS_TOUCH_DOWN || (flag) == FTS_TOUCH_CONTACT)
#define EVENT_UP(flag)		((flag) == FTS_TOUCH_UP)
#define EVENT_NO_DOWN(data)	(!(data)->point_num)

/* HID-to-standard I2C command */
#define FTS_HID2STD_CMD_LEN		3

/* Chip type mapping: {type, chip_idh, chip_idl, rom_idh, rom_idl, pb_idh, pb_idl, bl_idh, bl_idl} */
#define FTS_CHIP_TYPE_MAPPING { \
	{0x81, 0x54, 0x52, 0x54, 0x52, 0x00, 0x00, 0x54, 0x5C}, /* FT5452 */ \
	{0x81, 0x35, 0x18, 0x04, 0x81, 0x00, 0x00, 0x00, 0x00}, /* FT3518 */ \
}

struct ft_chip_id {
	u64 type;
	u8 chip_idh;
	u8 chip_idl;
	u8 rom_idh;
	u8 rom_idl;
	u8 pb_idh;
	u8 pb_idl;
	u8 bl_idh;
	u8 bl_idl;
};

struct ts_event {
	int x;
	int y;
	int p;
	int flag;
	int id;
	int area;
};

struct fts_ts_data {
	struct i2c_client *client;
	struct device *dev;
	struct input_dev *input_dev;
	struct touchscreen_properties prop;
	struct drm_panel_follower panel_follower;
	struct gpio_desc *reset_gpio;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	struct mutex bus_lock;
	struct mutex report_mutex;
	int irq;
	bool suspended;
	bool fw_loading;
	int max_touch_points;
	int touchs;
	int key_state;
	int touch_point;
	int point_num;

	struct ft_chip_id chip_id;
	bool is_incell;
	bool hid_supported;
	int fw_is_running;

	/* buffers */
	struct ts_event *events;
	u8 *bus_tx_buf;
	u8 *bus_rx_buf;
	u8 *point_buf;
	int pnt_buf_size;

	int log_level;
};

/* I2C communication */
int fts_read(struct fts_ts_data *ts_data, u8 *cmd, u32 cmdlen,
	     u8 *data, u32 datalen);
int fts_read_reg(struct fts_ts_data *ts_data, u8 addr, u8 *value);
int fts_write(struct fts_ts_data *ts_data, u8 *buf, u32 len);
int fts_write_reg(struct fts_ts_data *ts_data, u8 addr, u8 value);

/* Reset and recovery */
int fts_reset_proc(struct fts_ts_data *ts_data, int hdelayms);
int fts_wait_tp_to_valid(struct fts_ts_data *ts_data);

/* Firmware upgrade */
int fts_fwupg_init(struct fts_ts_data *ts_data);
void fts_fwupg_exit(struct fts_ts_data *ts_data);

#endif /* __FOCALTECH_CORE_H__ */
