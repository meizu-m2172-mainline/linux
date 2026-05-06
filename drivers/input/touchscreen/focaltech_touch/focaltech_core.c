// SPDX-License-Identifier: GPL-2.0-only
/*
 * FocalTech FT3518 TouchScreen driver (I2C)
 *
 * Copyright (c) 2012-2019, FocalTech Systems, Ltd.
 * Copyright (c) 2024, Meizu 18X Linux port
 *
 * Cleaned up from Android downstream for mainline compliance.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include "focaltech_core.h"

#define FTS_DRIVER_NAME		"focaltech_ts"
#define I2C_BUF_SIZE		256
#define I2C_RETRY_NUM		3

/* Timing (ms) */
#define FTS_RESET_DELAY		200
#define FTS_CHIP_READY_INTERVAL	20
#define FTS_CHIP_READY_TIMEOUT	1000

/* Commands */
#define FTS_CMD_START1		0x55
#define FTS_CMD_START2		0xAA
#define FTS_CMD_START_DELAY	12
#define FTS_CMD_READ_ID		0x90
#define FTS_CMD_READ_ID_LEN	4

static const u8 fts_hid2std_cmd[] = { 0xEB, 0xAA, 0x09 };

/*****************************************************************************
 * I2C communication
 *****************************************************************************/
static int fts_i2c_read(struct fts_ts_data *ts, u8 *cmd, u32 cmdlen,
			u8 *data, u32 datalen)
{
	struct i2c_msg msg[2];
	int msg_num = 1;
	int ret;
	int i;

	if (!ts || !ts->client || !data || !datalen ||
	    datalen >= I2C_BUF_SIZE || cmdlen >= I2C_BUF_SIZE)
		return -EINVAL;

	mutex_lock(&ts->bus_lock);
	if (cmd && cmdlen) {
		memcpy(ts->bus_tx_buf, cmd, cmdlen);
		msg[0].addr = ts->client->addr;
		msg[0].flags = 0;
		msg[0].len = cmdlen;
		msg[0].buf = ts->bus_tx_buf;
		msg[1].addr = ts->client->addr;
		msg[1].flags = I2C_M_RD;
		msg[1].len = datalen;
		msg[1].buf = ts->bus_rx_buf;
		msg_num = 2;
	} else {
		msg[0].addr = ts->client->addr;
		msg[0].flags = I2C_M_RD;
		msg[0].len = datalen;
		msg[0].buf = ts->bus_rx_buf;
	}

	for (i = 0; i < I2C_RETRY_NUM; i++) {
		ret = i2c_transfer(ts->client->adapter, msg, msg_num);
		if (ret >= 0) {
			memcpy(data, ts->bus_rx_buf, datalen);
			goto out;
		}
		dev_dbg(ts->dev, "I2C read retry %d (ret=%d)\n", i, ret);
	}
out:
	mutex_unlock(&ts->bus_lock);
	return ret;
}

int fts_read(struct fts_ts_data *ts, u8 *cmd, u32 cmdlen, u8 *data, u32 datalen)
{
	if (!cmd || !cmdlen)
		return fts_i2c_read(ts, NULL, 0, data, datalen);

	return fts_i2c_read(ts, cmd, cmdlen, data, datalen);
}

int fts_read_reg(struct fts_ts_data *ts, u8 addr, u8 *value)
{
	return fts_read(ts, &addr, 1, value, 1);
}

int fts_write(struct fts_ts_data *ts, u8 *buf, u32 len)
{
	struct i2c_msg msg;
	int ret;
	int i;

	if (!ts || !ts->client || !buf || !len || len >= I2C_BUF_SIZE)
		return -EINVAL;

	mutex_lock(&ts->bus_lock);
	memcpy(ts->bus_tx_buf, buf, len);
	msg.addr = ts->client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = ts->bus_tx_buf;

	for (i = 0; i < I2C_RETRY_NUM; i++) {
		ret = i2c_transfer(ts->client->adapter, &msg, 1);
		if (ret >= 0)
			goto out;
		dev_dbg(ts->dev, "I2C write retry %d (ret=%d)\n", i, ret);
	}
out:
	mutex_unlock(&ts->bus_lock);
	return ret;
}

int fts_write_reg(struct fts_ts_data *ts, u8 addr, u8 value)
{
	u8 buf[2] = { addr, value };

	return fts_write(ts, buf, sizeof(buf));
}

static int fts_bus_init(struct fts_ts_data *ts)
{
	ts->bus_tx_buf = devm_kzalloc(ts->dev, I2C_BUF_SIZE, GFP_KERNEL);
	ts->bus_rx_buf = devm_kzalloc(ts->dev, I2C_BUF_SIZE, GFP_KERNEL);
	if (!ts->bus_tx_buf || !ts->bus_rx_buf)
		return -ENOMEM;

	return 0;
}

/*****************************************************************************
 * Reset and chip identification
 *****************************************************************************/
int fts_reset_proc(struct fts_ts_data *ts, int hdelayms)
{
	dev_dbg(ts->dev, "reset tp\n");
	gpiod_set_value(ts->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value(ts->reset_gpio, 0);
	if (hdelayms)
		msleep(hdelayms);

	return 0;
}

int fts_wait_tp_to_valid(struct fts_ts_data *ts)
{
	u8 idh, idl;
	int cnt = 0;
	int ret;

	do {
		ret = fts_read_reg(ts, FTS_REG_CHIP_ID, &idh);
		if (ret < 0)
			goto retry;
		ret = fts_read_reg(ts, FTS_REG_CHIP_ID2, &idl);
		if (ret < 0)
			goto retry;
		if (idh == ts->chip_id.chip_idh && idl == ts->chip_id.chip_idl) {
			dev_info(ts->dev, "TP ready: chip ID 0x%02x%02x\n", idh, idl);
			return 0;
		}
retry:
		cnt++;
		msleep(FTS_CHIP_READY_INTERVAL);
	} while ((cnt * FTS_CHIP_READY_INTERVAL) < FTS_CHIP_READY_TIMEOUT);

	dev_err(ts->dev, "TP not ready after timeout\n");
	return -EIO;
}

static void fts_hid2std(struct fts_ts_data *ts)
{
	u8 buf[3] = { 0 };
	int ret;

	ret = fts_write(ts, (u8 *)fts_hid2std_cmd, FTS_HID2STD_CMD_LEN);
	if (ret < 0)
		return;

	msleep(10);
	ret = fts_read(ts, NULL, 0, buf, 3);
	if (ret < 0)
		return;

	if (buf[0] == 0xEB && buf[1] == 0xAA && buf[2] == 0x08)
		dev_dbg(ts->dev, "HID to STD I2C mode switch ok\n");
}

static int fts_get_chip_types(struct fts_ts_data *ts, u8 id_h, u8 id_l,
			      bool fw_valid)
{
	struct ft_chip_id ctype[] = FTS_CHIP_TYPE_MAPPING;
	int i, entries = sizeof(ctype) / sizeof(struct ft_chip_id);

	if (!id_h || !id_l)
		return -EINVAL;

	dev_dbg(ts->dev, "verify chip ID: 0x%02x%02x\n", id_h, id_l);
	for (i = 0; i < entries; i++) {
		if (fw_valid) {
			if (id_h == ctype[i].chip_idh && id_l == ctype[i].chip_idl)
				break;
		} else {
			if ((id_h == ctype[i].rom_idh && id_l == ctype[i].rom_idl) ||
			    (id_h == ctype[i].pb_idh && id_l == ctype[i].pb_idl) ||
			    (id_h == ctype[i].bl_idh && id_l == ctype[i].bl_idl))
				break;
		}
	}

	if (i >= entries)
		return -ENODATA;

	ts->chip_id = ctype[i];
	return 0;
}

static int fts_read_bootid(struct fts_ts_data *ts, u8 *chip_id)
{
	u8 cmd[4] = { FTS_CMD_START1, FTS_CMD_START2, 0, 0 };
	int ret;

	ret = fts_write(ts, cmd, 2);
	if (ret < 0)
		return ret;

	msleep(FTS_CMD_START_DELAY);
	cmd[0] = FTS_CMD_READ_ID;
	ret = fts_read(ts, cmd, FTS_CMD_READ_ID_LEN, chip_id, 2);
	if (ret < 0 || !chip_id[0] || !chip_id[1]) {
		dev_err(ts->dev, "read boot id fail: 0x%02x%02x\n",
			chip_id[0], chip_id[1]);
		return -EIO;
	}

	return 0;
}

static int fts_get_ic_information(struct fts_ts_data *ts)
{
	u8 chip_id[2] = { 0 };
	int cnt = 0;
	int ret;

	ts->is_incell = false;
	ts->hid_supported = true;

	do {
		ret = fts_read_reg(ts, FTS_REG_CHIP_ID, &chip_id[0]);
		if (ret < 0)
			goto retry_ic;
		ret = fts_read_reg(ts, FTS_REG_CHIP_ID2, &chip_id[1]);
		if (ret < 0)
			goto retry_ic;
		if (chip_id[0] && chip_id[1]) {
			ret = fts_get_chip_types(ts, chip_id[0], chip_id[1], true);
			if (!ret)
				break;
		}
retry_ic:
		cnt++;
		msleep(FTS_CHIP_READY_INTERVAL);
	} while ((cnt * FTS_CHIP_READY_INTERVAL) < FTS_CHIP_READY_TIMEOUT);

	if ((cnt * FTS_CHIP_READY_INTERVAL) >= FTS_CHIP_READY_TIMEOUT) {
		dev_info(ts->dev, "fw not running, reading boot id\n");
		if (ts->hid_supported)
			fts_hid2std(ts);

		ret = fts_read_bootid(ts, chip_id);
		if (ret < 0)
			return ret;

		ret = fts_get_chip_types(ts, chip_id[0], chip_id[1], false);
		if (ret < 0) {
			dev_err(ts->dev, "unknown chip type\n");
			return ret;
		}
		ts->fw_is_running = 0;
	} else {
		ts->fw_is_running = 1;
	}

	dev_info(ts->dev, "chip ID: 0x%02x%02x\n",
		 ts->chip_id.chip_idh, ts->chip_id.chip_idl);
	return 0;
}

/*****************************************************************************
 * Touch data parsing
 *****************************************************************************/
static void fts_release_all_fingers(struct fts_ts_data *ts)
{
	int i;

	for (i = 0; i < ts->max_touch_points; i++) {
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
	}
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_sync(ts->input_dev);
	ts->touchs = 0;
	ts->key_state = 0;
}

static int fts_read_touchdata(struct fts_ts_data *ts)
{
	u8 *buf = ts->point_buf;
	int ret;

	memset(buf, 0xFF, ts->pnt_buf_size);
	buf[0] = 0x01;

	ret = fts_read(ts, buf, 1, buf + 1, ts->pnt_buf_size - 1);
	if (ret < 0) {
		dev_err(ts->dev, "read touch data fail: %d\n", ret);
		return ret;
	}

	return 0;
}

static int fts_read_parse_touchdata(struct fts_ts_data *ts)
{
	u8 *buf = ts->point_buf;
	struct ts_event *events = ts->events;
	int max_touch = ts->max_touch_points;
	int i, base, pointid;
	int ret;

	ret = fts_read_touchdata(ts);
	if (ret)
		return ret;

	ts->point_num = buf[FTS_TOUCH_POINT_NUM] & 0x0F;
	ts->touch_point = 0;

	/* Check for all-0xFF buffer (in-cell recovery) */
	if (ts->is_incell && ts->point_num == 0x0F &&
	    buf[2] == 0xFF && buf[3] == 0xFF &&
	    buf[4] == 0xFF && buf[5] == 0xFF && buf[6] == 0xFF) {
		dev_warn(ts->dev, "touch buffer all 0xFF, recovering\n");
		fts_release_all_fingers(ts);
		return -EIO;
	}

	if (ts->point_num > max_touch) {
		dev_warn(ts->dev, "invalid point_num %d\n", ts->point_num);
		return -EIO;
	}

	for (i = 0; i < max_touch; i++) {
		base = FTS_ONE_TCH_LEN * i;
		pointid = buf[FTS_TOUCH_ID_POS + base] >> 4;
		if (pointid >= FTS_MAX_ID)
			break;
		if (pointid >= max_touch) {
			dev_err(ts->dev, "ID %d exceeds max\n", pointid);
			return -EINVAL;
		}

		ts->touch_point++;
		events[i].x = ((buf[FTS_TOUCH_X_H_POS + base] & 0x0F) << 8) +
			      (buf[FTS_TOUCH_X_L_POS + base] & 0xFF);
		events[i].y = ((buf[FTS_TOUCH_Y_H_POS + base] & 0x0F) << 8) +
			      (buf[FTS_TOUCH_Y_L_POS + base] & 0xFF);
		events[i].flag = buf[FTS_TOUCH_EVENT_POS + base] >> 6;
		events[i].id = buf[FTS_TOUCH_ID_POS + base] >> 4;
		events[i].area = buf[FTS_TOUCH_AREA_POS + base] >> 4;
		events[i].p = buf[FTS_TOUCH_PRE_POS + base];

		if (EVENT_DOWN(events[i].flag) && ts->point_num == 0) {
			dev_warn(ts->dev, "abnormal touch data from fw\n");
			return -EIO;
		}
	}

	if (ts->touch_point == 0)
		return -EIO;

	return 0;
}

static void fts_input_report(struct fts_ts_data *ts)
{
	struct input_dev *input = ts->input_dev;
	struct ts_event *events = ts->events;
	int i, touchs = 0;
	bool reported = false;
	u32 max_touch = ts->max_touch_points;

	for (i = 0; i < ts->touch_point; i++) {
		input_mt_slot(input, events[i].id);

		if (EVENT_DOWN(events[i].flag)) {
			input_mt_report_slot_state(input, MT_TOOL_FINGER, true);

			if (events[i].p <= 0)
				events[i].p = 0x3f;
			input_report_abs(input, ABS_MT_PRESSURE, events[i].p);

			if (events[i].area <= 0)
				events[i].area = 0x09;
			input_report_abs(input, ABS_MT_TOUCH_MAJOR, events[i].area);

			touchscreen_report_pos(input, &ts->prop,
					       events[i].x, events[i].y, true);

			touchs |= BIT(events[i].id);

			dev_dbg(ts->dev, "P%d(%d,%d) DOWN\n",
				events[i].id, events[i].x, events[i].y);
		} else {
			input_mt_report_slot_state(input, MT_TOOL_FINGER, false);
			dev_dbg(ts->dev, "P%d UP\n", events[i].id);
		}
		reported = true;
	}

	/* Release any slots still tracked but not in current frame */
	for (i = 0; i < max_touch; i++) {
		if ((ts->touchs & BIT(i)) && !(touchs & BIT(i))) {
			input_mt_slot(input, i);
			input_mt_report_slot_state(input, MT_TOOL_FINGER,
						   false);
			reported = true;
			dev_dbg(ts->dev, "P%d UP (stale)\n", i);
		}
	}
	ts->touchs = touchs;

	if (reported) {
		input_report_key(input, BTN_TOUCH, touchs > 0);
		input_sync(input);
	}
}

/*****************************************************************************
 * IRQ handler
 *****************************************************************************/
static irqreturn_t fts_irq_handler(int irq, void *dev_id)
{
	struct fts_ts_data *ts = dev_id;
	int ret;

	ret = fts_read_parse_touchdata(ts);
	if (ret == 0) {
		mutex_lock(&ts->report_mutex);
		fts_input_report(ts);
		mutex_unlock(&ts->report_mutex);
	}

	return IRQ_HANDLED;
}

/*****************************************************************************
 * Suspend / Resume
 *****************************************************************************/
static int fts_suspend(struct device *dev)
{
	struct fts_ts_data *ts = dev_get_drvdata(dev);
	int ret;

	if (ts->suspended)
		return 0;

	if (ts->fw_loading) {
		dev_info(dev, "fw upgrade in progress, can't suspend\n");
		return 0;
	}

	disable_irq(ts->irq);

	ret = fts_write_reg(ts, FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_SLEEP);
	if (ret < 0)
		dev_err(dev, "set sleep mode fail: %d\n", ret);

	fts_release_all_fingers(ts);
	ts->suspended = true;

	return 0;
}

static int fts_resume(struct device *dev)
{
	struct fts_ts_data *ts = dev_get_drvdata(dev);

	if (!ts->suspended)
		return 0;

	fts_reset_proc(ts, FTS_RESET_DELAY);
	fts_wait_tp_to_valid(ts);

	enable_irq(ts->irq);
	ts->suspended = false;

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(fts_pm_ops, fts_suspend, fts_resume);

/*****************************************************************************
 * Probe / Remove
 *****************************************************************************/
static int fts_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fts_ts_data *ts;
	struct input_dev *input;
	u32 max_touch = FTS_MAX_POINTS;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "I2C not supported\n");
		return -ENODEV;
	}

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	ts->dev = dev;
	ts->log_level = 1;
	i2c_set_clientdata(client, ts);
	mutex_init(&ts->bus_lock);
	mutex_init(&ts->report_mutex);

	/* GPIOs */
	ts->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "failed to get reset GPIO\n");

	/* Regulators */
	ts->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(ts->vdd))
		return dev_err_probe(dev, PTR_ERR(ts->vdd),
				     "failed to get vdd regulator\n");

	ts->vcc_i2c = devm_regulator_get_optional(dev, "vcc_i2c");
	if (IS_ERR(ts->vcc_i2c))
		ts->vcc_i2c = NULL;

	/* Power on: assert reset, enable regulators, then deassert */
	gpiod_set_value(ts->reset_gpio, 1);
	msleep(1);

	ret = regulator_enable(ts->vdd);
	if (ret) {
		dev_err(dev, "failed to enable vdd: %d\n", ret);
		return ret;
	}

	if (ts->vcc_i2c) {
		ret = regulator_enable(ts->vcc_i2c);
		if (ret) {
			dev_err(dev, "failed to enable vcc_i2c: %d\n", ret);
			regulator_disable(ts->vdd);
			return ret;
		}
	}

	/* I2C buffers */
	ret = fts_bus_init(ts);
	if (ret) {
		dev_err(dev, "bus init fail\n");
		goto err_power;
	}

	/* Deassert reset and wait for chip to stabilize */
	fts_reset_proc(ts, FTS_RESET_DELAY);

	ret = fts_get_ic_information(ts);
	if (ret) {
		dev_err(dev, "failed to identify IC\n");
		goto err_power;
	}

	/* Max touch points from DT */
	device_property_read_u32(dev, "focaltech,max-touch-number", &max_touch);
	if (max_touch < 2)
		max_touch = 2;
	if (max_touch > FTS_MAX_POINTS)
		max_touch = FTS_MAX_POINTS;
	ts->max_touch_points = max_touch;

	/* Input device */
	input = devm_input_allocate_device(dev);
	if (!input) {
		ret = -ENOMEM;
		goto err_power;
	}

	input->name = FTS_DRIVER_NAME;
	input->id.bustype = BUS_I2C;
	input->dev.parent = dev;

	input_set_capability(input, EV_KEY, BTN_TOUCH);
	__set_bit(INPUT_PROP_DIRECT, input->propbit);

	input_set_abs_params(input, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);

	touchscreen_parse_properties(input, true, &ts->prop);
	if (!ts->prop.max_x || !ts->prop.max_y) {
		/* Fallback to default */
		input_set_abs_params(input, ABS_MT_POSITION_X, 0, 1080, 0, 0);
		input_set_abs_params(input, ABS_MT_POSITION_Y, 0, 2400, 0, 0);
	}

	input_mt_init_slots(input, max_touch, INPUT_MT_DIRECT);

	ret = input_register_device(input);
	if (ret) {
		dev_err(dev, "failed to register input: %d\n", ret);
		goto err_power;
	}
	ts->input_dev = input;

	/* Touch buffer */
	ts->pnt_buf_size = max_touch * FTS_ONE_TCH_LEN + 3;
	ts->point_buf = devm_kzalloc(dev, ts->pnt_buf_size + 1, GFP_KERNEL);
	ts->events = devm_kcalloc(dev, max_touch, sizeof(struct ts_event),
				  GFP_KERNEL);
	if (!ts->point_buf || !ts->events) {
		ret = -ENOMEM;
		goto err_power;
	}

	/* IRQ */
	ts->irq = client->irq;
	ret = devm_request_threaded_irq(dev, ts->irq, NULL, fts_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					FTS_DRIVER_NAME, ts);
	if (ret) {
		dev_err(dev, "failed to request IRQ: %d\n", ret);
		goto err_power;
	}

	/* Firmware upgrade */
	ret = fts_fwupg_init(ts);
	if (ret)
		dev_warn(dev, "fw upgrade init fail: %d\n", ret);

	dev_info(dev, "probe success (max_touch=%d)\n", ts->max_touch_points);
	return 0;

err_power:
	if (ts->vcc_i2c)
		regulator_disable(ts->vcc_i2c);
	regulator_disable(ts->vdd);
	return ret;
}

static void fts_remove(struct i2c_client *client)
{
	struct fts_ts_data *ts = i2c_get_clientdata(client);

	fts_fwupg_exit(ts);
	disable_irq(ts->irq);

	if (ts->vcc_i2c)
		regulator_disable(ts->vcc_i2c);
	regulator_disable(ts->vdd);
}

static const struct i2c_device_id fts_id[] = {
	{ FTS_DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fts_id);

static const struct of_device_id fts_dt_match[] = {
	{ .compatible = "focaltech,fts" },
	{ .compatible = "focaltech,fts_ts" },
	{ }
};
MODULE_DEVICE_TABLE(of, fts_dt_match);

static struct i2c_driver fts_driver = {
	.probe = fts_probe,
	.remove = fts_remove,
	.driver = {
		.name = FTS_DRIVER_NAME,
		.of_match_table = fts_dt_match,
		.pm = pm_sleep_ptr(&fts_pm_ops),
	},
	.id_table = fts_id,
};
module_i2c_driver(fts_driver);

MODULE_AUTHOR("FocalTech Driver Team");
MODULE_DESCRIPTION("FocalTech FT3518 Touchscreen Driver");
MODULE_LICENSE("GPL");
