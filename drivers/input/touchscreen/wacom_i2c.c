// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Wacom Penabled Driver for I2C
 *
 * Copyright (c) 2011 - 2013 Tatsunosuke Tobita, Wacom.
 * <tobita.tatsunosuke@wacom.co.jp>
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/of.h>
#include <asm/unaligned.h>

// Registers
#define WACOM_COMMAND_LSB   0x04
#define WACOM_COMMAND_MSB   0x00

#define WACOM_DATA_LSB      0x05
#define WACOM_DATA_MSB      0x00

// Report types
#define REPORT_FEATURE      0x30

// Requests / operations
#define OPCODE_GET_REPORT   0x02

// Power settings
#define POWER_ON            0x00
#define POWER_SLEEP         0x01

// Input report ids
#define WACOM_PEN_DATA_REPORT           2
#define WACOM_SHINONOME_REPORT          26

#define WACOM_QUERY_REPORT	3
#define WACOM_QUERY_SIZE	22

struct wacom_features {
	int x_max;
	int y_max;
	int pressure_max;
	int distance_max;
	int distance_physical_max;
	int tilt_x_max;
	int tilt_y_max;
	char fw_version;
};

struct wacom_i2c {
	struct i2c_client *client;
	struct input_dev *input;
	struct touchscreen_properties props;
	struct regulator *vdd;
	u8 data[WACOM_QUERY_SIZE];
	bool prox;
	int tool;
};

static int wacom_query_device(struct i2c_client *client,
			      struct wacom_features *features)
{
	int ret;
	u8 data[WACOM_QUERY_SIZE];
	struct reset_control *rstc;

	u8 get_query_data_cmd[] = {
		WACOM_COMMAND_LSB,
		WACOM_COMMAND_MSB,
		REPORT_FEATURE | WACOM_QUERY_REPORT,
		OPCODE_GET_REPORT,
		WACOM_DATA_LSB,
		WACOM_DATA_MSB,
	};

	struct i2c_msg msgs[] = {
		// Request reading of feature ReportID: 3 (Pen Query Data)
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(get_query_data_cmd),
			.buf = get_query_data_cmd,
		},
		// Read 21 bytes
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = WACOM_QUERY_SIZE - 1,
			.buf = data,
		},
	};

	rstc = devm_reset_control_get_optional_exclusive(&client->dev, NULL);
	if (IS_ERR(rstc))
		dev_err(&client->dev, "Failed to get reset control before init\n");
	else
		reset_control_reset(rstc);

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	features->x_max = get_unaligned_le16(&data[3]);
	features->y_max = get_unaligned_le16(&data[5]);
	features->pressure_max = get_unaligned_le16(&data[11]);
	features->fw_version = get_unaligned_le16(&data[13]);
	features->distance_max = data[15];
	features->distance_physical_max = data[16];
	features->tilt_x_max = get_unaligned_le16(&data[17]);
	features->tilt_y_max = get_unaligned_le16(&data[19]);

	dev_dbg(&client->dev,
		"x_max:%d, y_max:%d, pressure:%d, fw:%d, "
		"distance: %d, phys distance: %d, "
		"tilt_x_max: %d, tilt_y_max: %d\n",
		features->x_max, features->y_max,
		features->pressure_max, features->fw_version,
		features->distance_max, features->distance_physical_max,
		features->tilt_x_max, features->tilt_y_max);

	return 0;
}

static irqreturn_t wacom_i2c_irq(int irq, void *dev_id)
{
	struct wacom_i2c *wac_i2c = dev_id;
	struct input_dev *input = wac_i2c->input;
	u8 *data = wac_i2c->data;
	unsigned int x, y, pressure;
	unsigned char tsw, f1, f2, ers;
	short tilt_x, tilt_y, distance;
	int error;

	error = i2c_master_recv(wac_i2c->client,
				wac_i2c->data, sizeof(wac_i2c->data));
	if (error < 0)
		goto out;

	tsw = data[3] & 0x01;
	ers = data[3] & 0x04;
	f1 = data[3] & 0x02;
	f2 = data[3] & 0x10;
	x = le16_to_cpup((__le16 *)&data[4]);
	y = le16_to_cpup((__le16 *)&data[6]);
	pressure = le16_to_cpup((__le16 *)&data[8]);
	distance = data[10];

	/* Signed */
	tilt_x = le16_to_cpup((__le16 *)&data[11]);
	tilt_y = le16_to_cpup((__le16 *)&data[13]);

	if (!wac_i2c->prox)
		wac_i2c->tool = (data[3] & 0x0c) ?
			BTN_TOOL_RUBBER : BTN_TOOL_PEN;

	wac_i2c->prox = data[3] & 0x20;

	input_report_key(input, BTN_TOUCH, tsw || ers);
	input_report_key(input, wac_i2c->tool, wac_i2c->prox);
	input_report_key(input, BTN_STYLUS, f1);
	input_report_key(input, BTN_STYLUS2, f2);
	input_report_abs(input, ABS_X, x);
	input_report_abs(input, ABS_Y, y);
	input_report_abs(input, ABS_PRESSURE, pressure);
	input_report_abs(input, ABS_DISTANCE, distance);
	input_report_abs(input, ABS_TILT_X, tilt_x);
	input_report_abs(input, ABS_TILT_Y, tilt_y);
	input_sync(input);

out:
	return IRQ_HANDLED;
}

static int wacom_i2c_open(struct input_dev *dev)
{
	struct wacom_i2c *wac_i2c = input_get_drvdata(dev);
	struct i2c_client *client = wac_i2c->client;

	enable_irq(client->irq);

	return 0;
}

static void wacom_i2c_close(struct input_dev *dev)
{
	struct wacom_i2c *wac_i2c = input_get_drvdata(dev);
	struct i2c_client *client = wac_i2c->client;

	disable_irq(client->irq);
}

static int wacom_i2c_probe(struct i2c_client *client,
				     const struct i2c_device_id *id)
{
	struct wacom_i2c *wac_i2c;
	struct input_dev *input;
	struct wacom_features features = { 0 };
	int error;

	wac_i2c = kzalloc(sizeof(*wac_i2c), GFP_KERNEL);
	if (!wac_i2c)
		return -ENOMEM;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -EIO;
	}

	wac_i2c->vdd = regulator_get(&client->dev, "vdd");
	if (IS_ERR(wac_i2c->vdd)) {
		error = PTR_ERR(wac_i2c->vdd);
		kfree(wac_i2c);
		return error;
	}

	error = regulator_enable(wac_i2c->vdd);
	if (error) {
		regulator_put(wac_i2c->vdd);
		kfree(wac_i2c);
		return error;
	}

	error = wacom_query_device(client, &features);
	if (error)
		return error;

	wac_i2c = kzalloc(sizeof(*wac_i2c), GFP_KERNEL);
	input = input_allocate_device();
	if (!wac_i2c || !input) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	wac_i2c->client = client;
	wac_i2c->input = input;

	input->name = "Wacom I2C Digitizer";
	input->id.bustype = BUS_I2C;
	input->id.vendor = 0x56a;
	input->id.version = features.fw_version;
	input->dev.parent = &client->dev;
	input->open = wacom_i2c_open;
	input->close = wacom_i2c_close;

	input->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

	__set_bit(BTN_TOOL_PEN, input->keybit);
	__set_bit(BTN_TOOL_RUBBER, input->keybit);
	__set_bit(BTN_STYLUS, input->keybit);
	__set_bit(BTN_STYLUS2, input->keybit);
	__set_bit(BTN_TOUCH, input->keybit);

	touchscreen_parse_properties(input, true, &wac_i2c->props);
	input_set_abs_params(input, ABS_X, 0, features.x_max, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, features.y_max, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE,
			     0, features.pressure_max, 0, 0);
	input_set_abs_params(input, ABS_DISTANCE, 0, features.distance_max, 0, 0);
	input_set_abs_params(input, ABS_TILT_X, -features.tilt_x_max,
			     features.tilt_x_max, 0, 0);
	input_set_abs_params(input, ABS_TILT_Y, -features.tilt_y_max,
			     features.tilt_y_max, 0, 0);
	input_set_drvdata(input, wac_i2c);

	error = request_threaded_irq(client->irq, NULL, wacom_i2c_irq,
				     IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				     "wacom_i2c", wac_i2c);
	if (error) {
		dev_err(&client->dev,
			"Failed to enable IRQ, error: %d\n", error);
		goto err_free_mem;
	}

	/* Disable the IRQ, we'll enable it in wac_i2c_open() */
	disable_irq(client->irq);

	error = input_register_device(wac_i2c->input);
	if (error) {
		dev_err(&client->dev,
			"Failed to register input device, error: %d\n", error);
		goto err_free_irq;
	}

	i2c_set_clientdata(client, wac_i2c);
	return 0;

err_free_irq:
	free_irq(client->irq, wac_i2c);
err_free_mem:
	regulator_disable(wac_i2c->vdd);
	regulator_put(wac_i2c->vdd);
	input_free_device(input);
	kfree(wac_i2c);

	return error;
}

static int wacom_i2c_remove(struct i2c_client *client)
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);

	free_irq(client->irq, wac_i2c);
	input_unregister_device(wac_i2c->input);
	kfree(wac_i2c);

	return 0;
}

static int __maybe_unused wacom_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	disable_irq(client->irq);

	return 0;
}

static int __maybe_unused wacom_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	enable_irq(client->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(wacom_i2c_pm, wacom_i2c_suspend, wacom_i2c_resume);

static const struct i2c_device_id wacom_i2c_id[] = {
	{ "WAC_I2C_EMR", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, wacom_i2c_id);

static const struct of_device_id wacom_i2c_of_match_table[] __maybe_unused = {
	{ .compatible = "wacom,generic" },
	{}
};
MODULE_DEVICE_TABLE(of, wacom_i2c_of_match_table);

static struct i2c_driver wacom_i2c_driver = {
	.driver	= {
		.name	= "wacom_i2c",
		.pm	= &wacom_i2c_pm,
		.of_match_table = of_match_ptr(wacom_i2c_of_match_table),
	},

	.probe		= wacom_i2c_probe,
	.remove		= wacom_i2c_remove,
	.id_table	= wacom_i2c_id,
};
module_i2c_driver(wacom_i2c_driver);

MODULE_AUTHOR("Tatsunosuke Tobita <tobita.tatsunosuke@wacom.co.jp>");
MODULE_DESCRIPTION("WACOM EMR I2C Driver");
MODULE_LICENSE("GPL");
