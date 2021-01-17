// SPDX-License-Identifier: GPL-2.0+
//
// Functions to access SY3686A power management chip voltages
//
// Copyright (C) 2019 reMarkable AS - http://www.remarkable.com/
//
// Authors: Lars Ivar Miljeteig <lars.ivar.miljeteig@remarkable.com>
//          Alistair Francis <alistair@alistair23.me>

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/mfd/sy7636a.h>

static int sy7636a_get_vcom_voltage_mv(struct regmap *regmap)
{
	int ret;
	unsigned int val, val_h;

	ret = regmap_read(regmap, SY7636A_REG_VCOM_ADJUST_CTRL_L, &val);
	if (ret)
		return ret;

	ret = regmap_read(regmap, SY7636A_REG_VCOM_ADJUST_CTRL_H, &val_h);
	if (ret)
		return ret;

	val |= (val_h << 8);

	return (val & 0x1FF) * 10000;
}

static int sy7636a_get_vcom_voltage_op(struct regulator_dev *rdev)
{
	return sy7636a_get_vcom_voltage_mv(rdev->regmap);
}

static int sy7636a_disable_regulator(struct regulator_dev *rdev)
{
	struct sy7636a *sy7636a = dev_get_drvdata(rdev->dev.parent);
	int ret = 0;

	ret = regulator_disable_regmap(rdev);
	// Delay for ~35ms after disabling the regulator, to allow power ramp
	// down to go undisturbed
	usleep_range(30000, 35000);

	return ret;
}

static int sy7636a_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct sy7636a *sy7636a = dev_get_drvdata(rdev->dev.parent);
	int ret;

	ret = regulator_is_enabled_regmap(rdev);

	return ret;
}

static int sy7636a_get_status(struct regulator_dev *rdev)
{
	struct sy7636a *sy7636a = dev_get_drvdata(rdev->dev.parent);
	int pwr_good = 0;
	const unsigned int wait_time = 500;
	unsigned int wait_cnt;
	int ret = 0;

	for (wait_cnt = 0; wait_cnt < wait_time; wait_cnt++) {
		pwr_good = gpiod_get_value_cansleep(sy7636a->pgood_gpio);
		if (pwr_good < 0) {
			dev_err(&rdev->dev, "Failed to read pgood gpio: %d\n", pwr_good);
			ret = pwr_good;
			return ret;
		} else if (pwr_good)
			break;

		usleep_range(1000, 1500);
	}

	return ret;
}

static int sy7636a_enable_regulator_pgood(struct regulator_dev *rdev)
{
	struct sy7636a *sy7636a = dev_get_drvdata(rdev->dev.parent);
	int pwr_good = 0;
	int ret = 0;
	unsigned long t0, t1;
	const unsigned int wait_time = 500;
	unsigned int wait_cnt;

	t0 = jiffies;

	ret = regulator_enable_regmap(rdev);
	if (ret)
		goto finish;

	for (wait_cnt = 0; wait_cnt < wait_time; wait_cnt++) {
		pwr_good = gpiod_get_value_cansleep(sy7636a->pgood_gpio);
		if (pwr_good < 0) {
			dev_err(&rdev->dev, "Failed to read pgood gpio: %d\n", pwr_good);
			ret = pwr_good;
			goto finish;
		} else if (pwr_good)
			break;

		usleep_range(1000, 1500);
	}

	t1 = jiffies;

	if (!pwr_good) {
		dev_err(&rdev->dev, "Power good signal timeout after %u ms\n",
				jiffies_to_msecs(t1 - t0));
		ret = -ETIME;
		sy7636a_disable_regulator(rdev);
		goto finish;
	}

	dev_dbg(&rdev->dev, "Power good OK (took %u ms, %u waits)\n",
		jiffies_to_msecs(t1 - t0),
		wait_cnt);

finish:
	return ret;
}

static const struct regulator_ops sy7636a_vcom_volt_ops = {
	.get_voltage = sy7636a_get_vcom_voltage_op,
	.enable = sy7636a_enable_regulator_pgood,
	.disable = sy7636a_disable_regulator,
	.is_enabled = sy7636a_regulator_is_enabled,
	.get_status = sy7636a_get_status,
};

struct regulator_desc desc = {
	.name = "vcom",
	.id = 0,
	.ops = &sy7636a_vcom_volt_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.enable_reg = SY7636A_REG_OPERATION_MODE_CRL,
	.enable_mask = SY7636A_OPERATION_MODE_CRL_ONOFF,
	.regulators_node = of_match_ptr("regulators"),
	.of_match = of_match_ptr("vcom"),
};

static int sy7636a_regulator_init(struct sy7636a *sy7636a)
{
	return regmap_write(sy7636a->regmap,
				SY7636A_REG_POWER_ON_DELAY_TIME,
				0x0);
}

static int sy7636a_regulator_probe(struct platform_device *pdev)
{
	struct sy7636a *sy7636a = dev_get_drvdata(pdev->dev.parent);
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	struct gpio_desc *gdp;
	int ret;

	if (!sy7636a)
		return -EPROBE_DEFER;

	platform_set_drvdata(pdev, sy7636a);

	gdp = devm_gpiod_get(sy7636a->dev, "epd-pwr-good", GPIOD_IN);
	if (IS_ERR(gdp)) {
		dev_err(sy7636a->dev, "Power good GPIO fault %ld\n", PTR_ERR(gdp));
		return PTR_ERR(gdp);
	}

	sy7636a->pgood_gpio = gdp;

	ret = sy7636a_regulator_init(sy7636a);
	if (ret) {
		dev_err(sy7636a->dev, "Failed to initialize regulator: %d\n", ret);
		return ret;
	}

	config.dev = &pdev->dev;
	config.dev->of_node = sy7636a->dev->of_node;
	config.driver_data = sy7636a;
	config.regmap = sy7636a->regmap;

	rdev = devm_regulator_register(&pdev->dev, &desc, &config);
	if (IS_ERR(rdev)) {
		dev_err(sy7636a->dev, "Failed to register %s regulator\n",
			pdev->name);
		return PTR_ERR(rdev);
	}

	return 0;
}

static const struct platform_device_id sy7636a_regulator_id_table[] = {
	{ "sy7636a-regulator", },
};
MODULE_DEVICE_TABLE(platform, sy7636a_regulator_id_table);

static struct platform_driver sy7636a_regulator_driver = {
	.driver = {
		.name = "sy7636a-regulator",
	},
	.probe = sy7636a_regulator_probe,
	.id_table = sy7636a_regulator_id_table,
};
module_platform_driver(sy7636a_regulator_driver);

MODULE_AUTHOR("Lars Ivar Miljeteig <lars.ivar.miljeteig@remarkable.com>");
MODULE_DESCRIPTION("SY7636A voltage regulator driver");
MODULE_LICENSE("GPL v2");
