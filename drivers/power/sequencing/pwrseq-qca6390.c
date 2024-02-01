// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2024 Linaro Ltd.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/pwrseq/provider.h>
#include <linux/string.h>
#include <linux/types.h>

struct pwrseq_qca6390_vreg {
	const char *name;
	unsigned int load_uA;
};

struct pwrseq_qca6390_pdata {
	const struct pwrseq_qca6390_vreg *vregs;
	size_t num_vregs;
	unsigned int pwup_delay_msec;
};

struct pwrseq_qca6390_ctx {
	struct pwrseq_device *pwrseq;
	struct device_node *of_node;
	const struct pwrseq_qca6390_pdata *pdata;
	struct regulator_bulk_data *regs;
	struct gpio_desc *bt_gpio;
	struct gpio_desc *wlan_gpio;
};

static const struct pwrseq_qca6390_vreg pwrseq_qca6390_vregs[] = {
	{
		.name = "vddio",
		.load_uA = 20000,
	},
	{
		.name = "vddaon",
		.load_uA = 100000,
	},
	{
		.name = "vddpmu",
		.load_uA = 1250000,
	},
	{
		.name = "vddpcie1",
		.load_uA = 35000,
	},
	{
		.name = "vddpcie2",
		.load_uA = 15000,
	},
	{
		.name = "vddrfa1",
		.load_uA = 200000,
	},
	{
		.name = "vddrfa2",
		.load_uA = 400000,
	},
	{
		.name = "vddrfa3",
		.load_uA = 400000,
	},
};

static const struct pwrseq_qca6390_pdata pwrseq_qca6390_of_data = {
	.vregs = pwrseq_qca6390_vregs,
	.num_vregs = ARRAY_SIZE(pwrseq_qca6390_vregs),
	.pwup_delay_msec = 16,
};

static int pwrseq_qca6390_power_on(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qca6390_ctx *ctx = pwrseq_device_get_data(pwrseq);
	int ret;

	ret = regulator_bulk_enable(ctx->pdata->num_vregs, ctx->regs);
	if (ret)
		return ret;

	gpiod_set_value_cansleep(ctx->bt_gpio, 1);
	gpiod_set_value_cansleep(ctx->wlan_gpio, 1);

	if (ctx->pdata->pwup_delay_msec)
		msleep(ctx->pdata->pwup_delay_msec);

	return 0;
}

static int pwrseq_qca6390_power_off(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qca6390_ctx *ctx = pwrseq_device_get_data(pwrseq);

	gpiod_set_value_cansleep(ctx->bt_gpio, 0);
	gpiod_set_value_cansleep(ctx->wlan_gpio, 0);

	return regulator_bulk_disable(ctx->pdata->num_vregs, ctx->regs);
}

static int pwrseq_qca6390_match(struct pwrseq_device *pwrseq,
				struct device *dev)
{
	struct pwrseq_qca6390_ctx *ctx = pwrseq_device_get_data(pwrseq);
	struct device_node *dev_node = dev->of_node;

	/*
	 * The PMU supplies power to the Bluetooth and WLAN modules. both
	 * consume the PMU AON output so check the presence of the
	 * 'vddaon-supply' property and whether it leads us to the right
	 * device.
	 */
	if (!of_property_present(dev_node, "vddaon-supply"))
		return 0;

	struct device_node *reg_node __free(of_node) =
			of_parse_phandle(dev_node, "vddaon-supply", 0);
	if (!reg_node)
		return 0;

	/*
	 * `reg_node` is the PMU AON regulator, its parent is the `regulators`
	 * node and finally its grandparent is the PMU device node that we're
	 * looking for.
	 */
	if (!reg_node->parent || !reg_node->parent->parent ||
	    reg_node->parent->parent != ctx->of_node)
		return 0;

	return 1;
}

static int pwrseq_qca6390_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pwrseq_qca6390_ctx *ctx;
	struct pwrseq_config config;
	int ret, i;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->of_node = dev->of_node;

	ctx->pdata = of_device_get_match_data(dev);
	if (!ctx->pdata)
		return dev_err_probe(dev, -ENODEV,
				     "Failed to obtain platform data\n");

	if (ctx->pdata->vregs) {
		ctx->regs = devm_kcalloc(dev, ctx->pdata->num_vregs,
					 sizeof(*ctx->regs), GFP_KERNEL);
		if (!ctx->regs)
			return -ENOMEM;

		for (i = 0; i < ctx->pdata->num_vregs; i++)
			ctx->regs[i].supply = ctx->pdata->vregs[i].name;

		ret = devm_regulator_bulk_get(dev, ctx->pdata->num_vregs,
					      ctx->regs);
		if (ret < 0)
			return dev_err_probe(dev, ret,
					     "Failed to get all regulators\n");

		for (i = 0; i < ctx->pdata->num_vregs; i++) {
			if (!ctx->pdata->vregs[1].load_uA)
				continue;

			ret = regulator_set_load(ctx->regs[i].consumer,
						 ctx->pdata->vregs[i].load_uA);
			if (ret)
				return dev_err_probe(dev, ret,
						     "Failed to set vreg load\n");
		}
	}

	ctx->bt_gpio = devm_gpiod_get_optional(dev, "bt-enable", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->bt_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->bt_gpio),
				     "Failed to get the Bluetooth enable GPIO\n");

	ctx->wlan_gpio = devm_gpiod_get_optional(dev, "wlan-enable",
						 GPIOD_OUT_LOW);
	if (IS_ERR(ctx->wlan_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->wlan_gpio),
				     "Failed to get the WLAN enable GPIO\n");

	memset(&config, 0, sizeof(config));

	config.parent = dev;
	config.owner = THIS_MODULE;
	config.drvdata = ctx;
	config.match = pwrseq_qca6390_match;
	config.power_on = pwrseq_qca6390_power_on;
	config.power_off = pwrseq_qca6390_power_off;

	ctx->pwrseq = devm_pwrseq_device_register(dev, &config);
	if (IS_ERR(ctx->pwrseq))
		return dev_err_probe(dev, PTR_ERR(ctx->pwrseq),
				     "Failed to register the power sequencer\n");

	return 0;
}

static const struct of_device_id pwrseq_qca6390_of_match[] = {
	{
		.compatible = "qcom,qca6390-pmu",
		.data = &pwrseq_qca6390_of_data,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, pwrseq_qca6390_of_match);

static struct platform_driver pwrseq_qca6390_driver = {
	.driver = {
		.name = "pwrseq-qca6390",
		.of_match_table = pwrseq_qca6390_of_match,
	},
	.probe = pwrseq_qca6390_probe,
};
module_platform_driver(pwrseq_qca6390_driver);

MODULE_AUTHOR("Bartosz Golaszewski <bartosz.golaszewski@linaro.org>");
MODULE_DESCRIPTION("QCA6390 PMU power sequencing driver");
MODULE_LICENSE("GPL");
