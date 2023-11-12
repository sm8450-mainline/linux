// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020, Linaro Limited
 */
#include "asm-generic/errno-base.h"
#include "linux/dev_printk.h"
#include "linux/device.h"
#include "linux/gfp_types.h"
#include "linux/gpio/consumer.h"
#include "linux/overflow.h"
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/devinfo.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include "../gpio/gpiolib.h"
//#include <linux/gpio/consumer.h>

struct vreg {
	const char *name;
	unsigned int load_uA;
};

struct qca_cfg_data {
	const struct vreg *vregs;
	size_t num_vregs;
};

static const struct vreg qca639x_vregs[] = {
	/* 2.0 V */
	{ "vddpcie2", 15000 },
	{ "vddrfa3", 400000 },

	/* 0.95 V */
	{ "vddaon", 100000 },
	{ "vddpmu", 1250000 },
	{ "vddrfa1", 200000 },

	/* 1.35 V */
	{ "vddrfa2", 400000 },
	{ "vddpcie1", 35000 },

	/* 1.8 V */
	{ "vddio", 20000 },
};

static const struct qca_cfg_data qca639x_cfg_data = {
	.vregs = qca639x_vregs,
	.num_vregs = ARRAY_SIZE(qca639x_vregs),
};

/* QCA6490 does *not* like it when we set regulator load */
static const struct vreg qca6490_vregs[] = {
	/* 2.8 V */
	{ "vddasd", 0 },

	/* 2.0 V */
	{ "vddpcie2", 0 },

	/* 1.9 V */
	{ "vddrfa1", 0 },
	{ "vddrfa2", 0 },
	{ "vddrfa3", 0 }, /* also labeled vdd-s1c */
	{ "vddpcie1", 0 },

	/* 0.95 V */
	{ "vddaonbt", 0 },
	{ "vddaonwl", 0 },
	//{ "vdddig", 0 },
	{ "vddio", 0 }
};

static const struct qca_cfg_data qca6490_cfg_data = {
	.vregs = qca6490_vregs,
	.num_vregs = ARRAY_SIZE(qca6490_vregs),
};

struct qca6xxx_data {
	size_t num_vregs;
	size_t num_gpios;
	struct device *dev;
	struct gpio_desc *rf_en;
	struct pinctrl_state *active_state;
	struct generic_pm_domain pd;
	struct regulator_bulk_data *regulators;
};

#define domain_to_data(domain) container_of(domain, struct qca6xxx_data, pd)

static int qca6xxx_power_on(struct generic_pm_domain *domain)
{
	struct qca6xxx_data *data = domain_to_data(domain);
	int ret;

	ret = regulator_bulk_enable(data->num_vregs, data->regulators);
	if (ret) {
		dev_err(data->dev, "Failed to enable regulators");
		return ret;
	}

	/* Wait for 1ms before toggling enable pins. */
	usleep_range(1000, 2000);

	ret = pinctrl_select_state(data->dev->pins->p, data->active_state);
	if (ret) {
		dev_err(data->dev, "Failed to select active state");
		return ret;
	}

	/* Wait for all power levels to stabilize */
	usleep_range(6000, 7000);

	return 0;
}

static int qca6xxx_power_off(struct generic_pm_domain *domain)
{
	struct qca6xxx_data *data = domain_to_data(domain);

	pinctrl_select_default_state(data->dev);
	regulator_bulk_disable(data->num_vregs, data->regulators);

	return 0;
}

static int qca6xxx_probe(struct platform_device *pdev)
{
	const struct qca_cfg_data *cfg;
	struct qca6xxx_data *data;
	struct device *dev = &pdev->dev;
	int i, ret = 0;

	cfg = device_get_match_data(&pdev->dev);
	if (!cfg)
		return -EINVAL;

	if (!dev->pins || IS_ERR_OR_NULL(dev->pins->default_state))
		return -EINVAL;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	data->num_vregs = cfg->num_vregs;

	data->active_state = pinctrl_lookup_state(dev->pins->p, "active");
	if (IS_ERR(data->active_state)) {
		ret = PTR_ERR(data->active_state);
		dev_err(dev, "Failed to get active_state: %d\n", ret);
		return ret;
	}

	data->num_gpios = gpiod_count(data->dev, "rf");
	if (data->num_gpios > 0) {
		data->rf_en = devm_kmalloc_array(data->dev,
				data->num_gpios,
				sizeof(*data->rf_en),
				GFP_KERNEL);
		if (data->rf_en == 0)
			return -ENOMEM;
		for (i = 0; i < data->num_gpios; i++) {
			(&data->rf_en)[i] = devm_gpiod_get_index(data->dev, "rf", i, GPIOD_ASIS);
			if ((&data->rf_en)[i] > 0)
				gpiod_direction_input((&data->rf_en)[i]);
			else {
				dev_warn(dev, "Could not get RF pin %d; continuing\n", i);
				goto cont;
			}
		}
	}
	else
		dev_info(data->dev, "No RF pins provided, ignoring");
cont:
	data->regulators = devm_kmalloc_array(data->dev,
					      data->num_vregs,
					      sizeof(*data->regulators),
					      GFP_KERNEL);
	if (data->regulators == 0)
		return -ENOMEM;
	for (i = 0; i < data->num_vregs; i++)
		data->regulators[i].supply = cfg->vregs[i].name;
	ret = devm_regulator_bulk_get(dev, data->num_vregs, data->regulators);
	if (ret < 0)
		return ret;

	for (i = 0; i < data->num_vregs; i++) {
		if (cfg->vregs[i].load_uA > 0) {
			ret = regulator_set_load(data->regulators[i].consumer, cfg->vregs[i].load_uA);
			if (ret)
				return ret;
		}
	}

	data->pd.name = dev_name(dev);
	data->pd.power_on = qca6xxx_power_on;
	data->pd.power_off = qca6xxx_power_off;

	ret = pm_genpd_init(&data->pd, NULL, true);
	if (ret < 0)
		return ret;

	ret = of_genpd_add_provider_simple(dev->of_node, &data->pd);
	if (ret < 0) {
		pm_genpd_remove(&data->pd);
		return ret;
	}

	platform_set_drvdata(pdev, data);

	return 0;
}

static int qca6xxx_remove(struct platform_device *pdev)
{
	struct qca6xxx_data *data = platform_get_drvdata(pdev);

	pm_genpd_remove(&data->pd);

	return 0;
}

static const struct of_device_id qca6xxx_of_match[] = {
	{ .compatible = "qcom,qca639x", .data = &qca639x_cfg_data },
	{ .compatible = "qcom,qca6490", .data = &qca6490_cfg_data },
	{},
};

static struct platform_driver qca6xxx_driver = {
	.probe = qca6xxx_probe,
	.remove = qca6xxx_remove,
	.driver = {
		.name = "qca6xxx",
		.of_match_table = qca6xxx_of_match,
	},
};

module_platform_driver(qca6xxx_driver);
MODULE_AUTHOR("Dmitry Baryshkov <dmitry.baryshkov@linaro.org>");
MODULE_DESCRIPTION("Power control for Qualcomm QCA6xxx BT/WiFi chip");
MODULE_LICENSE("GPL v2");