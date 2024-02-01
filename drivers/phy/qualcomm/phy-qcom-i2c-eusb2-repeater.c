// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023, Linaro Limited
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/phy/phy.h>

#define EUSB2_3P0_VOL_MIN			3075000 /* uV */
#define EUSB2_3P0_VOL_MAX			3300000 /* uV */
#define EUSB2_3P0_HPM_LOAD			3500	/* uA */

#define EUSB2_1P8_VOL_MIN			1800000 /* uV */
#define EUSB2_1P8_VOL_MAX			1800000 /* uV */
#define EUSB2_1P8_HPM_LOAD			32000	/* uA */

/* NXP eUSB2 repeater registers */
#define RESET_CONTROL			0x01
#define LINK_CONTROL1			0x02
#define LINK_CONTROL2			0x03
#define eUSB2_RX_CONTROL		0x04
#define eUSB2_TX_CONTROL		0x05
#define USB2_RX_CONTROL			0x06
#define USB2_TX_CONTROL1		0x07
#define USB2_TX_CONTROL2		0x08
#define USB2_HS_TERMINATION		0x09
#define RAP_SIGNATURE			0x0D
#define VDX_CONTROL			0x0E
#define DEVICE_STATUS			0x0F
#define LINK_STATUS			0x10
#define REVISION_ID			0x13
#define CHIP_ID_0			0x14
#define CHIP_ID_1			0x15
#define CHIP_ID_2			0x16

/* TI eUSB2 repeater registers */
#define GPIO0_CONFIG			0x00
#define GPIO1_CONFIG			0x40
#define UART_PORT1			0x50
#define EXTRA_PORT1			0x51
#define U_TX_ADJUST_PORT1		0x70
#define U_HS_TX_PRE_EMPHASIS_P1		0x71
#define U_RX_ADJUST_PORT1		0x72
#define U_DISCONNECT_SQUELCH_PORT1	0x73
#define E_HS_TX_PRE_EMPHASIS_P1		0x77
#define E_TX_ADJUST_PORT1		0x78
#define E_RX_ADJUST_PORT1		0x79
#define REV_ID				0xB0
#define GLOBAL_CONFIG			0xB2
#define INT_ENABLE_1			0xB3
#define INT_ENABLE_2			0xB4
#define BC_CONTROL			0xB6
#define BC_STATUS_1			0xB7
#define INT_STATUS_1			0xA3
#define INT_STATUS_2			0xA4

enum eusb2_repeater_type {
	TI_REPEATER,
	NXP_REPEATER,
};

struct i2c_repeater_chip {
	enum eusb2_repeater_type repeater_type;
};

struct i2c_eusb2_repeater {
	struct device			*dev;
	struct phy			*phy;
	struct regmap			*regmap;
	const struct i2c_repeater_chip	*chip;
	u16				reg_base;
	struct regulator		*vdd18;
	struct regulator		*vdd3;
	bool				power_enabled;

	struct gpio_desc		*reset_gpiod;
	u32				*param_override_seq;
	u8				param_override_seq_cnt;
	u32				*param_override_seq_host;
	u8				param_override_seq_cnt_host;
};

static int eusb2_i2c_read_reg(struct i2c_eusb2_repeater *rptr, u8 reg, u8 *val)
{
	int ret;
	unsigned int reg_val;

	ret = regmap_read(rptr->regmap, reg, &reg_val);
	if (ret < 0) {
		dev_err(rptr->dev, "Failed to read reg: 0x%02x ret=%d\n", reg, ret);
		return ret;
	}

	*val = reg_val;
	dev_dbg(rptr->dev, "read reg: 0x%02x val:0x%02x\n", reg, *val);

	return 0;
}

static int eusb2_i2c_write_reg(struct i2c_eusb2_repeater *rptr, u8 reg, u8 val)
{
	int ret;

	ret = regmap_write(rptr->regmap, reg, val);
	if (ret < 0) {
		dev_err(rptr->dev, "failed to write 0x%02x to reg: 0x%02x ret=%d\n", val, reg, ret);
		return ret;
	}

	dev_dbg(rptr->dev, "write reg: 0x%02x val:0x%02x\n", reg, val);

	return 0;
}

static void eusb2_repeater_update_seq(struct i2c_eusb2_repeater *rptr, u32 *seq, u8 cnt)
{
	int i;

	dev_dbg(rptr->dev, "param override seq count: %d\n", cnt);
	for (i = 0; i < cnt; i = i+2) {
		dev_dbg(rptr->dev, "write 0x%02x to 0x%02x\n", seq[i], seq[i+1]);
		eusb2_i2c_write_reg(rptr, seq[i+1], seq[i]);
	}
}

static int i2c_eusb2_repeater_init(struct phy *phy)
{
	struct i2c_eusb2_repeater *rptr = phy_get_drvdata(phy);
	const struct i2c_repeater_chip *chip = rptr->chip;
	int ret;
	u8 reg_val;

	if (rptr->power_enabled) {
		dev_info(rptr->dev, "regulators are already on\n");
		return 0;
	}

	ret = regulator_set_load(rptr->vdd18, EUSB2_1P8_HPM_LOAD);
	if (ret < 0) {
		dev_err(rptr->dev, "Unable to set HPM of vdd12: %d\n", ret);
		return ret;
	}

	ret = regulator_set_voltage(rptr->vdd18, EUSB2_1P8_VOL_MIN,
						EUSB2_1P8_VOL_MAX);
	if (ret) {
		dev_err(rptr->dev,
				"Unable to set voltage for vdd18: %d\n", ret);
		return ret;
	}

	ret = regulator_enable(rptr->vdd18);
	if (ret) {
		dev_err(rptr->dev, "Unable to enable vdd18: %d\n", ret);
		return ret;
	}

	ret = regulator_set_load(rptr->vdd3, EUSB2_3P0_HPM_LOAD);
	if (ret < 0) {
		dev_err(rptr->dev, "Unable to set HPM of vdd3: %d\n", ret);
		return ret;
	}

	ret = regulator_set_voltage(rptr->vdd3, EUSB2_3P0_VOL_MIN,
						EUSB2_3P0_VOL_MAX);
	if (ret) {
		dev_err(rptr->dev,
				"Unable to set voltage for vdd3: %d\n", ret);
		return ret;
	}

	ret = regulator_enable(rptr->vdd3);
	if (ret) {
		dev_err(rptr->dev, "Unable to enable vdd3: %d\n", ret);
		return ret;
	}

	rptr->power_enabled = true;
	pr_debug("I2C eUSB2 repeater regulators are turned on\n");

	switch (chip->repeater_type) {
	case TI_REPEATER:
		eusb2_i2c_read_reg(rptr, REV_ID, &reg_val);
		break;
	case NXP_REPEATER:
		eusb2_i2c_read_reg(rptr, REVISION_ID, &reg_val);
		break;
	default:
		dev_err(rptr->dev, "Invalid repeater\n");
	}

	dev_info(rptr->dev, "eUSB2 repeater version = 0x%x\n", reg_val);
	dev_info(rptr->dev, "eUSB2 repeater init\n");

	return ret;
}

static int i2c_eusb2_repeater_set_mode(struct phy *phy,
				   enum phy_mode mode, int submode)
{
	struct i2c_eusb2_repeater *rptr = phy_get_drvdata(phy);

	switch (mode) {
	case PHY_MODE_USB_HOST:
		eusb2_repeater_update_seq(rptr, rptr->param_override_seq_host,
					rptr->param_override_seq_cnt_host);
		dev_info(rptr->dev, "Set phy mode to usb host\n");
		break;
	case PHY_MODE_USB_DEVICE:
		eusb2_repeater_update_seq(rptr, rptr->param_override_seq,
					rptr->param_override_seq_cnt);
		dev_info(rptr->dev, "Set phy mode to usb device\n");
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int i2c_eusb2_repeater_exit(struct phy *phy)
{
	struct i2c_eusb2_repeater *rptr = phy_get_drvdata(phy);
	int ret;

	ret = regulator_disable(rptr->vdd3);
	if (ret)
		dev_err(rptr->dev, "Unable to disable vdd3: %d\n", ret);

	ret = regulator_set_voltage(rptr->vdd3, 0, EUSB2_3P0_VOL_MAX);
	if (ret)
		dev_err(rptr->dev,
			"Unable to set (0) voltage for vdd3: %d\n", ret);

	ret = regulator_set_load(rptr->vdd3, 0);
	if (ret < 0)
		dev_err(rptr->dev, "Unable to set (0) HPM of vdd3\n");

	ret = regulator_disable(rptr->vdd18);
	if (ret)
		dev_err(rptr->dev, "Unable to disable vdd18:%d\n", ret);

	ret = regulator_set_voltage(rptr->vdd18, 0, EUSB2_1P8_VOL_MAX);
	if (ret)
		dev_err(rptr->dev,
			"Unable to set (0) voltage for vdd18: %d\n", ret);

	ret = regulator_set_load(rptr->vdd18, 0);
	if (ret < 0)
		dev_err(rptr->dev, "Unable to set LPM of vdd18\n");

	rptr->power_enabled = false;
	dev_dbg(rptr->dev, "I2C eUSB2 repeater's regulators are turned off.\n");

	return ret;
}

static const struct phy_ops i2c_eusb2_repeater_ops = {
	.init		= i2c_eusb2_repeater_init,
	.exit		= i2c_eusb2_repeater_exit,
	.set_mode	= i2c_eusb2_repeater_set_mode,
	.owner		= THIS_MODULE,
};

static const struct regmap_config i2c_eusb2_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
};

static struct i2c_repeater_chip repeater_chip[] = {
	[NXP_REPEATER] = {
		.repeater_type = NXP_REPEATER,
	},
	[TI_REPEATER] = {
		.repeater_type = TI_REPEATER,
	}
};

static const struct of_device_id i2c_eusb2_repeater_of_match_table[] = {
	{
		.compatible = "nxp,eusb2-repeater",
		.data = &repeater_chip[NXP_REPEATER],
	},
	{
		.compatible = "ti,eusb2-repeater",
		.data = &repeater_chip[TI_REPEATER],
	},
	{ },
};
MODULE_DEVICE_TABLE(of, i2c_eusb2_repeater_of_match_table);

static int i2c_eusb2_repeater_probe(struct i2c_client *client)
{
	struct i2c_eusb2_repeater *rptr;
	struct device *dev = &client->dev;
	struct phy_provider *phy_provider;
	struct device_node *np = dev->of_node;
	const struct of_device_id *match;
	int ret, num_elem;
	u32 res;

	rptr = devm_kzalloc(dev, sizeof(*rptr), GFP_KERNEL);
	if (!rptr) {
		dev_err(dev, "unable to allocate i2c_eusb2_repeater\n");
		return -ENOMEM;
	}

	rptr->dev = dev;
	dev_set_drvdata(dev, rptr);

	match = of_match_node(i2c_eusb2_repeater_of_match_table, dev->of_node);
	rptr->chip = match->data;

	rptr->regmap = devm_regmap_init_i2c(client, &i2c_eusb2_regmap);
	if (!rptr->regmap) {
		dev_err(dev, "devm_regmap_init_i2c failed\n");
		return -ENODEV;
	}

	ret = of_property_read_u32(np, "reg", &res);
	if (ret < 0) {
		dev_err(dev, "failed to read reg\n");
		return ret;
	}

	rptr->vdd3 = devm_regulator_get(dev, "vdd3");
	if (IS_ERR(rptr->vdd3)) {
		dev_err(dev, "unable to get vdd3 supply\n");
		return PTR_ERR(rptr->vdd3);
	}

	rptr->vdd18 = devm_regulator_get(dev, "vdd18");
	if (IS_ERR(rptr->vdd18)) {
		dev_err(dev, "unable to get vdd18 supply\n");
		return PTR_ERR(rptr->vdd18);
	}

	rptr->reset_gpiod = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(rptr->reset_gpiod)) {
		return PTR_ERR(rptr->reset_gpiod);
	}

	num_elem = of_property_count_elems_of_size(dev->of_node, "qcom,param-override-seq",
				sizeof(*rptr->param_override_seq));
	if (num_elem > 0) {
		if (num_elem % 2) {
			dev_err(dev, "invalid param_override_seq_len\n");
			return -EINVAL;
		}

		rptr->param_override_seq_cnt = num_elem;
		rptr->param_override_seq = devm_kcalloc(dev,
				rptr->param_override_seq_cnt,
				sizeof(*rptr->param_override_seq), GFP_KERNEL);
		if (!rptr->param_override_seq) {
			return -ENOMEM;
		}

		ret = of_property_read_u32_array(dev->of_node,
				"qcom,param-override-seq",
				rptr->param_override_seq,
				rptr->param_override_seq_cnt);
		if (ret) {
			dev_err(dev, "qcom,param-override-seq read failed %d\n",
									ret);
			return ret;
		}
	}

	num_elem = of_property_count_elems_of_size(dev->of_node, "qcom,param-override-seq-host",
				sizeof(*rptr->param_override_seq_host));
	if (num_elem > 0) {
		if (num_elem % 2) {
			dev_err(dev, "invalid param_override_seq_host_len\n");
			return -EINVAL;
		}

		rptr->param_override_seq_cnt_host = num_elem;
		rptr->param_override_seq_host = devm_kcalloc(dev,
				rptr->param_override_seq_cnt_host,
				sizeof(*rptr->param_override_seq_host), GFP_KERNEL);
		if (!rptr->param_override_seq_host) {
			return -ENOMEM;
		}

		ret = of_property_read_u32_array(dev->of_node,
				"qcom,param-override-seq-host",
				rptr->param_override_seq_host,
				rptr->param_override_seq_cnt_host);
		if (ret) {
			dev_err(dev, "qcom,param-override-seq-host read failed %d\n",
									ret);
			return ret;
		}
	}


	rptr->phy = devm_phy_create(dev, np, &i2c_eusb2_repeater_ops);
	if (IS_ERR(rptr->phy)) {
		dev_err(dev, "failed to create PHY: %d\n", ret);
		return PTR_ERR(rptr->phy);
	}

	phy_set_drvdata(rptr->phy, rptr);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider))
		return PTR_ERR(phy_provider);

	dev_info(dev, "Registered Qcom-I2C-eUSB2 repeater\n");

	return 0;
}

static void i2c_eusb2_repeater_remove(struct i2c_client *client)
{
	struct i2c_eusb2_repeater *rptr = i2c_get_clientdata(client);

	if (!rptr)
		return;

	i2c_eusb2_repeater_exit(rptr->phy);
}

static struct i2c_driver i2c_eusb2_repeater_driver = {
	.probe		= i2c_eusb2_repeater_probe,
	.remove		= i2c_eusb2_repeater_remove,
	.driver = {
		.name	= "qcom-i2c-eusb2-repeater",
		.of_match_table = i2c_eusb2_repeater_of_match_table,
	},
};

module_i2c_driver(i2c_eusb2_repeater_driver);

MODULE_DESCRIPTION("Qualcomm I2C eUSB2 Repeater driver");
MODULE_LICENSE("GPL");
