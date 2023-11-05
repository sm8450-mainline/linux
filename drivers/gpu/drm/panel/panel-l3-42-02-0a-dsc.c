// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2023 FIXME
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
//   Copyright (c) 2013, The Linux Foundation. All rights reserved. (FIXME)

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>

#include <video/mipi_display.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct l3_42_02_0a_dsc {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator_bulk_data supplies[3];
	struct drm_dsc_config dsc;
	struct gpio_desc *reset_gpio;
};

static inline
struct l3_42_02_0a_dsc *to_l3_42_02_0a_dsc(struct drm_panel *panel)
{
	return container_of(panel, struct l3_42_02_0a_dsc, panel);
}

static void l3_42_02_0a_dsc_reset(struct l3_42_02_0a_dsc *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(11000, 12000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(11000, 12000);
}

static int l3_42_02_0a_dsc_on(struct l3_42_02_0a_dsc *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	/* VESA ON */
	mipi_dsi_dcs_write_seq(dsi, 0x90, 0x01);
	/* VESA Edition */
	mipi_dsi_dcs_write_seq(dsi, 0x91,
			       0xab, 0x28, 0x00, 0x0c, 0xc2, 0x00, 0x03, 0x1c,
			       0x01, 0x7e, 0x00, 0x0f, 0x08, 0xbb, 0x04, 0x3d,
			       0x10, 0xf0);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_GET_COMPRESSION_MODE, 0x01);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_MEMORY_START, 0x00);

	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0) {
		dev_err(dev, "Failed to set tear on: %d\n", ret);
		return ret;
	}

	/* BACKLIGHT ON */
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x20);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_SET_DISPLAY_BRIGHTNESS, 0x00, 0x00, 0x00, 0x00);

	ret = mipi_dsi_dcs_set_column_address(dsi, 0x0000, 0x0437);
	if (ret < 0) {
		dev_err(dev, "Failed to set column address: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_set_page_address(dsi, 0x0000, 0x095f);
	if (ret < 0) {
		dev_err(dev, "Failed to set page address: %d\n", ret);
		return ret;
	}

	/* 60 Hz */
	mipi_dsi_dcs_write_seq(dsi, 0x2f, 0x01);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_SET_GAMMA_CURVE, 0x01);
	/* OSC calibration */
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x01);
	mipi_dsi_dcs_write_seq(dsi, 0xc3,
			       0x94, 0x01, 0x8c, 0xd0, 0x22, 0x02, 0x00);
	/* enter aod with no black */
	mipi_dsi_dcs_write_seq(dsi, 0x6f, 0x01);
	mipi_dsi_dcs_write_seq(dsi, 0xd2, 0x00, 0x00, 0x11);
	mipi_dsi_dcs_write_seq(dsi, 0x6f, 0x06);
	mipi_dsi_dcs_write_seq(dsi, 0xd2, 0x05);
	mipi_dsi_dcs_write_seq(dsi, 0x6f, 0x0f);
	mipi_dsi_dcs_write_seq(dsi, 0xd2, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0x6f, 0x09);
	mipi_dsi_dcs_write_seq(dsi, 0xd2, 0x00, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xce, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xff, 0xaa, 0x55, 0xa5, 0x80);
	mipi_dsi_dcs_write_seq(dsi, 0x6f, 0x61);
	mipi_dsi_dcs_write_seq(dsi, 0xf3, 0x80);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xc0, 0x46);
	mipi_dsi_dcs_write_seq(dsi, 0x6f, 0x0b);
	mipi_dsi_dcs_write_seq(dsi, 0xb5, 0x23, 0x2b);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0x6f, 0x29);
	mipi_dsi_dcs_write_seq(dsi, 0xd9, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0x6f, 0x01);
	mipi_dsi_dcs_write_seq(dsi, 0xd9, 0x80);
	mipi_dsi_dcs_write_seq(dsi, 0x6f, 0x07);
	mipi_dsi_dcs_write_seq(dsi, 0xb2, 0x07, 0xff);
	mipi_dsi_dcs_write_seq(dsi, 0x6f, 0x17);
	mipi_dsi_dcs_write_seq(dsi, 0xb2, 0x07, 0xff);
	mipi_dsi_dcs_write_seq(dsi, 0x6f, 0x1f);
	mipi_dsi_dcs_write_seq(dsi, 0xb2, 0x00, 0x50);
	/* round for fpr, corner and cup */
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x07);
	mipi_dsi_dcs_write_seq(dsi, 0xc9,
			       0x21, 0x00, 0x27, 0xd9, 0x27, 0xd9, 0x00, 0x00,
			       0x3f, 0xe0, 0x8e, 0xc6, 0x3f, 0xe0, 0x8e, 0xc6,
			       0x80, 0x06, 0x33, 0xd5, 0xf1, 0x00, 0x16, 0x13,
			       0x00, 0x7b, 0x78, 0x33, 0xd0, 0x27, 0xd9, 0x0f,
			       0x9b, 0x9b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			       0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xca,
			       0x27, 0x00, 0x27, 0xd9, 0x27, 0xd9, 0x00, 0x00,
			       0x00, 0x00, 0x00, 0x00, 0x3f, 0xe0, 0x8e, 0xc6,
			       0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x13,
			       0x00, 0xe1, 0x78, 0x33, 0xd0, 0x00, 0x00, 0x03,
			       0x65, 0x9b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			       0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xcb,
			       0x2d, 0x00, 0x27, 0xd9, 0x27, 0xd9, 0x00, 0x00,
			       0x3f, 0xe0, 0x8e, 0xc6, 0x80, 0x00, 0x00, 0x00,
			       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x79,
			       0x00, 0x7b, 0xde, 0x33, 0x70, 0x00, 0x00, 0x0c,
			       0x9b, 0x65, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			       0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xcc,
			       0x2b, 0x00, 0x27, 0xd9, 0x27, 0xd9, 0x00, 0x00,
			       0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00,
			       0x7f, 0xf9, 0xcc, 0x2a, 0x0f, 0x00, 0x7c, 0x79,
			       0x00, 0xe1, 0xde, 0x33, 0x7f, 0xd8, 0x27, 0x00,
			       0x65, 0x65, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			       0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xc1, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xc2, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xc3, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xc4, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xc5, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xc6, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xc7, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xc8, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xcd, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xce, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xcf, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xd0, 0x00);
	/* round on */
	mipi_dsi_dcs_write_seq(dsi, 0xc0, 0x05, 0x02);
	/* ESD error flag, active high */
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0x6f, 0x05);
	mipi_dsi_dcs_write_seq(dsi, 0xbe, 0x0a);
	/* SRAM not power off at idle mode */
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0x6f, 0x01);
	mipi_dsi_dcs_write_seq(dsi, 0xc0, 0xb3);
	mipi_dsi_dcs_write_seq(dsi, 0xff, 0xaa, 0x55, 0xa5, 0x80);
	mipi_dsi_dcs_write_seq(dsi, 0x6f, 0x2e);
	mipi_dsi_dcs_write_seq(dsi, 0xfb, 0xd1);

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode: %d\n", ret);
		return ret;
	}
	msleep(80);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display on: %d\n", ret);
		return ret;
	}

	return 0;
}

static int l3_42_02_0a_dsc_off(struct l3_42_02_0a_dsc *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display off: %d\n", ret);
		return ret;
	}
	msleep(20);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode: %d\n", ret);
		return ret;
	}
	msleep(100);

	return 0;
}

static int l3_42_02_0a_dsc_prepare(struct drm_panel *panel)
{
	struct l3_42_02_0a_dsc *ctx = to_l3_42_02_0a_dsc(panel);
	struct device *dev = &ctx->dsi->dev;
	struct drm_dsc_picture_parameter_set pps;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators: %d\n", ret);
		return ret;
	}

	l3_42_02_0a_dsc_reset(ctx);

	ret = l3_42_02_0a_dsc_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
		return ret;
	}

	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);

	ret = mipi_dsi_picture_parameter_set(ctx->dsi, &pps);
	if (ret < 0) {
		dev_err(panel->dev, "failed to transmit PPS: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_compression_mode(ctx->dsi, true);
	if (ret < 0) {
		dev_err(dev, "failed to enable compression mode: %d\n", ret);
		return ret;
	}

	msleep(28); /* TODO: Is this panel-dependent? */

	return 0;
}

static int l3_42_02_0a_dsc_unprepare(struct drm_panel *panel)
{
	struct l3_42_02_0a_dsc *ctx = to_l3_42_02_0a_dsc(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = l3_42_02_0a_dsc_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);

	return 0;
}

static const struct drm_display_mode l3_42_02_0a_dsc_mode = {
	.clock = (1080 + 32 + 16 + 32) * (2400 + 64 + 16 + 32) * 60 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 32,
	.hsync_end = 1080 + 32 + 16,
	.htotal = 1080 + 32 + 16 + 32,
	.vdisplay = 2400,
	.vsync_start = 2400 + 64,
	.vsync_end = 2400 + 64 + 16,
	.vtotal = 2400 + 64 + 16 + 32,
	.width_mm = 65,
	.height_mm = 145,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int l3_42_02_0a_dsc_get_modes(struct drm_panel *panel,
				     struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &l3_42_02_0a_dsc_mode);
}

static const struct drm_panel_funcs l3_42_02_0a_dsc_panel_funcs = {
	.prepare = l3_42_02_0a_dsc_prepare,
	.unprepare = l3_42_02_0a_dsc_unprepare,
	.get_modes = l3_42_02_0a_dsc_get_modes,
};

static int l3_42_02_0a_dsc_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

static int l3_42_02_0a_dsc_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness_large(dsi, &brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return brightness;
}

static const struct backlight_ops l3_42_02_0a_dsc_bl_ops = {
	.update_status = l3_42_02_0a_dsc_bl_update_status,
	.get_brightness = l3_42_02_0a_dsc_bl_get_brightness,
};

static struct backlight_device *
l3_42_02_0a_dsc_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 2048,
		.max_brightness = 4095,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &l3_42_02_0a_dsc_bl_ops, &props);
}

static int l3_42_02_0a_dsc_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct l3_42_02_0a_dsc *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->supplies[0].supply = "vddd";
	ctx->supplies[1].supply = "vci";
	ctx->supplies[2].supply = "vddio";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->supplies),
				      ctx->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM;

	drm_panel_init(&ctx->panel, dev, &l3_42_02_0a_dsc_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = l3_42_02_0a_dsc_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	/* This panel only supports DSC; unconditionally enable it */
	dsi->dsc = &ctx->dsc;

	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;

	/* TODO: Pass slice_per_pkt = 1 */
	ctx->dsc.slice_height = 12;
	ctx->dsc.slice_width = 1080;
	/*
	 * TODO: hdisplay should be read from the selected mode once
	 * it is passed back to drm_panel (in prepare?)
	 */
	WARN_ON(1080 % ctx->dsc.slice_width);
	ctx->dsc.slice_count = 1080 / ctx->dsc.slice_width;
	ctx->dsc.bits_per_component = 8;
	ctx->dsc.bits_per_pixel = 8 << 4; /* 4 fractional bits */
	ctx->dsc.block_pred_enable = true;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void l3_42_02_0a_dsc_remove(struct mipi_dsi_device *dsi)
{
	struct l3_42_02_0a_dsc *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id l3_42_02_0a_dsc_of_match[] = {
	{ .compatible = "mdss,l3-42-02-0a-dsc" }, // FIXME
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, l3_42_02_0a_dsc_of_match);

static struct mipi_dsi_driver l3_42_02_0a_dsc_driver = {
	.probe = l3_42_02_0a_dsc_probe,
	.remove = l3_42_02_0a_dsc_remove,
	.driver = {
		.name = "panel-l3-42-02-0a-dsc",
		.of_match_table = l3_42_02_0a_dsc_of_match,
	},
};
module_mipi_dsi_driver(l3_42_02_0a_dsc_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator <fix@me>"); // FIXME
MODULE_DESCRIPTION("DRM driver for xiaomi 42 02 0a mp cmd mode dsc dsi panel");
MODULE_LICENSE("GPL");
