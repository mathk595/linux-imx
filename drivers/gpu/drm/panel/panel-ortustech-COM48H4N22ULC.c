/*
 * Copyright (C) 2019 Keith & Koep GmbH
 * Author: Philipp Speeth <speeth@keith-koep.com>
 * 
 * Ortustech COM48H4N22ULC HD Mipi Display
 * for Trizeps8mini / Myon 2
 */

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>


struct ortus_panel {
	struct drm_panel base;
        struct drm_panel panel;
	struct mipi_dsi_device *dsi;

	struct gpio_desc *enable_gpio;
	struct gpio_desc *reset_gpio;

	struct backlight_device *backlight;

	bool prepared;
	bool enabled;

	const struct drm_display_mode *mode;
};

static const struct drm_display_mode default_mode = {
	.clock = 67000,
	.hdisplay = 720,
	.hsync_start = 720 + 91,
	.hsync_end = 720 + 91 + 4,
	.htotal = 720 + 91 + 4 + 112,
	.vdisplay = 1280,
	.vsync_start = 1280 + 6,
	.vsync_end = 1280 + 6 + 3,
	.vtotal = 1280 + 6 + 3 + 3,
};

static inline struct ortus_panel *to_ortus_panel(struct drm_panel *panel)
{
	return container_of(panel, struct ortus_panel, base);
}

static int ortus_panel_prepare(struct drm_panel *panel)
{
	struct ortus_panel *ortus = to_ortus_panel(panel);
	//struct device *dev = &ortus->dsi->dev;

	if (ortus->prepared)
		return 0;


	gpiod_set_value(ortus->enable_gpio, 1);
	msleep(5);

	gpiod_set_value(ortus->reset_gpio, 1);
	msleep(5);
	gpiod_set_value(ortus->reset_gpio, 0);;
	msleep(20);
	gpiod_set_value(ortus->reset_gpio, 1);

	ortus->prepared = true;

	return 0;
}

static int ortus_panel_unprepare(struct drm_panel *panel)
{
	struct ortus_panel *ortus = to_ortus_panel(panel);
	struct device *dev = &ortus->dsi->dev;

	if (!ortus->prepared)
		return 0;

	if (ortus->enabled) {
		DRM_DEV_ERROR(dev, "Panel still enabled!\n");
		return -EPERM;
	}

	gpiod_set_value(ortus->enable_gpio, 0);

	ortus->prepared = false;
	return 0;
}

static int ortus_panel_enable(struct drm_panel *panel)
{
	struct ortus_panel *ortus = to_ortus_panel(panel);
	struct mipi_dsi_device *dsi = ortus->dsi;
	struct device *dev = &dsi->dev;
	u16 brightness;
	int ret;

	if (ortus->enabled) {
		DRM_DEV_ERROR(dev, "Panel still enabled!\n");
		return 0;
	}

	if (!ortus->prepared) {
		DRM_DEV_ERROR(dev, "Panel not prepared!\n");
		return -EPERM;
	}

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	
	ret = mipi_dsi_dcs_write_buffer(dsi, (u8[]){0xB9, 0xFF, 0x83, 0x94}, 4);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "failed to set SETEXTC (%d)\n", ret);
		goto fail;
	}

	ret = mipi_dsi_dcs_write_buffer(dsi, (u8[])
				     {0xB1, 0x7C, 0x00, 0x24, 0x06, 0x01, 
					  0x10, 0x10, 0x26, 0x2E, 0x1D, 0x1D, 0x57, 0x12, 0x01, 0xE6, 0xE2}, 0x11);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "failed to set SETPOWER (%d)\n", ret);
		goto fail;
	}

	ret = mipi_dsi_dcs_write_buffer(dsi, (u8[]){0xB2, 0x00, 0xC8, 0x04, 0x04, 0x00, 0x22}, 0x07);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "failed to set SETDISP (%d)\n", ret);
		goto fail;
	}


	ret = mipi_dsi_dcs_write(dsi, MIPI_DCS_EXIT_INVERT_MODE, NULL, 0);
		if (ret < 0) {
		DRM_DEV_ERROR(dev, "failed to set MIPI_DCS_EXIT_INVERT_MODE (%d)\n", ret);
		goto fail;
	}

	ret = mipi_dsi_dcs_write(dsi, MIPI_DCS_SET_ADDRESS_MODE, (u8[]){ 0x00 }, 1);
		if (ret < 0) {
		DRM_DEV_ERROR(dev, "failed to set MIPI_DCS_SET_ADDRESS_MODE (%d)\n", ret);
		goto fail;
	}

	ret = mipi_dsi_dcs_set_pixel_format(dsi, 0x70);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "failed to set pixel format (%d)\n", ret);
		goto fail;
	}

	ret = mipi_dsi_dcs_write_buffer(dsi, (u8[])
					{0xB4, 0x00, 0x04, 0x32, 0x10, 0x00, 0x32, 0x15, 0x05,
	 				0x32, 0x10, 0x08, 0x27, 0x01, 0x43, 0x03, 0x37, 0x01, 
					 0x43, 0x01, 0x61, 0x61, 0x01, 0x01, 0x43, 0x01, 0x43, 
					 0x01, 0x61, 0x61, 0x01}, 0x1F);
	if (ret < 0) {

		DRM_DEV_ERROR(dev, "failed to set SETCYC (%d)\n", ret);
		goto fail;
	}

	ret = mipi_dsi_dcs_write_buffer(dsi, (u8[]){0xBF, 0x06, 0x02, 0x10, 0x04}, 0x05);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "failed to set not open 1 (%d)\n", ret);
		goto fail;
	}

	ret = mipi_dsi_dcs_write_buffer(dsi, (u8[]){0xC7,0x00, 0x10, 0x00, 0x10}, 0x05);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "failed to set not open 2 (%d)\n", ret);
		goto fail;
	}

	ret = mipi_dsi_dcs_write_buffer(dsi, (u8[]){0xC6, 0x08, 0x08}, 0x03);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "failed to set not open 3 (%d)\n", ret);
		goto fail;
	}

	ret = mipi_dsi_dcs_write_buffer(dsi, (u8[]){0xC0, 0x0C, 0x15}, 0x03);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "failed to set not open 4 (%d)\n", ret);
		goto fail;
	}

	ret = mipi_dsi_dcs_write_buffer(dsi, (u8[]){
				 0xD5, 0x00, 0x00, 0x04, 0x00, 0x0A, 0x00, 0x01,
				 0x33, 0x00, 0x00, 0x33, 0x00, 0x10, 0x32, 0x54, 0x76, 0x10,
				 0x32, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x99, 0x99,
				 0x99, 0x54, 0x76, 0x88, 0x88}, 0x21);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "failed to set SETGIP (%d)\n", ret);
		goto fail;
	}

	ret = mipi_dsi_dcs_write_buffer(dsi, (u8[]){
		0xC1, 0x01, 0x00, 0x06, 0x0E, 0x16, 0x1E, 0x26, 0x2E, 0x36, 0x3E, 0x46, 0x4E,
		0x56, 0x5E, 0x66, 0x6E, 0x76, 0x7E, 0x86, 0x8E, 0x96, 0x9E, 0xA6, 0xAE, 0xB6, 
		0xBE, 0xC6, 0xCE, 0xD6, 0xDE, 0xE6, 0xEE, 0xF6, 0xFE, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0B, 0x13, 0x1B, 0x23, 0x2B, 0x33,
		0x3B, 0x43, 0x4B, 0x53, 0x5B, 0x63, 0x6B, 0x73, 0x7B, 0x83, 0x8B, 0x93, 0x9B,
		0xA3, 0xAB, 0xB3, 0xBB, 0xC3, 0xCB, 0xD3, 0xDB, 0xE3, 0xEB, 0xF3, 0xFB, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x10, 0x18,
		0x1E, 0x26, 0x2E, 0x36, 0x3E, 0x46, 0x4E, 0x56, 0x5E, 0x66, 0x6E, 0x76, 0x7E,
		0x86, 0x8E, 0x96, 0x9E, 0xA6, 0xAE, 0xB6, 0xBE, 0xC6, 0xCE, 0xD6, 0xDE, 0xE6, 
		0xED, 0xF5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0x80);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "failed to set SETDGC (%d)\n", ret);
		goto fail;
	}

	ret = mipi_dsi_dcs_write_buffer(dsi, (u8[]){
		0xE0, 0x00, 0x00, 0x00, 0x12, 0x1D, 0x34, 0x13, 0x2A, 0x04, 0x0C, 0x12, 0x16, 0x19, 
		0x16, 0x16, 0x0E, 0x11, 0x00, 0x00, 0x00, 0x18, 0x24, 0x3F, 0x16, 0x2C, 0x04, 0x0D,
		0x10, 0x16, 0x18, 0x16, 0x16, 0x10, 0x11, 0x07, 0x16, 0x06, 0x0F, 0x08, 0x16, 0x07, 0x10}, 0x2B);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "failed to set SETGAMMA (%d)\n", ret);
		goto fail;
	}

	ret = mipi_dsi_dcs_write_buffer(dsi, (u8[]){0xC9, 0x0F, 0x00, 0x1E, 0x1E, 0x00, 0x00, 0x00, 0x01, 0x3E}, 0x0A);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "failed to set SETCABC (%d)\n", ret);
		goto fail;
	}

	brightness = ortus->backlight->props.brightness;
	DRM_DEV_INFO(dev, "MIPI_DCS_SET_DISPLAY_BRIGHTNESS (%d)\n", brightness);
	ret = mipi_dsi_dcs_write(dsi, MIPI_DCS_SET_DISPLAY_BRIGHTNESS, (u8[]){ brightness }, 1);
		if (ret < 0) {
		DRM_DEV_ERROR(dev, "failed to set MIPI_DCS_SET_DISPLAY_BRIGHTNESS (%d)\n", ret);
		goto fail;
	}

	ret = mipi_dsi_dcs_write(dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY, (u8[]){ 0x00 }, 1);
		if (ret < 0) {
		DRM_DEV_ERROR(dev, "failed to set MIPI_DCS_WRITE_CONTROL_DISPLAY (%d)\n", ret);
		goto fail;
	}

	ret = mipi_dsi_dcs_write_buffer(dsi, (u8[]){0xBC, 0x07}, 0x02);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "failed to set SETVDC (%d)\n", ret);
		goto fail;
	}

	msleep(5);

	ret = mipi_dsi_dcs_write_buffer(dsi, (u8[]){0xCC, 0x09}, 0x02);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "failed to set SETVDC (%d)\n", ret);
		goto fail;
	}

	msleep(50);

	ret = mipi_dsi_dcs_write_buffer(dsi, (u8[]){0xBA, 0x03}, 0x02);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "failed to set SETMIPI (%d)\n", ret);
		goto fail;
	}

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "failed to set exit sleep mode (%d)\n", ret);
		goto fail;
	}

	msleep(200);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set display ON (%d)\n", ret);
		goto fail;
	}

	msleep(50);

	backlight_enable(ortus->backlight);

	ortus->enabled = true;

	return 0;

fail:

	gpiod_set_value(ortus->enable_gpio, 0);

	return ret;
}

static int ortus_panel_disable(struct drm_panel *panel)
{
	struct ortus_panel *ortus = to_ortus_panel(panel);
	struct mipi_dsi_device *dsi = ortus->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	if (!ortus->enabled)
		return 0;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;


	backlight_disable(ortus->backlight);



	usleep_range(10000, 15000);

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set display OFF (%d)\n", ret);
		return ret;
	}

	usleep_range(5000, 10000);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to enter sleep mode (%d)\n", ret);
		return ret;
	}

	ortus->enabled = false;

	return 0;
}

static int ortus_panel_get_modes(struct drm_panel *panel,
				 struct drm_connector *connector)
{
        struct ortus_panel *ortus =  to_ortus_panel(panel);
	struct drm_display_mode *mode;
	const struct drm_display_mode *display_mode;
	u32 bus_format = MEDIA_BUS_FMT_RGB888_1X24;
	// u32 *bus_flags = &connector->display_info.bus_flags;
	int ret;
	
	display_mode = &default_mode;
	mode = drm_mode_duplicate(connector->dev, display_mode);
	if (!mode) {
		dev_err(&ortus->dsi->dev, "failed to add mode %ux%ux@%u\n",
			display_mode->hdisplay, display_mode->vdisplay,
			drm_mode_vrefresh(display_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	ret = drm_display_info_set_bus_formats(&connector->display_info,
					       &bus_format, 1);
	if (ret)
		return ret;

	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = 62;
	connector->display_info.height_mm = 110;

	return 1;
}


static int ortus_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct ortus_panel *ortus = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	u16 brightness;
	int ret;

	if (!ortus->prepared)
		return 0;

	DRM_DEV_DEBUG_DRIVER(dev, "\n");

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness(dsi, &brightness);
	if (ret < 0)
		return ret;

	bl->props.brightness = brightness;

	return brightness & 0xff;
}

static int ortus_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct ortus_panel *ortus = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret = 0;

	if (!ortus->prepared)
		return 0;

	DRM_DEV_DEBUG_DRIVER(dev, "New brightness: %d\n", bl->props.brightness);

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness(dsi, bl->props.brightness);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct backlight_ops ortus_bl_ops = {
	.update_status = ortus_bl_update_status,
	.get_brightness = ortus_bl_get_brightness,
};

static const struct drm_panel_funcs ortus_panel_funcs = {
	.prepare = ortus_panel_prepare,
	.unprepare = ortus_panel_unprepare,
	.enable = ortus_panel_enable,
	.disable = ortus_panel_disable,
	.get_modes = ortus_panel_get_modes,
};



static int ortus_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *np = dev->of_node;
	struct ortus_panel *opanel;
	struct backlight_properties bl_props;
	int ret;
	u32 video_mode;

	DRM_DEV_INFO(dev, "ortus_panel_probe ++ \n");

	opanel = devm_kzalloc(&dsi->dev, sizeof(*opanel), GFP_KERNEL);
	if (!opanel)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, opanel);

	opanel->dsi = dsi;

	dsi->channel = 0x0;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags =  MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO 
				 | MIPI_DSI_CLOCK_NON_CONTINUOUS;


	ret = of_property_read_u32(np, "video-mode", &video_mode);
	if (!ret) {
		switch (video_mode) {
		case 0:
			/* burst mode */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_BURST;
			break;
		case 1:
			/* non-burst mode with sync event */
			break;
		case 2:
			/* non-burst mode with sync pulse */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
			break;
		default:
			dev_warn(dev, "invalid video mode %d\n", video_mode);
			break;

		}
	}

	ret = of_property_read_u32(np, "dsi-lanes", &dsi->lanes);
	if (ret < 0) {
		dev_err(dev, "Failed to get dsi-lanes property (%d)\n", ret);
		return ret;
	}


	opanel->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(opanel->enable_gpio)) {
		ret = PTR_ERR(opanel->enable_gpio);
		dev_err(dev, "cannot get enable-gpio %d\n", ret);
		return ret;
	}

	opanel->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(opanel->reset_gpio)) {
		ret = PTR_ERR(opanel->reset_gpio);
		dev_err(dev, "cannot get reset-gpios %d\n", ret);
		return ret;
	}


	memset(&bl_props, 0, sizeof(bl_props));
	bl_props.type = BACKLIGHT_RAW;
	bl_props.brightness = 255;
	bl_props.max_brightness = 255;

	opanel->backlight = devm_backlight_device_register(
				dev, dev_name(dev),
				dev, dsi,
				&ortus_bl_ops, &bl_props);
	if (IS_ERR(opanel->backlight)) {
		ret = PTR_ERR(opanel->backlight);
		dev_err(dev, "Failed to register backlight (%d)\n", ret);
		return ret;
	}

	drm_panel_init(&opanel->panel, dev, &ortus_panel_funcs,DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&opanel->base);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
	{	
		dev_err(dev, "failed to attach dsi to host\n");
		drm_panel_remove(&opanel->base);
	}

	return ret;
}

static int ortus_panel_remove(struct mipi_dsi_device *dsi)
{
	struct ortus_panel *ortus = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		DRM_DEV_ERROR(dev, "Failed to detach from host (%d)\n",
			ret);

	drm_panel_remove(&ortus->base);
	return 0;
}

static void ortus_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct ortus_panel *ortus = mipi_dsi_get_drvdata(dsi);
	ortus_panel_disable(&ortus->base);
	ortus_panel_unprepare(&ortus->base);
}

#ifdef CONFIG_PM
static int ortus_panel_suspend(struct device *dev)
{
	struct ortus_panel *ortus = dev_get_drvdata(dev);

	devm_gpiod_put(dev, ortus->reset_gpio);
	devm_gpiod_put(dev, ortus->enable_gpio);

	return 0;
}

static int ortus_panel_resume(struct device *dev)
{
	struct ortus_panel *ortus = dev_get_drvdata(dev);

	ortus->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	ortus->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_HIGH);


	return 1;
}

#endif

static const struct dev_pm_ops ortus_pm_ops = {
	SET_RUNTIME_PM_OPS(ortus_panel_suspend, ortus_panel_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(ortus_panel_suspend, ortus_panel_resume)
};

static const struct of_device_id ortus_of_match[] = {
	{ .compatible = "ortustech,COM48H4N22ULC", },
	{ }
};
MODULE_DEVICE_TABLE(of, ortus_of_match);

static struct mipi_dsi_driver ortus_panel_driver = {
	.driver = {
		.name = "panel-ortustech-COM48H4N22ULC",
		.of_match_table = ortus_of_match,
		.pm	= &ortus_pm_ops,
	},
	.probe = ortus_panel_probe,
	.remove = ortus_panel_remove,
	.shutdown = ortus_panel_shutdown,
};
module_mipi_dsi_driver(ortus_panel_driver);

MODULE_AUTHOR("Philipp Speeth <speeth@keith-koep.com>");
MODULE_DESCRIPTION("Ortustech COM48H4N22ULC HD");
MODULE_LICENSE("GPL v2");
