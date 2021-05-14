/*
 * ADP1650 LED Flash Driver
 *
 * Copyright 2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <linux/i2c/adp1650.h>

struct adp1650_chip {
	struct i2c_client *client;
	struct led_classdev cdev;
	struct adp1650_leds_platform_data *pdata;
	unsigned char iocfg;
	unsigned char current_set;
	bool use_enable;
};

static struct adp1650_leds_platform_data ad1650_default_pdata = {
	.timer_iocfg =	ADP1650_IOCFG_IO2_HIGH_IMP |
			ADP1650_IOCFG_IO1_TORCH |
			ADP1650_FL_TIMER_ms(500),

	.current_set =	ADP1650_I_FL_mA(900) |
			ADP1650_I_TOR_mA(100),

	.output_mode =	ADP1650_IL_PEAK_2A25 |
			ADP1650_STR_LV_EDGE |
			ADP1650_FREQ_FB_EN |
			ADP1650_OUTPUT_EN |
			ADP1650_STR_MODE_HW |
			ADP1650_STR_MODE_STBY,

	.control =	ADP1650_I_TX2_mA(400) |
			ADP1650_I_TX1_mA(400),

	.ad_mode =	ADP1650_DYN_OVP_EN |
			ADP1650_STR_POL_ACTIVE_HIGH |
			ADP1650_I_ILED_2mA75 |
			ADP1650_IL_DC_1A50 |
			ADP1650_IL_DC_EN,

	.batt_low =	ADP1650_CL_SOFT_EN |
			ADP1650_I_VB_LO_mA(400) |
			ADP1650_V_VB_LO_3V50,

	.gpio_enable = -1,

	.flash_stabilize = 0,
};

static inline int adp1650_write(struct i2c_client *client, u8 reg, u8 value)
{
	int ret = i2c_smbus_write_byte_data(client, reg, value);
	if (ret < 0)
		dev_err(&client->dev, "i2c write failed\n");

	return ret;
}

static int adp1650_read(struct i2c_client *client, u8 reg, u8 *buf)
{
	int ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "i2c read failed\n");
		return ret;
	}

	*buf = ret;

	return 0;
}

static int adp1650_get_fault_status(struct i2c_client *client)
{
	unsigned char fault;
	int ret = adp1650_read(client, ADP1650_REG_FAULT, &fault);
	if (ret < 0)
		return ret;

	dev_dbg(&client->dev, "FAULT = %X:\n%s%s%s%s%s%s%s%s\n", fault,
		fault & ADP1650_FL_OVP ? "FL_OVP\n" : "",
		fault & ADP1650_FL_SC ? "FL_SC\n" : "",
		fault & ADP1650_FL_OT ? "FL_OT\n" : "",
		fault & ADP1650_FL_TO ? "FL_TO\n" : "",
		fault & ADP1650_FL_TX1 ? "FL_TX1\n" : "",
		fault & ADP1650_FL_IO2 ? "FL_IO2\n" : "",
		fault & ADP1650_FL_IL ? "FL_IL\n" : "",
		fault & ADP1650_FL_IDC ? "FL_IDC\n" : "");

	return fault;
}

static int adp1650_setup(struct i2c_client *client)
{
	struct adp1650_chip *chip = i2c_get_clientdata(client);
	const struct adp1650_leds_platform_data *pdata = chip->pdata;
	int ret;

	ret = adp1650_write(client, ADP1650_REG_TIMER_IOCFG,
			    pdata->timer_iocfg);
	if (ret < 0)
		return ret;

	ret = adp1650_write(client, ADP1650_REG_CURRENT_SET,
			    pdata->current_set);
	if (ret < 0)
		return ret;

	ret = adp1650_write(client, ADP1650_REG_OUTPUT_MODE,
			    pdata->output_mode);
	if (ret < 0)
		return ret;

	ret = adp1650_write(client, ADP1650_REG_CONTROL, pdata->control);
	if (ret < 0)
		return ret;

	ret = adp1650_write(client, ADP1650_REG_AD_MODE, pdata->ad_mode);
	if (ret < 0)
		return ret;

	ret = adp1650_write(client, ADP1650_REG_BATT_LOW, pdata->batt_low);
	if (ret < 0)
		return ret;

	chip->iocfg = pdata->timer_iocfg;
	chip->current_set = pdata->current_set;

	adp1650_get_fault_status(client); /* Clear Fault Register */

	return 0;
}

static int adp1650_led_mode_set(struct i2c_client *client, int mode)
{
	struct adp1650_chip *chip = i2c_get_clientdata(client);
	const struct adp1650_leds_platform_data *pdata = chip->pdata;
	int ret;
	unsigned char output, iocfg, current_set;

#ifdef ADP1650_CHECK_AND_CLEAR_FAULT
	ret = adp1650_get_fault_status(client);
	if (ret < 0)
		return ret;
	else if (ret & (ADP1650_FL_OVP | ADP1650_FL_SC | ADP1650_FL_OT |
			ADP1650_FL_TO | ADP1650_FL_IL | ADP1650_FL_IDC))
		dev_err(&client->dev, "critical fault status (0x%X)\n", ret);
#endif

	iocfg = chip->iocfg;
	current_set = chip->current_set;

	switch (mode) {
	case FL_MODE_OFF:
		output = pdata->output_mode &
			~(ADP1650_OUTPUT_EN | ADP1650_LED_MODE_FLASH);
		break;
	case FL_MODE_TORCH_25mA...FL_MODE_TORCH_200mA:
		output = pdata->output_mode & ~(ADP1650_LED_MODE_FLASH);
		output |= ADP1650_OUTPUT_EN | ADP1650_LED_MODE_ASSIST_LIGHT;
		iocfg &= ~ADP1650_IOCFG_IO1_TORCH;
		current_set = (current_set & 0xF8) |
			(mode - FL_MODE_TORCH_25mA);
		break;
	case FL_MODE_TORCH_TRIG_EXT_25mA...FL_MODE_TORCH_TRIG_EXT_200mA:
		output = pdata->output_mode & ~(ADP1650_LED_MODE_FLASH);
		output |= ADP1650_OUTPUT_EN | ADP1650_STR_MODE_STBY;
		iocfg |= ADP1650_IOCFG_IO1_TORCH;
		current_set = (current_set & 0xF8) |
			(mode - FL_MODE_TORCH_TRIG_EXT_25mA);
		break;
	case FL_MODE_FLASH:
		output = pdata->output_mode &
			~(ADP1650_LED_MODE_FLASH | ADP1650_STR_MODE_HW);
		output |= ADP1650_OUTPUT_EN | ADP1650_LED_MODE_FLASH |
			ADP1650_STR_MODE_SW;
		break;
	case FL_MODE_FLASH_TRIG_EXT:
		output = pdata->output_mode & ~(ADP1650_LED_MODE_FLASH);
		output |= ADP1650_OUTPUT_EN | ADP1650_LED_MODE_FLASH |
			ADP1650_STR_MODE_HW;
		break;
	case FL_MODE_TORCH:
		output = pdata->output_mode & ~(ADP1650_LED_MODE_FLASH);
		output |= ADP1650_OUTPUT_EN | ADP1650_LED_MODE_ASSIST_LIGHT;
		iocfg &= ~ADP1650_IOCFG_IO1_TORCH;
		break;
	default:
		return -EINVAL;
	}

	if (current_set != chip->current_set) {
		ret = adp1650_write(client, ADP1650_REG_CURRENT_SET,
				    current_set);
		if (ret < 0)
			return ret;
		chip->current_set = current_set;
	}

	if (iocfg != chip->iocfg) {
		ret = adp1650_write(client, ADP1650_REG_TIMER_IOCFG, iocfg);
		if (ret < 0)
			return ret;
		chip->iocfg = iocfg;
	}

	ret = adp1650_write(client, ADP1650_REG_OUTPUT_MODE, output);
	if (ret < 0)
		return ret;

	if(mode == FL_MODE_FLASH && pdata->flash_stabilize)
		msleep(pdata->flash_stabilize);

	return 0;
}

static void adp1650_brightness_set(struct led_classdev *led_cdev,
						enum led_brightness brightness)
{
	struct adp1650_chip *chip = container_of(led_cdev,
						 struct adp1650_chip, cdev);

	adp1650_led_mode_set(chip->client, brightness);
}

static int adp1650_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct adp1650_chip *chip;
	int ret;
	int tmp;
	int retval;

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "i2c byte data not supported\n");
		return -EIO;
	}

	chip = kzalloc(sizeof(struct adp1650_chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	if (!client->dev.platform_data) {
		dev_warn(&client->dev, "pdata is not available, using default\n");
		chip->pdata = &ad1650_default_pdata;
	} else {
		chip->pdata = client->dev.platform_data;
	}


	//Get Devtree Entries
	retval = of_property_read_u32(client->dev.of_node, "flash-timeout-ms",
					&tmp);
	if (retval) {
		dev_err(&client->dev, "no flash-timeout-ms use default 500 ms\n");
		return retval;
	}
	else
	{
		dev_dbg(&client->dev, "flash-timeout-ms %d ms\n", tmp);
		chip->pdata->timer_iocfg =	ADP1650_IOCFG_IO2_HIGH_IMP |
									ADP1650_IOCFG_IO1_TORCH |
									ADP1650_FL_TIMER_ms(tmp);
	}
		

	retval = of_property_read_u32(client->dev.of_node, "flash-intensity-mA",
					&tmp);
	if (retval) {
		dev_err(&client->dev, "no flash-intensity-mA use default 900 mA \n");
		return retval;
	}
	else
	{
		dev_dbg(&client->dev, "flash-intensity-mA %d mA\n", tmp);
		chip->pdata->current_set =	ADP1650_I_FL_mA(tmp) |
									ADP1650_I_TOR_mA(100);
	}

	retval = of_property_read_u32(client->dev.of_node, "torch-intensity-mA",
					&tmp);
	if (retval) {
		dev_err(&client->dev, "no torch-intensity-mA use default 100 mA \n");
		return retval;
	}
	else
	{
		dev_dbg(&client->dev, "torch-intensity-mA %d mA\n", tmp);
		chip->pdata->current_set =	(chip->pdata->current_set & 0xF8) |
									ADP1650_I_TOR_mA(tmp);
	}

	retval = of_property_read_u32(client->dev.of_node, "flash-stabilize-ms",
					&tmp);
	if (retval) {
		dev_err(&client->dev, "no flash-stabilize-ms use default 0 ms \n");
		return retval;
	}
	else
	{
		dev_dbg(&client->dev, "flash-stabilize-ms %d ms\n", tmp);
		chip->pdata->flash_stabilize =	tmp;
	}

	


	chip->client = client;
	i2c_set_clientdata(client, chip);

	if (chip->pdata->setup) {
		ret = chip->pdata->setup(client, true);
		if (ret < 0) {
			dev_err(&client->dev, "setup callback failed!\n");
			goto err_free_mem;
		}
	}

	/* request enable pin */
	chip->pdata->gpio_enable = of_get_named_gpio(client->dev.of_node, "enable-gpios", 0);
	ret = gpio_is_valid(chip->pdata->gpio_enable);
	if (ret) {
		retval = devm_gpio_request_one(&client->dev, chip->pdata->gpio_enable, GPIOF_OUT_INIT_HIGH,
						"adp1650_enable");
		if (retval < 0) {
			dev_warn(&client->dev, "Failed to set reset pin\n");
			goto err_setup;
		}
		chip->use_enable = true;
	}

	ret = adp1650_setup(chip->client);
	if (ret < 0) {
		dev_err(&client->dev, "device setup failed %d\n", ret);
		goto err_release_gpio;
	}

	chip->cdev.name = client->name;
	chip->cdev.brightness = FL_MODE_OFF;
	chip->cdev.brightness_set = adp1650_brightness_set;

	ret = led_classdev_register(&client->dev, &chip->cdev);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register led");
		goto err_release_gpio;
	}

	return 0;

err_release_gpio:
	if (chip->use_enable)
		gpio_free(chip->pdata->gpio_enable);
err_setup:
	if (chip->pdata->setup)
		chip->pdata->setup(client, true);
err_free_mem:
	kfree(chip);

	return ret;
}

static int adp1650_remove(struct i2c_client *client)
{
	struct adp1650_chip *chip = i2c_get_clientdata(client);

	led_classdev_unregister(&chip->cdev);
	adp1650_led_mode_set(client, FL_MODE_OFF);

	if (chip->use_enable)
		gpio_free(chip->pdata->gpio_enable);

	if (chip->pdata->setup)
		chip->pdata->setup(client, false);

	kfree(chip);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int adp1650_suspend(struct device *dev)
{
	struct adp1650_chip *chip = dev_get_drvdata(dev);
	int ret;

	if (chip->use_enable)
		gpio_set_value_cansleep(chip->pdata->gpio_enable, 0);
	else
		adp1650_led_mode_set(chip->client, FL_MODE_OFF);

	if (chip->pdata->setup) {
		ret = chip->pdata->setup(chip->client, false);
		if (ret) {
			dev_err(dev, "setup failed\n");
			return ret;
		}
	}
	return 0;
}

static int adp1650_resume(struct device *dev)
{
	struct adp1650_chip *chip = dev_get_drvdata(dev);
	int ret;

	if (chip->pdata->setup) {
		ret = chip->pdata->setup(chip->client, true);
		if (ret) {
			dev_err(dev, "setup failed\n");
			return ret;
		}
	}

	adp1650_setup(chip->client);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(adp1650_pm_ops, adp1650_suspend, adp1650_resume);

static const struct i2c_device_id adp1650_id[] = {
	{"adp1650", 0},
	{ },
};
MODULE_DEVICE_TABLE(i2c, adp1650_id);

static struct i2c_driver adp1650_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.pm = &adp1650_pm_ops,
		.owner = THIS_MODULE,
	},
	.probe = adp1650_probe,
	.remove = adp1650_remove,
	.id_table = adp1650_id,
};

static int __init adp1650_init(void)
{
	return i2c_add_driver(&adp1650_driver);
}
module_init(adp1650_init);

static void __exit adp1650_exit(void)
{
	i2c_del_driver(&adp1650_driver);
}
module_exit(adp1650_exit);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("ADP1650 LED Flash Driver");
MODULE_LICENSE("GPL v2");
