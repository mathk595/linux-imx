/*
 * Keith & Koep FPGA DSI to RGB bridge driver.
 *
 * Copyright (c) 2019, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio/consumer.h>

static int tr8fpga_i2c_read(struct i2c_client *client, char *writebuf,
			   int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};

		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c read error.\n", __func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};

		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}

	return ret;
}

static int tr8fpga_i2c_write(struct i2c_client *client, char *writebuf,
			    int writelen)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = writelen,
			.buf = writebuf,
		},
	};
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s: i2c write error.\n", __func__);

	return ret;
}

static int tr8fpga_write_reg(struct i2c_client *client, u8 addr, const u8 val)
{
	u8 buf[2] = {0};

	buf[0] = addr;
	buf[1] = val;

	return tr8fpga_i2c_write(client, buf, sizeof(buf));
}

static int tr8fpga_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct property *prop;
	int err;
	int i, size;
	struct device_node *np = client->dev.of_node;
	int addresses[100];
	int values[100];
	char address, value;
	struct gpio_desc *enable_gpio;

	enable_gpio = devm_gpiod_get_optional(&client->dev, "enable", GPIOD_OUT_HIGH);
	if (enable_gpio)
		gpiod_set_value_cansleep(enable_gpio, 1);

	address = (char)0;
	err = tr8fpga_i2c_read(client, &address, 1, &value, 1);
	if (err < 0) {
		dev_err(&client->dev, "failed to read chip id\n");
		return err;
	}
	if (value != 0x61) {
		dev_err(&client->dev, "chip id is not correct\n");
		return err;
	}

	prop = of_find_property(np, "tr8fpga,addresses", NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	size = prop->length / sizeof(int);

	err = of_property_read_u32_array(np, "tr8fpga,addresses", addresses, size);
	if (err && (err != -EINVAL)) {
		dev_err(&client->dev, "Unable to read 'tr8fpga,addresses'\n");
		return err;
	}

	prop = of_find_property(np, "tr8fpga,values", NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	i = prop->length / sizeof(u32);
	if (i != size) {
		dev_err(&client->dev, "invalid 'tr8fpga,values' length should be same as addresses\n");
		return -EINVAL;
	}

	err = of_property_read_u32_array(np, "tr8fpga,values", values, i);
	if (err && (err != -EINVAL)) {
		dev_err(&client->dev, "Unable to read 'tr8fpga,values'\n");
		return err;
	}

	for (i = 0; i < size; i++)
	{
	        printk(KERN_ERR"tr8fpga_probe: tr8fpga_write_reg 0x%x 0x%x\n",addresses[i], values[i]);
	  
		tr8fpga_write_reg(client, addresses[i], values[i]);
		if (err < 0) {
			dev_err(&client->dev, "failed to write data to the chip\n");
			return err;
		}
	}


	return 0;
}

static int tr8fpga_remove(struct i2c_client *client)
{
	struct gpio_desc *enable_gpio;

	enable_gpio = devm_gpiod_get_optional(&client->dev, "enable", GPIOD_OUT_LOW);
	if (enable_gpio)
		gpiod_set_value_cansleep(enable_gpio, 0);

	return 0;
}

static const struct i2c_device_id tr8fpga_id[] = {
	{"tr8fpga", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, tr8fpga_id);

static struct of_device_id tr8fpga_match_table[] = {
	{ .compatible = "kuk,tr8fpga",},
	{ },
};

static struct i2c_driver tr8fpga_i2c_driver = {
	.probe = tr8fpga_probe,
	.remove = tr8fpga_remove,
	.driver = {
		.name = "tr8fpga",
		.owner = THIS_MODULE,
		.of_match_table = tr8fpga_match_table,
	},
	.id_table = tr8fpga_id,
};

static int __init tr8fpga_init(void)
{
	return i2c_add_driver(&tr8fpga_i2c_driver);
}

static void __exit tr8fpga_exit(void)
{
	i2c_del_driver(&tr8fpga_i2c_driver);
}

module_init(tr8fpga_init);
module_exit(tr8fpga_exit);

MODULE_DESCRIPTION("KuK tr8fpga DSI to RGB bridge driver");
MODULE_LICENSE("GPL v2");

