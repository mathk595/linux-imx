/*
 * Touch Screen driver for SiS 9200 family I2C Touch panels
 *
 * Copyright (C) 2015 SiS, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/irq.h>
#include <asm/unaligned.h>
#include <linux/input/mt.h>
#include <linux/crc-itu-t.h>
#include <linux/delay.h>
#include "touch_gpio.h"

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif

#define SIS_I2C_NAME					"sisi2c"
#define MAX_FINGERS						10

/*Resolution mode*/
/*Constant value*/
#define SIS_MAX_X						4095
#define SIS_MAX_Y						4095

#define PACKET_BUFFER_SIZE				128

/* for new i2c format */
#define TOUCHDOWN						0x3
#define TOUCHUP							0x0
#define MAX_BYTE						64
#define PRESSURE_MAX					255

/*Resolution diagonal */
#define AREA_LENGTH_LONGER				5792
/*((SIS_MAX_X^2) + (SIS_MAX_Y^2))^0.5*/
#define AREA_LENGTH_SHORT				5792
#define AREA_UNIT						(5792/32)

#define P_BYTECOUNT						0
#define ALL_IN_ONE_PACKAGE				0x10
#define IS_TOUCH(x)						(x & 0x1)
#define IS_HIDI2C(x)					((x & 0xF) == 0x06)
#define IS_AREA(x)						((x >> 4) & 0x1)
#define IS_PRESSURE(x)				    ((x >> 5) & 0x1)
#define IS_SCANTIME(x)			        ((x >> 6) & 0x1)
#define NORMAL_LEN_PER_POINT			6
#define AREA_LEN_PER_POINT				2
#define PRESSURE_LEN_PER_POINT			1

#define TOUCH_FORMAT					0x1
#define HIDI2C_FORMAT					0x6
#define P_REPORT_ID						2
#define BYTE_BYTECOUNT					2
#define BYTE_ReportID					1
#define BYTE_CRC_HIDI2C					0
#define BYTE_CRC_I2C					2
#define BYTE_SCANTIME					2

#define MAX_SLOTS						15

struct sis_slot {
	int check_id;
	int id;
	unsigned short x, y;
	u16 pressure;
	u16 width;
	u16 height;
};

struct point {
	int id;
	unsigned short x, y;
	u16 pressure;
	u16 width;
	u16 height;
};

struct sistp_driver_data {
	int id;
	int fingers;
	struct point pt[MAX_FINGERS];
};

struct sis_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct sistp_driver_data *tp_info;
	struct touch_gpio * gpios;
};

static void sis_ts_reset(struct i2c_client *client,
						struct sis_ts_data *ts)
{
	/* GuF variant: We do not control the RESET pin of the SiS9255, instead we
	                control the VDD active low. That means, we have to take care
	                about POR time 5ms, p. 14. */

	touch_gpio_reset_out(ts->gpios, true);
	msleep(5);
	touch_gpio_reset_out(ts->gpios, false);
	mdelay(5); /* POR reset time */
}

/* Addresses to scan */
static void sis_tpinfo_clear(struct sistp_driver_data *tp_info, int max);

static int sis_cul_unit(u8 report_id)
{
	int ret = NORMAL_LEN_PER_POINT;

	if (report_id != ALL_IN_ONE_PACKAGE) {
		if (IS_AREA(report_id))
			ret += AREA_LEN_PER_POINT;
		if (IS_PRESSURE(report_id))
			ret += PRESSURE_LEN_PER_POINT;
	}

	return ret;
}

/*
static int sis_writepacket(struct i2c_client *client, u32 len, u8 *buf)
{
    struct i2c_msg msg[1];

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].len = len;
    msg[0].buf = (unsigned char *)buf;

    return i2c_transfer(client->adapter, msg, 1);
}
*/

static int sis_readpacket(struct i2c_client *client, u8 *buf)
{
	u8 tmpbuf[MAX_BYTE] = {0};
	int ret;
	int touchnum = 0;
	int p_count = 0;
	int touch_format_id = 0;
	int locate = 0;
	bool read_first = true;
	/*
	 * New i2c format
	 * buf[0] = Low 8 bits of byte count value
	 * buf[1] = High 8 bits of byte count value
	 * buf[2] = Report ID
	 * buf[touch num * 6 + 2 ] = Touch information;
	 * 1 touch point has 6 bytes, it could be none if no touch
	 * buf[touch num * 6 + 3] = Touch numbers
	 *
	 * One touch point information include 6 bytes, the order is
	 *
	 * 1. status = touch down or touch up
	 * 2. id = finger id
	 * 3. x axis low 8 bits
	 * 4. x axis high 8 bits
	 * 5. y axis low 8 bits
	 * 6. y axis high 8 bits
	 */
	do {
		if (locate >= PACKET_BUFFER_SIZE) {
			dev_err(&client->dev, "%s: Buf Overflow\n", __func__);
			return -EPERM;
		}
		ret = i2c_master_recv(client, tmpbuf, MAX_BYTE);

		if (ret < 0) {
			dev_err(&client->dev, "%s: i2c transfer error\n", __func__);
			return ret;
		}
		/* error package length of receiving data */
		else if (tmpbuf[P_BYTECOUNT] > MAX_BYTE) {
			dev_err(&client->dev, "%s: Error Bytecount\n", __func__);
			return -EPERM;
		}
		if (read_first) {
			/* access NO TOUCH event unless BUTTON NO TOUCH event */
			if (tmpbuf[P_BYTECOUNT] == 0)
				return 0;	/* touchnum is 0 */
		}
		
		/*
		 * skip parsing data when two devices are registered
		 * at the same slave address
		 * parsing data when P_REPORT_ID && 0xf is TOUCH_FORMAT
		 * or P_REPORT_ID is ALL_IN_ONE_PACKAGE
		 */
		touch_format_id = tmpbuf[P_REPORT_ID] & 0xf;
		if ((touch_format_id != TOUCH_FORMAT) &&
			(touch_format_id != HIDI2C_FORMAT) &&
			(tmpbuf[P_REPORT_ID] != ALL_IN_ONE_PACKAGE)) {
			dev_err(&client->dev, "sis_readpacket: Error Report_ID\n");
			return -EPERM;
		}
		p_count = (int) tmpbuf[P_BYTECOUNT] - 1;	/* start from 0 */
		if (tmpbuf[P_REPORT_ID] != ALL_IN_ONE_PACKAGE) {
			if (IS_TOUCH(tmpbuf[P_REPORT_ID])) {
				/* delete 2 byte crc */
				p_count -= BYTE_CRC_I2C;
			} else if (IS_HIDI2C(tmpbuf[P_REPORT_ID])) {
				p_count -= BYTE_CRC_HIDI2C;
			} else {	/* should not be happen */
				dev_err(&client->dev, "%s: delete crc error\n", __func__);
				return -EPERM;
			}
			if (IS_SCANTIME(tmpbuf[P_REPORT_ID]))
				p_count -= BYTE_SCANTIME;
		}
		/* For ALL_IN_ONE_PACKAGE */
		if (read_first) {
			touchnum = tmpbuf[p_count];
		} else {
			if (tmpbuf[p_count] != 0) {
				dev_err(&client->dev, "%s: get error package\n", __func__);
				return -EPERM;
			}
		}

		if ((touch_format_id != HIDI2C_FORMAT) &&
			(tmpbuf[P_BYTECOUNT] > 3)) {
			int crc_end = p_count + (IS_SCANTIME(
				tmpbuf[P_REPORT_ID]) * 2);
			u16 buf_crc = crc_itu_t(
				0, tmpbuf + 2, crc_end - 1);
			int l_package_crc = (IS_SCANTIME(
				tmpbuf[P_REPORT_ID]) * 2) + p_count + 1;
			u16 package_crc = get_unaligned_le16(
				&tmpbuf[l_package_crc]);

			if (buf_crc != package_crc) {
				dev_err(&client->dev, "%s: CRC Error\n", __func__);
				return -EPERM;
			}
		}

		memcpy(&buf[locate], &tmpbuf[0], MAX_BYTE);
		/* Buf_Data [0~63] [64~128] */
		locate += MAX_BYTE;
		read_first = false;
	} while (tmpbuf[P_REPORT_ID] != ALL_IN_ONE_PACKAGE &&
			tmpbuf[p_count] > 5);
	return touchnum;
}

static int sis_parse_i2c_event(u8 *buf, u8 fingers,
			       struct sistp_driver_data *tp_info,
			       int *check_id, struct sis_slot *sisdata)
{
	int point_unit;
	int slot;
	u8 i = 0, j, pstatus = 0;
	u8 px = 0, py = 0;
	u8 p_area = 0;
	u8 p_preasure = 0;
	//printk("%s: fingers=%d\n",__FUNCTION__,(int)fingers);
	/* Parser and Get the sis data */
	point_unit = sis_cul_unit(buf[P_REPORT_ID]);
	tp_info->fingers = fingers = (fingers > MAX_FINGERS ? 0 : fingers);

	/* fingers 10 =  0 ~ 9 */
	for (i = 0; i < fingers; i++) {
		if ((buf[P_REPORT_ID] != ALL_IN_ONE_PACKAGE) && (i >= 5)) {
			/*Calc point status*/
			pstatus = BYTE_BYTECOUNT + BYTE_ReportID
					+ ((i - 5) * point_unit);
			pstatus += 64;
		} else {
			pstatus = BYTE_BYTECOUNT + BYTE_ReportID
					+ (i * point_unit);
					/* Calc point status */
		}
		px = pstatus + 2;	/* Calc point x_coord */
		py = px + 2;	/* Calc point y_coord */
		if ((buf[pstatus]) == TOUCHUP) {
			tp_info->pt[i].width = 0;
			tp_info->pt[i].height = 0;
			tp_info->pt[i].pressure = 0;
		} else if (buf[P_REPORT_ID] == ALL_IN_ONE_PACKAGE
					&& (buf[pstatus]) == TOUCHDOWN) {
			tp_info->pt[i].width = 1;
			tp_info->pt[i].height = 1;
			tp_info->pt[i].pressure = 1;
		} else if ((buf[pstatus]) == TOUCHDOWN) {
			p_area = py + 2;
			p_preasure = py + 2 + (IS_AREA(buf[P_REPORT_ID]) * 2);
			/* area */
			if (IS_AREA(buf[P_REPORT_ID])) {
				tp_info->pt[i].width = buf[p_area];
				tp_info->pt[i].height = buf[p_area + 1];
			} else {
				tp_info->pt[i].width = 1;
				tp_info->pt[i].height = 1;
			}
			/* pressure */
			if (IS_PRESSURE(buf[P_REPORT_ID]))
				tp_info->pt[i].pressure = (buf[p_preasure]);
			else
				tp_info->pt[i].pressure = 1;
		} else
			return -EPERM;

		tp_info->pt[i].id = (buf[pstatus + 1]);
		tp_info->pt[i].x = get_unaligned_le16(&buf[px]);
		tp_info->pt[i].y = get_unaligned_le16(&buf[py]);

		/* Find the slot matching the id, 
		   was something like: int slot = tp_info->pt[i].id, 
		   but the id happend to be higher then the array size
		   */
		slot = -1;
		for(j  = 0; j < MAX_FINGERS; j++)
		{
			if( sisdata[j].id == tp_info->pt[i].id){
				slot = j;
				break;
			}
			if( sisdata[j].id == -1 && slot == -1)
				slot = j;
		}
		if( slot == -1) pr_err("Failed to find a free slot for tp %d\n",i);

		check_id[slot] = 1;
		sisdata[slot].id = tp_info->pt[i].id;
		sisdata[slot].pressure = tp_info->pt[i].pressure;
		sisdata[slot].x = tp_info->pt[i].x;
		sisdata[slot].y = tp_info->pt[i].y;
		sisdata[slot].width = tp_info->pt[i].width
						* AREA_UNIT;
		sisdata[slot].height = tp_info->pt[i].height
						* AREA_UNIT;
	}
	return 0;
}

static irqreturn_t sis_ts_irq_handler(int irq, void *dev_id)
{
	struct sis_ts_data *ts = dev_id;
	struct sistp_driver_data *tp_info = ts->tp_info;
	int ret;
	u8 buf[PACKET_BUFFER_SIZE] = {0};
	u8 i = 0, fingers = 0;
	int check_id[MAX_FINGERS];
	static int pre_check_id[MAX_FINGERS];
	static struct sis_slot sisdata[MAX_FINGERS] = {{ .id=-1}, { .id=-1},{ .id=-1},{ .id=-1},{ .id=-1},{ .id=-1},{ .id=-1},{ .id=-1},{ .id=-1},{ .id=-1} };
	memset(check_id, 0, sizeof(int)*MAX_FINGERS);

	//printk("%s\n",__FUNCTION__);
	/* I2C or SMBUS block data read */
	ret = sis_readpacket(ts->client, buf);
	if (ret < 0)
		goto out;
	/* Access NO TOUCH event unless BUTTON NO TOUCH event */
	else if (ret == 0) {
		fingers = 0;
		sis_tpinfo_clear(tp_info, MAX_FINGERS);
		goto type_b_report;
	}

	sis_tpinfo_clear(tp_info, MAX_FINGERS);
	fingers = ret;

	ret = sis_parse_i2c_event(buf, fingers, tp_info, check_id, sisdata);
	if (ret < 0)
		goto out;

type_b_report:

	for (i = 0; i < MAX_FINGERS; i++) {
		if (check_id[i] == 1) {
			if( i == 0){
				input_report_abs(ts->input_dev,
					BTN_TOUCH, 1);
				input_report_abs(ts->input_dev,
					ABS_X, sisdata[i].x);
				input_report_abs(ts->input_dev,
					ABS_Y, sisdata[i].y);
				input_report_abs(ts->input_dev,
					ABS_PRESSURE, sisdata[i].pressure);
			}
			input_mt_slot(ts->input_dev, i);
			input_report_abs(ts->input_dev,
				ABS_MT_TRACKING_ID, sisdata[i].id);

			input_report_abs(ts->input_dev,
				ABS_MT_PRESSURE, sisdata[i].pressure);
			input_report_abs(ts->input_dev,
				ABS_MT_POSITION_X, sisdata[i].x);
			input_report_abs(ts->input_dev,
				ABS_MT_POSITION_Y, sisdata[i].y);
		} else if (pre_check_id[i] == 1) { // Was pressed last time, but not anymode -> send a release
			if( i == 0){
				input_report_abs(ts->input_dev,
					BTN_TOUCH, 0);
				input_report_abs(ts->input_dev,
					ABS_PRESSURE, 0);
			}
			input_mt_slot(ts->input_dev, i);
			input_report_abs(ts->input_dev,
				ABS_MT_PRESSURE, 0);
			input_report_abs(ts->input_dev,
				ABS_MT_TRACKING_ID, -1);
			sisdata[i].id = -1;
		}
		pre_check_id[i] = check_id[i];
	}

	input_sync(ts->input_dev);

out:
	usleep_range(10000, 20000);
	return IRQ_HANDLED;
}

static void sis_tpinfo_clear(struct sistp_driver_data *tp_info, int max)
{
	int i = 0;

	for (i = 0; i < max; i++) {
		tp_info->pt[i].id = -1;
		tp_info->pt[i].x = 0;
		tp_info->pt[i].y = 0;
		tp_info->pt[i].pressure = 0;
		tp_info->pt[i].width = 0;
	}
	tp_info->id = 0x0;
	tp_info->fingers = 0;
}


static int sis_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct sis_ts_data *ts;

	printk("%s\n",__FUNCTION__);
	dev_dbg(&client->dev, "%s\n", __func__);
	ts = devm_kzalloc(&client->dev, sizeof(struct sis_ts_data),
			  GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->tp_info = devm_kzalloc(&client->dev,
				   sizeof(struct sistp_driver_data),
				   GFP_KERNEL);
	if (!ts->tp_info) {
		dev_err(&client->dev, "%s: Failed to allocate memory\n", __func__);
		return -ENOMEM;
	}
	
	ts->gpios = touch_gpio_read_from_dt(&client->dev, 
			reset_mandatory | wake_optional | irq_mandatory);
	if (IS_ERR(ts->gpios)){
			dev_err(&client->dev, "Probe gpios from dt and from platform data failed\n");
			err= PTR_ERR(ts->gpios);
			goto err_touch_gpio_failed;
	}
	sis_ts_reset(client, ts);

	/* 1. Init necessary buffers */
	ts->client = client;
	i2c_set_clientdata(client, ts);

	/* 2. Allocate input device */
	ts->input_dev = devm_input_allocate_device(&client->dev);
	if (!ts->input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "%s: Failed to allocate input device\n", __func__);
		goto err_input_dev_alloc_failed;
	}

	ts->input_dev->name = "sis_touch";

	set_bit(EV_ABS, ts->input_dev->evbit);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, SIS_MAX_X, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, SIS_MAX_Y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, PRESSURE_MAX, 0, 0);
        /* This also adds ABS_X, ABS_Y and ABS_PRESSURE events to the device */
	input_mt_init_slots(ts->input_dev, MAX_SLOTS, INPUT_MT_DIRECT);

	/* 3. Register input device to core */
	err = input_register_device(ts->input_dev);
	if (err) {
		dev_err(&client->dev,
			"%s: Unable to register %s input device\n", __func__,
			ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	/* 4. irq setup, polarity is configured in devicetree */
	err = devm_request_threaded_irq(&client->dev, client->irq, NULL,
									sis_ts_irq_handler,
									IRQF_ONESHOT | IRQF_TRIGGER_RISING,
									client->name,
					ts);
	if (err < 0) {
		dev_err(&client->dev, " %s: Failed to request touchscreen IRQ\n", __func__);
		goto err_request_threaded_irq;	
	}
	printk("%s: irq=%d, irq_pin=%d, irq_low_active=%d\n", __FUNCTION__,
		   client->irq, ts->gpios->irq_pin, ts->gpios->irq_low_active);
	printk("%s: OK\n",__FUNCTION__);
	return 0;
err_request_threaded_irq:
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
	touch_gpio_free(ts->gpios);
err_touch_gpio_failed:
	kfree(ts->tp_info);
	kfree(ts);
	printk("%s: err=%d\n",__FUNCTION__,err);
	return err;
}

static int sis_ts_remove(struct i2c_client *client) {
	struct sis_ts_data *ts = i2c_get_clientdata(client);

	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
	touch_gpio_free(ts->gpios);
	kfree(ts->tp_info);
	kfree(ts);
	return 0;
}

static int __maybe_unused sis_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
 
	disable_irq(client->irq);
	return 0;
}

static int __maybe_unused sis_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	enable_irq(client->irq);
	return 0;
}

static SIMPLE_DEV_PM_OPS(sis_ts_pm, sis_ts_suspend, sis_ts_resume);

static struct of_device_id sis_ts_of_match[] = {
	{ .compatible = "sis,sisi2c", },
	{ },
};
MODULE_DEVICE_TABLE(of, sis_ts_of_match);

static const struct i2c_device_id sis_ts_id[] = {
	{ SIS_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sis_ts_id);

static struct i2c_driver sis_ts_driver = {
	.driver = {
		.name = SIS_I2C_NAME,
		.owner = THIS_MODULE,
		.pm = &sis_ts_pm,
                .of_match_table = sis_ts_of_match,
	},
	.probe		= sis_ts_probe,
	.remove		= sis_ts_remove,
	.id_table	= sis_ts_id,
};
module_i2c_driver(sis_ts_driver);

MODULE_DESCRIPTION("SiS 9200 Family Touchscreen Driver"); 
MODULE_LICENSE("GPL v2");
