/*
 *  Generic DT helper functions for touchscreen gpio configuration
 *
 *  Copyright (c) 2016 Jonas Höppner <jonas.hoeppner@garz-fricke.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */
#ifndef _TOUCH_GPIO_H_
#define _TOUCH_GPIO_H_

#include <linux/types.h>
#include <linux/version.h>

struct device;
//struct touch_gpio;
struct touch_gpio{
	struct device * dev;
	int reset_pin;
	int wake_pin;
	int irq_pin;
	// 0 activ_high, !=0: active_low z.B. GPIOF_ACTIVE_LOW OF_GPIO_ACTIVE_LOW
	int reset_low_active;
	int wake_low_active;
	int irq_low_active;
	int irq_flags;

	int usage;	// Which gpios are unused, which are optional, which are mandatory
};

#define ACTIVE_HIGH 0
#define ACTIVE_LOW GPIOF_ACTIVE_LOW

enum {
	reset_unused	= 1<<0,
	reset_mandatory = 2<<0,
	reset_optional  = 3<<0,
	wake_unused     = 1<<2,
	wake_mandatory  = 2<<2,
	wake_optional   = 3<<2,
	irq_unused      = 1<<4,
	irq_mandatory   = 2<<4,
	irq_optional    = 3<<4,
	export_reset    = 1<<6,
	export_irq      = 1<<7,
	export_wake     = 1<<8,
};


/* Setup the touch gpio interface from pdata,  */
struct touch_gpio * touch_gpio_set_from_pdata( struct device *dev, int usage, 
	int  reset_gpio, int  reset_low_active, int wake_gpio, int wake_low_active, int irq_gpio, int irq_low_active);
/* Setup the touch gpio interface from dt */
struct touch_gpio * touch_gpio_read_from_dt( struct device *dev, int  usage);

void touch_gpio_free(struct touch_gpio * gpios);

/* True: activate reset, False: release reset */
void touch_gpio_reset_out ( const struct touch_gpio * gpios, bool state );

/* Set the reset for udelay usecs*/
void touch_gpio_do_reset ( const struct touch_gpio * gpios, int udelay );
/* True: activate wake, False: release wake */
void touch_gpio_wake_out ( const struct touch_gpio * gpios, bool state );
/* Returns true if irq pin is active, false if it is not active */
bool touch_gpio_get_irq_state ( const struct touch_gpio * gpios);
/* Returns irq flags if configure in devivetree*/
int touch_gpio_get_irq_flags ( const struct touch_gpio * gpios);

void touch_gpio_irq_direction_output ( const struct touch_gpio * gpios, bool initial_state);
void touch_gpio_irq_direction_input ( const struct touch_gpio * gpios);
/* True: activate irq, False: release irq */
void touch_gpio_irq_out ( const struct touch_gpio * gpios, bool state );
#endif

/* gpio_sysfs_set_active_low has been deprecated in linux-imx_4.9.x, so we have
 * declare it here */
#if LINUX_VERSION_CODE > KERNEL_VERSION(4,1,15)
static inline int gpio_sysfs_set_active_low(unsigned gpio, int value)
{
	/* GPIO can never have been requested */
	WARN_ON(1);
	return -EINVAL;
}
#endif
