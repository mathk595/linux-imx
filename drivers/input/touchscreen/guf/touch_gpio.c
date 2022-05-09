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
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include "touch_gpio.h"
/*
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
*/

static int request_gpios(struct touch_gpio * gpios)
{
	struct device *dev = gpios->dev;
	int error = 0;
	const int usage = gpios->usage;

	if (gpio_is_valid(gpios->reset_pin)) {
		// request the gpio, leave it in inactive state
		error = devm_gpio_request_one(dev, gpios->reset_pin, 
			0 == gpios->reset_low_active ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW,
			dev_name(dev));
		if (error) {
			dev_err(dev, "Failed to request GPIO %d as reset pin, error %d\n", gpios->reset_pin, error);
			gpios->reset_pin = -EINVAL;
			goto out;
		}
		if (usage & export_reset)
		{
			error = gpio_export(gpios->reset_pin, false);
			if (error)
			{
				dev_err(dev, "Failed to export reset pin, error %d\n", error);
				goto out;
			}
			error = gpio_export_link(dev, "reset", gpios->reset_pin);
			if (error)
				dev_warn(dev, "Unable to link exported reset pin, error %d\n", error);
			error = gpio_sysfs_set_active_low(gpios->reset_pin, gpios->reset_low_active);
			if (error)
				dev_warn(dev, "Unable to set active-low state of reset pin, error %d\n", error);
		}
	}
	if (gpio_is_valid(gpios->wake_pin)) {
		// request the gpio, leave it in inactive state
		error = devm_gpio_request_one(dev, gpios->wake_pin, 
			0 == gpios->wake_low_active ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW,
			dev_name(dev));
		if (error) {
			dev_err(dev, "Failed to request GPIO %d as wake pin, error %d\n", gpios->wake_pin, error);
			gpios->wake_pin = -EINVAL;
			goto out;
		}
		if (usage & export_wake)
		{
			error = gpio_export(gpios->wake_pin, false);
			if (error)
			{
				dev_err(dev, "Failed to export wake pin, error %d\n", error);
				goto out;
			}
			error = gpio_export_link(dev, "wake", gpios->wake_pin);
			if (error)
				dev_warn(dev, "Unable to link exported wake pin, error %d\n", error);
			error = gpio_sysfs_set_active_low(gpios->wake_pin, gpios->wake_low_active);
			if (error)
				dev_warn(dev, "Unable to set active-low state of wake pin, error %d\n", error);
		}
	}
	if (gpio_is_valid(gpios->irq_pin)) {
		// request the gpio
		error = devm_gpio_request_one(dev, gpios->irq_pin, GPIOF_IN, dev_name(dev));
		if (error) {
			dev_err(dev, "Failed to request GPIO %d as irq pin, error %d\n", gpios->irq_pin, error);
			gpios->irq_pin = -EINVAL;
			goto out;
		}
		if (usage & export_irq)
		{
			error = gpio_export(gpios->irq_pin, false);
			if (error)
			{
				dev_err(dev, "Failed to export irq pin, error %d\n", error);
				goto out;
			}
			error = gpio_export_link(dev, "irq", gpios->irq_pin);
			if (error)
				dev_warn(dev, "Unable to link exported irq pin, error %d\n", error);
			error = gpio_sysfs_set_active_low(gpios->irq_pin, gpios->irq_low_active);
			if (error)
				dev_warn(dev, "Unable to set active-low state of irq pin, error %d\n", error);
		}
	}
out:
	return error;
}

static struct touch_gpio * touch_gpio_allocate(struct device *dev)
{
	struct touch_gpio * gpios;

	gpios = devm_kzalloc( dev, sizeof( struct touch_gpio), GFP_KERNEL ) ;
	if(!gpios)
		dev_err(dev, "Failed to allocate touch_gpio struct\n");
	else
		gpios->dev = dev;
	return gpios;

}

void touch_gpio_free ( struct touch_gpio * gpios)
{
	struct device * dev = gpios->dev;

	if (gpio_is_valid(gpios->reset_pin)) 
	{
		gpio_unexport(gpios->reset_pin);
		devm_gpio_free(dev, gpios->reset_pin);
	}
	if (gpio_is_valid(gpios->wake_pin)) 
	{
		gpio_unexport(gpios->wake_pin);
		devm_gpio_free(dev, gpios->wake_pin);
	}
	if (gpio_is_valid(gpios->irq_pin)) 
	{
		gpio_unexport(gpios->irq_pin);
		devm_gpio_free(dev, gpios->irq_pin);
	}

	devm_kfree(dev, gpios);
}
EXPORT_SYMBOL(touch_gpio_free);

struct touch_gpio * touch_gpio_set_from_pdata( struct device *dev, int usage, 
	int  reset_gpio, int  reset_low_active, int wake_gpio, int wake_low_active, int irq_gpio, int irq_low_active)
{
	struct touch_gpio * gpios;
	int error = 0;
	/*
	 * irq_pin is not needed for DT setup.
	 * irq is associated via 'interrupts' property in DT
	 */
	gpios = touch_gpio_allocate(dev);
	if(!gpios){ error = -ENOMEM; goto out; }
	gpios->usage = usage;

	if( usage & reset_mandatory )
	{
		// Read the reset gpio from dt, allow both reset-gpios and reset-gpio
		gpios->reset_pin = reset_gpio;
		if (!gpio_is_valid(gpios->reset_pin)) {
			error = -EINVAL;
			gpios->reset_pin = -EINVAL;	
			if ( usage & reset_optional ){
				// No pin configured but we don't care
			}else{
				dev_err(dev,"Error setting RESET pin %d.\n", gpios->reset_pin);
				goto out;
			}
		};
		gpios->reset_low_active = !!reset_low_active;
	}
	if( usage & wake_mandatory )
	{
		// Read the wake gpio from dt, allow both wake-gpios and wake-gpio
		gpios->wake_pin = wake_gpio;
		if (!gpio_is_valid(gpios->wake_pin)) {
			error = -EINVAL;
			gpios->wake_pin = -EINVAL;	
			if ( usage & wake_optional ){
				// No pin configured but we don't care
			}else{
				dev_err(dev,"Error setting wake pin %d.\n", gpios->wake_pin);
				goto out;
			}
		};
		gpios->wake_low_active = !!wake_low_active;
	}
	if( usage & irq_mandatory )
	{
		// Read the irq gpio from dt, allow both irq-gpios and irq-gpio
		gpios->irq_pin = irq_gpio;
		if (!gpio_is_valid(gpios->irq_pin)) {
			error = -EINVAL;
			gpios->irq_pin = -EINVAL;	
			if ( usage & irq_optional ){
				// No pin configured but we don't care
			}else{
				dev_err(dev,"Error setting irq pin %d.\n", gpios->irq_pin);
				goto out;
			}
		};
		gpios->irq_low_active = !!irq_low_active;
	}
	
	error = request_gpios(gpios);
	dev_dbg(dev, "gpios initialized: IRQ %d, WAKE pin %d, Reset pin %d.\n",
		gpios->irq_pin, gpios->wake_pin, gpios->reset_pin);
	return gpios;
out:
	devm_kfree(dev, gpios);
	return ERR_PTR(error);
}
EXPORT_SYMBOL(touch_gpio_set_from_pdata);

#ifdef CONFIG_OF
struct touch_gpio * touch_gpio_read_from_dt( struct device *dev, int  usage)
{
	struct device_node *np = dev->of_node;
	struct touch_gpio * gpios;
	int error = 0;
	/*
	 * irq_pin is not needed for DT setup.
	 * irq is associated via 'interrupts' property in DT
	 */
	gpios = touch_gpio_allocate(dev);
	if(!gpios){ error = -ENOMEM; goto out; }

	gpios->usage = usage;

	if( usage & reset_mandatory )
	{
		// Read the reset gpio from dt, allow both reset-gpios and reset-gpio
		gpios->reset_pin = of_get_named_gpio(np, "reset-gpios", 0);
		if (!gpio_is_valid(gpios->reset_pin)) {
			gpios->reset_pin = of_get_named_gpio(np, "reset-gpio", 0);
			if (!gpio_is_valid(gpios->reset_pin)) {
				error = gpios->reset_pin;
				gpios->reset_pin = -EINVAL;	
				if ( usage & reset_optional ){
					// No pin configured but we don't care
				}else{
					dev_err(dev,"Failed to read RESET pin %d.\n", gpios->reset_pin);
					goto out;
				}
			};
		};

		if((error = of_property_read_u32( np, "reset-low-active", &gpios->reset_low_active)))
		{
			gpios->reset_low_active = ( error == -ENODATA || error == -EOVERFLOW ? 1 : 0 );   // if the reset-low-active field exist but has no data we set this field
		}

	}
	if( usage & wake_mandatory )
	{
		// Read the wake gpio from dt, allow both wake-gpios and wake-gpio
		gpios->wake_pin = of_get_named_gpio(np, "wake-gpios", 0);
		if (!gpio_is_valid(gpios->wake_pin)) {
			gpios->wake_pin = of_get_named_gpio(np, "wake-gpio", 0);
			if (!gpio_is_valid(gpios->wake_pin)) {
				error = gpios->wake_pin;
				gpios->wake_pin = -EINVAL;	
				if ( usage & wake_optional ){
					// No pin configured but we don't care
				}else{
					dev_err(dev,"Failed to read wake pin %d.\n", gpios->wake_pin);
					goto out;
				}
			};
		};

		if((error = of_property_read_u32( np, "wake-low-active", &gpios->wake_low_active)))
		{
			gpios->wake_low_active = ( error == -ENODATA || error == -EOVERFLOW ? 1 : 0 );   // if the wake-low-active field exist but has no data we set this field
		}

	}
	if( usage & irq_mandatory )
	{
		// Read the irq gpio from dt, allow both irq-gpios and irq-gpio
		gpios->irq_pin = of_get_named_gpio(np, "irq-gpios", 0);
		if (!gpio_is_valid(gpios->irq_pin)) {
			gpios->irq_pin = of_get_named_gpio(np, "irq-gpio", 0);
			if (!gpio_is_valid(gpios->irq_pin)) {
				error = gpios->irq_pin;
				gpios->irq_pin = -EINVAL;	
				if ( usage & irq_optional ){
					// No pin configured but we don't care
				}else{
					dev_err(dev,"Failed to read irq pin %d.\n", gpios->irq_pin);
					goto out;
				}
			};
		};

		/* Zusätzlich oder alternativ oder nur
		if( 0 == of_property_read_u32_index( np, "interrupts", 1, &i))
			p_egalax_i2c_dev->irq_flags = i;
			*/
		if((error = of_property_read_u32( np, "irq-low-active", &gpios->irq_low_active)))
		{
			gpios->irq_low_active = ( error == -ENODATA || error == -EOVERFLOW ? 1 : 0 );   // if the irq-low-active field exist but has no data we set this field
		}

	}
	if( 0 != of_property_read_u32( np, "irq-flags", &gpios->irq_flags))
	{
		if(gpios->irq_low_active )
		{
			gpios->irq_flags = IRQF_TRIGGER_LOW;
		}else{
			gpios->irq_flags = IRQF_TRIGGER_HIGH;
		}
	}
	
	error = request_gpios(gpios);
	dev_dbg(dev, "gpios initialized: IRQ %d, WAKE pin %d, Reset pin %d.\n",
		gpios->irq_pin, gpios->wake_pin, gpios->reset_pin);

	return gpios;
out:
	devm_kfree(dev, gpios);
	return ERR_PTR(error);
}
#else
struct touch_gpio * touch_gpio_read_from_dt( struct device *dev, int  usage)
{
	return ERR_PTR(-EINVAL);
}
#endif
EXPORT_SYMBOL(touch_gpio_read_from_dt);

int touch_gpio_get_irq_flags ( const struct touch_gpio * gpios)
{
	return gpios->irq_flags;
}
EXPORT_SYMBOL(touch_gpio_get_irq_flags);

/* True: activate reset, False: release reset */
void touch_gpio_reset_out ( const struct touch_gpio * gpios, bool state )
{
	int pin_state=state ? 1 : 0;

	dev_dbg(gpios->dev, "Set reset pin %d, %s to %d\n",gpios->reset_pin, gpios->reset_low_active?"low_active":"high_active",state);
	if (! gpio_is_valid(gpios->reset_pin)) {
		if(!(gpios->usage & reset_optional))
			pr_err("Reset gpio is invalid\n");
		return;
	}

	if(gpios->reset_low_active)
		pin_state=!pin_state;

	gpio_set_value(gpios->reset_pin, pin_state);
}
EXPORT_SYMBOL(touch_gpio_reset_out);

void touch_gpio_do_reset ( const struct touch_gpio * gpios, int delay_us )
{
	int delay_ms=0;
	if(delay_us > 1000)
	{
		delay_ms=delay_us/1000;
		delay_us=delay_us- (delay_ms * 1000);
	}
	dev_dbg(gpios->dev, "Reset for %d msec and %d usecs\n",delay_ms, delay_us);
	touch_gpio_reset_out(gpios, true);
	if(delay_ms)	msleep_interruptible(delay_ms);
	if(delay_us)	udelay(delay_us);
	touch_gpio_reset_out(gpios, false);
}
EXPORT_SYMBOL(touch_gpio_do_reset);

/* True: activate wake, False: release wake */
void touch_gpio_wake_out ( const struct touch_gpio * gpios, bool state )
{
	int pin_state=state ? 1 : 0;

	dev_dbg(gpios->dev, "Set wake pin %d, %s to %d\n",gpios->wake_pin, gpios->wake_low_active?"low_active":"high_active",state);
	if (! gpio_is_valid(gpios->wake_pin)) {
		if(!(gpios->usage & wake_optional))
			pr_err("wake gpio is invalid\n");
		return;
	}

	if(gpios->wake_low_active)
		pin_state=!pin_state;

	gpio_set_value(gpios->wake_pin, pin_state);
}
EXPORT_SYMBOL(touch_gpio_wake_out);


/* Returns true if irq pin is active, false if it is not active */
bool touch_gpio_get_irq_state ( const struct touch_gpio * gpios)
{
	int pin_state;
	if (!gpio_is_valid(gpios->irq_pin)) {
		if(!(gpios->usage & irq_optional))
			pr_err("irq gpio is invalid\n");
		return false;
	}

	pin_state = gpio_get_value(gpios->irq_pin);

	if(gpios->irq_low_active)
		pin_state=!pin_state;

	return pin_state==0 ? false : true;
}
EXPORT_SYMBOL(touch_gpio_get_irq_state);

void touch_gpio_irq_direction_output ( const struct touch_gpio * gpios, bool initial_state)
{
	int pin_state = initial_state;
	if( 0 != gpios->irq_low_active)
		pin_state = !pin_state;

	if (!gpio_is_valid(gpios->irq_pin)) {
		if(!(gpios->usage & irq_optional))
			pr_err("irq gpio is invalid\n");
	}

	if(gpio_direction_output(gpios->irq_pin, pin_state ))
		dev_err(gpios->dev, "Failed to change direction of irq pin %d to output\n", gpios->irq_pin);
}
EXPORT_SYMBOL(touch_gpio_irq_direction_output);

void touch_gpio_irq_direction_input ( const struct touch_gpio * gpios)
{
	if (!gpio_is_valid(gpios->irq_pin)) {
		if(!(gpios->usage & irq_optional))
			pr_err("irq gpio is invalid\n");
	}

	if(gpio_direction_input(gpios->irq_pin))
		dev_err(gpios->dev, "Failed to change direction of irq pin %d to input\n", gpios->irq_pin);
}
EXPORT_SYMBOL(touch_gpio_irq_direction_input);

/* True: activate irq, False: release irq */
void touch_gpio_irq_out ( const struct touch_gpio * gpios, bool state )
{
	int pin_state=state ? 1 : 0;

	dev_dbg(gpios->dev, "Set irq pin %d, %s to %d\n",gpios->irq_pin, gpios->irq_low_active?"low_active":"high_active",state);
	if (! gpio_is_valid(gpios->irq_pin)) {
		if(!(gpios->usage & irq_optional))
			pr_err("irq gpio is invalid\n");
		return;
	}

	if(gpios->irq_low_active)
		pin_state=!pin_state;

	gpio_set_value(gpios->irq_pin, pin_state);
}
EXPORT_SYMBOL(touch_gpio_irq_out);

static int touch_gpio_init(void){return 0;}
static void touch_gpio_exit(void){return;}

module_init(touch_gpio_init);
module_exit(touch_gpio_exit);

MODULE_AUTHOR("garz-fricke.com <jonas.hoeppner@garz-fricke.com>");
MODULE_DESCRIPTION("Some function to configure interupt and reset line for touches.");
MODULE_LICENSE("GPL v2");
