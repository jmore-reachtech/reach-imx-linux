/*
 * Copyright (C) 2014 Reach Technology, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*!
 * @file g2h_dip.c
 *
 * @brief This file contains the 4 position DIP switch driver device interface 
 * and fops functions.
 */

//#define DEBUG

#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/kfifo.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/g2h_dip.h>

typedef struct dipswitch_t {
    int pin1;
    int pin2;
    int pin3;
    int pin4;
};

static struct dipswitch_t dipswitch;
static struct kobject *dipswitch_kobj;

int dipswitch_get_value()
{
    int val = 0;
    
    val |= (gpio_get_value(dipswitch.pin1) >> 20);
    val |= (gpio_get_value(dipswitch.pin2) >> 16);
    val |= (gpio_get_value(dipswitch.pin3) >> 17);
    val |= (gpio_get_value(dipswitch.pin4) >> 18);
  
    printk("%s: dip value %d \n", __func__, val);

    return val;
}
EXPORT_SYMBOL_GPL(dipswitch_get_value);

static ssize_t dipswitch_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", dipswitch_get_value());
}

static DEVICE_ATTR(val, 0664, dipswitch_show, NULL);

static struct attribute *dipswitch_attributes[] = {
	&dev_attr_val.attr,
	NULL,
};

static struct attribute_group dipswitch_attr_group = {
	.attrs = dipswitch_attributes,
};


static int dipswitch_parse_dt(void)
{
    struct device_node *np;

    pr_debug("%s: \n", __func__);
    
    np = of_find_node_by_path("/dipswitch@0");
    if (!np) {
        pr_err("dipswitch of node not found! \n");
        return -ENODEV;
    }
    
    /* GPIO already set up in platform, just get GPIO pins */
    dipswitch.pin1 = of_get_named_gpio(np,"dip-pin-1", 0);
    if (!gpio_is_valid(dipswitch.pin1)) {
        return -ENODEV;
    }
    dipswitch.pin2 = of_get_named_gpio(np,"dip-pin-2", 0);
    if (!gpio_is_valid(dipswitch.pin2)) {
        return -ENODEV;
    }
    dipswitch.pin3 = of_get_named_gpio(np,"dip-pin-3", 0);
    if (!gpio_is_valid(dipswitch.pin3)) {
        return -ENODEV;
    }
    dipswitch.pin4 = of_get_named_gpio(np,"dip-pin-4", 0);
    if (!gpio_is_valid(dipswitch.pin4)) {
        return -ENODEV;
    }

    return 0;
}

static int __init dipswitch_init(void)
{
    int ret;

    ret = dipswitch_parse_dt();
    if (ret) {
        pr_err("G2H dipswitch failed to parse dt \n");
        return -ENODEV;
    }

    dipswitch_kobj = kobject_create_and_add("dipswitch", kernel_kobj);

    ret = sysfs_create_group(dipswitch_kobj, &dipswitch_attr_group);

    pr_info("%s: registered \n", __func__);    

    return 0;
}

static void __exit dipswitch_exit(void)
{
    pr_info("%s: \n", __func__);    
}

//module_init(dipswitch_init);
subsys_initcall(dipswitch_init);
module_exit(dipswitch_exit);

MODULE_AUTHOR("Jeff Horn");
MODULE_DESCRIPTION("G2H DIP Switch Driver");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL v2");
