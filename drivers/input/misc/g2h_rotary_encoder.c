/*
 * g2h_rotary_encoder.c
 *
 * (c) 2015 Jeff Horn <jeff@everlook.net>
 * 
 * Driver based on rotary_encoder.c
 * (c) 2009 Daniel Mack <daniel@caiaq.de>
 * Copyright (C) 2011 Johan Hovold <jhovold@gmail.com>
 *
 * state machine code inspired by code from Tim Ruetz
 *
 * A generic driver for rotary encoders connected to GPIO lines.
 * See file:Documentation/input/rotary-encoder.txt for more information
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/of_device.h>

#define DRV_NAME "g2h-rotary-encoder"

#define REG_INPUT   0x00
#define REG_OUTPUT  0x01
#define REG_POL     0x02
#define REG_CFG     0x03

#define ROTARY_MASK 0x0C

struct rotary_encoder_data {
	struct i2c_client   *client;
	struct input_dev    *input;
    struct work_struct work;

    unsigned int axis;
	unsigned int pos;
    unsigned int steps;

	unsigned int irq;

	short dir;	/* 1 = CW, -1 = CCW */

	char last_stable;
};

static void g2h_rotary_encoder_report_event(struct rotary_encoder_data *encoder)
{
    /* report event */
    //printk("reporting %d \n", encoder->dir);
    input_report_rel(encoder->input, REL_Y, encoder->dir);
    input_sync(encoder->input);
}

static void g2h_rotary_encoder_work_func(struct work_struct *work)
{
    struct rotary_encoder_data *encoder =
		container_of(work, struct rotary_encoder_data, work);
    int ret, cur;

    /* read the input pins */
    ret = i2c_smbus_read_byte_data(encoder->client, REG_INPUT);
    if(ret < 0) {
        printk("%s: error reading encoder %d\n", __func__, ret);
        return;
    }
    
    cur = (ret & ROTARY_MASK) >> 2;
    
    if(encoder->last_stable == cur) {
        return;
    }

    switch (cur) {
        case 0x0:
            if(encoder->last_stable == 0x2)
                encoder->dir = -1;
            else
                encoder->dir = 1; break;
        case 0x2:
            if(encoder->last_stable == 0x3)
                encoder->dir = -1;
            else
                encoder->dir = 1; break;
        case 0x3:
            if(encoder->last_stable == 0x1)
                encoder->dir = -1;
            else
                encoder->dir = 1; break;
        case 0x1:
            if(encoder->last_stable == 0x0)
                encoder->dir = -1;
            else
                encoder->dir = 1; break;
    }

    encoder->last_stable = cur;

    g2h_rotary_encoder_report_event(encoder);
}

static irqreturn_t g2h_rotary_encoder_irq(int irq, void *dev_id)
{
	struct rotary_encoder_data *encoder = dev_id;

    schedule_work(&encoder->work);

	return IRQ_HANDLED;
}

/* Return 0 if detection is successful, -ENODEV otherwise */
static int g2h_rotary_encoder_detect(struct i2c_client *client)
{
        int ret = 0;
	struct i2c_adapter *adapter = client->adapter;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

    /* check for device, set all pins as input */
    if(i2c_smbus_write_byte_data(client, REG_CFG, 0xFF) != 0) {
        return -ENODEV;
    }

    // hack so the INT does not get stuck, see PCA953X errata
    ret = i2c_smbus_read_byte_data(client, REG_INPUT);
    if(ret < 0) {
        printk("%s: error reading encoder %d\n", __func__, ret);
    }

	return 0;
}

static void g2h_rotary_encoder_parse_dt(struct i2c_client *client,
            struct rotary_encoder_data *encoder)
{
    struct device_node *np = client->dev.of_node;

    if(np == NULL) {
        return;
    }

    encoder->axis = 16;
    encoder->pos = 0;
}

static int g2h_rotary_encoder_probe(struct i2c_client *client,
					 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct rotary_encoder_data *encoder;
	struct input_dev *input;
	int err;

    printk("%s: \n", __func__);

    if(g2h_rotary_encoder_detect(client) != 0) {
       	dev_err(dev, "%s: Could not detect gpio expander.\n",
			DRV_NAME);
		return -ENODEV;
    }

	encoder = kzalloc(sizeof(struct rotary_encoder_data), GFP_KERNEL);
	input = input_allocate_device();
	if (!encoder || !input) {
		err = -ENOMEM;
		goto exit_free_mem;
	}

    encoder->client = client;
	encoder->input = input;

	input->name = DRV_NAME;
	input->id.product = encoder->client->addr;

    g2h_rotary_encoder_parse_dt(client, encoder);
    
    input->evbit[0] = BIT_MASK(EV_REL);
    input->relbit[0] = BIT_MASK(REL_X) | BIT_MASK(REL_Y); 

    encoder->irq = client->irq;
	/* request the IRQs */
	err = request_irq(encoder->irq, g2h_rotary_encoder_irq,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  DRV_NAME, encoder);
	if (err) {
		dev_err(dev, "unable to request IRQ %d\n", encoder->irq);
		goto exit_free_mem;
	}

	err = input_register_device(input);
	if (err) {
		dev_err(dev, "failed to register input device\n");
		goto exit_free_mem;
	}

    INIT_WORK(&encoder->work, g2h_rotary_encoder_work_func); 
    
    i2c_set_clientdata(client, encoder);

	return 0;

exit_free_mem:
	input_free_device(input);
	kfree(encoder);

	return err;
}

static int g2h_rotary_encoder_remove(struct i2c_client *client)
{
	struct rotary_encoder_data *encoder = i2c_get_clientdata(client);

	free_irq(encoder->irq, encoder);

	input_unregister_device(encoder->input);
	kfree(encoder);

	return 0;
}

static const struct i2c_device_id g2h_rotary_encoder_id[] = {
	{ "g2h-rotary_encoder", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, g2h_rotary_encoder_id);

static struct of_device_id g2h_rotary_encoder_dt_ids[] = {
	{ .compatible = "g2h-rotary-encoder" },
	{ /* sentinel */ }
};

static struct i2c_driver g2h_rotary_encoder_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = DRV_NAME,
        .of_match_table = g2h_rotary_encoder_dt_ids,
	},
	.id_table = g2h_rotary_encoder_id,
	.probe    = g2h_rotary_encoder_probe,
	.remove   = g2h_rotary_encoder_remove,
};

module_i2c_driver(g2h_rotary_encoder_driver);

MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DESCRIPTION("GPIO rotary encoder driver");
MODULE_AUTHOR("Jeff Horn <jeff@everlook.net>");
MODULE_LICENSE("GPL v2");
