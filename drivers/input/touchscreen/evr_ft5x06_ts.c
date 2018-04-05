/*
 * Copyright (C) 2017 Jeff Horn, <jeff.horn@reachtech.com>
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
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*
 * This is a driver for the "Evervision" family of touch controllers
 * based on the FocalTech FT5x06 line of chips.
 *
 */

//#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/i2c/ft5x06_ts.h>

#define TS_NAME "evr-ft5x06-ts"

#define FTS_POINT_UP		0x01
#define FTS_POINT_DOWN		0x00
#define FTS_POINT_CONTACT	0x02

struct point {
	int x;
	int y;
	int id;
};

struct ts_event {
	u16 au16_x[CFG_MAX_TOUCH_POINTS];			/*x coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];			/*y coordinate */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];	/*touch event: 
													0 -- down; 1-- contact; 2 -- contact */
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];		/*touch ID */
	u16 pressure;
	u8 touch_point;
};

struct evr_ft5x0x_ts_data {
	unsigned int irq;
	unsigned int x_max;
	unsigned int y_max;
	unsigned int reset_gpio;
	struct i2c_client *client;
	struct input_dev *input_dev;
#ifdef CONFIG_PM
	struct early_suspend *early_suspend;
#endif
};


int evr_ft5x0x_i2c_write(struct i2c_client *client, int reg_num, int reg_val)
{
    int ret = 0;

    u8 regnval[] = {
		reg_num,
		reg_val
	};

    struct i2c_msg pkt = {
		.addr   = client->addr, 
        .flags  = 0,
        .len    = sizeof(regnval), 
        .buf    = regnval,
	};
	
    ret = i2c_transfer(client->adapter, &pkt, 1);
	if (ret != 1) {
		printk(KERN_WARNING "%s: i2c_transfer failed\n", __func__);
    } else {
		printk(KERN_DEBUG "%s: set register 0x%02x to 0x%02x\n",
		       __func__, reg_num, reg_val);
    }

    return ret;
}

int evr_ft5x0x_i2c_read(struct i2c_client *client, char *w_buf, int w_len, char *r_buf, int r_len)
{
	int ret;

	if (w_len > 0) {
		struct i2c_msg msgs[] = {
			{
				.addr 	= client->addr,
				.flags 	= 0,
				.len 	= w_len,
				.buf 	= w_buf,
			},
			{
				.addr 	= client->addr,
				.flags 	= I2C_M_RD,
				.len 	= r_len,
				.buf 	= r_buf,
			},
		};
		
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0) {
			dev_err(&client->dev, "f%s: i2c read error.\n",__func__);
		}
	} else {
		struct i2c_msg msgs[] = {
			{
				.addr 	= client->addr,
				.flags 	= I2C_M_RD,
				.len 	= r_len,
				.buf 	= r_buf,
			},
		};
		
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0) {
			dev_err(&client->dev, "f%s: i2c read error.\n",__func__);
		}
	}

	return ret;
}

static irqreturn_t evr_ft5x0x_ts_interrupt(int irq, void *dev_id)
{
	struct evr_ft5x0x_ts_data *ts = dev_id;
	int ret = 0;
	int mode, points,contact, x, y;
	u8 buf[POINT_READ_BUF] = { 0 };

	/* we need to write addr 0x0 to the controller before reading */
	ret = evr_ft5x0x_i2c_read(ts->client, buf, 1, buf, sizeof(buf));
	if (ret < 0) {
		pr_debug("%s: i2c transfer failed \n", __func__);
		goto out;
	}

	mode = (buf[0] & 0x70) >> 4;
	points = buf[2];
	contact = (buf[3] & 0xC0) >> 6;
	x = ((buf[3] & 0x0F) << 8) | (buf[4]);
	y = ((buf[5] & 0x0F) << 8) | (buf[6]);
	
#ifdef DEBUG
	pr_debug("m=%d, p=%d, c=%d, x=%d, y=%d \n",mode, points, contact, x ,y);
#endif
	switch (contact) {
		case 0:
		case 2:
			input_report_abs(ts->input_dev, ABS_X, x);
            input_report_abs(ts->input_dev, ABS_Y, y);
            input_report_abs(ts->input_dev, ABS_PRESSURE, 255);
			input_report_key(ts->input_dev, BTN_TOUCH, 1);
            input_sync(ts->input_dev);
            break;
		case 1:
            input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
			input_report_key(ts->input_dev, BTN_TOUCH, 0);
            input_sync(ts->input_dev);
	}

out:
	return IRQ_HANDLED;
}

static int evr_ft5x06_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err = 0;
	struct evr_ft5x0x_ts_data *ft5x0x_ts;
    struct device_node *np = client->dev.of_node;
    struct input_dev *input_dev;
	unsigned char uc_reg_value;
    unsigned char uc_reg_addr;    

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        pr_debug("%s: I2C checked failed\n", __func__);
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

    ft5x0x_ts = kzalloc(sizeof(struct evr_ft5x0x_ts_data), GFP_KERNEL);
    if (!ft5x0x_ts) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

    i2c_set_clientdata(client, ft5x0x_ts);
    ft5x0x_ts->irq = client->irq;
	ft5x0x_ts->client = client;
	// TODO: pull this from the dt
	ft5x0x_ts->x_max = 2047;
	ft5x0x_ts->y_max = 2047;
    
    err = request_threaded_irq(client->irq, NULL, evr_ft5x0x_ts_interrupt,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT, client->name, ft5x0x_ts);
	if (err < 0) {
		dev_err(&client->dev, "ft5x0x_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
	disable_irq(client->irq);

    input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

    ft5x0x_ts->input_dev = input_dev;
    input_dev->name = TS_NAME;

    set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
    set_bit(BTN_TOUCH, input_dev->keybit);
	set_bit(EV_SYN, input_dev->evbit);

	input_set_abs_params(input_dev, ABS_X, 0, ft5x0x_ts->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, ft5x0x_ts->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 1, 0, 0);
    
    err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
			"ft5x0x_ts_probe: failed to register input device: %s\n",
			dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

	/* reset controller */
	ft5x0x_ts->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (!gpio_is_valid(ft5x0x_ts->reset_gpio)) {
		pr_debug("invalid reset-gpio \n");
	} else {
		err = gpio_request(ft5x0x_ts->reset_gpio, "ft5x06_reset");
		if (err < 0) {
			pr_debug("failed to request reset-gpio \n");
		} else {
			err = gpio_direction_output(ft5x0x_ts->reset_gpio, 0);
			if (err < 0) {
				pr_debug("failed to set reset-gpio as output\n");
			}
			msleep(10);
			gpio_set_value(ft5x0x_ts->reset_gpio, 1);
			msleep(100);
		}
	}

    uc_reg_addr = FT5x0x_REG_FW_VER;
    evr_ft5x0x_i2c_read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
    dev_dbg(&client->dev, "[FTS] Firmware version = 0x%x\n", uc_reg_value);

    uc_reg_addr = FT5x0x_REG_POINT_RATE;
    evr_ft5x0x_i2c_read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
    dev_dbg(&client->dev, "[FTS] report rate is %dHz \n", uc_reg_value * 10);

#ifdef CONFIG_TOUCHSCREEN_EVR_FT5X06_TUNE
    /*  tune touch controller sensitivity for coverglass thickness */
    evr_ft5x0x_i2c_write(client, FT5x0x_REG_THRESHOLD, 0x0c);
#endif
    uc_reg_addr = FT5x0x_REG_THRESHOLD;
    evr_ft5x0x_i2c_read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
    dev_dbg(&client->dev, "[FTS] touch threshold 0x%x\n", uc_reg_value);

    enable_irq(client->irq);

    return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);

exit_input_dev_alloc_failed:
	free_irq(client->irq, ft5x0x_ts);

exit_irq_request_failed:
	i2c_set_clientdata(client, NULL);
	kfree(ft5x0x_ts);

exit_alloc_data_failed:
exit_check_functionality_failed:
    return err; 
}

static int evr_ft5x06_ts_remove(struct i2c_client *client)
{
    struct evr_ft5x0x_ts_data *ft5x0x_ts;
    ft5x0x_ts = i2c_get_clientdata(client);

	input_unregister_device(ft5x0x_ts->input_dev);
    free_irq(client->irq, ft5x0x_ts);
	gpio_free(ft5x0x_ts->reset_gpio);
    kfree(ft5x0x_ts);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id evr_ft5x06_ts_id[] = {
	{ TS_NAME, 0 },
	{ /* sentinel */}
};
MODULE_DEVICE_TABLE(i2c, evr_ft5x06_ts_id);

static struct of_device_id evr_ft5x06_dt_ids[] = {
	{ .compatible = TS_NAME },
	{ /* sentinel */ }
};

static struct i2c_driver evr_ft5x06_ts_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = TS_NAME,
        .of_match_table = evr_ft5x06_dt_ids,
	},
	.id_table = evr_ft5x06_ts_id,
	.probe    = evr_ft5x06_ts_probe,
	.remove   = evr_ft5x06_ts_remove,
};

static int __init evr_ft5x06_ts_init(void)
{
	int ret;
	ret = i2c_add_driver(&evr_ft5x06_ts_driver);
	if (ret) {
		pr_debug(KERN_WARNING "Adding %s driver failed "
		       "(errno = %d)\n", TS_NAME, ret);
	} else {
		pr_info("Successfully added driver %s\n", 
                evr_ft5x06_ts_driver.driver.name);
	}
	return ret;
}

static void __exit evr_ft5x06_ts_exit(void)
{
	i2c_del_driver(&evr_ft5x06_ts_driver);
}

module_init(evr_ft5x06_ts_init);
module_exit(evr_ft5x06_ts_exit);

MODULE_AUTHOR("Jeff Horn <jeff.horn@reachtech.com>");
MODULE_DESCRIPTION("Evervision FT5x06 I2C Touchscreen Driver");
MODULE_LICENSE("GPL");
