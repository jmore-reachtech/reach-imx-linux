/*
 * Copyright (C) 2012 Simon Budig, <simon.budig@kernelconcepts.de>
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
 * This is a driver for the EDT "Polytouch" family of touch controllers
 * based on the FocalTech FT5x06 line of chips.
 *
 * Development of this driver has been sponsored by Glyn:
 *    http://www.glyn.com/Products/Displays
 */

#include <linux/module.h>
#include <linux/ratelimit.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/fs.h> 

#define EDT_NAME_LEN    23
#define USE_ABS_MT
#define MAX_TOUCHES     5
#define X_RES           0x7FF
#define Y_RES           0x7FF

static const char *client_name = "evr-ft5x06";

struct evr_ft5x06_ts_data {
	struct i2c_client   *client;
	struct input_dev    *input;
    int			        irq;
	u16                 num_x;
	u16                 num_y;
};

struct point {
	int	x;
	int	y;
	int	id;
};

static int calibration[7] = {
   	65536,0,0,
	0,65536,0,
	65536 
};
module_param_array(calibration, int, NULL, S_IRUGO | S_IWUSR);

static int swap_x_y = 0;
module_param(swap_x_y, int, S_IRUGO | S_IWUSR);

static void translate(int *px, int *py)
{
	int x, y, x1, y1;
	if (calibration[6]) {
		x1 = *px;
		y1 = *py;

		x =  (calibration[0] * x1) + (calibration[1] * y1) + calibration[2];
		x /= calibration[6];
		if (x < 0)
			x = 0;
		y =  (calibration[3] * x1) + (calibration[4] * y1) + calibration[5];
		y /= calibration[6];
		if (y < 0)
			y = 0;
        if(swap_x_y) {
		    *px = y ;
		    *py = x ;
        } else {
		    *px = x ;
		    *py = y ;
        }
	}
}

static inline void evr_ft5x06_ts_evt_add(struct evr_ft5x06_ts_data *ts,
			      unsigned touches, struct point *p)
{
	struct input_dev *idev = ts->input;
	int i;
	if (!touches) {
		/* send release to user space. */
#ifdef USE_ABS_MT
		input_event(idev, EV_ABS, ABS_MT_TOUCH_MAJOR, 0);
		input_mt_sync(idev);
#else
		input_report_abs(idev, ABS_PRESSURE, 0);
		input_report_key(idev, BTN_TOUCH, 0);
		input_sync(idev);
#endif
	} else {
		for (i = 0; i < touches; i++) {
			translate(&p[i].x, &p[i].y);
#ifdef USE_ABS_MT
            pr_debug("x=%d, y=%d, t=%d\n",p[i].x, p[i].y, p[i].id);
			input_event(idev, EV_ABS, ABS_MT_POSITION_X, p[i].x);
			input_event(idev, EV_ABS, ABS_MT_POSITION_Y, p[i].y);
			input_event(idev, EV_ABS, ABS_MT_TRACKING_ID, p[i].id);
			input_event(idev, EV_ABS, ABS_MT_TOUCH_MAJOR, 1);
			input_mt_sync(idev);
#else
			input_report_abs(idev, ABS_X, p[i].x);
			input_report_abs(idev, ABS_Y, p[i].y);
			input_report_abs(idev, ABS_PRESSURE, 1);
			input_report_key(idev, BTN_TOUCH, 1);
			input_sync(idev);
#endif
		}
	}
#ifdef USE_ABS_MT
	input_sync(idev);
#endif
}

static irqreturn_t evr_ft5x06_ts_isr(int irq, void *dev_id)
{
	struct evr_ft5x06_ts_data *ts = dev_id;
	int ret;
    struct point points[MAX_TOUCHES];
    unsigned char buf[3+(6*MAX_TOUCHES)];

    unsigned char startch[1] = { 0 };
	struct i2c_msg readpkt[2] = {
		{ts->client->addr, 0, 1, startch},
		{ts->client->addr, I2C_M_RD, sizeof(buf), buf}
	};
	int touches = 0 ;

    ret = i2c_transfer(ts->client->adapter, readpkt,
	                    ARRAY_SIZE(readpkt));
	if (ret != ARRAY_SIZE(readpkt)) {
	    pr_warn(KERN_WARNING "%s: i2c_transfer failed\n",
		        client_name);
	} else {
        int i;
        unsigned char *p = buf+3;

        touches = buf[2];
        if(touches > MAX_TOUCHES) {
	        pr_warn(KERN_WARNING "%s: invalid touch count %d\n",
		        client_name, touches);
        } else {
            for (i = 0; i < touches; i++) {
			    points[i].x  = (((p[0] & 0x0F) << 8) | p[1]) & X_RES;
				points[i].y  = (((p[2] & 0x0F) << 8) | p[3]) & Y_RES;
				points[i].id = (p[2] >> 4);
				p += 6;
			}
            evr_ft5x06_ts_evt_add(ts, touches, points); 
        }
    }

    return IRQ_HANDLED;
}

static int evr_ft5x06_ts_register(struct evr_ft5x06_ts_data *ts)
{
	struct input_dev *idev;
	idev = input_allocate_device();
	if (idev == NULL)
		return -ENOMEM;

	ts->input = idev;
	idev->name = client_name ;
	idev->id.product = ts->client->addr;

	__set_bit(EV_ABS, idev->evbit);
	__set_bit(EV_KEY, idev->evbit);
	__set_bit(BTN_TOUCH, idev->keybit);

#ifdef USE_ABS_MT
	input_set_abs_params(idev, ABS_MT_POSITION_X, 0, ts->num_x, 0, 0);
	input_set_abs_params(idev, ABS_MT_POSITION_Y, 0, ts->num_y, 0, 0);
	input_set_abs_params(idev, ABS_MT_TRACKING_ID, 0, MAX_TOUCHES, 0, 0);
	input_set_abs_params(idev, ABS_X, 0, ts->num_x, 0, 0);
	input_set_abs_params(idev, ABS_Y, 0, ts->num_y, 0, 0);
	input_set_abs_params(idev, ABS_MT_TOUCH_MAJOR, 0, 1, 0, 0);
#else
	__set_bit(EV_SYN, idev->evbit);
	input_set_abs_params(idev, ABS_X, 0, ts->num_x, 0, 0);
	input_set_abs_params(idev, ABS_Y, 0, ts->num_y, 0, 0);
	input_set_abs_params(idev, ABS_PRESSURE, 0, 1, 0, 0);
#endif

	input_set_drvdata(idev, ts);
	return input_register_device(idev);
}

static void evr_ft5x06_ts_deregister(struct evr_ft5x06_ts_data *ts)
{
	if (ts->input) {
		input_unregister_device(ts->input);
		input_free_device(ts->input);
		ts->input = NULL;
	}
}

/* Return 0 if detection is successful, -ENODEV otherwise */
static int evr_ft5x06_detect(struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	char buffer;
	struct i2c_msg pkt = {
		client->addr,
		I2C_M_RD,
		sizeof(buffer),
		&buffer
	};
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;
	if (i2c_transfer(adapter, &pkt, 1) != 1)
		return -ENODEV;

	return 0;
}

static int evr_ft5x06_ts_probe(struct i2c_client *client,
					 const struct i2c_device_id *id)
{
	int err = 0;
	struct evr_ft5x06_ts_data *ts;
	//struct input_dev *input;
    struct device *dev = &client->dev;
    //struct device_node *np = client->dev.of_node;

	dev_dbg(dev, "probing for Evervision FT5x06 I2C\n");

    if(evr_ft5x06_detect(client) != 0) {
       	dev_err(dev, "%s: Could not detect touch screen.\n",
			client_name);
		return -ENODEV;
    }

    ts = kzalloc(sizeof(struct evr_ft5x06_ts_data), GFP_KERNEL);
	if (!ts) {
		dev_err(dev, "Couldn't allocate memory for %s\n", client_name);
		return -ENOMEM;
	}
    
    ts->client = client;
	ts->irq = client->irq;
    ts->num_x = X_RES;
    ts->num_y = Y_RES;

    err = request_threaded_irq(ts->irq, NULL, evr_ft5x06_ts_isr,
				     IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				     client_name, ts);
	if (err) {
		pr_err("%s: error requesting irq %d\n", __func__, ts->irq);
		goto err_free_mem;
	}

	i2c_set_clientdata(client, ts);
	err = evr_ft5x06_ts_register(ts);
	if (err) {
		pr_err("%s: errorr registering input \n", __func__);
		goto err_free_mem;
	}

	return 0;

err_free_mem:
	evr_ft5x06_ts_deregister(ts);
    free_irq(ts->irq, ts); 
	kfree(ts);

    return err;
}

static int evr_ft5x06_ts_remove(struct i2c_client *client)
{
	struct evr_ft5x06_ts_data *ts = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "removing Evervision FT5x06 I2C\n");
	free_irq(client->irq, ts);
    evr_ft5x06_ts_deregister(ts);

	kfree(ts);

	return 0;
}

static const struct i2c_device_id evr_ft5x06_ts_id[] = {
	{ "evervision", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, evr_ft5x06_ts_id);

static struct of_device_id evr_ft5x06_dt_ids[] = {
	{ .compatible = "evervision" },
	{ /* sentinel */ }
};

static struct i2c_driver evr_ft5x06_ts_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "evr-ft5x06",
        .of_match_table = evr_ft5x06_dt_ids,
	},
	.id_table = evr_ft5x06_ts_id,
	.probe    = evr_ft5x06_ts_probe,
	.remove   = evr_ft5x06_ts_remove,
};

module_i2c_driver(evr_ft5x06_ts_driver);

MODULE_AUTHOR("Jeff Horn <jeff@everlook.net>");
MODULE_DESCRIPTION("Evervision FT5x06 I2C Touchscreen Driver");
MODULE_LICENSE("GPL");
