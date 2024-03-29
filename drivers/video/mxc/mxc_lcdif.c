/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/init.h>
#include <linux/ipu.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mxcfb.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>

#include "mxc_dispdrv.h"

struct mxc_lcd_platform_data {
	u32 default_ifmt;
	u32 ipu_id;
	u32 disp_id;
	u32 lcd_enable_gpio;
	u32 backlight_enable_gpio;
};

struct mxc_lcdif_data {
	struct platform_device *pdev;
	struct mxc_dispdrv_handle *disp_lcdif;
};

#define DISPDRV_LCD	"lcd"

static struct fb_videomode lcdif_modedb[] = {
	{
          .name                   = "CLAA-WVGA",
          .refresh                = 60,
          .xres                   = 800,
          .yres                   = 480,
          .pixclock               = KHZ2PICOS(28000),
          .left_margin			  = 100,
          .right_margin			  = 100,
          .hsync_len              = 56,
          .upper_margin			  = 15,
          .lower_margin			  = 5,
          .vsync_len              = 10,
          .sync                   = FB_SYNC_CLK_LAT_FALL,
          .vmode                  = FB_VMODE_NONINTERLACED,
          .flag                   = 0,
    },
    {   
          .name                   = "EVRV-VGA",
          .refresh                = 60, 
          .xres                   = 640,
          .yres                   = 480,
          .pixclock               = KHZ2PICOS(25000),
          .left_margin                    = 48, 
          .right_margin                   = 16, 
          .hsync_len              = 96, 
          .upper_margin                   = 31, 
          .lower_margin                   = 11, 
          .vsync_len              = 2,  
          .sync                   = FB_SYNC_CLK_LAT_FALL,
          .vmode                  = FB_VMODE_NONINTERLACED,
          .flag                   = 0,
    },

};
static int lcdif_modedb_sz = ARRAY_SIZE(lcdif_modedb);

static int lcdif_init(struct mxc_dispdrv_handle *disp,
	struct mxc_dispdrv_setting *setting)
{
	int ret, i, gpio;
	struct mxc_lcdif_data *lcdif = mxc_dispdrv_getdata(disp);
	struct mxc_lcd_platform_data *plat_data
			= lcdif->pdev->dev.platform_data;
	struct fb_videomode *modedb = lcdif_modedb;
	int modedb_sz = lcdif_modedb_sz;

	/* use platform defined ipu/di */
	setting->dev_id = plat_data->ipu_id;
	setting->disp_id = plat_data->disp_id;

	ret = fb_find_mode(&setting->fbi->var, setting->fbi, setting->dft_mode_str,
				modedb, modedb_sz, NULL, setting->default_bpp);
	if (!ret) {
		fb_videomode_to_var(&setting->fbi->var, &modedb[0]);
		setting->if_fmt = plat_data->default_ifmt;
	}

	INIT_LIST_HEAD(&setting->fbi->modelist);
	for (i = 0; i < modedb_sz; i++) {
		fb_add_videomode(&modedb[i],
				&setting->fbi->modelist);
	}

	/* setup lcd and backlight gpio */
	if (gpio_is_valid(plat_data->lcd_enable_gpio)) {
        gpio = gpio_request_one(plat_data->lcd_enable_gpio,
			GPIOF_OUT_INIT_LOW  , "lcd-enable-gpio");
		pr_debug("%s: disable lcd [%d] err [%d]\n", __func__,
			plat_data->lcd_enable_gpio, gpio);
		gpio_set_value(plat_data->lcd_enable_gpio, 0);
	}

	if (gpio_is_valid(plat_data->backlight_enable_gpio)) {
        gpio = gpio_request_one(plat_data->backlight_enable_gpio,
			GPIOF_OUT_INIT_LOW  , "backlight-enable-gpio");
		pr_debug("%s: disable backlight [%d] err [%d]\n", __func__,
			plat_data->backlight_enable_gpio, gpio);
		gpio_set_value(plat_data->backlight_enable_gpio, 0);
	}

	return ret;
}

void lcdif_deinit(struct mxc_dispdrv_handle *disp)
{
	pr_debug("%s: \n", __func__);
}

/* display driver enable function for extension */
int lcdif_enable (struct mxc_dispdrv_handle *disp,
	struct fb_info *fbinfo)
{
	struct mxc_lcdif_data *lcdif = mxc_dispdrv_getdata(disp);
	struct mxc_lcd_platform_data *plat_data
			= lcdif->pdev->dev.platform_data;

	if (gpio_is_valid(plat_data->backlight_enable_gpio)) {
        pr_debug("%s: enable backlight gpio \n", __func__);
        gpio_set_value(plat_data->backlight_enable_gpio, 0);
	}

	if (gpio_is_valid(plat_data->lcd_enable_gpio)) {
        pr_debug("%s: enable lcd gpio \n", __func__);
        gpio_set_value(plat_data->lcd_enable_gpio, 1);
	}

	mdelay(400);
	if (gpio_is_valid(plat_data->backlight_enable_gpio)) {
        pr_debug("%s: enable backlight gpio \n", __func__);
        gpio_set_value(plat_data->backlight_enable_gpio, 1);
	}

	return 0;
}

/* display driver disable function, called at early part of fb_blank */
void lcdif_disable (struct mxc_dispdrv_handle *disp,
 struct fb_info *fbinfo)
{
	struct mxc_lcdif_data *lcdif = mxc_dispdrv_getdata(disp);
	struct mxc_lcd_platform_data *plat_data
			= lcdif->pdev->dev.platform_data;

	if (gpio_is_valid(plat_data->backlight_enable_gpio)) {
        pr_debug("%s: disable backlight gpio \n", __func__);
        gpio_set_value(plat_data->backlight_enable_gpio, 0);
	}

	if (gpio_is_valid(plat_data->lcd_enable_gpio)) {
        pr_debug("%s: disable lcd gpio \n", __func__);
        gpio_set_value(plat_data->lcd_enable_gpio, 0);
	}
	mdelay(100);
}

static struct mxc_dispdrv_driver lcdif_drv = {
	.name 	= DISPDRV_LCD,
	.init 	= lcdif_init,
	.deinit	= lcdif_deinit,
	.enable = lcdif_enable,
	.disable = lcdif_disable,
};

static int lcd_get_of_property(struct platform_device *pdev,
				struct mxc_lcd_platform_data *plat_data)
{
	struct device_node *np = pdev->dev.of_node;
	int err;
	u32 ipu_id, disp_id,lcd_enable, backlight_enable;
	const char *default_ifmt;

	err = of_property_read_string(np, "default_ifmt", &default_ifmt);
	if (err) {
		dev_dbg(&pdev->dev, "get of property default_ifmt fail\n");
		return err;
	}
	err = of_property_read_u32(np, "ipu_id", &ipu_id);
	if (err) {
		dev_dbg(&pdev->dev, "get of property ipu_id fail\n");
		return err;
	}
	err = of_property_read_u32(np, "disp_id", &disp_id);
	if (err) {
		dev_dbg(&pdev->dev, "get of property disp_id fail\n");
		return err;
	}

	lcd_enable = of_get_named_gpio(np, "lcd-enable-gpio", 0);
	if (!gpio_is_valid(lcd_enable)) {
		dev_dbg(&pdev->dev, "get of property lcd-enable-gpio fail\n");
		return -ENODEV;
	} else {
		plat_data->lcd_enable_gpio = lcd_enable;
	}

	backlight_enable = of_get_named_gpio(np, "backlight-enable-gpio", 0);
	if (!gpio_is_valid(backlight_enable)) {
		dev_dbg(&pdev->dev, "get of property backlight-enable-gpio fail\n");
		return -ENODEV;
	} else {
		plat_data->backlight_enable_gpio = backlight_enable;
	}

	pr_debug("%s: lcd gpio [%d]\n", __func__, plat_data->lcd_enable_gpio);
	pr_debug("%s: backlight gpio [%d]\n", __func__, plat_data->backlight_enable_gpio);

	plat_data->ipu_id = ipu_id;
	plat_data->disp_id = disp_id;
	if (!strncmp(default_ifmt, "RGB24", 5))
		plat_data->default_ifmt = IPU_PIX_FMT_RGB24;
	else if (!strncmp(default_ifmt, "BGR24", 5))
		plat_data->default_ifmt = IPU_PIX_FMT_BGR24;
	else if (!strncmp(default_ifmt, "GBR24", 5))
		plat_data->default_ifmt = IPU_PIX_FMT_GBR24;
	else if (!strncmp(default_ifmt, "RGB565", 6))
		plat_data->default_ifmt = IPU_PIX_FMT_RGB565;
	else if (!strncmp(default_ifmt, "RGB666", 6))
		plat_data->default_ifmt = IPU_PIX_FMT_RGB666;
	else if (!strncmp(default_ifmt, "YUV444", 6))
		plat_data->default_ifmt = IPU_PIX_FMT_YUV444;
	else if (!strncmp(default_ifmt, "LVDS666", 7))
		plat_data->default_ifmt = IPU_PIX_FMT_LVDS666;
	else if (!strncmp(default_ifmt, "YUYV16", 6))
		plat_data->default_ifmt = IPU_PIX_FMT_YUYV;
	else if (!strncmp(default_ifmt, "UYVY16", 6))
		plat_data->default_ifmt = IPU_PIX_FMT_UYVY;
	else if (!strncmp(default_ifmt, "YVYU16", 6))
		plat_data->default_ifmt = IPU_PIX_FMT_YVYU;
	else if (!strncmp(default_ifmt, "VYUY16", 6))
		plat_data->default_ifmt = IPU_PIX_FMT_VYUY;
	else if (!strncmp(default_ifmt, "BT656", 5))
		plat_data->default_ifmt = IPU_PIX_FMT_BT656;
	else if (!strncmp(default_ifmt, "BT1120", 6))
		plat_data->default_ifmt = IPU_PIX_FMT_BT1120;
	else {
		dev_err(&pdev->dev, "err default_ifmt!\n");
		return -ENOENT;
	}

	return err;
}

static int mxc_lcdif_probe(struct platform_device *pdev)
{
	int ret;
	struct pinctrl *pinctrl;
	struct mxc_lcdif_data *lcdif;
	struct mxc_lcd_platform_data *plat_data;

	dev_dbg(&pdev->dev, "%s enter\n", __func__);
	lcdif = devm_kzalloc(&pdev->dev, sizeof(struct mxc_lcdif_data),
				GFP_KERNEL);
	if (!lcdif)
		return -ENOMEM;
	plat_data = devm_kzalloc(&pdev->dev,
				sizeof(struct mxc_lcd_platform_data),
				GFP_KERNEL);
	if (!plat_data)
		return -ENOMEM;
	pdev->dev.platform_data = plat_data;

	ret = lcd_get_of_property(pdev, plat_data);
	if (ret < 0) {
		dev_err(&pdev->dev, "get lcd of property fail\n");
		return ret;
	}

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		dev_err(&pdev->dev, "can't get/select pinctrl\n");
		return PTR_ERR(pinctrl);
	}

	lcdif->pdev = pdev;
	lcdif->disp_lcdif = mxc_dispdrv_register(&lcdif_drv);
	mxc_dispdrv_setdata(lcdif->disp_lcdif, lcdif);

	dev_set_drvdata(&pdev->dev, lcdif);
	dev_dbg(&pdev->dev, "%s exit\n", __func__);

	return ret;
}

static int mxc_lcdif_remove(struct platform_device *pdev)
{
	struct mxc_lcdif_data *lcdif = dev_get_drvdata(&pdev->dev);

	mxc_dispdrv_puthandle(lcdif->disp_lcdif);
	mxc_dispdrv_unregister(lcdif->disp_lcdif);
	kfree(lcdif);
	return 0;
}

static const struct of_device_id imx_lcd_dt_ids[] = {
	{ .compatible = "fsl,lcd"},
	{ /* sentinel */ }
};
static struct platform_driver mxc_lcdif_driver = {
	.driver = {
		.name = "mxc_lcdif",
		.of_match_table	= imx_lcd_dt_ids,
	},
	.probe = mxc_lcdif_probe,
	.remove = mxc_lcdif_remove,
};

static int __init mxc_lcdif_init(void)
{
	return platform_driver_register(&mxc_lcdif_driver);
}

static void __exit mxc_lcdif_exit(void)
{
	platform_driver_unregister(&mxc_lcdif_driver);
}

module_init(mxc_lcdif_init);
module_exit(mxc_lcdif_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX ipuv3 LCD extern port driver");
MODULE_LICENSE("GPL");
