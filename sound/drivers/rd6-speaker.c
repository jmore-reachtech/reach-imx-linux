/*
 * Copyright 2018 Reach Technology.
 *
 * Author: Jeff Horn <jeff.horn@reachtech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/gpio.h>

static int rd6_speaker_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
    u32 amp_enable_gpio = 0;

    amp_enable_gpio = of_get_named_gpio(np, "amp-en-gpio", 0);
    if (!gpio_is_valid(amp_enable_gpio)) {
        dev_err(&pdev->dev, "failed to find amp enable GPIO\n");
		return -EINVAL;
    }

    gpio_request_one(amp_enable_gpio, GPIOF_DIR_OUT, "audio-stdby");
    gpio_set_value(amp_enable_gpio, 1);
    pr_debug("Audio amp enabled \n");

    return 0;
}

static int rd6_speaker_remove(struct platform_device *pdev)
{
    return 0;
}

static const struct of_device_id rd6_speaker_dt_ids[] = {
	{ .compatible = "fsl,rd6-speaker", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rd6_speaker_dt_ids);

static struct platform_driver rd6_speaker_driver = {
    .driver = {
        .name = "rd6-speaker",
        .of_match_table = rd6_speaker_dt_ids,
    },
    .probe = rd6_speaker_probe,
    .remove = rd6_speaker_remove,
};
module_platform_driver(rd6_speaker_driver);


MODULE_AUTHOR("Reach Technology");
MODULE_DESCRIPTION("Freescale i.MX6 Speaker Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:rd6-speaker");
