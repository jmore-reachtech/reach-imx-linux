/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2009-2010 Reach Technology. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/hardware.h>
#include <mach/device.h>
#include <mach/pinctrl.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>

#include "device.h"
#include "mx28_canby.h"
#include "mx28_pins.h"

#define LED_STATUS		MXS_PIN_TO_GPIO(PINID_SPDIF)
#define LCD_DISP_ON		MXS_PIN_TO_GPIO(PINID_GPMI_CE3N)
#define LCD_BL_ENABLE	MXS_PIN_TO_GPIO(PINID_AUART0_CTS)

#if defined(CONFIG_GPIO_PCA953X) || \
	defined(CONFIG_GPIO_PCA953X_MODULE)

#include <linux/i2c/pca953x.h>

#define PCA9534_GPIO_BASE	160
#define PCA9534_PIN(n)		(PCA9534_GPIO_BASE + n)

int pca953x_setup(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context)
{
	gpio_request(PCA9534_PIN(0),"pca-pin0");
	gpio_direction_input(PCA9534_PIN(0));
	gpio_set_value(PCA9534_PIN(0), 1);
	gpio_free(PCA9534_PIN(0));

	gpio_request(PCA9534_PIN(1),"pca-pin1");
	gpio_direction_input(PCA9534_PIN(1));
	gpio_set_value(PCA9534_PIN(1), 1);
	gpio_free(PCA9534_PIN(1));

	gpio_request(PCA9534_PIN(2),"pca-pin2");
	gpio_direction_input(PCA9534_PIN(2));
	gpio_set_value(PCA9534_PIN(2), 1);
	gpio_free(PCA9534_PIN(2));

	gpio_request(PCA9534_PIN(3),"pca-pin3");
	gpio_direction_input(PCA9534_PIN(3));
	gpio_set_value(PCA9534_PIN(3), 1);
	gpio_free(PCA9534_PIN(3));

	gpio_request(PCA9534_PIN(4),"pca-pin4");
	gpio_direction_input(PCA9534_PIN(4));
	gpio_set_value(PCA9534_PIN(4), 1);
	gpio_free(PCA9534_PIN(4));

	gpio_request(PCA9534_PIN(5),"pca-pin5");
	gpio_direction_input(PCA9534_PIN(5));
	gpio_set_value(PCA9534_PIN(5), 1);
	gpio_free(PCA9534_PIN(5));

	gpio_request(PCA9534_PIN(6),"pca-pin6");
	gpio_direction_input(PCA9534_PIN(6));
	gpio_set_value(PCA9534_PIN(6), 1);
	gpio_free(PCA9534_PIN(6));

	gpio_request(PCA9534_PIN(7),"pca-pin7");
	gpio_direction_input(PCA9534_PIN(7));
	gpio_set_value(PCA9534_PIN(7), 1);
	gpio_free(PCA9534_PIN(7));

	return 0;
}

static struct pca953x_platform_data pca9534_data = {
		.invert = 1,
		.gpio_base = PCA9534_GPIO_BASE,
		.setup = &pca953x_setup,
};
#endif

#if defined(CONFIG_TOUCHSCREEN_SITRONIX_I2C_TOUCH) || \
	defined(CONFIG_TOUCHSCREEN_SITRONIX_I2C_TOUCH_MODULE)

#include <linux/input/sitronix_i2c_touch.h>
#include <linux/delay.h>

/*
*  Some constants defined for the PINCTRL banks
*  These banks are defined in the
*  i.mx28 applications processor reference manual
*
* On the board (canby), we have BANK2 PIN19 and PIN20 disabled
* and their pullup registers need to be enabled
*  in order for the signals to be driven
*
* Each pullup bank has 16 bytes of register space and
* the base starts at the MEM_BASE below
*
* The bank2 [pin19 and pin20] are the pull up registers
* for the SSP2_SS0 and SSP2_SS1 pins. The first one is an INT
* pin for the Sitronix touch screen controller and the 2nd
* one is the reset (output) to the Sitronix touch screen controller
*/

#define	I_MX28_HW_PINCTRL_MEM_BASE		0x80018000
#define	I_MX28_HW_PINCTRL_MEM_RANGE_PER_BANK	0x10

/*
* On the canby board for reachtech using the i.mx28 app processor,
* the SSP2_SS0 is an input and is used as WAKE/INT from the touch controller
*   This is pin 19 of bank2. Each bank is 32 pins.
*
* the SSP2_SS1 is an output and is used to reset the touch controller
*   This is pin 20 of bank2.
*/
#define	MULTITOUCH_RESET_GPIO	MXS_PIN_ENCODE(0x2, 20) /* bank2 starts at 64 and this is pin 20*/
#define MULTITOUCH_INT_GPIO		MXS_PIN_ENCODE(0x2, 19) /* bank2 starts at 64 and this is pin 19*/


static void sitronix_reset_ic(void)
{
	printk("%s: \n",__func__);
	gpio_request(MXS_PIN_TO_GPIO(MULTITOUCH_RESET_GPIO), "Multitouch reset");
	gpio_direction_output(MXS_PIN_TO_GPIO(MULTITOUCH_RESET_GPIO),1);
	gpio_set_value(MXS_PIN_TO_GPIO(MULTITOUCH_RESET_GPIO),0);
	mdelay(1);
	gpio_set_value(MXS_PIN_TO_GPIO(MULTITOUCH_RESET_GPIO),1);
}

static int sitronix_get_int_status(void)
{
	return gpio_get_value(MXS_PIN_TO_GPIO(MULTITOUCH_INT_GPIO));
}

struct sitronix_i2c_touch_platform_data touch_i2c_conf = {
	.get_int_status=sitronix_get_int_status,
	.reset_ic = sitronix_reset_ic,
};
#endif

static struct i2c_board_info __initdata mxs_i2c_device[] = {
	{ 
		I2C_BOARD_INFO("pcf8523", 0x68), 
		.flags = I2C_M_TEN,
	},
#if defined(CONFIG_GPIO_PCA953X) || \
	defined(CONFIG_GPIO_PCA953X_MODULE)
	{ 
		I2C_BOARD_INFO("pca9534", 0x3E),
		.platform_data = &pca9534_data,
	},
#endif
#if defined(CONFIG_TOUCHSCREEN_SITRONIX_I2C_TOUCH) || \
	defined(CONFIG_TOUCHSCREEN_SITRONIX_I2C_TOUCH_MODULE)
	{
		I2C_BOARD_INFO(SITRONIX_I2C_TOUCH_DRV_NAME, 0x55),
		.irq = MXS_PIN_TO_GPIO(MULTITOUCH_INT_GPIO),
		.platform_data = &touch_i2c_conf,
	},
#endif
};

static void __init i2c_device_init(void)
{
	i2c_register_board_info(1, mxs_i2c_device, ARRAY_SIZE(mxs_i2c_device));
}
#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct flash_platform_data mx28_spi_flash_data = {
	.name = "m25p80",
	.type = "w25x80",
};
#endif

static struct spi_board_info spi_board_info[] __initdata = {
#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
	{
		/* the modalias must be the same as spi device driver name */
		.modalias = "m25p80", /* Name of spi_driver for this device */
		.max_speed_hz = 20000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 1, /* Framework bus number */
		.chip_select = 0, /* Framework chip select. */
		.platform_data = &mx28_spi_flash_data,
	},
#endif
};

static void spi_device_init(void)
{
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
}

static void __init fixup_board(struct machine_desc *desc, struct tag *tags,
			       char **cmdline, struct meminfo *mi)
{
	mx28_set_input_clk(24000000, 24000000, 32000, 50000000);
}

#if defined(CONFIG_LEDS_MXS) || defined(CONFIG_LEDS_MXS_MODULE)
static struct mxs_pwm_led  mx28canby_led_pwm[2] = {
	[0] = {
		.name = "beeper-pwm4",
		.pwm = 4,
		},
	[1] = {
		.name = "beeper-pwm7",
		.pwm = 7,
		},
};

struct mxs_pwm_leds_plat_data mx28canby_led_data = {
	.num = ARRAY_SIZE(mx28canby_led_pwm),
	.leds = mx28canby_led_pwm,
};

static struct resource mx28canby_led_res = {
	.flags = IORESOURCE_MEM,
	.start = PWM_PHYS_ADDR,
	.end   = PWM_PHYS_ADDR + 0x3FFF,
};

static void __init mx28canby_init_leds(void)
{
	struct platform_device *pdev;

	pdev = mxs_get_device("mxs-leds", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;

	pdev->resource = &mx28canby_led_res;
	pdev->num_resources = 1;
	pdev->dev.platform_data = &mx28canby_led_data;
	mxs_add_device(pdev, 3);
}
#else
static void __init mx28canby_init_leds(void)
{
	;
}
#endif

static void led_status_init(void)
{
	gpio_request(LED_STATUS, "led-status");
	gpio_direction_output(LED_STATUS,1);
	gpio_set_value(LED_STATUS, 0);
	gpio_free(LED_STATUS);
}

static void lcd_disp_init(void)
{
	gpio_request(LCD_DISP_ON, "lcd-disp");
	gpio_direction_output(LCD_DISP_ON,1);
	/* disable display for now */
	gpio_set_value(LCD_DISP_ON, 0);
	/* don't free LCD_DISP_ON gpio         */
	/* we need access in the lms430 driver */

	gpio_request(LCD_BL_ENABLE, "lcd-bl");
	gpio_direction_output(LCD_BL_ENABLE,1);
	/* disable backlight for now    */
	gpio_set_value(LCD_BL_ENABLE, 0);
	/* don't free LCD_BL_ENABLE gpio      */
	/* we need access in the mxsfb driver */
}

static void __init mx28canby_device_init(void)
{
	/* Add mx28canby special code */
	i2c_device_init();
	spi_device_init();
	mx28canby_init_leds();
	led_status_init();
}

static void __init mx28canby_init_machine(void)
{
	mx28_pinctrl_init();
	/* Init iram allocate */
#ifdef CONFIG_VECTORS_PHY_ADDR
	/* reserve the first page for irq vector table*/
	iram_init(MX28_OCRAM_PHBASE + PAGE_SIZE, MX28_OCRAM_SIZE - PAGE_SIZE);
#else
	iram_init(MX28_OCRAM_PHBASE, MX28_OCRAM_SIZE);
#endif

	mx28_gpio_init();
	mx28canby_pins_init();
	lcd_disp_init();
	mx28_device_init();
	mx28canby_device_init();
}

MACHINE_START(MX28_CANBY, "Reach Technology Canby-i.MX28")
	.phys_io	= 0x80000000,
	.io_pg_offst	= ((0xf0000000) >> 18) & 0xfffc,
	.boot_params	= 0x40000100,
	.fixup		= fixup_board,
	.map_io		= mx28_map_io,
	.init_irq	= mx28_irq_init,
	.init_machine	= mx28canby_init_machine,
	.timer		= &mx28_timer.timer,
MACHINE_END
