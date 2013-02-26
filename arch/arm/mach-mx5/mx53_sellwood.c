/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/powerkey.h>
#include <linux/ahci_platform.h>
#include <linux/gpio_keys.h>
#include <linux/mfd/da9052/da9052.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/keypad.h>
#include <asm/mach/flash.h>
#include <mach/memory.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/mxc_dvfs.h>
#include <mach/iomux-mx53.h>
#include <mach/i2c.h>
#include <mach/mxc_iim.h>
#include <mach/check_fuse.h>

#include "crm_regs.h"
#include "devices.h"
#include "usb.h"
#include "pmic.h"

/*!
 * @file mach-mx5/mx53_sellwood.c
 *
 * @brief This file contains MX53 Sellwood board specific initialization routines.
 *
 * @ingroup MSL_MX53
 */

/* MX53 SELLWOOD GPIO PIN configurations */
#define PWM2_BL_EN			(0*32 + 5)	/* GPIO_1_5 */

#define LVDS_POWER_EN		(2*32 + 15)	/* GPIO_3_15 */
#define DISP0_DET_INT		(2*32 + 12)	/* GPIO_3_12 */
#define DISP0_RESET			(2*32 + 13)	/* GPIO_3_13 */

#define HEADPHONE_DEC_B		(1*32 + 5)	/* GPIO_2_5 */
#define USB_PWREN			(6*32 + 8)	/* GPIO_7_8 */
#define USB_RESET			(2*32 + 26)	/* GPIO_3_26 */

#define USER_UI1			(1*32 + 14)	/* GPIO_2_14 */
#define USER_UI2			(1*32 + 15)	/* GPIO_2_15 */
#define MX53_nONKEY			(0*32 + 8)	/* GPIO_1_8 */

#define FEC_RST				(6*32 + 6)	/* GPIO_7_6 */
#define FEC_INT				(1*32 + 4)	/* GPIO_2_4 */

#define USER_LED_EN			(6*32 + 7)	/* GPIO_7_7 */
#define USB_PWREN			(6*32 + 8)	/* GPIO_7_8 */
#define NIRQ				(6*32 + 11)	/* GPIO7_11 */

#define ISP_LS_RST			(4*32 + 22)	/* GPIO5_22 */
#define ISP_LS_MODE1		(4*32 + 23)	/* GPIO5_23 */
#define ISP_LS_MODE0		(4*32 + 24)	/* GPIO5_24 */

#define MX53_OFFSET					(0x20000000)
#define TZIC_WAKEUP0_OFFSET         (0x0E00)
#define TZIC_WAKEUP1_OFFSET         (0x0E04)
#define TZIC_WAKEUP2_OFFSET         (0x0E08)
#define TZIC_WAKEUP3_OFFSET         (0x0E0C)
#define GPIO7_0_11_IRQ_BIT			(0x1<<11)

#define AUDIO_AMP_EN		(2*32 + 31)	/* GPIO_3_31 */

extern void pm_i2c_init(u32 base_addr);

static iomux_v3_cfg_t mx53_sellwood_pads[] = {
	/* FEC */
	MX53_PAD_FEC_MDC__FEC_MDC,
	MX53_PAD_FEC_MDIO__FEC_MDIO,
	MX53_PAD_FEC_REF_CLK__FEC_TX_CLK,
	MX53_PAD_FEC_RX_ER__FEC_RX_ER,
	MX53_PAD_FEC_CRS_DV__FEC_RX_DV,
	MX53_PAD_FEC_RXD1__FEC_RDATA_1,
	MX53_PAD_FEC_RXD0__FEC_RDATA_0,
	MX53_PAD_FEC_TX_EN__FEC_TX_EN,
	MX53_PAD_FEC_TXD1__FEC_TDATA_1,
	MX53_PAD_FEC_TXD0__FEC_TDATA_0,
	/* FEC_nRST */
	MX53_PAD_PATA_DA_0__GPIO7_6,
	/* FEC_nINT */
	MX53_PAD_PATA_DATA4__GPIO2_4,
	/* AUDMUX5 */
	MX53_PAD_KEY_COL0__AUDMUX_AUD5_TXC,
	MX53_PAD_KEY_ROW0__AUDMUX_AUD5_TXD,
	MX53_PAD_KEY_COL1__AUDMUX_AUD5_TXFS,
	MX53_PAD_KEY_ROW1__AUDMUX_AUD5_RXD,
	/* I2C2 */
	MX53_PAD_KEY_COL3__I2C2_SCL,
	MX53_PAD_KEY_ROW3__I2C2_SDA,
	/* I2C3 */
	MX53_PAD_GPIO_3__I2C3_SCL,
	MX53_PAD_GPIO_6__I2C3_SDA,
	/* SD1 */
	MX53_PAD_SD1_CMD__ESDHC1_CMD,
	MX53_PAD_SD1_CLK__ESDHC1_CLK,
	MX53_PAD_SD1_DATA0__ESDHC1_DAT0,
	MX53_PAD_SD1_DATA1__ESDHC1_DAT1,
	MX53_PAD_SD1_DATA2__ESDHC1_DAT2,
	MX53_PAD_SD1_DATA3__ESDHC1_DAT3,
	/* SD2 */
	MX53_PAD_SD2_CMD__ESDHC2_CMD,
	MX53_PAD_SD2_CLK__ESDHC2_CLK,
	MX53_PAD_SD2_DATA0__ESDHC2_DAT0,
	MX53_PAD_SD2_DATA1__ESDHC2_DAT1,
	MX53_PAD_SD2_DATA2__ESDHC2_DAT2,
	MX53_PAD_SD2_DATA3__ESDHC2_DAT3,
	/* VGA */
	MX53_PAD_EIM_OE__IPU_DI1_PIN7,
	MX53_PAD_EIM_RW__IPU_DI1_PIN8,
	/* DISPLB */
	MX53_PAD_EIM_D20__IPU_SER_DISP0_CS,
	MX53_PAD_EIM_D21__IPU_DISPB0_SER_CLK,
	MX53_PAD_EIM_D22__IPU_DISPB0_SER_DIN,
	MX53_PAD_EIM_D23__IPU_DI0_D0_CS,
	/* DISP0_POWER_EN */
	MX53_PAD_EIM_DA15__GPIO3_15,
	/* DISP0 DET INT */
	MX53_PAD_EIM_DA12__GPIO3_12,
	/* DISP0 RESET */
	MX53_PAD_EIM_DA13__GPIO3_13,
	/* USB RESET */
	MX53_PAD_EIM_D26__GPIO3_26,
	/* AUDIO */
	MX53_PAD_EIM_D31__GPIO3_31,
	/* flexcan0 */
	MX53_PAD_GPIO_7__CAN1_TXCAN,
	MX53_PAD_GPIO_8__CAN1_RXCAN,
	/* flexcan1 */
	MX53_PAD_PATA_RESET_B__CAN2_TXCAN,
	MX53_PAD_PATA_IORDY__CAN2_RXCAN,
	/* LVDS */
	MX53_PAD_LVDS0_TX3_P__LDB_LVDS0_TX3,
	MX53_PAD_LVDS0_CLK_P__LDB_LVDS0_CLK,
	MX53_PAD_LVDS0_TX2_P__LDB_LVDS0_TX2,
	MX53_PAD_LVDS0_TX1_P__LDB_LVDS0_TX1,
	MX53_PAD_LVDS0_TX0_P__LDB_LVDS0_TX0,
	MX53_PAD_LVDS1_TX3_P__LDB_LVDS1_TX3,
	MX53_PAD_LVDS1_TX2_P__LDB_LVDS1_TX2,
	MX53_PAD_LVDS1_CLK_P__LDB_LVDS1_CLK,
	MX53_PAD_LVDS1_TX1_P__LDB_LVDS1_TX1,
	MX53_PAD_LVDS1_TX0_P__LDB_LVDS1_TX0,
	/* I2C1 */
	MX53_PAD_CSI0_DAT8__I2C1_SDA,
	MX53_PAD_CSI0_DAT9__I2C1_SCL,
	/* UART1 */
	MX53_PAD_CSI0_DAT10__UART1_TXD_MUX,
	MX53_PAD_CSI0_DAT11__UART1_RXD_MUX,
	/* UART2 */
	MX53_PAD_PATA_DMARQ__UART2_TXD_MUX,
	MX53_PAD_PATA_BUFFER_EN__UART2_RXD_MUX,
	/* UART3 */
	MX53_PAD_PATA_CS_0__UART3_TXD_MUX,
	MX53_PAD_PATA_CS_1__UART3_RXD_MUX,
	/* CSI0 */
	MX53_PAD_CSI0_DAT12__IPU_CSI0_D_12,
	MX53_PAD_CSI0_DAT13__IPU_CSI0_D_13,
	MX53_PAD_CSI0_DAT14__IPU_CSI0_D_14,
	MX53_PAD_CSI0_DAT15__IPU_CSI0_D_15,
	MX53_PAD_CSI0_DAT16__IPU_CSI0_D_16,
	MX53_PAD_CSI0_DAT17__IPU_CSI0_D_17,
	MX53_PAD_CSI0_DAT18__IPU_CSI0_D_18,
	MX53_PAD_CSI0_DAT19__IPU_CSI0_D_19,
	MX53_PAD_CSI0_VSYNC__IPU_CSI0_VSYNC,
	MX53_PAD_CSI0_MCLK__IPU_CSI0_HSYNC,
	MX53_PAD_CSI0_PIXCLK__IPU_CSI0_PIXCLK,
	/* DISPLAY */
	MX53_PAD_DI0_DISP_CLK__IPU_DI0_DISP_CLK,
	MX53_PAD_DI0_PIN15__IPU_DI0_PIN15,
	MX53_PAD_DI0_PIN2__IPU_DI0_PIN2,
	MX53_PAD_DI0_PIN3__IPU_DI0_PIN3,
	MX53_PAD_DISP0_DAT0__IPU_DISP0_DAT_0,
	MX53_PAD_DISP0_DAT1__IPU_DISP0_DAT_1,
	MX53_PAD_DISP0_DAT2__IPU_DISP0_DAT_2,
	MX53_PAD_DISP0_DAT3__IPU_DISP0_DAT_3,
	MX53_PAD_DISP0_DAT4__IPU_DISP0_DAT_4,
	MX53_PAD_DISP0_DAT5__IPU_DISP0_DAT_5,
	MX53_PAD_DISP0_DAT6__IPU_DISP0_DAT_6,
	MX53_PAD_DISP0_DAT7__IPU_DISP0_DAT_7,
	MX53_PAD_DISP0_DAT8__IPU_DISP0_DAT_8,
	MX53_PAD_DISP0_DAT9__IPU_DISP0_DAT_9,
	MX53_PAD_DISP0_DAT10__IPU_DISP0_DAT_10,
	MX53_PAD_DISP0_DAT11__IPU_DISP0_DAT_11,
	MX53_PAD_DISP0_DAT12__IPU_DISP0_DAT_12,
	MX53_PAD_DISP0_DAT13__IPU_DISP0_DAT_13,
	MX53_PAD_DISP0_DAT14__IPU_DISP0_DAT_14,
	MX53_PAD_DISP0_DAT15__IPU_DISP0_DAT_15,
	MX53_PAD_DISP0_DAT16__IPU_DISP0_DAT_16,
	MX53_PAD_DISP0_DAT17__IPU_DISP0_DAT_17,
	MX53_PAD_DISP0_DAT18__IPU_DISP0_DAT_18,
	MX53_PAD_DISP0_DAT19__IPU_DISP0_DAT_19,
	MX53_PAD_DISP0_DAT20__IPU_DISP0_DAT_20,
	MX53_PAD_DISP0_DAT21__IPU_DISP0_DAT_21,
	MX53_PAD_DISP0_DAT22__IPU_DISP0_DAT_22,
	MX53_PAD_DISP0_DAT23__IPU_DISP0_DAT_23,
	/* Audio CLK*/
	MX53_PAD_GPIO_0__CCM_SSI_EXT1_CLK,
	/* PWM2 BL EN */
	MX53_PAD_GPIO_5__GPIO1_5,
	/* PWM2 BL  */
	MX53_PAD_GPIO_1__PWM2_PWMO,
	/* SPDIF */
	MX53_PAD_GPIO_7__SPDIF_PLOCK,
	MX53_PAD_GPIO_17__SPDIF_OUT1,
	/* GPIO */
	MX53_PAD_PATA_DA_1__GPIO7_7,
	MX53_PAD_PATA_DA_2__GPIO7_8,
	MX53_PAD_PATA_DATA5__GPIO2_5,
	MX53_PAD_PATA_DATA6__GPIO2_6,
	MX53_PAD_PATA_DATA14__GPIO2_14,
	MX53_PAD_PATA_DATA15__GPIO2_15,
	MX53_PAD_PATA_INTRQ__GPIO7_2,
	MX53_PAD_EIM_WAIT__GPIO5_0,
	MX53_PAD_NANDF_WP_B__GPIO6_9,
	MX53_PAD_NANDF_RB0__GPIO6_10,
	MX53_PAD_NANDF_CS1__GPIO6_14,
	MX53_PAD_NANDF_CS2__GPIO6_15,
	MX53_PAD_NANDF_CS3__GPIO6_16,
	MX53_PAD_GPIO_5__GPIO1_5,
	MX53_PAD_GPIO_16__GPIO7_11,
	MX53_PAD_GPIO_8__GPIO1_8,
	MX53_PAD_CSI0_DAT4__GPIO5_22,
	MX53_PAD_CSI0_DAT5__GPIO5_23,
	MX53_PAD_CSI0_DAT6__GPIO5_24,
};

static void sellwood_da9053_irq_wakeup_only_fixup(void)
{
	void __iomem *tzic_base;
	tzic_base = ioremap(MX53_TZIC_BASE_ADDR, SZ_4K);
	if (NULL == tzic_base) {
		pr_err("fail to map MX53_TZIC_BASE_ADDR\n");
		return;
	}
	__raw_writel(0, tzic_base + TZIC_WAKEUP0_OFFSET);
	__raw_writel(0, tzic_base + TZIC_WAKEUP1_OFFSET);
	__raw_writel(0, tzic_base + TZIC_WAKEUP2_OFFSET);
	/* only enable irq wakeup for da9053 */
	__raw_writel(GPIO7_0_11_IRQ_BIT, tzic_base + TZIC_WAKEUP3_OFFSET);
	iounmap(tzic_base);
	pr_info("only da9053 irq is wakeup-enabled\n");
}

static void sellwood_suspend_enter(void)
{
	if (!board_is_mx53_loco_mc34708()) {
		sellwood_da9053_irq_wakeup_only_fixup();
		da9053_suspend_cmd_sw();
	}
}

static void sellwood_suspend_exit(void)
{
	if (!board_is_mx53_loco_mc34708()) {
		if (da9053_get_chip_version())
			da9053_restore_volt_settings();
	}
}

static struct mxc_pm_platform_data sellwood_pm_data = {
	.suspend_enter = sellwood_suspend_enter,
	.suspend_exit = sellwood_suspend_exit,
};

static struct fb_videomode video_modes[] = {
	{
				.name 			= "XGA",
				.refresh		= 60,
				.xres			= 1024,
				.yres			= 768,
				.pixclock		= 15385,
				.left_margin	= 220,
				.right_margin	= 40,
				.upper_margin	= 21,
				.lower_margin	= 7,
				.hsync_len		= 60,
				.vsync_len		= 10,
				.sync 			= 0,
				.vmode			= FB_VMODE_NONINTERLACED,
				.flag			= FB_MODE_IS_DETAILED,
		},
		{
				.name 			= "SEIKO-WVGA",
				.refresh		= 60,
				.xres			= 800,
				.yres			= 480,
				.pixclock		= 29850,
				.left_margin	= 89,
				.right_margin	= 164,
				.upper_margin	= 23,
				.lower_margin	= 10,
				.hsync_len		= 10,
				.vsync_len		= 10,
				.sync 			= FB_SYNC_CLK_LAT_FALL,
				.vmode			= FB_VMODE_NONINTERLACED,
				.flag			= 0,
		},
		{
				.name 			= "UXGA",
				.refresh		= 60,
				.xres			= 1600,
				.yres			= 1200,
				.pixclock		= 6172,
				.left_margin	= 304,
				.right_margin	= 64,
				.upper_margin	= 1,
				.lower_margin	= 46,
				.hsync_len		= 192,
				.vsync_len		= 3,
				.sync 			= FB_SYNC_HOR_HIGH_ACT|FB_SYNC_VERT_HIGH_ACT,
				.vmode			= FB_VMODE_NONINTERLACED,
				.flag			= 0,
		},
};

static struct platform_pwm_backlight_data mxc_pwm_backlight_data = {
	.pwm_id = 1,
	.max_brightness = 255,
	.dft_brightness = 255,
	.pwm_period_ns = 5200000,
};

extern void mx5_ipu_reset(void);
static struct mxc_ipu_config mxc_ipu_data = {
	.rev = 3,
	.reset = mx5_ipu_reset,
};

extern void mx5_vpu_reset(void);
static struct mxc_vpu_platform_data mxc_vpu_data = {
	.iram_enable = true,
	.iram_size = 0x14000,
	.reset = mx5_vpu_reset,
};

static struct fec_platform_data fec_data = {
	.phy = PHY_INTERFACE_MODE_RMII,
};

static struct mxc_dvfs_platform_data dvfs_core_data = {
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.gpc_vcr_offset = MXC_GPC_VCR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 30,
};

static struct mxc_bus_freq_platform_data bus_freq_data;

static struct tve_platform_data tve_data = {
	.boot_enable = MXC_TVE_VGA,
};

static struct ldb_platform_data ldb_data = {
	.ext_ref = 1,
};

static void mxc_iim_enable_fuse(void)
{
	u32 reg;

	if (!ccm_base)
		return;

	/* enable fuse blown */
	reg = readl(ccm_base + 0x64);
	reg |= 0x10;
	writel(reg, ccm_base + 0x64);
}

static void mxc_iim_disable_fuse(void)
{
	u32 reg;

	if (!ccm_base)
		return;
	/* enable fuse blown */
	reg = readl(ccm_base + 0x64);
	reg &= ~0x10;
	writel(reg, ccm_base + 0x64);
}

static struct mxc_iim_data iim_data = {
	.bank_start = MXC_IIM_MX53_BANK_START_ADDR,
	.bank_end   = MXC_IIM_MX53_BANK_END_ADDR,
	.enable_fuse = mxc_iim_enable_fuse,
	.disable_fuse = mxc_iim_disable_fuse,
};

static struct resource mxcfb_resources[] = {
	[0] = {
	       .flags = IORESOURCE_MEM,
	       },
};

static struct mxc_fb_platform_data fb_data[] = {
	{
	 .interface_pix_fmt = IPU_PIX_FMT_RGB24,
	 .mode_str = "XGA",
	 .mode = video_modes,
	 .num_modes = ARRAY_SIZE(video_modes),
	},
	{
	 .interface_pix_fmt = IPU_PIX_FMT_RGB24,
	 .mode_str = "XGA",
	 .mode = video_modes,
	 .num_modes = ARRAY_SIZE(video_modes),
	},
};

static struct flexcan_platform_data flexcan_data[] = {

};

extern int primary_di;
static int __init mxc_init_fb(void)
{
	if (!machine_is_mx53_sellwood())
		return 0;

	if (primary_di) {
		printk(KERN_INFO "DI1 is primary\n");
		/* DI1 -> DP-BG channel: */
		mxc_fb_devices[1].num_resources = ARRAY_SIZE(mxcfb_resources);
		mxc_fb_devices[1].resource = mxcfb_resources;
		mxc_register_device(&mxc_fb_devices[1], &fb_data[1]);

		/* DI0 -> DC channel: */
		mxc_register_device(&mxc_fb_devices[0], &fb_data[0]);
	} else {
		printk(KERN_INFO "DI0 is primary\n");

		/* DI0 -> DP-BG channel: */
		mxc_fb_devices[0].num_resources = ARRAY_SIZE(mxcfb_resources);
		mxc_fb_devices[0].resource = mxcfb_resources;
		mxc_register_device(&mxc_fb_devices[0], &fb_data[0]);

		/* DI1 -> DC channel: */
		mxc_register_device(&mxc_fb_devices[1], &fb_data[1]);
	}

	/*
	 * DI0/1 DP-FG channel:
	 */
	mxc_register_device(&mxc_fb_devices[2], NULL);

	return 0;
}
device_initcall(mxc_init_fb);

static void sii902x_hdmi_reset(void)
{
	gpio_set_value(DISP0_RESET, 0);
	msleep(10);
	gpio_set_value(DISP0_RESET, 1);
	msleep(10);
}

static int sii902x_get_pins(void)
{
	/* Sii902x HDMI controller */
	gpio_request(DISP0_RESET, "disp0-reset");
	gpio_direction_output(DISP0_RESET, 0);
	gpio_request(DISP0_DET_INT, "disp0-detect");
	gpio_direction_input(DISP0_DET_INT);
	return 1;
}

static void sii902x_put_pins(void)
{
	gpio_free(DISP0_RESET);
	gpio_free(DISP0_DET_INT);
}

static struct mxc_lcd_platform_data sii902x_hdmi_data = {
	.reset = sii902x_hdmi_reset,
	.fb_id = "DISP3 BG",
	.get_pins = sii902x_get_pins,
	.put_pins = sii902x_put_pins,
};

static struct imxi2c_platform_data mxci2c_data = {
       .bitrate = 100000,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("pcf8523", 0x68),
		.flags = I2C_M_TEN,
	},
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {

};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
	 .type = "sgtl5000-i2c",
	 .addr = 0x0a,
	 },
	{
	 .type = "sii902x",
	 .addr = 0x39,
	 .irq = gpio_to_irq(DISP0_DET_INT),
	 .platform_data = &sii902x_hdmi_data,
	},
};

static struct mxc_mmc_platform_data mmc1_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 1,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};

static struct mxc_mmc_platform_data mmc2_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 1,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};

static int headphone_det_status(void)
{
	return (gpio_get_value(HEADPHONE_DEC_B) == 0);
}

static int mxc_sgtl5000_init(void);
static int mxc_sgtl5000_amp_enable(int);

static struct mxc_audio_platform_data sgtl5000_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 5,
	.hp_irq = gpio_to_irq(HEADPHONE_DEC_B),
	.hp_status = headphone_det_status,
	.init = mxc_sgtl5000_init,
	.ext_ram_rx = 1,
	.amp_enable = mxc_sgtl5000_amp_enable,
};

static int mxc_sgtl5000_amp_enable(int enable)
{
	gpio_request(AUDIO_AMP_EN, "audio-power-en");
	gpio_set_value(AUDIO_AMP_EN,enable);

	return 0;
}

static int mxc_sgtl5000_init(void)
{
	struct clk *ssi_ext1;
	int rate;

	ssi_ext1 = clk_get(NULL, "ssi_ext1_clk");
	if (IS_ERR(ssi_ext1))
			return -1;

	rate = clk_round_rate(ssi_ext1, 24000000);
	if (rate < 8000000 || rate > 27000000) {
			printk(KERN_ERR "Error: SGTL5000 mclk freq %d out of range!\n",
				   rate);
			clk_put(ssi_ext1);
			return -1;
	}

	clk_set_rate(ssi_ext1, rate);
	clk_enable(ssi_ext1);
	sgtl5000_data.sysclk = rate;

	return 0;
}

static struct platform_device mxc_sgtl5000_device = {
	.name = "imx-3stack-sgtl5000",
};

static struct mxc_asrc_platform_data mxc_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx = 1,
	.spdif_rx = 0,
	.spdif_clk_44100 = -1,	/* Souce from CKIH1 for 44.1K */
	/* Source from CCM spdif_clk (24M) for 48k and 32k
	 * It's not accurate
	 */
	.spdif_clk_48000 = 1,
	.spdif_clkid = 0,
	.spdif_clk = NULL,	/* spdif bus clk */
};

static struct mxc_audio_platform_data spdif_audio_data = {
	.ext_ram_tx = 1,
};

static struct platform_device mxc_spdif_audio_device = {
	.name = "imx-spdif-audio-device",
};

static void mx53_sellwood_usbh1_vbus(bool on)
{
	if (on)
		gpio_set_value(USB_PWREN, 1);
	else
		gpio_set_value(USB_PWREN, 0);
}

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake)	\
{								\
	.gpio		= gpio_num,				\
	.type		= EV_KEY,				\
	.code		= ev_code,				\
	.active_low	= act_low,				\
	.desc		= "btn " descr,				\
	.wakeup		= wake,					\
}

static struct gpio_keys_button sellwood_buttons[] = {
	GPIO_BUTTON(MX53_nONKEY, KEY_POWER, 1, "power", 0),
	GPIO_BUTTON(USER_UI1, KEY_VOLUMEUP, 1, "volume-up", 0),
	GPIO_BUTTON(USER_UI2, KEY_VOLUMEDOWN, 1, "volume-down", 0),
};

static struct gpio_keys_platform_data sellwood_button_data = {
	.buttons	= sellwood_buttons,
	.nbuttons	= ARRAY_SIZE(sellwood_buttons),
};

static struct platform_device sellwood_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &sellwood_button_data,
	}
};

static void __init sellwood_add_device_buttons(void)
{
	platform_device_register(&sellwood_button_device);
}
#else
static void __init sellwood_add_device_buttons(void) {}
#endif

static void mxc_register_powerkey(pwrkey_callback pk_cb)
{
	pmic_event_callback_t power_key_event;

	power_key_event.param = (void *)1;
	power_key_event.func = (void *)pk_cb;
	pmic_event_subscribe(EVENT_PWRONI, power_key_event);
}

static int mxc_pwrkey_getstatus(int id)
{
	int sense;

	pmic_read_reg(REG_INT_SENSE1, &sense, 0xffffffff);
	if (sense & (1 << 3))
		return 0;

	return 1;
}

static struct power_key_platform_data pwrkey_data = {
	.key_value = KEY_F4,
	.register_pwrkey = mxc_register_powerkey,
	.get_key_status = mxc_pwrkey_getstatus,
};

/*!
 * Board specific fixup function. It is called by \b setup_arch() in
 * setup.c file very early on during kernel starts. It allows the user to
 * statically fill in the proper values for the passed-in parameters. None of
 * the parameters is used currently.
 *
 * @param  desc         pointer to \b struct \b machine_desc
 * @param  tags         pointer to \b struct \b tag
 * @param  cmdline      pointer to the command line
 * @param  mi           pointer to \b struct \b meminfo
 */
static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	struct tag *t;
	struct tag *mem_tag = 0;
	int total_mem = SZ_1G;
	int left_mem = 0;
	int gpu_mem = SZ_128M;
	int fb_mem = SZ_32M;
	char *str;
	int i = 0;
	struct fb_videomode mode;
	struct mxc_fb_platform_data mxc_mode;
	int modelines[13];
	char *end;

	mxc_set_cpu_type(MXC_CPU_MX53);

	for_each_tag(mem_tag, tags) {
		if (mem_tag->hdr.tag == ATAG_MEM) {
			total_mem = mem_tag->u.mem.size;
			break;
		}
	}

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "mem=");
			if (str != NULL) {
				str += 4;
				left_mem = memparse(str, &str);
			}

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_nommu");
			if (str != NULL)
				gpu_data.enable_mmu = 0;

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_memory=");
			if (str != NULL) {
				str += 11;
				gpu_mem = memparse(str, &str);
			}

			break;
		}
	}

	if (gpu_data.enable_mmu)
		gpu_mem = 0;

	if (left_mem == 0 || left_mem > total_mem)
		left_mem = total_mem - gpu_mem - fb_mem;

	if (mem_tag) {
		fb_mem = total_mem - left_mem - gpu_mem;
		if (fb_mem < 0) {
			gpu_mem = total_mem - left_mem;
			fb_mem = 0;
		}
		mem_tag->u.mem.size = left_mem;

		/*reserve memory for gpu*/
		if (!gpu_data.enable_mmu) {
			gpu_device.resource[5].start =
				mem_tag->u.mem.start + left_mem;
			gpu_device.resource[5].end =
				gpu_device.resource[5].start + gpu_mem - 1;
		}
#if defined(CONFIG_FB_MXC_SYNC_PANEL) || \
	defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
		if (fb_mem) {
			mxcfb_resources[0].start =
				gpu_data.enable_mmu ?
				mem_tag->u.mem.start + left_mem :
				gpu_device.resource[5].end + 1;
			mxcfb_resources[0].end =
				mxcfb_resources[0].start + fb_mem - 1;
		} else {
			mxcfb_resources[0].start = 0;
			mxcfb_resources[0].end = 0;
		}
#endif
	}

	/* pull out panel timings */
	/* panel=60,800,480,33303,40,206,10,25,10,10,0,0,0 */
	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "panel=");
			if(str == NULL) {
				printk("setup: panel setting not found\n");
				return;
			}

			str += 6;
			memset(modelines,0,ARRAY_SIZE(modelines));
			memset(&mode,0,sizeof(struct fb_videomode));
			printk("panel= %s\n",str);
			end = get_options(str, ARRAY_SIZE(modelines)+1, modelines);

			for(i=1;i<=ARRAY_SIZE(modelines);i++) {
				switch(i) {
					case 1: mode.refresh = modelines[i];
						printk("refresh=%d\n",mode.refresh);break;
					case 2: mode.xres = modelines[i];
						printk("xres=%d\n",mode.xres);break;
					case 3: mode.yres = modelines[i];
						printk("yres=%d\n",mode.yres);break;
					case 4: mode.pixclock = modelines[i];
						printk("pixclock=%d\n",mode.pixclock);break;
					case 5: mode.left_margin = modelines[i];
						printk("left_margin=%d\n",mode.left_margin);break;
					case 6: mode.right_margin = modelines[i];
						printk("right_margin=%d\n",mode.right_margin);break;
					case 7: mode.upper_margin = modelines[i];
						printk("upper_margin=%d\n",mode.upper_margin);break;
					case 8: mode.lower_margin = modelines[i];
						printk("lower_margin=%d\n",mode.lower_margin);break;
					case 9: mode.hsync_len = modelines[i];
						printk("hsync_len=%d\n",mode.hsync_len);break;
					case 10: mode.vsync_len = modelines[i];
						printk("vsync_len=%d\n",mode.hsync_len);break;
					case 11: mode.sync = modelines[i];
						printk("sync=%d\n",mode.sync);break;
					case 12: mode.vmode = modelines[i];
						printk("vmode=%d\n",mode.vmode);break;
					case 13: mode.flag = modelines[i];
						printk("flag=%d\n",mode.flag);break;
				}
			}
			mode.name = "R-VGA";
			// register our video mode
			video_modes[0] = mode;
		}
	}

	// parse out the video option
	// video=mxcdi0fb:RGB666,R-WVGA
	//we default to RGB24
	mxc_mode.interface_pix_fmt = IPU_PIX_FMT_RGB24;
	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, ":RGB");
			printk("rgb=%s\n",str);
			str += 1;
			if(!strncmp(str, "RGB666", 6)) {
				printk("found RGB666\n");
				mxc_mode.interface_pix_fmt = IPU_PIX_FMT_RGB666;
			} else if(!strncmp(str, "RGB565", 6)) {
				printk("found RGB565\n");
				mxc_mode.interface_pix_fmt = IPU_PIX_FMT_RGB565;
			} else {
				printk("defaulting to RGB24\n");
			}
		}
	}
	mxc_mode.mode_str = "R-VGA";
	mxc_mode.mode = video_modes;
	mxc_mode.num_modes = ARRAY_SIZE(video_modes);
	fb_data[0] = mxc_mode;
}

static void __init mx53_sellwood_io_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx53_sellwood_pads,
					ARRAY_SIZE(mx53_sellwood_pads));

	/* reset FEC PHY */
	gpio_request(FEC_RST, "fec-rst");
	gpio_direction_output(FEC_RST, 0);
	gpio_set_value(FEC_RST, 0);
	msleep(1);
	gpio_set_value(FEC_RST, 1);

	/* headphone_det_b */
	gpio_request(HEADPHONE_DEC_B, "headphone-dec");
	gpio_direction_input(HEADPHONE_DEC_B);

	/* USB PWR enable */
	gpio_request(USB_PWREN, "usb-pwr");
	gpio_direction_output(USB_PWREN, 0);

	/* Sii902x HDMI controller */
	gpio_request(DISP0_RESET, "disp0-reset");
	gpio_direction_output(DISP0_RESET, 0);
	gpio_request(DISP0_DET_INT, "disp0-detect");
	gpio_direction_input(DISP0_DET_INT);

	/* LVDS panel power enable */
	gpio_request(LVDS_POWER_EN, "disp0-power-en");
	gpio_direction_output(LVDS_POWER_EN, 1);

	/* PWM2 backlight enable */
	gpio_request(PWM2_BL_EN, "pwm2-bl-power-en");
	gpio_direction_output(PWM2_BL_EN, 1);

	/* Audio power enable */
	gpio_request(AUDIO_AMP_EN, "audio-power-en");
	gpio_direction_output(AUDIO_AMP_EN, 1);
	gpio_set_value(AUDIO_AMP_EN, 0);

}

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	iomux_v3_cfg_t da9052_csi0_d12;

	mxc_ipu_data.di_clk[0] = clk_get(NULL, "ipu_di0_clk");
	mxc_ipu_data.di_clk[1] = clk_get(NULL, "ipu_di1_clk");
	mxc_ipu_data.csi_clk[0] = clk_get(NULL, "ssi_ext1_clk");
	mxc_spdif_data.spdif_core_clk = clk_get(NULL, "spdif_xtal_clk");
	clk_put(mxc_spdif_data.spdif_core_clk);

	mxc_cpu_common_init();
	mx53_sellwood_io_init();

	mxc_register_device(&mxc_dma_device, NULL);
	mxc_register_device(&mxc_wdt_device, NULL);
	mxc_register_device(&mxci2c_devices[0], &mxci2c_data);
	mxc_register_device(&mxci2c_devices[1], &mxci2c_data);
	mxc_register_device(&mxci2c_devices[2], &mxci2c_data);

	da9052_csi0_d12 = MX53_PAD_CSI0_DAT12__IPU_CSI0_D_12;
	mxc_iomux_v3_setup_pad(da9052_csi0_d12);
	mx53_loco_init_da9052();
	dvfs_core_data.reg_id = "DA9052_BUCK_CORE";
	tve_data.dac_reg = "DA9052_LDO7";
	bus_freq_data.gp_reg_id = "DA9052_BUCK_CORE";
	bus_freq_data.lp_reg_id = "DA9052_BUCK_PRO";

	mxc_register_device(&mxc_rtc_device, NULL);
	mxc_register_device(&mxc_ipu_device, &mxc_ipu_data);
	mxc_register_device(&mxc_ldb_device, &ldb_data);
	mxc_register_device(&mxc_tve_device, &tve_data);
	if (!mxc_fuse_get_vpu_status())
		mxc_register_device(&mxcvpu_device, &mxc_vpu_data);
	if (!mxc_fuse_get_gpu_status())
		mxc_register_device(&gpu_device, &gpu_data);
	mxc_register_device(&mxcscc_device, NULL);
	mxc_register_device(&pm_device, &sellwood_pm_data);
	mxc_register_device(&mxc_dvfs_core_device, &dvfs_core_data);
	mxc_register_device(&busfreq_device, &bus_freq_data);
	mxc_register_device(&mxc_iim_device, &iim_data);
	mxc_register_device(&mxc_pwm2_device, NULL);
	mxc_register_device(&mxc_pwm1_backlight_device, &mxc_pwm_backlight_data);
	mxc_register_device(&mxcsdhc1_device, &mmc1_data);
	mxc_register_device(&mxcsdhc2_device, &mmc2_data);
	mxc_register_device(&mxc_ssi1_device, NULL);
	mxc_register_device(&mxc_ssi2_device, NULL);
	mxc_register_device(&mxc_alsa_spdif_device, &mxc_spdif_data);
	mxc_register_device(&ahci_fsl_device, &sata_data);
	mxc_register_device(&imx_ahci_device_hwmon, NULL);
	mxc_register_device(&mxc_fec_device, &fec_data);
	mxc_register_device(&mxc_ptp_device, NULL);
	/* ASRC is only available for MX53 TO2.0 */
	if (mx53_revision() >= IMX_CHIP_REVISION_2_0) {
		mxc_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
		clk_put(mxc_asrc_data.asrc_core_clk);
		mxc_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
		clk_put(mxc_asrc_data.asrc_audio_clk);
		mxc_register_device(&mxc_asrc_device, &mxc_asrc_data);
	}

	//mxc_register_device(&mxc_flexcan0_device, &flexcan_data);
	//mxc_register_device(&mxc_flexcan1_device, &flexcan_data);

	i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
				ARRAY_SIZE(mxc_i2c2_board_info));

	sgtl5000_data.ext_ram_clk = clk_get(NULL, "emi_fast_clk");
	clk_put(sgtl5000_data.ext_ram_clk);
	mxc_register_device(&mxc_sgtl5000_device, &sgtl5000_data);

	spdif_audio_data.ext_ram_clk = clk_get(NULL, "emi_fast_clk");
	clk_put(spdif_audio_data.ext_ram_clk);
	mxc_register_device(&mxc_spdif_audio_device, &spdif_audio_data);

	mx5_usb_dr_init();
	mx5_set_host1_vbus_func(mx53_sellwood_usbh1_vbus);
	mx5_usbh1_init();
	mxc_register_device(&mxc_v4l2_device, NULL);
	mxc_register_device(&mxc_v4l2out_device, NULL);
	sellwood_add_device_buttons();
	pm_power_off = da9053_power_off;
	pm_i2c_init(I2C1_BASE_ADDR - MX53_OFFSET);
}

static void __init mx53_sellwood_timer_init(void)
{
	struct clk *uart_clk;

	mx53_clocks_init(32768, 24000000, 0, 0);

#if defined CONFIG_MX53_SELLWOOD_DEBUG_UART0
	uart_clk = clk_get_sys("mxcintuart.0", NULL);
	early_console_setup(MX53_BASE_ADDR(UART1_BASE_ADDR), uart_clk);
#endif
#if defined CONFIG_MX53_SELLWOOD_DEBUG_UART1
	uart_clk = clk_get_sys("mxcintuart.1", NULL);
	early_console_setup(MX53_BASE_ADDR(UART2_BASE_ADDR), uart_clk);
#endif
#if defined CONFIG_MX53_SELLWOOD_DEBUG_UART2
	uart_clk = clk_get_sys("mxcintuart.2", NULL);
	early_console_setup(MX53_BASE_ADDR(UART3_BASE_ADDR), uart_clk);
#endif

}

static struct sys_timer mxc_timer = {
	.init	= mx53_sellwood_timer_init,
};

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX53_SELLWOOD data structure.
 */
MACHINE_START(MX53_SELLWOOD, "Freescale MX53 SELLWOOD Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.fixup = fixup_mxc_board,
	.map_io = mx5_map_io,
	.init_irq = mx5_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END
