/*
    Hawthorne board file. Copyright (C) 2013 Tapani Utriainen
    Authors: Tapani Utriainen, Edward Lin

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/memblock.h>
#include <linux/phy.h>
#include <linux/mxcfb.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <mach/ahci_sata.h>
#include <mach/common.h>
#include <mach/gpio.h>
#include <mach/iomux-mx6dl.h>
#include <mach/iomux-mx6q.h>
#include <mach/iomux-v3.h>
#include <mach/mx6.h>

#include "crm_regs.h"
#include "devices-imx6q.h"
#include "edm.h"
#include "usb.h"

#define HAWTHORNE_BT_ON		IMX_GPIO_NR(3, 13)
#define HAWTHORNE_BT_WAKE		IMX_GPIO_NR(3, 14)
#define HAWTHORNE_BT_HOST_WAKE	IMX_GPIO_NR(3, 15)

//#define HAWTHORNE_PCIE_NRST		IMX_GPIO_NR(3, 31)

#define HAWTHORNE_RGMII_INT		IMX_GPIO_NR(1, 28)
#define HAWTHORNE_RGMII_RST		IMX_GPIO_NR(3, 29)

#define HAWTHORNE_SD1_CD		IMX_GPIO_NR(1, 2)
#define HAWTHORNE_SD3_CD		IMX_GPIO_NR(3, 9)
#define HAWTHORNE_SD3_WP		IMX_GPIO_NR(1, 10)

#define HAWTHORNE_USB_OTG_OC		IMX_GPIO_NR(1, 9)
#define HAWTHORNE_USB_OTG_PWR	IMX_GPIO_NR(3, 31)
#define HAWTHORNE_USB_H1_OC		IMX_GPIO_NR(1, 3)

#define HAWTHORNE_WL_REF_ON		IMX_GPIO_NR(2, 29)
#define HAWTHORNE_WL_RST_N		IMX_GPIO_NR(5, 2)
#define HAWTHORNE_WL_REG_ON		IMX_GPIO_NR(1, 26)
#define HAWTHORNE_WL_HOST_WAKE	IMX_GPIO_NR(1, 29)
#define HAWTHORNE_WL_WAKE		IMX_GPIO_NR(1, 30)

#define HAWTHORNE_MIPICSI_PWN	IMX_GPIO_NR(1, 6)
#define HAWTHORNE_MIPICSI_RESET	IMX_GPIO_NR(4, 14)

#define HAWTHORNE_ECSPI1_CS2    IMX_GPIO_NR(3, 24)

#define HAWTHORNE_LVDS_BL_EN	IMX_GPIO_NR(2,9)	
#define HAWTHORNE_DISP_BL_EN	IMX_GPIO_NR(2,10)	
#define HAWTHORNE_DISP_EN		IMX_GPIO_NR(2,11)	


/* Syntactic sugar for pad configuration */
#define IMX6_SETUP_PAD(p) \
do { \
	if (cpu_is_mx6q()) \
		mxc_iomux_v3_setup_pad(MX6Q_PAD_##p); \
	else \
		mxc_iomux_v3_setup_pad(MX6DL_PAD_##p); \
} while (0)

/* See arch/arm/plat-mxc/include/mach/iomux-mx6dl.h for definitions */

/****************************************************************************
 *
 * DMA controller init
 *
 ****************************************************************************/

static __init void hawthorne_init_dma(void)
{
	imx6q_add_dma();
}


/****************************************************************************
 *
 * SD init
 *
 * SD1 is routed to EDM connector (external SD on hawthorne baseboard)
 * SD2 is WiFi
 * SD3 is boot SD on the module
 *
 ****************************************************************************/

static const struct esdhc_platform_data hawthorne_sd_data[3] = {
	{
		.cd_gpio = HAWTHORNE_SD1_CD,
		.wp_gpio = -EINVAL,
		.keep_power_at_suspend = 1,
		.support_8bit = 0,
		.delay_line = 0,
		.cd_type = ESDHC_CD_CONTROLLER,
	}, {
		.cd_gpio = -EINVAL,
		.wp_gpio = -EINVAL,
		.keep_power_at_suspend = 1,
		.support_8bit = 0,
		.delay_line = 0,
		.always_present = 1,
		.cd_type = ESDHC_CD_PERMANENT,
	}, {
		.cd_gpio = HAWTHORNE_SD3_CD,
		.wp_gpio = HAWTHORNE_SD3_WP,
		.keep_power_at_suspend = 1,
		.support_8bit = 0,
		.delay_line = 0,
		.cd_type = ESDHC_CD_CONTROLLER,
	}
};

/* ------------------------------------------------------------------------ */

static void hawthorne_init_sd(void)
{
	int i;

	IMX6_SETUP_PAD(SD1_CLK__USDHC1_CLK_50MHZ_40OHM);
	IMX6_SETUP_PAD(SD1_CMD__USDHC1_CMD_50MHZ_40OHM);
	IMX6_SETUP_PAD(SD1_DAT0__USDHC1_DAT0_50MHZ_40OHM);
	IMX6_SETUP_PAD(SD1_DAT1__USDHC1_DAT1_50MHZ_40OHM);
	IMX6_SETUP_PAD(SD1_DAT2__USDHC1_DAT2_50MHZ_40OHM);
	IMX6_SETUP_PAD(SD1_DAT3__USDHC1_DAT3_50MHZ_40OHM);

	IMX6_SETUP_PAD(SD2_CLK__USDHC2_CLK);
	IMX6_SETUP_PAD(SD2_CMD__USDHC2_CMD);
	IMX6_SETUP_PAD(SD2_DAT0__USDHC2_DAT0);
	IMX6_SETUP_PAD(SD2_DAT1__USDHC2_DAT1);
	IMX6_SETUP_PAD(SD2_DAT2__USDHC2_DAT2);
	IMX6_SETUP_PAD(SD2_DAT3__USDHC2_DAT3);

	IMX6_SETUP_PAD(SD3_CLK__USDHC3_CLK_50MHZ);
	IMX6_SETUP_PAD(SD3_CMD__USDHC3_CMD_50MHZ);
	IMX6_SETUP_PAD(SD3_DAT0__USDHC3_DAT0_50MHZ);
	IMX6_SETUP_PAD(SD3_DAT1__USDHC3_DAT1_50MHZ);
	IMX6_SETUP_PAD(SD3_DAT2__USDHC3_DAT2_50MHZ);
	IMX6_SETUP_PAD(SD3_DAT3__USDHC3_DAT3_50MHZ);

	/* Card Detect for SD1 & SD3, respectively */
	IMX6_SETUP_PAD(GPIO_2__GPIO_1_2);
	IMX6_SETUP_PAD(EIM_DA9__GPIO_3_9);

	/* Add mmc devices in reverse order, so mmc0 always is boot sd (SD3) */
	for (i = 2; i >= 0; i--)
		imx6q_add_sdhci_usdhc_imx(i, &hawthorne_sd_data[i]);

}


/******************************************************************
*
*       SPI NOR
*
*******************************************************************/
static int hawthorne_spi_cs[] = {
        HAWTHORNE_ECSPI1_CS2,
};

static const struct spi_imx_master hawthorne_spi_data __initconst = {
        .chipselect     = hawthorne_spi_cs,
        .num_chipselect = ARRAY_SIZE(hawthorne_spi_cs),
};

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition hawthorne_spi_nor_partitions[] = {
        {
                .name   = "bootloader",
                .offset = 0,
                .size   = SZ_512K,
        }, 
		{
                .name   = "bootenv",
                .offset = MTDPART_OFS_APPEND,
                .size   = SZ_128K,
        },
        {
                .name   = "genenv",
                .offset = MTDPART_OFS_APPEND,
                .size   = SZ_256K,
        }, 
		{
                .name   = "kernel",
                .offset = MTDPART_OFS_APPEND,
                .size   = MTDPART_SIZ_FULL,
        },      
};

static struct flash_platform_data hawthorne__spi_flash_data = {
        .name = "m25p32",
        .parts = hawthorne_spi_nor_partitions,
        .nr_parts = ARRAY_SIZE(hawthorne_spi_nor_partitions),
        .type = "m25p32",
};
#endif

static struct spi_board_info hawthorne_spi_nor_device[] __initdata = {
#if defined(CONFIG_MTD_M25P80)
        {
                .modalias = "m25p80",
                .max_speed_hz = 20000000, /* max spi clock (SCK) speed in HZ */
                .bus_num = 0,
                .chip_select = 0,
                .platform_data = &hawthorne__spi_flash_data,
        },
#endif
};

static void hawthorne_spi_nor_init(void)
{
        /* ECSPI1 */
        IMX6_SETUP_PAD( EIM_D17__ECSPI1_MISO );
        IMX6_SETUP_PAD( EIM_D18__ECSPI1_MOSI );
        IMX6_SETUP_PAD( EIM_D16__ECSPI1_SCLK );
        IMX6_SETUP_PAD( EIM_D24__GPIO_3_24 );

        spi_register_board_info(hawthorne_spi_nor_device,
                                ARRAY_SIZE(hawthorne_spi_nor_device));
}

/****************************************************************************
 * NAND
 *                                                                          
 ****************************************************************************/

static int __init hawthorne_gpmi_nand_platform_init(void)
{
    return 0;
}
 
static struct mtd_partition nand_flash_partitions[] = {
	{
		.name	= "rootfs",
		.offset = 0,
		.size	= MTDPART_SIZ_FULL,
	},
};


static const struct gpmi_nand_platform_data
     hawthorne_gpmi_nand_platform_data __initconst = {
    .platform_init           = hawthorne_gpmi_nand_platform_init,
    .min_prop_delay_in_ns    = 5,
    .max_prop_delay_in_ns    = 9,
    .max_chip_count          = 1,
    .enable_bbt              = 1,
	.partitions				 = nand_flash_partitions,
	.partition_count		 = ARRAY_SIZE(nand_flash_partitions),
};

static void __init hawthorne_init_nand(void)
{

        IMX6_SETUP_PAD( NANDF_ALE__RAWNAND_ALE );
        IMX6_SETUP_PAD( NANDF_CS0__RAWNAND_CE0N );
        IMX6_SETUP_PAD( NANDF_CS1__RAWNAND_CE1N );
        IMX6_SETUP_PAD( NANDF_CLE__RAWNAND_CLE );
        IMX6_SETUP_PAD( NANDF_D0__RAWNAND_D0 );
        IMX6_SETUP_PAD( NANDF_D1__RAWNAND_D1 );
        IMX6_SETUP_PAD( NANDF_D2__RAWNAND_D2 );
        IMX6_SETUP_PAD( NANDF_D3__RAWNAND_D3 );
        IMX6_SETUP_PAD( NANDF_D4__RAWNAND_D4 );
        IMX6_SETUP_PAD( NANDF_D5__RAWNAND_D5 );
        IMX6_SETUP_PAD( NANDF_D6__RAWNAND_D6 );
        IMX6_SETUP_PAD( NANDF_D7__RAWNAND_D7 );
        IMX6_SETUP_PAD( NANDF_RB0__RAWNAND_READY0 );
        IMX6_SETUP_PAD( SD4_CMD__RAWNAND_RDN );
        IMX6_SETUP_PAD( SD4_CLK__RAWNAND_WRN );
        IMX6_SETUP_PAD( NANDF_WP_B__RAWNAND_RESETN );

        imx6q_add_gpmi(&hawthorne_gpmi_nand_platform_data);
}


/****************************************************************************
 *
 * I2C
 *
 ****************************************************************************/
static struct imxi2c_platform_data hawthorne_i2c_data[] = {
	{ .bitrate = 400000, },
	{ .bitrate = 400000, },
	{ .bitrate = 400000, },
};

/* ------------------------------------------------------------------------ */

static void __init hawthorne_init_i2c(void)
{
	int i;

	IMX6_SETUP_PAD(EIM_D21__I2C1_SCL);
	IMX6_SETUP_PAD(EIM_D28__I2C1_SDA);

	IMX6_SETUP_PAD(KEY_COL3__I2C2_SCL);
	IMX6_SETUP_PAD(KEY_ROW3__I2C2_SDA);

	IMX6_SETUP_PAD(GPIO_5__I2C3_SCL);
	IMX6_SETUP_PAD(GPIO_16__I2C3_SDA);

	for (i = 0; i < 3; i++)
		imx6q_add_imx_i2c(i, &hawthorne_i2c_data[i]);

}


/****************************************************************************
 *
 * Initialize debug console (UART1)
 *
 ****************************************************************************/

static __init void hawthorne_init_uart(void)
{
    IMX6_SETUP_PAD( CSI0_DAT10__UART1_TXD );
    IMX6_SETUP_PAD( CSI0_DAT11__UART1_RXD );
        
    IMX6_SETUP_PAD( SD4_DAT4__UART2_RXD );
    IMX6_SETUP_PAD( SD4_DAT7__UART2_TXD );

    //IMX6_SETUP_PAD( EIM_D19__UART1_CTS );
    //IMX6_SETUP_PAD( EIM_D20__UART1_RTS );

    imx6q_add_imx_uart(0, NULL);
    imx6q_add_imx_uart(1, NULL);
}
 
/*****************************************************************************
 *
 * Init FEC and AR8031 PHY
 *
 *****************************************************************************/

static int hawthorne_fec_phy_init(struct phy_device *phydev)
{
	unsigned short val;

	/* Enable AR8031 125MHz clk */
	phy_write(phydev, 0x0d, 0x0007); /* Set device address to 7*/
	phy_write(phydev, 0x00, 0x8000); /* Apply by soft reset */
	udelay(500);

	phy_write(phydev, 0x0e, 0x8016); /* set mmd reg */
	phy_write(phydev, 0x0d, 0x4007); /* apply */

	val = phy_read(phydev, 0xe);
	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, 0xe, val);
	phy_write(phydev, 0x0d, 0x4007); /* Post data */

	/* Introduce random tx clock delay. Why is this needed? */
	phy_write(phydev, 0x1d, 0x5);
	val = phy_read(phydev, 0x1e);
	val |= 0x0100;
	phy_write(phydev, 0x1e, val);

	return 0;
}

/* ------------------------------------------------------------------------ */

static int hawthorne_fec_power_hibernate(struct phy_device *phydev) { return 0; }

/* ------------------------------------------------------------------------ */

static struct fec_platform_data hawthorne_fec_data = {
	.init			= hawthorne_fec_phy_init,
	.power_hibernate	= hawthorne_fec_power_hibernate,
	.phy			= PHY_INTERFACE_MODE_RGMII,
};

/* ------------------------------------------------------------------------ */

static __init void hawthorne_init_ethernet(void)
{
	IMX6_SETUP_PAD(ENET_MDIO__ENET_MDIO);
	IMX6_SETUP_PAD(ENET_MDC__ENET_MDC);
	IMX6_SETUP_PAD(ENET_REF_CLK__ENET_TX_CLK);
	IMX6_SETUP_PAD(RGMII_TXC__ENET_RGMII_TXC);
	IMX6_SETUP_PAD(RGMII_TD0__ENET_RGMII_TD0);
	IMX6_SETUP_PAD(RGMII_TD1__ENET_RGMII_TD1);
	IMX6_SETUP_PAD(RGMII_TD2__ENET_RGMII_TD2);
	IMX6_SETUP_PAD(RGMII_TD3__ENET_RGMII_TD3);
	IMX6_SETUP_PAD(RGMII_TX_CTL__ENET_RGMII_TX_CTL);
	IMX6_SETUP_PAD(RGMII_RXC__ENET_RGMII_RXC);
	IMX6_SETUP_PAD(RGMII_RD0__ENET_RGMII_RD0);
	IMX6_SETUP_PAD(RGMII_RD1__ENET_RGMII_RD1);
	IMX6_SETUP_PAD(RGMII_RD2__ENET_RGMII_RD2);
	IMX6_SETUP_PAD(RGMII_RD3__ENET_RGMII_RD3);
	IMX6_SETUP_PAD(RGMII_RX_CTL__ENET_RGMII_RX_CTL);
	IMX6_SETUP_PAD(ENET_TX_EN__GPIO_1_28);
	IMX6_SETUP_PAD(EIM_D29__GPIO_3_29);

	gpio_request(HAWTHORNE_RGMII_RST, "rgmii reset");
	gpio_direction_output(HAWTHORNE_RGMII_RST, 0);
#ifdef CONFIG_FEC_1588
	mxc_iomux_set_gpr_register(1, 21, 1, 1);
#endif
	msleep(10);
	gpio_set_value(HAWTHORNE_RGMII_RST, 1);
	imx6_init_fec(hawthorne_fec_data);
}


/****************************************************************************
 *
 * USB
 *
 ****************************************************************************/

static void hawthorne_usbotg_vbus(bool on)
{
	gpio_set_value_cansleep(HAWTHORNE_USB_OTG_PWR, !on);
}

/* ------------------------------------------------------------------------ */

static __init void hawthorne_init_usb(void)
{
	IMX6_SETUP_PAD(GPIO_9__GPIO_1_9);
	IMX6_SETUP_PAD(GPIO_1__USBOTG_ID);
	IMX6_SETUP_PAD(EIM_D22__GPIO_3_22);
	IMX6_SETUP_PAD(EIM_D30__GPIO_3_30);

	gpio_request(HAWTHORNE_USB_OTG_OC, "otg oc");
	gpio_direction_input(HAWTHORNE_USB_OTG_OC);

	gpio_request(HAWTHORNE_USB_OTG_PWR, "otg pwr");
	gpio_direction_output(HAWTHORNE_USB_OTG_PWR, 0);

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	mxc_iomux_set_gpr_register(1, 13, 1, 1);

	mx6_set_otghost_vbus_func(hawthorne_usbotg_vbus);

	gpio_request(HAWTHORNE_USB_H1_OC, "usbh1 oc");
	gpio_direction_input(HAWTHORNE_USB_H1_OC);
}


/****************************************************************************
 *
 * IPU
 *
 ****************************************************************************/

static struct imx_ipuv3_platform_data hawthorne_ipu_data[] = {
	{
		.rev = 4,
		.csi_clk[0] = "clko2_clk",
	}, {
		.rev = 4,
		.csi_clk[0] = "clko2_clk",
	},
};

/* ------------------------------------------------------------------------ */

static __init void hawthorne_init_ipu(void)
{
	imx6q_add_ipuv3(0, &hawthorne_ipu_data[0]);
}

#ifdef CONFIG_IMX_HAVE_PLATFORM_IMX_MIPI_CSI2
/****************************************************************************
 *
 * MIPI CSI
 *
 ****************************************************************************/
static void hawthorne_mipi_sensor_io_init(void)
{
	struct clk *mipi_csi_mclk;
	int rate;

	IMX6_SETUP_PAD(GPIO_3__CCM_CLKO2);  /* Camera clock */
	IMX6_SETUP_PAD(KEY_COL4__GPIO_4_14);/* Camera reset */
	IMX6_SETUP_PAD(GPIO_6__GPIO_1_6);   /* Camera power down */

	pr_debug("%s\n", __func__);

	gpio_request(HAWTHORNE_MIPICSI_RESET, "cam-reset");
	gpio_direction_output(HAWTHORNE_MIPICSI_RESET, 1);

	gpio_request(HAWTHORNE_MIPICSI_PWN, "cam-pwdn");
	gpio_direction_output(HAWTHORNE_MIPICSI_PWN, 1);

	/* Master clock for the sensor */
	mipi_csi_mclk = clk_get(NULL, "clko2_clk");
	if (IS_ERR(mipi_csi_mclk)) {
		pr_err("can't get CLKO2 clock.\n");
		return;
	}

	rate = clk_round_rate(mipi_csi_mclk, 24000000);
	clk_set_rate(mipi_csi_mclk, rate);
	clk_enable(mipi_csi_mclk);

	msleep(5);
	/* Power up */
	gpio_set_value(HAWTHORNE_MIPICSI_PWN, 0);
	msleep(5);
	/* Reset on */
	gpio_set_value(HAWTHORNE_MIPICSI_RESET, 0);
	msleep(1);
	/* Reset off */
	gpio_set_value(HAWTHORNE_MIPICSI_RESET, 1);
	msleep(5);
	/* power down */
	gpio_set_value(HAWTHORNE_MIPICSI_PWN, 1);

	/* For MX6Q:
	 * GPR1 bit19 and bit20 meaning:
	 * Bit19:       0 - Enable mipi to IPU1 CSI0
	 *                      virtual channel is fixed to 0
	 *              1 - Enable parallel interface to IPU1 CSI0
	 * Bit20:       0 - Enable mipi to IPU2 CSI1
	 *                      virtual channel is fixed to 3
	 *              1 - Enable parallel interface to IPU2 CSI1
	 * IPU1 CSI1 directly connect to mipi csi2,
	 *      virtual channel is fixed to 1
	 * IPU2 CSI0 directly connect to mipi csi2,
	 *      virtual channel is fixed to 2
	 *
	 * For MX6DL:
	 * GPR13 bit 0-2 IPU_CSI0_MUX
	 *   000 MIPI_CSI0
	 *   100 IPU CSI0
	 */

	if (cpu_is_mx6q())
		mxc_iomux_set_gpr_register(1, 19, 1, 0);
	else if (cpu_is_mx6dl())
		mxc_iomux_set_gpr_register(13, 0, 3, 0);
}

static void hawthorne_mipi_powerdown(int powerdown)
{
	pr_debug("%s: powerdown=%d\n", __func__, powerdown);

	if (powerdown)
		gpio_set_value(HAWTHORNE_MIPICSI_PWN, 1); /* Power off */
	else
		gpio_set_value(HAWTHORNE_MIPICSI_PWN, 0); /* Power on */

	msleep(2);
}

/* Platform data for CSI sensor driver, passed to driver
   through i2c platform data */
static struct fsl_mxc_camera_platform_data hawthorne_mipi_csi2_data = {
	.mclk = 24000000,
	.mclk_source = 0,
	.csi = 0,
	.io_init = hawthorne_mipi_sensor_io_init,
	.pwdn = hawthorne_mipi_powerdown,
};

/* TODO - reorg code so that adding i2c board info is done all in one
   step at init.  This makes adding I2C devices easier */
static struct i2c_board_info hawthorne_mipi_csi_i2c_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("ov5640_mipi", 0x3C),
		.platform_data = (void *)&hawthorne_mipi_csi2_data,
	},
};

/* Platform data for MIPI CSI init */
static struct mipi_csi2_platform_data hawthorne_mipi_csi2_platform_data = {
	.ipu_id = 0,
	.csi_id = 0,
	.v_channel = 0,
	.lanes = 2,
	.dphy_clk = "mipi_pllref_clk",
	.pixel_clk = "emi_clk",
};


static struct fsl_mxc_capture_platform_data capture_data[] = {
#if defined(CONFIG_MXC_CAMERA_OV5640_MIPI) || \
		defined(CONFIG_MXC_CAMERA_OV5640_MIPI_MODULE)
	{
		.ipu = 0,
		.csi = 0,
		.mclk_source = 0,
		.is_mipi = 1,
	},
#endif
};

/* Wandboard MIPI CSI init function */
static void __init hawthorne_init_mipi_csi(void)
{
	int i;
	pr_debug("%s\n", __func__);

	/* Initialize IPU Capture Path */
	for (i = 0; i < ARRAY_SIZE(capture_data); i++) {
		if (!cpu_is_mx6q())
			capture_data[i].ipu = 0;
		imx6q_add_v4l2_capture(i, &capture_data[i]);
	}

	/* Add CSI2 */
	imx6q_add_mipi_csi2(&hawthorne_mipi_csi2_platform_data);

	/* Register MIPI CSI I2C board info.  This sits on I2C2, which
	   is i2c device index 1 */
	i2c_register_board_info(1, hawthorne_mipi_csi_i2c_board_info,
		ARRAY_SIZE(hawthorne_mipi_csi_i2c_board_info));
}
#else
static void __init hawthorne_init_mipi_csi(void)
{
	return;
}
#endif

/****************************************************************************
 *
 * LCD/LVDS/TTL
 *
 ****************************************************************************/

static struct fsl_mxc_lcd_platform_data hawthorne_lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB24
};

/* ------------------------------------------------------------------------ */

static struct fsl_mxc_ldb_platform_data hawthorne_ldb_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.ext_ref = 1,
	.mode = LDB_SIN0,
	.sec_ipu_id = 0,
	.sec_disp_id = 0,
};

/* ------------------------------------------------------------------------ */

static struct ipuv3_fb_platform_data hawthorne_lvds_fb[] = {
	{
		.disp_dev = "ldb",
		.interface_pix_fmt = IPU_PIX_FMT_RGB24,
		.mode_str = "LDB-VGA",
		.default_bpp = 32,
		.int_clk = false,
	},
};

/* ------------------------------------------------------------------------ */

static void __init hawthorne_init_lcd(void)
{
	/* TTL */
	IMX6_SETUP_PAD(DI0_DISP_CLK__IPU1_DI0_DISP_CLK);
	IMX6_SETUP_PAD(DI0_PIN2__IPU1_DI0_PIN2);		/* HSync */
	IMX6_SETUP_PAD(DI0_PIN3__IPU1_DI0_PIN3);		/* VSync */
	IMX6_SETUP_PAD(DI0_PIN4__IPU1_DI0_PIN4);		/* Contrast */
	IMX6_SETUP_PAD(DI0_PIN15__IPU1_DI0_PIN15);	/* DISP0_DRDY */
	IMX6_SETUP_PAD(DISP0_DAT0__IPU1_DISP0_DAT_0);
	IMX6_SETUP_PAD(DISP0_DAT1__IPU1_DISP0_DAT_1);
	IMX6_SETUP_PAD(DISP0_DAT2__IPU1_DISP0_DAT_2);
	IMX6_SETUP_PAD(DISP0_DAT3__IPU1_DISP0_DAT_3);
	IMX6_SETUP_PAD(DISP0_DAT4__IPU1_DISP0_DAT_4);
	IMX6_SETUP_PAD(DISP0_DAT5__IPU1_DISP0_DAT_5);
	IMX6_SETUP_PAD(DISP0_DAT6__IPU1_DISP0_DAT_6);
	IMX6_SETUP_PAD(DISP0_DAT7__IPU1_DISP0_DAT_7);
	IMX6_SETUP_PAD(DISP0_DAT8__IPU1_DISP0_DAT_8);
	IMX6_SETUP_PAD(DISP0_DAT9__IPU1_DISP0_DAT_9);
	IMX6_SETUP_PAD(DISP0_DAT10__IPU1_DISP0_DAT_10);
	IMX6_SETUP_PAD(DISP0_DAT11__IPU1_DISP0_DAT_11);
	IMX6_SETUP_PAD(DISP0_DAT12__IPU1_DISP0_DAT_12);
	IMX6_SETUP_PAD(DISP0_DAT13__IPU1_DISP0_DAT_13);
	IMX6_SETUP_PAD(DISP0_DAT14__IPU1_DISP0_DAT_14);
	IMX6_SETUP_PAD(DISP0_DAT15__IPU1_DISP0_DAT_15);
	IMX6_SETUP_PAD(DISP0_DAT16__IPU1_DISP0_DAT_16);
	IMX6_SETUP_PAD(DISP0_DAT17__IPU1_DISP0_DAT_17);
	IMX6_SETUP_PAD(DISP0_DAT18__IPU1_DISP0_DAT_18);
	IMX6_SETUP_PAD(DISP0_DAT19__IPU1_DISP0_DAT_19);
	IMX6_SETUP_PAD(DISP0_DAT20__IPU1_DISP0_DAT_20);
	IMX6_SETUP_PAD(DISP0_DAT21__IPU1_DISP0_DAT_21);
	IMX6_SETUP_PAD(DISP0_DAT22__IPU1_DISP0_DAT_22);
	IMX6_SETUP_PAD(DISP0_DAT23__IPU1_DISP0_DAT_23);

	/* LVDS */
	IMX6_SETUP_PAD(SD4_DAT0__GPIO_2_8);
	IMX6_SETUP_PAD(SD4_DAT1__GPIO_2_9);
	IMX6_SETUP_PAD(SD4_DAT2__GPIO_2_10);
	IMX6_SETUP_PAD(SD4_DAT3__GPIO_2_11);

	IMX6_SETUP_PAD(LVDS0_CLK_P__LDB_LVDS0_CLK);
	IMX6_SETUP_PAD(LVDS0_TX0_P__LDB_LVDS0_TX0);
	IMX6_SETUP_PAD(LVDS0_TX1_P__LDB_LVDS0_TX1);
	IMX6_SETUP_PAD(LVDS0_TX2_P__LDB_LVDS0_TX2);
	IMX6_SETUP_PAD(LVDS0_TX3_P__LDB_LVDS0_TX3);

	gpio_request(IMX_GPIO_NR(2, 8), "lvds0_en");
	gpio_direction_output(IMX_GPIO_NR(2, 8), 1);

	gpio_request(IMX_GPIO_NR(2, 9), "lvds0_blt_ctrl");
	gpio_direction_output(IMX_GPIO_NR(2, 9), 1);

	gpio_request(IMX_GPIO_NR(2, 10), "disp0_bklen");
	gpio_direction_output(IMX_GPIO_NR(2, 10), 1);

	gpio_request(IMX_GPIO_NR(2, 11), "disp0_vdden");
	gpio_direction_output(IMX_GPIO_NR(2, 11), 1);

	//gpio_set_value(IMX_GPIO_NR(2,10),1);
	//gpio_set_value(IMX_GPIO_NR(2,9),1);
	//gpio_set_value(IMX_GPIO_NR(2,11),0);

	gpio_set_value(HAWTHORNE_DISP_BL_EN,0);
	gpio_set_value(HAWTHORNE_LVDS_BL_EN,0);
	gpio_set_value(HAWTHORNE_DISP_EN,0);

	imx6q_add_vdoa();

	imx6q_add_ldb(&hawthorne_ldb_data);
	imx6q_add_lcdif(&hawthorne_lcdif_data);

	imx6q_add_ipuv3fb(0, &hawthorne_lvds_fb[0]);
}

/****************************************************************************
 *
 * Power and thermal management
 *
 ****************************************************************************/

extern bool enable_wait_mode;

static const struct anatop_thermal_platform_data hawthorne_thermal = {
	.name = "anatop_thermal",
};

/* ------------------------------------------------------------------------ */

static void hawthorne_suspend_enter(void)
{
	gpio_set_value(HAWTHORNE_WL_WAKE, 0);
	gpio_set_value(HAWTHORNE_BT_WAKE, 0);
}

/* ------------------------------------------------------------------------ */

static void hawthorne_suspend_exit(void)
{
	gpio_set_value(HAWTHORNE_WL_WAKE, 1);
	gpio_set_value(HAWTHORNE_BT_WAKE, 1);
}

/* ------------------------------------------------------------------------ */

static const struct pm_platform_data hawthorne_pm_data = {
	.name = "imx_pm",
	.suspend_enter = hawthorne_suspend_enter,
	.suspend_exit = hawthorne_suspend_exit,
};

/* ------------------------------------------------------------------------ */

static const struct mxc_dvfs_platform_data hawthorne_dvfscore_data = {
	.reg_id  = "cpu_vddgp",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset   = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset   = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset  = MXC_CCM_CACRR_OFFSET,
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
	.delay_time = 80,
};

/* ------------------------------------------------------------------------ */

static __init void hawthorne_init_pm(void)
{
	enable_wait_mode = false;
	imx6q_add_anatop_thermal_imx(1, &hawthorne_thermal);
	imx6q_add_pm_imx(0, &hawthorne_pm_data);
	imx6q_add_dvfs_core(&hawthorne_dvfscore_data);
	imx6q_add_busfreq();
}

/****************************************************************************
 *
 * SPI - while not used on the Wandboard, the pins are routed out
 *
 ****************************************************************************/

//static const int hawthorne_spi1_chipselect[] = { IMX_GPIO_NR(2, 30) };

/* platform device */
//static const struct spi_imx_master hawthorne_spi1_data = {
//	.chipselect     = (int *)hawthorne_spi1_chipselect,
//	.num_chipselect = ARRAY_SIZE(hawthorne_spi1_chipselect),
//};

/* ------------------------------------------------------------------------ */

//static const int hawthorne_spi2_chipselect[] = { IMX_GPIO_NR(2, 26), IMX_GPIO_NR(2, 27) };

//static const struct spi_imx_master hawthorne_spi2_data = {
//	.chipselect     = (int *)hawthorne_spi2_chipselect,
//	.num_chipselect = ARRAY_SIZE(hawthorne_spi2_chipselect),
//};

/* ------------------------------------------------------------------------ */

//static void __init hawthorne_init_spi(void)
//{
//	IMX6_SETUP_PAD(EIM_D16__ECSPI1_SCLK);
//	IMX6_SETUP_PAD(EIM_D17__ECSPI1_MISO);
//	IMX6_SETUP_PAD(EIM_D18__ECSPI1_MOSI);
//	IMX6_SETUP_PAD(EIM_EB2__GPIO_2_30);

//	IMX6_SETUP_PAD(EIM_CS0__ECSPI2_SCLK);
//	IMX6_SETUP_PAD(EIM_CS1__ECSPI2_MOSI);
//	IMX6_SETUP_PAD(EIM_OE__ECSPI2_MISO);
/* The choice of using gpios for chipselect is deliberate,
   there can be issues using the dedicated mux modes for cs.*/
//	IMX6_SETUP_PAD(EIM_RW__GPIO_2_26);
//	IMX6_SETUP_PAD(EIM_LBA__GPIO_2_27);

//	imx6q_add_ecspi(0, &hawthorne_spi1_data);
//	imx6q_add_ecspi(1, &hawthorne_spi2_data);
//}


/****************************************************************************
 *
 * Vivante GPU/VPU
 *
 ****************************************************************************/

static const __initconst struct imx_viv_gpu_data hawthorne_gpu_data = {
	.phys_baseaddr = 0,
	.iobase_3d = GPU_3D_ARB_BASE_ADDR,
	.irq_3d = MXC_INT_GPU3D_IRQ,
	.iobase_2d = GPU_2D_ARB_BASE_ADDR,
	.irq_2d = MXC_INT_GPU2D_IRQ,
	.iobase_vg = OPENVG_ARB_BASE_ADDR,
	.irq_vg = MXC_INT_OPENVG_XAQ2,
};

static struct viv_gpu_platform_data hawthorne_gpu_pdata = {
	.reserved_mem_size = SZ_128M + SZ_64M - SZ_16M,
};

static __init void hawthorne_init_gpu(void)
{
	imx_add_viv_gpu(&hawthorne_gpu_data, &hawthorne_gpu_pdata);
	imx6q_add_vpu();
	imx6q_add_v4l2_output(0);
}

/*****************************************************************************
 *
 * Init clocks and early boot console
 *
 *****************************************************************************/

extern void __iomem *twd_base;

static void __init hawthorne_init_timer(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

/* ------------------------------------------------------------------------ */

static struct sys_timer hawthorne_timer = {
	.init = hawthorne_init_timer,
};

/* ------------------------------------------------------------------------ */

static void __init hawthorne_reserve(void)
{
	phys_addr_t phys;
	struct meminfo *mi = &meminfo;
	unsigned long total_mem = 0;
	int i;

	if (hawthorne_gpu_pdata.reserved_mem_size) {
		for (i = 0; i < mi->nr_banks; i++)
			total_mem += mi->bank[i].size;
		phys = memblock_alloc_base(
				hawthorne_gpu_pdata.reserved_mem_size,
				SZ_4K, total_mem);
		memblock_remove(phys, hawthorne_gpu_pdata.reserved_mem_size);
		hawthorne_gpu_pdata.reserved_mem_base = phys;
	}

	edm_external_gpio[0] = IMX_GPIO_NR(3, 11);        
	edm_external_gpio[1] = IMX_GPIO_NR(3, 27);
	edm_external_gpio[2] = IMX_GPIO_NR(6, 31);
	edm_external_gpio[3] = IMX_GPIO_NR(1, 24);
	edm_external_gpio[4] = IMX_GPIO_NR(7, 8);
	edm_external_gpio[5] = IMX_GPIO_NR(3, 26);
	edm_external_gpio[6] = IMX_GPIO_NR(3, 8);
	edm_external_gpio[7] = IMX_GPIO_NR(4, 5);
        
	edm_i2c[0] = -EINVAL;
	edm_i2c[1] = -EINVAL;
	edm_i2c[2] = -EINVAL;
	edm_ddc = -EINVAL;
        
}

/*****************************************************************************
 *
 * BOARD INIT
 *
 *****************************************************************************/

static void __init hawthorne_board_init(void)
{
	hawthorne_init_dma();
	hawthorne_init_uart();
	hawthorne_init_sd();
	hawthorne_init_i2c();
	hawthorne_init_ethernet();
	hawthorne_init_usb();
	hawthorne_init_ipu();
	hawthorne_init_mipi_csi();
	hawthorne_init_lcd();
	hawthorne_init_pm();
	//hawthorne_init_spi();
	hawthorne_init_gpu();
    hawthorne_init_nand();

    imx6q_add_ecspi(0, &hawthorne_spi_data);

	hawthorne_spi_nor_init();
}

/* ------------------------------------------------------------------------ */

MACHINE_START(HAWTHORNE, "Hawthorne")
	.boot_params	= MX6_PHYS_OFFSET + 0x100,
	.map_io		= mx6_map_io,
	.init_irq	= mx6_init_irq,
	.init_machine	= hawthorne_board_init,
	.timer		= &hawthorne_timer,
	.reserve        = hawthorne_reserve,
MACHINE_END
