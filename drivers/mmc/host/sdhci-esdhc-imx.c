/*
 * Freescale eSDHC i.MX controller driver for the platform bus.
 *
 * derived from the OF-version.
 *
 * Copyright (c) 2010 Pengutronix e.K.
 *   Author: Wolfram Sang <w.sang@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 */

#include <linux/busfreq-imx6.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_data/mmc-esdhc-imx.h>
#include <linux/pm_runtime.h>
#include "sdhci-pltfm.h"
#include "sdhci-esdhc.h"

#define	ESDHC_CTRL_D3CD			0x08
/* VENDOR SPEC register */
#define ESDHC_VENDOR_SPEC		0xc0
#define  ESDHC_VENDOR_SPEC_SDIO_QUIRK	(1 << 1)
#define  ESDHC_VENDOR_SPEC_VSELECT	(1 << 1)
#define  ESDHC_VENDOR_SPEC_FRC_SDCLK_ON	(1 << 8)
#define ESDHC_WTMK_LVL			0x44
#define ESDHC_MIX_CTRL			0x48
#define  ESDHC_MIX_CTRL_DDREN		(1 << 3)
#define  ESDHC_MIX_CTRL_AC23EN		(1 << 7)
#define  ESDHC_MIX_CTRL_EXE_TUNE	(1 << 22)
#define  ESDHC_MIX_CTRL_SMPCLK_SEL	(1 << 23)
#define  ESDHC_MIX_CTRL_FBCLK_SEL	(1 << 25)
/* Bits 3 and 6 are not SDHCI standard definitions */
#define  ESDHC_MIX_CTRL_SDHCI_MASK	0xb7
/* Tuning bits */
#define  ESDHC_MIX_CTRL_TUNING_MASK	0x03c00000

/* dll control register */
#define ESDHC_DLL_CTRL			0x60
#define ESDHC_DLL_OVERRIDE_VAL_SHIFT	9
#define ESDHC_DLL_OVERRIDE_EN_SHIFT	8

/* tune control register */
#define ESDHC_TUNE_CTRL_STATUS		0x68
#define  ESDHC_TUNE_CTRL_STEP		1
#define  ESDHC_TUNE_CTRL_MIN		0
#define  ESDHC_TUNE_CTRL_MAX		((1 << 7) - 1)

#define ESDHC_TUNING_CTRL		0xcc
#define ESDHC_STD_TUNING_EN		(1 << 24)
/* NOTE: the minimum valid tuning start tap for mx6sl is 1 */
#define ESDHC_TUNING_START_TAP		0x1

#define ESDHC_TUNING_BLOCK_PATTERN_LEN	64

/* pinctrl state */
#define ESDHC_PINCTRL_STATE_100MHZ	"state_100mhz"
#define ESDHC_PINCTRL_STATE_200MHZ	"state_200mhz"

/*
 * Our interpretation of the SDHCI_HOST_CONTROL register
 */
#define ESDHC_CTRL_4BITBUS		(0x1 << 1)
#define ESDHC_CTRL_8BITBUS		(0x2 << 1)
#define ESDHC_CTRL_BUSWIDTH_MASK	(0x3 << 1)

/*
 * There is an INT DMA ERR mis-match between eSDHC and STD SDHC SPEC:
 * Bit25 is used in STD SPEC, and is reserved in fsl eSDHC design,
 * but bit28 is used as the INT DMA ERR in fsl eSDHC design.
 * Define this macro DMA error INT for fsl eSDHC
 */
#define ESDHC_INT_VENDOR_SPEC_DMA_ERR	(1 << 28)

/*
 * The CMDTYPE of the CMD register (offset 0xE) should be set to
 * "11" when the STOP CMD12 is issued on imx53 to abort one
 * open ended multi-blk IO. Otherwise the TC INT wouldn't
 * be generated.
 * In exact block transfer, the controller doesn't complete the
 * operations automatically as required at the end of the
 * transfer and remains on hold if the abort command is not sent.
 * As a result, the TC flag is not asserted and SW  received timeout
 * exeception. Bit1 of Vendor Spec registor is used to fix it.
 */
#define ESDHC_FLAG_MULTIBLK_NO_INT	BIT(1)
/*
 * The flag enables the workaround for ESDHC errata ENGcm07207 which
 * affects i.MX25 and i.MX35.
 */
#define ESDHC_FLAG_ENGCM07207		BIT(2)
/*
 * The flag tells that the ESDHC controller is an USDHC block that is
 * integrated on the i.MX6 series.
 */
#define ESDHC_FLAG_USDHC		BIT(3)
/* The IP supports manual tuning process */
#define ESDHC_FLAG_MAN_TUNING		BIT(4)
/* The IP supports standard tuning process */
#define ESDHC_FLAG_STD_TUNING		BIT(5)
/* The IP has SDHCI_CAPABILITIES_1 register */
#define ESDHC_FLAG_HAVE_CAP1		BIT(6)
/* The IP has errata ERR004536 */
#define ESDHC_FLAG_ERR004536		BIT(7)
/* need request bus freq during low power */
#define ESDHC_FLAG_BUSFREQ		BIT(8)

struct esdhc_soc_data {
	u32 flags;
};

static struct esdhc_soc_data esdhc_imx25_data = {
	.flags = ESDHC_FLAG_ENGCM07207,
};

static struct esdhc_soc_data esdhc_imx35_data = {
	.flags = ESDHC_FLAG_ENGCM07207,
};

static struct esdhc_soc_data esdhc_imx51_data = {
	.flags = 0,
};

static struct esdhc_soc_data esdhc_imx53_data = {
	.flags = ESDHC_FLAG_MULTIBLK_NO_INT,
};

static struct esdhc_soc_data usdhc_imx6q_data = {
	.flags = ESDHC_FLAG_USDHC | ESDHC_FLAG_MAN_TUNING,
};

static struct esdhc_soc_data usdhc_imx6sl_data = {
	.flags = ESDHC_FLAG_USDHC | ESDHC_FLAG_STD_TUNING
			| ESDHC_FLAG_HAVE_CAP1 | ESDHC_FLAG_ERR004536
			| ESDHC_FLAG_BUSFREQ,
};

struct pltfm_imx_data {
	u32 scratchpad;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_100mhz;
	struct pinctrl_state *pins_200mhz;
	const struct esdhc_soc_data *socdata;
	struct esdhc_platform_data boarddata;
	struct clk *clk_ipg;
	struct clk *clk_ahb;
	struct clk *clk_per;
	enum {
		NO_CMD_PENDING,      /* no multiblock command pending*/
		MULTIBLK_IN_PROCESS, /* exact multiblock cmd in process */
		WAIT_FOR_INT,        /* sent CMD12, waiting for response INT */
	} multiblock_status;
	u32 uhs_mode;
	u32 is_ddr;
	unsigned max_clock;
};

static struct platform_device_id imx_esdhc_devtype[] = {
	{
		.name = "sdhci-esdhc-imx25",
		.driver_data = (kernel_ulong_t) &esdhc_imx25_data,
	}, {
		.name = "sdhci-esdhc-imx35",
		.driver_data = (kernel_ulong_t) &esdhc_imx35_data,
	}, {
		.name = "sdhci-esdhc-imx51",
		.driver_data = (kernel_ulong_t) &esdhc_imx51_data,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(platform, imx_esdhc_devtype);

static const struct of_device_id imx_esdhc_dt_ids[] = {
	{ .compatible = "fsl,imx25-esdhc", .data = &esdhc_imx25_data, },
	{ .compatible = "fsl,imx35-esdhc", .data = &esdhc_imx35_data, },
	{ .compatible = "fsl,imx51-esdhc", .data = &esdhc_imx51_data, },
	{ .compatible = "fsl,imx53-esdhc", .data = &esdhc_imx53_data, },
	{ .compatible = "fsl,imx6sl-usdhc", .data = &usdhc_imx6sl_data, },
	{ .compatible = "fsl,imx6q-usdhc", .data = &usdhc_imx6q_data, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_esdhc_dt_ids);

static inline int is_imx25_esdhc(struct pltfm_imx_data *data)
{
	return data->socdata == &esdhc_imx25_data;
}

static inline int is_imx53_esdhc(struct pltfm_imx_data *data)
{
	return data->socdata == &esdhc_imx53_data;
}

static inline int is_imx6q_usdhc(struct pltfm_imx_data *data)
{
	return data->socdata == &usdhc_imx6q_data;
}

static inline int esdhc_is_usdhc(struct pltfm_imx_data *data)
{
	return !!(data->socdata->flags & ESDHC_FLAG_USDHC);
}

static inline void esdhc_clrset_le(struct sdhci_host *host, u32 mask, u32 val, int reg)
{
	void __iomem *base = host->ioaddr + (reg & ~0x3);
	u32 shift = (reg & 0x3) * 8;

	writel(((readl(base) & ~(mask << shift)) | (val << shift)), base);
}

static u32 esdhc_readl_le(struct sdhci_host *host, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = pltfm_host->priv;
	u32 val = readl(host->ioaddr + reg);

	if (unlikely(reg == SDHCI_PRESENT_STATE)) {
		u32 fsl_prss = val;
		/* save the least 20 bits */
		val = fsl_prss & 0x000FFFFF;
		/* move dat[0-3] bits */
		val |= (fsl_prss & 0x0F000000) >> 4;
		/* move cmd line bit */
		val |= (fsl_prss & 0x00800000) << 1;
	}

	if (unlikely(reg == SDHCI_CAPABILITIES)) {
		/* ignore bit[0-15] as it stores cap_1 register val for mx6sl */
		if (imx_data->socdata->flags & ESDHC_FLAG_HAVE_CAP1)
			val &= 0xffff0000;

		/* In FSL esdhc IC module, only bit20 is used to indicate the
		 * ADMA2 capability of esdhc, but this bit is messed up on
		 * some SOCs (e.g. on MX25, MX35 this bit is set, but they
		 * don't actually support ADMA2). So set the BROKEN_ADMA
		 * uirk on MX25/35 platforms.
		 */

		if (val & SDHCI_CAN_DO_ADMA1) {
			val &= ~SDHCI_CAN_DO_ADMA1;
			val |= SDHCI_CAN_DO_ADMA2;
		}
	}

	if (unlikely(reg == SDHCI_CAPABILITIES_1)) {
		if (esdhc_is_usdhc(imx_data)) {
			if (imx_data->socdata->flags & ESDHC_FLAG_HAVE_CAP1)
				val = readl(host->ioaddr + SDHCI_CAPABILITIES) & 0xFFFF;
			else
				/* imx6q/dl does not have cap_1 register, fake one */
				val = SDHCI_SUPPORT_DDR50 | SDHCI_SUPPORT_SDR104
					| SDHCI_SUPPORT_SDR50
					| SDHCI_USE_SDR50_TUNING;
		}
	}

	if (unlikely(reg == SDHCI_MAX_CURRENT) && esdhc_is_usdhc(imx_data)) {
		val = 0;
		val |= 0xFF << SDHCI_MAX_CURRENT_330_SHIFT;
		val |= 0xFF << SDHCI_MAX_CURRENT_300_SHIFT;
		val |= 0xFF << SDHCI_MAX_CURRENT_180_SHIFT;
	}

	if (unlikely(reg == SDHCI_INT_STATUS)) {
		if (val & ESDHC_INT_VENDOR_SPEC_DMA_ERR) {
			val &= ~ESDHC_INT_VENDOR_SPEC_DMA_ERR;
			val |= SDHCI_INT_ADMA_ERROR;
		}

		/*
		 * mask off the interrupt we get in response to the manually
		 * sent CMD12
		 */
		if ((imx_data->multiblock_status == WAIT_FOR_INT) &&
		    ((val & SDHCI_INT_RESPONSE) == SDHCI_INT_RESPONSE)) {
			val &= ~SDHCI_INT_RESPONSE;
			writel(SDHCI_INT_RESPONSE, host->ioaddr +
						   SDHCI_INT_STATUS);
			imx_data->multiblock_status = NO_CMD_PENDING;
		}
	}

	return val;
}

static void esdhc_writel_le(struct sdhci_host *host, u32 val, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = pltfm_host->priv;
	u32 data;

	if (unlikely(reg == SDHCI_INT_ENABLE || reg == SDHCI_SIGNAL_ENABLE)) {
		if (val & SDHCI_INT_CARD_INT) {
			/*
			 * Clear and then set D3CD bit to avoid missing the
			 * card interrupt.  This is a eSDHC controller problem
			 * so we need to apply the following workaround: clear
			 * and set D3CD bit will make eSDHC re-sample the card
			 * interrupt. In case a card interrupt was lost,
			 * re-sample it by the following steps.
			 */
			data = readl(host->ioaddr + SDHCI_HOST_CONTROL);
			data &= ~ESDHC_CTRL_D3CD;
			writel(data, host->ioaddr + SDHCI_HOST_CONTROL);
			data |= ESDHC_CTRL_D3CD;
			writel(data, host->ioaddr + SDHCI_HOST_CONTROL);
		}
	}

	if (unlikely((imx_data->socdata->flags & ESDHC_FLAG_MULTIBLK_NO_INT)
				&& (reg == SDHCI_INT_STATUS)
				&& (val & SDHCI_INT_DATA_END))) {
			u32 v;
			v = readl(host->ioaddr + ESDHC_VENDOR_SPEC);
			v &= ~ESDHC_VENDOR_SPEC_SDIO_QUIRK;
			writel(v, host->ioaddr + ESDHC_VENDOR_SPEC);

			if (imx_data->multiblock_status == MULTIBLK_IN_PROCESS)
			{
				/* send a manual CMD12 with RESPTYP=none */
				data = MMC_STOP_TRANSMISSION << 24 |
				       SDHCI_CMD_ABORTCMD << 16;
				writel(data, host->ioaddr + SDHCI_TRANSFER_MODE);
				imx_data->multiblock_status = WAIT_FOR_INT;
			}
	}

	if (unlikely(reg == SDHCI_INT_ENABLE || reg == SDHCI_SIGNAL_ENABLE)) {
		if (val & SDHCI_INT_ADMA_ERROR) {
			val &= ~SDHCI_INT_ADMA_ERROR;
			val |= ESDHC_INT_VENDOR_SPEC_DMA_ERR;
		}
	}

	writel(val, host->ioaddr + reg);
}

static u16 esdhc_readw_le(struct sdhci_host *host, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = pltfm_host->priv;
	u16 ret = 0;
	u32 val;

	if (unlikely(reg == SDHCI_HOST_VERSION)) {
		reg ^= 2;
		if (esdhc_is_usdhc(imx_data)) {
			/*
			 * The usdhc register returns a wrong host version.
			 * Correct it here.
			 */
			return SDHCI_SPEC_300;
		}
	}

	if (unlikely(reg == SDHCI_HOST_CONTROL2)) {
		val = readl(host->ioaddr + ESDHC_VENDOR_SPEC);
		if (val & ESDHC_VENDOR_SPEC_VSELECT)
			ret |= SDHCI_CTRL_VDD_180;

		if (esdhc_is_usdhc(imx_data)) {
			if (imx_data->socdata->flags & ESDHC_FLAG_MAN_TUNING)
				val = readl(host->ioaddr + ESDHC_MIX_CTRL);
			else if (imx_data->socdata->flags & ESDHC_FLAG_STD_TUNING)
				/* the std tuning bits is in ACMD12_ERR for imx6sl */
				val = readl(host->ioaddr + SDHCI_ACMD12_ERR);
		}

		if (val & ESDHC_MIX_CTRL_EXE_TUNE)
			ret |= SDHCI_CTRL_EXEC_TUNING;
		if (val & ESDHC_MIX_CTRL_SMPCLK_SEL)
			ret |= SDHCI_CTRL_TUNED_CLK;

		ret |= (imx_data->uhs_mode & SDHCI_CTRL_UHS_MASK);
		ret &= ~SDHCI_CTRL_PRESET_VAL_ENABLE;

		return ret;
	}

	if (unlikely(reg == SDHCI_TRANSFER_MODE)) {
		if (esdhc_is_usdhc(imx_data)) {
			u32 m = readl(host->ioaddr + ESDHC_MIX_CTRL);
			ret = m & ESDHC_MIX_CTRL_SDHCI_MASK;
			/* Swap AC23 bit */
			if (m & ESDHC_MIX_CTRL_AC23EN) {
				ret &= ~ESDHC_MIX_CTRL_AC23EN;
				ret |= SDHCI_TRNS_AUTO_CMD23;
			}
		} else {
			ret = readw(host->ioaddr + SDHCI_TRANSFER_MODE);
		}

		return ret;
	}

	return readw(host->ioaddr + reg);
}

static void esdhc_writew_le(struct sdhci_host *host, u16 val, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = pltfm_host->priv;
	u32 new_val = 0;

	switch (reg) {
	case SDHCI_CLOCK_CONTROL:
		new_val = readl(host->ioaddr + ESDHC_VENDOR_SPEC);
		if (val & SDHCI_CLOCK_CARD_EN)
			new_val |= ESDHC_VENDOR_SPEC_FRC_SDCLK_ON;
		else
			new_val &= ~ESDHC_VENDOR_SPEC_FRC_SDCLK_ON;
			writel(new_val, host->ioaddr + ESDHC_VENDOR_SPEC);
		return;
	case SDHCI_HOST_CONTROL2:
		new_val = readl(host->ioaddr + ESDHC_VENDOR_SPEC);
		if (val & SDHCI_CTRL_VDD_180)
			new_val |= ESDHC_VENDOR_SPEC_VSELECT;
		else
			new_val &= ~ESDHC_VENDOR_SPEC_VSELECT;
		writel(new_val, host->ioaddr + ESDHC_VENDOR_SPEC);
		imx_data->uhs_mode = val & SDHCI_CTRL_UHS_MASK;
		if (imx_data->socdata->flags & ESDHC_FLAG_MAN_TUNING) {
			new_val = readl(host->ioaddr + ESDHC_MIX_CTRL);
			if (val & SDHCI_CTRL_TUNED_CLK)
				new_val |= ESDHC_MIX_CTRL_SMPCLK_SEL;
			else
				new_val &= ~ESDHC_MIX_CTRL_SMPCLK_SEL;
			writel(new_val , host->ioaddr + ESDHC_MIX_CTRL);
		} else if (imx_data->socdata->flags & ESDHC_FLAG_STD_TUNING) {
			u32 v = readl(host->ioaddr + SDHCI_ACMD12_ERR);
			u32 m = readl(host->ioaddr + ESDHC_MIX_CTRL);
			if (val & SDHCI_CTRL_TUNED_CLK) {
				v |= ESDHC_MIX_CTRL_SMPCLK_SEL;
			} else {
				v &= ~ESDHC_MIX_CTRL_SMPCLK_SEL;
				m &= ~ESDHC_MIX_CTRL_FBCLK_SEL;
			}

			if (val & SDHCI_CTRL_EXEC_TUNING) {
				v |= ESDHC_MIX_CTRL_EXE_TUNE;
				m |= ESDHC_MIX_CTRL_FBCLK_SEL;
			} else {
				v &= ~ESDHC_MIX_CTRL_EXE_TUNE;
			}

			writel(v, host->ioaddr + SDHCI_ACMD12_ERR);
			writel(m, host->ioaddr + ESDHC_MIX_CTRL);
		}
		return;
	case SDHCI_TRANSFER_MODE:
		if ((imx_data->socdata->flags & ESDHC_FLAG_MULTIBLK_NO_INT)
				&& (host->cmd->opcode == SD_IO_RW_EXTENDED)
				&& (host->cmd->data->blocks > 1)
				&& (host->cmd->data->flags & MMC_DATA_READ)) {
			u32 v;
			v = readl(host->ioaddr + ESDHC_VENDOR_SPEC);
			v |= ESDHC_VENDOR_SPEC_SDIO_QUIRK;
			writel(v, host->ioaddr + ESDHC_VENDOR_SPEC);
		}

		if (esdhc_is_usdhc(imx_data)) {
			u32 m = readl(host->ioaddr + ESDHC_MIX_CTRL);
			/* Swap AC23 bit */
			if (val & SDHCI_TRNS_AUTO_CMD23) {
				val &= ~SDHCI_TRNS_AUTO_CMD23;
				val |= ESDHC_MIX_CTRL_AC23EN;
			}
			m = val | (m & ~ESDHC_MIX_CTRL_SDHCI_MASK);
			writel(m, host->ioaddr + ESDHC_MIX_CTRL);
		} else {
			/*
			 * Postpone this write, we must do it together with a
			 * command write that is down below.
			 */
			imx_data->scratchpad = val;
		}
		return;
	case SDHCI_COMMAND:
		if (host->cmd->opcode == MMC_STOP_TRANSMISSION)
			val |= SDHCI_CMD_ABORTCMD;

		if ((host->cmd->opcode == MMC_SET_BLOCK_COUNT) &&
		    (imx_data->socdata->flags & ESDHC_FLAG_MULTIBLK_NO_INT))
			imx_data->multiblock_status = MULTIBLK_IN_PROCESS;

		if (esdhc_is_usdhc(imx_data))
			writel(val << 16,
			       host->ioaddr + SDHCI_TRANSFER_MODE);
		else
			writel(val << 16 | imx_data->scratchpad,
			       host->ioaddr + SDHCI_TRANSFER_MODE);
		return;
	case SDHCI_BLOCK_SIZE:
		val &= ~SDHCI_MAKE_BLKSZ(0x7, 0);
		break;
	}
	esdhc_clrset_le(host, 0xffff, val, reg);
}

static void esdhc_writeb_le(struct sdhci_host *host, u8 val, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = pltfm_host->priv;
	u32 new_val;
	u32 mask;

	switch (reg) {
	case SDHCI_POWER_CONTROL:
		/*
		 * FSL put some DMA bits here
		 * If your board has a regulator, code should be here
		 */
		return;
	case SDHCI_HOST_CONTROL:
		/* FSL messed up here, so we need to manually compose it. */
		new_val = val & SDHCI_CTRL_LED;
		/* ensure the endianness */
		new_val |= ESDHC_HOST_CONTROL_LE;
		/* bits 8&9 are reserved on mx25 */
		if (!is_imx25_esdhc(imx_data)) {
			/* DMA mode bits are shifted */
			new_val |= (val & SDHCI_CTRL_DMA_MASK) << 5;
		}

		/*
		 * Do not touch buswidth bits here. This is done in
		 * esdhc_pltfm_bus_width.
		 * Do not touch the D3CD bit either which is used for the
		 * SDIO interrupt errata workaround.
		 */
		mask = 0xffff & ~(ESDHC_CTRL_BUSWIDTH_MASK | ESDHC_CTRL_D3CD);

		esdhc_clrset_le(host, mask, new_val, reg);
		return;
	}
	esdhc_clrset_le(host, 0xff, val, reg);

	/*
	 * The esdhc has a design violation to SDHC spec which tells
	 * that software reset should not affect card detection circuit.
	 * But esdhc clears its SYSCTL register bits [0..2] during the
	 * software reset.  This will stop those clocks that card detection
	 * circuit relies on.  To work around it, we turn the clocks on back
	 * to keep card detection circuit functional.
	 */
	if ((reg == SDHCI_SOFTWARE_RESET) && (val & 1)) {
		esdhc_clrset_le(host, 0x7, 0x7, ESDHC_SYSTEM_CONTROL);
		/*
		 * The reset on usdhc fails to clear MIX_CTRL register.
		 * Do it manually here.
		 */
		if (esdhc_is_usdhc(imx_data)) {
			/* the tuning bits should be kept during reset */
			new_val = readl(host->ioaddr + ESDHC_MIX_CTRL);
			writel(new_val & ESDHC_MIX_CTRL_TUNING_MASK,
					host->ioaddr + ESDHC_MIX_CTRL);
			imx_data->is_ddr = 0;
		}
	}
}

static unsigned int esdhc_pltfm_get_min_clock(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	return clk_get_rate(pltfm_host->clk) / 256 / 16;
}

static inline void esdhc_pltfm_set_clock(struct sdhci_host *host,
						unsigned int clock)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = pltfm_host->priv;
	int pre_div = 2;
	int div = 1;
	u32 temp, val;

	if (clock == 0) {
		if (esdhc_is_usdhc(imx_data)) {
			val = readl(host->ioaddr + ESDHC_VENDOR_SPEC);
			writel(val & ~ESDHC_VENDOR_SPEC_FRC_SDCLK_ON,
					host->ioaddr + ESDHC_VENDOR_SPEC);
		}
		goto out;
	}
	if (clock > imx_data->max_clock)
		clock = imx_data->max_clock;
	if (esdhc_is_usdhc(imx_data) && !imx_data->is_ddr)
		pre_div = 1;

	temp = sdhci_readl(host, ESDHC_SYSTEM_CONTROL);
	temp &= ~(ESDHC_CLOCK_IPGEN | ESDHC_CLOCK_HCKEN | ESDHC_CLOCK_PEREN
		| ESDHC_CLOCK_MASK);
	sdhci_writel(host, temp, ESDHC_SYSTEM_CONTROL);

	while (host->max_clk / pre_div / 16 > clock && pre_div < 256)
		pre_div *= 2;

	while (host->max_clk / pre_div / div > clock && div < 16)
		div++;

	host->mmc->actual_clock = host->max_clk / pre_div / div;
	dev_dbg(mmc_dev(host->mmc), "desired SD clock: %d, actual: %d\n",
			clock, host->mmc->actual_clock);

	if (imx_data->is_ddr)
		pre_div >>= 2;
	else
		pre_div >>= 1;
	div--;

	temp = sdhci_readl(host, ESDHC_SYSTEM_CONTROL);
	temp |= (ESDHC_CLOCK_IPGEN | ESDHC_CLOCK_HCKEN | ESDHC_CLOCK_PEREN
		| (div << ESDHC_DIVIDER_SHIFT)
		| (pre_div << ESDHC_PREDIV_SHIFT));
	sdhci_writel(host, temp, ESDHC_SYSTEM_CONTROL);

	if (esdhc_is_usdhc(imx_data)) {
		val = readl(host->ioaddr + ESDHC_VENDOR_SPEC);
		writel(val | ESDHC_VENDOR_SPEC_FRC_SDCLK_ON,
		host->ioaddr + ESDHC_VENDOR_SPEC);
	}

	mdelay(1);
out:
	host->clock = clock;
}

static unsigned int esdhc_pltfm_get_ro(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = pltfm_host->priv;
	struct esdhc_platform_data *boarddata = &imx_data->boarddata;

	switch (boarddata->wp_type) {
	case ESDHC_WP_GPIO:
		return mmc_gpio_get_ro(host->mmc);
	case ESDHC_WP_CONTROLLER:
		return !(readl(host->ioaddr + SDHCI_PRESENT_STATE) &
			       SDHCI_WRITE_PROTECT);
	case ESDHC_WP_NONE:
		break;
	}

	return -ENOSYS;
}

static int esdhc_pltfm_bus_width(struct sdhci_host *host, int width)
{
	u32 ctrl;

	switch (width) {
	case MMC_BUS_WIDTH_8:
		ctrl = ESDHC_CTRL_8BITBUS;
		break;
	case MMC_BUS_WIDTH_4:
		ctrl = ESDHC_CTRL_4BITBUS;
		break;
	default:
		ctrl = 0;
		break;
	}

	esdhc_clrset_le(host, ESDHC_CTRL_BUSWIDTH_MASK, ctrl,
			SDHCI_HOST_CONTROL);

	return 0;
}

static void esdhc_reset(struct sdhci_host *host, u32 rst_bits)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = pltfm_host->priv;
	u32 timeout;
	u32 reg;

	reg = readl(host->ioaddr + ESDHC_SYSTEM_CONTROL);
	reg |= rst_bits;
	writel(reg, host->ioaddr + ESDHC_SYSTEM_CONTROL);

	/* Wait for max 100ms */
	timeout = 100;

	/* hw clears the bit when it's done */
	while (readl(host->ioaddr + ESDHC_SYSTEM_CONTROL) & rst_bits) {
		if (timeout == 0) {
			dev_err(mmc_dev(host->mmc),
				"Reset never completes!\n");
			return;
		}
		timeout--;
		mdelay(1);
	}

	/*
	* The RSTA, reset all, on usdhc will not clear following regs:
	* > SDHCI_MIX_CTRL
	* > SDHCI_TUNE_CTRL_STATUS
	*
	* Do it manually here.
	*/
	if ((rst_bits & ESDHC_SYS_CTRL_RSTA) && is_imx6q_usdhc(imx_data)) {
		writel(0, host->ioaddr + ESDHC_MIX_CTRL);
		writel(0, host->ioaddr + ESDHC_TUNE_CTRL_STATUS);
		/* FIXME: delay for clear tuning status or some cards may not work */
		mdelay(1);
	}
}

static void esdhc_prepare_tuning(struct sdhci_host *host, u32 val)
{
	u32 reg;

	/* reset controller before tuning or it may fail on some cards */
	esdhc_reset(host, ESDHC_SYS_CTRL_RSTA);

	pm_runtime_get_sync(host->mmc->parent);
	reg = readl(host->ioaddr + ESDHC_MIX_CTRL);
	reg |= ESDHC_MIX_CTRL_EXE_TUNE | ESDHC_MIX_CTRL_SMPCLK_SEL |
			ESDHC_MIX_CTRL_FBCLK_SEL;
	writel(reg, host->ioaddr + ESDHC_MIX_CTRL);
	writel(val << 8, host->ioaddr + ESDHC_TUNE_CTRL_STATUS);
	dev_dbg(mmc_dev(host->mmc),
		"tunning with delay 0x%x ESDHC_TUNE_CTRL_STATUS 0x%x\n",
			val, readl(host->ioaddr + ESDHC_TUNE_CTRL_STATUS));
}

static void esdhc_request_done(struct mmc_request *mrq)
{
	complete(&mrq->completion);
}

static int esdhc_send_tuning_cmd(struct sdhci_host *host, u32 opcode)
{
	struct mmc_command cmd = {0};
	struct mmc_request mrq = {0};
	struct mmc_data data = {0};
	struct scatterlist sg;
	char tuning_pattern[ESDHC_TUNING_BLOCK_PATTERN_LEN];

	cmd.opcode = opcode;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = ESDHC_TUNING_BLOCK_PATTERN_LEN;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;

	sg_init_one(&sg, tuning_pattern, sizeof(tuning_pattern));

	mrq.cmd = &cmd;
	mrq.cmd->mrq = &mrq;
	mrq.data = &data;
	mrq.data->mrq = &mrq;
	mrq.cmd->data = mrq.data;

	mrq.done = esdhc_request_done;
	init_completion(&(mrq.completion));

	disable_irq(host->irq);
	spin_lock(&host->lock);
	host->mrq = &mrq;

	sdhci_send_command(host, mrq.cmd);

	spin_unlock(&host->lock);
	enable_irq(host->irq);

	wait_for_completion(&mrq.completion);

	if (cmd.error)
		return cmd.error;
	if (data.error)
		return data.error;

	return 0;
}

static void esdhc_post_tuning(struct sdhci_host *host)
{
	u32 reg;

	reg = readl(host->ioaddr + ESDHC_MIX_CTRL);
	reg &= ~ESDHC_MIX_CTRL_EXE_TUNE;
	writel(reg, host->ioaddr + ESDHC_MIX_CTRL);
}

static int esdhc_executing_tuning(struct sdhci_host *host, u32 opcode)
{
	int min, max, avg, ret;

	/* find the mininum delay first which can pass tuning */
	min = ESDHC_TUNE_CTRL_MIN;
	while (min < ESDHC_TUNE_CTRL_MAX) {
		esdhc_prepare_tuning(host, min);
		if (!esdhc_send_tuning_cmd(host, opcode))
			break;
		min += ESDHC_TUNE_CTRL_STEP;
	}

	/* find the maxinum delay which can not pass tuning */
	max = min + ESDHC_TUNE_CTRL_STEP;
	while (max < ESDHC_TUNE_CTRL_MAX) {
		esdhc_prepare_tuning(host, max);
		if (esdhc_send_tuning_cmd(host, opcode)) {
			max -= ESDHC_TUNE_CTRL_STEP;
			break;
		}
		max += ESDHC_TUNE_CTRL_STEP;
	}

	/* use average delay to get the best timing */
	avg = (min + max) / 2;
	esdhc_prepare_tuning(host, avg);
	ret = esdhc_send_tuning_cmd(host, opcode);
	esdhc_post_tuning(host);

	dev_dbg(mmc_dev(host->mmc), "tunning %s at 0x%x ret %d\n",
		ret ? "failed" : "passed", avg, ret);

	return ret;
}

static int esdhc_change_pinstate(struct sdhci_host *host,
						unsigned int uhs)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = pltfm_host->priv;
	struct pinctrl_state *pinctrl;

	dev_dbg(mmc_dev(host->mmc), "change pinctrl state for uhs %d\n", uhs);

	if (IS_ERR(imx_data->pinctrl) ||
		IS_ERR(imx_data->pins_default) ||
		IS_ERR(imx_data->pins_100mhz) ||
		IS_ERR(imx_data->pins_200mhz))
		return -EINVAL;

	switch (uhs) {
	case MMC_TIMING_UHS_SDR50:
		pinctrl = imx_data->pins_100mhz;
		break;
	case MMC_TIMING_UHS_SDR104:
	case MMC_TIMING_MMC_HS200:
		pinctrl = imx_data->pins_200mhz;
		break;
	default:
		/* back to default state for other legacy timing */
		pinctrl = imx_data->pins_default;
	}

	return pinctrl_select_state(imx_data->pinctrl, pinctrl);
}

static int esdhc_set_uhs_signaling(struct sdhci_host *host, unsigned int uhs)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = pltfm_host->priv;
	struct esdhc_platform_data *boarddata = &imx_data->boarddata;

	switch (uhs) {
	case MMC_TIMING_UHS_SDR12:
		imx_data->uhs_mode = SDHCI_CTRL_UHS_SDR12;
		break;
	case MMC_TIMING_UHS_SDR25:
		imx_data->uhs_mode = SDHCI_CTRL_UHS_SDR25;
		break;
	case MMC_TIMING_UHS_SDR50:
		imx_data->uhs_mode = SDHCI_CTRL_UHS_SDR50;
		break;
	case MMC_TIMING_UHS_SDR104:
	case MMC_TIMING_MMC_HS200:
		imx_data->uhs_mode = SDHCI_CTRL_UHS_SDR104;
		break;
	case MMC_TIMING_UHS_DDR50:
		imx_data->uhs_mode = SDHCI_CTRL_UHS_DDR50;
		writel(readl(host->ioaddr + ESDHC_MIX_CTRL) |
				ESDHC_MIX_CTRL_DDREN,
				host->ioaddr + ESDHC_MIX_CTRL);
		imx_data->is_ddr = 1;
		if (boarddata->delay_line) {
			u32 v;
			v = boarddata->delay_line <<
				ESDHC_DLL_OVERRIDE_VAL_SHIFT |
				(1 << ESDHC_DLL_OVERRIDE_EN_SHIFT);
			if (is_imx53_esdhc(imx_data))
				v <<= 1;
			writel(v, host->ioaddr + ESDHC_DLL_CTRL);
		}
		break;
	}

	return esdhc_change_pinstate(host, uhs);
}

static unsigned int esdhc_get_max_timeout_counter(struct sdhci_host *host)
{
	return 1 << 28;
}

static void sdhci_platform_set_power
	(struct sdhci_host *host, int on)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = pltfm_host ? pltfm_host->priv : 0;
	struct esdhc_platform_data *boarddata = &imx_data->boarddata;

	if (imx_data && gpio_is_valid(boarddata->power_gpio)) {
		dev_dbg(mmc_dev(host->mmc),"%s: imx_data %p, power-gpio %d, on %d\n",
			 __func__, imx_data, imx_data ? boarddata->power_gpio : -2, on);
		gpio_direction_output(boarddata->power_gpio, on);
	}
}

static struct sdhci_ops sdhci_esdhc_ops = {
	.read_l = esdhc_readl_le,
	.read_w = esdhc_readw_le,
	.write_l = esdhc_writel_le,
	.write_w = esdhc_writew_le,
	.write_b = esdhc_writeb_le,
	.set_clock = esdhc_pltfm_set_clock,
	.get_max_clock = sdhci_pltfm_clk_get_max_clock,
	.get_min_clock = esdhc_pltfm_get_min_clock,
	.get_ro = esdhc_pltfm_get_ro,
	.platform_bus_width = esdhc_pltfm_bus_width,
	.set_uhs_signaling = esdhc_set_uhs_signaling,
};

static const struct sdhci_pltfm_data sdhci_esdhc_imx_pdata = {
	.quirks = ESDHC_DEFAULT_QUIRKS | SDHCI_QUIRK_NO_HISPD_BIT
			| SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC
			| SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC
			| SDHCI_QUIRK_BROKEN_CARD_DETECTION,
	.ops = &sdhci_esdhc_ops,
};

#ifdef CONFIG_OF
static int
sdhci_esdhc_imx_probe_dt(struct platform_device *pdev,
			 struct esdhc_platform_data *boarddata)
{
	struct device_node *np = pdev->dev.of_node;
	struct sdhci_host *host = platform_get_drvdata(pdev);

	if (!np)
		return -ENODEV;

	if (of_get_property(np, "non-removable", NULL))
		boarddata->cd_type = ESDHC_CD_PERMANENT;

	if (of_get_property(np, "fsl,cd-controller", NULL))
		boarddata->cd_type = ESDHC_CD_CONTROLLER;

	if (of_get_property(np, "fsl,wp-controller", NULL))
		boarddata->wp_type = ESDHC_WP_CONTROLLER;

	boarddata->cd_gpio = of_get_named_gpio(np, "cd-gpios", 0);
	if (gpio_is_valid(boarddata->cd_gpio))
		boarddata->cd_type = ESDHC_CD_GPIO;

	boarddata->wp_gpio = of_get_named_gpio(np, "wp-gpios", 0);
	if (gpio_is_valid(boarddata->wp_gpio))
		boarddata->wp_type = ESDHC_WP_GPIO;

	boarddata->power_gpio = of_get_named_gpio(np, "power-gpio", 0);
	dev_info(mmc_dev(host->mmc),
		"%s: power-gpio %d\n", __func__, boarddata->power_gpio);
	if (gpio_is_valid(boarddata->power_gpio)) {
		int rc = gpio_request(boarddata->power_gpio, "sdhci_power");
		if (rc) {
			dev_err(mmc_dev(host->mmc),
				"failed to allocate power gpio\n");
			boarddata->power_gpio = -ENODEV;
		}
		gpio_direction_output(boarddata->power_gpio, 0);
	}

	of_property_read_u32(np, "bus-width", &boarddata->max_bus_width);
	of_property_read_u32(np, "max-clock", &boarddata->max_clock);

	if (of_find_property(np, "no-1-8-v", NULL))
		boarddata->support_vsel = false;
	else
		boarddata->support_vsel = true;

	if (of_find_property(np, "vqmmc-1-8-v", NULL))
		boarddata->vqmmc_18v = true;

	if (of_property_read_u32(np, "fsl,delay-line", &boarddata->delay_line))
		boarddata->delay_line = 0;

	if (of_find_property(np, "keep-power-in-suspend", NULL))
		host->mmc->pm_caps |= MMC_PM_KEEP_POWER;

	if (of_find_property(np, "enable-sdio-wakeup", NULL))
		host->mmc->pm_caps |= MMC_PM_WAKE_SDIO_IRQ;

	return 0;
}
#else
static inline int
sdhci_esdhc_imx_probe_dt(struct platform_device *pdev,
			 struct esdhc_platform_data *boarddata)
{
	return -ENODEV;
}
#endif

static int sdhci_esdhc_imx_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(imx_esdhc_dt_ids, &pdev->dev);
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_host *host;
	struct esdhc_platform_data *boarddata;
	int err;
	struct pltfm_imx_data *imx_data;

	host = sdhci_pltfm_init(pdev, &sdhci_esdhc_imx_pdata);
	if (IS_ERR(host))
		return PTR_ERR(host);

	pltfm_host = sdhci_priv(host);

	imx_data = devm_kzalloc(&pdev->dev, sizeof(*imx_data), GFP_KERNEL);
	if (!imx_data) {
		err = -ENOMEM;
		goto free_sdhci;
	}

	imx_data->socdata = of_id ? of_id->data : (struct esdhc_soc_data *)
						  pdev->id_entry->driver_data;
	pltfm_host->priv = imx_data;

	imx_data->clk_ipg = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(imx_data->clk_ipg)) {
		err = PTR_ERR(imx_data->clk_ipg);
		goto free_sdhci;
	}

	imx_data->clk_ahb = devm_clk_get(&pdev->dev, "ahb");
	if (IS_ERR(imx_data->clk_ahb)) {
		err = PTR_ERR(imx_data->clk_ahb);
		goto free_sdhci;
	}

	imx_data->clk_per = devm_clk_get(&pdev->dev, "per");
	if (IS_ERR(imx_data->clk_per)) {
		err = PTR_ERR(imx_data->clk_per);
		goto free_sdhci;
	}

	pltfm_host->clk = imx_data->clk_per;

	if (imx_data->socdata->flags & ESDHC_FLAG_BUSFREQ)
		request_bus_freq(BUS_FREQ_HIGH);

	clk_prepare_enable(imx_data->clk_per);
	clk_prepare_enable(imx_data->clk_ipg);
	clk_prepare_enable(imx_data->clk_ahb);

	imx_data->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(imx_data->pinctrl)) {
		err = PTR_ERR(imx_data->pinctrl);
		goto disable_clk;
	}

	imx_data->pins_default = pinctrl_lookup_state(imx_data->pinctrl,
						PINCTRL_STATE_DEFAULT);
	if (IS_ERR(imx_data->pins_default)) {
		err = PTR_ERR(imx_data->pins_default);
		dev_err(mmc_dev(host->mmc), "could not get default state\n");
		goto disable_clk;
	}

	host->quirks |= SDHCI_QUIRK_BROKEN_TIMEOUT_VAL;

	if (imx_data->socdata->flags & ESDHC_FLAG_ENGCM07207)
		/* Fix errata ENGcm07207 present on i.MX25 and i.MX35 */
		host->quirks |= SDHCI_QUIRK_NO_MULTIBLOCK
			| SDHCI_QUIRK_BROKEN_ADMA;

	/*
	 * The imx6q ROM code will change the default watermark level setting
	 * to something insane.  Change it back here.
	 */
	if (esdhc_is_usdhc(imx_data)) {
		writel(0x08100810, host->ioaddr + ESDHC_WTMK_LVL);
		host->quirks2 |= SDHCI_QUIRK2_PRESET_VALUE_BROKEN |
					SDHCI_QUIRK2_NOSTD_TIMEOUT_COUNTER;
		host->mmc->caps |= MMC_CAP_1_8V_DDR;

		/*
		 * errata ESDHC_FLAG_ERR004536 fix for MX6Q TO1.2 and MX6DL
		 * TO1.1, it's harmless for MX6SL
		 */
		writel(readl(host->ioaddr + 0x6c) | BIT(7), host->ioaddr + 0x6c);
		sdhci_esdhc_ops.get_max_timeout_counter =
					esdhc_get_max_timeout_counter;
	}

	if (imx_data->socdata->flags & ESDHC_FLAG_MAN_TUNING)
		sdhci_esdhc_ops.platform_execute_tuning =
					esdhc_executing_tuning;

	if (imx_data->socdata->flags & ESDHC_FLAG_STD_TUNING)
		writel(readl(host->ioaddr + ESDHC_TUNING_CTRL) |
			ESDHC_STD_TUNING_EN | ESDHC_TUNING_START_TAP,
			host->ioaddr + ESDHC_TUNING_CTRL);

	if (imx_data->socdata->flags & ESDHC_FLAG_ERR004536)
		host->quirks |= SDHCI_QUIRK_BROKEN_ADMA;

	boarddata = &imx_data->boarddata;
	if (sdhci_esdhc_imx_probe_dt(pdev, boarddata) < 0) {
		if (!host->mmc->parent->platform_data) {
			dev_err(mmc_dev(host->mmc), "no board data!\n");
			err = -EINVAL;
			goto disable_clk;
		}
		imx_data->boarddata = *((struct esdhc_platform_data *)
					host->mmc->parent->platform_data);
	}
	imx_data->max_clock = ~0;
	if (boarddata->max_clock) {
		imx_data->max_clock = boarddata->max_clock;
		dev_info(mmc_dev(host->mmc),
			"clock limited to %d\n", imx_data->max_clock);
	}
	if (gpio_is_valid(boarddata->power_gpio))
		sdhci_esdhc_ops. platform_set_power = sdhci_platform_set_power;

	/* write_protect */
	if (boarddata->wp_type == ESDHC_WP_GPIO) {
		err = mmc_gpio_request_ro(host->mmc, boarddata->wp_gpio);
		if (err) {
			dev_err(mmc_dev(host->mmc),
				"failed to request write-protect gpio!\n");
			goto disable_clk;
		}
		host->mmc->caps2 |= MMC_CAP2_RO_ACTIVE_HIGH;
	}

	/* card_detect */
	switch (boarddata->cd_type) {
	case ESDHC_CD_GPIO:
		err = mmc_gpio_request_cd(host->mmc, boarddata->cd_gpio);
		if (err) {
			dev_err(mmc_dev(host->mmc),
				"failed to request card-detect gpio!\n");
			goto disable_clk;
		}
		/* fall through */

	case ESDHC_CD_CONTROLLER:
		/* we have a working card_detect back */
		host->quirks &= ~SDHCI_QUIRK_BROKEN_CARD_DETECTION;
		break;

	case ESDHC_CD_PERMANENT:
		host->mmc->caps |= MMC_CAP_NONREMOVABLE;
		break;

	case ESDHC_CD_NONE:
		break;
	}

	switch (boarddata->max_bus_width) {
	case 8:
		host->mmc->caps |= MMC_CAP_8_BIT_DATA | MMC_CAP_4_BIT_DATA;
		break;
	case 4:
		host->mmc->caps |= MMC_CAP_4_BIT_DATA;
		break;
	case 1:
	default:
		host->quirks |= SDHCI_QUIRK_FORCE_1_BIT_DATA;
		break;
	}

	/* sdr50 and sdr104 needs work on 1.8v signal voltage */
	if ((boarddata->support_vsel) && esdhc_is_usdhc(imx_data)) {
		imx_data->pins_100mhz = pinctrl_lookup_state(imx_data->pinctrl,
						ESDHC_PINCTRL_STATE_100MHZ);
		imx_data->pins_200mhz = pinctrl_lookup_state(imx_data->pinctrl,
						ESDHC_PINCTRL_STATE_200MHZ);
		if (IS_ERR(imx_data->pins_100mhz) ||
				IS_ERR(imx_data->pins_200mhz)) {
			dev_warn(mmc_dev(host->mmc),
				"could not get ultra high speed state, work on normal mode\n");
			/* fall back to not support uhs by specify no 1.8v quirk */
			host->quirks2 |= SDHCI_QUIRK2_NO_1_8_V;
		}
	} else {
		host->quirks2 |= SDHCI_QUIRK2_NO_1_8_V;
	}
	if (boarddata->vqmmc_18v)
		host->quirks2 |= SDHCI_QUIRK2_VQMMC_1_8_V;

	if (host->mmc->pm_caps & MMC_PM_KEEP_POWER &&
		host->mmc->pm_caps & MMC_PM_WAKE_SDIO_IRQ)
		device_init_wakeup(&pdev->dev, 1);

	err = sdhci_add_host(host);
	if (err)
		goto disable_clk;

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, 50);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_suspend_ignore_children(&pdev->dev, 1);

	return 0;

disable_clk:
	clk_disable_unprepare(imx_data->clk_per);
	clk_disable_unprepare(imx_data->clk_ipg);
	clk_disable_unprepare(imx_data->clk_ahb);
	if (imx_data->socdata->flags & ESDHC_FLAG_BUSFREQ)
		release_bus_freq(BUS_FREQ_HIGH);
free_sdhci:
	sdhci_pltfm_free(pdev);
	return err;
}

static int sdhci_esdhc_imx_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	int dead = (readl(host->ioaddr + SDHCI_INT_STATUS) == 0xffffffff);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = pltfm_host ? pltfm_host->priv : 0;
	struct esdhc_platform_data *boarddata = &imx_data->boarddata;

	if (imx_data && gpio_is_valid(boarddata->power_gpio)) {
		gpio_direction_output(boarddata->power_gpio, 0);
		gpio_free(boarddata->power_gpio);
	}

	sdhci_remove_host(host, dead);

	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	sdhci_pltfm_free(pdev);

	return 0;
}

#ifdef CONFIG_PM_RUNTIME
static int sdhci_esdhc_runtime_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = pltfm_host->priv;
	int ret;

	ret = sdhci_runtime_suspend_host(host);

	clk_disable_unprepare(imx_data->clk_per);
	clk_disable_unprepare(imx_data->clk_ipg);
	clk_disable_unprepare(imx_data->clk_ahb);

	if (imx_data->socdata->flags & ESDHC_FLAG_BUSFREQ)
		release_bus_freq(BUS_FREQ_HIGH);

	return ret;
}

static int sdhci_esdhc_runtime_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = pltfm_host->priv;

	if (imx_data->socdata->flags & ESDHC_FLAG_BUSFREQ)
		request_bus_freq(BUS_FREQ_HIGH);

	clk_prepare_enable(imx_data->clk_per);
	clk_prepare_enable(imx_data->clk_ipg);
	clk_prepare_enable(imx_data->clk_ahb);

	return sdhci_runtime_resume_host(host);
}
#endif

static const struct dev_pm_ops sdhci_esdhc_pmops = {
	SET_SYSTEM_SLEEP_PM_OPS(sdhci_pltfm_suspend, sdhci_pltfm_resume)
	SET_RUNTIME_PM_OPS(sdhci_esdhc_runtime_suspend,
				sdhci_esdhc_runtime_resume, NULL)
};

static struct platform_driver sdhci_esdhc_imx_driver = {
	.driver		= {
		.name	= "sdhci-esdhc-imx",
		.owner	= THIS_MODULE,
		.of_match_table = imx_esdhc_dt_ids,
		.pm	= &sdhci_esdhc_pmops,
	},
	.id_table	= imx_esdhc_devtype,
	.probe		= sdhci_esdhc_imx_probe,
	.remove		= sdhci_esdhc_imx_remove,
};

module_platform_driver(sdhci_esdhc_imx_driver);

MODULE_DESCRIPTION("SDHCI driver for Freescale i.MX eSDHC");
MODULE_AUTHOR("Wolfram Sang <w.sang@pengutronix.de>");
MODULE_LICENSE("GPL v2");
