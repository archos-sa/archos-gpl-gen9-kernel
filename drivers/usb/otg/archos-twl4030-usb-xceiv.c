/*
 * archos_twl4030_usb_xceiv - Archos specific USB transceiver driver, 
 *                    talking to OMAP OTG controller plus Archos Gen9 stuff
 *
 * Copyright (C) 2011 Archos S.A.
 * Contact: Niklas Schroeter <schroeter@archos.com>
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
 
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/usb/otg.h>
#include <linux/usb/ulpi.h>
#include <linux/usb/gpio_vbus.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#include <linux/freezer.h>

#define CONFIG_TPS65921_CHARGE_DETECT

/* Register defines */

#define VENDOR_ID_LO			0x00
#define VENDOR_ID_HI			0x01
#define PRODUCT_ID_LO			0x02
#define PRODUCT_ID_HI			0x03

#define FUNC_CTRL			0x04
#define FUNC_CTRL_SET			0x05
#define FUNC_CTRL_CLR			0x06
#define FUNC_CTRL_SUSPENDM		(1 << 6)
#define FUNC_CTRL_RESET			(1 << 5)
#define FUNC_CTRL_OPMODE_MASK		(3 << 3) /* bits 3 and 4 */
#define FUNC_CTRL_OPMODE_NORMAL		(0 << 3)
#define FUNC_CTRL_OPMODE_NONDRIVING	(1 << 3)
#define FUNC_CTRL_OPMODE_DISABLE_BIT_NRZI	(2 << 3)
#define FUNC_CTRL_TERMSELECT		(1 << 2)
#define FUNC_CTRL_XCVRSELECT_MASK	(3 << 0) /* bits 0 and 1 */
#define FUNC_CTRL_XCVRSELECT_HS		(0 << 0)
#define FUNC_CTRL_XCVRSELECT_FS		(1 << 0)
#define FUNC_CTRL_XCVRSELECT_LS		(2 << 0)
#define FUNC_CTRL_XCVRSELECT_FS4LS	(3 << 0)

#define IFC_CTRL			0x07
#define IFC_CTRL_SET			0x08
#define IFC_CTRL_CLR			0x09
#define IFC_CTRL_INTERFACE_PROTECT_DISABLE	(1 << 7)
#define IFC_CTRL_AUTORESUME		(1 << 4)
#define IFC_CTRL_CLOCKSUSPENDM		(1 << 3)
#define IFC_CTRL_CARKITMODE		(1 << 2)
#define IFC_CTRL_FSLSSERIALMODE_3PIN	(1 << 1)

#define TWL4030_OTG_CTRL		0x0A
#define TWL4030_OTG_CTRL_SET		0x0B
#define TWL4030_OTG_CTRL_CLR		0x0C
#define TWL4030_OTG_CTRL_DRVVBUS	(1 << 5)
#define TWL4030_OTG_CTRL_CHRGVBUS	(1 << 4)
#define TWL4030_OTG_CTRL_DISCHRGVBUS	(1 << 3)
#define TWL4030_OTG_CTRL_DMPULLDOWN	(1 << 2)
#define TWL4030_OTG_CTRL_DPPULLDOWN	(1 << 1)
#define TWL4030_OTG_CTRL_IDPULLUP	(1 << 0)

#define USB_INT_EN_RISE			0x0D
#define USB_INT_EN_RISE_SET		0x0E
#define USB_INT_EN_RISE_CLR		0x0F
#define USB_INT_EN_FALL			0x10
#define USB_INT_EN_FALL_SET		0x11
#define USB_INT_EN_FALL_CLR		0x12
#define USB_INT_STS			0x13
#define USB_INT_LATCH			0x14
#define USB_INT_IDGND			(1 << 4)
#define USB_INT_SESSEND			(1 << 3)
#define USB_INT_SESSVALID		(1 << 2)
#define USB_INT_VBUSVALID		(1 << 1)
#define USB_INT_HOSTDISCONNECT		(1 << 0)

#define CARKIT_CTRL			0x19
#define CARKIT_CTRL_SET			0x1A
#define CARKIT_CTRL_CLR			0x1B
#define CARKIT_CTRL_MICEN		(1 << 6)
#define CARKIT_CTRL_SPKRIGHTEN		(1 << 5)
#define CARKIT_CTRL_SPKLEFTEN		(1 << 4)
#define CARKIT_CTRL_RXDEN		(1 << 3)
#define CARKIT_CTRL_TXDEN		(1 << 2)
#define CARKIT_CTRL_IDGNDDRV		(1 << 1)
#define CARKIT_CTRL_CARKITPWR		(1 << 0)
#define CARKIT_PLS_CTRL			0x22
#define CARKIT_PLS_CTRL_SET		0x23
#define CARKIT_PLS_CTRL_CLR		0x24
#define CARKIT_PLS_CTRL_SPKRRIGHT_BIASEN	(1 << 3)
#define CARKIT_PLS_CTRL_SPKRLEFT_BIASEN	(1 << 2)
#define CARKIT_PLS_CTRL_RXPLSEN		(1 << 1)
#define CARKIT_PLS_CTRL_TXPLSEN		(1 << 0)

#define CARKIT_ANA_CTRL			0xBB
#define SEL_MADC_MCPC			(1 << 3)

#define MCPC_CTRL			0x30
#define MCPC_CTRL_SET			0x31
#define MCPC_CTRL_CLR			0x32
#define MCPC_CTRL_RTSOL			(1 << 7)
#define MCPC_CTRL_EXTSWR		(1 << 6)
#define MCPC_CTRL_EXTSWC		(1 << 5)
#define MCPC_CTRL_VOICESW		(1 << 4)
#define MCPC_CTRL_OUT64K		(1 << 3)
#define MCPC_CTRL_RTSCTSSW		(1 << 2)
#define MCPC_CTRL_HS_UART		(1 << 0)

#define MCPC_IO_CTRL			0x33
#define MCPC_IO_CTRL_SET		0x34
#define MCPC_IO_CTRL_CLR		0x35
#define MCPC_IO_CTRL_MICBIASEN		(1 << 5)
#define MCPC_IO_CTRL_CTS_NPU		(1 << 4)
#define MCPC_IO_CTRL_RXD_PU		(1 << 3)
#define MCPC_IO_CTRL_TXDTYP		(1 << 2)
#define MCPC_IO_CTRL_CTSTYP		(1 << 1)
#define MCPC_IO_CTRL_RTSTYP		(1 << 0)

#define MCPC_CTRL2			0x36
#define MCPC_CTRL2_SET			0x37
#define MCPC_CTRL2_CLR			0x38
#define MCPC_CTRL2_MCPC_CK_EN		(1 << 0)

#define OTHER_FUNC_CTRL			0x80
#define OTHER_FUNC_CTRL_SET		0x81
#define OTHER_FUNC_CTRL_CLR		0x82
#define OTHER_FUNC_CTRL_BDIS_ACON_EN	(1 << 4)
#define OTHER_FUNC_CTRL_FIVEWIRE_MODE	(1 << 2)

#define OTHER_IFC_CTRL			0x83
#define OTHER_IFC_CTRL_SET		0x84
#define OTHER_IFC_CTRL_CLR		0x85
#define OTHER_IFC_CTRL_OE_INT_EN	(1 << 6)
#define OTHER_IFC_CTRL_CEA2011_MODE	(1 << 5)
#define OTHER_IFC_CTRL_FSLSSERIALMODE_4PIN	(1 << 4)
#define OTHER_IFC_CTRL_HIZ_ULPI_60MHZ_OUT	(1 << 3)
#define OTHER_IFC_CTRL_HIZ_ULPI		(1 << 2)
#define OTHER_IFC_CTRL_ALT_INT_REROUTE	(1 << 0)

#define OTHER_INT_EN_RISE		0x86
#define OTHER_INT_EN_RISE_SET		0x87
#define OTHER_INT_EN_RISE_CLR		0x88
#define OTHER_INT_EN_FALL		0x89
#define OTHER_INT_EN_FALL_SET		0x8A
#define OTHER_INT_EN_FALL_CLR		0x8B
#define OTHER_INT_STS			0x8C
#define OTHER_INT_LATCH			0x8D
#define OTHER_INT_VB_SESS_VLD		(1 << 7)
#define OTHER_INT_DM_HI			(1 << 6) /* not valid for "latch" reg */
#define OTHER_INT_DP_HI			(1 << 5) /* not valid for "latch" reg */
#define OTHER_INT_BDIS_ACON		(1 << 3) /* not valid for "fall" regs */
#define OTHER_INT_MANU			(1 << 1)
#define OTHER_INT_ABNORMAL_STRESS	(1 << 0)

#define ID_STATUS			0x96
#define ID_RES_FLOAT			(1 << 4)
#define ID_RES_440K			(1 << 3)
#define ID_RES_200K			(1 << 2)
#define ID_RES_102K			(1 << 1)
#define ID_RES_GND			(1 << 0)

#define POWER_CTRL			0xAC
#define POWER_CTRL_SET			0xAD
#define POWER_CTRL_CLR			0xAE
#define POWER_CTRL_OTG_ENAB		(1 << 5)

#define OTHER_IFC_CTRL2			0xAF
#define OTHER_IFC_CTRL2_SET		0xB0
#define OTHER_IFC_CTRL2_CLR		0xB1
#define OTHER_IFC_CTRL2_ULPI_STP_LOW	(1 << 4)
#define OTHER_IFC_CTRL2_ULPI_TXEN_POL	(1 << 3)
#define OTHER_IFC_CTRL2_ULPI_4PIN_2430	(1 << 2)
#define OTHER_IFC_CTRL2_USB_INT_OUTSEL_MASK	(3 << 0) /* bits 0 and 1 */
#define OTHER_IFC_CTRL2_USB_INT_OUTSEL_INT1N	(0 << 0)
#define OTHER_IFC_CTRL2_USB_INT_OUTSEL_INT2N	(1 << 0)

#define REG_CTRL_EN			0xB2
#define REG_CTRL_EN_SET			0xB3
#define REG_CTRL_EN_CLR			0xB4
#define REG_CTRL_ERROR			0xB5
#define ULPI_I2C_CONFLICT_INTEN		(1 << 0)

#define OTHER_FUNC_CTRL2		0xB8
#define OTHER_FUNC_CTRL2_SET		0xB9
#define OTHER_FUNC_CTRL2_CLR		0xBA
#define OTHER_FUNC_CTRL2_VBAT_TIMER_EN	(1 << 0)

/* following registers do not have separate _clr and _set registers */
#define VBUS_DEBOUNCE			0xC0
#define ID_DEBOUNCE			0xC1
#define VBAT_TIMER			0xD3
#define PHY_PWR_CTRL			0xFD
#define PHY_PWR_PHYPWD			(1 << 0)
#define PHY_CLK_CTRL			0xFE
#define PHY_CLK_CTRL_CLOCKGATING_EN	(1 << 2)
#define PHY_CLK_CTRL_CLK32K_EN		(1 << 1)
#define REQ_PHY_DPLL_CLK		(1 << 0)
#define PHY_CLK_CTRL_STS		0xFF
#define PHY_DPLL_CLK			(1 << 0)

/* In module TWL4030_MODULE_PM_MASTER */
#define PROTECT_KEY			0x0E
#define PKEY_1				0xFC
#define PKEY_2 				0x96
#define STS_HW_CONDITIONS		0x0F
#define RESERVED_E			0x1B

/* In module TWL4030_MODULE_PM_RECEIVER */
#define VUSB_DEDICATED1			0x7D
#define VUSB_DEDICATED2			0x7E
#define VUSB1V5_DEV_GRP			0x71
#define VUSB1V5_TYPE			0x72
#define VUSB1V5_REMAP			0x73
#define VUSB1V8_DEV_GRP			0x74
#define VUSB1V8_TYPE			0x75
#define VUSB1V8_REMAP			0x76
#define VUSB3V1_DEV_GRP			0x77
#define VUSB3V1_TYPE			0x78
#define VUSB3V1_REMAP			0x79

/* In module TWL4030_MODULE_INTBR */
#define PMBR1				0x0D
#define GPIO_USB_4PIN_ULPI_2430C	(3 << 0)

/* Special TPS65921 registers in ACCESSORY_VINTDIG */
#define USB_DTCT_CTRL			0x02
#define USB_SW_CHRG_CTRL_EN		(1 << 1)
#define USB_HW_CHRG_DET_EN		(1 << 0)
#define USB_DET_STS_MASK		(3 << 2)
#define USB_DET_NO_CHARGER		(0 << 2)
#define USB_DET_100			(1 << 2)
#define USB_DET_500			(2 << 2)
#define USB_DET_UNDEFINED		(3 << 2)
#define USB_SW_CHRG_CTRL		0x03
#define BCIA_CTRL			0x04
#define ACCISR1				0x05
#define USB_CHRG_TYPE_ISR1		(1 << 0)
#define ACCIMR1				0x06
#define USB_CHRG_TYPE_IMR1		(1 << 0)
#define ACCEDR1				0x0A
#define ACCSIHCTRL			0x0B

enum linkstat {
	USB_LINK_UNKNOWN = 0,
	USB_LINK_NONE,
	USB_LINK_VBUS,
	USB_LINK_ID,
	USB_LINK_CHARGE,
};

struct archos_twl4030_usb {
	struct otg_transceiver	otg;
	struct device		*dev;

	/* TWL4030 internal USB regulator supplies */
	struct regulator	*usb1v5;
	struct regulator	*usb1v8;
	struct regulator	*usb3v1;

	/* for vbus reporting with irqs disabled */
	spinlock_t		lock;

	/* pin configuration */
	enum twl4030_usb_mode	usb_mode;

	int			irq;
	u8			linkstat;
	u8			previous_linkstat;
	u8			asleep;
	bool			irq_enabled;
	
	int			accessory_irq;
	struct task_struct 	*poll_usbid_thread;
	int			enable_charge_detect;
	
	int			usb_id_irq;

	struct switch_dev 	usb_switch;
	struct workqueue_struct *workqueue;
	struct work_struct 	work;
	struct completion	charge_detect_done;

	struct gpio_vbus_mach_info *vbus_info;

	struct regulator       *vbus_draw;	
	bool 			vbus_enabled;
};

/* internal define on top of container_of */
#define xceiv_to_twl(x)		container_of((x), struct archos_twl4030_usb, otg);

/*-------------------------------------------------------------------------*/

static int twl4030_i2c_write_u8_verify(struct archos_twl4030_usb *twl,
		u8 module, u8 data, u8 address)
{
	u8 check;

	if ((twl_i2c_write_u8(module, data, address) >= 0) &&
	    (twl_i2c_read_u8(module, &check, address) >= 0) &&
						(check == data))
		return 0;
	dev_dbg(twl->dev, "Write%d[%d,0x%x] wrote %02x but read %02x\n",
			1, module, address, check, data);

	/* Failed once: Try again */
	if ((twl_i2c_write_u8(module, data, address) >= 0) &&
	    (twl_i2c_read_u8(module, &check, address) >= 0) &&
						(check == data))
		return 0;
	dev_dbg(twl->dev, "Write%d[%d,0x%x] wrote %02x but read %02x\n",
			2, module, address, check, data);

	/* Failed again: Return error */
	return -EBUSY;
}

#define twl4030_usb_write_verify(twl, address, data)	 \
	twl4030_i2c_write_u8_verify(twl, TWL4030_MODULE_USB, (data), (address))

static inline int twl4030_usb_write(struct archos_twl4030_usb *twl,
		u8 address, u8 data)
{
	int ret = 0;

	ret = twl_i2c_write_u8(TWL4030_MODULE_USB, data, address);
	if (ret < 0)
		dev_dbg(twl->dev,
			"TWL4030:USB:Write[0x%x] Error %d\n", address, ret);
	return ret;
}

static inline int twl4030_readb(struct archos_twl4030_usb *twl, u8 module, u8 address)
{
	u8 data;
	int ret = 0;

	ret = twl_i2c_read_u8(module, &data, address);
	if (ret >= 0)
		ret = data;
	else
		dev_dbg(twl->dev,
			"TWL4030:readb[0x%x,0x%x] Error %d\n",
					module, address, ret);

	return ret;
}

static inline int twl4030_usb_read(struct archos_twl4030_usb *twl, u8 address)
{
	return twl4030_readb(twl, TWL4030_MODULE_USB, address);
}

/*-------------------------------------------------------------------------*/

static inline int
twl4030_usb_set_bits(struct archos_twl4030_usb *twl, u8 reg, u8 bits)
{
	return twl4030_usb_write(twl, reg + 1, bits);
}

static inline int
twl4030_usb_clear_bits(struct archos_twl4030_usb *twl, u8 reg, u8 bits)
{
	return twl4030_usb_write(twl, reg + 2, bits);
}

/*-------------------------------------------------------------------------*/

static enum linkstat archos_twl4030_usb_linkstat(struct archos_twl4030_usb *twl)
{
	int	status;
	int	linkstat = USB_LINK_UNKNOWN;

	/*
	 * For ID/VBUS sensing, see manual section 15.4.8 ...
	 * except when using only battery backup power, two
	 * comparators produce VBUS_PRES and ID_PRES signals,
	 * which don't match docs elsewhere.  But ... BIT(7)
	 * and BIT(2) of STS_HW_CONDITIONS, respectively, do
	 * seem to match up.  If either is true the USB_PRES
	 * signal is active, the OTG module is activated, and
	 * its interrupt may be raised (may wake the system).
	 */
	status = twl4030_readb(twl, TWL4030_MODULE_PM_MASTER,
			STS_HW_CONDITIONS);
	if (status < 0)
		dev_err(twl->dev, "USB link status err %d\n", status);
	else if (status & (BIT(7) | BIT(2))) {
		if (status & BIT(2))
			linkstat = USB_LINK_ID;
		else
			linkstat = USB_LINK_VBUS;
	} else
		linkstat = USB_LINK_NONE;

	/*
	dev_dbg(twl->dev, "HW_CONDITIONS 0x%02x/%d; link %d\n",
			status, status, linkstat);
	*/

	return linkstat;
}

static void archos_twl4030_usb_set_mode(struct archos_twl4030_usb *twl, int mode)
{
	twl->usb_mode = mode;

	switch (mode) {
	case T2_USB_MODE_ULPI:
		twl4030_usb_clear_bits(twl, ULPI_IFC_CTRL,
					ULPI_IFC_CTRL_CARKITMODE);
		twl4030_usb_set_bits(twl, POWER_CTRL, POWER_CTRL_OTG_ENAB);
		twl4030_usb_clear_bits(twl, ULPI_FUNC_CTRL,
					ULPI_FUNC_CTRL_XCVRSEL_MASK |
					ULPI_FUNC_CTRL_OPMODE_MASK);
		break;
	case -1:
		/* FIXME: power on defaults */
		break;
	default:
		dev_err(twl->dev, "unsupported T2 transceiver mode %d\n",
				mode);
		break;
	};
}

static void archos_twl4030_i2c_access(struct archos_twl4030_usb *twl, int on)
{
	unsigned long timeout;
	int val = twl4030_usb_read(twl, PHY_CLK_CTRL);

	if (val >= 0) {
		if (on) {
			/* enable DPLL to access PHY registers over I2C */
			val |= REQ_PHY_DPLL_CLK;
			WARN_ON(twl4030_usb_write_verify(twl, PHY_CLK_CTRL,
						(u8)val) < 0);

			timeout = jiffies + HZ;
			while (!(twl4030_usb_read(twl, PHY_CLK_CTRL_STS) &
							PHY_DPLL_CLK)
				&& time_before(jiffies, timeout))
					udelay(10);
			if (!(twl4030_usb_read(twl, PHY_CLK_CTRL_STS) &
							PHY_DPLL_CLK))
				dev_err(twl->dev, "Timeout setting T2 HSUSB "
						"PHY DPLL clock\n");
		} else {
			/* let ULPI control the DPLL clock */
			val &= ~REQ_PHY_DPLL_CLK;
			WARN_ON(twl4030_usb_write_verify(twl, PHY_CLK_CTRL,
						(u8)val) < 0);
		}
	}
}

static void archos_twl4030_usb_power(struct archos_twl4030_usb *twl, int on)
{
	if (on) {
		regulator_enable(twl->usb3v1);
		regulator_enable(twl->usb1v8);
		/*
		 * Disabling usb3v1 regulator (= writing 0 to VUSB3V1_DEV_GRP
		 * in twl4030) resets the VUSB_DEDICATED2 register. This reset
		 * enables VUSB3V1_SLEEP bit that remaps usb3v1 ACTIVE state to
		 * SLEEP. We work around this by clearing the bit after usv3v1
		 * is re-activated. This ensures that VUSB3V1 is really active.
		 */
		twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0,
							VUSB_DEDICATED2);
		regulator_enable(twl->usb1v5);
	} else {
		regulator_disable(twl->usb1v5);
		regulator_disable(twl->usb1v8);
		regulator_disable(twl->usb3v1);
	}
}

static void archos_twl4030_phy_power(struct archos_twl4030_usb *twl, int on)
{
	u8 pwr;
	pwr = twl4030_usb_read(twl, PHY_PWR_CTRL);
	if (on) {
		archos_twl4030_usb_power(twl, 1);
		pwr &= ~PHY_PWR_PHYPWD;
		WARN_ON(twl4030_usb_write_verify(twl, PHY_PWR_CTRL, pwr) < 0);
		twl4030_usb_write(twl, PHY_CLK_CTRL,
				  twl4030_usb_read(twl, PHY_CLK_CTRL) |
					(PHY_CLK_CTRL_CLOCKGATING_EN |
						PHY_CLK_CTRL_CLK32K_EN));
	} else  {
		pwr |= PHY_PWR_PHYPWD;
		WARN_ON(twl4030_usb_write_verify(twl, PHY_PWR_CTRL, pwr) < 0);
		archos_twl4030_usb_power(twl, 0);
	}
}

static void archos_twl4030_phy_suspend(struct archos_twl4030_usb *twl, int controller_off)
{
	if (twl->asleep)
		return;

	archos_twl4030_phy_power(twl, 0);
	twl->asleep = 1;
}

static void archos_twl4030_phy_resume(struct archos_twl4030_usb *twl)
{
	if (!twl->asleep)
		return;

	archos_twl4030_phy_power(twl, 1);
	archos_twl4030_i2c_access(twl, 1);
	archos_twl4030_usb_set_mode(twl, twl->usb_mode);
	if (twl->usb_mode == T2_USB_MODE_ULPI)
		archos_twl4030_i2c_access(twl, 0);
	twl->asleep = 0;

#if 0
	spin_lock(&twl->otg.lock);
	if (twl->otg.link_restore_context)
		twl->otg.link_restore_context(&twl->otg);
	spin_unlock(&twl->otg.lock);
#endif
}

static void archos_twl4030_link_force_active(struct archos_twl4030_usb *twl, int active)
{
#if 0
	spin_lock(&twl->otg.lock);
	if (twl->otg.link_force_active)
		twl->otg.link_force_active(active);
	spin_unlock(&twl->otg.lock);
#endif
}

static int archos_twl4030_usb_ldo_init(struct archos_twl4030_usb *twl)
{
	/* Enable writing to power configuration registers */
	twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, PKEY_1, PROTECT_KEY);
	twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, PKEY_2, PROTECT_KEY);

	/* put VUSB3V1 LDO in active state */
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB_DEDICATED2);

	/* input to VUSB3V1 LDO is from VBAT, not VBUS */
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x14, VUSB_DEDICATED1);

	/* Initialize 3.1V regulator */
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB3V1_DEV_GRP);

	twl->usb3v1 = regulator_get(twl->dev, "usb3v1");
	if (IS_ERR(twl->usb3v1))
		return -ENODEV;

	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB3V1_TYPE);

	/* Initialize 1.5V regulator */
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB1V5_DEV_GRP);

	twl->usb1v5 = regulator_get(twl->dev, "usb1v5");
	if (IS_ERR(twl->usb1v5))
		goto fail1;

	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB1V5_TYPE);

	/* Initialize 1.8V regulator */
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB1V8_DEV_GRP);

	twl->usb1v8 = regulator_get(twl->dev, "usb1v8");
	if (IS_ERR(twl->usb1v8))
		goto fail2;

	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB1V8_TYPE);

	/* disable access to power configuration registers */
	twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0, PROTECT_KEY);

	return 0;

fail2:
	regulator_put(twl->usb1v5);
	twl->usb1v5 = NULL;
fail1:
	regulator_put(twl->usb3v1);
	twl->usb3v1 = NULL;
	return -ENODEV;
}

static ssize_t archos_twl4030_usb_vbus_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct archos_twl4030_usb *twl = dev_get_drvdata(dev);
	unsigned long flags;
	int ret = -EINVAL;

	spin_lock_irqsave(&twl->lock, flags);
	ret = sprintf(buf, "%s\n",
			(twl->linkstat == USB_LINK_VBUS) ? "on" : "off");
	spin_unlock_irqrestore(&twl->lock, flags);

	return ret;
}
static DEVICE_ATTR(vbus, 0444, archos_twl4030_usb_vbus_show, NULL);

/*
 * switch class handlers
 */
//Signalling other events
#define USB_SW_NAME "USB_SWITCH" 

static ssize_t usb_switch_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", sdev->name);
}

static ssize_t usb_switch_print_state(struct switch_dev *sdev, char *buf)
{
	ssize_t buflen = 0;

	switch (sdev->state) {
	case USB_LINK_UNKNOWN:
	default:
		buflen = sprintf(buf, "UNKNOWN\n");
		break;
	case USB_LINK_VBUS:
		buflen = sprintf(buf, "ATTACHED_VBUS\n");
		break;
	case USB_LINK_CHARGE:
		buflen = sprintf(buf, "ATTACHED_CHARGER\n");
		break;
	case USB_LINK_ID:
		buflen = sprintf(buf, "ATTACHED_HOST\n");
		break;
	case USB_LINK_NONE:
		buflen = sprintf(buf, "DETACHED\n");
		break;
	}
	return buflen;
}

static int archos_twl4030_poll_usbid( void *_twl );

static void archos_twl4030_linkstate_worker(struct work_struct *work)
{
	struct archos_twl4030_usb *twl = container_of(work, struct archos_twl4030_usb, work);
	int linkstat = twl->linkstat;
	
	spin_lock_irq(&twl->lock);
	if (linkstat == USB_LINK_ID) {
		twl->otg.default_a = true;
		twl->otg.state = OTG_STATE_A_IDLE;
	} else {
		twl->otg.default_a = false;
		twl->otg.state = OTG_STATE_B_IDLE;
	}

	spin_unlock_irq(&twl->lock);

	/* stop the polling thread */
	if ( twl->poll_usbid_thread ) {
		kthread_stop(twl->poll_usbid_thread);
	}

	if (linkstat != USB_LINK_UNKNOWN) {

		/* FIXME add a set_power() method so that B-devices can
		 * configure the charger appropriately.  It's not always
		 * correct to consume VBUS power, and how much current to
		 * consume is a function of the USB configuration chosen
		 * by the host.
		 *
		 * REVISIT usb_gadget_vbus_connect(...) as needed, ditto
		 * its disconnect() sibling, when changing to/from the
		 * USB_LINK_VBUS state.  musb_hdrc won't care until it
		 * starts to handle softconnect right.
		 */

		if (linkstat == USB_LINK_NONE) {
#ifdef CONFIG_TPS65921_CHARGE_DETECT
			if ( twl->enable_charge_detect ) {

				archos_twl4030_usb_power(twl, 1);
				/* disable default D+/D- pull-downs */
				twl4030_usb_write( twl, TWL4030_OTG_CTRL_CLR, TWL4030_OTG_CTRL_DMPULLDOWN | TWL4030_OTG_CTRL_DPPULLDOWN);

				/* turn on HW charge detection */
				twl_i2c_write_u8(TPS65921_MODULE_ACCESSORY_VINTDIG, USB_SW_CHRG_CTRL_EN, USB_DTCT_CTRL);
				twl_i2c_write_u8(TPS65921_MODULE_ACCESSORY_VINTDIG, USB_HW_CHRG_DET_EN, USB_DTCT_CTRL);

				archos_twl4030_usb_power(twl, 0);
			}
#endif
			if (twl->previous_linkstat == USB_LINK_VBUS) {
				printk("STATE_VBUS -> STATE_NONE\n");
				blocking_notifier_call_chain(&twl->otg.notifier, USB_EVENT_NONE,
						twl->otg.gadget);
			
			} else if (twl->previous_linkstat == USB_LINK_ID) {
				printk("STATE_GND -> STATE_NONE\n");
				blocking_notifier_call_chain(&twl->otg.notifier, USB_EVENT_NONE,
						twl->otg.gadget);
			}

			/* Hack!!! If the driver is not waiting
			 * 100ms at this point and turns off the phy
			 * then DISCONNECT interrupt will not be
			 * reached to mentor
			 */
			msleep(100);
			archos_twl4030_link_force_active(twl, 0);
			archos_twl4030_phy_suspend(twl, 0);
		} else {
			/* on A101 we only get transitions between USB_LINK_ID and USB_LINK_VBUS,
			 * but we need the phy to be off for a short moment for some reason
			 */
			if( (twl->previous_linkstat == USB_LINK_ID && linkstat == USB_LINK_VBUS) ||
			    (twl->previous_linkstat == USB_LINK_VBUS && linkstat == USB_LINK_ID)) {
				
				archos_twl4030_link_force_active(twl, 0);
				archos_twl4030_phy_suspend(twl, 0);

				msleep(100);
			}

			if ( linkstat == USB_LINK_ID ) {
#ifdef CONFIG_TPS65921_CHARGE_DETECT
				if ( twl->enable_charge_detect ) {
					u8 val;
					/* turn off charge detection for host */
					twl_i2c_read_u8(TPS65921_MODULE_ACCESSORY_VINTDIG, &val, USB_DTCT_CTRL);
					val &= ~USB_HW_CHRG_DET_EN;
					twl_i2c_write_u8(TPS65921_MODULE_ACCESSORY_VINTDIG, val, USB_DTCT_CTRL);
				}
#endif
			}
#ifdef CONFIG_TPS65921_CHARGE_DETECT
			if ( twl->enable_charge_detect && linkstat == USB_LINK_VBUS ) {
				u8 usb_dtc_ctrl;	

				archos_twl4030_usb_power(twl, 1);
				/* disable default D+/D- pull-downs */
				twl4030_usb_write( twl, TWL4030_OTG_CTRL_CLR, TWL4030_OTG_CTRL_DMPULLDOWN | TWL4030_OTG_CTRL_DPPULLDOWN);
				/* clear ISR */
				twl_i2c_write_u8(TPS65921_MODULE_ACCESSORY_VINTDIG, USB_CHRG_TYPE_ISR1, ACCISR1);

				/* enable charge detection IRQ */
				twl_i2c_write_u8(TPS65921_MODULE_ACCESSORY_VINTDIG, 0, ACCIMR1);

				init_completion(&twl->charge_detect_done);
				if (!wait_for_completion_timeout( &twl->charge_detect_done, msecs_to_jiffies(500))) {
					printk("charge_detect_done timeout\n");
				}

				usb_dtc_ctrl = twl4030_readb(twl, TPS65921_MODULE_ACCESSORY_VINTDIG, USB_DTCT_CTRL);

				switch(usb_dtc_ctrl & USB_DET_STS_MASK) {
					case USB_DET_100:
						/* we are connected to a "normal" USB host */

						break;
					case USB_DET_500:
						/* we are connected to a USB charger - OTG etc. does not need to be bothered */
						linkstat = USB_LINK_CHARGE;
						archos_twl4030_usb_power(twl, 0);
						break;
					case USB_DET_NO_CHARGER:
					case USB_DET_UNDEFINED:
					default:
						printk("%x ??!? No charger detected but linkstat==USB_LINK_VBUS", usb_dtc_ctrl);
				}
			}
#endif
			if ( linkstat != USB_LINK_CHARGE ) {
#ifdef CONFIG_TPS65921_CHARGE_DETECT
				if ( twl->enable_charge_detect && linkstat == USB_LINK_VBUS ) {
					/* enable default D+/D- pull-downs again, make sure charge pump is off */
					twl4030_usb_write(twl, TWL4030_OTG_CTRL_CLR, TWL4030_OTG_CTRL_DRVVBUS);
					twl4030_usb_write( twl, TWL4030_OTG_CTRL_SET, (TWL4030_OTG_CTRL_DMPULLDOWN | TWL4030_OTG_CTRL_DPPULLDOWN));

					archos_twl4030_usb_power(twl, 0); /* turn if off, archos_twl4030_usb_power() will turn it back on */
				}
#endif
				archos_twl4030_link_force_active(twl, 1);
				archos_twl4030_phy_resume(twl);

				if ( linkstat == USB_LINK_ID ) {

					/* if we have no separate IRQ for USB_ID, we need to poll for changes, start it here */
					if ( !twl->usb_id_irq ) {
						/* start the polling thread - we need to poll for changes to USB_ID, 
						 * which unfortunately do not generate a USB_PREP irq
						 * if VBUS is turned on by the host
						 */
						struct task_struct *th;
						if ( twl->poll_usbid_thread ) {
							printk("poll thread already active!!\n");
						} else {
							th = kthread_run(archos_twl4030_poll_usbid, twl, "archos_twl4030_usb-pollid");
							if (IS_ERR(th)) {
								dev_err(twl->dev, "Unable to start poll_id thread\n");
							} else
								twl->poll_usbid_thread = th;
						}
					}
					printk("STATE_NONE -> STATE_GND\n");
					blocking_notifier_call_chain(&twl->otg.notifier,
						USB_EVENT_ID, twl->otg.gadget);
				} else {
					printk("STATE_NONE -> STATE_VBUS\n");
					blocking_notifier_call_chain(&twl->otg.notifier,
						USB_EVENT_VBUS, twl->otg.gadget);
				}
			}
		}
#ifndef CONFIG_MACH_ARCHOS
		twl4030charger_usb_en(linkstat == USB_LINK_VBUS);
#endif
	}
	
	sysfs_notify(&twl->dev->kobj, NULL, "vbus");
	switch_set_state(&twl->usb_switch, linkstat);
}

static irqreturn_t archos_twl4030_usb_irq(int irq, void *_twl)
{
	struct archos_twl4030_usb *twl = _twl;
	int linkstat;
	
#ifdef CONFIG_LOCKDEP
	/* WORKAROUND for lockdep forcing IRQF_DISABLED on us, which
	 * we don't want and can't tolerate.  Although it might be
	 * friendlier not to borrow this thread context...
	 */
	local_irq_enable();
#endif

	linkstat = archos_twl4030_usb_linkstat(twl);

	spin_lock_irq(&twl->lock);
	if ( linkstat != twl->linkstat ) {
		twl->previous_linkstat = twl->linkstat;
		twl->linkstat = linkstat;
		queue_work( twl->workqueue, &twl->work );
	}
	spin_unlock_irq(&twl->lock);
	return IRQ_HANDLED;
}
#ifdef CONFIG_TPS65921_CHARGE_DETECT
static irqreturn_t archos_twl4030_accessory_irq(int irq, void *_twl)
{
	struct archos_twl4030_usb *twl = _twl;

	/* clear ISR */
	twl_i2c_write_u8(TPS65921_MODULE_ACCESSORY_VINTDIG, USB_CHRG_TYPE_ISR1, ACCISR1);
	
	/* we don't anything here, we just inform the thread that the charge detection is finished */
	complete( &twl->charge_detect_done );
	return IRQ_HANDLED;
}
#endif

static irqreturn_t archos_twl4030_usb_id_irq(int irq, void *_twl)
{
	struct archos_twl4030_usb *twl = _twl;
	int usb_id = gpio_get_value_cansleep(twl->vbus_info->gpio_id);
	int status;
	int i;

	/* we need to wait a bit, because USB_PREP is using debouncing
	 * and we are not...
	 * Note that we can schedule() here, as this is not a real IRQ
	 */
	for (i=0; i<10; i++) {
		msleep_interruptible( 10 );
		if ( usb_id != gpio_get_value_cansleep(twl->vbus_info->gpio_id)) {
			break;
		}
		status = twl4030_readb(twl, TWL4030_MODULE_PM_MASTER,
				STS_HW_CONDITIONS);
		/* bit2 reflects the USB_ID status of the USB_PREP */
		if (status > 0 && ( (!(status & BIT(2))) == usb_id ))
			break;
	}

	if ( twl->linkstat != USB_LINK_UNKNOWN &&
	    ((!usb_id 
		&& twl->linkstat != USB_LINK_VBUS && twl->linkstat != USB_LINK_NONE ) ||
	     ( usb_id 
	        && twl->linkstat != USB_LINK_ID ) )) {
		/* nothing to do */
		return IRQ_HANDLED;
	}

	spin_lock_irq(&twl->lock);
	twl->previous_linkstat = twl->linkstat;
	twl->linkstat = usb_id ? USB_LINK_VBUS : USB_LINK_ID;
	spin_unlock_irq(&twl->lock);

	queue_work( twl->workqueue, &twl->work );
	return IRQ_HANDLED;
}


/* Poll STS_HW_CONDITIONS for changes to STS_USB, which are masked by STS_VBUS=1 as long as VBUS is generated
 * by our host.
 */

static int archos_twl4030_poll_usbid( void *_twl )
{
	struct archos_twl4030_usb *twl = (struct archos_twl4030_usb*) _twl;
	
	set_freezable();
	while (!kthread_should_stop()) {
		int linkstat;

		msleep_interruptible( 1000 );
		try_to_freeze();
		
		if ( kthread_should_stop() )
			break;
		linkstat = archos_twl4030_usb_linkstat(twl);
		if ( linkstat != twl->linkstat ) {
			twl->poll_usbid_thread = NULL;
			
			spin_lock_irq(&twl->lock);
			twl->previous_linkstat = twl->linkstat;
			/* if we have turned on VBUS ourselves, we assume nothing is connected now */
			if ( twl->previous_linkstat == USB_LINK_ID && linkstat == USB_LINK_VBUS ) {
				spin_unlock_irq(&twl->lock);
				if ( twl4030_usb_read(twl, TWL4030_OTG_CTRL) & TWL4030_OTG_CTRL_DRVVBUS ) {
					/* force charge pump off */
					twl4030_usb_write(twl, TWL4030_OTG_CTRL_CLR, TWL4030_OTG_CTRL_DRVVBUS);
					linkstat = USB_LINK_NONE;
				}
				spin_lock_irq(&twl->lock);
			}
			twl->linkstat = linkstat;
			spin_unlock_irq(&twl->lock);
			queue_work( twl->workqueue, &twl->work );

			return 0;
		}
	}
	twl->poll_usbid_thread = NULL;
	return 0;
}

static int archos_twl4030_set_vbus(struct otg_transceiver *x, bool enabled)
{
	struct regulator *vbus_draw;
	struct archos_twl4030_usb *twl = xceiv_to_twl(x);

	vbus_draw = twl->vbus_draw;
	
	if (!vbus_draw)
		return -1;

	if (enabled) {
		if (!twl->vbus_enabled) {
			regulator_enable(vbus_draw);
			twl->vbus_enabled = true;
		}
	} else {
		if (twl->vbus_enabled) {
			regulator_disable(vbus_draw);
			twl->vbus_enabled = false;
		}
	}

	return 0;
}

static int archos_twl4030_enable_irq(struct otg_transceiver *x)
{
	struct archos_twl4030_usb *twl = xceiv_to_twl(x);
	
	twl->linkstat = USB_LINK_UNKNOWN;
	
	archos_twl4030_usb_irq(twl->irq, twl);
	return 0;
}

static int archos_twl4030_set_suspend(struct otg_transceiver *x, int suspend)
{
	struct archos_twl4030_usb *twl = xceiv_to_twl(x);

	if (suspend)
		archos_twl4030_phy_suspend(twl, 1);
	else
		archos_twl4030_phy_resume(twl);

	return 0;
}

static int archos_twl4030_set_peripheral(struct otg_transceiver *x,
		struct usb_gadget *gadget)
{
	struct archos_twl4030_usb *twl;

	if (!x)
		return -ENODEV;

	twl = xceiv_to_twl(x);
	twl->otg.gadget = gadget;
	if (!gadget)
		twl->otg.state = OTG_STATE_UNDEFINED;

	return 0;
}

static int archos_twl4030_set_host(struct otg_transceiver *x, struct usb_bus *host)
{
	struct archos_twl4030_usb *twl;

	if (!x)
		return -ENODEV;

	twl = xceiv_to_twl(x);
	twl->otg.host = host;
	if (!host)
		twl->otg.state = OTG_STATE_UNDEFINED;

	return 0;
}
#define ARCHOS_USB_XCEIV_IRQ_FLAGS \
	( IRQF_SAMPLE_RANDOM | IRQF_SHARED \
	| IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING )

static int __init archos_twl4030_usb_probe(struct platform_device *pdev)
{
	struct twl4030_usb_data *pdata = pdev->dev.platform_data;
	struct archos_twl4030_usb	*twl;
	int			status, err;

	if (!pdata) {
		dev_dbg(&pdev->dev, "platform_data not available\n");
		return -EINVAL;
	}

	twl = kzalloc(sizeof *twl, GFP_KERNEL);
	if (!twl)
		return -ENOMEM;

	twl->vbus_info = (struct gpio_vbus_mach_info*)pdata->platform;

	twl->dev		= &pdev->dev;
	twl->irq		= platform_get_irq(pdev, 0);
	twl->accessory_irq	= platform_get_irq(pdev, 1);
	twl->otg.dev		= twl->dev;
	twl->otg.label		= "twl4030";
	twl->otg.set_host	= archos_twl4030_set_host;
	twl->otg.set_peripheral	= archos_twl4030_set_peripheral;
	twl->otg.set_suspend	= archos_twl4030_set_suspend;
	twl->otg.enable_irq	= archos_twl4030_enable_irq;
	twl->usb_mode 		= pdata->usb_mode;
	twl->asleep		= 1;
	twl->otg.set_vbus	= archos_twl4030_set_vbus;

	twl->enable_charge_detect = (pdata->caps & TWL_USB_CAPS_CHARGING) ? 1:0;

	/* init spinlock for workqueue */
	spin_lock_init(&twl->lock);

	err = archos_twl4030_usb_ldo_init(twl);
	if (err) {
		dev_err(&pdev->dev, "ldo init failed\n");
		kfree(twl);
		return err;
	}
	
	BLOCKING_INIT_NOTIFIER_HEAD(&twl->otg.notifier);

#if 0
	spin_lock_init(&twl->otg.lock);
#endif
	otg_set_transceiver(&twl->otg);

	twl->vbus_draw = regulator_get(&pdev->dev, "vbus_musb");
	if (IS_ERR(twl->vbus_draw)) {
		dev_err(&pdev->dev, "can't get vbus_draw regulator, err: %ld\n",
			PTR_ERR(twl->vbus_draw));
		twl->vbus_draw = NULL;
		//goto fail;
	}

	/* Create the workqueue */
	twl->workqueue = create_singlethread_workqueue("archos_twl4030_usb");
	if (!twl->workqueue) {
		err = -ENOMEM;
		goto fail;
	}

	/* initialize switches */
	twl->usb_switch.name = USB_SW_NAME;
	twl->usb_switch.print_name = usb_switch_print_name;
	twl->usb_switch.print_state = usb_switch_print_state;
	if (switch_dev_register(&twl->usb_switch) < 0)
		dev_info(&pdev->dev, "Error creating USB switch\n");

	platform_set_drvdata(pdev, twl);
	if (device_create_file(&pdev->dev, &dev_attr_vbus))
		dev_warn(&pdev->dev, "could not create sysfs file\n");

	archos_twl4030_usb_power(twl, 1);
#ifndef CONFIG_MACH_ARCHOS
	/*
	 * One time configuration to route MCPC pins to the MADC for
	 * monitoring */
	twl4030_usb_write(twl, CARKIT_ANA_CTRL,
	twl4030_usb_read(twl, CARKIT_ANA_CTRL) | SEL_MADC_MCPC);
#else
	/* Enable writing to power configuration registers */
	twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, KEY_1, PROTECT_KEY);
	twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, KEY_2, PROTECT_KEY);
	
	/* set VBUS debounce time to maximum - 233ms */
	twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0x7, RESERVED_E);

	/* Disable write access to power configuration registers again */
	twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0, PROTECT_KEY);	
#endif
#ifdef CONFIG_TPS65921_CHARGE_DETECT
	if ( twl->enable_charge_detect ) {
		/* disable default D+/D- pull-downs - they would interfere with charger detection */
		twl4030_usb_write( twl, TWL4030_OTG_CTRL_CLR, TWL4030_OTG_CTRL_DMPULLDOWN | TWL4030_OTG_CTRL_DPPULLDOWN);

		/* use default value */
		twl_i2c_write_u8(TPS65921_MODULE_ACCESSORY_VINTDIG, 0x01, ACCSIHCTRL);

		/* turn on HW charge detection */
		/* NB: CSR OMAPS00220841 - work-around: first turn on SW detection to reset FSM, then turn on HW detection */
		twl_i2c_write_u8(TPS65921_MODULE_ACCESSORY_VINTDIG, USB_SW_CHRG_CTRL_EN, USB_DTCT_CTRL);
		twl_i2c_write_u8(TPS65921_MODULE_ACCESSORY_VINTDIG, USB_HW_CHRG_DET_EN, USB_DTCT_CTRL);
	} else 
#endif
	{
		/* disable usb hw charge detection */
		twl_i2c_write_u8(TPS65921_MODULE_ACCESSORY_VINTDIG, 0x00, ACCSIHCTRL);
		/* disable charge detection IRQ */
		twl_i2c_write_u8(TPS65921_MODULE_ACCESSORY_VINTDIG, 1, ACCIMR1);
		
		/* enable default D+/D- pull-downs again, make sure charge pump is off */
		twl4030_usb_write(twl, TWL4030_OTG_CTRL_CLR, TWL4030_OTG_CTRL_DRVVBUS);
		twl4030_usb_write( twl, TWL4030_OTG_CTRL_SET, (TWL4030_OTG_CTRL_DMPULLDOWN | TWL4030_OTG_CTRL_DPPULLDOWN));
	}
	
	archos_twl4030_usb_power(twl, 0);
	
	INIT_WORK(&twl->work, archos_twl4030_linkstate_worker);

	/* Our job is to use irqs and status from the power module
	 * to keep the transceiver disabled when nothing's connected.
	 *
	 * FIXME we actually shouldn't start enabling it until the
	 * USB controller drivers have said they're ready, by calling
	 * set_host() and/or set_peripheral() ... OTG_capable boards
	 * need both handles, otherwise just one suffices.
	 */
	twl->irq_enabled = true;
	status = request_irq(twl->irq, archos_twl4030_usb_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			"archos_twl4030_usb", twl);
	if (status < 0) {
		dev_dbg(&pdev->dev, "can't get IRQ %d, err %d\n",
			twl->irq, status);
		err = status;
		goto fail;
	}

#ifdef CONFIG_TPS65921_CHARGE_DETECT
	if ( twl->enable_charge_detect ) {
		init_completion(&twl->charge_detect_done);
		status = request_irq(twl->accessory_irq, archos_twl4030_accessory_irq,
				0, "tps65921_accessory", twl);
		if (status < 0) {
			dev_dbg(&pdev->dev, "can't get IRQ %d, err %d\n",
				twl->accessory_irq, status);
			err = status;
			free_irq(twl->irq, twl);
			goto fail;
		}
	}
#endif

	if (twl->vbus_info && gpio_is_valid(twl->vbus_info->gpio_id)) {
		err = gpio_request(twl->vbus_info->gpio_id, "usb_id");
		if (err) {
			dev_err(&pdev->dev, "can't request id gpio %d, err: %d\n",
					twl->vbus_info->gpio_id, err);
				goto fail;
		}
		gpio_direction_input(twl->vbus_info->gpio_id);

		twl->usb_id_irq = gpio_to_irq(twl->vbus_info->gpio_id);
		err = request_threaded_irq(twl->usb_id_irq, NULL, archos_twl4030_usb_id_irq,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"usb_id", twl);
		if (err) {
			dev_err(&pdev->dev, "can't request irq %i, err: %d\n",
				twl->usb_id_irq, err);
			gpio_free(twl->vbus_info->gpio_id);
			twl->usb_id_irq = 0;
			goto fail;
		}
		
	}

	/* The IRQ handler just handles changes from the previous states
	 * of the ID and VBUS pins ... in probe() we must initialize that
	 * previous state.  The easy way:  fake an IRQ.
	 *
	 * REVISIT:  a real IRQ might have happened already, if PREEMPT is
	 * enabled.  Else the IRQ may not yet be configured or enabled,
	 * because of scheduling delays.
	 */
	archos_twl4030_usb_irq(twl->irq, twl);

	dev_info(&pdev->dev, "Initialized Archos TWL4030 USB tranceiver module\n");
	return 0;

fail:
	if (twl->vbus_draw)
		regulator_put(twl->vbus_draw);
	platform_set_drvdata(pdev, NULL);
	switch_dev_unregister(&twl->usb_switch);
	kfree(twl);
	return err;
}

static int __exit archos_twl4030_usb_remove(struct platform_device *pdev)
{
	struct archos_twl4030_usb *twl = platform_get_drvdata(pdev);
	int val;

	free_irq(twl->irq, twl);
#ifdef CONFIG_TPS65921_CHARGE_DETECT
	if (twl->accessory_irq)
		free_irq(twl->accessory_irq, twl);
#endif
	if (twl->vbus_info && gpio_is_valid(twl->vbus_info->gpio_id)) {
		free_irq(gpio_to_irq(twl->vbus_info->gpio_id), twl);
		gpio_free(twl->vbus_info->gpio_id);
	}
	device_remove_file(twl->dev, &dev_attr_vbus);

	/* set transceiver mode to power on defaults */
	archos_twl4030_usb_set_mode(twl, -1);

	/* autogate 60MHz ULPI clock,
	 * clear dpll clock request for i2c access,
	 * disable 32KHz
	 */
	val = twl4030_usb_read(twl, PHY_CLK_CTRL);
	if (val >= 0) {
		val |= PHY_CLK_CTRL_CLOCKGATING_EN;
		val &= ~(PHY_CLK_CTRL_CLK32K_EN | REQ_PHY_DPLL_CLK);
		twl4030_usb_write(twl, PHY_CLK_CTRL, (u8)val);
	}

	/* stop the polling thread */
	if ( twl->poll_usbid_thread )
		kthread_stop(twl->poll_usbid_thread);
	
	/* disable complete OTG block */
	twl4030_usb_clear_bits(twl, POWER_CTRL, POWER_CTRL_OTG_ENAB);

	archos_twl4030_phy_power(twl, 0);
	regulator_put(twl->usb1v5);
	regulator_put(twl->usb1v8);
	regulator_put(twl->usb3v1);
	regulator_put(twl->vbus_draw);
	
	switch_dev_unregister(&twl->usb_switch);

	kfree(twl);

	return 0;
}

static struct platform_driver archos_twl4030_usb_driver = {
	.probe		= archos_twl4030_usb_probe,
	.remove		= __exit_p(archos_twl4030_usb_remove),
	.driver		= {
		.name	= "archos_twl4030_usb_xceiv",
		.owner	= THIS_MODULE,
	},
};

static int __init archos_twl4030_usb_init(void)
{
	return platform_driver_register(&archos_twl4030_usb_driver);
}
subsys_initcall(archos_twl4030_usb_init);

static void __exit archos_twl4030_usb_exit(void)
{
	platform_driver_unregister(&archos_twl4030_usb_driver);
}
module_exit(archos_twl4030_usb_exit);

MODULE_ALIAS("platform:archos_twl4030_usb_xceiv");
MODULE_AUTHOR("Archos S.A.");
MODULE_DESCRIPTION("Archos specific USB transceiver driver");
MODULE_LICENSE("GPL");
