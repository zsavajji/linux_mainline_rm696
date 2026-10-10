// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * twl4030_usb - TWL4030 USB transceiver, talking to OMAP OTG controller
 *
 * Copyright (C) 2004-2007 Texas Instruments
 * Copyright (C) 2008 Nokia Corporation
 * Contact: Felipe Balbi <felipe.balbi@nokia.com>
 *
 * Current status:
 *	- HS USB ULPI mode works.
 *	- 3-pin mode support may be added in future.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/usb/otg.h>
#include <linux/phy/phy.h>
#include <linux/pm_runtime.h>
#include <linux/usb/musb.h>
#include <linux/usb/ulpi.h>
#include <linux/mfd/twl.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/slab.h>

/* Register defines */

#define MCPC_CTRL			0x30
#define MCPC_CTRL_RTSOL			(1 << 7)
#define MCPC_CTRL_EXTSWR		(1 << 6)
#define MCPC_CTRL_EXTSWC		(1 << 5)
#define MCPC_CTRL_VOICESW		(1 << 4)
#define MCPC_CTRL_OUT64K		(1 << 3)
#define MCPC_CTRL_RTSCTSSW		(1 << 2)
#define MCPC_CTRL_HS_UART		(1 << 0)

#define MCPC_IO_CTRL			0x33
#define MCPC_IO_CTRL_MICBIASEN		(1 << 5)
#define MCPC_IO_CTRL_CTS_NPU		(1 << 4)
#define MCPC_IO_CTRL_RXD_PU		(1 << 3)
#define MCPC_IO_CTRL_TXDTYP		(1 << 2)
#define MCPC_IO_CTRL_CTSTYP		(1 << 1)
#define MCPC_IO_CTRL_RTSTYP		(1 << 0)

#define MCPC_CTRL2			0x36
#define MCPC_CTRL2_MCPC_CK_EN		(1 << 0)

#define OTHER_FUNC_CTRL			0x80
#define OTHER_FUNC_CTRL_BDIS_ACON_EN	(1 << 4)
#define OTHER_FUNC_CTRL_FIVEWIRE_MODE	(1 << 2)

#define OTHER_IFC_CTRL			0x83
#define OTHER_IFC_CTRL_OE_INT_EN	(1 << 6)
#define OTHER_IFC_CTRL_CEA2011_MODE	(1 << 5)
#define OTHER_IFC_CTRL_FSLSSERIALMODE_4PIN	(1 << 4)
#define OTHER_IFC_CTRL_HIZ_ULPI_60MHZ_OUT	(1 << 3)
#define OTHER_IFC_CTRL_HIZ_ULPI		(1 << 2)
#define OTHER_IFC_CTRL_ALT_INT_REROUTE	(1 << 0)

#define OTHER_INT_EN_RISE		0x86
#define OTHER_INT_EN_FALL		0x89
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
#define POWER_CTRL_OTG_ENAB		(1 << 5)

#define OTHER_IFC_CTRL2			0xAF
#define OTHER_IFC_CTRL2_ULPI_STP_LOW	(1 << 4)
#define OTHER_IFC_CTRL2_ULPI_TXEN_POL	(1 << 3)
#define OTHER_IFC_CTRL2_ULPI_4PIN_2430	(1 << 2)
#define OTHER_IFC_CTRL2_USB_INT_OUTSEL_MASK	(3 << 0) /* bits 0 and 1 */
#define OTHER_IFC_CTRL2_USB_INT_OUTSEL_INT1N	(0 << 0)
#define OTHER_IFC_CTRL2_USB_INT_OUTSEL_INT2N	(1 << 0)

#define REG_CTRL_EN			0xB2
#define REG_CTRL_ERROR			0xB5
#define ULPI_I2C_CONFLICT_INTEN		(1 << 0)

#define OTHER_FUNC_CTRL2		0xB8
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

/* In module TWL_MODULE_PM_MASTER */
#define STS_HW_CONDITIONS		0x0F

/* In module TWL_MODULE_PM_RECEIVER */
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

/* In module TWL5031_MODULE_BCC, after Nokia's twl5031-bcc.c (N9 2.6.32) */
#define TWL5031_BCC_CTRL2		0x01
#define TWL5031_CHGMODE_SW		BIT(0)
#define TWL5031_SW_USBCHRG_EN		BIT(1)
#define TWL5031_SW_CHRG_DET		BIT(3)
#define TWL5031_SW_USB_DET_EN		BIT(4)
#define TWL5031_BCC_STS			0x02
#define TWL5031_USB_DET_STS_MASK	(BIT(1) | BIT(0))
#define TWL5031_USB_DET_STS_100MA	1
#define TWL5031_USB_DET_STS_500MA	2
#define TWL5031_USB_P_STS		BIT(2)
#define TWL5031_BATTERY_PRESENCE_STS	BIT(4)
#define TWL5031_BCC_STS_RESET		0xaa	/* no VBUS, no charger: the BCC in reset */
#define TWL5031_USB_CHRG_CTRL1		0x03
#define TWL5031_USB_SW_CTRL_EN		BIT(0)
#define TWL5031_CHGD_IDX_SRC_EN_LOWV	BIT(1)
#define TWL5031_CHGD_SERX_EN_LOWV	BIT(3)
#define TWL5031_CHGD_SERX_DP_LOWV	BIT(5)

static irqreturn_t twl4030_usb_irq(int irq, void *_twl);
/*
 * If VBUS is valid or ID is ground, then we know a
 * cable is present and we need to be runtime-enabled
 */
static inline bool cable_present(enum musb_vbus_id_status stat)
{
	return stat == MUSB_VBUS_VALID ||
		stat == MUSB_ID_GROUND;
}

struct twl4030_usb {
	struct usb_phy		phy;
	struct device		*dev;

	/* TWL4030 internal USB regulator supplies */
	struct regulator	*usb1v5;
	struct regulator	*usb1v8;
	struct regulator	*usb3v1;

	/* for vbus reporting with irqs disabled */
	struct mutex		lock;

	/* pin configuration */
	enum twl4030_usb_mode	usb_mode;

	int			irq;
	enum musb_vbus_id_status linkstat;
	atomic_t		connected;
	bool			vbus_supplied;
	bool			musb_mailbox_pending;
	unsigned long		runtime_suspended:1;
	unsigned long		needs_resume:1;
	bool			powered;	/* supplies on, PHY powered up */
	bool			dcd;		/* the last charger detection saw data contact */
	bool			musb_cable;	/* musb was told of this cable */

	struct delayed_work	id_workaround_work;
	struct work_struct	charger_work;
};

/* internal define on top of container_of */
#define phy_to_twl(x)		container_of((x), struct twl4030_usb, phy)

/*-------------------------------------------------------------------------*/

static int twl4030_i2c_write_u8_verify(struct twl4030_usb *twl,
		u8 module, u8 data, u8 address)
{
	u8 check = 0xFF;

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

#define twl4030_usb_write_verify(twl, address, data)	\
	twl4030_i2c_write_u8_verify(twl, TWL_MODULE_USB, (data), (address))

static inline int twl4030_usb_write(struct twl4030_usb *twl,
		u8 address, u8 data)
{
	int ret = 0;

	ret = twl_i2c_write_u8(TWL_MODULE_USB, data, address);
	if (ret < 0)
		dev_dbg(twl->dev,
			"TWL4030:USB:Write[0x%x] Error %d\n", address, ret);
	return ret;
}

static inline int twl4030_readb(struct twl4030_usb *twl, u8 module, u8 address)
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

static inline int twl4030_usb_read(struct twl4030_usb *twl, u8 address)
{
	return twl4030_readb(twl, TWL_MODULE_USB, address);
}

/*-------------------------------------------------------------------------*/

static inline int
twl4030_usb_set_bits(struct twl4030_usb *twl, u8 reg, u8 bits)
{
	return twl4030_usb_write(twl, ULPI_SET(reg), bits);
}

static inline int
twl4030_usb_clear_bits(struct twl4030_usb *twl, u8 reg, u8 bits)
{
	return twl4030_usb_write(twl, ULPI_CLR(reg), bits);
}

/*-------------------------------------------------------------------------*/

static bool twl4030_is_driving_vbus(struct twl4030_usb *twl)
{
	int ret;

	ret = twl4030_usb_read(twl, PHY_CLK_CTRL_STS);
	if (ret < 0 || !(ret & PHY_DPLL_CLK))
		/*
		 * if clocks are off, registers are not updated,
		 * but we can assume we don't drive VBUS in this case
		 */
		return false;

	ret = twl4030_usb_read(twl, ULPI_OTG_CTRL);
	if (ret < 0)
		return false;

	return (ret & (ULPI_OTG_DRVVBUS | ULPI_OTG_CHRGVBUS)) ? true : false;
}

static enum musb_vbus_id_status
	twl4030_usb_linkstat(struct twl4030_usb *twl)
{
	int	status;
	enum musb_vbus_id_status linkstat = MUSB_UNKNOWN;

	twl->vbus_supplied = false;

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
	status = twl4030_readb(twl, TWL_MODULE_PM_MASTER, STS_HW_CONDITIONS);
	if (status < 0)
		dev_err(twl->dev, "USB link status err %d\n", status);
	else if (status & (BIT(7) | BIT(2))) {
		if (status & BIT(7)) {
			if (twl4030_is_driving_vbus(twl))
				status &= ~BIT(7);
			else
				twl->vbus_supplied = true;
		}

		if (status & BIT(2))
			linkstat = MUSB_ID_GROUND;
		else if (status & BIT(7))
			linkstat = MUSB_VBUS_VALID;
		else
			linkstat = MUSB_VBUS_OFF;
	} else {
		if (twl->linkstat != MUSB_UNKNOWN)
			linkstat = MUSB_VBUS_OFF;
	}

	kobject_uevent(&twl->dev->kobj, linkstat == MUSB_VBUS_VALID
					? KOBJ_ONLINE : KOBJ_OFFLINE);

	dev_dbg(twl->dev, "HW_CONDITIONS 0x%02x/%d; link %d\n",
			status, status, linkstat);

	/* REVISIT this assumes host and peripheral controllers
	 * are registered, and that both are active...
	 */

	return linkstat;
}

static void twl4030_usb_set_mode(struct twl4030_usb *twl, int mode)
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
	}
}

static void twl4030_i2c_access(struct twl4030_usb *twl, int on)
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

static void __twl4030_phy_power(struct twl4030_usb *twl, int on)
{
	u8 pwr = twl4030_usb_read(twl, PHY_PWR_CTRL);

	if (on)
		pwr &= ~PHY_PWR_PHYPWD;
	else
		pwr |= PHY_PWR_PHYPWD;

	WARN_ON(twl4030_usb_write_verify(twl, PHY_PWR_CTRL, pwr) < 0);
}

static int twl4030_usb_runtime_suspend(struct device *dev);
static int twl4030_usb_runtime_resume(struct device *dev);

static int __maybe_unused twl4030_usb_suspend(struct device *dev)
{
	struct twl4030_usb *twl = dev_get_drvdata(dev);

	/*
	 * we need enabled runtime on resume,
	 * so turn irq off here, so we do not get it early
	 * note: wakeup on usb plug works independently of this
	 */
	dev_dbg(twl->dev, "%s\n", __func__);
	disable_irq(twl->irq);
	if (!twl->runtime_suspended && !atomic_read(&twl->connected)) {
		twl4030_usb_runtime_suspend(dev);
		twl->needs_resume = 1;
	}

	return 0;
}

static int __maybe_unused twl4030_usb_resume(struct device *dev)
{
	struct twl4030_usb *twl = dev_get_drvdata(dev);

	dev_dbg(twl->dev, "%s\n", __func__);
	enable_irq(twl->irq);
	if (twl->needs_resume)
		twl4030_usb_runtime_resume(dev);
	/* check whether cable status changed */
	twl4030_usb_irq(0, twl);

	twl->runtime_suspended = 0;

	return 0;
}

static int __maybe_unused twl4030_usb_runtime_suspend(struct device *dev)
{
	struct twl4030_usb *twl = dev_get_drvdata(dev);

	dev_dbg(twl->dev, "%s\n", __func__);

	twl->runtime_suspended = 1;
	/* already off: charger detection powers the PHY down while the cable stays */
	if (!twl->powered)
		return 0;
	twl->powered = false;

	__twl4030_phy_power(twl, 0);
	regulator_disable(twl->usb1v5);
	regulator_disable(twl->usb1v8);
	regulator_disable(twl->usb3v1);

	return 0;
}

static int __maybe_unused twl4030_usb_runtime_resume(struct device *dev)
{
	struct twl4030_usb *twl = dev_get_drvdata(dev);
	int res;

	dev_dbg(twl->dev, "%s\n", __func__);

	if (twl->powered)
		return 0;
	twl->powered = true;

	res = regulator_enable(twl->usb3v1);
	if (res)
		dev_err(twl->dev, "Failed to enable usb3v1\n");

	res = regulator_enable(twl->usb1v8);
	if (res)
		dev_err(twl->dev, "Failed to enable usb1v8\n");

	/*
	 * Disabling usb3v1 regulator (= writing 0 to VUSB3V1_DEV_GRP
	 * in twl4030) resets the VUSB_DEDICATED2 register. This reset
	 * enables VUSB3V1_SLEEP bit that remaps usb3v1 ACTIVE state to
	 * SLEEP. We work around this by clearing the bit after usv3v1
	 * is re-activated. This ensures that VUSB3V1 is really active.
	 */
	twl_i2c_write_u8(TWL_MODULE_PM_RECEIVER, 0, VUSB_DEDICATED2);

	res = regulator_enable(twl->usb1v5);
	if (res)
		dev_err(twl->dev, "Failed to enable usb1v5\n");

	__twl4030_phy_power(twl, 1);
	twl4030_usb_write(twl, PHY_CLK_CTRL,
			  twl4030_usb_read(twl, PHY_CLK_CTRL) |
			  (PHY_CLK_CTRL_CLOCKGATING_EN |
			   PHY_CLK_CTRL_CLK32K_EN));

	twl4030_i2c_access(twl, 1);
	twl4030_usb_set_mode(twl, twl->usb_mode);
	if (twl->usb_mode == T2_USB_MODE_ULPI)
		twl4030_i2c_access(twl, 0);
	/*
	 * According to the TPS65950 TRM, there has to be at least 50ms
	 * delay between setting POWER_CTRL_OTG_ENAB and enabling charging
	 * so wait here so that a fully enabled phy can be expected after
	 * resume
	 */
	msleep(50);
	return 0;
}

/*
 * Nokia's charger detection on the TWL5031, from twl5031-bcc.c in the N9's 2.6.32 kernel (its
 * twl4030-core flagged the USB PHY can_detect_charger on a TWL5031): on VBUS, with the PHY powered
 * down, data contact detection (DCD), then the BCC's detection state machine, then for a charging
 * port D+'s line state with its pull-up on. Only then may the gadget connect (stock's
 * USB_EVENT_TRY_CONN), and only after DCD and not to a dedicated charger. The result is the
 * usb_phy's charger type and state, for the "usb" power supply BME reads.
 */
static int twl5031_bcc_read(u8 reg)
{
	u8 val;
	int ret = twl_i2c_read_u8(TWL5031_MODULE_BCC, &val, reg);

	return ret ? ret : val;
}

static int twl5031_bcc_write(u8 reg, u8 val)
{
	return twl_i2c_write_u8(TWL5031_MODULE_BCC, val, reg);
}

/* DCD: D+ pulled low against a small current source once the data pins touch */
static bool twl5031_dcd(struct twl4030_usb *twl)
{
	const u8 dcd = TWL5031_USB_SW_CTRL_EN | TWL5031_CHGD_IDX_SRC_EN_LOWV |
		       TWL5031_CHGD_SERX_EN_LOWV;
	unsigned long timeout;
	bool online = false;
	int res;

	/* DCD init: enable DP source and DM pulldown */
	res = twl5031_bcc_read(TWL5031_USB_CHRG_CTRL1);
	if (res < 0 || twl5031_bcc_write(TWL5031_USB_CHRG_CTRL1, res | dcd) < 0)
		return false;

	/* Nokia: "twl5031 specification claims for typical value of 2.72 sec", made 3 s */
	timeout = jiffies + msecs_to_jiffies(3000);
	do {
		/* read comparator output */
		res = twl5031_bcc_read(TWL5031_USB_CHRG_CTRL1);
		if (res < 0)
			break;
		if (!(res & TWL5031_CHGD_SERX_DP_LOWV)) {
			online = true;
			break;
		}
		cpu_relax();
	} while (!time_after(jiffies, timeout));

	/* DCD de-init: disable DP source and DM pulldown */
	res = twl5031_bcc_read(TWL5031_USB_CHRG_CTRL1);
	if (res >= 0)
		twl5031_bcc_write(TWL5031_USB_CHRG_CTRL1, res & ~dcd);

	return online;
}

/* A charging port: with D+ pulled up, a dedicated charger (D+ shorted to D-) reads SE1 */
static enum usb_charger_type twl5031_charging_port(struct twl4030_usb *twl)
{
	int func_ctrl, otg_ctrl, linestate;

	/* The transceiver must be resumed before this operation */
	twl4030_usb_runtime_resume(twl->dev);

	func_ctrl = twl4030_usb_read(twl, ULPI_FUNC_CTRL);
	otg_ctrl = twl4030_usb_read(twl, ULPI_OTG_CTRL);
	if (func_ctrl < 0 || otg_ctrl < 0)
		return CDP_TYPE;

	/* disable pulldowns */
	twl4030_usb_clear_bits(twl, ULPI_OTG_CTRL,
			       ULPI_OTG_CTRL_DM_PULLDOWN | ULPI_OTG_CTRL_DP_PULLDOWN);
	/* full speed */
	twl4030_usb_clear_bits(twl, ULPI_FUNC_CTRL, ULPI_FUNC_CTRL_XCVRSEL_MASK);
	twl4030_usb_set_bits(twl, ULPI_FUNC_CTRL, ULPI_FUNC_CTRL_FULL_SPEED);
	/* enable DP pullup */
	twl4030_usb_set_bits(twl, ULPI_FUNC_CTRL, ULPI_FUNC_CTRL_TERMSELECT);
	udelay(100);

	linestate = twl4030_usb_read(twl, ULPI_DEBUG);

	twl4030_usb_write(twl, ULPI_FUNC_CTRL, func_ctrl);
	twl4030_usb_write(twl, ULPI_OTG_CTRL, otg_ctrl);

	if (linestate < 0 || (linestate & (ULPI_DEBUG_LINESTATE0 | ULPI_DEBUG_LINESTATE1)) !=
			     (ULPI_DEBUG_LINESTATE0 | ULPI_DEBUG_LINESTATE1))
		return CDP_TYPE;

	twl4030_usb_runtime_suspend(twl->dev);
	return DCP_TYPE;
}

/*
 * The usb_phy's charger_detect, from usb_phy_set_charger_state(). The BCC writes are stock's: the
 * DCD sources, the hand-over of the current sources to the state machine (SW_CTRL_EN off), and
 * BCC_CTRL2's software charge mode with USB charging on for a charger found and off for none.
 * No current or voltage limit: BME sets those.
 */
static enum usb_charger_type twl5031_charger_detect(struct usb_phy *phy)
{
	struct twl4030_usb *twl = phy_to_twl(phy);
	enum usb_charger_type type = UNKNOWN_TYPE;
	unsigned long timeout;
	bool usb3v1;
	int res, ctl;

	/* disable all regulators + put the transceiver into nondriving mode */
	twl4030_usb_runtime_suspend(twl->dev);

	/* this particular regulator is required for DCD */
	usb3v1 = !regulator_enable(twl->usb3v1);

	/* perform DCD to secure following charger detection */
	twl->dcd = twl5031_dcd(twl);
	if (!twl->dcd)
		dev_notice(twl->dev, "dcd failed\n");	/* ignore error and force charger detection */

	/* Let USB dedicated FSM to control current sources */
	res = twl5031_bcc_read(TWL5031_USB_CHRG_CTRL1);
	if (res < 0 ||
	    twl5031_bcc_write(TWL5031_USB_CHRG_CTRL1, res & ~TWL5031_USB_SW_CTRL_EN) < 0)
		goto out;

	/* Enable USB FSM (STM mode) */
	ctl = twl5031_bcc_read(TWL5031_BCC_CTRL2);
	if (ctl < 0)
		goto out;
	ctl |= TWL5031_CHGMODE_SW | TWL5031_SW_USB_DET_EN;
	if (twl5031_bcc_write(TWL5031_BCC_CTRL2, ctl) < 0)
		goto out;

	/* Read status: until 500 mA, for 300 ms; once if DCD failed */
	timeout = jiffies + msecs_to_jiffies(300);
	do {
		res = twl5031_bcc_read(TWL5031_BCC_STS);
		if (res < 0 || res == TWL5031_BCC_STS_RESET)
			continue;
		if (!(res & TWL5031_BATTERY_PRESENCE_STS)) {
			dev_err_ratelimited(twl->dev, "no battery present\n");
			continue;
		}
		/* USB charger and battery presence bits should be high */
		if (!(res & TWL5031_USB_P_STS))
			continue;
		res &= TWL5031_USB_DET_STS_MASK;
		if (res == TWL5031_USB_DET_STS_500MA)
			break;
	} while (!time_after(jiffies, timeout) && twl->dcd);

	switch (res) {
	case TWL5031_USB_DET_STS_500MA:
		ctl |= TWL5031_SW_CHRG_DET | TWL5031_SW_USBCHRG_EN;
		type = twl5031_charging_port(twl);
		break;
	case TWL5031_USB_DET_STS_100MA:
		ctl |= TWL5031_SW_USBCHRG_EN;
		type = SDP_TYPE;
		break;
	default:
		ctl &= ~TWL5031_SW_USBCHRG_EN;
		/*
		 * Nothing found: stock left the port a standard downstream one and, after DCD,
		 * connected; its power supply then went present once the host configured the phone
		 */
		if (twl->dcd)
			type = SDP_TYPE;
		break;
	}
	dev_dbg(twl->dev, "charger detection: STS %02x, type %d, dcd %d\n", res, type, twl->dcd);

	ctl &= ~TWL5031_SW_USB_DET_EN;	/* save result, stop FSM */
	twl5031_bcc_write(TWL5031_BCC_CTRL2, ctl);
out:
	if (usb3v1)
		regulator_disable(twl->usb3v1);
	return type;
}

static void twl5031_charger_work(struct work_struct *work)
{
	struct twl4030_usb *twl = container_of(work, struct twl4030_usb, charger_work);

	usb_phy_set_charger_state(&twl->phy, USB_CHARGER_PRESENT);
	if (!twl->dcd || twl->phy.chg_type == DCP_TYPE)
		return;

	/* stock's USB_EVENT_TRY_CONN: the PHY up, musb told of VBUS */
	twl4030_usb_runtime_resume(twl->dev);
	twl->musb_cable = true;
	if (musb_mailbox(MUSB_VBUS_VALID))
		twl->musb_mailbox_pending = true;
}

static int twl4030_phy_power_off(struct phy *phy)
{
	struct twl4030_usb *twl = phy_get_drvdata(phy);

	dev_dbg(twl->dev, "%s\n", __func__);

	return 0;
}

static int twl4030_phy_power_on(struct phy *phy)
{
	struct twl4030_usb *twl = phy_get_drvdata(phy);

	dev_dbg(twl->dev, "%s\n", __func__);
	pm_runtime_get_sync(twl->dev);
	schedule_delayed_work(&twl->id_workaround_work, HZ);
	pm_runtime_mark_last_busy(twl->dev);
	pm_runtime_put_autosuspend(twl->dev);

	return 0;
}

static int twl4030_usb_ldo_init(struct twl4030_usb *twl)
{
	/* Enable writing to power configuration registers */
	twl_i2c_write_u8(TWL_MODULE_PM_MASTER, TWL4030_PM_MASTER_KEY_CFG1,
			 TWL4030_PM_MASTER_PROTECT_KEY);

	twl_i2c_write_u8(TWL_MODULE_PM_MASTER, TWL4030_PM_MASTER_KEY_CFG2,
			 TWL4030_PM_MASTER_PROTECT_KEY);

	/* Keep VUSB3V1 LDO in sleep state until VBUS/ID change detected*/
	/*twl_i2c_write_u8(TWL_MODULE_PM_RECEIVER, 0, VUSB_DEDICATED2);*/

	/* input to VUSB3V1 LDO is from VBAT, not VBUS */
	twl_i2c_write_u8(TWL_MODULE_PM_RECEIVER, 0x14, VUSB_DEDICATED1);

	/* Initialize 3.1V regulator */
	twl_i2c_write_u8(TWL_MODULE_PM_RECEIVER, 0, VUSB3V1_DEV_GRP);

	twl->usb3v1 = devm_regulator_get(twl->dev, "usb3v1");
	if (IS_ERR(twl->usb3v1))
		return -ENODEV;

	twl_i2c_write_u8(TWL_MODULE_PM_RECEIVER, 0, VUSB3V1_TYPE);

	/* Initialize 1.5V regulator */
	twl_i2c_write_u8(TWL_MODULE_PM_RECEIVER, 0, VUSB1V5_DEV_GRP);

	twl->usb1v5 = devm_regulator_get(twl->dev, "usb1v5");
	if (IS_ERR(twl->usb1v5))
		return -ENODEV;

	twl_i2c_write_u8(TWL_MODULE_PM_RECEIVER, 0, VUSB1V5_TYPE);

	/* Initialize 1.8V regulator */
	twl_i2c_write_u8(TWL_MODULE_PM_RECEIVER, 0, VUSB1V8_DEV_GRP);

	twl->usb1v8 = devm_regulator_get(twl->dev, "usb1v8");
	if (IS_ERR(twl->usb1v8))
		return -ENODEV;

	twl_i2c_write_u8(TWL_MODULE_PM_RECEIVER, 0, VUSB1V8_TYPE);

	/* disable access to power configuration registers */
	twl_i2c_write_u8(TWL_MODULE_PM_MASTER, 0,
			 TWL4030_PM_MASTER_PROTECT_KEY);

	return 0;
}

static ssize_t vbus_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct twl4030_usb *twl = dev_get_drvdata(dev);
	int ret = -EINVAL;

	mutex_lock(&twl->lock);
	ret = sprintf(buf, "%s\n",
			twl->vbus_supplied ? "on" : "off");
	mutex_unlock(&twl->lock);

	return ret;
}
static DEVICE_ATTR_RO(vbus);

static irqreturn_t twl4030_usb_irq(int irq, void *_twl)
{
	struct twl4030_usb *twl = _twl;
	enum musb_vbus_id_status status;
	int err;

	status = twl4030_usb_linkstat(twl);

	mutex_lock(&twl->lock);
	twl->linkstat = status;
	mutex_unlock(&twl->lock);

	if (cable_present(status)) {
		if (atomic_add_unless(&twl->connected, 1, 1)) {
			dev_dbg(twl->dev, "%s: cable connected %i\n",
				__func__, status);
			pm_runtime_get_sync(twl->dev);
			/* VBUS from a host or charger: musb hears of it after detection */
			if (status == MUSB_VBUS_VALID && twl->phy.charger_detect)
				schedule_work(&twl->charger_work);
			else
				twl->musb_mailbox_pending = twl->musb_cable = true;
		}
	} else {
		if (atomic_add_unless(&twl->connected, -1, 0)) {
			dev_dbg(twl->dev, "%s: cable disconnected %i\n",
				__func__, status);
			if (twl->phy.charger_detect) {
				cancel_work_sync(&twl->charger_work);
				usb_phy_set_charger_state(&twl->phy, USB_CHARGER_ABSENT);
			}
			pm_runtime_mark_last_busy(twl->dev);
			pm_runtime_put_autosuspend(twl->dev);
			/*
			 * Not for a cable musb never heard of (a dedicated charger, a failed DCD),
			 * as stock: its VBUS_OFF would run the controller with the PHY the
			 * detection left powered down (no ULPI clock), after which CORE stayed out
			 * of retention until the PHY next powered up
			 */
			twl->musb_mailbox_pending = twl->musb_cable;
			twl->musb_cable = false;
		}
	}
	if (twl->musb_mailbox_pending) {
		err = musb_mailbox(status);
		if (!err)
			twl->musb_mailbox_pending = false;
	}

	/* don't schedule during sleep - irq works right then */
	if (status == MUSB_ID_GROUND && pm_runtime_active(twl->dev)) {
		cancel_delayed_work(&twl->id_workaround_work);
		schedule_delayed_work(&twl->id_workaround_work, HZ);
	}

	if (irq)
		sysfs_notify(&twl->dev->kobj, NULL, "vbus");

	return IRQ_HANDLED;
}

static void twl4030_id_workaround_work(struct work_struct *work)
{
	struct twl4030_usb *twl = container_of(work, struct twl4030_usb,
		id_workaround_work.work);

	twl4030_usb_irq(0, twl);
}

static int twl4030_phy_init(struct phy *phy)
{
	struct twl4030_usb *twl = phy_get_drvdata(phy);

	pm_runtime_get_sync(twl->dev);
	twl->linkstat = MUSB_UNKNOWN;
	schedule_delayed_work(&twl->id_workaround_work, HZ);
	pm_runtime_mark_last_busy(twl->dev);
	pm_runtime_put_autosuspend(twl->dev);

	return 0;
}

static int twl4030_set_peripheral(struct usb_otg *otg,
					struct usb_gadget *gadget)
{
	if (!otg)
		return -ENODEV;

	otg->gadget = gadget;
	if (!gadget)
		otg->state = OTG_STATE_UNDEFINED;

	return 0;
}

static int twl4030_set_host(struct usb_otg *otg, struct usb_bus *host)
{
	if (!otg)
		return -ENODEV;

	otg->host = host;
	if (!host)
		otg->state = OTG_STATE_UNDEFINED;

	return 0;
}

static const struct phy_ops ops = {
	.init		= twl4030_phy_init,
	.power_on	= twl4030_phy_power_on,
	.power_off	= twl4030_phy_power_off,
	.owner		= THIS_MODULE,
};

static const struct dev_pm_ops twl4030_usb_pm_ops = {
	SET_RUNTIME_PM_OPS(twl4030_usb_runtime_suspend,
			   twl4030_usb_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(twl4030_usb_suspend, twl4030_usb_resume)
};

static int twl4030_usb_probe(struct platform_device *pdev)
{
	struct twl4030_usb_data *pdata = dev_get_platdata(&pdev->dev);
	struct twl4030_usb	*twl;
	struct phy		*phy;
	int			status, err;
	struct usb_otg		*otg;
	struct device_node	*np = pdev->dev.of_node;
	struct phy_provider	*phy_provider;

	twl = devm_kzalloc(&pdev->dev, sizeof(*twl), GFP_KERNEL);
	if (!twl)
		return -ENOMEM;

	if (np)
		of_property_read_u32(np, "usb_mode",
				(enum twl4030_usb_mode *)&twl->usb_mode);
	else if (pdata) {
		twl->usb_mode = pdata->usb_mode;
	} else {
		dev_err(&pdev->dev, "twl4030 initialized without pdata\n");
		return -EINVAL;
	}

	otg = devm_kzalloc(&pdev->dev, sizeof(*otg), GFP_KERNEL);
	if (!otg)
		return -ENOMEM;

	twl->dev		= &pdev->dev;
	twl->irq		= platform_get_irq(pdev, 0);
	twl->vbus_supplied	= false;
	twl->linkstat		= MUSB_UNKNOWN;
	twl->musb_mailbox_pending = false;

	twl->phy.dev		= twl->dev;
	twl->phy.label		= "twl4030";
	twl->phy.otg		= otg;
	twl->phy.type		= USB_PHY_TYPE_USB2;

	otg->usb_phy		= &twl->phy;
	otg->set_host		= twl4030_set_host;
	otg->set_peripheral	= twl4030_set_peripheral;

	phy = devm_phy_create(twl->dev, NULL, &ops);
	if (IS_ERR(phy)) {
		dev_dbg(&pdev->dev, "Failed to create PHY\n");
		return PTR_ERR(phy);
	}

	phy_set_drvdata(phy, twl);

	phy_provider = devm_of_phy_provider_register(twl->dev,
		of_phy_simple_xlate);
	if (IS_ERR(phy_provider))
		return PTR_ERR(phy_provider);

	/* init mutex for workqueue */
	mutex_init(&twl->lock);

	INIT_DELAYED_WORK(&twl->id_workaround_work, twl4030_id_workaround_work);
	INIT_WORK(&twl->charger_work, twl5031_charger_work);
	if (np && of_device_is_compatible(np->parent, "ti,twl5031"))
		twl->phy.charger_detect = twl5031_charger_detect;

	err = twl4030_usb_ldo_init(twl);
	if (err) {
		dev_err(&pdev->dev, "ldo init failed\n");
		return err;
	}
	usb_add_phy_dev(&twl->phy);

	platform_set_drvdata(pdev, twl);
	if (device_create_file(&pdev->dev, &dev_attr_vbus))
		dev_warn(&pdev->dev, "could not create sysfs file\n");

	ATOMIC_INIT_NOTIFIER_HEAD(&twl->phy.notifier);

	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, 2000);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	/* Our job is to use irqs and status from the power module
	 * to keep the transceiver disabled when nothing's connected.
	 *
	 * FIXME we actually shouldn't start enabling it until the
	 * USB controller drivers have said they're ready, by calling
	 * set_host() and/or set_peripheral() ... OTG_capable boards
	 * need both handles, otherwise just one suffices.
	 */
	status = devm_request_threaded_irq(twl->dev, twl->irq, NULL,
			twl4030_usb_irq, IRQF_TRIGGER_FALLING |
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, "twl4030_usb", twl);
	if (status < 0) {
		dev_dbg(&pdev->dev, "can't get IRQ %d, err %d\n",
			twl->irq, status);
		return status;
	}

	if (pdata)
		err = phy_create_lookup(phy, "usb", "musb-hdrc.0");
	if (err)
		return err;

	pm_runtime_mark_last_busy(&pdev->dev);
	pm_runtime_put_autosuspend(twl->dev);

	return 0;
}

static void twl4030_usb_remove(struct platform_device *pdev)
{
	struct twl4030_usb *twl = platform_get_drvdata(pdev);
	int val;

	usb_remove_phy(&twl->phy);
	pm_runtime_get_sync(twl->dev);
	cancel_delayed_work_sync(&twl->id_workaround_work);
	cancel_work_sync(&twl->charger_work);
	device_remove_file(twl->dev, &dev_attr_vbus);

	/* set transceiver mode to power on defaults */
	twl4030_usb_set_mode(twl, -1);

	/* idle ulpi before powering off */
	if (cable_present(twl->linkstat))
		pm_runtime_put_noidle(twl->dev);
	pm_runtime_mark_last_busy(twl->dev);
	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_put_sync(twl->dev);
	pm_runtime_disable(twl->dev);

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

	/* disable complete OTG block */
	twl4030_usb_clear_bits(twl, POWER_CTRL, POWER_CTRL_OTG_ENAB);
}

#ifdef CONFIG_OF
static const struct of_device_id twl4030_usb_id_table[] = {
	{ .compatible = "ti,twl4030-usb" },
	{}
};
MODULE_DEVICE_TABLE(of, twl4030_usb_id_table);
#endif

static struct platform_driver twl4030_usb_driver = {
	.probe		= twl4030_usb_probe,
	.remove		= twl4030_usb_remove,
	.driver		= {
		.name	= "twl4030_usb",
		.pm	= &twl4030_usb_pm_ops,
		.of_match_table = of_match_ptr(twl4030_usb_id_table),
	},
};

static int __init twl4030_usb_init(void)
{
	return platform_driver_register(&twl4030_usb_driver);
}
subsys_initcall(twl4030_usb_init);

static void __exit twl4030_usb_exit(void)
{
	platform_driver_unregister(&twl4030_usb_driver);
}
module_exit(twl4030_usb_exit);

MODULE_ALIAS("platform:twl4030_usb");
MODULE_AUTHOR("Texas Instruments, Inc, Nokia Corporation");
MODULE_DESCRIPTION("TWL4030 USB transceiver driver");
MODULE_LICENSE("GPL");
