// SPDX-License-Identifier: GPL-2.0
/*
 * TWL5031 battery charging controller (BCC): the charger, as the "usb" and "vac" power supplies that
 * Nokia's battery daemon (BME) reads on the N9 (PRESENT, the type, and CURRENT_NOW as the input
 * current in mA). After Nokia's twl5031-bcc (drivers/power/twl5031-bcc.c in the N9's 2.6.32 kernel).
 * BME drives the charger itself, over /dev/i2c-1. This driver only reads.
 *
 * "usb" is the charger type the USB PHY's detection found (Nokia's, in phy-twl4030-usb.c) with
 * stock's currents: a host port (USB) 100 mA, then what the host configured (2 mA while it suspends
 * the bus); a charging downstream port (USB_CDP) 500 mA, 900 once configured, 500 while suspended;
 * a dedicated charger (USB_DCP) 500 mA. BME (bme_RX-71) charges from USB_DCP as from a wall charger,
 * from USB and USB_CDP at 100, 500 or 900 mA by CURRENT_NOW, and not at all for another value.
 *
 * BME reads the supplies when a "change@.../power_supply/usb" uevent reaches its netlink socket,
 * which it sizes with SO_RCVBUF 600. On 6.18 that holds about one uevent: one more that comes before
 * BME has read the first is dropped, along with everything after it until BME drains the queue. The
 * TWL PHY's own uevents (VBUS online/offline, the usb_phy's charger state) come right before ours, so
 * BME could miss an unplug, go on charging, and fail at its next 10 s charge check ("Charging
 * stopped by unknown error", the UI's wrong-charger note). Stock's kernel sent nothing else around
 * its USB events. So each change is announced once more after the burst, when BME's queue is empty.
 *
 * ponytail: "vac" is polled every 2 s instead of taken from the charger-detect interrupt.
 */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/usb/phy.h>
#include <linux/workqueue.h>
#include <linux/mfd/twl.h>

#define TWL5031_BCC_STS		0x02
#define TWL5031_VAC_P_STS	BIT(3)
#define TWL5031_BCC_STS_RESET	0xaa	/* no VBUS, no charger: the BCC in reset */
#define TWL5031_BCC_ANNOUNCE_MS	200	/* after the PHY's uevents; BME needs a few ms for them */

struct twl5031_bcc {
	struct power_supply *usb, *vac;
	struct delayed_work work;
	struct delayed_work announce;		/* the usb state once more after a change */
	bool usb_present, vac_present;
	struct power_supply_desc usb_desc;
	struct usb_phy *phy;
	struct notifier_block nb;
	enum usb_charger_type usb_type;		/* the last detection's */
	int usb_current;			/* mA */
};

static int twl5031_bcc_get_property(struct power_supply *psy, enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct twl5031_bcc *bcc = power_supply_get_drvdata(psy);
	bool usb = psy == bcc->usb;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = usb ? bcc->usb_present : bcc->vac_present;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* mA, as Nokia's driver reported it; it never set one for VAC */
		val->intval = usb ? bcc->usb_current : 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/*
 * Stock's USB events (twl5031_bcc_event_work()) from the usb_phy's charger state, on each of its
 * notifications: a new type is a detection's result, a new current for the same type the gadget's
 * vbus_draw (stock's USB_EVENT_ENUMERATED, SUSPENDED and RESUMED; it ignored 0 mA).
 */
static void twl5031_bcc_usb_update(struct twl5031_bcc *bcc)
{
	enum usb_charger_type type = bcc->phy->chg_type;
	unsigned int min, max;

	usb_phy_get_charger_current(bcc->phy, &min, &max);

	if (bcc->phy->chg_state != USB_CHARGER_PRESENT || type == UNKNOWN_TYPE) {
		bcc->usb_present = false;
		bcc->usb_current = 0;
		type = UNKNOWN_TYPE;
	} else if (type != bcc->usb_type) {
		/* the BCC state machine's 100 mA (a host port) or 500 mA */
		bcc->usb_present = true;
		bcc->usb_current = type == SDP_TYPE ? 100 : 500;
	} else if (max) {
		/* Nokia's "theoretical maximum" for a configured charging downstream port */
		if (type == CDP_TYPE)
			bcc->usb_current = max == 2 ? 500 : 900;
		else
			bcc->usb_current = max;
	}
	bcc->usb_type = type;
	bcc->usb_desc.type = type == CDP_TYPE ? POWER_SUPPLY_TYPE_USB_CDP :
			     type == DCP_TYPE ? POWER_SUPPLY_TYPE_USB_DCP : POWER_SUPPLY_TYPE_USB;
	power_supply_changed(bcc->usb);
	mod_delayed_work(system_wq, &bcc->announce, msecs_to_jiffies(TWL5031_BCC_ANNOUNCE_MS));
}

static void twl5031_bcc_announce(struct work_struct *work)
{
	power_supply_changed(container_of(work, struct twl5031_bcc, announce.work)->usb);
}

static int twl5031_bcc_usb_event(struct notifier_block *nb, unsigned long event, void *data)
{
	struct twl5031_bcc *bcc = container_of(nb, struct twl5031_bcc, nb);

	/* the usb_phy's charger notifications carry the PHY, musb's VBUS events nothing */
	if (data != bcc->phy)
		return NOTIFY_DONE;
	twl5031_bcc_usb_update(bcc);
	return NOTIFY_OK;
}

static void twl5031_bcc_poll(struct work_struct *work)
{
	struct twl5031_bcc *bcc = container_of(work, struct twl5031_bcc, work.work);
	bool vac = false;
	u8 sts;

	if (!twl_i2c_read_u8(TWL5031_MODULE_BCC, &sts, TWL5031_BCC_STS) &&
	    sts != TWL5031_BCC_STS_RESET)
		vac = sts & TWL5031_VAC_P_STS;
	if (vac != bcc->vac_present) {
		bcc->vac_present = vac;
		power_supply_changed(bcc->vac);
	}
	schedule_delayed_work(&bcc->work, 2 * HZ);
}

static enum power_supply_property twl5031_bcc_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static const struct power_supply_desc twl5031_bcc_usb_desc = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = twl5031_bcc_props,
	.num_properties = ARRAY_SIZE(twl5031_bcc_props),
	.get_property = twl5031_bcc_get_property,
};

static const struct power_supply_desc twl5031_bcc_vac_desc = {
	.name = "vac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = twl5031_bcc_props,
	.num_properties = ARRAY_SIZE(twl5031_bcc_props),
	.get_property = twl5031_bcc_get_property,
};

static int twl5031_bcc_probe(struct platform_device *pdev)
{
	struct power_supply_config cfg = {};
	struct twl5031_bcc *bcc;
	int ret;
	u8 sts;

	bcc = devm_kzalloc(&pdev->dev, sizeof(*bcc), GFP_KERNEL);
	if (!bcc)
		return -ENOMEM;

	bcc->phy = devm_usb_get_phy_by_phandle(&pdev->dev, "usb-phy", 0);
	if (IS_ERR(bcc->phy))
		return dev_err_probe(&pdev->dev, PTR_ERR(bcc->phy), "no USB PHY\n");

	cfg.drv_data = bcc;
	bcc->usb_desc = twl5031_bcc_usb_desc;
	bcc->usb = devm_power_supply_register(&pdev->dev, &bcc->usb_desc, &cfg);
	if (IS_ERR(bcc->usb))
		return PTR_ERR(bcc->usb);
	bcc->vac = devm_power_supply_register(&pdev->dev, &twl5031_bcc_vac_desc, &cfg);
	if (IS_ERR(bcc->vac))
		return PTR_ERR(bcc->vac);

	platform_set_drvdata(pdev, bcc);
	INIT_DELAYED_WORK(&bcc->work, twl5031_bcc_poll);
	INIT_DELAYED_WORK(&bcc->announce, twl5031_bcc_announce);

	bcc->nb.notifier_call = twl5031_bcc_usb_event;
	ret = usb_register_notifier(bcc->phy, &bcc->nb);
	if (ret)
		return ret;
	twl5031_bcc_usb_update(bcc);

	if (!twl_i2c_read_u8(TWL5031_MODULE_BCC, &sts, TWL5031_BCC_STS))
		dev_info(&pdev->dev, "status 0x%02x\n", sts);
	twl5031_bcc_poll(&bcc->work.work);

	return 0;
}

static void twl5031_bcc_remove(struct platform_device *pdev)
{
	struct twl5031_bcc *bcc = platform_get_drvdata(pdev);

	usb_unregister_notifier(bcc->phy, &bcc->nb);
	cancel_delayed_work_sync(&bcc->announce);
	cancel_delayed_work_sync(&bcc->work);
}

static const struct of_device_id twl5031_bcc_of_match[] = {
	{ .compatible = "ti,twl5031-bcc" },
	{ }
};
MODULE_DEVICE_TABLE(of, twl5031_bcc_of_match);

static struct platform_driver twl5031_bcc_driver = {
	.probe = twl5031_bcc_probe,
	.remove = twl5031_bcc_remove,
	.driver = {
		.name = "twl5031_bcc",
		.of_match_table = twl5031_bcc_of_match,
	},
};
module_platform_driver(twl5031_bcc_driver);

MODULE_DESCRIPTION("TWL5031 charger presence and type for the Nokia N9's BME");
MODULE_LICENSE("GPL");
