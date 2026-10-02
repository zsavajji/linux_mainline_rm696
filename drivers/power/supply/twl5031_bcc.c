// SPDX-License-Identifier: GPL-2.0
/*
 * TWL5031 battery charging controller (BCC): the charger's presence, as the "usb" and "vac" power
 * supplies that Nokia's battery daemon (BME) reads on the N9 (PRESENT, and CURRENT_NOW as the input
 * current in mA). After Nokia's twl5031-bcc (drivers/power/twl5031-bcc.c in the N9's 2.6.32
 * kernel), reduced to what BME needs from the kernel: BME drives the charger itself, over
 * /dev/i2c-1. This driver only reads.
 *
 * ponytail: presence is polled every 2 s instead of taken from the charger-detect interrupt, and a
 * USB charger is reported as a 500 mA host port. Nokia's charger-type detection (data contact
 * detection and the BCC's detection state machine) drives the USB PHY's data lines, which on the
 * N9 test kernel carry its network; a dedicated wall charger then charges at the host rate.
 */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/mfd/twl.h>

#define TWL5031_BCC_STS		0x02
#define TWL5031_USB_P_STS	BIT(2)
#define TWL5031_VAC_P_STS	BIT(3)
#define TWL5031_BCC_STS_RESET	0xaa	/* no VBUS, no charger: the BCC in reset */

struct twl5031_bcc {
	struct power_supply *usb, *vac;
	struct delayed_work work;
	bool usb_present, vac_present;
};

static int twl5031_bcc_get_property(struct power_supply *psy, enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct twl5031_bcc *bcc = power_supply_get_drvdata(psy);
	bool usb = psy == bcc->usb;
	bool present = usb ? bcc->usb_present : bcc->vac_present;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = present;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* mA, as Nokia's driver reported it; it never set one for VAC */
		val->intval = present && usb ? 500 : 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void twl5031_bcc_poll(struct work_struct *work)
{
	struct twl5031_bcc *bcc = container_of(work, struct twl5031_bcc, work.work);
	bool usb = false, vac = false;
	u8 sts;

	if (!twl_i2c_read_u8(TWL5031_MODULE_BCC, &sts, TWL5031_BCC_STS) &&
	    sts != TWL5031_BCC_STS_RESET) {
		usb = sts & TWL5031_USB_P_STS;
		vac = sts & TWL5031_VAC_P_STS;
	}
	if (usb != bcc->usb_present) {
		bcc->usb_present = usb;
		power_supply_changed(bcc->usb);
	}
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
	u8 sts;

	bcc = devm_kzalloc(&pdev->dev, sizeof(*bcc), GFP_KERNEL);
	if (!bcc)
		return -ENOMEM;

	cfg.drv_data = bcc;
	bcc->usb = devm_power_supply_register(&pdev->dev, &twl5031_bcc_usb_desc, &cfg);
	if (IS_ERR(bcc->usb))
		return PTR_ERR(bcc->usb);
	bcc->vac = devm_power_supply_register(&pdev->dev, &twl5031_bcc_vac_desc, &cfg);
	if (IS_ERR(bcc->vac))
		return PTR_ERR(bcc->vac);

	platform_set_drvdata(pdev, bcc);
	INIT_DELAYED_WORK(&bcc->work, twl5031_bcc_poll);

	if (!twl_i2c_read_u8(TWL5031_MODULE_BCC, &sts, TWL5031_BCC_STS))
		dev_info(&pdev->dev, "status 0x%02x\n", sts);
	twl5031_bcc_poll(&bcc->work.work);

	return 0;
}

static void twl5031_bcc_remove(struct platform_device *pdev)
{
	struct twl5031_bcc *bcc = platform_get_drvdata(pdev);

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

MODULE_DESCRIPTION("TWL5031 charger presence for the Nokia N9's BME");
MODULE_LICENSE("GPL");
