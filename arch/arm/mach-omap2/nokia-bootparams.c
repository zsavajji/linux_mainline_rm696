// SPDX-License-Identifier: GPL-2.0-only
/*
 * NOLO's gpio= boot parameter on the Nokia N9, as Nokia's kernel (kernel-plus-harmattan
 * arch/arm/mach-omap2/nokia-bootparams.c and board-gpio-export.c): NOLO lists the lines userspace
 * drives, e.g. gpio=cmt_apeslpx:93:sysfs:output,cmt_en:23:sysfs:init_active:output,... ; each is
 * requested under its name, set up and exported to sysfs, where Harmattan's udev rule
 * (82-ssc-daemon.rules: ATTR{name}=="cmt_*") links it as /dev/cmt/<name> for sscd, which powers the
 * modem up and down through them.
 *
 * <name>:<gpio>:<type>[:<flag>...], type sysfs (key: the gpio-keys NOLO lists on other boards, not
 * the N9's), flags output, init_active (inverted is unused on the N9). A line something else already
 * claimed (the DT's sleep_ind LED) is left to it.
 *
 * A modem that went off with cmt_en still high (a root `reboot` that cut sscd's power-off short, the
 * first boot after a flash) doesn't start when cmt_en is driven active here. sscd powers it on only
 * when it starts, ~20 s in and after csd, and csd then kept the SIM as PIN-required and the network
 * offline while sscd had the modem registered (2026-10-18), until csd was restarted. So once
 * udev's netif-up has opened phonet0 (~8 s), a link still without carrier 2 s later (ssi-protocol
 * turns it on at the modem's answer, 0.3 s on a normal boot) gets sscd's own RAPU2 power-on:
 * cmt_rst_rq and cmt_en low, 20 ms, both high. Only until the link first comes up and for at most
 * a minute: later modem resets are sscd's.
 */
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/workqueue.h>
#include <net/net_namespace.h>

static char *nokia_gpio_param __initdata;
static int nokia_cmt_en = -1, nokia_cmt_rst_rq = -1;
static unsigned int nokia_modem_checks, nokia_modem_silent;

static void nokia_modem_check(struct work_struct *work);
static DECLARE_DELAYED_WORK(nokia_modem_work, nokia_modem_check);

static void nokia_modem_check(struct work_struct *work)
{
	struct net_device *dev = dev_get_by_name(&init_net, "phonet0");
	bool open = dev && netif_running(dev), up = dev && netif_carrier_ok(dev);

	if (dev)
		dev_put(dev);
	if (up || ++nokia_modem_checks > 60)
		return;
	if (!open || ++nokia_modem_silent < 2) {
		schedule_delayed_work(&nokia_modem_work, HZ);
		return;
	}
	pr_info("nokia-bootparams: no answer from the modem on phonet0: powering it on\n");
	if (nokia_cmt_rst_rq >= 0)
		gpio_set_value_cansleep(nokia_cmt_rst_rq, 0);
	gpio_set_value_cansleep(nokia_cmt_en, 0);
	msleep(20);
	gpio_set_value_cansleep(nokia_cmt_en, 1);
	if (nokia_cmt_rst_rq >= 0)
		gpio_set_value_cansleep(nokia_cmt_rst_rq, 1);
}

static int __init nokia_gpio_setup(char *str)
{
	nokia_gpio_param = str;
	return 1;
}
__setup("gpio=", nokia_gpio_setup);

static void __init nokia_gpio_export_one(char *line)
{
	char *name = strsep(&line, ":");
	char *num = strsep(&line, ":");
	char *type = strsep(&line, ":");
	unsigned long flags = GPIOF_IN;
	bool output = false, active = false;
	unsigned int gpio;
	char *flag;
	int err;

	if (!name || !*name || !num || kstrtouint(num, 0, &gpio) || !type)
		return;
	if (strcmp(type, "sysfs")) {
		pr_info("nokia-bootparams: %s: type %s not supported\n", name, type);
		return;
	}
	while ((flag = strsep(&line, ":"))) {
		if (!strcmp(flag, "output"))
			output = true;
		else if (!strcmp(flag, "init_active"))
			active = true;
		else
			pr_info("nokia-bootparams: %s: flag %s not supported\n", name, flag);
	}
	if (output)
		flags = active ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW;

	/* the label outlives this initcall: it names the line in sysfs */
	name = kstrdup(name, GFP_KERNEL);
	if (!name)
		return;
	err = gpio_request_one(gpio, flags, name);
	if (!err) {
		err = gpiod_export(gpio_to_desc(gpio), false);
		if (err)
			gpio_free(gpio);
	}
	if (err) {
		pr_info("nokia-bootparams: %s (gpio %u) not exported: %d\n", name, gpio, err);
		kfree(name);
	} else if (output && !strcmp(name, "cmt_en")) {
		nokia_cmt_en = gpio;
	} else if (output && !strcmp(name, "cmt_rst_rq")) {
		nokia_cmt_rst_rq = gpio;
	}
}

static int __init nokia_gpio_init(void)
{
	char *param, *p, *line;

	if (!nokia_gpio_param || !of_machine_is_compatible("nokia,omap3-n9"))
		return 0;

	param = p = kstrdup(nokia_gpio_param, GFP_KERNEL);
	if (!param)
		return -ENOMEM;
	while ((line = strsep(&p, ",")))
		nokia_gpio_export_one(line);
	kfree(param);
	if (nokia_cmt_en >= 0)
		schedule_delayed_work(&nokia_modem_work, HZ);
	return 0;
}
late_initcall(nokia_gpio_init);
