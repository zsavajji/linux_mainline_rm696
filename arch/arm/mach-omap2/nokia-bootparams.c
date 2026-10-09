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
 */
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/string.h>

static char *nokia_gpio_param __initdata;

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
	return 0;
}
late_initcall(nokia_gpio_init);
