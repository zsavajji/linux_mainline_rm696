// SPDX-License-Identifier: GPL-2.0
/*
 * A CPU boost on input, as the Nokia N9's stock kernel had it (its OMAP3 PM optimizer,
 * arch/arm/mach-omap2/pm-optimizer.c, set up in board-rm680.c): after any key or touch event, CPU
 * 0's policy runs at its highest frequency and the CPU stays out of deep idle (at most 6 ms exit
 * latency) for a second. A gesture then starts at full speed, before the governor notices the load.
 * Only devices with keys (a touchscreen has BTN_TOUCH), so an accelerometer's stream doesn't keep
 * the CPU up, as Nokia's ignore list did.
 * The optimizer's other trigger is here too: a write to /sys/power/pm_optimizer_rotation (Harmattan's
 * sensord, when the screen's top edge changes) runs the CPU at full speed for a second, for the
 * rotation animation; without an idle limit, as stock.
 */
#include <linux/cpufreq.h>
#include <linux/input.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_qos.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

static unsigned int boost_ms = 1000;
module_param(boost_ms, uint, 0644);
MODULE_PARM_DESC(boost_ms, "How long a boost lasts after the last input event (ms)");

static unsigned int latency_us = 6000;
module_param(latency_us, uint, 0644);
MODULE_PARM_DESC(latency_us, "The CPU's idle exit latency limit while boosted (us)");

static struct freq_qos_request freq_req, rotation_req;
static struct pm_qos_request latency_req;
static bool boosted;
static DEFINE_MUTEX(rotation_lock);

static void input_boost_on(struct work_struct *work);
static void input_boost_off(struct work_struct *work);
static void rotation_boost_off(struct work_struct *work);
static DECLARE_WORK(boost_on_work, input_boost_on);
static DECLARE_DELAYED_WORK(boost_off_work, input_boost_off);
static DECLARE_DELAYED_WORK(rotation_off_work, rotation_boost_off);

/* CPU 0's policy at its highest frequency until freq_unboost(req); false without cpufreq */
static bool freq_boost(struct freq_qos_request *req)
{
	struct cpufreq_policy *policy;
	int ret;

	if (!freq_qos_request_active(req)) {
		policy = cpufreq_cpu_get(0);
		if (!policy)
			return false;	/* no cpufreq (yet): nothing to boost */
		ret = freq_qos_add_request(&policy->constraints, req, FREQ_QOS_MIN,
					   FREQ_QOS_MIN_DEFAULT_VALUE);
		cpufreq_cpu_put(policy);
		if (ret < 0)
			return false;
	}
	/* cpufreq clamps it to the policy's maximum now (boost OPPs count only with boost on) */
	freq_qos_update_request(req, FREQ_QOS_MAX_DEFAULT_VALUE);
	return true;
}

static void freq_unboost(struct freq_qos_request *req)
{
	if (freq_qos_request_active(req))
		freq_qos_update_request(req, FREQ_QOS_MIN_DEFAULT_VALUE);
}

static void input_boost_on(struct work_struct *work)
{
	if (!freq_boost(&freq_req))
		return;
	cpu_latency_qos_update_request(&latency_req, latency_us);
	mod_delayed_work(system_wq, &boost_off_work, msecs_to_jiffies(boost_ms));
}

static void input_boost_off(struct work_struct *work)
{
	WRITE_ONCE(boosted, false);
	freq_unboost(&freq_req);
	cpu_latency_qos_update_request(&latency_req, PM_QOS_DEFAULT_VALUE);
}

static void rotation_boost_off(struct work_struct *work)
{
	freq_unboost(&rotation_req);
}

/* Stock's file: any number written is a rotation; reads say load:100 while the boost lasts */
static ssize_t rotation_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "load:%u\n", delayed_work_pending(&rotation_off_work) ? 100 : 0);
}

static ssize_t rotation_store(struct kobject *kobj, struct kobj_attribute *attr,
			      const char *buf, size_t n)
{
	unsigned int value;

	if (kstrtouint(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&rotation_lock);	/* freq_boost() adds the request at the first one */
	if (freq_boost(&rotation_req))
		mod_delayed_work(system_wq, &rotation_off_work, msecs_to_jiffies(boost_ms));
	mutex_unlock(&rotation_lock);
	return n;
}

static struct kobj_attribute rotation_attr =
	__ATTR(pm_optimizer_rotation, 0644, rotation_show, rotation_store);

static void input_boost_event(struct input_handle *handle, unsigned int type,
			      unsigned int code, int value)
{
	if (!READ_ONCE(boosted)) {
		WRITE_ONCE(boosted, true);
		schedule_work(&boost_on_work);
	} else {
		mod_delayed_work(system_wq, &boost_off_work, msecs_to_jiffies(boost_ms));
	}
}

static int input_boost_connect(struct input_handler *handler, struct input_dev *dev,
			       const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpufreq-input-boost";

	error = input_register_handle(handle);
	if (error)
		goto err_free;
	error = input_open_device(handle);
	if (error)
		goto err_unregister;
	return 0;

err_unregister:
	input_unregister_handle(handle);
err_free:
	kfree(handle);
	return error;
}

static void input_boost_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id input_boost_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ }
};

/*
 * Not the accelerometer: it has (click) buttons too, but streams ABS events whenever its user
 * (Harmattan's sensord) polls it, which would keep the boost on for good.
 */
static bool input_boost_match(struct input_handler *handler, struct input_dev *dev)
{
	return !test_bit(EV_ABS, dev->evbit) || test_bit(BTN_TOUCH, dev->keybit);
}

static struct input_handler input_boost_handler = {
	.match = input_boost_match,
	.event = input_boost_event,
	.connect = input_boost_connect,
	.disconnect = input_boost_disconnect,
	.name = "cpufreq-input-boost",
	.id_table = input_boost_ids,
};

static int __init input_boost_init(void)
{
	int error;

	cpu_latency_qos_add_request(&latency_req, PM_QOS_DEFAULT_VALUE);
	error = sysfs_create_file(power_kobj, &rotation_attr.attr);
	if (error)
		pr_warn("cpufreq-input-boost: no /sys/power/%s: %d\n", rotation_attr.attr.name, error);
	return input_register_handler(&input_boost_handler);
}
late_initcall(input_boost_init);
