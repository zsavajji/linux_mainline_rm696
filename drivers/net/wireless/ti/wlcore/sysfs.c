// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is part of wlcore
 *
 * Copyright (C) 2013 Texas Instruments Inc.
 */

#include <linux/pm_runtime.h>

#include "acx.h"
#include "wlcore.h"
#include "debug.h"
#include "sysfs.h"

static ssize_t bt_coex_state_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct wl1271 *wl = dev_get_drvdata(dev);
	ssize_t len;

	mutex_lock(&wl->mutex);
	len = sysfs_emit(buf, "%d\n\n0 - off\n1 - on\n", wl->sg_enabled);
	mutex_unlock(&wl->mutex);

	return len;

}

static ssize_t bt_coex_state_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct wl1271 *wl = dev_get_drvdata(dev);
	unsigned long res;
	int ret;

	ret = kstrtoul(buf, 10, &res);
	if (ret < 0) {
		wl1271_warning("incorrect value written to bt_coex_mode");
		return count;
	}

	mutex_lock(&wl->mutex);

	res = !!res;

	if (res == wl->sg_enabled)
		goto out;

	wl->sg_enabled = res;

	if (unlikely(wl->state != WLCORE_STATE_ON))
		goto out;

	ret = pm_runtime_resume_and_get(wl->dev);
	if (ret < 0)
		goto out;

	wl1271_acx_sg_enable(wl, wl->sg_enabled);
	pm_runtime_mark_last_busy(wl->dev);
	pm_runtime_put_autosuspend(wl->dev);

 out:
	mutex_unlock(&wl->mutex);
	return count;
}

static DEVICE_ATTR_RW(bt_coex_state);

static ssize_t hw_pg_ver_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct wl1271 *wl = dev_get_drvdata(dev);
	ssize_t len;

	mutex_lock(&wl->mutex);
	if (wl->hw_pg_ver >= 0)
		len = sysfs_emit(buf, "%d\n", wl->hw_pg_ver);
	else
		len = sysfs_emit(buf, "n/a\n");
	mutex_unlock(&wl->mutex);

	return len;
}

static DEVICE_ATTR_RO(hw_pg_ver);

/*
 * As Nokia's driver (kernel-plus-harmattan drivers/net/wireless/wl12xx/main.c): Harmattan's
 * wl1271-cal writes allow_adhoc for the phone's region, at stock's path
 * /sys/devices/platform/wl1271/allow_adhoc (a link to this device, made below); 0 refuses ad-hoc
 */
static ssize_t allow_adhoc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct wl1271 *wl = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", !wl->no_adhoc);
}

static ssize_t allow_adhoc_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct wl1271 *wl = dev_get_drvdata(dev);
	unsigned long res;
	int ret;

	ret = kstrtoul(buf, 10, &res);
	if (ret < 0)
		return ret;

	mutex_lock(&wl->mutex);
	wl->no_adhoc = !res;
	mutex_unlock(&wl->mutex);

	return count;
}

static DEVICE_ATTR_RW(allow_adhoc);

static ssize_t wl1271_sysfs_read_fwlog(struct file *filp, struct kobject *kobj,
				       const struct bin_attribute *bin_attr,
				       char *buffer, loff_t pos, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct wl1271 *wl = dev_get_drvdata(dev);
	ssize_t len;
	int ret;

	ret = mutex_lock_interruptible(&wl->mutex);
	if (ret < 0)
		return -ERESTARTSYS;

	/* Check if the fwlog is still valid */
	if (wl->fwlog_size < 0) {
		mutex_unlock(&wl->mutex);
		return 0;
	}

	/* Seeking is not supported - old logs are not kept. Disregard pos. */
	len = min_t(size_t, count, wl->fwlog_size);
	wl->fwlog_size -= len;
	memcpy(buffer, wl->fwlog, len);

	/* Make room for new messages */
	memmove(wl->fwlog, wl->fwlog + len, wl->fwlog_size);

	mutex_unlock(&wl->mutex);

	return len;
}

static const struct bin_attribute fwlog_attr = {
	.attr = { .name = "fwlog", .mode = 0400 },
	.read = wl1271_sysfs_read_fwlog,
};

int wlcore_sysfs_init(struct wl1271 *wl)
{
	int ret;

	/* Create sysfs file to control bt coex state */
	ret = device_create_file(wl->dev, &dev_attr_bt_coex_state);
	if (ret < 0) {
		wl1271_error("failed to create sysfs file bt_coex_state");
		goto out;
	}

	/* Create sysfs file to get HW PG version */
	ret = device_create_file(wl->dev, &dev_attr_hw_pg_ver);
	if (ret < 0) {
		wl1271_error("failed to create sysfs file hw_pg_ver");
		goto out_bt_coex_state;
	}

	/* Create sysfs file for the FW log */
	ret = device_create_bin_file(wl->dev, &fwlog_attr);
	if (ret < 0) {
		wl1271_error("failed to create sysfs file fwlog");
		goto out_hw_pg_ver;
	}

	ret = device_create_file(wl->dev, &dev_attr_allow_adhoc);
	if (ret < 0) {
		wl1271_error("failed to create sysfs file allow_adhoc");
		goto out_fwlog;
	}

	/* the files at stock's path too (Harmattan's wl1271-cal, wlancond) */
	if (sysfs_create_link(&platform_bus.kobj, &wl->dev->kobj, "wl1271"))
		wl1271_warning("no /sys/devices/platform/wl1271");

	goto out;

out_fwlog:
	device_remove_bin_file(wl->dev, &fwlog_attr);

out_hw_pg_ver:
	device_remove_file(wl->dev, &dev_attr_hw_pg_ver);

out_bt_coex_state:
	device_remove_file(wl->dev, &dev_attr_bt_coex_state);

out:
	return ret;
}

void wlcore_sysfs_free(struct wl1271 *wl)
{
	sysfs_remove_link(&platform_bus.kobj, "wl1271");
	device_remove_file(wl->dev, &dev_attr_allow_adhoc);

	device_remove_bin_file(wl->dev, &fwlog_attr);

	device_remove_file(wl->dev, &dev_attr_hw_pg_ver);

	device_remove_file(wl->dev, &dev_attr_bt_coex_state);
}
