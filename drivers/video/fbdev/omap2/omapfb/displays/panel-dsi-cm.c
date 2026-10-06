// SPDX-License-Identifier: GPL-2.0-only
/*
 * Generic DSI Command Mode panel driver
 *
 * Copyright (C) 2013 Texas Instruments
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 */

/* #define DEBUG */

#include <linux/backlight.h>
#include <linux/clk/ti.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/gpio/consumer.h>
#include <linux/hrtimer.h>
#include <video/of_display_timing.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/sched/signal.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <video/omapfb_dss.h>
#include <video/mipi_display.h>

/* DSI Virtual channel. Hardcoded for now. */
#define TCH 0

#define DCS_READ_NUM_ERRORS	0x05
#define DCS_BRIGHTNESS		0x51
#define DCS_READ_BRIGHTNESS	0x52
#define DCS_CTRL_DISPLAY	0x53
#define DCS_WRITE_CABC		0x55
#define DCS_GET_ID1		0xda
#define DCS_GET_ID2		0xdb
#define DCS_GET_ID3		0xdc

struct panel_drv_data {
	struct omap_dss_device dssdev;
	struct omap_dss_device *in;

	struct omap_video_timings timings;
	/* the visible area's offset in the panel's frame; DCS address mode, 0 = the panel's default */
	u16 x_offset, y_offset;
	u8 address_mode;
	/* the panel's rows to show when its frame is larger than the visible area (DCS partial mode) */
	u16 partial_start, partial_rows;
	u8 rotate;	/* the DT rotation in quarter turns */

	struct platform_device *pdev;

	struct mutex lock;

	struct backlight_device *bldev;

	unsigned long	hw_guard_end;	/* next value of jiffies when we can
					 * issue the next sleep in/out command
					 */
	unsigned long	hw_guard_wait;	/* max guard time in jiffies */

	/* panel HW configuration from DT or platform data */
	struct gpio_desc *reset_gpio;
	struct gpio_desc *ext_te_gpio;

	bool use_dsi_backlight;

	struct omap_dsi_pin_config pin_config;

	/* runtime variables */
	bool enabled;
	u8 id[3];	/* DCS ID1-ID3, read at each power on (hw_revision) */

	bool te_enabled;

	atomic_t do_update;
	int channel;

	struct delayed_work te_timeout_work;

	/*
	 * Landscape through the panel's row/column exchange (rotate odd) with the TE line: each update
	 * in two halves, see dsicm_split_half()
	 */
	atomic_t split_state;
	struct hrtimer split_timer;
	struct work_struct split_work;
	ktime_t te_last;
	u32 frame_us;
	/* each half transfer, last and longest (us), in sysfs split_timing */
	ktime_t split_t0;
	u32 split_us[2], split_max[2];
	/*
	 * Calibration, in sysfs: the right half first instead of stock's left (rotate 1), and the
	 * first half's start after TE (0: half the TE period)
	 */
	bool split_right_first;
	u32 split_mid_us;
	/*
	 * Stock's scheme for the N9's SMD Pyrenees panel (id fe): TE at mid-scan, the first half from
	 * TE, the second from the timer (sysfs split_reverse); frame_rows is the panel's frame height
	 */
	bool split_reverse;
	u16 frame_rows;

	bool intro_printed;

	bool ulps_enabled;
	unsigned ulps_timeout;
	struct delayed_work ulps_work;

	/*
	 * The N9 stock driver's Pyrenees controls, which Harmattan's mce drives through sysfs (see
	 * dsicm_pyrenees_attrs): low power mode (DCS idle mode, for the standby screen), display
	 * dimming (the panel fades brightness changes) and high brightness mode with its current
	 * limiting (ACL, through the CABC command)
	 */
	bool pyrenees;
	bool lpm, lpm_pending;
	struct delayed_work lpm_work;
	bool dimming;
	u8 hbm, acl;

	/*
	 * Frame pacing, in sysfs update_stats: frames by the TE periods since the one before as they
	 * start on the panel (1, 2, 3, 4 or more; 1 is 60 a second) and pauses (over 100 ms); update
	 * requests by when they come after TE; first halves started late (dsicm_split_late_start).
	 * And the CPU's idle exit latency limit while the panel is on outside low power mode (sysfs
	 * idle_latency_us, 0 = none), stock's PM optimizer's display rule: 6 ms, which keeps CORE out
	 * of retention mid-animation
	 */
	ktime_t stats_last;
	u32 stats[5], phase[4], late_starts;
	/*
	 * Halves out of their window (a tear, or the halves a frame apart): first halves that ended
	 * after the scan came back to their rows, second halves whose timer found them not ready (out
	 * late in the same frame if they still fit, else a frame late), second halves still going at
	 * the next TE
	 */
	u32 half1_over, half2_missed, half2_late, half2_over;
	/*
	 * A split update's frame latched when it's asked for (sysfs region_latch, as stock) or at its
	 * first half; early_pans: frames a flip changed between the two (kept, or replaced unseen), and
	 * blocked: updates that waited over 1 ms for the previous frame (X stalls meanwhile)
	 */
	bool region_latch;
	u32 early_pans, early_pans_base, blocked, blocked_ms;
	u32 split_late_us;
	struct pm_qos_request idle_qos;
	u32 idle_latency_us;

	/*
	 * L3 (OMAP3 CORE/L3 scaling) held high, as stock asked while a display was on: the DSS's
	 * 172.8 MHz functional clock is too fast for VDD2 at OPP50 (stock's omap_dss_start_device:
	 * "DSS does not work properly if DSS clocks are over 100MHz and VDD2 is at the lowest OPP").
	 * Held while the DSS runs, and while the panel is on outside its standby screen (LPM); stock
	 * held it in LPM too, sysfs l3_lpm_hold does that (the fallback if LPM misbehaves at OPP50)
	 */
	bool l3_held, l3_lpm_hold;
};

#define to_panel_data(p) container_of(p, struct panel_drv_data, dssdev)

static irqreturn_t dsicm_te_isr(int irq, void *data);
static void dsicm_te_timeout_work_callback(struct work_struct *work);
static int _dsicm_enable_te(struct panel_drv_data *ddata, bool enable);

static int dsicm_panel_reset(struct panel_drv_data *ddata);

static void dsicm_ulps_work(struct work_struct *work);

static void hw_guard_start(struct panel_drv_data *ddata, int guard_msec)
{
	ddata->hw_guard_wait = msecs_to_jiffies(guard_msec);
	ddata->hw_guard_end = jiffies + ddata->hw_guard_wait;
}

static void hw_guard_wait(struct panel_drv_data *ddata)
{
	unsigned long wait = ddata->hw_guard_end - jiffies;

	if ((long)wait > 0 && time_before_eq(wait, ddata->hw_guard_wait)) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(wait);
	}
}

static int dsicm_dcs_read_1(struct panel_drv_data *ddata, u8 dcs_cmd, u8 *data)
{
	struct omap_dss_device *in = ddata->in;
	int r;
	u8 buf[1];

	r = in->ops.dsi->dcs_read(in, ddata->channel, dcs_cmd, buf, 1);

	if (r < 0)
		return r;

	*data = buf[0];

	return 0;
}

static int dsicm_dcs_write_0(struct panel_drv_data *ddata, u8 dcs_cmd)
{
	struct omap_dss_device *in = ddata->in;
	return in->ops.dsi->dcs_write(in, ddata->channel, &dcs_cmd, 1);
}

static int dsicm_dcs_write_1(struct panel_drv_data *ddata, u8 dcs_cmd, u8 param)
{
	struct omap_dss_device *in = ddata->in;
	u8 buf[2] = { dcs_cmd, param };

	return in->ops.dsi->dcs_write(in, ddata->channel, buf, 2);
}

static int dsicm_sleep_in(struct panel_drv_data *ddata)

{
	struct omap_dss_device *in = ddata->in;
	u8 cmd;
	int r;

	hw_guard_wait(ddata);

	cmd = MIPI_DCS_ENTER_SLEEP_MODE;
	r = in->ops.dsi->dcs_write_nosync(in, ddata->channel, &cmd, 1);
	if (r)
		return r;

	hw_guard_start(ddata, 120);

	usleep_range(5000, 10000);

	return 0;
}

static int dsicm_sleep_out(struct panel_drv_data *ddata)
{
	int r;

	hw_guard_wait(ddata);

	r = dsicm_dcs_write_0(ddata, MIPI_DCS_EXIT_SLEEP_MODE);
	if (r)
		return r;

	hw_guard_start(ddata, 120);

	usleep_range(5000, 10000);

	return 0;
}

static int dsicm_get_id(struct panel_drv_data *ddata, u8 *id1, u8 *id2, u8 *id3)
{
	int r;

	r = dsicm_dcs_read_1(ddata, DCS_GET_ID1, id1);
	if (r)
		return r;
	r = dsicm_dcs_read_1(ddata, DCS_GET_ID2, id2);
	if (r)
		return r;
	r = dsicm_dcs_read_1(ddata, DCS_GET_ID3, id3);
	if (r)
		return r;

	return 0;
}

static int dsicm_set_update_window(struct panel_drv_data *ddata,
		u16 x, u16 y, u16 w, u16 h)
{
	struct omap_dss_device *in = ddata->in;
	int r;
	u16 x1 = x + ddata->x_offset;
	u16 x2 = x1 + w - 1;
	u16 y1 = y + ddata->y_offset;
	u16 y2 = y1 + h - 1;

	u8 buf[5];
	buf[0] = MIPI_DCS_SET_COLUMN_ADDRESS;
	buf[1] = (x1 >> 8) & 0xff;
	buf[2] = (x1 >> 0) & 0xff;
	buf[3] = (x2 >> 8) & 0xff;
	buf[4] = (x2 >> 0) & 0xff;

	r = in->ops.dsi->dcs_write_nosync(in, ddata->channel, buf, sizeof(buf));
	if (r)
		return r;

	buf[0] = MIPI_DCS_SET_PAGE_ADDRESS;
	buf[1] = (y1 >> 8) & 0xff;
	buf[2] = (y1 >> 0) & 0xff;
	buf[3] = (y2 >> 8) & 0xff;
	buf[4] = (y2 >> 0) & 0xff;

	r = in->ops.dsi->dcs_write_nosync(in, ddata->channel, buf, sizeof(buf));
	if (r)
		return r;

	in->ops.dsi->bta_sync(in, ddata->channel);

	return r;
}

static void dsicm_queue_ulps_work(struct panel_drv_data *ddata)
{
	if (ddata->ulps_timeout > 0)
		schedule_delayed_work(&ddata->ulps_work,
				msecs_to_jiffies(ddata->ulps_timeout));
}

static void dsicm_cancel_ulps_work(struct panel_drv_data *ddata)
{
	cancel_delayed_work(&ddata->ulps_work);
}

static void dsicm_set_l3_hold(struct panel_drv_data *ddata, bool dss_on)
{
	bool want = dss_on || (ddata->enabled && (!ddata->lpm || ddata->l3_lpm_hold));

	if (want == ddata->l3_held)
		return;
	ddata->l3_held = want;
	if (want)
		omap3_l3_hold(OMAP3_L3_DSS);
	else
		omap3_l3_release(OMAP3_L3_DSS);
}

static int dsicm_enter_ulps(struct panel_drv_data *ddata)
{
	struct omap_dss_device *in = ddata->in;
	int r;

	if (ddata->ulps_enabled)
		return 0;

	dsicm_cancel_ulps_work(ddata);

	r = _dsicm_enable_te(ddata, false);
	if (r)
		goto err;

	if (ddata->ext_te_gpio)
		disable_irq(gpiod_to_irq(ddata->ext_te_gpio));

	in->ops.dsi->disable(in, false, true);

	ddata->ulps_enabled = true;
	dsicm_set_l3_hold(ddata, false);

	return 0;

err:
	dev_err(&ddata->pdev->dev, "enter ULPS failed");
	dsicm_panel_reset(ddata);

	ddata->ulps_enabled = false;

	dsicm_queue_ulps_work(ddata);

	return r;
}

static int dsicm_exit_ulps(struct panel_drv_data *ddata)
{
	struct omap_dss_device *in = ddata->in;
	int r;

	if (!ddata->ulps_enabled)
		return 0;

	dsicm_set_l3_hold(ddata, true);
	r = in->ops.dsi->enable(in);
	if (r) {
		dev_err(&ddata->pdev->dev, "failed to enable DSI\n");
		goto err1;
	}

	in->ops.dsi->enable_hs(in, ddata->channel, true);

	r = _dsicm_enable_te(ddata, true);
	if (r) {
		dev_err(&ddata->pdev->dev, "failed to re-enable TE");
		goto err2;
	}

	if (ddata->ext_te_gpio)
		enable_irq(gpiod_to_irq(ddata->ext_te_gpio));

	dsicm_queue_ulps_work(ddata);

	ddata->ulps_enabled = false;

	return 0;

err2:
	dev_err(&ddata->pdev->dev, "failed to exit ULPS");

	r = dsicm_panel_reset(ddata);
	if (!r) {
		if (ddata->ext_te_gpio)
			enable_irq(gpiod_to_irq(ddata->ext_te_gpio));
		ddata->ulps_enabled = false;
	}
err1:
	dsicm_set_l3_hold(ddata, ddata->enabled && !ddata->ulps_enabled);
	dsicm_queue_ulps_work(ddata);

	return r;
}

static int dsicm_wake_up(struct panel_drv_data *ddata)
{
	if (ddata->ulps_enabled)
		return dsicm_exit_ulps(ddata);

	dsicm_cancel_ulps_work(ddata);
	dsicm_queue_ulps_work(ddata);
	return 0;
}

static int dsicm_bl_update_status(struct backlight_device *dev)
{
	struct panel_drv_data *ddata = dev_get_drvdata(&dev->dev);
	struct omap_dss_device *in = ddata->in;
	int r;
	int level = backlight_get_brightness(dev);

	dev_dbg(&ddata->pdev->dev, "update brightness to %d\n", level);

	mutex_lock(&ddata->lock);

	if (ddata->enabled) {
		in->ops.dsi->bus_lock(in);

		r = dsicm_wake_up(ddata);
		if (!r)
			r = dsicm_dcs_write_1(ddata, DCS_BRIGHTNESS, level);

		in->ops.dsi->bus_unlock(in);
	} else {
		r = 0;
	}

	mutex_unlock(&ddata->lock);

	return r;
}

static int dsicm_bl_get_intensity(struct backlight_device *dev)
{
	return backlight_get_brightness(dev);
}

static const struct backlight_ops dsicm_bl_ops = {
	.get_brightness = dsicm_bl_get_intensity,
	.update_status  = dsicm_bl_update_status,
};

static void dsicm_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	*xres = dssdev->panel.timings.x_res;
	*yres = dssdev->panel.timings.y_res;
}

static ssize_t dsicm_num_errors_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	struct omap_dss_device *in = ddata->in;
	u8 errors = 0;
	int r;

	mutex_lock(&ddata->lock);

	if (ddata->enabled) {
		in->ops.dsi->bus_lock(in);

		r = dsicm_wake_up(ddata);
		if (!r)
			r = dsicm_dcs_read_1(ddata, DCS_READ_NUM_ERRORS,
					&errors);

		in->ops.dsi->bus_unlock(in);
	} else {
		r = -ENODEV;
	}

	mutex_unlock(&ddata->lock);

	if (r)
		return r;

	return sysfs_emit(buf, "%d\n", errors);
}

/* As Nokia's driver: the IDs read at the last power on, so also with the panel off */
static ssize_t dsicm_hw_revision_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);

	guard(mutex)(&ddata->lock);
	return sysfs_emit(buf, "%02x.%02x.%02x\n", ddata->id[0], ddata->id[1], ddata->id[2]);
}

/* DCS WRITE_CTRL_DISPLAY: BL and BCTRL, with the Pyrenees' display dimming (DD) and HBM (bits 6-7) */
static int dsicm_write_ctrl_display(struct panel_drv_data *ddata)
{
	u8 ctrl = (1 << 2) | (1 << 5) | (ddata->dimming ? 1 << 3 : 0) | ddata->hbm << 6;

	return dsicm_dcs_write_1(ddata, DCS_CTRL_DISPLAY, ctrl);
}

/* Low power mode: DCS idle mode (8 colours, a lower frame rate: stock's timer assumed 15 fps), within
 * the visible area's partial mode, which stays on; stock switched partial mode with LPM only on
 * panels without one */
static int dsicm_write_lpm(struct panel_drv_data *ddata)
{
	int r = 0;

	if (!ddata->partial_rows)
		r = dsicm_dcs_write_0(ddata, ddata->lpm ? MIPI_DCS_ENTER_PARTIAL_MODE :
					     MIPI_DCS_ENTER_NORMAL_MODE);
	if (!r)
		r = dsicm_dcs_write_0(ddata, ddata->lpm ? MIPI_DCS_ENTER_IDLE_MODE :
					     MIPI_DCS_EXIT_IDLE_MODE);
	return r;
}

/* High brightness mode always with current limiting, as stock */
static int dsicm_write_hbm(struct panel_drv_data *ddata)
{
	int r = dsicm_dcs_write_1(ddata, DCS_WRITE_CABC, ddata->acl);

	return r ? r : dsicm_write_ctrl_display(ddata);
}

/* A changed setting to the panel, if it's on (dsicm_power_on writes them all); ddata->lock held */
static int dsicm_apply(struct panel_drv_data *ddata, int (*write)(struct panel_drv_data *))
{
	struct omap_dss_device *in = ddata->in;
	int r;

	if (!ddata->enabled)
		return 0;

	in->ops.dsi->bus_lock(in);
	r = dsicm_wake_up(ddata);
	if (!r)
		r = write(ddata);
	in->ops.dsi->bus_unlock(in);
	return r;
}

static void dsicm_set_idle_qos(struct panel_drv_data *ddata)
{
	bool want = ddata->enabled && !ddata->lpm && ddata->idle_latency_us;

	if (!want) {
		if (cpu_latency_qos_request_active(&ddata->idle_qos))
			cpu_latency_qos_remove_request(&ddata->idle_qos);
	} else if (cpu_latency_qos_request_active(&ddata->idle_qos)) {
		cpu_latency_qos_update_request(&ddata->idle_qos, ddata->idle_latency_us);
	} else {
		cpu_latency_qos_add_request(&ddata->idle_qos, ddata->idle_latency_us);
	}
}

static void dsicm_lpm_work(struct work_struct *work)
{
	struct panel_drv_data *ddata = container_of(work, struct panel_drv_data, lpm_work.work);

	mutex_lock(&ddata->lock);
	if (ddata->lpm_pending) {
		ddata->lpm_pending = false;
		ddata->lpm = true;
		dsicm_apply(ddata, dsicm_write_lpm);
		dsicm_set_idle_qos(ddata);
		dsicm_set_l3_hold(ddata, ddata->enabled && !ddata->ulps_enabled);
	}
	mutex_unlock(&ddata->lock);
}

/* Off at once; on 300 ms after the last update (dsicm_update postpones it), so that the standby
 * screen is drawn first, as stock */
#define DSICM_LPM_DELAY_MS	300

static ssize_t dsicm_lpm_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	bool t;
	int r;

	r = kstrtobool(buf, &t);
	if (r)
		return r;

	mutex_lock(&ddata->lock);
	ddata->lpm_pending = false;
	cancel_delayed_work(&ddata->lpm_work);
	if (t) {
		ddata->lpm_pending = true;
		schedule_delayed_work(&ddata->lpm_work, msecs_to_jiffies(DSICM_LPM_DELAY_MS));
	} else if (ddata->lpm) {
		ddata->lpm = false;
		r = dsicm_apply(ddata, dsicm_write_lpm);
		dsicm_set_idle_qos(ddata);
		dsicm_set_l3_hold(ddata, ddata->enabled && !ddata->ulps_enabled);
	}
	mutex_unlock(&ddata->lock);

	return r ? r : count;
}

static ssize_t dsicm_lpm_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", ddata->lpm);
}

static ssize_t dsicm_dimming_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	bool t;
	int r;

	r = kstrtobool(buf, &t);
	if (r)
		return r;

	mutex_lock(&ddata->lock);
	ddata->dimming = t;
	r = dsicm_apply(ddata, dsicm_write_ctrl_display);
	mutex_unlock(&ddata->lock);

	return r ? r : count;
}

static ssize_t dsicm_dimming_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", ddata->dimming);
}

/*
 * 0 off, 1 400 cd/m² with ACL 80%, 2 450 cd/m²: stock's values. The SMD Pyrenees has no 450, so 2
 * is its 500 with ACL 70%, shown as 2 (stock hid the difference from the LGD panel this way).
 */
static ssize_t dsicm_hbm_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	static const u8 hbm[] = { 0, 1, 3 }, acl[] = { 0x00, 0x10, 0x11 };
	unsigned int t;
	int r;

	r = kstrtouint(buf, 10, &t);
	if (r)
		return r;
	if (t >= ARRAY_SIZE(hbm))
		return -EINVAL;

	mutex_lock(&ddata->lock);
	ddata->hbm = hbm[t];
	ddata->acl = acl[t];
	r = dsicm_apply(ddata, dsicm_write_hbm);
	mutex_unlock(&ddata->lock);

	return r ? r : count;
}

static ssize_t dsicm_hbm_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", min_t(u8, ddata->hbm, 2));
}

static ssize_t dsicm_store_ulps(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	struct omap_dss_device *in = ddata->in;
	unsigned long t;
	int r;

	r = kstrtoul(buf, 0, &t);
	if (r)
		return r;

	mutex_lock(&ddata->lock);

	if (ddata->enabled) {
		in->ops.dsi->bus_lock(in);

		if (t)
			r = dsicm_enter_ulps(ddata);
		else
			r = dsicm_wake_up(ddata);

		in->ops.dsi->bus_unlock(in);
	}

	mutex_unlock(&ddata->lock);

	if (r)
		return r;

	return count;
}

static ssize_t dsicm_show_ulps(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	unsigned t;

	mutex_lock(&ddata->lock);
	t = ddata->ulps_enabled;
	mutex_unlock(&ddata->lock);

	return sysfs_emit(buf, "%u\n", t);
}

static ssize_t dsicm_store_ulps_timeout(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	struct omap_dss_device *in = ddata->in;
	unsigned long t;
	int r;

	r = kstrtoul(buf, 0, &t);
	if (r)
		return r;

	mutex_lock(&ddata->lock);
	ddata->ulps_timeout = t;

	if (ddata->enabled) {
		/* dsicm_wake_up will restart the timer */
		in->ops.dsi->bus_lock(in);
		r = dsicm_wake_up(ddata);
		in->ops.dsi->bus_unlock(in);
	}

	mutex_unlock(&ddata->lock);

	if (r)
		return r;

	return count;
}

static ssize_t dsicm_show_ulps_timeout(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	unsigned t;

	mutex_lock(&ddata->lock);
	t = ddata->ulps_timeout;
	mutex_unlock(&ddata->lock);

	return sysfs_emit(buf, "%u\n", t);
}

static ssize_t dsicm_split_timing_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = platform_get_drvdata(to_platform_device(dev));

	return sysfs_emit(buf, "te %u half1 %u max %u half2 %u max %u\n", ddata->frame_us,
			  ddata->split_us[0], ddata->split_max[0],
			  ddata->split_us[1], ddata->split_max[1]);
}

static ssize_t dsicm_split_right_first_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = platform_get_drvdata(to_platform_device(dev));

	return sysfs_emit(buf, "%d\n", ddata->split_right_first);
}

static ssize_t dsicm_split_right_first_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct panel_drv_data *ddata = platform_get_drvdata(to_platform_device(dev));
	bool v;
	int r = kstrtobool(buf, &v);

	if (r)
		return r;
	ddata->split_right_first = v;
	return count;
}

static ssize_t dsicm_split_mid_us_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = platform_get_drvdata(to_platform_device(dev));

	return sysfs_emit(buf, "%u\n", ddata->split_mid_us);
}

static ssize_t dsicm_split_mid_us_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct panel_drv_data *ddata = platform_get_drvdata(to_platform_device(dev));
	unsigned int v;
	int r = kstrtouint(buf, 0, &v);

	if (r)
		return r;
	if (v > 40000)
		return -EINVAL;
	ddata->split_mid_us = v;
	return count;
}

static int _dsicm_set_tear_scanline(struct panel_drv_data *ddata);

static ssize_t dsicm_split_reverse_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = platform_get_drvdata(to_platform_device(dev));

	return sysfs_emit(buf, "%d\n", ddata->split_reverse);
}

static ssize_t dsicm_split_reverse_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct panel_drv_data *ddata = platform_get_drvdata(to_platform_device(dev));
	struct omap_dss_device *in = ddata->in;
	bool v;
	int r = kstrtobool(buf, &v);

	if (r)
		return r;

	mutex_lock(&ddata->lock);
	ddata->split_reverse = v;
	if (ddata->enabled) {
		in->ops.dsi->bus_lock(in);
		r = dsicm_wake_up(ddata);
		if (!r && ddata->te_enabled)
			r = _dsicm_set_tear_scanline(ddata);
		in->ops.dsi->bus_unlock(in);
	}
	mutex_unlock(&ddata->lock);

	return r ? r : count;
}

static DEVICE_ATTR(num_dsi_errors, S_IRUGO, dsicm_num_errors_show, NULL);
static DEVICE_ATTR(split_reverse, S_IRUGO | S_IWUSR, dsicm_split_reverse_show,
		   dsicm_split_reverse_store);
static DEVICE_ATTR(split_right_first, S_IRUGO | S_IWUSR, dsicm_split_right_first_show,
		   dsicm_split_right_first_store);
static DEVICE_ATTR(split_mid_us, S_IRUGO | S_IWUSR, dsicm_split_mid_us_show,
		   dsicm_split_mid_us_store);
static DEVICE_ATTR(split_timing, S_IRUGO, dsicm_split_timing_show, NULL);
static DEVICE_ATTR(hw_revision, S_IRUGO, dsicm_hw_revision_show, NULL);
static DEVICE_ATTR(ulps, S_IRUGO | S_IWUSR,
		dsicm_show_ulps, dsicm_store_ulps);
static DEVICE_ATTR(ulps_timeout, S_IRUGO | S_IWUSR,
		dsicm_show_ulps_timeout, dsicm_store_ulps_timeout);

static void dsicm_update_stats(struct panel_drv_data *ddata)
{
	ktime_t now = ktime_get();
	s64 us = ktime_us_delta(now, ddata->stats_last);

	ddata->stats_last = now;
	if (us > 100000)
		ddata->stats[0]++;
	else
		ddata->stats[clamp_t(u32, DIV_ROUND_CLOSEST((u32)us, ddata->frame_us ?: 16667), 1, 4)]++;
}

static ssize_t dsicm_update_stats_show(struct device *dev, struct device_attribute *attr,
				       char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	u32 *n = ddata->stats;

	u32 *p = ddata->phase;

	return sysfs_emit(buf, "1:%u 2:%u 3:%u 4+:%u pauses:%u | requests at TE+ 0-1ms:%u 1-2ms:%u "
			  "2-15ms:%u 15ms+:%u | late starts:%u | early pans:%u (%s) blocked:%u (%u ms) | "
			  "halves late: 1st over %u, 2nd missed %u (out late %u), 2nd over %u\n",
			  n[1], n[2], n[3], n[4], n[0], p[0], p[1], p[2], p[3], ddata->late_starts,
			  ddata->early_pans - ddata->early_pans_base,
			  ddata->region_latch ? "kept" : "replaced", ddata->blocked, ddata->blocked_ms,
			  ddata->half1_over, ddata->half2_missed, ddata->half2_late, ddata->half2_over);
}

/* Any write clears them */
static ssize_t dsicm_update_stats_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);

	mutex_lock(&ddata->lock);
	memset(ddata->stats, 0, sizeof(ddata->stats));
	memset(ddata->phase, 0, sizeof(ddata->phase));
	ddata->late_starts = 0;
	ddata->early_pans_base = ddata->early_pans;
	ddata->blocked = ddata->blocked_ms = 0;
	ddata->half1_over = ddata->half2_missed = ddata->half2_late = ddata->half2_over = 0;
	mutex_unlock(&ddata->lock);
	return count;
}

static ssize_t dsicm_idle_latency_us_show(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", ddata->idle_latency_us);
}

static ssize_t dsicm_idle_latency_us_store(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	u32 t;
	int r;

	r = kstrtou32(buf, 0, &t);
	if (r)
		return r;

	mutex_lock(&ddata->lock);
	ddata->idle_latency_us = t;
	dsicm_set_idle_qos(ddata);
	mutex_unlock(&ddata->lock);
	return count;
}

static ssize_t dsicm_split_late_us_show(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", ddata->split_late_us);
}

static ssize_t dsicm_split_late_us_store(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	int r = kstrtou32(buf, 0, &ddata->split_late_us);

	return r ? r : count;
}

static ssize_t dsicm_l3_lpm_hold_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", ddata->l3_lpm_hold);
}

static ssize_t dsicm_l3_lpm_hold_store(struct device *dev, struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	bool t;
	int r = kstrtobool(buf, &t);

	if (r)
		return r;
	mutex_lock(&ddata->lock);
	ddata->l3_lpm_hold = t;
	dsicm_set_l3_hold(ddata, ddata->enabled && !ddata->ulps_enabled);
	mutex_unlock(&ddata->lock);
	return count;
}

static ssize_t dsicm_region_latch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", ddata->region_latch);
}

static ssize_t dsicm_region_latch_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	int r = kstrtobool(buf, &ddata->region_latch);

	return r ? r : count;
}

static DEVICE_ATTR(split_late_us, S_IRUGO | S_IWUSR, dsicm_split_late_us_show,
		   dsicm_split_late_us_store);
static DEVICE_ATTR(update_stats, S_IRUGO | S_IWUSR, dsicm_update_stats_show,
		   dsicm_update_stats_store);
static DEVICE_ATTR(idle_latency_us, S_IRUGO | S_IWUSR, dsicm_idle_latency_us_show,
		   dsicm_idle_latency_us_store);
static DEVICE_ATTR(l3_lpm_hold, S_IRUGO | S_IWUSR, dsicm_l3_lpm_hold_show,
		   dsicm_l3_lpm_hold_store);
static DEVICE_ATTR(region_latch, S_IRUGO | S_IWUSR, dsicm_region_latch_show,
		   dsicm_region_latch_store);

static struct attribute *dsicm_attrs[] = {
	&dev_attr_update_stats.attr,
	&dev_attr_idle_latency_us.attr,
	&dev_attr_l3_lpm_hold.attr,
	&dev_attr_region_latch.attr,
	&dev_attr_split_late_us.attr,
	&dev_attr_num_dsi_errors.attr,
	&dev_attr_hw_revision.attr,
	&dev_attr_ulps.attr,
	&dev_attr_ulps_timeout.attr,
	&dev_attr_split_timing.attr,
	&dev_attr_split_right_first.attr,
	&dev_attr_split_mid_us.attr,
	&dev_attr_split_reverse.attr,
	NULL,
};

static const struct attribute_group dsicm_attr_group = {
	.attrs = dsicm_attrs,
};

static DEVICE_ATTR(lpm, S_IRUGO | S_IWUSR, dsicm_lpm_show, dsicm_lpm_store);
static DEVICE_ATTR(dimming, S_IRUGO | S_IWUSR, dsicm_dimming_show, dsicm_dimming_store);
static DEVICE_ATTR(hbm, S_IRUGO | S_IWUSR, dsicm_hbm_show, dsicm_hbm_store);

/* The N9's panel: mce finds these at /sys/class/backlight/display0/device/ */
static struct attribute *dsicm_pyrenees_attrs[] = {
	&dev_attr_lpm.attr,
	&dev_attr_dimming.attr,
	&dev_attr_hbm.attr,
	NULL,
};

static const struct attribute_group dsicm_pyrenees_attr_group = {
	.attrs = dsicm_pyrenees_attrs,
};

static void dsicm_hw_reset(struct panel_drv_data *ddata)
{
	/*
	 * Note that we appear to activate the reset line here. However
	 * existing DTSes specified incorrect polarity for it (active high),
	 * so in fact this deasserts the reset line.
	 */
	gpiod_set_value_cansleep(ddata->reset_gpio, 1);
	udelay(10);
	/* reset the panel */
	gpiod_set_value_cansleep(ddata->reset_gpio, 0);
	/* keep reset asserted */
	udelay(10);
	/* release reset line */
	gpiod_set_value_cansleep(ddata->reset_gpio, 1);
	/* wait after releasing reset */
	usleep_range(5000, 10000);
}

static int dsicm_power_on(struct panel_drv_data *ddata)
{
	struct omap_dss_device *in = ddata->in;
	int r;
	bool skip_init = omapdss_def_disp_enabled();
	struct omap_dss_dsi_config dsi_config = {
		.mode = OMAP_DSS_DSI_CMD_MODE,
		.pixel_format = OMAP_DSS_DSI_FMT_RGB888,
		.timings = &ddata->timings,
		.hs_clk_min = 150000000,
		.hs_clk_max = 300000000,
		.lp_clk_min = 7000000,
		.lp_clk_max = 10000000,
	};

	if (ddata->pin_config.num_pins > 0) {
		r = in->ops.dsi->configure_pins(in, &ddata->pin_config);
		if (r) {
			dev_err(&ddata->pdev->dev,
				"failed to configure DSI pins\n");
			goto err0;
		}
	}

	r = in->ops.dsi->set_config(in, &dsi_config);
	if (r) {
		dev_err(&ddata->pdev->dev, "failed to configure DSI\n");
		goto err0;
	}

	dsicm_set_l3_hold(ddata, true);
	r = in->ops.dsi->enable(in);
	if (r) {
		dev_err(&ddata->pdev->dev, "failed to enable DSI\n");
		goto err0;
	}

	/* sgx: as stock's skip_first_init: a reset would lose NOLO's picture */
	if (!skip_init)
		dsicm_hw_reset(ddata);

	in->ops.dsi->enable_hs(in, ddata->channel, false);

	/*
	 * sgx: NOLO's panel is kept if it still holds its settings: a reset (or power loss) clears
	 * the brightness, a sleep in keeps it (and the picture). Asleep (power mode without sleep out
	 * 0x10 / display on 0x04) it's woken below (sleep out, display on), as stock.
	 */
	if (skip_init) {
		u8 pm = 0, bl = 0;

		dsicm_dcs_read_1(ddata, MIPI_DCS_GET_POWER_MODE, &pm);
		dsicm_dcs_read_1(ddata, DCS_READ_BRIGHTNESS, &bl);
		if (!bl) {
			dev_info(&ddata->pdev->dev, "NOLO's display is gone (power mode %02x)\n", pm);
			skip_init = false;
			omapdss_def_disp_done();
			dsicm_hw_reset(ddata);
		} else {
			dev_info(&ddata->pdev->dev, "keeping NOLO's display (power mode %02x, brightness %u)\n",
				 pm, bl);
		}
	}

	r = dsicm_sleep_out(ddata);
	if (r)
		goto err;

	r = dsicm_get_id(ddata, &ddata->id[0], &ddata->id[1], &ddata->id[2]);
	if (r)
		goto err;

	if (ddata->address_mode) {
		r = dsicm_dcs_write_1(ddata, MIPI_DCS_SET_ADDRESS_MODE, ddata->address_mode);
		if (r)
			goto err;
	}

	/* Show only the visible rows, as the N9's stock driver does: the rest of the panel's frame is
	 * never written, and would show as garbage */
	if (ddata->partial_rows) {
		u16 s = ddata->partial_start, e = s + ddata->partial_rows - 1;
		u8 buf[5] = { MIPI_DCS_SET_PARTIAL_ROWS, s >> 8, s & 0xff, e >> 8, e & 0xff };

		r = in->ops.dsi->dcs_write(in, ddata->channel, buf, sizeof(buf));
		if (r)
			goto err;
		r = dsicm_dcs_write_0(ddata, MIPI_DCS_ENTER_PARTIAL_MODE);
		if (r)
			goto err;
	}

	/* sgx: NOLO's brightness kept, as stock */
	if (skip_init && ddata->bldev) {
		u8 b;

		if (!dsicm_dcs_read_1(ddata, DCS_READ_BRIGHTNESS, &b))
			ddata->bldev->props.brightness = b;
	}

	/* The backlight's brightness, as stock */
	r = dsicm_dcs_write_1(ddata, DCS_BRIGHTNESS,
			      ddata->bldev ? ddata->bldev->props.brightness : 0xff);
	if (r)
		goto err;

	r = dsicm_write_ctrl_display(ddata);
	if (r)
		goto err;

	if (ddata->pyrenees) {
		r = dsicm_dcs_write_1(ddata, DCS_WRITE_CABC, ddata->acl);
		if (!r)
			r = dsicm_write_lpm(ddata);
		if (r)
			goto err;
	}

	r = dsicm_dcs_write_1(ddata, MIPI_DCS_SET_PIXEL_FORMAT,
		MIPI_DCS_PIXEL_FMT_24BIT);
	if (r)
		goto err;

	r = dsicm_dcs_write_0(ddata, MIPI_DCS_SET_DISPLAY_ON);
	if (r)
		goto err;

	r = _dsicm_enable_te(ddata, ddata->te_enabled);
	if (r)
		goto err;

	r = in->ops.dsi->enable_video_output(in, ddata->channel);
	if (r)
		goto err;

	ddata->enabled = 1;
	dsicm_set_idle_qos(ddata);

	if (!ddata->intro_printed) {
		dev_info(&ddata->pdev->dev, "panel revision %02x.%02x.%02x\n",
			ddata->id[0], ddata->id[1], ddata->id[2]);
		ddata->intro_printed = true;
	}

	in->ops.dsi->enable_hs(in, ddata->channel, true);

	return 0;
err:
	dev_err(&ddata->pdev->dev, "error while enabling panel, issuing HW reset\n");

	dsicm_hw_reset(ddata);

	in->ops.dsi->disable(in, true, false);
err0:
	dsicm_set_l3_hold(ddata, false);
	/* sgx: NOLO's panel didn't answer: a full power on, as stock */
	if (skip_init) {
		omapdss_def_disp_done();
		dev_err(&ddata->pdev->dev, "retrying panel power on\n");
		r = dsicm_power_on(ddata);
	}
	return r;
}

static void dsicm_power_off(struct panel_drv_data *ddata)
{
	struct omap_dss_device *in = ddata->in;
	int r;

	in->ops.dsi->disable_video_output(in, ddata->channel);

	r = dsicm_dcs_write_0(ddata, MIPI_DCS_SET_DISPLAY_OFF);
	if (!r)
		r = dsicm_sleep_in(ddata);

	if (r) {
		dev_err(&ddata->pdev->dev,
				"error disabling panel, issuing HW reset\n");
		dsicm_hw_reset(ddata);
	}

	in->ops.dsi->disable(in, true, false);

	ddata->enabled = 0;
	dsicm_set_idle_qos(ddata);
	dsicm_set_l3_hold(ddata, false);
}

static int dsicm_panel_reset(struct panel_drv_data *ddata)
{
	dev_err(&ddata->pdev->dev, "performing LCD reset\n");

	dsicm_power_off(ddata);
	dsicm_hw_reset(ddata);
	return dsicm_power_on(ddata);
}

static int dsicm_connect(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	struct device *dev = &ddata->pdev->dev;
	int r;

	if (omapdss_device_is_connected(dssdev))
		return 0;

	r = in->ops.dsi->connect(in, dssdev);
	if (r) {
		dev_err(dev, "Failed to connect to video source\n");
		return r;
	}

	r = in->ops.dsi->request_vc(ddata->in, &ddata->channel);
	if (r) {
		dev_err(dev, "failed to get virtual channel\n");
		goto err_req_vc;
	}

	r = in->ops.dsi->set_vc_id(ddata->in, ddata->channel, TCH);
	if (r) {
		dev_err(dev, "failed to set VC_ID\n");
		goto err_vc_id;
	}

	return 0;

err_vc_id:
	in->ops.dsi->release_vc(ddata->in, ddata->channel);
err_req_vc:
	in->ops.dsi->disconnect(in, dssdev);
	return r;
}

static void dsicm_disconnect(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	if (!omapdss_device_is_connected(dssdev))
		return;

	in->ops.dsi->release_vc(in, ddata->channel);
	in->ops.dsi->disconnect(in, dssdev);
}

static int dsicm_enable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;

	dev_dbg(&ddata->pdev->dev, "enable\n");

	mutex_lock(&ddata->lock);

	if (!omapdss_device_is_connected(dssdev)) {
		r = -ENODEV;
		goto err;
	}

	if (omapdss_device_is_enabled(dssdev)) {
		r = 0;
		goto err;
	}

	in->ops.dsi->bus_lock(in);

	r = dsicm_power_on(ddata);

	in->ops.dsi->bus_unlock(in);

	if (r)
		goto err;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	mutex_unlock(&ddata->lock);

	return 0;
err:
	dev_dbg(&ddata->pdev->dev, "enable failed\n");
	mutex_unlock(&ddata->lock);
	return r;
}

static void dsicm_disable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;

	dev_dbg(&ddata->pdev->dev, "disable\n");

	mutex_lock(&ddata->lock);

	dsicm_cancel_ulps_work(ddata);

	in->ops.dsi->bus_lock(in);

	if (omapdss_device_is_enabled(dssdev)) {
		r = dsicm_wake_up(ddata);
		if (!r)
			dsicm_power_off(ddata);
	}

	in->ops.dsi->bus_unlock(in);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

	mutex_unlock(&ddata->lock);
}

static void dsicm_framedone_cb(int err, void *data)
{
	struct panel_drv_data *ddata = data;
	struct omap_dss_device *in = ddata->in;

	dev_dbg(&ddata->pdev->dev, "framedone, err %d\n", err);
	in->ops.dsi->bus_unlock(ddata->in);
}

enum { DSICM_SPLIT_NONE, DSICM_HALF1_WAIT, DSICM_HALF1_ONGOING, DSICM_HALF2_WAIT,
	DSICM_HALF2_ONGOING };

static bool dsicm_split(struct panel_drv_data *ddata)
{
	return ddata->te_enabled && ddata->ext_te_gpio && (ddata->rotate & 1);
}

/*
 * The halves: in landscape each update is written across the panel's columns while it scans its
 * rows, so a whole frame always crosses the scan (a diagonal tear). The N9's stock driver
 * (panel-nokia-dsi, PND_UPD_MODE_LR/RL) writes each half of the landscape frame while the scan is in
 * the panel's other half: the first half from mid-frame, the second at the next TE. The mid-frame
 * timer runs from every TE, so an update asked for before mid-frame goes out in that frame.
 * Rotate 1 (address mode MV|MX): the left half first; rotate 3: the right half.
 */
static void dsicm_split_half(struct panel_drv_data *ddata, bool first, u16 *x, u16 *w)
{
	u16 width = ddata->timings.x_res, half = (width / 2) & ~1;

	if (first == ((ddata->rotate == 1) != ddata->split_right_first)) {
		*x = 0;
		*w = half;
	} else {
		*x = half;
		*w = width - half;
	}
}

static void dsicm_split_fail(struct panel_drv_data *ddata)
{
	dev_err(&ddata->pdev->dev, "split update failed\n");
	atomic_set(&ddata->split_state, DSICM_SPLIT_NONE);
	cancel_delayed_work(&ddata->te_timeout_work);
	ddata->in->ops.dsi->latch_region(ddata->in, DSS_REGION_RELEASE);
	ddata->in->ops.dsi->bus_unlock(ddata->in);
}

static void dsicm_split_record(struct panel_drv_data *ddata, int half)
{
	u32 us = ktime_us_delta(ktime_get(), ddata->split_t0);

	ddata->split_us[half] = us;
	if (half == 0 && ktime_us_delta(ktime_get(), ddata->te_last) > ddata->frame_us / 2)
		ddata->half1_over++;
	if (half == 1 && ktime_after(ddata->te_last, ddata->split_t0))
		ddata->half2_over++;
	/* a peak that fades by 1/32 a frame: one slow half (at boot) mustn't stop late starts for good */
	ddata->split_max[half] = max(ddata->split_max[half] - ddata->split_max[half] / 32, us);
}

static void dsicm_split_done2(int err, void *data)
{
	struct panel_drv_data *ddata = data;

	if (err) {
		dsicm_split_fail(ddata);
		return;
	}
	dsicm_split_record(ddata, 1);
	atomic_set(&ddata->split_state, DSICM_SPLIT_NONE);
	cancel_delayed_work(&ddata->te_timeout_work);
	ddata->in->ops.dsi->bus_unlock(ddata->in);
}

static void dsicm_split_late_half2(struct panel_drv_data *ddata);

/* After the first half: the second half's window (a DCS write, which can sleep), then its TE */
static void dsicm_split_work(struct work_struct *work)
{
	struct panel_drv_data *ddata = container_of(work, struct panel_drv_data, split_work);
	u16 x, w;

	dsicm_split_half(ddata, false, &x, &w);
	if (dsicm_set_update_window(ddata, x, 0, w, ddata->timings.y_res)) {
		dsicm_split_fail(ddata);
		return;
	}
	atomic_set(&ddata->split_state, DSICM_HALF2_WAIT);
	dsicm_split_late_half2(ddata);
}

static void dsicm_split_done1(int err, void *data)
{
	struct panel_drv_data *ddata = data;

	if (err) {
		dsicm_split_fail(ddata);
		return;
	}
	dsicm_split_record(ddata, 0);
	/* high priority: late, the second half misses its TE and the frame takes one more */
	queue_work(system_highpri_wq, &ddata->split_work);
}

static void dsicm_split_start1(struct panel_drv_data *ddata)
{
	struct omap_dss_device *in = ddata->in;
	u16 x, w;

	if (atomic_cmpxchg(&ddata->split_state, DSICM_HALF1_WAIT, DSICM_HALF1_ONGOING) ==
	    DSICM_HALF1_WAIT) {
		dsicm_split_half(ddata, true, &x, &w);
		ddata->split_t0 = ktime_get();
		dsicm_update_stats(ddata);
		if (in->ops.dsi->update_region(in, ddata->channel, x, w, true,
					       dsicm_split_done1, ddata))
			dsicm_split_fail(ddata);
	}
}

static void dsicm_split_start2(struct panel_drv_data *ddata)
{
	struct omap_dss_device *in = ddata->in;
	u16 x, w;

	if (atomic_cmpxchg(&ddata->split_state, DSICM_HALF2_WAIT, DSICM_HALF2_ONGOING) ==
	    DSICM_HALF2_WAIT) {
		dsicm_split_half(ddata, false, &x, &w);
		ddata->split_t0 = ktime_get();
		if (in->ops.dsi->update_region(in, ddata->channel, x, w, false,
					       dsicm_split_done2, ddata))
			dsicm_split_fail(ddata);
	}
}

/* The TE pulse at mid-scan in reverse mode (stock's for the SMD Pyrenees), else at the frame start */
static int _dsicm_set_tear_scanline(struct panel_drv_data *ddata)
{
	struct omap_dss_device *in = ddata->in;
	u16 line = ddata->split_reverse && ddata->frame_rows ? ddata->frame_rows / 2 : 0;
	u8 buf[3] = { MIPI_DCS_SET_TEAR_SCANLINE, line >> 8, line & 0xff };

	return in->ops.dsi->dcs_write(in, ddata->channel, buf, sizeof(buf));
}

/*
 * The timer: half the TE period; in reverse mode, just past it (52%: the scan has left the second
 * half's rows), so the second half is out before the next TE and updates keep up with 60 frames a
 * second. It follows the measured TE period, which the SMD Pyrenees stretches in ACL. Stock used a
 * fixed (1/46 s less the blanking) / 2, which ends a 60 Hz update just after the next TE (updates
 * at 30 a second). sysfs split_mid_us overrides it (0.87 of a half frame measured tear-free).
 */
static u32 dsicm_split_delay(struct panel_drv_data *ddata)
{
	if (ddata->split_mid_us)
		return ddata->split_mid_us;
	return ddata->split_reverse ? ddata->frame_us * 52 / 100 : ddata->frame_us / 2;
}

/*
 * An update asked for just after TE: in reverse mode its first half still fits before the scan comes
 * back to the half's rows (TE is at mid-scan, the half takes about 6 of the 8.3 ms), so it starts now
 * rather than at the next TE, which would show the previous frame twice. The second half's timer
 * already runs from that TE. Up to split_late_us after TE (sysfs, 0 = off), and only while the
 * longest first half so far still ends 0.3 ms before the scan's return.
 */
static void dsicm_split_late_start(struct panel_drv_data *ddata)
{
	u32 us = ktime_us_delta(ktime_get(), ddata->te_last);
	u32 half = ddata->split_max[0];

	if (!ddata->split_reverse || !half || us >= ddata->split_late_us ||
	    us + half + 300 >= ddata->frame_us / 2)
		return;
	ddata->late_starts++;
	dsicm_split_start1(ddata);
}

/*
 * The second half's window written only after its timer fired (a first half started late, or the
 * window's DCS write ran late): out now if it still ends before the next TE, where the scan comes
 * back to its rows, rather than at the next frame's timer with the halves a frame apart (the "half
 * screen tearing" seen with video, update_stats "2nd missed"). The same margin as a late start.
 */
static void dsicm_split_late_half2(struct panel_drv_data *ddata)
{
	u32 us = ktime_us_delta(ktime_get(), ddata->te_last);

	if (!ddata->split_reverse || us < dsicm_split_delay(ddata) ||
	    us + ddata->split_max[1] + 300 >= ddata->frame_us)
		return;
	ddata->half2_late++;
	dsicm_split_start2(ddata);
}

static enum hrtimer_restart dsicm_split_timer(struct hrtimer *timer)
{
	struct panel_drv_data *ddata = container_of(timer, struct panel_drv_data, split_timer);

	if (ddata->split_reverse) {
		if (atomic_read(&ddata->split_state) == DSICM_HALF1_ONGOING)
			ddata->half2_missed++;
		dsicm_split_start2(ddata);
	} else {
		dsicm_split_start1(ddata);
	}
	return HRTIMER_NORESTART;
}

static irqreturn_t dsicm_te_isr(int irq, void *data)
{
	struct panel_drv_data *ddata = data;
	struct omap_dss_device *in = ddata->in;
	ktime_t now = ktime_get();
	s64 frame = ktime_us_delta(now, ddata->te_last);
	int old;
	int r;

	/* the frame period, from the TE line (longer in low power mode) */
	ddata->te_last = now;
	if (frame > 10000 && frame < (ddata->lpm ? 100000 : 40000))
		ddata->frame_us = frame;

	if (dsicm_split(ddata)) {
		hrtimer_start(&ddata->split_timer, us_to_ktime(dsicm_split_delay(ddata)),
			      HRTIMER_MODE_REL);
		if (ddata->split_reverse)
			dsicm_split_start1(ddata);
		else
			dsicm_split_start2(ddata);
	}

	old = atomic_cmpxchg(&ddata->do_update, 1, 0);

	if (old) {
		cancel_delayed_work(&ddata->te_timeout_work);

		r = in->ops.dsi->update(in, ddata->channel, dsicm_framedone_cb,
				ddata);
		if (r)
			goto err;
	}

	return IRQ_HANDLED;
err:
	dev_err(&ddata->pdev->dev, "start update failed\n");
	in->ops.dsi->bus_unlock(in);
	return IRQ_HANDLED;
}

static void dsicm_te_timeout_work_callback(struct work_struct *work)
{
	struct panel_drv_data *ddata = container_of(work, struct panel_drv_data,
					te_timeout_work.work);
	struct omap_dss_device *in = ddata->in;

	dev_err(&ddata->pdev->dev, "TE not received for 250ms!\n");

	atomic_set(&ddata->do_update, 0);
	if (atomic_xchg(&ddata->split_state, DSICM_SPLIT_NONE) != DSICM_SPLIT_NONE)
		in->ops.dsi->latch_region(in, DSS_REGION_RELEASE);
	in->ops.dsi->bus_unlock(in);
}

static int dsicm_update(struct omap_dss_device *dssdev,
				    u16 x, u16 y, u16 w, u16 h)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	ktime_t t0;
	u32 waited;
	int r;

	dev_dbg(&ddata->pdev->dev, "update %d, %d, %d x %d\n", x, y, w, h);

	mutex_lock(&ddata->lock);
	if (ddata->lpm_pending)
		mod_delayed_work(system_wq, &ddata->lpm_work, msecs_to_jiffies(DSICM_LPM_DELAY_MS));
	t0 = ktime_get();
	in->ops.dsi->bus_lock(in);
	waited = ktime_us_delta(ktime_get(), t0);
	if (waited > 1000) {
		ddata->blocked++;
		ddata->blocked_ms += waited / 1000;
	}

	r = dsicm_wake_up(ddata);
	if (r)
		goto err;

	if (!ddata->enabled) {
		r = 0;
		goto err;
	}

	if (dsicm_split(ddata)) {
		u16 hx, hw;
		u32 us = ktime_us_delta(ktime_get(), ddata->te_last);

		ddata->phase[us < 1000 ? 0 : us < 2000 ? 1 : us < 15000 ? 2 : 3]++;
		dsicm_split_half(ddata, true, &hx, &hw);
		r = dsicm_set_update_window(ddata, hx, 0, hw, ddata->timings.y_res);
		if (r)
			goto err;
		ddata->early_pans = in->ops.dsi->latch_region(in, ddata->region_latch ?
							      DSS_REGION_LATCH : DSS_REGION_NOTE);
		schedule_delayed_work(&ddata->te_timeout_work, msecs_to_jiffies(250));
		atomic_set(&ddata->split_state, DSICM_HALF1_WAIT);
		dsicm_split_late_start(ddata);
		/* note: no bus_unlock here. unlock is in dsicm_split_done2 */
		mutex_unlock(&ddata->lock);
		return 0;
	}

	dsicm_update_stats(ddata);

	/* XXX no need to send this every frame, but dsi break if not done */
	r = dsicm_set_update_window(ddata, 0, 0,
			dssdev->panel.timings.x_res,
			dssdev->panel.timings.y_res);
	if (r)
		goto err;

	if (ddata->te_enabled && ddata->ext_te_gpio) {
		schedule_delayed_work(&ddata->te_timeout_work,
				msecs_to_jiffies(250));
		atomic_set(&ddata->do_update, 1);
	} else {
		r = in->ops.dsi->update(in, ddata->channel, dsicm_framedone_cb,
				ddata);
		if (r)
			goto err;
	}

	/* note: no bus_unlock here. unlock is in framedone_cb */
	mutex_unlock(&ddata->lock);
	return 0;
err:
	in->ops.dsi->bus_unlock(in);
	mutex_unlock(&ddata->lock);
	return r;
}

static int dsicm_sync(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	dev_dbg(&ddata->pdev->dev, "sync\n");

	mutex_lock(&ddata->lock);
	in->ops.dsi->bus_lock(in);
	in->ops.dsi->bus_unlock(in);
	mutex_unlock(&ddata->lock);

	dev_dbg(&ddata->pdev->dev, "sync done\n");

	return 0;
}

static int _dsicm_enable_te(struct panel_drv_data *ddata, bool enable)
{
	struct omap_dss_device *in = ddata->in;
	int r;

	if (enable)
		r = dsicm_dcs_write_1(ddata, MIPI_DCS_SET_TEAR_ON, 0);
	else
		r = dsicm_dcs_write_0(ddata, MIPI_DCS_SET_TEAR_OFF);

	/* landscape updates go in halves, timed from the TE line (see dsicm_split_half) */
	if (!r && enable && (ddata->rotate & 1))
		r = _dsicm_set_tear_scanline(ddata);

	if (!ddata->ext_te_gpio)
		in->ops.dsi->enable_te(in, enable);

	/* possible panel bug */
	msleep(100);

	return r;
}

static int dsicm_enable_te(struct omap_dss_device *dssdev, bool enable)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;

	mutex_lock(&ddata->lock);

	if (ddata->te_enabled == enable)
		goto end;

	in->ops.dsi->bus_lock(in);

	if (ddata->enabled) {
		r = dsicm_wake_up(ddata);
		if (r)
			goto err;

		r = _dsicm_enable_te(ddata, enable);
		if (r)
			goto err;
	}

	ddata->te_enabled = enable;

	in->ops.dsi->bus_unlock(in);
end:
	mutex_unlock(&ddata->lock);

	return 0;
err:
	in->ops.dsi->bus_unlock(in);
	mutex_unlock(&ddata->lock);

	return r;
}

static int dsicm_get_te(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	int r;

	mutex_lock(&ddata->lock);
	r = ddata->te_enabled;
	mutex_unlock(&ddata->lock);

	return r;
}

static int dsicm_memory_read(struct omap_dss_device *dssdev,
		void *buf, size_t size,
		u16 x, u16 y, u16 w, u16 h)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;
	int first = 1;
	int plen;
	unsigned buf_used = 0;

	if (size < w * h * 3)
		return -ENOMEM;

	mutex_lock(&ddata->lock);

	if (!ddata->enabled) {
		r = -ENODEV;
		goto err1;
	}

	size = min(w * h * 3,
			dssdev->panel.timings.x_res *
			dssdev->panel.timings.y_res * 3);

	in->ops.dsi->bus_lock(in);

	r = dsicm_wake_up(ddata);
	if (r)
		goto err2;

	/* plen 1 or 2 goes into short packet. until checksum error is fixed,
	 * use short packets. plen 32 works, but bigger packets seem to cause
	 * an error. */
	if (size % 2)
		plen = 1;
	else
		plen = 2;

	dsicm_set_update_window(ddata, x, y, w, h);

	r = in->ops.dsi->set_max_rx_packet_size(in, ddata->channel, plen);
	if (r)
		goto err2;

	while (buf_used < size) {
		u8 dcs_cmd = first ? 0x2e : 0x3e;
		first = 0;

		r = in->ops.dsi->dcs_read(in, ddata->channel, dcs_cmd,
				buf + buf_used, size - buf_used);

		if (r < 0) {
			dev_err(dssdev->dev, "read error\n");
			goto err3;
		}

		buf_used += r;

		if (r < plen) {
			dev_err(&ddata->pdev->dev, "short read\n");
			break;
		}

		if (signal_pending(current)) {
			dev_err(&ddata->pdev->dev, "signal pending, "
					"aborting memory read\n");
			r = -ERESTARTSYS;
			goto err3;
		}
	}

	r = buf_used;

err3:
	in->ops.dsi->set_max_rx_packet_size(in, ddata->channel, 1);
err2:
	in->ops.dsi->bus_unlock(in);
err1:
	mutex_unlock(&ddata->lock);
	return r;
}

static void dsicm_ulps_work(struct work_struct *work)
{
	struct panel_drv_data *ddata = container_of(work, struct panel_drv_data,
			ulps_work.work);
	struct omap_dss_device *dssdev = &ddata->dssdev;
	struct omap_dss_device *in = ddata->in;

	mutex_lock(&ddata->lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE || !ddata->enabled) {
		mutex_unlock(&ddata->lock);
		return;
	}

	in->ops.dsi->bus_lock(in);

	dsicm_enter_ulps(ddata);

	in->ops.dsi->bus_unlock(in);
	mutex_unlock(&ddata->lock);
}

/* rotate and mirror in sysfs, which Harmattan's X driver reads: the DT's, not changeable */
static u8 dsicm_get_rotate(struct omap_dss_device *dssdev)
{
	return to_panel_data(dssdev)->rotate;
}

static int dsicm_set_rotate(struct omap_dss_device *dssdev, u8 rotate)
{
	return rotate == to_panel_data(dssdev)->rotate ? 0 : -EINVAL;
}

/* The timings as the panel's frame is (portrait), before rotate, as Nokia's driver reports them:
 * Harmattan's X driver applies rotate to them itself */
static void dsicm_get_timings(struct omap_dss_device *dssdev, struct omap_video_timings *t)
{
	*t = dssdev->panel.timings;
	if (to_panel_data(dssdev)->rotate & 1)
		swap(t->x_res, t->y_res);
}

static bool dsicm_get_mirror(struct omap_dss_device *dssdev)
{
	return false;
}

static int dsicm_set_mirror(struct omap_dss_device *dssdev, bool enable)
{
	return enable ? -EINVAL : 0;
}

/*
 * sgx: the N9's framebuffer is RGB565 from boot, as stock's (board-rm696-video.c): preinit's
 * show_png draws its boot image only on a 16-bit framebuffer, and at the 32 bits omapfb chose
 * for a 24-bit panel the screen kept NOLO's logo until X's first frame, then stayed black until
 * X's first picture (boots 172-176). The DSS converts to the panel's 24 bits.
 */
static int dsicm_get_recommended_bpp(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);

	return ddata->pyrenees ? 16 : omapdss_default_get_recommended_bpp(dssdev);
}

static struct omap_dss_driver dsicm_ops = {
	.connect	= dsicm_connect,
	.disconnect	= dsicm_disconnect,

	.enable		= dsicm_enable,
	.disable	= dsicm_disable,

	.update		= dsicm_update,
	.sync		= dsicm_sync,

	.get_resolution	= dsicm_get_resolution,
	.get_recommended_bpp = dsicm_get_recommended_bpp,

	.enable_te	= dsicm_enable_te,
	.get_te		= dsicm_get_te,

	.memory_read	= dsicm_memory_read,
	.get_timings	= dsicm_get_timings,
	.get_rotate	= dsicm_get_rotate,
	.set_rotate	= dsicm_set_rotate,
	.get_mirror	= dsicm_get_mirror,
	.set_mirror	= dsicm_set_mirror,
};

static int dsicm_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct panel_drv_data *ddata;
	struct backlight_device *bldev = NULL;
	struct device *dev = &pdev->dev;
	struct omap_dss_device *dssdev;
	struct display_timing dt;
	int r;

	dev_dbg(dev, "probe\n");

	if (!pdev->dev.of_node)
		return -ENODEV;

	ddata = devm_kzalloc(dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	platform_set_drvdata(pdev, ddata);
	ddata->pdev = pdev;

	ddata->in = omapdss_of_find_source_for_first_ep(pdev->dev.of_node);
	r = PTR_ERR_OR_ZERO(ddata->in);
	if (r) {
		dev_err(&pdev->dev, "failed to find video source: %d\n", r);
		return r;
	}

	/* The frame from the DT's panel-timing (the N9 and N950 panels are 480x864, portrait), else the
	 * Taal panel's 864x480 */
	if (of_get_display_timing(pdev->dev.of_node, "panel-timing", &dt) == 0) {
		u32 rotation = 0;

		ddata->timings.x_res = dt.hactive.typ;
		ddata->timings.y_res = dt.vactive.typ;
		ddata->timings.pixelclock = dt.pixelclock.typ;
		/* The back porches place the visible area in the panel's frame (the N9 shows 854 of its
		 * 864 rows); DT rotation 90 or 270 makes it landscape through the panel's row/column
		 * exchange, as the N9's stock driver does (its rotate 1) */
		ddata->x_offset = dt.hback_porch.typ;
		ddata->y_offset = dt.vback_porch.typ;
		ddata->frame_rows = dt.vback_porch.typ + dt.vactive.typ + dt.vfront_porch.typ;
		if (dt.vback_porch.typ || dt.vfront_porch.typ) {
			ddata->partial_start = dt.vback_porch.typ;
			ddata->partial_rows = dt.vactive.typ;
		}
		of_property_read_u32(pdev->dev.of_node, "rotation", &rotation);
		if (rotation == 90 || rotation == 270) {
			swap(ddata->timings.x_res, ddata->timings.y_res);
			swap(ddata->x_offset, ddata->y_offset);
			ddata->address_mode = rotation == 90 ? 0x60 : 0xa0;	/* MV|MX, MV|MY */
			ddata->rotate = rotation / 90;
		}
	} else {
		ddata->timings.x_res = 864;
		ddata->timings.y_res = 480;
	}
	/*
	 * DISPC's pixel clock, the DT's, else a frame per 60th of a second (DSI asks DISPC for 1.5x
	 * that, and sets its own clock to match, up to its 300 MHz maximum)
	 */
	if (!ddata->timings.pixelclock)
		ddata->timings.pixelclock = ddata->timings.x_res * ddata->timings.y_res * 60;

	dssdev = &ddata->dssdev;
	dssdev->dev = dev;
	dssdev->driver = &dsicm_ops;
	dssdev->panel.timings = ddata->timings;
	dssdev->type = OMAP_DISPLAY_TYPE_DSI;
	dssdev->owner = THIS_MODULE;

	dssdev->panel.dsi_pix_fmt = OMAP_DSS_DSI_FMT_RGB888;
	dssdev->caps = OMAP_DSS_DISPLAY_CAP_MANUAL_UPDATE |
		OMAP_DSS_DISPLAY_CAP_TEAR_ELIM;

	mutex_init(&ddata->lock);

	r = omapdss_register_display(dssdev);
	if (r) {
		dev_err(dev, "Failed to register panel\n");
		goto err_reg;
	}

	atomic_set(&ddata->do_update, 0);

	/* sgx: left released (see dsicm_hw_reset) if NOLO's panel is on, keeping its picture */
	ddata->reset_gpio = devm_gpiod_get(&pdev->dev, "reset",
			omapdss_def_disp_enabled() ? GPIOD_OUT_HIGH : GPIOD_OUT_LOW);
	r = PTR_ERR_OR_ZERO(ddata->reset_gpio);
	if (r) {
		dev_err(&pdev->dev, "Failed to request reset gpio: %d\n", r);
		return r;
	}

	gpiod_set_consumer_name(ddata->reset_gpio, "taal rst");

	ddata->ext_te_gpio = devm_gpiod_get_optional(&pdev->dev, "te",
						     GPIOD_IN);
	r = PTR_ERR_OR_ZERO(ddata->ext_te_gpio);
	if (r) {
		dev_err(&pdev->dev, "Failed to request TE gpio: %d\n", r);
		return r;
	}

	if (ddata->ext_te_gpio) {
		gpiod_set_consumer_name(ddata->ext_te_gpio, "taal irq");

		r = devm_request_irq(dev, gpiod_to_irq(ddata->ext_te_gpio),
				dsicm_te_isr,
				IRQF_TRIGGER_RISING,
				"taal vsync", ddata);

		if (r) {
			dev_err(dev, "IRQ request failed\n");
			return r;
		}

		INIT_DEFERRABLE_WORK(&ddata->te_timeout_work,
					dsicm_te_timeout_work_callback);
		INIT_WORK(&ddata->split_work, dsicm_split_work);
		hrtimer_setup(&ddata->split_timer, dsicm_split_timer, CLOCK_MONOTONIC,
			      HRTIMER_MODE_REL);
		ddata->frame_us = 16667;
		ddata->split_reverse = true;

		dev_dbg(dev, "Using GPIO TE\n");
	}

	INIT_DELAYED_WORK(&ddata->ulps_work, dsicm_ulps_work);
	/* As stock (board-rm696-video.c): the DSI link to ULPS 500 ms after the last update, so DSS
	 * idles while the picture is still (sysfs ulps_timeout, 0 = never) */
	ddata->ulps_timeout = 500;
	INIT_DELAYED_WORK(&ddata->lpm_work, dsicm_lpm_work);
	ddata->idle_latency_us = 6000;
	ddata->split_late_us = 2000;
	ddata->region_latch = true;

	/* sgx: not while NOLO's panel is kept (omapdss.def_disp_enabled), as stock's pnd_probe */
	if (!omapdss_def_disp_enabled())
		dsicm_hw_reset(ddata);

	/*
	 * Brightness by DCS unless the DT names a backlight. It registers under the panel's alias,
	 * display0 on the N9 as on stock, which Harmattan's mce looks for.
	 */
	ddata->use_dsi_backlight = !of_property_present(dev->of_node, "backlight");
	ddata->pyrenees = of_device_is_compatible(dev->of_node, "omapdss,nokia,pyrenees");

	if (ddata->use_dsi_backlight) {
		memset(&props, 0, sizeof(struct backlight_properties));
		props.max_brightness = 255;

		props.type = BACKLIGHT_RAW;
		bldev = backlight_device_register(dssdev->alias,
				dev, ddata, &dsicm_bl_ops, &props);
		if (IS_ERR(bldev)) {
			r = PTR_ERR(bldev);
			goto err_reg;
		}

		ddata->bldev = bldev;

		bldev->props.power = BACKLIGHT_POWER_ON;
		bldev->props.brightness = 255;

		dsicm_bl_update_status(bldev);
	}

	r = sysfs_create_group(&dev->kobj, &dsicm_attr_group);
	if (!r && ddata->pyrenees) {
		r = sysfs_create_group(&dev->kobj, &dsicm_pyrenees_attr_group);
		if (r)
			sysfs_remove_group(&dev->kobj, &dsicm_attr_group);
	}
	if (r) {
		dev_err(dev, "failed to create sysfs files\n");
		goto err_sysfs_create;
	}

	return 0;

err_sysfs_create:
	if (bldev != NULL)
		backlight_device_unregister(bldev);
err_reg:
	return r;
}

static void dsicm_remove(struct platform_device *pdev)
{
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct omap_dss_device *dssdev = &ddata->dssdev;
	struct backlight_device *bldev;

	dev_dbg(&pdev->dev, "remove\n");

	omapdss_unregister_display(dssdev);

	dsicm_disable(dssdev);
	dsicm_disconnect(dssdev);

	sysfs_remove_group(&pdev->dev.kobj, &dsicm_attr_group);
	if (ddata->pyrenees)
		sysfs_remove_group(&pdev->dev.kobj, &dsicm_pyrenees_attr_group);
	cancel_delayed_work_sync(&ddata->lpm_work);

	bldev = ddata->bldev;
	if (bldev != NULL) {
		bldev->props.power = BACKLIGHT_POWER_OFF;
		dsicm_bl_update_status(bldev);
		backlight_device_unregister(bldev);
	}

	omap_dss_put_device(ddata->in);

	dsicm_cancel_ulps_work(ddata);

	/* reset, to be sure that the panel is in a valid state */
	dsicm_hw_reset(ddata);
}

static const struct of_device_id dsicm_of_match[] = {
	{ .compatible = "omapdss,panel-dsi-cm", },
	{},
};

MODULE_DEVICE_TABLE(of, dsicm_of_match);

static struct platform_driver dsicm_driver = {
	.probe = dsicm_probe,
	.remove = dsicm_remove,
	.driver = {
		.name = "panel-dsi-cm",
		.of_match_table = dsicm_of_match,
	},
};

module_platform_driver(dsicm_driver);

MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@ti.com>");
MODULE_DESCRIPTION("Generic DSI Command Mode Panel Driver");
MODULE_LICENSE("GPL");
