// SPDX-License-Identifier: GPL-2.0-only
/*
 * SmartReflex class 1.5 (periodic software calibration), as Nokia's N9 kernel ran it
 *
 * When a VDD changes to an OPP without a calibrated voltage, SmartReflex and the voltage processor
 * run from the OPP's voltage until the VP's voltage settles; that voltage becomes the OPP's
 * calibrated one, SR and the VP stop, and the OPP runs at it from then on (voltage.c,
 * omap_volt_operation()). Every 24 hours the VDD's OPPs are calibrated again, each starting 50 mV
 * above its last result. While a VDD calibrates its domains stay out of retention (pm34xx.c).
 *
 * From smartreflex-class1p5.c in Nokia's kernel-plus-harmattan, by Nishanth Menon <nm@ti.com>,
 * Copyright (C) 2010 Texas Instruments, Inc. Nokia's version sampled the VP's voltage on SR's
 * MCUBOUNDS interrupt, but its sr_late_init() left that interrupt disabled (disable_irq() after
 * request_irq(), and every calibration disabled it once more than it enabled it), so its
 * calibration always read "no samples, stable" at the second check and took the VP's voltage
 * there. This one reads the VP's voltage at the same checks: settled when two in a row agree,
 * else the highest after five, as Nokia's "oscillating" case meant to.
 */

#include <linux/debugfs.h>
#include <linux/power/smartreflex.h>
#include <linux/seq_file.h>
#include <linux/workqueue.h>
#include "soc.h"
#include "voltage.h"

#define SR1P5_CHECK_MS		5	/* SR1P5_SAMPLING_DELAY_MS x SR1P5_STABLE_SAMPLES */
#define SR1P5_CHECKS		5	/* SR1P5_MAX_TRIGGERS */
#define SR1P5_RECAL_MS		(24 * 3600 * 1000)
#define SR1P5_DYN_MARGIN_UV	50000	/* u_volt_dyn_margin of every OMAP3630 voltage */

static struct sr1p5 {
	struct delayed_work work;
	struct omap_sr *sr;
	unsigned long recal_at;
	u32 last, max;		/* the VP's voltage at the last check, the highest so far */
	u8 checks;
	bool calibrating;
} sr1p5[2];			/* one per VDD with SR: OMAP3's VDD1 and VDD2 */

static struct sr1p5 *sr1p5_get(struct omap_sr *sr)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sr1p5); i++) {
		if (!sr1p5[i].sr) {
			sr1p5[i].sr = sr;
			sr1p5[i].recal_at = jiffies + msecs_to_jiffies(SR1P5_RECAL_MS);
		}
		if (sr1p5[i].sr == sr)
			return &sr1p5[i];
	}
	return NULL;
}

bool sr1p5_calibrating(struct voltagedomain *voltdm)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sr1p5); i++)
		if (READ_ONCE(sr1p5[i].calibrating) && sr1p5[i].sr->voltdm == voltdm)
			return true;
	return false;
}

static u32 sr1p5_vp_voltage(struct voltagedomain *voltdm)
{
	struct omap_vp_instance *vp = voltdm->vp;

	return voltdm->pmic->vsel_to_uv(voltdm->read(vp->voltage) & vp->common->vpvoltage_mask);
}

/* As Nokia's: the VP, then SR */
static void sr1p5_stop(struct sr1p5 *s)
{
	omap_vp_disable(s->sr->voltdm);
	sr_disable(s->sr);
	WRITE_ONCE(s->calibrating, false);
}

static void sr1p5_check(struct work_struct *work)
{
	struct sr1p5 *s = container_of(to_delayed_work(work), struct sr1p5, work);
	struct voltagedomain *voltdm = s->sr->voltdm;
	struct omap_volt_data *vdata;
	u32 volt;

	/* omap_sr_scale() may hold the lock to cancel this: try again later, as Nokia's */
	if (!omap_sr_trylock()) {
		schedule_delayed_work(&s->work, msecs_to_jiffies(SR1P5_CHECK_MS));
		return;
	}
	if (!s->calibrating)
		goto out;

	volt = sr1p5_vp_voltage(voltdm);
	s->max = max(s->max, volt);
	if (volt != s->last && ++s->checks < SR1P5_CHECKS) {
		s->last = volt;
		schedule_delayed_work(&s->work, msecs_to_jiffies(SR1P5_CHECK_MS));
		goto out;
	}

	sr1p5_stop(s);
	vdata = omap_voltage_get_voltdata(voltdm, voltdm_get_voltage(voltdm));
	if (!IS_ERR(vdata)) {
		vdata->volt_calib = volt == s->last ? volt : s->max;
		vdata->volt_dyn_nominal = min(vdata->volt_calib + SR1P5_DYN_MARGIN_UV,
					      vdata->volt_nominal);
		/* there for good, the VC's ON voltage too (Nokia's vc_setup_on_voltage()) */
		voltdm_reset(voltdm);
	}
out:
	omap_sr_unlock();
}

/* After each voltage change and when autocomp is switched on, under omap_sr_scale()'s lock */
static int sr1p5_enable(struct omap_sr *sr)
{
	struct voltagedomain *voltdm = sr->voltdm;
	unsigned long volt = voltdm_get_voltage(voltdm);
	struct omap_volt_data *vdata = omap_voltage_get_voltdata(voltdm, volt);
	struct sr1p5 *s = sr1p5_get(sr);
	struct omap_volt_data *v;
	int ret;

	if (IS_ERR(vdata) || !s)
		return -ENODATA;
	if (s->calibrating)
		return 0;

	/* Nokia's sr_class1p5_reset_calib(): all OPPs again, each from its last result + 50 mV */
	if (time_after(jiffies, s->recal_at)) {
		for (v = voltdm->volt_data; v->volt_nominal; v++)
			v->volt_calib = 0;
		s->recal_at = jiffies + msecs_to_jiffies(SR1P5_RECAL_MS);
		voltdm_reset(voltdm);
	}
	if (vdata->volt_calib)
		return 0;

	omap_vp_enable(voltdm);
	ret = sr_enable(sr, volt);
	if (ret) {
		omap_vp_disable(voltdm);
		return ret;
	}
	s->checks = 0;
	s->last = 0;
	s->max = 0;
	WRITE_ONCE(s->calibrating, true);
	schedule_delayed_work(&s->work, msecs_to_jiffies(SR1P5_CHECK_MS));
	return 0;
}

/*
 * Before each voltage change (only while calibrating) and when autocomp is switched off
 * (@is_volt_reset): then the calibrated voltages go and the nominal ones come back, as Nokia's
 * sr_class1p5_cdeinit()
 */
static int sr1p5_disable(struct omap_sr *sr, int is_volt_reset)
{
	struct sr1p5 *s = sr1p5_get(sr);
	struct omap_volt_data *v;

	if (s && s->calibrating) {
		cancel_delayed_work_sync(&s->work);
		sr1p5_stop(s);
	}
	if (is_volt_reset) {
		for (v = sr->voltdm->volt_data; v->volt_nominal; v++)
			v->volt_calib = v->volt_dyn_nominal = 0;
		voltdm_reset(sr->voltdm);
	}
	return 0;
}

/* debugfs sr_calib: VDD, nominal, calibration start and calibrated voltage of each OPP (uV) */
static int sr1p5_calib_show(struct seq_file *m, void *unused)
{
	struct omap_volt_data *v;
	int i;

	for (i = 0; i < ARRAY_SIZE(sr1p5); i++) {
		if (!sr1p5[i].sr)
			continue;
		for (v = sr1p5[i].sr->voltdm->volt_data; v->volt_nominal; v++)
			seq_printf(m, "%s %u %u %u\n", sr1p5[i].sr->voltdm->name, v->volt_nominal,
				   v->volt_dyn_nominal, v->volt_calib);
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(sr1p5_calib);

static struct omap_sr_class_data class1p5_data = {
	.enable = sr1p5_enable,
	.disable = sr1p5_disable,
	.configure = sr_configure_errgen,
};

static int __init sr_class1p5_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sr1p5); i++)
		INIT_DELAYED_WORK(&sr1p5[i].work, sr1p5_check);
	debugfs_create_file("sr_calib", 0444, NULL, NULL, &sr1p5_calib_fops);
	pr_info("SmartReflex class 1.5 initialized\n");
	return sr_register_class(&class1p5_data);
}
omap_late_initcall(sr_class1p5_init);
