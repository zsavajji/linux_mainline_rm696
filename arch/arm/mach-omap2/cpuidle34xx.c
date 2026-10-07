// SPDX-License-Identifier: GPL-2.0-only
/*
 * linux/arch/arm/mach-omap2/cpuidle34xx.c
 *
 * OMAP3 CPU IDLE Routines
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Karthik Dasu <karthik-dp@ti.com>
 *
 * Copyright (C) 2006 Nokia Corporation
 * Tony Lindgren <tony@atomide.com>
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * Based on pm.c for omap2
 */

#include <linux/sched.h>
#include <linux/cpuidle.h>
#include <linux/export.h>
#include <linux/cpu_pm.h>
#include <asm/cpuidle.h>

#include "powerdomain.h"
#include "cm3xxx.h"
#include "clockdomain.h"

#include "pm.h"
#include "control.h"
#include "common.h"
#include "soc.h"

/* Mach specific information to be recorded in the C-state driver_data */
struct omap3_idle_statedata {
	u8 mpu_state;
	u8 core_state;
	u8 per_min_state;
	u8 flags;
};

static struct powerdomain *mpu_pd, *core_pd, *per_pd, *cam_pd;

/* Domains CORE OFF would reset under their feet: CORE goes OFF only once they are */
static const char * const core_off_pd_names[] = {
	"cam_pwrdm", "dss_pwrdm", "iva2_pwrdm", "sgx_pwrdm", "usbhost_pwrdm",
};
static struct powerdomain *core_off_pds[ARRAY_SIZE(core_off_pd_names)];

/*
 * Possible flag bits for struct omap3_idle_statedata.flags:
 *
 * OMAP_CPUIDLE_CX_NO_CLKDM_IDLE: don't allow the MPU clockdomain to go
 *    inactive.  This in turn prevents the MPU DPLL from entering autoidle
 *    mode, so wakeup latency is greatly reduced, at the cost of additional
 *    energy consumption.  This also prevents the CORE clockdomain from
 *    entering idle.
 */
#define OMAP_CPUIDLE_CX_NO_CLKDM_IDLE		BIT(0)

/*
 * Prevent PER OFF if CORE is not in RETention or OFF as this would
 * disable PER wakeups completely.
 */
static struct omap3_idle_statedata omap3_idle_data[] = {
	{
		.mpu_state = PWRDM_POWER_ON,
		.core_state = PWRDM_POWER_ON,
		/* In C1 do not allow PER state lower than CORE state */
		.per_min_state = PWRDM_POWER_ON,
		.flags = OMAP_CPUIDLE_CX_NO_CLKDM_IDLE,
	},
	{
		.mpu_state = PWRDM_POWER_ON,
		.core_state = PWRDM_POWER_ON,
		.per_min_state = PWRDM_POWER_RET,
	},
	{
		.mpu_state = PWRDM_POWER_RET,
		.core_state = PWRDM_POWER_ON,
		.per_min_state = PWRDM_POWER_RET,
	},
	{
		.mpu_state = PWRDM_POWER_OFF,
		.core_state = PWRDM_POWER_ON,
		.per_min_state = PWRDM_POWER_RET,
	},
	/* PER OFF only with CORE OFF, as Nokia's N9 kernel (cpuidle34xx.c) */
	{
		.mpu_state = PWRDM_POWER_RET,
		.core_state = PWRDM_POWER_RET,
		.per_min_state = PWRDM_POWER_RET,
	},
	{
		.mpu_state = PWRDM_POWER_OFF,
		.core_state = PWRDM_POWER_RET,
		.per_min_state = PWRDM_POWER_RET,
	},
	{
		.mpu_state = PWRDM_POWER_OFF,
		.core_state = PWRDM_POWER_OFF,
		.per_min_state = PWRDM_POWER_OFF,
	},
};

/**
 * omap3_enter_idle - Programs OMAP3 to enter the specified state
 * @dev: cpuidle device
 * @drv: cpuidle driver
 * @index: the index of state to be entered
 */
static int omap3_enter_idle(struct cpuidle_device *dev,
			    struct cpuidle_driver *drv,
			    int index)
{
	struct omap3_idle_statedata *cx = &omap3_idle_data[index];
	int error;

	if (omap_irq_pending() || need_resched())
		goto return_sleep_time;

	/* Deny idle for C1 */
	if (cx->flags & OMAP_CPUIDLE_CX_NO_CLKDM_IDLE) {
		clkdm_deny_idle(mpu_pd->pwrdm_clkdms[0]);
	} else {
		pwrdm_set_next_pwrst(mpu_pd, cx->mpu_state);
		pwrdm_set_next_pwrst(core_pd, cx->core_state);
	}

	/*
	 * Call idle CPU PM enter notifier chain so that
	 * VFP context is saved.
	 */
	if (cx->mpu_state == PWRDM_POWER_OFF) {
		error = cpu_pm_enter();
		if (error)
			goto out_clkdm_set;
	}

	/* Execute ARM wfi */
	omap_sram_idle(true);

	/*
	 * Call idle CPU PM enter notifier chain to restore
	 * VFP context.
	 */
	if (cx->mpu_state == PWRDM_POWER_OFF &&
	    pwrdm_read_prev_pwrst(mpu_pd) == PWRDM_POWER_OFF)
		cpu_pm_exit();

out_clkdm_set:
	/* Re-allow idle for C1 */
	if (cx->flags & OMAP_CPUIDLE_CX_NO_CLKDM_IDLE)
		clkdm_allow_idle(mpu_pd->pwrdm_clkdms[0]);

return_sleep_time:

	return index;
}

/**
 * next_valid_state - Find next valid C-state
 * @dev: cpuidle device
 * @drv: cpuidle driver
 * @index: Index of currently selected c-state
 *
 * If the state corresponding to index is valid, index is returned back
 * to the caller. Else, this function searches for a lower c-state which is
 * still valid (as defined in omap3_power_states[]) and returns its index.
 *
 * A state is valid if the 'valid' field is enabled and
 * if it satisfies the enable_off_mode condition.
 */
static int next_valid_state(struct cpuidle_device *dev,
			    struct cpuidle_driver *drv, int index)
{
	struct omap3_idle_statedata *cx = &omap3_idle_data[index];
	u32 mpu_deepest_state = PWRDM_POWER_RET;
	u32 core_deepest_state = PWRDM_POWER_RET;
	int idx;
	int next_index = 0; /* C1 is the default value */

	if (enable_off_mode) {
		mpu_deepest_state = PWRDM_POWER_OFF;
		/*
		 * Erratum i583: valable for ES rev < Es1.2 on 3630.
		 * CORE OFF mode is not supported in a stable form, restrict
		 * instead the CORE state to RET.
		 */
		if (!IS_PM34XX_ERRATUM(PM_SDRC_WAKEUP_ERRATUM_i583) &&
		    !disable_core_off)
			core_deepest_state = PWRDM_POWER_OFF;
	}

	/* Check if current state is valid */
	if ((cx->mpu_state >= mpu_deepest_state) &&
	    (cx->core_state >= core_deepest_state))
		return index;

	/*
	 * Drop to next valid state.
	 * Start search from the next (lower) state.
	 */
	for (idx = index - 1; idx >= 0; idx--) {
		cx = &omap3_idle_data[idx];
		if ((cx->mpu_state >= mpu_deepest_state) &&
		    (cx->core_state >= core_deepest_state)) {
			next_index = idx;
			break;
		}
	}

	return next_index;
}

/* The index of C6, MPU OFF + CORE RET */
#define OMAP3_IDLE_MPU_OFF_CORE_RET	5

/*
 * Whether the domains CORE OFF resets will be OFF with it. On 6.18 the hwmods keep the interface
 * clocks of DSS and USBHOST enabled, so their clockdomains keep their wakeup dependency on the
 * MPU and read ON whenever this runs; they still go OFF as the MPU sleeps, unless a functional
 * clock of theirs runs. Nokia's kernel saw them OFF here (its clock usecounts reached 0).
 */
static bool omap3_core_off_allowed(void)
{
	struct powerdomain *pd;
	int i;

	for (i = 0; i < ARRAY_SIZE(core_off_pds); i++) {
		pd = core_off_pds[i];
		if (!pd || pwrdm_read_pwrst(pd) == PWRDM_POWER_OFF)
			continue;
		if (pwrdm_read_next_pwrst(pd) != PWRDM_POWER_OFF ||
		    omap2_cm_read_mod_reg(pd->prcm_offs, CM_FCLKEN))
			return false;
	}

	return true;
}

/**
 * omap3_enter_idle_bm - Checks for any bus activity
 * @dev: cpuidle device
 * @drv: cpuidle driver
 * @index: array index of target state to be programmed
 *
 * This function checks for any pending activity and then programs
 * the device to the specified or a safer state.
 */
static int omap3_enter_idle_bm(struct cpuidle_device *dev,
			       struct cpuidle_driver *drv,
			       int index)
{
	int new_state_idx, ret;
	u8 per_next_state, per_saved_state;
	struct omap3_idle_statedata *cx;

	/*
	 * Use only C1 if CAM is active.
	 * CAM does not have wakeup capability in OMAP3.
	 */
	if (pwrdm_read_pwrst(cam_pd) == PWRDM_POWER_ON)
		new_state_idx = drv->safe_state_index;
	else
		new_state_idx = next_valid_state(dev, drv, index);

	/*
	 * CORE OFF resets the chip: only with CAM, DSS, IVA2, SGX and USBHOST off already, else
	 * MPU OFF + CORE RET, as Nokia's N9 kernel (cpuidle34xx.c, omap3_enter_idle_bm())
	 */
	if (omap3_idle_data[new_state_idx].core_state == PWRDM_POWER_OFF &&
	    !omap3_core_off_allowed())
		new_state_idx = OMAP3_IDLE_MPU_OFF_CORE_RET;

	/*
	 * FIXME: we currently manage device-specific idle states
	 *        for PER and CORE in combination with CPU-specific
	 *        idle states.  This is wrong, and device-specific
	 *        idle management needs to be separated out into
	 *        its own code.
	 */

	/* Program PER state */
	cx = &omap3_idle_data[new_state_idx];

	per_next_state = pwrdm_read_next_pwrst(per_pd);
	per_saved_state = per_next_state;
	if (per_next_state < cx->per_min_state) {
		per_next_state = cx->per_min_state;
		pwrdm_set_next_pwrst(per_pd, per_next_state);
	}

	ret = omap3_enter_idle(dev, drv, new_state_idx);

	/* Restore original PER state if it was modified */
	if (per_next_state != per_saved_state)
		pwrdm_set_next_pwrst(per_pd, per_saved_state);

	return ret;
}

static struct cpuidle_driver omap3_idle_driver = {
	.name             = "omap3_idle",
	.owner            = THIS_MODULE,
	.states = {
		{
			.flags		  = CPUIDLE_FLAG_RCU_IDLE,
			.enter		  = omap3_enter_idle_bm,
			.exit_latency	  = 2 + 2,
			.target_residency = 5,
			.name		  = "C1",
			.desc		  = "MPU ON + CORE ON",
		},
		{
			.flags		  = CPUIDLE_FLAG_RCU_IDLE,
			.enter		  = omap3_enter_idle_bm,
			.exit_latency	  = 10 + 10,
			.target_residency = 30,
			.name		  = "C2",
			.desc		  = "MPU ON + CORE ON",
		},
		{
			.flags		  = CPUIDLE_FLAG_RCU_IDLE,
			.enter		  = omap3_enter_idle_bm,
			.exit_latency	  = 50 + 50,
			.target_residency = 300,
			.name		  = "C3",
			.desc		  = "MPU RET + CORE ON",
		},
		{
			.flags		  = CPUIDLE_FLAG_RCU_IDLE,
			.enter		  = omap3_enter_idle_bm,
			.exit_latency	  = 1500 + 1800,
			.target_residency = 4000,
			.name		  = "C4",
			.desc		  = "MPU OFF + CORE ON",
		},
		{
			.flags		  = CPUIDLE_FLAG_RCU_IDLE,
			.enter		  = omap3_enter_idle_bm,
			.exit_latency	  = 2500 + 7500,
			.target_residency = 12000,
			.name		  = "C5",
			.desc		  = "MPU RET + CORE RET",
		},
		{
			.flags		  = CPUIDLE_FLAG_RCU_IDLE,
			.enter		  = omap3_enter_idle_bm,
			.exit_latency	  = 3000 + 8500,
			.target_residency = 15000,
			.name		  = "C6",
			.desc		  = "MPU OFF + CORE RET",
		},
		{
			.flags		  = CPUIDLE_FLAG_RCU_IDLE,
			.enter		  = omap3_enter_idle_bm,
			.exit_latency	  = 10000 + 30000,
			.target_residency = 30000,
			.name		  = "C7",
			.desc		  = "MPU OFF + CORE OFF",
		},
	},
	.state_count = ARRAY_SIZE(omap3_idle_data),
	.safe_state_index = 0,
};

/*
 * Numbers based on measurements made in October 2009 for PM optimized kernel
 * with CPU freq enabled on device Nokia N900. Assumes OPP2 (main idle OPP,
 * and worst case latencies).
 */
static struct cpuidle_driver omap3430_idle_driver = {
	.name             = "omap3430_idle",
	.owner            = THIS_MODULE,
	.states = {
		{
			.flags		  = CPUIDLE_FLAG_RCU_IDLE,
			.enter		  = omap3_enter_idle_bm,
			.exit_latency	  = 110 + 162,
			.target_residency = 5,
			.name		  = "C1",
			.desc		  = "MPU ON + CORE ON",
		},
		{
			.flags		  = CPUIDLE_FLAG_RCU_IDLE,
			.enter		  = omap3_enter_idle_bm,
			.exit_latency	  = 106 + 180,
			.target_residency = 309,
			.name		  = "C2",
			.desc		  = "MPU ON + CORE ON",
		},
		{
			.flags		  = CPUIDLE_FLAG_RCU_IDLE,
			.enter		  = omap3_enter_idle_bm,
			.exit_latency	  = 107 + 410,
			.target_residency = 46057,
			.name		  = "C3",
			.desc		  = "MPU RET + CORE ON",
		},
		{
			.flags		  = CPUIDLE_FLAG_RCU_IDLE,
			.enter		  = omap3_enter_idle_bm,
			.exit_latency	  = 121 + 3374,
			.target_residency = 46057,
			.name		  = "C4",
			.desc		  = "MPU OFF + CORE ON",
		},
		{
			.flags		  = CPUIDLE_FLAG_RCU_IDLE,
			.enter		  = omap3_enter_idle_bm,
			.exit_latency	  = 855 + 1146,
			.target_residency = 46057,
			.name		  = "C5",
			.desc		  = "MPU RET + CORE RET",
		},
		{
			.flags		  = CPUIDLE_FLAG_RCU_IDLE,
			.enter		  = omap3_enter_idle_bm,
			.exit_latency	  = 7580 + 4134,
			.target_residency = 484329,
			.name		  = "C6",
			.desc		  = "MPU OFF + CORE RET",
		},
		{
			.flags		  = CPUIDLE_FLAG_RCU_IDLE,
			.enter		  = omap3_enter_idle_bm,
			.exit_latency	  = 7505 + 15274,
			.target_residency = 484329,
			.name		  = "C7",
			.desc		  = "MPU OFF + CORE OFF",
		},
	},
	.state_count = ARRAY_SIZE(omap3_idle_data),
	.safe_state_index = 0,
};

/* Public functions */

/**
 * omap3_idle_init - Init routine for OMAP3 idle
 *
 * Registers the OMAP3 specific cpuidle driver to the cpuidle
 * framework with the valid set of states.
 */
int __init omap3_idle_init(void)
{
	int i;

	mpu_pd = pwrdm_lookup("mpu_pwrdm");
	core_pd = pwrdm_lookup("core_pwrdm");
	per_pd = pwrdm_lookup("per_pwrdm");
	cam_pd = pwrdm_lookup("cam_pwrdm");

	if (!mpu_pd || !core_pd || !per_pd || !cam_pd)
		return -ENODEV;

	for (i = 0; i < ARRAY_SIZE(core_off_pds); i++)
		core_off_pds[i] = pwrdm_lookup(core_off_pd_names[i]);

	if (cpu_is_omap3430())
		return cpuidle_register(&omap3430_idle_driver, NULL);
	else
		return cpuidle_register(&omap3_idle_driver, NULL);
}
