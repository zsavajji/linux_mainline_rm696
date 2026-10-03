/*
 * _tiomap_pwr.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Definitions and types for the DSP wake/sleep routines.
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef _TIOMAP_PWR_
#define _TIOMAP_PWR_

#ifdef CONFIG_PM
extern s32 dsp_test_sleepstate;

/*
 * The 3630's IVA OPPs as Nokia's dspbridge.c gave them to the DSP (kHz, uV) and the MPU's
 * cpufreq OPP of the same VDD1 level (kHz); the DSP numbers them from 1
 */
extern const u32 dsp_opps[4][3];

/* The IVA2 at the VDD1 OPP's rate from now on, following the CPU (before the DSP starts) */
void dsp_opp_init(struct bridge_dev_context *dev_context);
/* The DSP's request for an OPP (MBX_PM_OPP_REQ): a floor on VDD1, asynchronously */
void dsp_opp_request(struct bridge_dev_context *dev_context, u32 opp);
/* The DSP hibernates: no floor */
void dsp_opp_off(struct bridge_dev_context *dev_context);
#endif

extern struct mailbox_context mboxsetting;

/*
 * ======== wake_dsp =========
 * Wakes up the DSP from DeepSleep
 */
extern int wake_dsp(struct bridge_dev_context *dev_context,
							void *pargs);

/*
 * ======== sleep_dsp =========
 * Places the DSP in DeepSleep.
 */
extern int sleep_dsp(struct bridge_dev_context *dev_context,
			    u32 dw_cmd, void *pargs);
/*
 *  ========interrupt_dsp========
 *	Sends an interrupt to DSP unconditionally.
 */
extern void interrupt_dsp(struct bridge_dev_context *dev_context,
							u16 mb_val);

/*
 * ======== wake_dsp =========
 * Wakes up the DSP from DeepSleep
 */
extern int dsp_peripheral_clk_ctrl(struct bridge_dev_context
					*dev_context, void *pargs);
/*
 *  ======== handle_hibernation_from_dsp ========
 *	Handle Hibernation requested from DSP
 */
int handle_hibernation_from_dsp(struct bridge_dev_context *dev_context);
/*
 *  ======== post_scale_dsp ========
 *	Handle Post Scale notification to DSP
 */
int post_scale_dsp(struct bridge_dev_context *dev_context,
							void *pargs);
/*
 *  ======== pre_scale_dsp ========
 *	Handle Pre Scale notification to DSP
 */
int pre_scale_dsp(struct bridge_dev_context *dev_context,
							void *pargs);
/*
 *  ======== handle_constraints_set ========
 *	Handle constraints request from DSP
 */
int handle_constraints_set(struct bridge_dev_context *dev_context,
				  void *pargs);

/*
 *  ======== dsp_clk_wakeup_event_ctrl ========
 *     This function sets the group selction bits for while
 *     enabling/disabling.
 */
void dsp_clk_wakeup_event_ctrl(u32 clock_id, bool enable);

#endif /* _TIOMAP_PWR_ */
