/*
 * clk.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Clock and Timer services.
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

#include <linux/types.h>

/*  ----------------------------------- Host OS */
#include <dspbridge/host_os.h>

/*  ----------------------------------- DSP/BIOS Bridge */
#include <dspbridge/dbdefs.h>
#include <dspbridge/drv.h>
#include <dspbridge/dev.h>
#include "_tiomap.h"

/*  ----------------------------------- This */
#include <dspbridge/clk.h>

/*  ----------------------------------- Defines, Data Structures, Typedefs */

/* Clk types requested by the dsp */
#define IVA2_CLK	0
#define GPT_CLK		1
#define WDT_CLK		2
#define MCBSP_CLK	3
#define SSI_CLK		4

/* Bridge GPT id (1 - 4) is OMAP GPTIMER 5 - 8 */
#define DM_TIMER_CLOCKS		4

/* GPTIMER registers, for dsp_gpt_wait_overflow() */
#define GPT_TISR		0x18
#define GPT_TIER		0x1c
#define GPT_TCLR		0x24
#define GPT_TCRR		0x28
#define GPT_TLDR		0x2c
#define GPT_TWPS		0x34
#define GPT_TCLR_ST		BIT(0)
#define GPT_TCLR_AR		BIT(1)
#define GPT_INT_OVERFLOW	BIT(1)

static const u32 gpt_base[DM_TIMER_CLOCKS] = {
	0x49038000, 0x4903a000, 0x4903c000, 0x4903e000,
};

/*
 * GPTIMER 5-8 belong to the DSP: the bridge only gates their interface and
 * functional clocks (clkdev aliases of the OMAP3 clock tree) when the DSP
 * asks, the DSP programs the timers itself, as on Nokia's bridge.
 */
static struct clk *gpt_fck[DM_TIMER_CLOCKS];
static struct clk *gpt_ick[DM_TIMER_CLOCKS];
/* mapped at init: dsp_gpt_wait_overflow() runs in the MMU fault tasklet */
static void __iomem *gpt_regs[DM_TIMER_CLOCKS];

struct clk *iva2_clk;

static u32 dsp_clocks;

static inline u32 is_dsp_clk_active(u32 clk, u8 id)
{
	return clk & (1 << id);
}

static inline void set_dsp_clk_active(u32 *clk, u8 id)
{
	*clk |= (1 << id);
}

static inline void set_dsp_clk_inactive(u32 *clk, u8 id)
{
	*clk &= ~(1 << id);
}

static s8 get_clk_type(u8 id)
{
	s8 type;

	if (id == DSP_CLK_IVA2)
		type = IVA2_CLK;
	else if (id <= DSP_CLK_GPT8)
		type = GPT_CLK;
	else if (id == DSP_CLK_WDT3)
		type = WDT_CLK;
	else if (id <= DSP_CLK_MCBSP5)
		type = MCBSP_CLK;
	else if (id == DSP_CLK_SSI)
		type = SSI_CLK;
	else
		type = -1;

	return type;
}

/* A missing clock becomes NULL, which the clk API treats as a no-op. */
static struct clk *dsp_clk_get_prepared(const char *name)
{
	struct clk *clk = clk_get(bridge, name);

	if (IS_ERR(clk)) {
		dev_err(bridge, "failed to get clock %s: %ld\n", name,
			PTR_ERR(clk));
		return NULL;
	}
	if (clk_prepare(clk)) {
		dev_err(bridge, "failed to prepare clock %s\n", name);
		clk_put(clk);
		return NULL;
	}
	return clk;
}

static void dsp_clk_put_prepared(struct clk *clk)
{
	if (!clk)
		return;
	clk_unprepare(clk);
	clk_put(clk);
}

/*
 *  ======== dsp_clk_exit ========
 *  Purpose:
 *      Cleanup CLK module.
 */
void dsp_clk_exit(void)
{
	int i;

	dsp_clock_disable_all(dsp_clocks);

	for (i = 0; i < DM_TIMER_CLOCKS; i++) {
		dsp_clk_put_prepared(gpt_fck[i]);
		dsp_clk_put_prepared(gpt_ick[i]);
		gpt_fck[i] = NULL;
		gpt_ick[i] = NULL;
		if (gpt_regs[i])
			iounmap(gpt_regs[i]);
		gpt_regs[i] = NULL;
	}

	dsp_clk_put_prepared(iva2_clk);
	iva2_clk = NULL;
}

/*
 *  ======== dsp_clk_init ========
 *  Purpose:
 *      Initialize CLK module.
 */
void dsp_clk_init(void)
{
	char name[16];
	int i;

	for (i = 0; i < DM_TIMER_CLOCKS; i++) {
		snprintf(name, sizeof(name), "gpt%d_fck", i + 5);
		gpt_fck[i] = dsp_clk_get_prepared(name);
		snprintf(name, sizeof(name), "gpt%d_ick", i + 5);
		gpt_ick[i] = dsp_clk_get_prepared(name);
		gpt_regs[i] = ioremap(gpt_base[i], SZ_4K);
	}

	iva2_clk = dsp_clk_get_prepared("iva2_ck");
}

static void gpt_write(void __iomem *base, u32 reg, u32 val)
{
	int i;

	writel(val, base + reg);
	/* wait for a posted write to complete */
	for (i = 0; i < 1000 && readl(base + GPT_TWPS); i++)
		udelay(1);
}

/**
 * dsp_gpt_wait_overflow - set gpt overflow and wait for fixed timeout
 * @clk_id:      GP Timer clock id.
 * @load:        Overflow value.
 *
 * Sets an overflow interrupt for the desired GPT waiting for a timeout
 * of 5 msecs for the interrupt to occur.
 */
void dsp_gpt_wait_overflow(short int clk_id, unsigned int load)
{
	int i = clk_id - DSP_CLK_GPT5;
	void __iomem *base;
	unsigned long timeout;
	u32 l;

	if (i < 0 || i >= DM_TIMER_CLOCKS || !gpt_regs[i])
		return;
	base = gpt_regs[i];

	if (clk_enable(gpt_ick[i]))
		return;
	if (clk_enable(gpt_fck[i]))
		goto out_ick;

	/* Enable overflow interrupt */
	gpt_write(base, GPT_TIER, GPT_INT_OVERFLOW);

	/*
	 * Set counter value to overflow counter after
	 * one tick and start timer (one-shot).
	 */
	l = (readl(base + GPT_TCLR) & ~GPT_TCLR_AR) | GPT_TCLR_ST;
	gpt_write(base, GPT_TCRR, load);
	gpt_write(base, GPT_TCLR, l);

	/* Wait 80us for timer to overflow */
	udelay(80);

	timeout = jiffies + msecs_to_jiffies(5);
	/* Check interrupt status and wait for interrupt */
	while (!(readl(base + GPT_TISR) & GPT_INT_OVERFLOW)) {
		if (time_after(jiffies, timeout)) {
			pr_err("%s: GPTimer interrupt failed\n", __func__);
			break;
		}
	}

	clk_disable(gpt_fck[i]);
out_ick:
	clk_disable(gpt_ick[i]);
}

/*
 *  ======== dsp_clk_enable ========
 *  Purpose:
 *      Enable Clock .
 *
 */
int dsp_clk_enable(enum dsp_clk_id clk_id)
{
	int status = 0;

	if (is_dsp_clk_active(dsp_clocks, clk_id)) {
		dev_err(bridge, "WARN: clock id %d already enabled\n", clk_id);
		goto out;
	}

	switch (get_clk_type(clk_id)) {
	case IVA2_CLK:
		status = clk_enable(iva2_clk);
		break;
	case GPT_CLK:
		status = clk_enable(gpt_ick[clk_id - DSP_CLK_GPT5]);
		if (status)
			break;
		status = clk_enable(gpt_fck[clk_id - DSP_CLK_GPT5]);
		if (status)
			clk_disable(gpt_ick[clk_id - DSP_CLK_GPT5]);
		break;
	case WDT_CLK:
		dev_err(bridge, "ERROR: DSP requested to enable WDT3 clk\n");
		break;
	case MCBSP_CLK:
	case SSI_CLK:
		/*
		 * The McBSPs and the SSI belong to their own drivers now
		 * (ASoC, the modem's omap_ssi); the old request/clock-source
		 * and SSI SYSCONFIG overrides are gone, so these are not
		 * handed to the DSP.
		 */
		dev_err(bridge, "DSP peripheral clock %d not supported\n",
			clk_id);
		status = -EPERM;
		break;
	default:
		dev_err(bridge, "Invalid clock id for enable\n");
		status = -EPERM;
	}

	if (!status)
		set_dsp_clk_active(&dsp_clocks, clk_id);

out:
	return status;
}

/**
 * dsp_clock_enable_all - Enable clocks used by the DSP
 * @dev_context		Driver's device context strucure
 *
 * This function enables all the peripheral clocks that were requested by DSP.
 */
u32 dsp_clock_enable_all(u32 dsp_per_clocks)
{
	u32 clk_id;
	u32 status = -EPERM;

	for (clk_id = 0; clk_id < DSP_CLK_NOT_DEFINED; clk_id++) {
		if (is_dsp_clk_active(dsp_per_clocks, clk_id))
			status = dsp_clk_enable(clk_id);
	}

	return status;
}

/*
 *  ======== dsp_clk_disable ========
 *  Purpose:
 *      Disable the clock.
 *
 */
int dsp_clk_disable(enum dsp_clk_id clk_id)
{
	int status = 0;

	if (!is_dsp_clk_active(dsp_clocks, clk_id)) {
		dev_err(bridge, "ERR: clock id %d already disabled\n", clk_id);
		goto out;
	}

	switch (get_clk_type(clk_id)) {
	case IVA2_CLK:
		clk_disable(iva2_clk);
		break;
	case GPT_CLK:
		clk_disable(gpt_fck[clk_id - DSP_CLK_GPT5]);
		clk_disable(gpt_ick[clk_id - DSP_CLK_GPT5]);
		break;
	case WDT_CLK:
		dev_err(bridge, "ERROR: DSP requested to disable WDT3 clk\n");
		break;
	default:
		dev_err(bridge, "Invalid clock id for disable\n");
		status = -EPERM;
	}

	if (!status)
		set_dsp_clk_inactive(&dsp_clocks, clk_id);

out:
	return status;
}

/**
 * dsp_clock_disable_all - Disable all active clocks
 * @dev_context		Driver's device context structure
 *
 * This function disables all the peripheral clocks that were enabled by DSP.
 * It is meant to be called only when DSP is entering hibernation or when DSP
 * is in error state.
 */
u32 dsp_clock_disable_all(u32 dsp_per_clocks)
{
	u32 clk_id;
	u32 status = -EPERM;

	for (clk_id = 0; clk_id < DSP_CLK_NOT_DEFINED; clk_id++) {
		if (is_dsp_clk_active(dsp_per_clocks, clk_id))
			status = dsp_clk_disable(clk_id);
	}

	return status;
}

u32 dsp_clk_get_iva2_rate(void)
{
	u32 clk_speed_khz;

	clk_speed_khz = clk_get_rate(iva2_clk);
	clk_speed_khz /= 1000;
	dev_dbg(bridge, "%s: clk speed Khz = %d\n", __func__, clk_speed_khz);

	return clk_speed_khz;
}
