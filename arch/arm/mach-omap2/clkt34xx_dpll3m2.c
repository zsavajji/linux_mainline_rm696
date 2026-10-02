// SPDX-License-Identifier: GPL-2.0-only
/*
 * OMAP34xx CORE DPLL (DPLL3) M2 divider: CORE, L3 and the SDRAM controller's clock change together,
 * so the SDRC is reprogrammed with it from SRAM (omap3_sram_configure_core_dpll), and VDD2 (the "core"
 * voltage domain) follows the rate.
 *
 * Copyright (C) 2007-2008 Texas Instruments, Inc.
 * Copyright (C) 2007-2010 Nokia Corporation
 *
 * Richard Woodruff, Paul Walmsley, Jouni Högander
 *
 * sgx: Linux 4.1's clkt34xx_dpll3m2.c (board-file kernels only), for dpll3_m2_ck's
 * "ti,omap3-dpll3-m2-clock" (drivers/clk/ti/divider.c, ti_clk_ll_ops). 4.1 ran it with interrupts on:
 * the old clock framework's set_rate held a spinlock, the common one's holds a mutex; while the SDRAM
 * is idle no interrupt may reach its handlers there, so they are off around the SRAM call.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/irqflags.h>
#include <linux/io.h>

#include "soc.h"
#include "clock.h"
#include "sdrc.h"
#include "sram.h"
#include "voltage.h"

#define CYCLES_PER_MHZ			1000000

/* 3630 VDD2 (CORE) OPP50 (L3 100 MHz) and OPP100 (L3 200 MHz), as stock's omap36xx_l3_rate_table */
#define VDD2_OPP50_UV			1000000
#define VDD2_OPP100_UV			1200000

static unsigned long vdd2_uv(unsigned long sdrc_rate)
{
	return sdrc_rate <= 100000000 ? VDD2_OPP50_UV : VDD2_OPP100_UV;
}

/**
 * omap3_core_dpll_m2_set_rate - set CORE DPLL M2 divider
 * @rate: new dpll3_m2_ck rate
 * @clkrate: its current rate
 * @new_div: new M2 divider (register value)
 *
 * Program the DPLL M2 divider with the rounded target rate.  Returns
 * -EINVAL upon error, or 0 upon success.
 */
int omap3_core_dpll_m2_set_rate(unsigned long rate, unsigned long clkrate, u32 new_div)
{
	struct omap_sdrc_params *sdrc_cs0, *sdrc_cs1;
	struct voltagedomain *vdd2;
	struct clk *sdrc_ick, *arm_fck;
	unsigned long sdrcrate, mpurate, flags;
	u32 unlock_dll = 0, c;
	int ret = 0;

	if (!rate || !clkrate)
		return -EINVAL;

	sdrc_ick = clk_get(NULL, "sdrc_ick");
	arm_fck = clk_get(NULL, "arm_fck");
	if (IS_ERR(sdrc_ick) || IS_ERR(arm_fck)) {
		ret = -ENOENT;
		goto out;
	}

	sdrcrate = clk_get_rate(sdrc_ick);
	if (rate > clkrate)
		sdrcrate <<= ((rate / clkrate) >> 1);
	else
		sdrcrate >>= ((clkrate / rate) >> 1);

	if (omap2_sdrc_get_params(sdrcrate, &sdrc_cs0, &sdrc_cs1)) {
		pr_err("dpll3_m2: no SDRC timings for %lu Hz\n", sdrcrate);
		ret = -EINVAL;
		goto out;
	}

	if (sdrcrate < MIN_SDRC_DLL_LOCK_FREQ)
		unlock_dll = 1;

	mpurate = clk_get_rate(arm_fck) / CYCLES_PER_MHZ;
	c = (mpurate << SDRC_MPURATE_SCALE) >> SDRC_MPURATE_BASE_SHIFT;
	c += 1;  /* for safety */
	c *= SDRC_MPURATE_LOOPS;
	c >>= SDRC_MPURATE_SCALE;
	if (c == 0)
		c = 1;

	/* VDD2 up before a faster clock, down after a slower one */
	vdd2 = voltdm_lookup("core");
	if (vdd2 && rate > clkrate) {
		ret = voltdm_scale(vdd2, vdd2_uv(sdrcrate));
		if (ret) {
			pr_err("dpll3_m2: VDD2 to %lu uV failed: %d\n", vdd2_uv(sdrcrate), ret);
			goto out;
		}
	}

	pr_info("dpll3_m2: CORE %lu -> %lu Hz, SDRC %lu Hz, RFR %08x%s\n", clkrate, rate, sdrcrate,
		sdrc_cs0->rfr_ctrl, unlock_dll ? ", DLL unlocked" : "");

	local_irq_save(flags);
	omap3_configure_core_dpll(new_div, unlock_dll, c, rate > clkrate,
				  sdrc_cs0->rfr_ctrl, sdrc_cs0->actim_ctrla,
				  sdrc_cs0->actim_ctrlb, sdrc_cs0->mr,
				  sdrc_cs1 ? sdrc_cs1->rfr_ctrl : 0,
				  sdrc_cs1 ? sdrc_cs1->actim_ctrla : 0,
				  sdrc_cs1 ? sdrc_cs1->actim_ctrlb : 0,
				  sdrc_cs1 ? sdrc_cs1->mr : 0);
	local_irq_restore(flags);

	if (vdd2 && rate < clkrate)
		voltdm_scale(vdd2, vdd2_uv(sdrcrate));
out:
	if (!IS_ERR(sdrc_ick))
		clk_put(sdrc_ick);
	if (!IS_ERR(arm_fck))
		clk_put(arm_fck);
	return ret;
}

/* ponytail: a test knob (dpll3_m2_ck's rate) until the L3 policy drives it */
static int dpll3_m2_rate_get(void *data, u64 *val)
{
	struct clk *clk = clk_get(NULL, "dpll3_m2_ck");

	if (IS_ERR(clk))
		return PTR_ERR(clk);
	*val = clk_get_rate(clk);
	clk_put(clk);
	return 0;
}

static int dpll3_m2_rate_set(void *data, u64 val)
{
	struct clk *clk = clk_get(NULL, "dpll3_m2_ck");
	int ret;

	if (IS_ERR(clk))
		return PTR_ERR(clk);
	ret = clk_set_rate(clk, val);
	clk_put(clk);
	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(dpll3_m2_rate_fops, dpll3_m2_rate_get, dpll3_m2_rate_set, "%llu\n");

static int __init dpll3_m2_debugfs_init(void)
{
	if (cpu_is_omap34xx())
		debugfs_create_file_unsafe("dpll3_m2_rate", 0600, NULL, NULL, &dpll3_m2_rate_fops);
	return 0;
}
late_initcall(dpll3_m2_debugfs_init);
