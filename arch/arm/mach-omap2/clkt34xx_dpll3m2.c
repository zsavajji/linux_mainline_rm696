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
#include <linux/clk/ti.h>
#include <linux/cpufreq.h>
#include <linux/debugfs.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/irqflags.h>
#include <linux/io.h>
#include <linux/math64.h>

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

	pr_debug("dpll3_m2: CORE %lu -> %lu Hz, SDRC %lu Hz, RFR %08x%s\n", clkrate, rate, sdrcrate,
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

/*
 * The policy, as Nokia's N9 kernel: L3 at OPP50 (M2 = 2: L3 and the SDRC at 97.6 MHz, VDD2 1.0 V) only
 * while nothing needs it high. The CPU above its minimum (scaling_min_freq: stock's rule was its lowest
 * OPP, 300 MHz; the user's phone sets a 600 MHz floor) holds it high (cpufreq), and so do
 * omap3_l3_hold() callers (enum omap3_l3_user): the SGX from before it gets its clock until after it
 * lost it, so its clock (CORE/2) never changes while it runs; the display, USB and WLAN as stock asked
 * for bus throughput. Raised at once, lowered l3_lower_delay_ms after the last hold went (100 ms; in
 * /sys/module/clkt34xx_dpll3m2/parameters). Measured in settled standby (2026-10-03): 1000 ms kept L3
 * at OPP50 39% of the time with 39 switches a minute, 500 ms 61%/43, 250 ms 66%/49, 100 ms 77%/53,
 * 0 ms (stock) 79%/60.
 */
static unsigned int l3_lower_delay_ms = 100;
module_param(l3_lower_delay_ms, uint, 0644);

static struct clk *l3_m2;
static unsigned long l3_high, l3_low;	/* dpll3_m2_ck rates */
static DEFINE_MUTEX(l3_lock);
static int l3_holds[OMAP3_L3_USERS];
static const char * const l3_names[OMAP3_L3_USERS] = { "cpu", "sgx", "dss", "usb", "wlan" };
static unsigned long l3_ignored;	/* debugfs omap3_l3 "-user": holds not counted, for tests */
static int l3_forced;			/* debugfs omap3_l3: 0 auto, 1 low, 2 high */
static bool l3_cpu_high;
static unsigned long l3_since;		/* jiffies at the last accounting */
static u64 l3_ms[2];			/* time at OPP100, OPP50 */
static unsigned long l3_switches;

static void l3_account(void)
{
	unsigned long now = jiffies;

	l3_ms[clk_get_rate(l3_m2) == l3_low] += jiffies_to_msecs(now - l3_since);
	l3_since = now;
}

static unsigned long l3_target(void)
{
	int i;

	if (l3_forced)
		return l3_forced == 1 ? l3_low : l3_high;
	for (i = 0; i < OMAP3_L3_USERS; i++)
		if (l3_holds[i] && !test_bit(i, &l3_ignored))
			return l3_high;
	return l3_low;
}

static void l3_apply(void)
{
	unsigned long rate;

	lockdep_assert_held(&l3_lock);
	if (!l3_m2)
		return;
	rate = l3_target();
	if (clk_get_rate(l3_m2) == rate)
		return;
	l3_account();
	if (clk_set_rate(l3_m2, rate))
		pr_err("omap3-l3: dpll3_m2_ck to %lu Hz failed\n", rate);
	else
		l3_switches++;
}

static void l3_lower_work(struct work_struct *work)
{
	mutex_lock(&l3_lock);
	if (l3_target() == l3_low)
		l3_apply();
	mutex_unlock(&l3_lock);
}
static DECLARE_DELAYED_WORK(l3_lower, l3_lower_work);

/* L3 at OPP100 from now until omap3_l3_release(); may sleep. Holds count from boot (the display's and
 * USB's start before the policy) */
void omap3_l3_hold(enum omap3_l3_user who)
{
	mutex_lock(&l3_lock);
	l3_holds[who]++;
	l3_apply();
	mutex_unlock(&l3_lock);
}
EXPORT_SYMBOL_GPL(omap3_l3_hold);

void omap3_l3_release(enum omap3_l3_user who)
{
	mutex_lock(&l3_lock);
	if (!WARN_ON(l3_holds[who] <= 0) && !--l3_holds[who])
		mod_delayed_work(system_wq, &l3_lower, msecs_to_jiffies(l3_lower_delay_ms));
	mutex_unlock(&l3_lock);
}
EXPORT_SYMBOL_GPL(omap3_l3_release);

static void l3_cpu_work(struct work_struct *work)
{
	static bool held;
	bool high = READ_ONCE(l3_cpu_high);

	if (high == held)
		return;
	held = high;
	if (high)
		omap3_l3_hold(OMAP3_L3_CPU);
	else
		omap3_l3_release(OMAP3_L3_CPU);
}
static DECLARE_WORK(l3_cpu, l3_cpu_work);

static int l3_cpufreq_cb(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_freqs *f = data;

	if (val == CPUFREQ_POSTCHANGE) {
		/* a new scaling_min_freq counts from the next frequency change */
		WRITE_ONCE(l3_cpu_high, f->new > f->policy->min);
		schedule_work(&l3_cpu);
	}
	return NOTIFY_OK;
}
static struct notifier_block l3_cpufreq_nb = { .notifier_call = l3_cpufreq_cb };

static int l3_show(struct seq_file *m, void *v)
{
	int i;

	mutex_lock(&l3_lock);
	l3_account();
	seq_printf(m, "%s, %s; OPP50 %llu s, OPP100 %llu s, %lu switches; holds",
		   clk_get_rate(l3_m2) == l3_low ? "OPP50" : "OPP100",
		   (const char *[]){ "auto", "low", "high" }[l3_forced],
		   div_u64(l3_ms[1], 1000), div_u64(l3_ms[0], 1000), l3_switches);
	for (i = 0; i < OMAP3_L3_USERS; i++)
		seq_printf(m, " %s%s %d", test_bit(i, &l3_ignored) ? "-" : "", l3_names[i], l3_holds[i]);
	seq_putc(m, '\n');
	mutex_unlock(&l3_lock);
	return 0;
}

static int l3_open(struct inode *inode, struct file *file)
{
	return single_open(file, l3_show, NULL);
}

/*
 * For tests: "low" or "high" pins the level (even with the SGX on), "auto" gives it back; "-usb" stops
 * counting a user's holds (the CPU, USB or WLAN; not the SGX's or the display's, whose clocks must not
 * change while they run), "+usb" counts them again
 */
static ssize_t l3_write(struct file *file, const char __user *buf, size_t len, loff_t *ppos)
{
	char s[8] = {};
	int f, i;

	if (copy_from_user(s, buf, min(len, sizeof(s) - 1)))
		return -EFAULT;
	f = sysfs_streq(s, "auto") ? 0 : sysfs_streq(s, "low") ? 1 : sysfs_streq(s, "high") ? 2 : -1;
	i = (s[0] == '-' || s[0] == '+') ? sysfs_match_string(l3_names, s + 1) : -EINVAL;
	if (f < 0 && i < 0)
		return -EINVAL;
	if (f < 0 && (i == OMAP3_L3_SGX || i == OMAP3_L3_DSS))
		return -EPERM;
	mutex_lock(&l3_lock);
	if (f >= 0)
		l3_forced = f;
	else if (s[0] == '-')
		set_bit(i, &l3_ignored);
	else
		clear_bit(i, &l3_ignored);
	l3_apply();
	mutex_unlock(&l3_lock);
	return len;
}

static const struct file_operations l3_fops = {
	.open = l3_open, .read = seq_read, .llseek = seq_lseek, .release = single_release,
	.write = l3_write,
};

static int __init omap3_l3_policy_init(void)
{
	struct cpufreq_policy *policy;
	struct clk *m2, *dpll3;

	/* SDRAM timings for both rates come from the board (sdram-nokia.c for the N9 and N950) */
	if (!of_machine_is_compatible("nokia,omap3-n9") && !of_machine_is_compatible("nokia,omap3-n950"))
		return 0;
	m2 = clk_get(NULL, "dpll3_m2_ck");
	if (IS_ERR(m2))
		return PTR_ERR(m2);
	/*
	 * DM3730 erratum 1.85 (SPRZ319F): with DPLL3 in automatic mode, a wakeup while it goes to bypass
	 * (CORE retention) can release the SDRC from idle on the bypass clock before DPLL3 locks.
	 * Workaround 1: DPLL3 always locked (CM_AUTOIDLE_PLL AUTO_CORE_DPLL 0). Stock instead allowed
	 * automatic mode only around CORE retention, at L3 OPP50 and VDD2 OPP100 (workaround 2), which
	 * also stops DPLL3 in retention: worth it once CORE reaches retention at all
	 */
	dpll3 = clk_get(NULL, "dpll3_ck");
	if (!IS_ERR(dpll3))
		omap2_clk_deny_idle(dpll3);
	l3_high = clk_get_rate(m2);
	l3_low = l3_high / 2;
	policy = cpufreq_cpu_get(0);
	l3_cpu_high = !policy || policy->cur > policy->min;
	if (policy)
		cpufreq_cpu_put(policy);
	mutex_lock(&l3_lock);
	l3_m2 = m2;
	l3_since = jiffies;
	mutex_unlock(&l3_lock);
	schedule_work(&l3_cpu);
	mod_delayed_work(system_wq, &l3_lower, msecs_to_jiffies(l3_lower_delay_ms));
	cpufreq_register_notifier(&l3_cpufreq_nb, CPUFREQ_TRANSITION_NOTIFIER);
	debugfs_create_file("omap3_l3", 0600, NULL, NULL, &l3_fops);
	pr_info("omap3-l3: L3 scaling, OPP100 %lu Hz, OPP50 %lu Hz (dpll3_m2_ck)\n", l3_high, l3_low);
	return 0;
}
late_initcall_sync(omap3_l3_policy_init);
