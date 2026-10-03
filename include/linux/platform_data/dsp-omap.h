#ifndef __OMAP_DSP_H__
#define __OMAP_DSP_H__

#include <linux/types.h>

/**
 * struct omap_dsp_platform_data - OMAP3 IVA2 glue for the DSP bridge
 *
 * mach-omap2 attaches this to the "ti,omap3-dspbridge" device (OF auxdata):
 * the PRCM and the system control module are private to mach-omap2. All
 * register accessors and @set_bootaddr/@set_bootmode are required.
 *
 * @dsp_prm_read:	read a PRM register (module offset, register offset),
 *			like omap2_prm_read_mod_reg()
 * @dsp_prm_write:	write a PRM register (value, module, register)
 * @dsp_prm_rmw_bits:	read-modify-write a PRM register (mask, bits, module,
 *			register), returns the new value
 * @dsp_cm_read:	@dsp_prm_read for the CM
 * @dsp_cm_write:	@dsp_prm_write for the CM
 * @dsp_cm_rmw_bits:	@dsp_prm_rmw_bits for the CM
 *
 * The bridge only accesses IVA2 module (OMAP3430_IVA2_MOD) registers:
 * RM_RSTCTRL_IVA2 (reset lines), PM_PWSTCTRL/PM_PWSTST_IVA2 (power state),
 * CM_CLKSTCTRL_IVA2 (clock domain transitions), CM_CLKEN_PLL_IVA2 and
 * CM_AUTOIDLE_PLL_IVA2 (DPLL2).
 *
 * @set_bootaddr:	write CONTROL_IVA2_BOOTADDR (the DSP's reset vector)
 * @set_bootmode:	write CONTROL_IVA2_BOOTMOD (0: direct boot, 1: idle)
 *
 * @dsp_set_min_opp, @dsp_get_opp, @cpu_set_freq, @cpu_get_freq, @mpu_speed:
 *	DVFS hooks, only used with CONFIG_TIDSPBRIDGE_DVFS. Optional (NULL).
 */
struct omap_dsp_platform_data {
	void (*dsp_set_min_opp) (u8 opp_id);
	u8 (*dsp_get_opp) (void);
	void (*cpu_set_freq) (unsigned long f);
	unsigned long (*cpu_get_freq) (void);
	unsigned long mpu_speed[6];

	/* functions to write and read PRCM registers */
	void (*dsp_prm_write)(u32, s16 , u16);
	u32 (*dsp_prm_read)(s16 , u16);
	u32 (*dsp_prm_rmw_bits)(u32, u32, s16, s16);
	void (*dsp_cm_write)(u32, s16 , u16);
	u32 (*dsp_cm_read)(s16 , u16);
	u32 (*dsp_cm_rmw_bits)(u32, u32, s16, s16);

	void (*set_bootaddr)(u32);
	void (*set_bootmode)(u8);
};

#endif
