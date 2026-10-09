// SPDX-License-Identifier: GPL-2.0-only
/*
 * linux/arch/arm/mach-omap2/common.c
 *
 * Code common to all OMAP2+ machines.
 *
 * Copyright (C) 2009 Texas Instruments
 * Copyright (C) 2010 Nokia Corporation
 * Tony Lindgren <tony@atomide.com>
 * Added OMAP4 support - Santosh Shilimkar <santosh.shilimkar@ti.com>
 */
#include <linux/kernel.h>
#include <linux/init.h>

#include "common.h"
#include "omap-secure.h"

void __init omap_reserve(void)
{
	omap_barrier_reserve_memblock();
}
