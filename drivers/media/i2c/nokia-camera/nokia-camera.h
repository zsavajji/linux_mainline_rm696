/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The Nokia N9's camera board glue, shared by Harmattan's sensor (smiapp), lens (ad58xx) and
 * flash (as3645a) drivers: what stock's arch/arm/mach-omap2/board-rm696-camera.c and
 * drivers/media/video/smiapp-power.c did. Implemented in smiapp-core.c.
 */

#ifndef NOKIA_CAMERA_H
#define NOKIA_CAMERA_H

#include <linux/types.h>

struct regulator;

/*
 * Both sensors answer at I2C address 0x10 until the main one is moved to 0x37, so the users of
 * the ISP's XCLKA (the main sensor and its lens, which shares the sensor's supply and clock)
 * and of XCLKB (the front sensor) are never on together: a claim of one while the other is
 * held fails with -EBUSY, as stock's rm696_update_xclk()
 */
#define NOKIA_CAMERA_XCLKA	0
#define NOKIA_CAMERA_XCLKB	1
int nokia_camera_claim(unsigned int xclk);
void nokia_camera_release(unsigned int xclk);

/* VANA (shared by all three camera modules) on/off, kept off at least 10 ms between users */
int nokia_camera_set_vana(struct regulator *vana, bool on);

/* Whether the main camera module identified as manu_id/model_id (stock's board checks) */
bool nokia_camera_main_module_is(u32 manu_id, u32 model_id);

/* The flash driver's hooks into the main sensor's flash strobe (stock's board callbacks) */
void nokia_camera_setup_ext_strobe(int enable);
void nokia_camera_set_strobe_width(u32 width_us);

#endif
