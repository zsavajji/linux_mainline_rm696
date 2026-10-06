/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Harmattan's AS3645A flash interfaces: its include/linux/as3645a.h (the status event its camera
 * userspace subscribes to) and include/media/as3645a.h (the board data)
 */
#ifndef __AS3645A_H__
#define __AS3645A_H__

#include <linux/types.h>
#include <linux/videodev2.h>

#define V4L2_EVENT_AS3645A_CLASS	(V4L2_EVENT_PRIVATE_START | 0x200)
#define V4L2_EVENT_STATUS_CHANGED	(V4L2_EVENT_AS3645A_CLASS | 1)

enum v4l2_flash_events {
	V4L2_FLASH_READY = 0,
	V4L2_FLASH_COOLDOWN = 1,
};

#define AS3645A_NAME				"as3645a"
#define AS3645A_I2C_ADDR			(0x60 >> 1) /* W:0x60, R:0x61 */


struct as3645a_flash_torch_parms {
	unsigned int flash_min_current;
	unsigned int flash_max_current;
	unsigned int torch_min_current;
	unsigned int torch_max_current;
	unsigned int timeout_min;
	unsigned int timeout_max;
};

struct as3645a_platform_data {
	int (*set_power)(void *unused, int on);
	/* used to notify the entity which trigger external storbe signal */
	void (*setup_ext_strobe)(int enable);
	/* Sends the strobe width to the sensor strobe configuration */
	void (*set_strobe_width)(u32 width_in_us);
	/* positive value if Torch pin is used */
	int ext_torch;
	/* positive value if Flash Strobe pin is used for triggering
	 * the Flash light (no matter where is connected to, host processor or
	 * image sensor)
	 */
	int use_ext_flash_strobe;
	/* Number of attached LEDs, 1 or 2 */
	int num_leds;
	/* LED limitations with this flash chip */
	struct as3645a_flash_torch_parms *flash_torch_limits;
};


#endif /* __AS3645A_H__ */
