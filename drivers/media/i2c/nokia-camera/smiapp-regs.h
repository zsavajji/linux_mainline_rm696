/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Register lists and register access for Nokia's SMIA/SMIA++ camera module driver
 * (Harmattan's include/media/smiapp-regs.h)
 *
 * Copyright (C) 2010 Nokia Corporation
 */

#ifndef SMIAPP_REGS_H
#define SMIAPP_REGS_H

#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <media/v4l2-subdev.h>

#define SMIA_MAGIC			0x531A0002

struct smia_mode {
	/* Physical sensor resolution and current image window */
	__u16 sensor_width;
	__u16 sensor_height;
	__u16 sensor_window_origin_x;
	__u16 sensor_window_origin_y;
	__u16 sensor_window_width;
	__u16 sensor_window_height;

	/* Image data coming from sensor (after scaling) */
	__u16 width;
	__u16 height;
	__u16 window_origin_x;
	__u16 window_origin_y;
	__u16 window_width;
	__u16 window_height;

	__u32 pixel_clock;		/* in Hz */
	__u32 opsys_clock;		/* in Hz */
	__u32 ext_clock;		/* in Hz */
	struct v4l2_fract timeperframe;
	__u32 max_exp;			/* Maximum exposure value */
	__u32 pixel_format;		/* V4L2_PIX_FMT_xxx */
	__u32 sensitivity;		/* 16.16 fixed point */
};

#define SMIA_REG_8BIT			1
#define SMIA_REG_16BIT			2
#define SMIA_REG_32BIT			4
#define SMIA_REG_DELAY			100
#define SMIA_REG_TERM			0xff
struct smia_reg {
	u16 type;
	u16 reg;			/* 16-bit offset */
	u32 val;			/* 8/16/32-bit value */
};

/* Possible struct smia_reglist types. */
#define SMIA_REGLIST_STANDBY		0
#define SMIA_REGLIST_POWERON		1
#define SMIA_REGLIST_RESUME		2
#define SMIA_REGLIST_STREAMON		3
#define SMIA_REGLIST_STREAMOFF		4
#define SMIA_REGLIST_DISABLED		5

#define SMIA_REGLIST_MODE		10

#define SMIA_REGLIST_LSC_ENABLE		100
#define SMIA_REGLIST_LSC_DISABLE	101
#define SMIA_REGLIST_ANR_ENABLE		102
#define SMIA_REGLIST_ANR_DISABLE	103

struct smia_reglist {
	u32 type;
	struct smia_mode mode;
	struct smia_reg regs[];
};

#define SMIA_MAX_LEN			32
struct smia_meta_reglist {
	u32 magic;
	char version[SMIA_MAX_LEN];
	/*
	 * Offsets of the register lists from the start of the firmware, 32-bit, NULL-terminated;
	 * smia_reglist_import() turns them into pointers (the kernel is 32-bit)
	 */
	union {
		u32 offset;
		struct smia_reglist *ptr;
	} reglist[];
};

int smia_reglist_import(struct device *dev, struct smia_meta_reglist *meta);
struct smia_reglist *smia_reglist_find_type(struct smia_meta_reglist *meta,
					    u16 type);
struct smia_reglist **smia_reglist_first(struct smia_meta_reglist *meta);
struct smia_reglist *smia_reglist_find_mode_fmt(struct smia_meta_reglist *meta,
						struct v4l2_mbus_framefmt *fmt);
struct smia_reglist *smia_reglist_find_mode_ival(
	struct smia_meta_reglist *meta,
	struct smia_reglist *current_reglist,
	struct v4l2_fract *timeperframe);
int smia_reglist_enum_mbus_code(struct smia_meta_reglist *meta,
				struct v4l2_subdev_mbus_code_enum *code);
int smia_reglist_enum_frame_size(struct smia_meta_reglist *meta,
				 struct v4l2_subdev_frame_size_enum *fse);
int smia_reglist_enum_frame_ival(struct smia_meta_reglist *meta,
				 struct v4l2_subdev_frame_interval_enum *fie);
void smia_reglist_to_mbus(const struct smia_reglist *reglist,
			  struct v4l2_mbus_framefmt *fmt);

int smia_i2c_read_reg(struct i2c_client *client, u16 data_length,
		      u16 reg, u32 *val);
int smia_i2c_write_reg(struct i2c_client *client, u16 data_length, u16 reg,
		       u32 val);
int smia_i2c_write_regs(struct i2c_client *client,
			const struct smia_reg reglist[]);
int smia_i2c_reglist_find_write(struct i2c_client *client,
				struct smia_meta_reglist *meta, u16 type);

#endif
