// SPDX-License-Identifier: GPL-2.0-only
/*
 * Generic driver for SMIA/SMIA++ compliant camera modules: Nokia's (Harmattan's
 * drivers/media/video/smiapp-core.c), moved to Linux 6.18 for the N9's camera userspace, which
 * finds the sensor by its name ("smiapp-00N"), reads its sysfs (ident, mode, nvm) and its mode
 * controls, and whose tuning is calibrated against the modes of the sensor firmware
 * (/lib/firmware/smiapp-sensor-*.bin). Also the N9's board glue (stock's board-rm696-camera.c
 * and smiapp-power.c, nokia-camera.h), which the device tree can't express.
 *
 * Copyright (C) 2010 Nokia Corporation
 * Contact: Vimarsh Zutshi <vimarsh.zutshi@nokia.com>
 *
 * Based on jt8ev1.c by Vimarsh Zutshi <vimarsh.zutshi@nokia.com>
 * Based on smia-sensor.c by Tuukka Toivonen <tuukka.o.toivonen@nokia.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#include "nokia-camera.h"
#include "smiapp.h"

/*
 * smiapp_module_idents - supported camera modules
 */
static const struct smiapp_module_ident smiapp_module_idents[] = {
	{ 0x10,	0x4141,	"jt8ev1 2-0037" },
	{ 0x01,	0x022b,	"vs6555", },
	{ 0x0c,	0x208a,	"tcm8330md", },
	{ 0x01,	0x022e,	"vw6558", },
	{ 0x0c,	0x2134,	"tcm8500md", },
	{ 0x07, 0x7698, "ovm7698", },
	{ 0x0b,	0x4242,	"smiapp-003", },
	{ 0x0c,	0x560f,	"smiapp-004", },
	{ 0x0c,	0x213e,	"et8en2", },
	{ 0x0c,	0x2184,	"tcm8580md", },
};

/* -----------------------------------------------------------------------------
 * The N9's board glue (stock's board-rm696-camera.c and smiapp-power.c)
 */

static DEFINE_MUTEX(nokia_camera_lock);
static unsigned int nokia_camera_users[2];

int nokia_camera_claim(unsigned int xclk)
{
	int ret = 0;

	mutex_lock(&nokia_camera_lock);
	if (nokia_camera_users[!xclk])
		ret = -EBUSY;
	else
		nokia_camera_users[xclk]++;
	mutex_unlock(&nokia_camera_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(nokia_camera_claim);

void nokia_camera_release(unsigned int xclk)
{
	mutex_lock(&nokia_camera_lock);
	WARN_ON(!nokia_camera_users[xclk]);
	nokia_camera_users[xclk]--;
	mutex_unlock(&nokia_camera_lock);
}
EXPORT_SYMBOL_GPL(nokia_camera_release);

#define SMIAPP_POWER_DELAY_MS	10

static ktime_t nokia_camera_vana_off;

int nokia_camera_set_vana(struct regulator *vana, bool on)
{
	int rval;

	mutex_lock(&nokia_camera_lock);
	if (on) {
		/* Off for at least SMIAPP_POWER_DELAY_MS since its last user let go */
		if (!regulator_is_enabled(vana)) {
			s64 left = SMIAPP_POWER_DELAY_MS * 1000 -
				ktime_us_delta(ktime_get(), nokia_camera_vana_off);

			if (left > 0)
				fsleep(left);
		}
		rval = regulator_enable(vana);
	} else {
		rval = regulator_disable(vana);
		if (!regulator_is_enabled(vana))
			nokia_camera_vana_off = ktime_get();
	}
	mutex_unlock(&nokia_camera_lock);

	return rval;
}
EXPORT_SYMBOL_GPL(nokia_camera_set_vana);

static struct smiapp_flash_strobe_parms rm696_main_camera_strobe_setup = {
	.mode			= 0x0c,
	.strobe_width_high_us	= 100000,
	.strobe_delay		= 0,
	.stobe_start_point	= 0,
	.trigger		= 0,
};

void nokia_camera_setup_ext_strobe(int enable)
{
	rm696_main_camera_strobe_setup.trigger = !!enable;
}
EXPORT_SYMBOL_GPL(nokia_camera_setup_ext_strobe);

void nokia_camera_set_strobe_width(u32 width_in_us)
{
	rm696_main_camera_strobe_setup.strobe_width_high_us = width_in_us;
}
EXPORT_SYMBOL_GPL(nokia_camera_set_strobe_width);

/*
 * Lada TS samples don't have their own sensors in there yet.
 * Lada TS Toshiba has Toshiba Gambino Sensor
 * Lada TS Fujinon has Sony Gambino Sensor
 *
 * Moreover the SMIA++ registers of model_id and manufacturer_id are
 * not present on the modules yet. Instead we have to use sensor model id
 * and sensor manufacturer id for identification. This should hopefully
 * get resolved with the ES samples.
 *
 * For now provide the translation to the driver from sensor idents to
 * module idents
 *
 * Lada ES1.0 Fujinon has Fujinon Lada Sensor but the SMIA++ registers
 * are still missing.
 *
 * Lada ES1.0 Toshiba has Toshiba Lada sensor and SMIA++ registers are
 * programmed as well
 */
static const struct smiapp_module_ident rm696_main_camera_idents[] = {
	{
		.sensor_manu_id		= 0x0b,
		.sensor_model_id	= 0x0088,
		.name			= "smiapp-001",
	},  /* Sony Gambino - imx088es */
	{
		.sensor_manu_id		= 0x0c,
		.sensor_model_id	= 0x218e,
		.name			= "smiapp-002",
	}, /* Toshiba Gambino - tcm8596md */
	{
		.sensor_manu_id		= 0x0b,
		.sensor_model_id	= 0x0125,
		.name			= "smiapp-003",
	}, /* Fujinon Lada - imx125 */
	{
		.manu_id		= 0x0c,
		.model_id		= 0x560f,
		.name			= "smiapp-004",
	}, /* Toshiba Lada - jt8ew9 */
};

static struct smiapp_module_ident rm696_main_camera;

static const struct smiapp_module_ident *
rm696_main_camera_identify_module(const struct smiapp_module_ident *ident_in)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(rm696_main_camera_idents); i++) {
		if (rm696_main_camera_idents[i].manu_id
		    && rm696_main_camera_idents[i].manu_id
		    == ident_in->manu_id
		    && rm696_main_camera_idents[i].model_id
		    == ident_in->model_id)
			break;
		else if (rm696_main_camera_idents[i].sensor_manu_id
			 == ident_in->sensor_manu_id
			 && rm696_main_camera_idents[i].sensor_model_id
			 == ident_in->sensor_model_id)
				break;
	}

	if (i >= ARRAY_SIZE(rm696_main_camera_idents))
		return NULL;

	/*
	 * if manu_id and model_id are missing, copy over the corresponding
	 * sensor_manu_id and sensor_model_id to keep the smiapp driver going
	 */
	rm696_main_camera = rm696_main_camera_idents[i];
	if (!rm696_main_camera.manu_id) {
		rm696_main_camera.manu_id = rm696_main_camera.sensor_manu_id;
		rm696_main_camera.model_id = rm696_main_camera.sensor_model_id;
	}

	return &rm696_main_camera;
}

bool nokia_camera_main_module_is(u32 manu_id, u32 model_id)
{
	return rm696_main_camera.manu_id == manu_id &&
	       rm696_main_camera.model_id == model_id;
}
EXPORT_SYMBOL_GPL(nokia_camera_main_module_is);

static int rm696_get_analog_gain_limits(const struct smiapp_module_ident *ident,
					u32 *min, u32 *max, u32 *step)
{
	if (ident->manu_id == 0x0c && ident->model_id == 0x218e) {
		/* Toshiba Gambino - tcm8596md */
		*min = 69;	/* Below 27 gain doesn't have effect at all, */
		*max = 1858;	/* but ~69 is needed for full dynamic range */
		*step = 1;
	} else if (ident->manu_id == 0x0c && ident->model_id == 0x560f) {
		/* Toshiba Lada - jt8ew9 */
		*min = 59;	/* Below 24 gain doesn't have effect at all, */
		*max = 6000;	/* but ~59 is needed for full dynamic range */
		*step = 1;
	} else
		return -ENOSYS;

	return 0;
}

static const struct smiapp_board rm696_main_camera_board = {
	.i2c_addr_dfl		= 0x10,
	.nvm_size		= 16 * 64,
	.xclk			= NOKIA_CAMERA_XCLKA,
	.strobe_setup		= &rm696_main_camera_strobe_setup,
	.identify_module	= rm696_main_camera_identify_module,
	.get_analog_gain_limits	= rm696_get_analog_gain_limits,
};

static const struct smiapp_board rm696_front_camera_board = {
	.xclk			= NOKIA_CAMERA_XCLKB,
};

/* The front camera's XSHUTDOWN is held low (stock's board init did it at boot) */
static bool rm696_front_camera_off;

/*
 * Calculate average pixel clock per line. Assume buffers can spread
 * the data over horizontal blanking time. Rounding upwards.
 * Tuukka Toivonen: board-rx51-camera.c (stock's rm696_calc_pixelclk(), which it passed to the
 * ISP as its pixel clock: here the sensor's V4L2_CID_PIXEL_RATE)
 */
static unsigned int smiapp_calc_pixelclk(const struct smia_mode *mode)
{
	static const int S = 8;
	unsigned int pixelclk;

	pixelclk = mode->window_width
		 * (((mode->pixel_clock + (1 << S) - 1) >> S) + mode->width - 1)
		 / mode->width;
	pixelclk <<= S;

	return pixelclk;
}

/*
 *
 * Dynamic Capability Identification
 *
 */
/*
 * Sensor Frame Structure
 */
static int smiapp_read_frame_fmt_type1(struct smiapp_sensor *sensor,
				       int ncold, int nrowd)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->subdev);
	u32 i, val;
	u8 *p;
	int rval;

	p = &sensor->sof_rows;
	for (i = ncold; i < ncold + nrowd; i++) {
		rval = smia_i2c_read_reg(client, SMIA_REG_16BIT,
			SMIAPP_REG_U8_FFMT_TYPE1_DESC_ZERO + (i * 2), &val);
		if (rval)
			return rval;

		if (((val >> 12) & 0xf) == 5) {
			p = &sensor->eof_rows;
			continue;	/* Image Data */
		}

		*p += val & 0x0fff;
	}

	return 0;
}

static int smiapp_read_frame_fmt_type2(struct smiapp_sensor *sensor,
				       int ncold, int nrowd)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->subdev);
	u32 i, val;
	u8 *p;
	int rval;

	p = &sensor->sof_rows;
	for (i = ncold; i < ncold + nrowd; i++) {
		rval = smia_i2c_read_reg(client, SMIA_REG_8BIT,
			SMIAPP_REG_U8_FFMT_TYPE2_DESC_ZERO + (i * 4), &val);
		if (rval)
			return rval;

		if (((val >> 4) & 0xf) == 5) {
			p = &sensor->eof_rows;
			continue;	/* Image Data */
		}

		rval = smia_i2c_read_reg(client, SMIA_REG_16BIT,
			SMIAPP_REG_U8_FFMT_TYPE2_DESC_ZERO + (i * 4) + 2, &val);
		if (rval)
			return rval;

		*p += val;
	}

	return 0;
}

static int smiapp_read_frame_fmt(struct smiapp_sensor *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->subdev);
	u32 dtype, ncold, nrowd, val;
	int rval;

	sensor->sof_rows = 0;
	sensor->eof_rows = 0;

	rval = smia_i2c_read_reg(client, SMIA_REG_8BIT,
				 SMIAPP_REG_U8_FFMT_MODEL_TYPE, &dtype);
	if (rval)
		return rval;

	rval = smia_i2c_read_reg(client, SMIA_REG_8BIT,
				 SMIAPP_REG_U8_FFMT_MODEL_STYPE, &val);
	if (rval)
		return rval;

	ncold = (val & 0xf0) >> 4;
	nrowd = (val & 0x0f);

	if (dtype == SMIAPP_FFMT_MODEL_TYPE1)
		return smiapp_read_frame_fmt_type1(sensor, ncold, nrowd);
	else if (dtype == SMIAPP_FFMT_MODEL_TYPE2)
		return smiapp_read_frame_fmt_type2(sensor, ncold, nrowd);
	else
		return -EINVAL;
}

/*
 *
 * V4L2 Controls handling
 *
 */
/*
 * Return time of one row in microseconds, .8 fixed point format.
 * If the sensor is not set to any mode, return zero.
 */
static int smiapp_get_row_time(struct smiapp_sensor *sensor)
{
	unsigned int clock;	/* Pixel clock in Hz>>10 fixed point */
	unsigned int rt;	/* Row time in .8 fixed point */

	if (!sensor->current_reglist)
		return 0;

	clock = sensor->current_reglist->mode.pixel_clock;
	clock = (clock + (1 << 9)) >> 10;
	rt = sensor->current_reglist->mode.width * (1000000 >> 2);
	rt = (rt + (clock >> 1)) / clock;

	return rt;
}

/*
 * Convert exposure time `us' to rows. Modify `us' to make it to
 * correspond to the actual exposure time.
 */
static int smiapp_exposure_us_to_rows(struct smiapp_sensor *sensor, u32 *us)
{
	unsigned int rows;	/* Exposure value as written to HW (ie. rows) */
	unsigned int rt;	/* Row time in .8 fixed point */

	/* Assume that the maximum exposure time is at most ~8 s,
	 * and the maximum width (with blanking) ~8000 pixels.
	 * The formula here is in principle as simple as
	 *    rows = exptime / 1e6 / width * pixel_clock
	 * but to get accurate results while coping with value ranges,
	 * have to do some fixed point math.
	 */

	rt = smiapp_get_row_time(sensor);
	rows = ((*us << 8) + (rt >> 1)) / rt;

	/* Set the exposure time to the rounded value */
	*us = (rt * rows + (1 << 7)) >> 8;

	return rows;
}

/*
 * Convert exposure time in rows to microseconds
 */
static int smiapp_exposure_rows_to_us(struct smiapp_sensor *sensor, int rows)
{
	return (smiapp_get_row_time(sensor) * rows + (1 << 7)) >> 8;
}

/* -----------------------------------------------------------------------------
 * V4L2 controls
 */

static int smiapp_get_ctrl(struct v4l2_ctrl *ctrl)
{
	struct smiapp_sensor *sensor =
		container_of(ctrl->handler, struct smiapp_sensor, ctrl_handler);
	const struct smia_mode *mode = &sensor->current_reglist->mode;

	switch (ctrl->id) {
	case V4L2_CID_MODE_FRAME_WIDTH:
		ctrl->val = mode->width;
		break;
	case V4L2_CID_MODE_FRAME_HEIGHT:
		ctrl->val = sensor->format.height +
			sensor->ctrls[SMIAPP_CTRL_VBLANK]->val;
		break;
	case V4L2_CID_MODE_VISIBLE_WIDTH:
		ctrl->val = mode->window_width;
		break;
	case V4L2_CID_MODE_VISIBLE_HEIGHT:
		ctrl->val = mode->window_height;
		break;
	case V4L2_CID_MODE_PIXELCLOCK:
		ctrl->val = mode->pixel_clock;
		break;
	case V4L2_CID_MODE_SENSITIVITY:
		ctrl->val = mode->sensitivity;
		break;
	case V4L2_CID_MODE_OPSYSCLOCK:
		ctrl->val = mode->opsys_clock;
		break;
	}

	return 0;
}

static void __smiapp_update_exposure_limits(struct smiapp_sensor *sensor)
{
	struct v4l2_ctrl *ctrl = sensor->ctrls[SMIAPP_CTRL_EXPOSURE];
	int max, step;

	max = smiapp_exposure_rows_to_us(
		sensor,
		sensor->format.height + sensor->ctrls[SMIAPP_CTRL_VBLANK]->val -
		sensor->margin);
	step = smiapp_exposure_rows_to_us(sensor, 1);

	__v4l2_ctrl_modify_range(ctrl, ctrl->minimum, max, max_t(int, step, 1),
				 min_t(s64, ctrl->default_value, max));
}

static int smiapp_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct smiapp_sensor *sensor =
		container_of(ctrl->handler, struct smiapp_sensor, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->subdev);
	u32 orient;
	int rows = 0;
	int rval = 0;

	if (ctrl->id == V4L2_CID_EXPOSURE)
		rows = smiapp_exposure_us_to_rows(sensor, (u32 *)&ctrl->val);

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		return smia_i2c_write_reg(client, SMIA_REG_16BIT,
					  SMIAPP_REG_U16_ANALOG_GAIN,
					  ctrl->val);

	case V4L2_CID_EXPOSURE:
		return smia_i2c_write_reg(client, SMIA_REG_16BIT,
					  SMIAPP_REG_U16_COARSE_INTEG_TIME,
					  rows);

	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		orient = 0;

		if (sensor->ctrls[SMIAPP_CTRL_HFLIP]->val)
			orient |= SMIAPP_IMAGE_ORIENT_HFLIP;

		if (sensor->ctrls[SMIAPP_CTRL_VFLIP]->val)
			orient |= SMIAPP_IMAGE_ORIENT_VFLIP;

		orient ^= sensor->hvflip_inv_mask;
		return smia_i2c_write_reg(client, SMIA_REG_8BIT,
					  SMIAPP_REG_U8_IMAGE_ORIENT, orient);

	case V4L2_CID_MODE_VBLANK:
		__smiapp_update_exposure_limits(sensor);

		rval = smiapp_set_ctrl(sensor->ctrls[SMIAPP_CTRL_EXPOSURE]);

		if (rval < 0)
			return rval;

		return smia_i2c_write_reg(client, SMIA_REG_16BIT,
					  SMIAPP_REG_U16_FRAME_LENGTH_LINES,
					  sensor->format.height + ctrl->val);

	case V4L2_CID_PIXEL_RATE:
	case V4L2_CID_LINK_FREQ:
		return 0;

	default:
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops smiapp_ctrl_ops = {
	.g_volatile_ctrl = smiapp_get_ctrl,
	.s_ctrl = smiapp_set_ctrl,
};

/* SMIA/SMIA++ mode controls */
static const struct v4l2_ctrl_config smiapp_mode_ctrls[] = {
	{
		.id		= V4L2_CID_MODE_CLASS,
		.type		= V4L2_CTRL_TYPE_CTRL_CLASS,
		.name		= "SMIA-type sensor information",
		.min		= 0,
		.max		= 0,
		.step		= 0,
		.def		= 0,
		.flags		= V4L2_CTRL_FLAG_READ_ONLY
				| V4L2_CTRL_FLAG_WRITE_ONLY,
	},
	{
		.ops		= &smiapp_ctrl_ops,
		.id		= V4L2_CID_MODE_FRAME_WIDTH,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Frame width",
		.min		= 0,
		.max		= 0,
		.step		= 1,
		.def		= 0,
		.flags		= V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
	},
	{
		.ops		= &smiapp_ctrl_ops,
		.id		= V4L2_CID_MODE_FRAME_HEIGHT,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Frame height",
		.min		= 0,
		.max		= 0,
		.step		= 1,
		.def		= 0,
		.flags		= V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
	},
	{
		.ops		= &smiapp_ctrl_ops,
		.id		= V4L2_CID_MODE_VISIBLE_WIDTH,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Visible width",
		.min		= 0,
		.max		= 0,
		.step		= 1,
		.def		= 0,
		.flags		= V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
	},
	{
		.ops		= &smiapp_ctrl_ops,
		.id		= V4L2_CID_MODE_VISIBLE_HEIGHT,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Visible height",
		.min		= 0,
		.max		= 0,
		.step		= 1,
		.def		= 0,
		.flags		= V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
	},
	{
		.ops		= &smiapp_ctrl_ops,
		.id		= V4L2_CID_MODE_PIXELCLOCK,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Pixel clock [Hz]",
		.min		= 0,
		.max		= 0,
		.step		= 1,
		.def		= 0,
		.flags		= V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
	},
	{
		.ops		= &smiapp_ctrl_ops,
		.id		= V4L2_CID_MODE_SENSITIVITY,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Sensivity",
		.min		= 0,
		.max		= 0,
		.step		= 1,
		.def		= 0,
		.flags		= V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
	},
	{
		.ops		= &smiapp_ctrl_ops,
		.id		= V4L2_CID_MODE_OPSYSCLOCK,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Output pixel clock [Hz]",
		.min		= 0,
		.max		= 0,
		.step		= 1,
		.def		= 0,
		.flags		= V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
	},
	{
		.ops		= &smiapp_ctrl_ops,
		.id		= V4L2_CID_MODE_VBLANK,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Vertical blanking lines",
		.min		= 0,
		.max		= 0,
		.step		= 1,
		.def		= 0,
		.flags		= V4L2_CTRL_FLAG_UPDATE,
	},
};

static int smiapp_update_controls(struct smiapp_sensor *sensor);

/*
 * The CSI-2 link frequencies of the firmware's modes (V4L2_CID_LINK_FREQ), from which the ISP
 * sets its receiver's timing as stock's board file did from the mode's output clock
 */
static int smiapp_init_link_freqs(struct smiapp_sensor *sensor)
{
	struct smia_reglist **list;
	unsigned int i;

	if (!sensor->lanes)
		return 0;

	for (list = smia_reglist_first(sensor->meta_reglist); *list; list++)
		sensor->nlink_freqs++;
	sensor->link_freqs = devm_kcalloc(sensor->subdev.dev, sensor->nlink_freqs,
					  sizeof(*sensor->link_freqs), GFP_KERNEL);
	if (!sensor->link_freqs)
		return -ENOMEM;

	sensor->nlink_freqs = 0;
	for (list = smia_reglist_first(sensor->meta_reglist); *list; list++) {
		s64 f = (*list)->mode.opsys_clock / (2 * sensor->lanes);

		if ((*list)->type != SMIA_REGLIST_MODE || !f)
			continue;
		for (i = 0; i < sensor->nlink_freqs; i++)
			if (sensor->link_freqs[i] == f)
				break;
		if (i == sensor->nlink_freqs)
			sensor->link_freqs[sensor->nlink_freqs++] = f;
	}

	return 0;
}

static int smiapp_init_controls(struct smiapp_sensor *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->subdev);
	u32 min, max, step;
	unsigned int i;
	int rval = -1;

	rval = smiapp_init_link_freqs(sensor);
	if (rval)
		return rval;
	rval = -1;

	v4l2_ctrl_handler_init(&sensor->ctrl_handler,
			       ARRAY_SIZE(smiapp_mode_ctrls) + SMIAPP_NCTRLS);

	if (sensor->board->get_analog_gain_limits)
		rval = sensor->board->get_analog_gain_limits(
			sensor->ident, &min, &max, &step);

	/* V4L2_CID_GAIN */
	if (rval) {
		rval = smia_i2c_read_reg(client, SMIA_REG_16BIT,
				SMIAPP_REG_U16_ANALOG_GAIN_MAX, &max);
		rval |= smia_i2c_read_reg(client, SMIA_REG_16BIT,
				SMIAPP_REG_U16_ANALOG_GAIN_MIN, &min);
		rval |= smia_i2c_read_reg(client, SMIA_REG_16BIT,
				SMIAPP_REG_U16_ANALOG_GAIN_STEP, &step);
		if (rval) {
			dev_info(&client->dev, "can't read AG range\n");
			/** FIXME **/
			min = 0;
			max = 0;
			step = 0;
		}
	}

	sensor->ctrls[SMIAPP_CTRL_ANALOG_GAIN] =
		v4l2_ctrl_new_std(&sensor->ctrl_handler, &smiapp_ctrl_ops,
				  V4L2_CID_GAIN, min, max, max(step, 1U), min);

	/* V4L2_CID_EXPOSURE */
	sensor->ctrls[SMIAPP_CTRL_EXPOSURE] =
		v4l2_ctrl_new_std(
			&sensor->ctrl_handler, &smiapp_ctrl_ops,
			V4L2_CID_EXPOSURE, 0, 1, 1, 1);

	/* V4L2_CID_HFLIP and V4L2_CID_VFLIP */
	sensor->ctrls[SMIAPP_CTRL_HFLIP] =
		v4l2_ctrl_new_std(&sensor->ctrl_handler, &smiapp_ctrl_ops,
				  V4L2_CID_HFLIP, 0, 1, 1, 0);
	sensor->ctrls[SMIAPP_CTRL_VFLIP] =
		v4l2_ctrl_new_std(&sensor->ctrl_handler, &smiapp_ctrl_ops,
				  V4L2_CID_VFLIP, 0, 1, 1, 0);

	/* What stock's board file told its ISP (the ISP reads these in 6.18) */
	sensor->ctrls[SMIAPP_CTRL_PIXEL_RATE] =
		v4l2_ctrl_new_std(&sensor->ctrl_handler, &smiapp_ctrl_ops,
				  V4L2_CID_PIXEL_RATE, 1, INT_MAX, 1, 1);
	if (sensor->nlink_freqs) {
		sensor->ctrls[SMIAPP_CTRL_LINK_FREQ] =
			v4l2_ctrl_new_int_menu(&sensor->ctrl_handler,
					       &smiapp_ctrl_ops, V4L2_CID_LINK_FREQ,
					       sensor->nlink_freqs - 1, 0,
					       sensor->link_freqs);
		if (sensor->ctrls[SMIAPP_CTRL_LINK_FREQ])
			sensor->ctrls[SMIAPP_CTRL_LINK_FREQ]->flags |=
				V4L2_CTRL_FLAG_READ_ONLY;
	}

	/* V4L2_CID_MODE_* */
	for (i = 0; i < ARRAY_SIZE(smiapp_mode_ctrls); ++i)
		v4l2_ctrl_new_custom(&sensor->ctrl_handler,
				     &smiapp_mode_ctrls[i], NULL);

	sensor->ctrls[SMIAPP_CTRL_VBLANK] =
		v4l2_ctrl_find(&sensor->ctrl_handler, V4L2_CID_MODE_VBLANK);

	if (sensor->ctrl_handler.error) {
		dev_err(&client->dev, "controls initialization failed (%d)\n",
			sensor->ctrl_handler.error);
		return sensor->ctrl_handler.error;
	}

	v4l2_ctrl_cluster(2, &sensor->ctrls[SMIAPP_CTRL_HFLIP]);

	sensor->subdev.ctrl_handler = &sensor->ctrl_handler;

	return smiapp_update_controls(sensor);
}

static void smiapp_free_controls(struct smiapp_sensor *sensor)
{
	v4l2_ctrl_handler_free(&sensor->ctrl_handler);
}

static int __smiapp_update_vblank(struct smiapp_sensor *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->subdev);
	struct v4l2_ctrl *ctrl = sensor->ctrls[SMIAPP_CTRL_VBLANK];
	u32 max_frame_len;
	s32 min, max;
	int rval;

	rval = smia_i2c_read_reg(client, SMIA_REG_16BIT,
			SMIAPP_REG_U16_COARSE_INTEGRATION_TIME_MAX_MARGIN,
			&sensor->margin);
	if (rval < 0)
		return rval;

	rval = smia_i2c_read_reg(client, SMIA_REG_16BIT,
				 SMIAPP_REG_U16_MAX_FRAME_LENGTH_LINES_BIN,
				 &max_frame_len);
	if (rval < 0)
		return rval;

	min = sensor->current_reglist->mode.height - sensor->format.height;
	if (max_frame_len >= sensor->current_reglist->mode.height
	    - sensor->format.height)
		max = max_frame_len - sensor->format.height;
	else
		max = min;

	rval = __v4l2_ctrl_modify_range(ctrl, min, max, 1, min);
	if (rval < 0)
		return rval;

	/* Each mode starts at its own blanking, as stock */
	ctrl->val = min;
	ctrl->cur.val = min;

	return 0;
}

/* The mode's pixel rate and link frequency (stock's csi_configure board callback) */
static void __smiapp_update_link(struct smiapp_sensor *sensor)
{
	const struct smia_mode *mode = &sensor->current_reglist->mode;
	unsigned int i;

	__v4l2_ctrl_s_ctrl_int64(sensor->ctrls[SMIAPP_CTRL_PIXEL_RATE],
				 smiapp_calc_pixelclk(mode));

	if (!sensor->ctrls[SMIAPP_CTRL_LINK_FREQ])
		return;
	for (i = 0; i < sensor->nlink_freqs; i++)
		if (sensor->link_freqs[i] == mode->opsys_clock / (2 * sensor->lanes))
			__v4l2_ctrl_s_ctrl(sensor->ctrls[SMIAPP_CTRL_LINK_FREQ], i);
}

static int smiapp_update_controls(struct smiapp_sensor *sensor)
{
	struct v4l2_ctrl *ctrl = sensor->ctrls[SMIAPP_CTRL_VBLANK];
	int rval;

	v4l2_ctrl_lock(ctrl);
	__smiapp_update_link(sensor);
	rval = __smiapp_update_vblank(sensor);
	if (rval < 0)
		goto out;

	rval = smiapp_set_ctrl(ctrl);

out:
	v4l2_ctrl_unlock(ctrl);

	return rval;
}

/*
 *
 * SMIA++ NVM handling
 *
 */
static int smiapp_read_nvm(struct smiapp_sensor *sensor,
			   unsigned char *nvm)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->subdev);
	u32 i, s, p, np, v;
	int rval;

	np = sensor->nvm_size / SMIAPP_NVM_PAGE_SIZE;
	for (p = 0; p < np; p++) {
		rval = smia_i2c_write_reg(client, SMIA_REG_8BIT,
					  SMIAPP_REG_U8_DATATX_IF1_PAGESEL, p);
		if (rval)
			goto out;

		rval = smia_i2c_write_reg(client, SMIA_REG_8BIT,
					  SMIAPP_REG_U8_DATATX_IF1_CTRL,
					  SMIAPP_DATATX_IFX_CTRL_EN |
					  SMIAPP_DATATX_IFX_CTRL_RD_EN);
		if (rval)
			goto out;

		i = 1000;
		do {
			rval = smia_i2c_read_reg(client, SMIA_REG_8BIT,
					SMIAPP_REG_U8_DATATX_IF1_STATUS, &s);

			if (rval)
				goto out;

			if (s & SMIAPP_DATATX_IFX_STATUS_RD_READY)
				break;

			if (--i == 0)
				goto out;

		} while (1);

		for (i = 0; i < SMIAPP_NVM_PAGE_SIZE; i++) {
			rval = smia_i2c_read_reg(client, SMIA_REG_8BIT,
					SMIAPP_REG_U8_DATATX_IF1_DATA_BASE + i,
					&v);
			if (rval)
				goto out;

			*nvm++ = v;
		}
	}

out:
	rval |= smia_i2c_write_reg(client, SMIA_REG_8BIT,
				  SMIAPP_REG_U8_DATATX_IF1_CTRL, 0);
	return rval;
}

/*
 *
 * SMIA++ CCI address control
 *
 */
static int smiapp_change_cci_addr(struct smiapp_sensor *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->subdev);
	unsigned short alt = client->addr;
	int rval;
	u32 val;

	client->addr = sensor->board->i2c_addr_dfl;

	rval = smia_i2c_write_reg(client, SMIA_REG_8BIT, SMIAPP_REG_U8_CCI_ADDR,
				  alt << 1);
	client->addr = alt;
	if (rval)
		return rval;

	/* verify addr change went ok */
	rval = smia_i2c_read_reg(client, SMIA_REG_8BIT,
				 SMIAPP_REG_U8_CCI_ADDR, &val);
	if (rval)
		return rval;

	if (val != alt << 1)
		return -ENODEV;

	return 0;
}

/*
 *
 * SMIA++ Mode Control
 *
 */
static int smiapp_setup_flash_strobe(struct smiapp_sensor *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->subdev);
	struct smiapp_flash_strobe_parms *strobe_setup;
	unsigned int ext_freq = sensor->ext_clk_freq;
	int rval;
	u32 tmp;
	u32 strobe_adjustment;
	u32 strobe_width_high_rs;

	strobe_setup = sensor->board->strobe_setup;

	/*
	 * How to calculate registers related to strobe length. Please
	 * do not change, or if you do at least know what you're
	 * doing. :-)
	 *
	 * Sakari Ailus <sakari.ailus@maxwell.research.nokia.com> 2010-10-25
	 *
	 * flash_strobe_length [us] / 10^6 = (tFlash_strobe_width_ctrl
	 *	/ EXTCLK freq [Hz]) * flash_strobe_adjustment
	 *
	 * tFlash_strobe_width_ctrl E N, [1 - 0xffff]
	 * flash_strobe_adjustment E N, [1 - 0xff]
	 *
	 * The formula above is written as below to keep it on one
	 * line:
	 *
	 * l / 10^6 = w / e * a
	 *
	 * Let's mark w * a by x:
	 *
	 * x = w * a
	 *
	 * Thus, we get:
	 *
	 * x = l * e / 10^6
	 *
	 * The strobe width must be at least as long as requested,
	 * thus rounding upwards is needed.
	 *
	 * x = (l * e + 10^6 - 1) / 10^6
	 * -----------------------------
	 *
	 * Maximum possible accuracy is wanted at all times. Thus keep
	 * a as small as possible.
	 *
	 * Calculate a, assuming maximum w, with rounding upwards:
	 *
	 * a = (x + (2^16 - 1) - 1) / (2^16 - 1)
	 * -------------------------------------
	 *
	 * Thus, we also get w, with that a, with rounding upwards:
	 *
	 * w = (x + a - 1) / a
	 * -------------------
	 *
	 * To get limits:
	 *
	 * x E [1, (2^16 - 1) * (2^8 - 1)]
	 *
	 * Substituting maximum x to the original formula (with rounding),
	 * the maximum l is thus
	 *
	 * (2^16 - 1) * (2^8 - 1) * 10^6 = l * e + 10^6 - 1
	 *
	 * l = (10^6 * (2^16 - 1) * (2^8 - 1) - 10^6 + 1) / e
	 * --------------------------------------------------
	 *
	 * flash_strobe_length must be clamped between 1 and
	 * (10^6 * (2^16 - 1) * (2^8 - 1) - 10^6 + 1) / EXTCLK freq.
	 *
	 * Then,
	 *
	 * flash_strobe_adjustment = ((flash_strobe_length *
	 *	EXTCLK freq + 10^6 - 1) / 10^6 + (2^16 - 1) - 1) / (2^16 - 1)
	 *
	 * tFlash_strobe_width_ctrl = ((flash_strobe_length *
	 *	EXTCLK freq + 10^6 - 1) / 10^6 +
	 *	flash_strobe_adjustment - 1) / flash_strobe_adjustment
	 */
	tmp = div_u64(1000000ULL * ((1 << 16) - 1) * ((1 << 8) - 1) -
		      1000000 + 1, ext_freq);
	strobe_setup->strobe_width_high_us =
		clamp_t(u32, strobe_setup->strobe_width_high_us, 1, tmp);

	tmp = div_u64(((u64)strobe_setup->strobe_width_high_us * (u64)ext_freq +
			1000000 - 1), 1000000ULL);
	strobe_adjustment = (tmp + (1 << 16) - 1 - 1) / ((1 << 16) - 1);
	strobe_width_high_rs = (tmp + strobe_adjustment - 1) /
				strobe_adjustment;

	rval = smia_i2c_write_reg(client, SMIA_REG_8BIT, SMIAPP_FLASH_MODE_RS,
				  strobe_setup->mode);
	if (rval < 0)
		goto out;

	rval = smia_i2c_write_reg(client, SMIA_REG_8BIT,
				  SMIAPP_FLASH_STROBE_ADJUSTMENT,
				  strobe_adjustment);
	if (rval < 0)
		goto out;

	rval = smia_i2c_write_reg(client, SMIA_REG_16BIT,
				  SMIAPP_FLASH_STROBE_WIDTH_HIGH_RS_CTRL,
				  strobe_width_high_rs);
	if (rval < 0)
		goto out;

	rval = smia_i2c_write_reg(client, SMIA_REG_16BIT,
				  SMIAPP_FLASH_STROBE_DELAY_RS_CTRL,
				  strobe_setup->strobe_delay);
	if (rval < 0)
		goto out;

	rval = smia_i2c_write_reg(client, SMIA_REG_16BIT,
				  SMIAPP_FLASH_STROBE_START_POINT,
				  strobe_setup->stobe_start_point);
	if (rval < 0)
		goto out;

	rval = smia_i2c_write_reg(client, SMIA_REG_8BIT,
				  SMIAPP_FLASH_TRIGGER_RS,
				  strobe_setup->trigger);

out:
	sensor->board->strobe_setup->trigger = 0;

	return rval;
}

/* -----------------------------------------------------------------------------
 * Power management
 */

static int smiapp_power_on(struct smiapp_sensor *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->subdev);
	int rval;

	/* Stock's set_xclk() board callback refused a clock while the other camera's was on */
	rval = nokia_camera_claim(sensor->board->xclk);
	if (rval) {
		dev_dbg(&client->dev, "failed to set xclk\n");
		return rval;
	}

	rval = nokia_camera_set_vana(sensor->vana, 1);
	if (rval) {
		dev_err(&client->dev, "failed to enable vana regulator\n");
		goto out_vana_fail;
	}
	mdelay(1);

	rval = clk_prepare_enable(sensor->ext_clk);
	if (rval < 0) {
		dev_dbg(&client->dev, "failed to set xclk\n");
		goto out_xclk_fail;
	}
	mdelay(1);

	if (sensor->xshutdown) {
		gpiod_set_value_cansleep(sensor->xshutdown, 0);
		mdelay(3);
	}

	/*
	 * Failures to respond to the address change command have been noticed.
	 * Those failures seem to be caused by the sensor requiring a longer
	 * boot time than advertised. An additional 10ms delay seems to work
	 * around the issue, but the SMIA++ I2C write retry hack makes the delay
	 * unnecessary. The failures need to be investigated to find a proper
	 * fix, and a delay will likely need to be added here if the I2C write
	 * retry hack is reverted before the root cause of the boot time issue
	 * is found.
	 */

	if (sensor->board->i2c_addr_dfl) {
		rval = smiapp_change_cci_addr(sensor);
		if (rval) {
			dev_err(&client->dev, "cci address change error\n");
			goto out_cci_addr_fail;
		}
	}

	if (sensor->meta_reglist) {
		rval = smia_i2c_reglist_find_write(client, sensor->meta_reglist,
						   SMIA_REGLIST_POWERON);
		/*
		 * we will lose new CCI address after Software reset
		 * issued by SMIA_REGLIST_POWERON.
		 *
		 * TODO: remove it if software reset is dropped from firmware
		 */
		if (sensor->board->i2c_addr_dfl) {
			rval = smiapp_change_cci_addr(sensor);
			if (rval) {
				dev_err(&client->dev,
					"cci address change error\n");
				goto out_cci_addr_fail;
			}
		}
	}

	return 0;

out_cci_addr_fail:
	if (sensor->xshutdown)
		gpiod_set_value_cansleep(sensor->xshutdown, 1);
	clk_disable_unprepare(sensor->ext_clk);
out_xclk_fail:
	nokia_camera_set_vana(sensor->vana, 0);
out_vana_fail:
	nokia_camera_release(sensor->board->xclk);
	return rval;
}

static void smiapp_power_off(struct smiapp_sensor *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->subdev);

	/*
	 * Currently power/clock to lens are enable/disabled separately
	 * but they are essentially the same signals. So if the sensor is
	 * powered off while the lens is powered on the sensor does not
	 * really see a power off and next time the cci address change
	 * will fail. So do a soft reset explicitly here.
	 */
	if (sensor->board->i2c_addr_dfl)
		smia_i2c_write_reg(client, SMIA_REG_8BIT,
				   SMIAPP_REG_U8_SOFT_RESET, 1);

	if (sensor->xshutdown)
		gpiod_set_value_cansleep(sensor->xshutdown, 1);
	clk_disable_unprepare(sensor->ext_clk);
	mdelay(5);
	nokia_camera_set_vana(sensor->vana, 0);
	nokia_camera_release(sensor->board->xclk);
	sensor->streaming = 0;
}

static int smiapp_set_power(struct v4l2_subdev *subdev, int on)
{
	struct smiapp_sensor *sensor = to_smiapp_sensor(subdev);
	int ret = 0;

	mutex_lock(&sensor->power_lock);

	/* If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (sensor->power_count == !on) {
		if (on)
			ret = smiapp_power_on(sensor);
		else
			smiapp_power_off(sensor);
		if (ret < 0)
			goto done;
	}

	/* Update the power count. */
	sensor->power_count += on ? 1 : -1;
	WARN_ON(sensor->power_count < 0);

done:
	mutex_unlock(&sensor->power_lock);
	return ret;
}

/* -----------------------------------------------------------------------------
 * Video stream management
 */

static int smiapp_start_streaming(struct smiapp_sensor *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->subdev);
	struct smia_reglist *post_streamon;
	int rval = 0;

	rval = smia_i2c_write_regs(client, sensor->current_reglist->regs);
	if (rval)
		goto fail_sensor_cfg;

	smiapp_update_controls(sensor);

	/* Controls set while the power to the sensor is turned off are saved
	 * but not applied to the hardware. Now that we're about to start
	 * streaming apply all the current values to the hardware.
	 */
	rval = v4l2_ctrl_handler_setup(&sensor->ctrl_handler);
	if (rval)
		goto fail_sensor_cfg;

	if ((sensor->flash_capability & (SMIAPP_FLASH_CAP_SINGLE_STROBE |
					 SMIAPP_FLASH_CAP_MULTIPLE_STROBE)) &&
	    sensor->board->strobe_setup != NULL &&
	    sensor->board->strobe_setup->trigger != 0) {
		rval = smiapp_setup_flash_strobe(sensor);
		if (rval)
			goto fail_sensor_cfg;
	}

	rval = smia_i2c_write_reg(client, SMIA_REG_8BIT, SMIAPP_REG_U8_MODE, 1);
	if (rval)
		goto fail_sensor_cfg;

	post_streamon = smia_reglist_find_type(sensor->meta_reglist,
						SMIA_REGLIST_STREAMON);
	if (!post_streamon)
		return 0;
	rval = smia_i2c_write_regs(client, post_streamon->regs);

	return rval;

fail_sensor_cfg:
	dev_err(&client->dev, "sensor configuration failed\n");
	return rval;
}

static int smiapp_stop_streaming(struct smiapp_sensor *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->subdev);
	struct smia_reglist *post_streamoff;
	int rval;

	rval = smia_i2c_write_reg(client, SMIA_REG_8BIT, SMIAPP_REG_U8_MODE, 0);
	if (rval)
		return rval;

	post_streamoff = smia_reglist_find_type(sensor->meta_reglist,
						SMIA_REGLIST_STREAMOFF);
	if (!post_streamoff)
		return 0;

	return smia_i2c_write_regs(client, post_streamoff->regs);
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev video operations
 */

static int smiapp_set_stream(struct v4l2_subdev *subdev, int enable)
{
	struct smiapp_sensor *sensor = to_smiapp_sensor(subdev);
	int rval;

	if (sensor->streaming == enable)
		return 0;

	if (enable)
		rval = smiapp_start_streaming(sensor);
	else
		rval = smiapp_stop_streaming(sensor);

	if (rval == 0)
		sensor->streaming = enable;

	return rval;
}

static int smiapp_enum_mbus_code(struct v4l2_subdev *subdev,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct smiapp_sensor *sensor = to_smiapp_sensor(subdev);

	return smia_reglist_enum_mbus_code(sensor->meta_reglist, code);
}

static int smiapp_enum_frame_size(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct smiapp_sensor *sensor = to_smiapp_sensor(subdev);

	return smia_reglist_enum_frame_size(sensor->meta_reglist, fse);
}

static int smiapp_enum_frame_ival(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_interval_enum *fie)
{
	struct smiapp_sensor *sensor = to_smiapp_sensor(subdev);

	return smia_reglist_enum_frame_ival(sensor->meta_reglist, fie);
}

static struct v4l2_mbus_framefmt *
__smiapp_get_pad_format(struct smiapp_sensor *sensor,
			struct v4l2_subdev_state *state,
			unsigned int pad, enum v4l2_subdev_format_whence which)
{
	if (pad != 0)
		return NULL;

	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_state_get_format(state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &sensor->format;
	default:
		return NULL;
	}
}

static int smiapp_get_pad_format(struct v4l2_subdev *subdev,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct smiapp_sensor *sensor = to_smiapp_sensor(subdev);
	struct v4l2_mbus_framefmt *format;

	format = __smiapp_get_pad_format(sensor, state, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	fmt->format = *format;
	return 0;
}

static int smiapp_set_pad_format(struct v4l2_subdev *subdev,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct smiapp_sensor *sensor = to_smiapp_sensor(subdev);
	struct v4l2_mbus_framefmt *format;
	struct smia_reglist *reglist;

	format = __smiapp_get_pad_format(sensor, state, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	reglist = smia_reglist_find_mode_fmt(sensor->meta_reglist,
					     &fmt->format);
	memset(&fmt->format, 0, sizeof(fmt->format));
	smia_reglist_to_mbus(reglist, &fmt->format);
	*format = fmt->format;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		sensor->current_reglist = reglist;
		return smiapp_update_controls(sensor);
	}

	return 0;
}

static int smiapp_get_frame_interval(struct v4l2_subdev *subdev,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct smiapp_sensor *sensor = to_smiapp_sensor(subdev);

	fi->interval = sensor->current_reglist->mode.timeperframe;

	return 0;
}

static int smiapp_set_frame_interval(struct v4l2_subdev *subdev,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct smiapp_sensor *sensor = to_smiapp_sensor(subdev);
	struct smia_reglist *reglist;

	/* Is userspace being stupid? */
	if (!fi->interval.denominator) {
		fi->interval.numerator = 1;
		fi->interval.denominator = 1;
	}

	reglist = smia_reglist_find_mode_ival(sensor->meta_reglist,
					      sensor->current_reglist,
					      &fi->interval);
	if (!reglist)
		return -EINVAL;

	sensor->current_reglist = reglist;

	return smiapp_update_controls(sensor);
}

static int smiapp_get_skip_top_lines(struct v4l2_subdev *subdev, u32 *lines)
{
	struct smiapp_sensor *sensor = to_smiapp_sensor(subdev);

	*lines = sensor->sof_rows;
	return 0;
}

static int smiapp_get_skip_frames(struct v4l2_subdev *subdev, u32 *frames)
{
	struct smiapp_sensor *sensor = to_smiapp_sensor(subdev);

	*frames = sensor->frame_skip;
	return 0;
}

/* -----------------------------------------------------------------------------
 * sysfs attributes
 */

static ssize_t
nvm_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(to_i2c_client(dev));
	struct i2c_client *client = v4l2_get_subdevdata(subdev);
	struct smiapp_sensor *sensor = to_smiapp_sensor(subdev);
	int nbytes;

	if (!sensor->dev_init_done)
		return -EBUSY;

	if (!sensor->nvm_size) {
		/* NVM not read yet - read it now */
		sensor->nvm_size = sensor->board->nvm_size;
		if (smiapp_set_power(subdev, 1) < 0) {
			sensor->nvm_size = 0;
			return -ENODEV;
		}
		if (smiapp_read_nvm(sensor, sensor->nvm)) {
			dev_err(&client->dev, "nvm read failed\n");
			smiapp_set_power(subdev, 0);
			sensor->nvm_size = 0;
			return -ENODEV;
		}
		smiapp_set_power(subdev, 0);
	}
	/*
	 * NVM is still way below a PAGE_SIZE, so we can safely
	 * assume this for now.
	 */
	nbytes = min_t(unsigned int, sensor->nvm_size, PAGE_SIZE);
	memcpy(buf, sensor->nvm, nbytes);

	return nbytes;
}
static DEVICE_ATTR_RO(nvm);

/*
 * sensor mode information
 * power state      : on/off, standby/streaming
 * sensor mode      : wxh, raw10/10dpcm8, fps, exp, vblank, sof, eof
 * active interface : csi2(nlanes)/ccp2 (fixme), csi2 ddrclk
 */
static ssize_t
mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(to_i2c_client(dev));
	struct smiapp_sensor *sensor = to_smiapp_sensor(subdev);
	int vblank;
	int exposure;
	char *fmt;

	if (!sensor->dev_init_done || !sensor->current_reglist)
		return -EBUSY;

	switch (sensor->current_reglist->mode.pixel_format) {
	case V4L2_PIX_FMT_SGRBG10DPCM8:
		fmt = "10dpcm8";
		break;
	case V4L2_PIX_FMT_SGRBG10:
		fmt = "raw10";
		break;
	default:
		fmt = "unknown";
	}

	vblank = smiapp_exposure_rows_to_us(sensor,
			sensor->current_reglist->mode.height -
			sensor->current_reglist->mode.window_height);

	exposure = v4l2_ctrl_g_ctrl(sensor->ctrls[SMIAPP_CTRL_EXPOSURE]);

	return sysfs_emit(buf,
		 "%s %s "
		 "%dx%d %s %d/%d %d %d %d %d "
		 "%d\n",
		  (sensor->power_count) ? "on" : "off",
		  (sensor->streaming) ? "streaming" : "standby",
		  sensor->current_reglist->mode.window_width,
		  sensor->current_reglist->mode.window_height, fmt,
		  sensor->current_reglist->mode.timeperframe.numerator,
		  sensor->current_reglist->mode.timeperframe.denominator,
		  exposure, vblank, sensor->sof_rows, sensor->eof_rows,
		  sensor->current_reglist->mode.opsys_clock/(1000 *  1000));
}
static DEVICE_ATTR_RO(mode);

/*
 * sensor ident information
 * sensor name : manufacturer id : model id
 */
static ssize_t
ident_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(to_i2c_client(dev));
	struct smiapp_sensor *sensor = to_smiapp_sensor(subdev);

	if (!sensor->dev_init_done)
		return -EBUSY;

	return sysfs_emit(buf, "%s %04x %04x %04x\n",
			  sensor->ident->name, sensor->ident->manu_id,
			  sensor->ident->model_id, sensor->rev_major);
}
static DEVICE_ATTR_RO(ident);

/*
 * Informational entries in sysfs
 * 1. mode
 * 2. ident
 */
static void smiapp_sysfs_entries_create(struct smiapp_sensor *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->subdev);

	if (device_create_file(&client->dev, &dev_attr_mode) != 0)
		dev_info(&client->dev, "sysfs mode entry failed\n");
	else
		sensor->sysfs_mode = 1;

	if (device_create_file(&client->dev, &dev_attr_ident) != 0)
		dev_info(&client->dev, "sysfs ident entry failed\n");
	else
		sensor->sysfs_ident = 1;
}

static void smiapp_sysfs_entries_remove(struct smiapp_sensor *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->subdev);

	if (sensor->sysfs_mode)
		device_remove_file(&client->dev, &dev_attr_mode);

	if (sensor->sysfs_ident)
		device_remove_file(&client->dev, &dev_attr_ident);
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev core operations
 */

static int smiapp_identify_module(struct v4l2_subdev *subdev)
{
	struct smiapp_sensor *sensor = to_smiapp_sensor(subdev);
	struct i2c_client *client = v4l2_get_subdevdata(subdev);
	int i, rval = 0;
	struct smiapp_module_ident sid = { };
	u32 smia_ver, smiapp_ver, rev_major;

	rval |= smia_i2c_read_reg(client, SMIA_REG_16BIT,
				  SMIAPP_REG_U16_MODEL_ID, &sid.model_id);
	rval |= smia_i2c_read_reg(client, SMIA_REG_8BIT,
				  SMIAPP_REG_U8_MANU_ID, &sid.manu_id);
	rval |= smia_i2c_read_reg(client, SMIA_REG_16BIT,
				  SMIAPP_REG_U16_SENSOR_MODEL_ID,
				  &sid.sensor_model_id);
	rval |= smia_i2c_read_reg(client, SMIA_REG_8BIT,
				  SMIAPP_REG_U8_SENSOR_MANU_ID,
				  &sid.sensor_manu_id);
	if (rval) {
		dev_err(&client->dev, "sensor detection failed\n");
		return -ENODEV;
	}

	/*
	 * Give platform identification function priority. If it is not present
	 * fall back to our static module identification table
	 */
	if (sensor->board->identify_module) {
		sensor->ident = sensor->board->identify_module(&sid);
		if (!sensor->ident) {
			dev_err(&client->dev, "platform module ident error\n");
			return -ENODEV;
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(smiapp_module_idents); i++) {
			if (smiapp_module_idents[i].manu_id == sid.manu_id &&
			    smiapp_module_idents[i].model_id == sid.model_id)
				break;
		}

		if (i >= ARRAY_SIZE(smiapp_module_idents)) {
			dev_err(&client->dev, "unknown module: %02x-%04x\n",
				sid.manu_id, sid.model_id);
			return -ENODEV;
		}
		sensor->ident = &smiapp_module_idents[i];
	}

	/* Read module dynamic identification */
	rval |= smia_i2c_read_reg(client, SMIA_REG_8BIT,
				  SMIAPP_REG_U8_REV_NO_MAJOR, &rev_major);
	rval |= smia_i2c_read_reg(client, SMIA_REG_8BIT,
				  SMIAPP_REG_U8_SMIA_VER, &smia_ver);
	rval |= smia_i2c_read_reg(client, SMIA_REG_8BIT,
				  SMIAPP_REG_U8_SMIAPP_VER, &smiapp_ver);
	if (rval) {
		dev_err(&client->dev, "dynamic ident error\n");
		return -ENODEV;
	}

	strscpy(subdev->name, sensor->ident->name, sizeof(subdev->name));
	sensor->rev_major = rev_major;
	sensor->smia_version = smia_ver;
	sensor->smiapp_version = smiapp_ver;

	return 0;
}

/*
 * smiapp_init_sensor_overrides: Till we have the sensor specific hooks
 * framework in place, this function handles per-sensor model specific
 * overrides and ugly hacks
 */
static void smiapp_init_sensor_overrides(struct smiapp_sensor *sensor)
{
	/*
	 * Lada-Lada Toshiba
	 * 1. First frame after streamon needs to be skipped due to a sensor
	 * silicon bug. Currently all the ES samples have it. Future CS samples
	 * are expected to have a fix
	 */
	if (sensor->ident->manu_id == 0x0c &&
	    sensor->ident->model_id == 0x560f &&
	    sensor->rev_major < 0x03)
		sensor->frame_skip = 1;
}

/* Stock's smiapp_registered(), at probe: the ISP's clock is there (or the probe is deferred) */
static int smiapp_init(struct smiapp_sensor *sensor)
{
	struct v4l2_subdev *subdev = &sensor->subdev;
	struct i2c_client *client = v4l2_get_subdevdata(subdev);
	const struct firmware *fw;
	char name[SMIAPP_FIRMWARE_NAME_MAX];
	int rval;
	u32 tmp;

	rval = smiapp_power_on(sensor);
	if (rval)
		return rval == -EBUSY ? -EPROBE_DEFER : -ENODEV;

	rval = smiapp_identify_module(subdev);
	if (rval) {
		rval = -ENODEV;
		goto out_power_off;
	}

	/* SMIA++ NVM initialization - it will be read from the sensor
	 * when it is first requested by userspace.
	 */
	if (sensor->smiapp_version && sensor->board->nvm_size) {
		sensor->nvm = devm_kzalloc(&client->dev, sensor->board->nvm_size,
					   GFP_KERNEL);
		if (sensor->nvm == NULL) {
			dev_err(&client->dev, "nvm buf allocation failed\n");
			rval = -ENOMEM;
			goto out_power_off;
		}
	}

	/* Import sensor firmware */
	if (sensor->smiapp_version)
		snprintf(name, SMIAPP_FIRMWARE_NAME_MAX,
			 SMIAPP_NAME "-sensor-%02x-%04x-%02x.bin",
			 sensor->ident->manu_id, sensor->ident->model_id,
			 sensor->rev_major);
	else
		snprintf(name, SMIAPP_FIRMWARE_NAME_MAX,
			 SMIA_NAME "-sensor-%02x-%04x-%02x.bin",
			 sensor->ident->manu_id, sensor->ident->model_id,
			 sensor->rev_major);

	if (request_firmware(&fw, name, &client->dev)) {
		dev_err(&client->dev, "firmware load failed: %s\n", name);
		rval = -ENODEV;
		goto out_power_off;
	}

	/* A copy: the import turns the offsets into pointers, in place */
	sensor->meta_reglist = devm_kmemdup(&client->dev, fw->data, fw->size,
					    GFP_KERNEL);
	release_firmware(fw);
	if (!sensor->meta_reglist) {
		rval = -ENOMEM;
		goto out_power_off;
	}

	rval = smia_reglist_import(&client->dev, sensor->meta_reglist);
	if (rval) {
		dev_err(&client->dev, "invalid firmware: %s\n", name);
		rval = -ENODEV;
		goto out_power_off;
	}

	sensor->current_reglist = smia_reglist_find_type(sensor->meta_reglist,
							 SMIA_REGLIST_MODE);
	if (!sensor->current_reglist) {
		dev_err(&client->dev, "invalid firmware modes: %s\n", name);
		rval = -ENODEV;
		goto out_power_off;
	}

	/* final steps */
	smiapp_read_frame_fmt(sensor);

	smia_reglist_to_mbus(sensor->current_reglist, &sensor->format);

	rval = smiapp_init_controls(sensor);
	if (rval < 0)
		goto out_power_off;

	sensor->streaming = false;
	sensor->dev_init_done = true;

	/* check flash capability */
	rval = smia_i2c_read_reg(client, SMIA_REG_8BIT,
				 SMIAPP_FLASH_MODE_CAPABILITY, &tmp);
	sensor->flash_capability = tmp;
	if (rval)
		goto out_free_controls;

	/* non fatal stuff */
	smiapp_init_sensor_overrides(sensor);

	smiapp_power_off(sensor);

	return 0;

out_free_controls:
	smiapp_free_controls(sensor);
out_power_off:
	smiapp_power_off(sensor);
	return rval;
}

static int smiapp_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct smiapp_sensor *sensor = to_smiapp_sensor(sd);
	struct v4l2_mbus_framefmt *format;
	struct smia_reglist *reglist;

	reglist = smia_reglist_find_type(sensor->meta_reglist,
					 SMIA_REGLIST_MODE);
	format = __smiapp_get_pad_format(sensor, fh->state, 0,
					 V4L2_SUBDEV_FORMAT_TRY);
	smia_reglist_to_mbus(reglist, format);

	return smiapp_set_power(sd, 1);
}

static int smiapp_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return smiapp_set_power(sd, 0);
}

static const struct v4l2_subdev_video_ops smiapp_video_ops = {
	.s_stream = smiapp_set_stream,
};

static const struct v4l2_subdev_core_ops smiapp_core_ops = {
	.s_power = smiapp_set_power,
};

static const struct v4l2_subdev_pad_ops smiapp_pad_ops = {
	.enum_mbus_code = smiapp_enum_mbus_code,
	.enum_frame_size = smiapp_enum_frame_size,
	.enum_frame_interval = smiapp_enum_frame_ival,
	.get_fmt = smiapp_get_pad_format,
	.set_fmt = smiapp_set_pad_format,
	.get_frame_interval = smiapp_get_frame_interval,
	.set_frame_interval = smiapp_set_frame_interval,
};

static const struct v4l2_subdev_sensor_ops smiapp_sensor_ops = {
	.g_skip_top_lines = smiapp_get_skip_top_lines,
	.g_skip_frames = smiapp_get_skip_frames,
};

static const struct v4l2_subdev_ops smiapp_ops = {
	.core = &smiapp_core_ops,
	.video = &smiapp_video_ops,
	.pad = &smiapp_pad_ops,
	.sensor = &smiapp_sensor_ops,
};

static const struct v4l2_subdev_internal_ops smiapp_internal_ops = {
	.open = smiapp_open,
	.close = smiapp_close,
};

/* -----------------------------------------------------------------------------
 * I2C Driver
 */

static int smiapp_parse_dt(struct smiapp_sensor *sensor, struct device *dev)
{
	struct v4l2_fwnode_endpoint vep = { .bus_type = V4L2_MBUS_UNKNOWN };
	struct fwnode_handle *ep;
	u32 rotation = 0;
	int rval;

	rval = device_property_read_u32(dev, "clock-frequency",
					&sensor->ext_clk_freq);
	if (rval)
		return dev_err_probe(dev, rval, "no clock-frequency\n");

	/*
	 * Handle Sensor Module orientation on the board.
	 *
	 * The application of H-FLIP and V-FLIP on the sensor is modified by
	 * the sensor orientation on the board.
	 *
	 * For SMIAPP_BOARD_SENSOR_ORIENT_180 the default behaviour is to set
	 * both H-FLIP and V-FLIP for normal operation which also implies
	 * that a set/unset operation for user space HFLIP and VFLIP v4l2
	 * controls will need to be internally inverted.
	 *
	 * FIXME: rotation also changes the bayer pattern.
	 */
	device_property_read_u32(dev, "rotation", &rotation);
	if (rotation == 180)
		sensor->hvflip_inv_mask = SMIAPP_IMAGE_ORIENT_HFLIP |
					  SMIAPP_IMAGE_ORIENT_VFLIP;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!ep)
		return dev_err_probe(dev, -EINVAL, "no endpoint\n");
	rval = v4l2_fwnode_endpoint_parse(ep, &vep);
	fwnode_handle_put(ep);
	if (rval)
		return dev_err_probe(dev, rval, "bad endpoint\n");
	if (vep.bus_type == V4L2_MBUS_CSI2_DPHY)
		sensor->lanes = vep.bus.mipi_csi2.num_data_lanes;

	return 0;
}

static int smiapp_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct smiapp_sensor *sensor;
	int rval;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (sensor == NULL)
		return -ENOMEM;

	sensor->board = device_get_match_data(dev);
	mutex_init(&sensor->power_lock);

	/*
	 * Both sensors answer at the main one's power-up address until it is moved: the front
	 * camera's XSHUTDOWN must be held low first (stock's board init did it at boot)
	 */
	if (sensor->board->i2c_addr_dfl && !rm696_front_camera_off) {
		struct device_node *np =
			of_find_compatible_node(NULL, NULL, "nokia,rm696-front-camera");
		bool front = np && of_device_is_available(np);

		of_node_put(np);
		if (front)
			return -EPROBE_DEFER;
	}

	rval = smiapp_parse_dt(sensor, dev);
	if (rval)
		return rval;

	sensor->xshutdown = devm_gpiod_get_optional(dev, "reset",
						    GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->xshutdown))
		return dev_err_probe(dev, PTR_ERR(sensor->xshutdown),
				     "no XSHUTDOWN gpio\n");
	if (sensor->board->xclk == NOKIA_CAMERA_XCLKB)
		rm696_front_camera_off = true;

	sensor->vana = devm_regulator_get(dev, "vana");
	if (IS_ERR(sensor->vana))
		return dev_err_probe(dev, PTR_ERR(sensor->vana),
				     "could not get regulator for vana\n");

	sensor->ext_clk = devm_clk_get(dev, NULL);
	if (IS_ERR(sensor->ext_clk))
		return dev_err_probe(dev, PTR_ERR(sensor->ext_clk),
				     "no external clock\n");
	rval = clk_set_rate(sensor->ext_clk, sensor->ext_clk_freq);
	if (rval)
		return dev_err_probe(dev, rval, "can't set the external clock rate\n");

	v4l2_i2c_subdev_init(&sensor->subdev, client, &smiapp_ops);
	sensor->subdev.internal_ops = &smiapp_internal_ops;
	sensor->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	/* As stock: a plain subdev (its entities had no sensor/lens/flash subtypes) */
	sensor->subdev.entity.function = MEDIA_ENT_F_V4L2_SUBDEV_UNKNOWN;

	rval = smiapp_init(sensor);
	if (rval)
		return rval;

	if (sensor->nvm && device_create_file(dev, &dev_attr_nvm) != 0)
		dev_err(dev, "sysfs nvm entry failed\n");
	smiapp_sysfs_entries_create(sensor);

	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	rval = media_entity_pads_init(&sensor->subdev.entity, 1, &sensor->pad);
	if (rval < 0)
		goto out_cleanup;

	/* With its lens and flash (lens-focus, flash-leds), which aren't on the ISP's ports */
	rval = v4l2_async_register_subdev_sensor(&sensor->subdev);
	if (rval < 0)
		goto out_entity_cleanup;

	return 0;

out_entity_cleanup:
	media_entity_cleanup(&sensor->subdev.entity);
out_cleanup:
	smiapp_sysfs_entries_remove(sensor);
	if (sensor->nvm)
		device_remove_file(dev, &dev_attr_nvm);
	smiapp_free_controls(sensor);
	return rval;
}

static void smiapp_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct smiapp_sensor *sensor = to_smiapp_sensor(subdev);

	v4l2_async_unregister_subdev(subdev);
	if (sensor->power_count)
		smiapp_power_off(sensor);

	if (sensor->nvm)
		device_remove_file(&client->dev, &dev_attr_nvm);

	smiapp_sysfs_entries_remove(sensor);
	media_entity_cleanup(&sensor->subdev.entity);
	smiapp_free_controls(sensor);
}

static const struct of_device_id smiapp_of_table[] = {
	{ .compatible = "nokia,rm696-main-camera", .data = &rm696_main_camera_board },
	{ .compatible = "nokia,rm696-front-camera", .data = &rm696_front_camera_board },
	{ }
};
MODULE_DEVICE_TABLE(of, smiapp_of_table);

static struct i2c_driver smiapp_i2c_driver = {
	.driver	= {
		.name = SMIAPP_NAME,
		.of_match_table = smiapp_of_table,
	},
	.probe	= smiapp_probe,
	.remove	= smiapp_remove,
};

module_i2c_driver(smiapp_i2c_driver);

MODULE_AUTHOR("Vimarsh Zutshi <vimarsh.zutshi@nokia.com>");
MODULE_DESCRIPTION("Generic SMIA/SMIA++ camera module driver");
MODULE_LICENSE("GPL");
