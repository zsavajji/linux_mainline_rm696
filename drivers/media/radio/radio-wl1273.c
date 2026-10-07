// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the Texas Instruments WL1273 FM radio.
 *
 * Nokia's /dev/radio0 from its Harmattan kernel (2.6.32), as the N9 shipped it and as fmrxd and
 * Nokia's test servers use it: frequency, tuner (RSSI in .signal), hardware seek, mute and volume,
 * RDS as the chip's raw 3-byte blocks (two data bytes and a status byte) through read(). The mode
 * (RX/TX/off) is the codec's "Mode Switch" control. Ported to the control framework and v4l2_fh;
 * stock's old-style private controls (V4L2_CID_PRIVATE_BASE + n) aren't: nothing on Harmattan or in
 * the community apps uses them, the codec's controls cover the same settings.
 *
 * Copyright (C) Nokia Corporation
 * Author: Matti J. Aaltonen <matti.j.aaltonen@nokia.com>
 */

#include <linux/mfd/wl1273-core.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>

#define DRIVER_DESC "Wl1273 FM Radio - V4L2"

/*
 * static int radio_nr - The number of the radio device
 *
 * The default is 0.
 */
static int radio_nr = -1;
module_param(radio_nr, int, 0);
MODULE_PARM_DESC(radio_nr, "Radio Nr");

struct wl1273_device {
	struct v4l2_device v4l2dev;
	struct video_device videodev;
	struct v4l2_ctrl_handler ctrl_handler;
	struct device *dev;
	struct wl1273_core *core;
	bool rds_on;
};

static ssize_t wl1273_fm_fops_write(struct file *file, const char __user *buf,
				    size_t count, loff_t *ppos)
{
	struct wl1273_device *radio = video_drvdata(file);
	unsigned char *s;
	u16 val;
	int r;

	dev_dbg(radio->dev, "%s\n", __func__);

	if (radio->core->mode != WL1273_MODE_TX)
		return count;

	if (!radio->rds_on) {
		dev_warn(radio->dev, "%s: RDS not on.\n", __func__);
		return 0;
	}

	if (mutex_lock_interruptible(&radio->core->lock))
		return -EINTR;

	/* Manual Mode */
	if (count > 255)
		val = 255;
	else
		val = count;

	wl1273_fm_write_cmd(radio->core, WL1273_RDS_CONFIG_DATA_SET, val);

	s = kmalloc(val + 1, GFP_KERNEL);
	if (!s) {
		r = -ENOMEM;
		goto out;
	}

	if (copy_from_user(s + 1, buf, val)) {
		kfree(s);
		r = -EFAULT;
		goto out;
	}

	dev_dbg(radio->dev, "Count: %d\n", val);

	s[0] = WL1273_RDS_DATA_SET;
	wl1273_fm_write_data(radio->core, s, val + 1);

	kfree(s);
	r = val;

out:
	mutex_unlock(&radio->core->lock);

	return r;
}

static __poll_t wl1273_fm_fops_poll(struct file *file,
				    struct poll_table_struct *pts)
{
	struct wl1273_device *radio = video_drvdata(file);
	struct wl1273_core *core = radio->core;
	unsigned int rd_index, wr_index;

	/* TODO: handle the case of multiple readers */

	poll_wait(file, &core->read_queue, pts);

	rd_index = core->rd_index;
	wr_index = core->wr_index;
	if (rd_index != wr_index)
		return EPOLLIN | EPOLLRDNORM;

	return 0;
}

static int wl1273_fm_fops_open(struct file *file)
{
	struct wl1273_device *radio = video_drvdata(file);
	struct wl1273_core *core = radio->core;
	int r;

	dev_dbg(radio->dev, "%s\n", __func__);

	r = v4l2_fh_open(file);
	if (r)
		return r;

	if (core->mode == WL1273_MODE_RX && core->rds_on && !radio->rds_on) {
		if (mutex_lock_interruptible(&core->lock)) {
			v4l2_fh_release(file);
			return -EINTR;
		}

		core->irq_flags |= WL1273_RDS_EVENT;

		r = wl1273_fm_write_cmd(core, WL1273_INT_MASK_SET,
					core->irq_flags);
		if (r) {
			mutex_unlock(&core->lock);
			v4l2_fh_release(file);
			return r;
		}

		radio->rds_on = true;
		mutex_unlock(&core->lock);
	}

	return 0;
}

static int wl1273_fm_fops_release(struct file *file)
{
	struct wl1273_device *radio = video_drvdata(file);
	struct wl1273_core *core = radio->core;

	dev_dbg(radio->dev, "%s\n", __func__);

	if (radio->rds_on) {
		mutex_lock(&core->lock);

		core->irq_flags &= ~WL1273_RDS_EVENT;

		if (core->mode == WL1273_MODE_RX)
			wl1273_fm_write_cmd(core, WL1273_INT_MASK_SET,
					    core->irq_flags);

		radio->rds_on = false;
		mutex_unlock(&core->lock);
	}

	return v4l2_fh_release(file);
}

static ssize_t wl1273_fm_fops_read(struct file *file, char __user *buf,
				   size_t count, loff_t *ppos)
{
	int r = 0;
	u16 val;
	struct wl1273_device *radio = video_drvdata(file);
	struct wl1273_core *core = radio->core;
	unsigned int block_count = 0;

	/* TODO: handle the case of multiple readers */

	dev_dbg(radio->dev, "%s\n", __func__);

	if (radio->core->mode != WL1273_MODE_RX)
		return 0;

	if (!radio->rds_on) {
		dev_warn(radio->dev, "%s: RDS not on.\n", __func__);
		return 0;
	}

	if (mutex_lock_interruptible(&core->lock))
		return -EINTR;

	r = wl1273_fm_read_reg(core, WL1273_RDS_SYNC_GET, &val);
	if (r) {
		dev_err(radio->dev, "%s: Get RDS_SYNC fails.\n",
			__func__);
		goto out;
	} else if (val == 0) {
		dev_info(radio->dev, "RDS_SYNC: Not synchronized\n");
		r = -ENODATA;
		goto out;
	}

	/* block if no new data available */
	while (core->wr_index == core->rd_index) {
		if (file->f_flags & O_NONBLOCK) {
			r = -EWOULDBLOCK;
			goto out;
		}

		if (wait_event_interruptible(core->read_queue,
					     core->wr_index !=
					     core->rd_index) < 0) {
			r = -EINTR;
			goto out;
		}
	}

	/* calculate block count from byte count */
	count /= 3;

	/* copy RDS blocks from the internal buffer and to user buffer */

	while (block_count < count) {
		if (core->rd_index == core->wr_index)
			break;

		/* always transfer complete RDS blocks */
		if (copy_to_user(buf, &core->buffer[core->rd_index], 3))
			break;

		/* increment and wrap the read pointer */
		core->rd_index += 3;
		if (core->rd_index >= core->buf_size)
			core->rd_index = 0;

		/* increment counters */
		block_count++;
		buf += 3;
		r += 3;
	}

out:
	dev_dbg(radio->dev, "%s: exit\n", __func__);
	mutex_unlock(&core->lock);

	return r;
}

static const struct v4l2_file_operations wl1273_fops = {
	.owner		= THIS_MODULE,
	.read		= wl1273_fm_fops_read,
	.write		= wl1273_fm_fops_write,
	.poll		= wl1273_fm_fops_poll,
	.unlocked_ioctl	= video_ioctl2,
	.open		= wl1273_fm_fops_open,
	.release	= wl1273_fm_fops_release,
};

static int wl1273_fm_vidioc_querycap(struct file *file, void *priv,
				     struct v4l2_capability *capability)
{
	strscpy(capability->driver, WL1273_FM_DRIVER_NAME,
		sizeof(capability->driver));
	strscpy(capability->card, "Texas Instruments Wl1273 FM Radio",
		sizeof(capability->card));
	strscpy(capability->bus_info, "I2C", sizeof(capability->bus_info));

	return 0;
}

#define WL1273_MUTE_SOFT_ENABLE    (1 << 0)
#define WL1273_MUTE_AC             (1 << 1)
#define WL1273_MUTE_HARD_LEFT      (1 << 2)
#define WL1273_MUTE_HARD_RIGHT     (1 << 3)
#define WL1273_MUTE_SOFT_FORCE     (1 << 4)

/* As stock: every write reaches the chip (EXECUTE_ON_WRITE), the volume reads back the core's */
static int wl1273_fm_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct wl1273_device *radio = container_of(ctrl->handler,
						   struct wl1273_device, ctrl_handler);

	if (ctrl->id == V4L2_CID_AUDIO_VOLUME)
		ctrl->val = radio->core->volume;

	return 0;
}

static int wl1273_fm_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct wl1273_device *radio = container_of(ctrl->handler,
						   struct wl1273_device, ctrl_handler);
	struct wl1273_core *core = radio->core;
	int r = 0;

	switch (ctrl->id) {
	case V4L2_CID_AUDIO_MUTE:
		mutex_lock(&core->lock);

		if (core->mode == WL1273_MODE_RX && ctrl->val)
			r = wl1273_fm_write_cmd(core,
						WL1273_MUTE_STATUS_SET,
						WL1273_MUTE_HARD_LEFT |
						WL1273_MUTE_HARD_RIGHT);
		else if (core->mode == WL1273_MODE_RX)
			r = wl1273_fm_write_cmd(core,
						WL1273_MUTE_STATUS_SET, 0x0);
		else if (core->mode == WL1273_MODE_TX && ctrl->val)
			r = wl1273_fm_write_cmd(core, WL1273_MUTE, 1);
		else if (core->mode == WL1273_MODE_TX)
			r = wl1273_fm_write_cmd(core, WL1273_MUTE, 0);

		mutex_unlock(&core->lock);
		break;

	case V4L2_CID_AUDIO_VOLUME:
		r = wl1273_fm_set_volume(core, ctrl->val);
		break;
	}

	return r;
}

static const struct v4l2_ctrl_ops wl1273_ctrl_ops = {
	.g_volatile_ctrl = wl1273_fm_g_volatile_ctrl,
	.s_ctrl = wl1273_fm_s_ctrl,
};

#define WL1273_RDS_SYNCHRONIZED 1

static int wl1273_fm_vidioc_g_tuner(struct file *file, void *priv,
				    struct v4l2_tuner *tuner)
{
	struct wl1273_device *radio = video_drvdata(file);
	struct wl1273_core *core = radio->core;
	int r = 0;

	dev_dbg(radio->dev, "%s\n", __func__);

	if (tuner->index > 0)
		return -EINVAL;

	strscpy(tuner->name, WL1273_FM_DRIVER_NAME, sizeof(tuner->name));
	tuner->type = V4L2_TUNER_RADIO;

	tuner->rangelow	=
		core->regions[core->region].bottom_frequency * 10000 / 625;
	tuner->rangehigh =
		core->regions[core->region].top_frequency * 10000 / 625;

	tuner->rxsubchans = V4L2_TUNER_SUB_MONO | V4L2_TUNER_SUB_STEREO;
	tuner->capability = V4L2_TUNER_CAP_LOW;

	if (mutex_lock_interruptible(&core->lock))
		return -EINTR;

	if (core->mode == WL1273_MODE_RX) {
		u16 val;

		r = wl1273_fm_read_reg(core, WL1273_RSSI_LVL_GET, &val);
		if (r)
			goto out;

		/* stock's: the chip's RSSI, a signed value, not V4L2's 0..65535 */
		tuner->signal = (s16) val;
		dev_dbg(radio->dev, "Signal: %d\n", tuner->signal);

		tuner->afc = 0;

		r = wl1273_fm_read_reg(core, WL1273_RDS_SYNC_GET, &val);
		if (r)
			goto out;

		if (val == WL1273_RDS_SYNCHRONIZED)
			tuner->rxsubchans |= V4L2_TUNER_SUB_RDS;
	}
out:
	mutex_unlock(&core->lock);

	return r;
}

static int wl1273_fm_vidioc_s_tuner(struct file *file, void *priv,
				    const struct v4l2_tuner *tuner)
{
	struct wl1273_device *radio = video_drvdata(file);
	struct wl1273_core *core = radio->core;
	int r = 0;

	dev_dbg(radio->dev, "%s\n", __func__);

	if (tuner->index > 0)
		return -EINVAL;

	if (mutex_lock_interruptible(&core->lock))
		return -EINTR;

	if (tuner->audmode == V4L2_TUNER_MODE_MONO)
		r = wl1273_fm_write_cmd(core, WL1273_MOST_MODE_SET,
					WL1273_RX_MONO);
	else
		r = wl1273_fm_write_cmd(core, WL1273_MOST_MODE_SET,
					WL1273_RX_STEREO);

	if (r < 0)
		dev_warn(radio->dev, WL1273_FM_DRIVER_NAME
			 ": set tuner mode failed with %d\n", r);

	mutex_unlock(&core->lock);

	return r;
}

static int wl1273_fm_vidioc_g_frequency(struct file *file, void *priv,
					struct v4l2_frequency *freq)
{
	struct wl1273_device *radio = video_drvdata(file);
	struct wl1273_core *core = radio->core;

	dev_dbg(radio->dev, "%s\n", __func__);

	if (mutex_lock_interruptible(&core->lock))
		return -EINTR;

	freq->type = V4L2_TUNER_RADIO;
	freq->frequency = wl1273_fm_get_freq(core) * 10000 / 625;

	mutex_unlock(&core->lock);

	return 0;
}

static int wl1273_fm_vidioc_s_frequency(struct file *file, void *priv,
					const struct v4l2_frequency *freq)
{
	struct wl1273_device *radio = video_drvdata(file);
	struct wl1273_core *core = radio->core;
	int r;

	dev_dbg(radio->dev, "%s: %d\n", __func__, freq->frequency);

	if (freq->type != V4L2_TUNER_RADIO) {
		dev_dbg(radio->dev,
			"freq->type != V4L2_TUNER_RADIO: %d\n", freq->type);
		return -EINVAL;
	}

	if (mutex_lock_interruptible(&core->lock))
		return -EINTR;

	wl1273_fm_write_cmd(core, WL1273_INT_MASK_SET, core->irq_flags);

	if (core->mode == WL1273_MODE_RX) {
		r = wl1273_fm_set_rx_freq(core, freq->frequency * 625 / 10000);
		if (r)
			dev_warn(radio->dev, WL1273_FM_DRIVER_NAME
				 ": set frequency failed with %d\n", r);
	} else {
		r = wl1273_fm_set_tx_freq(core, freq->frequency * 625 / 10000);
		if (r)
			dev_warn(radio->dev, WL1273_FM_DRIVER_NAME
				 ": set frequency failed with %d\n", r);
	}

	mutex_unlock(&core->lock);

	dev_dbg(radio->dev, "wl1273_vidioc_s_frequency: DONE\n");
	return r;
}

static int wl1273_fm_vidioc_s_hw_freq_seek(struct file *file, void *priv,
					   const struct v4l2_hw_freq_seek *seek)
{
	struct wl1273_device *radio = video_drvdata(file);
	struct wl1273_core *core = radio->core;
	int r;

	dev_dbg(radio->dev, "%s\n", __func__);

	if (core->mode != WL1273_MODE_RX)
		return 0;

	if (seek->tuner != 0 || seek->type != V4L2_TUNER_RADIO)
		return -EINVAL;

	if (mutex_lock_interruptible(&core->lock))
		return -EINTR;

	r = wl1273_fm_set_seek(core, seek->wrap_around, seek->seek_upward);
	if (r)
		dev_warn(radio->dev, "HW seek failed: %d\n", r);

	mutex_unlock(&core->lock);

	return r;
}

static int wl1273_fm_vidioc_log_status(struct file *file, void *priv)
{
	struct wl1273_device *radio = video_drvdata(file);
	struct wl1273_core *core = radio->core;
	u16 val;
	int r;

	dev_info(radio->dev, DRIVER_DESC);

	if (core->mode == WL1273_MODE_OFF) {
		dev_info(radio->dev, "Mode: Off\n");
		return 0;
	}

	if (core->mode == WL1273_MODE_SUSPENDED) {
		dev_info(radio->dev, "Mode: Suspended\n");
		return 0;
	}

	r = wl1273_fm_read_reg(core, WL1273_ASIC_ID_GET, &val);
	if (r)
		dev_err(radio->dev, "%s: Get ASIC_ID fails.\n", __func__);
	else
		dev_info(radio->dev, "ASIC_ID: 0x%04x\n", val);

	r = wl1273_fm_read_reg(core, WL1273_ASIC_VER_GET, &val);
	if (r)
		dev_err(radio->dev, "%s: Get ASIC_VER fails.\n",
			__func__);
	else
		dev_info(radio->dev, "ASIC Version: 0x%04x\n", val);

	r = wl1273_fm_read_reg(core, WL1273_FIRM_VER_GET, &val);
	if (r)
		dev_err(radio->dev, "%s: Get FIRM_VER fails.\n", __func__);
	else
		dev_info(radio->dev, "FW version: %d(0x%04x)\n", val, val);

	/* TODO: Add TX stuff */
	if (core->mode != WL1273_MODE_RX)
		return 0;

	r = wl1273_fm_read_reg(core, WL1273_FREQ_SET, &val);
	if (r)
		dev_err(radio->dev, "%s: Get FREQ fails.\n", __func__);
	else
		dev_info(radio->dev, "RX Frequency: %dkHz\n",
			core->regions[core->region].bottom_frequency + val*50);

	r = wl1273_fm_read_reg(core, WL1273_MOST_MODE_SET, &val);
	if (r)
		dev_err(radio->dev, "%s: Get MOST_MODE fails.\n",
			__func__);
	else if (val == 0)
		dev_info(radio->dev,
			 "MOST_MODE: Stereo output according to blend\n");
	else if (val == 1)
		dev_info(radio->dev, "MOST_MODE: Force mono output\n");
	else
		dev_info(radio->dev, "MOST_MODE: Unexpected value: %d\n", val);

	r = wl1273_fm_read_reg(core, WL1273_MOST_BLEND_SET, &val);
	if (r)
		dev_err(radio->dev, "%s: Get MOST_BLEND fails.\n", __func__);
	else if (val == 0)
		dev_info(radio->dev,
			 "MOST_BLEND: Switched blend with hysteresis.\n");
	else if (val == 1)
		dev_info(radio->dev, "MOST_BLEND: Soft blend.\n");
	else
		dev_info(radio->dev, "MOST_BLEND: Unexpected value: %d\n", val);

	r = wl1273_fm_read_reg(core, WL1273_STEREO_GET, &val);
	if (r)
		dev_err(radio->dev, "%s: Get STEREO fails.\n", __func__);
	else if (val == 0)
		dev_info(radio->dev, "STEREO: Not detected\n");
	else if (val == 1)
		dev_info(radio->dev, "STEREO: Detected\n");
	else
		dev_info(radio->dev, "STEREO: Unexpected value: %d\n", val);

	r = wl1273_fm_read_reg(core, WL1273_RSSI_LVL_GET, &val);
	if (r)
		dev_err(radio->dev, "%s: Get RSSI_LVL fails.\n", __func__);
	else
		dev_info(radio->dev, "RX signal strength: %d\n", (s16) val);

	r = wl1273_fm_read_reg(core, WL1273_POWER_SET, &val);
	if (r)
		dev_err(radio->dev, "%s: Get POWER fails.\n", __func__);
	else
		dev_info(radio->dev, "POWER: 0x%04x\n", val);

	r = wl1273_fm_read_reg(core, WL1273_INT_MASK_SET, &val);
	if (r)
		dev_err(radio->dev, "%s: Get INT_MASK fails.\n", __func__);
	else
		dev_info(radio->dev, "INT_MASK: 0x%04x\n", val);

	r = wl1273_fm_read_reg(core, WL1273_RDS_SYNC_GET, &val);
	if (r)
		dev_err(radio->dev, "%s: Get RDS_SYNC fails.\n",
			__func__);
	else if (val == 0)
		dev_info(radio->dev, "RDS_SYNC: Not synchronized\n");

	else if (val == 1)
		dev_info(radio->dev, "RDS_SYNC: Synchronized\n");
	else
		dev_info(radio->dev, "RDS_SYNC: Unexpected value: %d\n", val);

	r = wl1273_fm_read_reg(core, WL1273_I2S_MODE_CONFIG_SET, &val);
	if (r)
		dev_err(radio->dev, "%s: Get I2S_MODE_CONFIG fails.\n",
			__func__);
	else
		dev_info(radio->dev, "I2S_MODE_CONFIG: 0x%04x\n", val);

	return 0;
}

static const struct v4l2_ioctl_ops wl1273_ioctl_ops = {
	.vidioc_querycap	= wl1273_fm_vidioc_querycap,
	.vidioc_g_tuner		= wl1273_fm_vidioc_g_tuner,
	.vidioc_s_tuner		= wl1273_fm_vidioc_s_tuner,
	.vidioc_g_frequency	= wl1273_fm_vidioc_g_frequency,
	.vidioc_s_frequency	= wl1273_fm_vidioc_s_frequency,
	.vidioc_s_hw_freq_seek	= wl1273_fm_vidioc_s_hw_freq_seek,
	.vidioc_log_status	= wl1273_fm_vidioc_log_status,
};

static int wl1273_fm_radio_probe(struct platform_device *pdev)
{
	struct wl1273_core **pdata = pdev->dev.platform_data;
	struct wl1273_device *radio;
	struct v4l2_ctrl_handler *hdl;
	struct v4l2_ctrl *ctrl;
	int r;

	if (!pdata)
		return dev_err_probe(&pdev->dev, -EINVAL, "No platform data.\n");

	radio = devm_kzalloc(&pdev->dev, sizeof(*radio), GFP_KERNEL);
	if (!radio)
		return -ENOMEM;

	radio->core = *pdata;
	radio->dev = &pdev->dev;

	r = v4l2_device_register(&pdev->dev, &radio->v4l2dev);
	if (r)
		return r;

	hdl = &radio->ctrl_handler;
	v4l2_ctrl_handler_init(hdl, 2);
	ctrl = v4l2_ctrl_new_std(hdl, &wl1273_ctrl_ops, V4L2_CID_AUDIO_VOLUME,
				 0, WL1273_MAX_VOLUME, 1, WL1273_DEFAULT_VOLUME);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
	ctrl = v4l2_ctrl_new_std(hdl, &wl1273_ctrl_ops, V4L2_CID_AUDIO_MUTE, 0, 1, 1, 1);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
	if (hdl->error) {
		r = hdl->error;
		goto err;
	}
	radio->v4l2dev.ctrl_handler = hdl;

	strscpy(radio->videodev.name, WL1273_FM_DRIVER_NAME, sizeof(radio->videodev.name));
	radio->videodev.v4l2_dev = &radio->v4l2dev;
	radio->videodev.fops = &wl1273_fops;
	radio->videodev.ioctl_ops = &wl1273_ioctl_ops;
	radio->videodev.release = video_device_release_empty;
	radio->videodev.device_caps = V4L2_CAP_HW_FREQ_SEEK | V4L2_CAP_TUNER |
		V4L2_CAP_RADIO | V4L2_CAP_READWRITE | V4L2_CAP_AUDIO;
	video_set_drvdata(&radio->videodev, radio);
	platform_set_drvdata(pdev, radio);

	r = video_register_device(&radio->videodev, VFL_TYPE_RADIO, radio_nr);
	if (r) {
		dev_err(&pdev->dev, "Could not register video device\n");
		goto err;
	}

	return 0;

err:
	v4l2_ctrl_handler_free(hdl);
	v4l2_device_unregister(&radio->v4l2dev);
	return r;
}

static void wl1273_fm_radio_remove(struct platform_device *pdev)
{
	struct wl1273_device *radio = platform_get_drvdata(pdev);

	video_unregister_device(&radio->videodev);
	v4l2_ctrl_handler_free(&radio->ctrl_handler);
	v4l2_device_unregister(&radio->v4l2dev);
}

MODULE_ALIAS("platform:wl1273_fm_radio");

static struct platform_driver wl1273_fm_radio_driver = {
	.probe		= wl1273_fm_radio_probe,
	.remove		= wl1273_fm_radio_remove,
	.driver		= {
		.name	= "wl1273_fm_radio",
	},
};
module_platform_driver(wl1273_fm_radio_driver);

MODULE_AUTHOR("Matti Aaltonen <matti.j.aaltonen@nokia.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
