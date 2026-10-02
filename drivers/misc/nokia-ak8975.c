// SPDX-License-Identifier: GPL-2.0-only
/*
 * AK8975 magnetometer as Nokia's Harmattan kernel drives it: a misc device /dev/ak8975N whose
 * read() takes one measurement and returns struct ak8975_data (Harmattan's sensord, adaptor
 * "magnetometeradaptor", mag_ak8975_dev_path), plus sysfs selftest, memory, range and chip_id.
 *
 * From drivers/misc/ak8975.c in github harmattan/kernel-plus-harmattan
 * (Copyright (C) 2010 Nokia Corporation, Samu Onkalo), on 6.18 APIs, probed from the device tree
 * with the board's axis mapping as match data.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#define AK8975_WIA		0x00	/* Device ID */
#define AK8975_INFO		0x01	/* Information */
#define AK8975_ST1		0x02	/* Status 1 */
#define AK8975_HXL		0x03	/* Measurement data X, Y, Z (little endian) */
#define AK8975_ST2		0x09	/* Status 2 */
#define AK8975_CNTL		0x0A	/* Control */
#define AK8975_ASTC		0x0C	/* Self-test */
#define AK8975_ASAX		0x10	/* Sensitivity adjustment X, Y, Z */

#define AK8975_WHOAMI_VALUE	0x48

#define AK8975_POWERON_DELAY	50
#define AK8975_POWEROFF_DELAY	1
#define AK8975_MEASTIME		8

#define AK8975_MEMORYMODE	0xf
#define AK8975_SELFTESTMODE	0x8
#define AK8975_MEASMODE		0x1
#define AK8975_PWRDOWNMODE	0x0

#define AK8975_SELFTEST		(0x1 << 6)
#define AK8975_ST1_DRDY		0x1
#define AK8975_ST2_HOFL		(0x1 << 3)	/* Magnetic overflow */
#define AK8975_MAX_RANGE	4096

/* Axis mapping: 1..3 = the chip's X..Z, negative = inverted */
#define AK8975_DEV_X		1
#define AK8975_DEV_Y		2
#define AK8975_DEV_Z		3

/* What read() returns (Nokia's include/linux/i2c/ak8975.h) */
struct ak8975_data {
	__s16 x;
	__s16 y;
	__s16 z;
	__u16 valid;
} __packed;

struct ak8975_axes {
	s8 x, y, z;
};

struct ak8975_chip {
	struct miscdevice	miscdev;
	struct mutex		lock;	/* Serialize access to chip */
	struct mutex		users_lock;
	struct i2c_client	*client;
	struct regulator_bulk_data regs[2];
	struct delayed_work	dwork;
	wait_queue_head_t	misc_wait;
	loff_t			offset;

	int			users;
	u8			info;
	u8			memory[3];
	s16			x, y, z;	/* Latest measurements */
	struct ak8975_axes	axes;
	bool			valid;
	bool			meas_ongoing;

	char			name[20];
};

static atomic_t device_number = ATOMIC_INIT(0);

static int ak8975_write(struct ak8975_chip *chip, u8 reg, u8 data)
{
	return i2c_smbus_write_byte_data(chip->client, reg, data);
}

static int ak8975_read(struct ak8975_chip *chip, u8 reg)
{
	return i2c_smbus_read_byte_data(chip->client, reg);
}

static int ak8975_poll_drdy(struct ak8975_chip *chip)
{
	int ret = ak8975_read(chip, AK8975_ST1);

	if (ret < 0)
		return ret;
	return ret & AK8975_ST1_DRDY ? 0 : ret;
}

static void ak8975_trigmeas(struct ak8975_chip *chip)
{
	mutex_lock(&chip->lock);
	if (!chip->meas_ongoing) {
		ak8975_write(chip, AK8975_CNTL, AK8975_MEASMODE);
		chip->meas_ongoing = true;
		schedule_delayed_work(&chip->dwork, msecs_to_jiffies(AK8975_MEASTIME));
	}
	mutex_unlock(&chip->lock);
}

static void ak8975_reset(struct ak8975_chip *chip)
{
	ak8975_write(chip, AK8975_CNTL, AK8975_PWRDOWNMODE);
	msleep(AK8975_POWEROFF_DELAY);
	ak8975_write(chip, AK8975_ASTC, 0);
}

static int ak8975_getmemory(struct ak8975_chip *chip)
{
	int ret;

	ret = ak8975_write(chip, AK8975_CNTL, AK8975_MEMORYMODE);
	if (ret < 0)
		return ret;
	ret = i2c_smbus_read_i2c_block_data(chip->client, AK8975_ASAX, 3, chip->memory);
	ak8975_write(chip, AK8975_CNTL, AK8975_PWRDOWNMODE);
	msleep(AK8975_POWEROFF_DELAY);
	return ret < 0 ? ret : 0;
}

static int ak8975_getresult(struct ak8975_chip *chip, s16 *result)
{
	__le16 raw[3];
	int i, ret;

	ret = ak8975_poll_drdy(chip);
	/* Measurement overflow? */
	ret |= ak8975_read(chip, AK8975_ST2);
	if (ret < 0)
		return ret;
	ret = ret & AK8975_ST2_HOFL ? -ERANGE : 0;

	i2c_smbus_read_i2c_block_data(chip->client, AK8975_HXL, sizeof(raw), (u8 *)raw);
	for (i = 0; i < 3; i++)
		result[i] = ((s32)(s16)le16_to_cpu(raw[i]) * (s32)(chip->memory[i] + 128)) / 256;
	return ret;
}

static int ak8975_selftest(struct ak8975_chip *chip, s16 *result)
{
	int ret;

	ret = ak8975_write(chip, AK8975_CNTL, AK8975_PWRDOWNMODE);
	msleep(AK8975_POWEROFF_DELAY);
	ret |= ak8975_write(chip, AK8975_ASTC, AK8975_SELFTEST);
	ret |= ak8975_write(chip, AK8975_CNTL, AK8975_SELFTESTMODE);

	msleep(AK8975_MEASTIME);
	ret |= ak8975_poll_drdy(chip);
	ret |= ak8975_getresult(chip, result);
	ret |= ak8975_write(chip, AK8975_ASTC, 0);

	return ret;
}

static int ak8975_regulators_on(struct ak8975_chip *chip)
{
	int ret = regulator_bulk_enable(ARRAY_SIZE(chip->regs), chip->regs);

	if (ret == 0)
		msleep(AK8975_POWERON_DELAY);
	return ret;
}

static void ak8975_regulators_off(struct ak8975_chip *chip)
{
	regulator_bulk_disable(ARRAY_SIZE(chip->regs), chip->regs);
}

static int ak8975_add_users(struct ak8975_chip *chip)
{
	int r = 0;

	mutex_lock(&chip->users_lock);
	if (chip->users == 0) {
		r = ak8975_regulators_on(chip);
		if (r == 0)
			ak8975_reset(chip);
	}
	if (r == 0)
		chip->users++;
	mutex_unlock(&chip->users_lock);
	return r;
}

static void ak8975_remove_users(struct ak8975_chip *chip)
{
	mutex_lock(&chip->users_lock);
	if (chip->users != 0)
		chip->users--;
	if (chip->users == 0) {
		cancel_delayed_work_sync(&chip->dwork);
		chip->meas_ongoing = false;
		ak8975_regulators_off(chip);
	}
	mutex_unlock(&chip->users_lock);
}

static s16 ak8975_get_axis(s8 axis, s16 hw_values[3])
{
	return axis > 0 ? hw_values[axis - 1] : -hw_values[-axis - 1];
}

static void ak8975_dwork(struct work_struct *work)
{
	struct ak8975_chip *chip = container_of(work, struct ak8975_chip, dwork.work);
	s16 hw_values[3];
	int ret;

	mutex_lock(&chip->lock);
	ret = ak8975_getresult(chip, hw_values);
	chip->x = ak8975_get_axis(chip->axes.x, hw_values);
	chip->y = ak8975_get_axis(chip->axes.y, hw_values);
	chip->z = ak8975_get_axis(chip->axes.z, hw_values);
	chip->valid = ret == 0;
	chip->offset += sizeof(struct ak8975_data);
	chip->meas_ongoing = false;
	mutex_unlock(&chip->lock);
	wake_up_interruptible(&chip->misc_wait);
}

static int ak8975_misc_open(struct inode *inode, struct file *file)
{
	struct ak8975_chip *chip = container_of(file->private_data, struct ak8975_chip, miscdev);

	file->f_pos = chip->offset;
	return ak8975_add_users(chip);
}

static int ak8975_misc_close(struct inode *inode, struct file *file)
{
	struct ak8975_chip *chip = container_of(file->private_data, struct ak8975_chip, miscdev);

	ak8975_remove_users(chip);
	return 0;
}

/* The file position counts measurements: a read waits for one newer than the reader's last */
static ssize_t ak8975_misc_read(struct file *file, char __user *buf, size_t count,
				loff_t *offset)
{
	struct ak8975_chip *chip = container_of(file->private_data, struct ak8975_chip, miscdev);
	struct ak8975_data data;

	if (count < sizeof(data))
		return -EINVAL;

	if (*offset >= chip->offset) {
		ak8975_trigmeas(chip);
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		if (wait_event_interruptible(chip->misc_wait, *offset < chip->offset))
			return -ERESTARTSYS;
	}

	mutex_lock(&chip->lock);
	data.x = chip->x;
	data.y = chip->y;
	data.z = chip->z;
	data.valid = chip->valid;
	*offset = chip->offset;
	mutex_unlock(&chip->lock);

	return copy_to_user(buf, &data, sizeof(data)) ? -EFAULT : sizeof(data);
}

static const struct file_operations ak8975_fops = {
	.owner		= THIS_MODULE,
	.read		= ak8975_misc_read,
	.open		= ak8975_misc_open,
	.release	= ak8975_misc_close,
};

static ssize_t selftest_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ak8975_chip *chip = dev_get_drvdata(dev);
	s16 results[3] = { 0 };
	int ret;

	ret = ak8975_add_users(chip);
	if (ret)
		return ret;
	mutex_lock(&chip->lock);
	ret = ak8975_selftest(chip, results);
	mutex_unlock(&chip->lock);
	ak8975_remove_users(chip);

	return sysfs_emit(buf, "%s %d %d %d\n", ret ? "FAIL" : "OK", results[0], results[1],
			  results[2]);
}
static DEVICE_ATTR_RO(selftest);

static ssize_t memory_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ak8975_chip *chip = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d %d %d\n", chip->memory[0], chip->memory[1], chip->memory[2]);
}
static DEVICE_ATTR_RO(memory);

static ssize_t chip_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ak8975_chip *chip = dev_get_drvdata(dev);

	return sysfs_emit(buf, "ak8975 rev %d\n", chip->info);
}
static DEVICE_ATTR_RO(chip_id);

static ssize_t range_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", AK8975_MAX_RANGE);
}
static DEVICE_ATTR_RO(range);

static struct attribute *ak8975_attrs[] = {
	&dev_attr_selftest.attr,
	&dev_attr_memory.attr,
	&dev_attr_range.attr,
	&dev_attr_chip_id.attr,
	NULL
};
ATTRIBUTE_GROUPS(ak8975);

static int ak8975_probe(struct i2c_client *client)
{
	const struct ak8975_axes *axes = of_device_get_match_data(&client->dev);
	struct ak8975_chip *chip;
	int err;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	i2c_set_clientdata(client, chip);
	chip->client = client;
	chip->axes = axes ? *axes : (struct ak8975_axes){ AK8975_DEV_X, AK8975_DEV_Y, AK8975_DEV_Z };
	mutex_init(&chip->lock);
	mutex_init(&chip->users_lock);
	init_waitqueue_head(&chip->misc_wait);
	INIT_DELAYED_WORK(&chip->dwork, ak8975_dwork);

	chip->regs[0].supply = "AVdd";
	chip->regs[1].supply = "DVdd";
	err = devm_regulator_bulk_get(&client->dev, ARRAY_SIZE(chip->regs), chip->regs);
	if (err)
		return dev_err_probe(&client->dev, err, "Cannot get regulators\n");

	err = ak8975_regulators_on(chip);
	if (err)
		return dev_err_probe(&client->dev, err, "Cannot enable regulators\n");

	if (ak8975_read(chip, AK8975_WIA) != AK8975_WHOAMI_VALUE) {
		ak8975_regulators_off(chip);
		return dev_err_probe(&client->dev, -ENODEV, "AK8975 not found\n");
	}
	chip->info = ak8975_read(chip, AK8975_INFO);
	ak8975_getmemory(chip);
	ak8975_reset(chip);
	ak8975_regulators_off(chip);

	snprintf(chip->name, sizeof(chip->name), "ak8975%d",
		 atomic_add_return(1, &device_number) - 1);
	chip->miscdev.minor = MISC_DYNAMIC_MINOR;
	chip->miscdev.name = chip->name;
	chip->miscdev.fops = &ak8975_fops;
	chip->miscdev.parent = &client->dev;
	err = misc_register(&chip->miscdev);
	if (err)
		return dev_err_probe(&client->dev, err, "Device registration failed\n");

	dev_info(&client->dev, "/dev/%s, sensitivity %d %d %d\n", chip->name, chip->memory[0],
		 chip->memory[1], chip->memory[2]);
	return 0;
}

static void ak8975_remove(struct i2c_client *client)
{
	struct ak8975_chip *chip = i2c_get_clientdata(client);

	misc_deregister(&chip->miscdev);
	cancel_delayed_work_sync(&chip->dwork);
}

/* The N9 (board-rm696-peripherals.c): x = the chip's Z, y = -X, z = -Y */
static const struct ak8975_axes n9_axes = { AK8975_DEV_Z, -AK8975_DEV_X, -AK8975_DEV_Y };

static const struct of_device_id ak8975_of_match[] = {
	{ .compatible = "nokia,n9-ak8975", .data = &n9_axes },
	{ }
};
MODULE_DEVICE_TABLE(of, ak8975_of_match);

static struct i2c_driver ak8975_driver = {
	.driver = {
		.name		= "nokia-ak8975",
		.of_match_table	= ak8975_of_match,
		.dev_groups	= ak8975_groups,
	},
	.probe		= ak8975_probe,
	.remove		= ak8975_remove,
};
module_i2c_driver(ak8975_driver);

MODULE_DESCRIPTION("AK8975 3-axis magnetometer driver (Nokia's misc device interface)");
MODULE_AUTHOR("Samu Onkalo, Nokia Corporation");
MODULE_LICENSE("GPL v2");
