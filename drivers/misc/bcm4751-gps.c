// SPDX-License-Identifier: GPL-2.0
/*
 * The BCM4751 GPS receiver as Nokia's Harmattan kernel exposes it: /dev/bcm4751-gps passes raw I2C
 * reads and writes (at most 64 bytes) to the closed location stack (nped), which picks the address
 * and 10-bit mode itself with I2C_SLAVE and I2C_TENBIT; sysfs enable (power: the supplies and the
 * NSHUTDOWN GPIO), wakeup (a GPIO) and hostreq (the chip's host request line, which interrupts and
 * is notified to pollers).
 *
 * From drivers/misc/bcm4751-gps.c in github harmattan/kernel-plus-harmattan (Andrei Emeltchenko,
 * Yuri Zaporozhets), with the board callbacks of board-rm696-peripherals.c as GPIO descriptors,
 * probed from the device tree.
 */

#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

/* Max packet sizes for RX and TX */
#define BCM4751_MAX_BINPKT_RX_LEN	64
#define BCM4751_MAX_BINPKT_TX_LEN	64

struct bcm4751_gps_data {
	struct miscdevice		miscdev;
	struct i2c_client		*client;
	struct mutex			mutex; /* Serialize things */
	struct regulator_bulk_data	regs[2];
	struct gpio_desc		*gpio_irq;
	struct gpio_desc		*gpio_enable;
	struct gpio_desc		*gpio_wakeup;
	int				enable;
	int				wakeup;
};

static void bcm4751_gps_enable(struct bcm4751_gps_data *self)
{
	mutex_lock(&self->mutex);
	if (!self->enable) {
		if (regulator_bulk_enable(ARRAY_SIZE(self->regs), self->regs) == 0) {
			gpiod_set_value_cansleep(self->gpio_enable, 1);
			self->enable = 1;
		}
	}
	mutex_unlock(&self->mutex);
}

static void bcm4751_gps_disable(struct bcm4751_gps_data *self)
{
	mutex_lock(&self->mutex);
	if (self->enable) {
		gpiod_set_value_cansleep(self->gpio_enable, 0);
		self->enable = 0;
		regulator_bulk_disable(ARRAY_SIZE(self->regs), self->regs);
	}
	mutex_unlock(&self->mutex);
}

static void bcm4751_gps_wakeup_value(struct bcm4751_gps_data *self, int value)
{
	mutex_lock(&self->mutex);
	gpiod_set_value_cansleep(self->gpio_wakeup, value);
	self->wakeup = value;
	mutex_unlock(&self->mutex);
}

/*
 * miscdevice interface
 */

static ssize_t bcm4751_gps_read(struct file *file, char __user *buf, size_t count,
				loff_t *offset)
{
	struct bcm4751_gps_data *data = container_of(file->private_data, struct bcm4751_gps_data,
						     miscdev);
	struct i2c_client *client = data->client;
	u8 tmp[BCM4751_MAX_BINPKT_RX_LEN];
	int num_read;

	/* Adjust for binary packet size */
	if (count > BCM4751_MAX_BINPKT_RX_LEN)
		count = BCM4751_MAX_BINPKT_RX_LEN;

	num_read = i2c_master_recv(client, tmp, count);
	if (num_read < 0) {
		dev_err(&client->dev, "read of %zu bytes failed: %d\n", count, num_read);
		return num_read;
	}

	return copy_to_user(buf, tmp, num_read) ? -EFAULT : num_read;
}

static ssize_t bcm4751_gps_write(struct file *file, const char __user *buf, size_t count,
				 loff_t *offset)
{
	struct bcm4751_gps_data *data = container_of(file->private_data, struct bcm4751_gps_data,
						     miscdev);
	u8 tmp[BCM4751_MAX_BINPKT_TX_LEN];

	if (count > BCM4751_MAX_BINPKT_TX_LEN)
		count = BCM4751_MAX_BINPKT_TX_LEN;

	if (copy_from_user(tmp, buf, count))
		return -EFAULT;

	return i2c_master_send(data->client, tmp, count);
}

static long bcm4751_gps_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct bcm4751_gps_data *data = container_of(file->private_data, struct bcm4751_gps_data,
						     miscdev);
	struct i2c_client *client = data->client;

	dev_dbg(&client->dev, "ioctl: cmd = 0x%02x, arg=0x%02lx\n", cmd, arg);

	switch (cmd) {
	case I2C_SLAVE:
	case I2C_SLAVE_FORCE:
		if (arg > 0x3ff || (!(client->flags & I2C_CLIENT_TEN) && arg > 0x7f))
			return -EINVAL;
		client->addr = arg;
		return 0;
	case I2C_TENBIT:
		if (arg)
			client->flags |= I2C_CLIENT_TEN;
		else
			client->flags &= ~I2C_CLIENT_TEN;
		return 0;
	default:
		return -ENOTTY;
	}
}

static const struct file_operations bcm4751_gps_fileops = {
	.owner		= THIS_MODULE,
	.read		= bcm4751_gps_read,
	.write		= bcm4751_gps_write,
	.unlocked_ioctl	= bcm4751_gps_ioctl,
};

/*
 * sysfs interface
 */

static ssize_t hostreq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bcm4751_gps_data *self = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", gpiod_get_value_cansleep(self->gpio_irq));
}
static DEVICE_ATTR_RO(hostreq);

static ssize_t enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bcm4751_gps_data *self = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", self->enable);
}

static ssize_t enable_store(struct device *dev, struct device_attribute *attr, const char *buf,
			    size_t len)
{
	struct bcm4751_gps_data *self = dev_get_drvdata(dev);
	int value;

	if (kstrtoint(buf, 0, &value))
		return -EINVAL;

	switch (value) {
	case 0:
		bcm4751_gps_disable(self);
		break;
	case 1:
		bcm4751_gps_enable(self);
		break;
	default:
		return -EINVAL;
	}
	return len;
}
static DEVICE_ATTR_RW(enable);

static ssize_t wakeup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bcm4751_gps_data *self = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", self->wakeup);
}

static ssize_t wakeup_store(struct device *dev, struct device_attribute *attr, const char *buf,
			    size_t len)
{
	struct bcm4751_gps_data *self = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 0, &val) || val > 1)
		return -EINVAL;
	bcm4751_gps_wakeup_value(self, val);
	return len;
}
static DEVICE_ATTR_RW(wakeup);

static struct attribute *bcm4751_gps_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_hostreq.attr,
	&dev_attr_wakeup.attr,
	NULL
};
ATTRIBUTE_GROUPS(bcm4751_gps);

/* IRQ thread */
static irqreturn_t bcm4751_gps_irq_thread(int irq, void *dev_id)
{
	struct bcm4751_gps_data *data = dev_id;

	/* Update sysfs GPIO line here */
	sysfs_notify(&data->client->dev.kobj, NULL, "hostreq");
	return IRQ_HANDLED;
}

static int bcm4751_gps_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct bcm4751_gps_data *data;
	int err, irq;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	i2c_set_clientdata(client, data);
	data->client = client;
	mutex_init(&data->mutex);

	data->regs[0].supply = "Vbat";
	data->regs[1].supply = "Vddio";
	err = devm_regulator_bulk_get(dev, ARRAY_SIZE(data->regs), data->regs);
	if (err)
		return dev_err_probe(dev, err, "Can't get regulators\n");

	/* As the board's setup: NSHUTDOWN (GPS_ENABLE) and WAKEUP low, HOST_REQ an input */
	data->gpio_enable = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(data->gpio_enable))
		return dev_err_probe(dev, PTR_ERR(data->gpio_enable), "GPS_ENABLE\n");
	data->gpio_wakeup = devm_gpiod_get(dev, "wakeup", GPIOD_OUT_LOW);
	if (IS_ERR(data->gpio_wakeup))
		return dev_err_probe(dev, PTR_ERR(data->gpio_wakeup), "GPS_WAKEUP\n");
	data->gpio_irq = devm_gpiod_get(dev, "host-req", GPIOD_IN);
	if (IS_ERR(data->gpio_irq))
		return dev_err_probe(dev, PTR_ERR(data->gpio_irq), "HOST_REQ\n");

	irq = gpiod_to_irq(data->gpio_irq);
	if (irq < 0)
		return dev_err_probe(dev, irq, "HOST_REQ has no IRQ\n");
	err = devm_request_threaded_irq(dev, irq, NULL, bcm4751_gps_irq_thread,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT, "bcm4751-gps", data);
	if (err)
		return dev_err_probe(dev, err, "could not get GPS_IRQ %d\n", irq);

	data->miscdev.minor = MISC_DYNAMIC_MINOR;
	data->miscdev.name = "bcm4751-gps";
	data->miscdev.fops = &bcm4751_gps_fileops;
	data->miscdev.parent = dev;
	err = misc_register(&data->miscdev);
	if (err)
		return dev_err_probe(dev, err, "Miscdevice register failed\n");

	return 0;
}

static void bcm4751_gps_remove(struct i2c_client *client)
{
	struct bcm4751_gps_data *data = i2c_get_clientdata(client);

	misc_deregister(&data->miscdev);
	bcm4751_gps_disable(data);
}

static void bcm4751_gps_shutdown(struct i2c_client *client)
{
	bcm4751_gps_disable(i2c_get_clientdata(client));
}

static const struct of_device_id bcm4751_gps_of_match[] = {
	{ .compatible = "nokia,bcm4751-gps" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm4751_gps_of_match);

static struct i2c_driver bcm4751_gps_i2c_driver = {
	.driver = {
		.name		= "bcm4751-gps",
		.of_match_table	= bcm4751_gps_of_match,
		.dev_groups	= bcm4751_gps_groups,
	},
	.probe		= bcm4751_gps_probe,
	.remove		= bcm4751_gps_remove,
	.shutdown	= bcm4751_gps_shutdown,
};
module_i2c_driver(bcm4751_gps_i2c_driver);

MODULE_AUTHOR("Andrei Emeltchenko, Yuri Zaporozhets");
MODULE_DESCRIPTION("BCM4751 GPS driver");
MODULE_LICENSE("GPL");
