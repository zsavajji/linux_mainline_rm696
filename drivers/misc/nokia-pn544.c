// SPDX-License-Identifier: GPL-2.0
/*
 * The PN544 NFC controller as Nokia's Harmattan kernel exposes it: /dev/pn544, one LLC frame (length,
 * header, data, CRC-CCITT, built by Harmattan's libNFCDAL) per write() and read(), poll() on the
 * chip's interrupt; open() powers the chip up in HCI mode, release() powers it down; ioctl
 * PN544_SET_FW_MODE restarts it in firmware download mode (the firmware GPIO high), where frames are
 * the chip's 3-byte header and data instead. Harmattan's nfcd compares the chip's firmware with the
 * one in libNXP_NFC_HAL at every start and rewrites the chip's flash on a mismatch: here firmware
 * mode is refused unless allow_fw_mode is set (stock allowed it).
 *
 * From drivers/misc/pn544.c in github harmattan/kernel-plus-harmattan (Jari Vanhala, Matti Aaltonen),
 * as stock PR1.3's pn544.ko, with the board callbacks of board-rm696-peripherals.c as GPIO descriptors,
 * probed from the device tree. Left out: sysfs nfc_test (the board's GPIO test, read only by
 * stserver's factory tests at a 2.6.32 path; it left the chip on in firmware mode).
 */

#include <linux/crc-ccitt.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/poll.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/termios.h>
#include <linux/uaccess.h>

#define PN544_DRIVER_NAME	"pn544"

#define PN544_MSG_MAX_SIZE	0x21 /* HCI mode */
#define PN544_MAX_I2C_TRANSFER	0x0400

/* IOCTLs, as stock's include/linux/pn544.h */
#define PN544_CHAR_BASE		'P'
#define PN544_GET_FW_MODE	_IOR(PN544_CHAR_BASE, 1, unsigned int)
#define PN544_SET_FW_MODE	_IOW(PN544_CHAR_BASE, 2, unsigned int)
#define PN544_GET_DEBUG		_IOR(PN544_CHAR_BASE, 3, unsigned int)
#define PN544_SET_DEBUG		_IOW(PN544_CHAR_BASE, 4, unsigned int)
#define PN544_GET_IODELAY	_IOR(PN544_CHAR_BASE, 5, unsigned int)
#define PN544_SET_IODELAY	_IOW(PN544_CHAR_BASE, 6, unsigned int)
#define PN544_GET_RESETDELAY	_IOR(PN544_CHAR_BASE, 7, unsigned int)
#define PN544_SET_RESETDELAY	_IOW(PN544_CHAR_BASE, 8, unsigned int)

/* Normal (HCI) mode */
#define PN544_LLC_HCI_OVERHEAD	3 /* header + crc (to length) */
#define PN544_LLC_MIN_SIZE	(1 + PN544_LLC_HCI_OVERHEAD) /* length + */

/* Firmware upgrade mode */
#define PN544_FW_HEADER_SIZE	3

/* Timing restrictions */
#define PN544_RESETVEN_TIME	30 /* ms */
#define PN544_WAKEUP_ACK	5 /* ms */
#define PN544_BOOT_TIME		10000 /* us */
#define PN544_WAKEUP_GUARD	(PN544_WAKEUP_ACK + 1)
#define PN544_WAKEUP_GUARD_US	(PN544_WAKEUP_GUARD * 1000)
#define PN544_IO_DELAY		300 /* us */
#define PN544_RESET_DELAY	102000 /* us */
#define PN544_RESET_BYTE	0xf9
#define PN544_RESET_MSG_LEN	5

static int debug;
static bool allow_fw_mode;
static unsigned int io_delay    = PN544_IO_DELAY;
static unsigned int reset_delay = PN544_RESET_DELAY;

#define HCI_MODE	0
#define FW_MODE		1

enum pn544_state {
	PN544_ST_COLD,
	PN544_ST_FW_READY,
	PN544_ST_READY,
};

enum pn544_irq {
	PN544_NONE,
	PN544_INT,
};

struct pn544_info {
	struct miscdevice miscdev;
	struct i2c_client *i2c_dev;
	struct regulator *vdd_io;	/* VBat: straight from the battery */
	struct gpio_desc *gpio_enable;
	struct gpio_desc *gpio_fw;

	struct mutex ioctl_mutex; /* stock's ioctl ran under the BKL */
	enum pn544_state state;
	wait_queue_head_t read_wait;
	enum pn544_irq read_irq;
	struct mutex read_mutex; /* read_irq */
	u8 *buf;
	unsigned int buflen;

	unsigned int io_delay; /* minimum time between transactions */
	unsigned int reset_delay; /* minimum idle time before reset */
	unsigned long lastio; /* timestamp for the last I2C transaction */
};

/* current time in microseconds */
static inline unsigned long pn544_now(void)
{
	return (unsigned long)ktime_to_us(ktime_get());
}

static void pn544_io_delay(struct pn544_info *info, int delay)
{
	if (info->lastio) {
		const unsigned long now = pn544_now();
		const unsigned long nextio = info->lastio + delay;

		if (nextio > now) {
			const unsigned long wait_min = nextio - now;

			dev_dbg(&info->i2c_dev->dev, "waiting %lu us\n", wait_min);
			usleep_range(wait_min, wait_min + 200);
		}
	}
}

static int pn544_enable(struct pn544_info *info, int mode)
{
	int r;

	r = regulator_enable(info->vdd_io);
	if (r < 0)
		return r;

	info->read_irq = PN544_NONE;
	/* The board's enable callback: the firmware GPIO picks the mode the chip starts in */
	gpiod_set_value_cansleep(info->gpio_fw, mode);
	gpiod_set_value_cansleep(info->gpio_enable, 1);

	if (mode) {
		info->state = PN544_ST_FW_READY;
		dev_dbg(&info->i2c_dev->dev, "now in FW-mode\n");
	} else {
		info->state = PN544_ST_READY;
		dev_dbg(&info->i2c_dev->dev, "now in HCI-mode\n");
	}

	usleep_range(PN544_BOOT_TIME, (PN544_BOOT_TIME * 3) / 2);

	return 0;
}

static void pn544_disable(struct pn544_info *info)
{
	gpiod_set_value_cansleep(info->gpio_enable, 0);
	info->state = PN544_ST_COLD;

	dev_dbg(&info->i2c_dev->dev, "Now in OFF-mode\n");

	msleep(PN544_RESETVEN_TIME);

	info->read_irq = PN544_NONE;
	regulator_disable(info->vdd_io);
}

static int check_crc(u8 *buf, int buflen)
{
	u8 len;
	u16 crc;

	len = buf[0] + 1;
	if (len < 4 || len != buflen || len > PN544_MSG_MAX_SIZE) {
		pr_err(PN544_DRIVER_NAME ": CRC; corrupted packet len %u (%d)\n", len, buflen);
		print_hex_dump_debug("crc: ", DUMP_PREFIX_NONE, 16, 1, buf, buflen, false);
		return -EPERM;
	}
	crc = crc_ccitt(0xffff, buf, len - 2);
	crc = ~crc;

	if (buf[len - 2] != (crc & 0xff) || buf[len - 1] != (crc >> 8)) {
		pr_err(PN544_DRIVER_NAME ": CRC error 0x%x != 0x%x 0x%x\n",
		       crc, buf[len - 1], buf[len - 2]);
		print_hex_dump_debug("crc: ", DUMP_PREFIX_NONE, 16, 1, buf, buflen, false);
		return -EPERM;
	}
	return 0;
}

static int pn544_i2c_write(struct pn544_info *info, u8 *buf, int len)
{
	struct i2c_client *client = info->i2c_dev;
	int r;

	if (len < 4 || len != (buf[0] + 1) || check_crc(buf, len))
		return -EBADMSG;

	if (len >= PN544_RESET_MSG_LEN && buf[1] == PN544_RESET_BYTE) {
		/* Longer delay before the reset */
		dev_dbg(&client->dev, "reset detected\n");
		pn544_io_delay(info, info->reset_delay);
	} else {
		/* Normal case */
		pn544_io_delay(info, info->io_delay);
	}

	r = i2c_master_send(client, buf, len);
	dev_dbg(&client->dev, "send: %d\n", r);

	if (r == -EREMOTEIO) { /* Retry, chip was in standby */
		usleep_range(PN544_WAKEUP_GUARD_US, 2 * PN544_WAKEUP_GUARD_US);
		r = i2c_master_send(client, buf, len);
		dev_dbg(&client->dev, "send2: %d\n", r);
	}

	if (r != len)
		return -EREMOTEIO;

	info->lastio = pn544_now();
	return r;
}

static int pn544_i2c_read(struct pn544_info *info, u8 *buf, int buflen)
{
	struct i2c_client *client = info->i2c_dev;
	int r;
	u8 len;

	/*
	 * You could read a packet in one go, but then you'd need to read
	 * max size and rest would be 0xff fill, so we do split reads.
	 */
	r = i2c_master_recv(client, &len, 1);
	dev_dbg(&client->dev, "recv1: %d\n", r);

	if (r != 1)
		return -EREMOTEIO;

	if (len < PN544_LLC_HCI_OVERHEAD)
		len = PN544_LLC_HCI_OVERHEAD;
	else if (len > (PN544_MSG_MAX_SIZE - 1))
		len = PN544_MSG_MAX_SIZE - 1;

	if (1 + len > buflen) /* len+(data+crc16) */
		return -EOVERFLOW;

	buf[0] = len;

	r = i2c_master_recv(client, buf + 1, len);
	dev_dbg(&client->dev, "recv2: %d\n", r);

	if (r == len) {
		r += 1;
		info->lastio = pn544_now();
	} else {
		r = -EREMOTEIO;
	}

	return r;
}

static int pn544_fw_write(struct i2c_client *client, u8 *buf, int len)
{
	int r;

	dev_dbg(&client->dev, "%s\n", __func__);

	if (len < PN544_FW_HEADER_SIZE ||
	    (PN544_FW_HEADER_SIZE + (buf[1] << 8) + buf[2]) != len)
		return -EBADMSG;

	r = i2c_master_send(client, buf, len);
	dev_dbg(&client->dev, "fw send: %d\n", r);

	if (r == -EREMOTEIO) { /* Retry, chip was in standby */
		usleep_range(PN544_WAKEUP_GUARD_US, 2 * PN544_WAKEUP_GUARD_US);
		r = i2c_master_send(client, buf, len);
		dev_dbg(&client->dev, "fw send2: %d\n", r);
	}

	if (r != len)
		return -EREMOTEIO;

	return r;
}

static int pn544_fw_read(struct i2c_client *client, u8 *buf, int buflen)
{
	int r, len;

	if (buflen < PN544_FW_HEADER_SIZE)
		return -ENOSPC;

	r = i2c_master_recv(client, buf, PN544_FW_HEADER_SIZE);
	dev_dbg(&client->dev, "FW recv1: %d\n", r);

	if (r < 0)
		return r;

	if (r < PN544_FW_HEADER_SIZE)
		return -EBADMSG;

	len = (buf[1] << 8) + buf[2];
	if (len == 0) /* just header, no additional data */
		return r;

	if (len > buflen - PN544_FW_HEADER_SIZE)
		return -ENOSPC;

	r = i2c_master_recv(client, buf + PN544_FW_HEADER_SIZE, len);
	dev_dbg(&client->dev, "fw recv2: %d\n", r);

	if (r != len)
		return -EBADMSG;

	return r + PN544_FW_HEADER_SIZE;
}

static irqreturn_t pn544_irq_thread_fn(int irq, void *dev_id)
{
	struct pn544_info *info = dev_id;

	dev_dbg(&info->i2c_dev->dev, "IRQ\n");

	mutex_lock(&info->read_mutex);
	info->read_irq = PN544_INT;
	mutex_unlock(&info->read_mutex);

	wake_up_interruptible(&info->read_wait);

	return IRQ_HANDLED;
}

static enum pn544_irq pn544_irq_state(struct pn544_info *info)
{
	enum pn544_irq irq;

	mutex_lock(&info->read_mutex);
	irq = info->read_irq;
	mutex_unlock(&info->read_mutex);

	return irq;
}

static ssize_t pn544_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	struct pn544_info *info = container_of(file->private_data, struct pn544_info, miscdev);
	size_t len;
	int r;

	dev_dbg(&info->i2c_dev->dev, "%s: count: %zu\n", __func__, count);

	if (info->state == PN544_ST_COLD)
		return -ENODEV;

	if (pn544_irq_state(info) == PN544_NONE) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		if (wait_event_interruptible(info->read_wait, (info->read_irq == PN544_INT)))
			return -ERESTARTSYS;
	}

	len = min_t(size_t, count, info->buflen);

	mutex_lock(&info->read_mutex);
	if (info->state == PN544_ST_FW_READY)
		r = pn544_fw_read(info->i2c_dev, info->buf, len);
	else
		r = pn544_i2c_read(info, info->buf, len);
	info->read_irq = PN544_NONE;
	mutex_unlock(&info->read_mutex);

	if (r < 0) {
		dev_err(&info->i2c_dev->dev, "%s failed: %d\n",
			info->state == PN544_ST_FW_READY ? "FW read" : "read", r);
		return r;
	}
	print_hex_dump_debug("read: ", DUMP_PREFIX_NONE, 16, 1, info->buf, r, false);

	if (copy_to_user(buf, info->buf, r))
		return -EFAULT;

	return r;
}

static __poll_t pn544_poll(struct file *file, poll_table *wait)
{
	struct pn544_info *info = container_of(file->private_data, struct pn544_info, miscdev);

	if (info->state == PN544_ST_COLD)
		return EPOLLERR;

	poll_wait(file, &info->read_wait, wait);

	if (pn544_irq_state(info) == PN544_INT)
		return EPOLLIN | EPOLLRDNORM;

	return 0;
}

static ssize_t pn544_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	struct pn544_info *info = container_of(file->private_data, struct pn544_info, miscdev);
	size_t len;

	dev_dbg(&info->i2c_dev->dev, "%s: count %zu\n", __func__, count);

	if (info->state == PN544_ST_COLD)
		return -ENODEV;

	if (info->state == PN544_ST_FW_READY) {
		size_t fw_len;

		if (count < PN544_FW_HEADER_SIZE)
			return -EINVAL;

		len = min_t(size_t, count, info->buflen);
		if (copy_from_user(info->buf, buf, len))
			return -EFAULT;

		fw_len = PN544_FW_HEADER_SIZE + (info->buf[1] << 8) + info->buf[2];
		if (len > fw_len) /* 1 msg at a time */
			len = fw_len;

		return pn544_fw_write(info->i2c_dev, info->buf, len);
	}

	if (count < PN544_LLC_MIN_SIZE)
		return -EINVAL;

	len = min_t(size_t, count, info->buflen);
	if (copy_from_user(info->buf, buf, len))
		return -EFAULT;

	print_hex_dump_debug("write: ", DUMP_PREFIX_NONE, 16, 1, info->buf, len, false);
	if (len > (info->buf[0] + 1)) /* 1 msg at a time */
		len = info->buf[0] + 1;

	return pn544_i2c_write(info, info->buf, len);
}

static long pn544_do_ioctl(struct pn544_info *info, unsigned int cmd, unsigned long arg)
{
	struct device *dev = &info->i2c_dev->dev;
	void __user *uarg = (void __user *)arg;
	unsigned int val;
	int r;

	if (info->state == PN544_ST_COLD)
		return -ENODEV;

	switch (cmd) {
	case PN544_GET_FW_MODE:
		val = (info->state == PN544_ST_FW_READY);
		return put_user(val, (unsigned int __user *)uarg);

	case PN544_SET_FW_MODE:
		if (get_user(val, (unsigned int __user *)uarg))
			return -EFAULT;
		if (val) {
			if (info->state == PN544_ST_FW_READY)
				break;
			if (!allow_fw_mode) {
				dev_warn(dev, "firmware download mode refused (allow_fw_mode=0)\n");
				return -EPERM;
			}
			dev_info(dev, "firmware download mode\n");
			pn544_disable(info);
			r = pn544_enable(info, FW_MODE);
			if (r < 0)
				return r;
		} else {
			if (info->state == PN544_ST_READY)
				break;
			pn544_disable(info);
			r = pn544_enable(info, HCI_MODE);
			if (r < 0)
				return r;
		}
		break;

	case PN544_GET_DEBUG:
		return put_user(debug, (unsigned int __user *)uarg);

	case PN544_SET_DEBUG:
		if (get_user(val, (unsigned int __user *)uarg))
			return -EFAULT;
		debug = val;
		break;

	case PN544_GET_IODELAY:
		return put_user(info->io_delay, (unsigned int __user *)uarg);

	case PN544_SET_IODELAY:
		if (get_user(val, (unsigned int __user *)uarg))
			return -EFAULT;
		info->io_delay = val;
		break;

	case PN544_GET_RESETDELAY:
		return put_user(info->reset_delay, (unsigned int __user *)uarg);

	case PN544_SET_RESETDELAY:
		if (get_user(val, (unsigned int __user *)uarg))
			return -EFAULT;
		info->reset_delay = val;
		break;

	case TCGETS:
		return -ENOIOCTLCMD;

	default:
		dev_err(dev, "Unknown ioctl 0x%x\n", cmd);
		return -ENOIOCTLCMD;
	}

	return 0;
}

static long pn544_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct pn544_info *info = container_of(file->private_data, struct pn544_info, miscdev);
	long r;

	mutex_lock(&info->ioctl_mutex);
	r = pn544_do_ioctl(info, cmd, arg);
	mutex_unlock(&info->ioctl_mutex);

	return r;
}

static int pn544_open(struct inode *inode, struct file *file)
{
	struct pn544_info *info = container_of(file->private_data, struct pn544_info, miscdev);
	int r = -EBUSY;

	/* Only 1 at a time */
	mutex_lock(&info->ioctl_mutex);
	if (info->state == PN544_ST_COLD)
		r = pn544_enable(info, HCI_MODE);
	mutex_unlock(&info->ioctl_mutex);

	return r;
}

static int pn544_close(struct inode *inode, struct file *file)
{
	struct pn544_info *info = container_of(file->private_data, struct pn544_info, miscdev);

	mutex_lock(&info->ioctl_mutex);
	if (info->state != PN544_ST_COLD)
		pn544_disable(info);
	mutex_unlock(&info->ioctl_mutex);

	return 0;
}

static const struct file_operations pn544_fops = {
	.owner		= THIS_MODULE,
	.read		= pn544_read,
	.write		= pn544_write,
	.poll		= pn544_poll,
	.open		= pn544_open,
	.release	= pn544_close,
	.unlocked_ioctl	= pn544_ioctl,
};

/* Stock's probe-time check that the chip answers: a U-RSET frame and its UA reply */
static int pn544_rset_test(struct pn544_info *info)
{
	struct i2c_client *client = info->i2c_dev;
	/* U RSET frame without baudrate */
	u8 rset[] = { 0x05, 0xf9, 0x04, 0x00, 0xc3, 0xe5 };
	static const u8 answer[] = { 0x03, 0xe6, 0x17, 0xa7 };
	u8 read_buf[PN544_MSG_MAX_SIZE];
	int r;

	r = pn544_enable(info, HCI_MODE);
	if (r < 0)
		return r;

	r = pn544_i2c_write(info, rset, sizeof(rset));
	if (r < 0) {
		dev_err(&client->dev, "Write error to rset: %d\n", r);
		goto out;
	}

	wait_event_timeout(info->read_wait, (info->read_irq == PN544_INT),
			   msecs_to_jiffies(PN544_WAKEUP_GUARD));
	if (info->read_irq == PN544_NONE) {
		dev_err(&client->dev, "No answer to rset\n");
		r = -ENODEV;
		goto out;
	}

	mutex_lock(&info->read_mutex);
	r = pn544_i2c_read(info, read_buf, sizeof(read_buf));
	info->read_irq = PN544_NONE;
	mutex_unlock(&info->read_mutex);

	if (r < 0) {
		dev_err(&client->dev, "Read error to rset: %d\n", r);
		goto out;
	}

	if (r != sizeof(answer) || memcmp(read_buf, answer, r)) {
		dev_err(&client->dev, "Wrong answer to rset: %*ph\n", r, read_buf);
		r = -ENODEV;
		goto out;
	}

	dev_dbg(&client->dev, "RSET test ok.\n");
	r = 0;
out:
	pn544_disable(info);
	return r;
}

static int pn544_suspend(struct device *dev)
{
	struct pn544_info *info = dev_get_drvdata(dev);

	/* Do not suspend while upgrading FW, please! */
	return info->state == PN544_ST_FW_READY ? -EPERM : 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(pn544_pm_ops, pn544_suspend, NULL);

static int pn544_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct pn544_info *info;
	int r;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->buflen = max(PN544_MSG_MAX_SIZE, PN544_MAX_I2C_TRANSFER);
	info->buf = devm_kzalloc(dev, info->buflen, GFP_KERNEL);
	if (!info->buf)
		return -ENOMEM;

	info->vdd_io = devm_regulator_get(dev, "Vdd_IO");
	if (IS_ERR(info->vdd_io))
		return dev_err_probe(dev, PTR_ERR(info->vdd_io), "Can't get Vdd_IO\n");

	/* As the board's setup: ENABLE and FW_RESET low (off, HCI mode) */
	info->gpio_enable = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(info->gpio_enable))
		return dev_err_probe(dev, PTR_ERR(info->gpio_enable), "NFC Enable\n");
	info->gpio_fw = devm_gpiod_get(dev, "firmware", GPIOD_OUT_LOW);
	if (IS_ERR(info->gpio_fw))
		return dev_err_probe(dev, PTR_ERR(info->gpio_fw), "NFC FW Reset\n");

	info->i2c_dev = client;
	info->state = PN544_ST_COLD;
	info->read_irq = PN544_NONE;
	info->reset_delay = reset_delay;
	info->io_delay = io_delay;
	info->lastio = pn544_now();
	mutex_init(&info->read_mutex);
	mutex_init(&info->ioctl_mutex);
	init_waitqueue_head(&info->read_wait);
	i2c_set_clientdata(client, info);

	r = devm_request_threaded_irq(dev, client->irq, NULL, pn544_irq_thread_fn,
				      IRQF_TRIGGER_RISING | IRQF_ONESHOT, PN544_DRIVER_NAME, info);
	if (r)
		return dev_err_probe(dev, r, "Unable to register IRQ handler\n");

	r = pn544_rset_test(info);
	if (r < 0)
		return r;

	info->miscdev.minor = MISC_DYNAMIC_MINOR;
	info->miscdev.name = PN544_DRIVER_NAME;
	info->miscdev.fops = &pn544_fops;
	info->miscdev.parent = dev;
	r = misc_register(&info->miscdev);
	if (r)
		return dev_err_probe(dev, r, "Device registration failed\n");

	return 0;
}

static void pn544_remove(struct i2c_client *client)
{
	struct pn544_info *info = i2c_get_clientdata(client);

	misc_deregister(&info->miscdev);
	if (info->state != PN544_ST_COLD)
		pn544_disable(info);
}

static const struct of_device_id pn544_of_match[] = {
	{ .compatible = "nokia,n9-pn544" },
	{ }
};
MODULE_DEVICE_TABLE(of, pn544_of_match);

static struct i2c_driver pn544_driver = {
	.driver = {
		.name		= PN544_DRIVER_NAME,
		.of_match_table	= pn544_of_match,
		.pm		= pm_sleep_ptr(&pn544_pm_ops),
	},
	.probe		= pn544_probe,
	.remove		= pn544_remove,
};
module_i2c_driver(pn544_driver);

module_param(allow_fw_mode, bool, 0644);
MODULE_PARM_DESC(allow_fw_mode, "Allow firmware download mode (a rewrite of the chip's flash), default: no");
module_param(io_delay, uint, 0444);
MODULE_PARM_DESC(io_delay, "Minimum time between I2C transactions in us, default: "
		 __stringify(PN544_IO_DELAY));
module_param(reset_delay, uint, 0444);
MODULE_PARM_DESC(reset_delay, "Minimum idle time before pn544 reset in us, default: "
		 __stringify(PN544_RESET_DELAY));

MODULE_AUTHOR("Jari Vanhala <ext-jari.vanhala@nokia.com>");
MODULE_DESCRIPTION("NFC driver for PN544");
MODULE_LICENSE("GPL");
