// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  Bluetooth HCI UART H4 driver with Nokia Extensions AKA Nokia H4+
 *
 *  Copyright (C) 2015 Marcel Holtmann <marcel@holtmann.org>
 *  Copyright (C) 2015-2017 Sebastian Reichel <sre@kernel.org>
 */

#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/serdev.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "hci_uart.h"
#include "btbcm.h"

#define VERSION "0.1"

#define NOKIA_ID_BCM2048	0x04
#define NOKIA_ID_TI1271		0x31

#define FIRMWARE_BCM2048	"nokia/bcmfw.bin"
#define FIRMWARE_TI1271		"nokia/ti1273.bin"
#define FIRMWARE_TI1271_LE	"nokia/ti1273_le.bin"
#define FIRMWARE_TI1271_PRELE	"nokia/ti1273_prele.bin"

/*
 * The TI1271 at 921600 after its firmware (which leaves it at 3.69 Mbaud), with TI's own command for
 * it (as hci_ll for WiLink): at 3.69 Mbaud the OMAP3 UART loses received bytes (frame reassembly
 * failures, lost inquiry events), with or without DMA.
 */
#define TI1271_BAUD_RATE		921600
#define HCI_VS_UPDATE_UART_HCI_BAUDRATE	0xff36

#define HCI_NOKIA_NEG_PKT	0x06
#define HCI_NOKIA_ALIVE_PKT	0x07
#define HCI_NOKIA_RADIO_PKT	0x08

#define HCI_NOKIA_NEG_HDR_SIZE		1
#define HCI_NOKIA_MAX_NEG_SIZE		255
#define HCI_NOKIA_ALIVE_HDR_SIZE	1
#define HCI_NOKIA_MAX_ALIVE_SIZE	255
#define HCI_NOKIA_RADIO_HDR_SIZE	2
#define HCI_NOKIA_MAX_RADIO_SIZE	255

#define NOKIA_PROTO_PKT		0x44
#define NOKIA_PROTO_BYTE	0x4c

#define NOKIA_NEG_REQ		0x00
#define NOKIA_NEG_ACK		0x20
#define NOKIA_NEG_NAK		0x40

#define H4_TYPE_SIZE		1

#define NOKIA_RECV_ALIVE \
	.type = HCI_NOKIA_ALIVE_PKT, \
	.hlen = HCI_NOKIA_ALIVE_HDR_SIZE, \
	.loff = 0, \
	.lsize = 1, \
	.maxlen = HCI_NOKIA_MAX_ALIVE_SIZE \

#define NOKIA_RECV_NEG \
	.type = HCI_NOKIA_NEG_PKT, \
	.hlen = HCI_NOKIA_NEG_HDR_SIZE, \
	.loff = 0, \
	.lsize = 1, \
	.maxlen = HCI_NOKIA_MAX_NEG_SIZE \

#define NOKIA_RECV_RADIO \
	.type = HCI_NOKIA_RADIO_PKT, \
	.hlen = HCI_NOKIA_RADIO_HDR_SIZE, \
	.loff = 1, \
	.lsize = 1, \
	.maxlen = HCI_NOKIA_MAX_RADIO_SIZE \

struct hci_nokia_neg_hdr {
	u8	dlen;
} __packed;

struct hci_nokia_neg_cmd {
	u8	ack;
	u16	baud;
	u16	unused1;
	u8	proto;
	u16	sys_clk;
	u16	unused2;
} __packed;

#define NOKIA_ALIVE_REQ   0x55
#define NOKIA_ALIVE_RESP  0xcc

struct hci_nokia_alive_hdr {
	u8	dlen;
} __packed;

struct hci_nokia_alive_pkt {
	u8	mid;
	u8	unused;
} __packed;

struct hci_nokia_neg_evt {
	u8	ack;
	u16	baud;
	u16	unused1;
	u8	proto;
	u16	sys_clk;
	u16	unused2;
	u8	man_id;
	u8	ver_id;
} __packed;

#define MAX_BAUD_RATE		3692300
#define SETUP_BAUD_RATE		921600
#define INIT_BAUD_RATE		120000

struct nokia_bt_dev {
	struct hci_uart hu;
	struct serdev_device *serdev;

	struct gpio_desc *reset;
	struct gpio_desc *wakeup_host;
	struct gpio_desc *wakeup_bt;
	unsigned long sysclk_speed;

	int wake_irq;
	struct sk_buff *rx_skb;
	struct sk_buff_head txq;
	bdaddr_t bdaddr;

	int init_error;
	struct completion init_completion;

	u8 man_id;
	u8 ver_id;

	bool initialized;
	bool tx_enabled;
	bool rx_enabled;
	bool uart_released;
	int (*hci_close)(struct hci_dev *hdev);

	struct mutex tx_lock;		/* tx_enabled, tx_idle */
	bool tx_idle;
	struct delayed_work tx_release;

	unsigned long rx_last;
};

static int nokia_enqueue(struct hci_uart *hu, struct sk_buff *skb);

static void nokia_flow_control(struct serdev_device *serdev, bool enable)
{
	if (enable) {
		serdev_device_set_rts(serdev, true);
		serdev_device_set_flow_control(serdev, true);
	} else {
		serdev_device_set_flow_control(serdev, false);
		serdev_device_set_rts(serdev, false);
	}
}

static irqreturn_t wakeup_handler(int irq, void *data)
{
	struct nokia_bt_dev *btdev = data;
	struct device *dev = &btdev->serdev->dev;
	int wake_state = gpiod_get_value(btdev->wakeup_host);

	if (btdev->rx_enabled == wake_state)
		return IRQ_HANDLED;

	if (wake_state)
		pm_runtime_get(dev);
	else
		pm_runtime_put(dev);

	btdev->rx_enabled = wake_state;

	return IRQ_HANDLED;
}

static int nokia_reset(struct hci_uart *hu)
{
	struct nokia_bt_dev *btdev = hu->priv;
	struct device *dev = &btdev->serdev->dev;
	int err;

	/* reset routine */
	gpiod_set_value_cansleep(btdev->reset, 1);
	gpiod_set_value_cansleep(btdev->wakeup_bt, 1);

	msleep(100);

	/* safety check */
	err = gpiod_get_value_cansleep(btdev->wakeup_host);
	if (err == 1) {
		dev_err(dev, "reset: host wakeup not low!");
		return -EPROTO;
	}

	/* flush queue */
	serdev_device_write_flush(btdev->serdev);

	/* init uart */
	nokia_flow_control(btdev->serdev, false);
	serdev_device_set_baudrate(btdev->serdev, INIT_BAUD_RATE);

	gpiod_set_value_cansleep(btdev->reset, 0);

	/* wait for cts */
	err = serdev_device_wait_for_cts(btdev->serdev, true, 200);
	if (err < 0) {
		dev_err(dev, "CTS not received: %d", err);
		return err;
	}

	nokia_flow_control(btdev->serdev, true);

	return 0;
}

static int nokia_send_alive_packet(struct hci_uart *hu)
{
	struct nokia_bt_dev *btdev = hu->priv;
	struct device *dev = &btdev->serdev->dev;
	struct hci_nokia_alive_hdr *hdr;
	struct hci_nokia_alive_pkt *pkt;
	struct sk_buff *skb;
	int len;

	init_completion(&btdev->init_completion);

	len = H4_TYPE_SIZE + sizeof(*hdr) + sizeof(*pkt);
	skb = bt_skb_alloc(len, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	hci_skb_pkt_type(skb) = HCI_NOKIA_ALIVE_PKT;
	memset(skb->data, 0x00, len);

	hdr = skb_put(skb, sizeof(*hdr));
	hdr->dlen = sizeof(*pkt);
	pkt = skb_put(skb, sizeof(*pkt));
	pkt->mid = NOKIA_ALIVE_REQ;

	nokia_enqueue(hu, skb);
	hci_uart_tx_wakeup(hu);

	dev_dbg(dev, "Alive sent");

	if (!wait_for_completion_interruptible_timeout(&btdev->init_completion,
		msecs_to_jiffies(1000))) {
		return -ETIMEDOUT;
	}

	if (btdev->init_error < 0)
		return btdev->init_error;

	return 0;
}

static int nokia_send_negotiation(struct hci_uart *hu)
{
	struct nokia_bt_dev *btdev = hu->priv;
	struct device *dev = &btdev->serdev->dev;
	struct hci_nokia_neg_cmd *neg_cmd;
	struct hci_nokia_neg_hdr *neg_hdr;
	struct sk_buff *skb;
	int len, err;
	u16 baud = DIV_ROUND_CLOSEST(btdev->sysclk_speed * 10, SETUP_BAUD_RATE);
	int sysclk = btdev->sysclk_speed / 1000;

	len = H4_TYPE_SIZE + sizeof(*neg_hdr) + sizeof(*neg_cmd);
	skb = bt_skb_alloc(len, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	hci_skb_pkt_type(skb) = HCI_NOKIA_NEG_PKT;

	neg_hdr = skb_put(skb, sizeof(*neg_hdr));
	neg_hdr->dlen = sizeof(*neg_cmd);

	neg_cmd = skb_put(skb, sizeof(*neg_cmd));
	neg_cmd->ack = NOKIA_NEG_REQ;
	neg_cmd->baud = cpu_to_le16(baud);
	neg_cmd->unused1 = 0x0000;
	neg_cmd->proto = NOKIA_PROTO_BYTE;
	neg_cmd->sys_clk = cpu_to_le16(sysclk);
	neg_cmd->unused2 = 0x0000;

	btdev->init_error = 0;
	init_completion(&btdev->init_completion);

	nokia_enqueue(hu, skb);
	hci_uart_tx_wakeup(hu);

	dev_dbg(dev, "Negotiation sent");

	if (!wait_for_completion_interruptible_timeout(&btdev->init_completion,
		msecs_to_jiffies(10000))) {
		return -ETIMEDOUT;
	}

	if (btdev->init_error < 0)
		return btdev->init_error;

	/* Change to previously negotiated speed. Flow Control
	 * is disabled until bluetooth adapter is ready to avoid
	 * broken bytes being received.
	 */
	nokia_flow_control(btdev->serdev, false);
	serdev_device_set_baudrate(btdev->serdev, SETUP_BAUD_RATE);
	err = serdev_device_wait_for_cts(btdev->serdev, true, 200);
	if (err < 0) {
		dev_err(dev, "CTS not received: %d", err);
		return err;
	}
	nokia_flow_control(btdev->serdev, true);

	dev_dbg(dev, "Negotiation successful");

	return 0;
}

static int nokia_setup_fw(struct hci_uart *hu)
{
	struct nokia_bt_dev *btdev = hu->priv;
	struct device *dev = &btdev->serdev->dev;
	const char *fwname;
	const struct firmware *fw;
	const u8 *fw_ptr;
	size_t fw_size;
	int err;

	dev_dbg(dev, "setup firmware");

	if (btdev->man_id == NOKIA_ID_BCM2048) {
		fwname = FIRMWARE_BCM2048;
	} else if (btdev->man_id == NOKIA_ID_TI1271) {
		/*
		 * By chip version, as Nokia's own driver (hci_h4p/fw.c): the N9's (0xd1/0xf1) takes the
		 * LE firmware; with the plain one, it never answers the HCI reset after the download.
		 */
		switch (btdev->ver_id) {
		case 0xe1:
			fwname = FIRMWARE_TI1271_PRELE;
			break;
		case 0xd1:
		case 0xf1:
			fwname = FIRMWARE_TI1271_LE;
			break;
		default:
			fwname = FIRMWARE_TI1271;
		}
	} else {
		dev_err(dev, "Unsupported bluetooth device!");
		return -ENODEV;
	}

	err = request_firmware(&fw, fwname, dev);
	if (err < 0) {
		dev_err(dev, "%s: Failed to load Nokia firmware file (%d)",
			hu->hdev->name, err);
		return err;
	}

	fw_ptr = fw->data;
	fw_size = fw->size;

	while (fw_size >= 4) {
		u16 pkt_size = get_unaligned_le16(fw_ptr);
		u8 pkt_type = fw_ptr[2];
		const struct hci_command_hdr *cmd;
		u16 opcode;
		struct sk_buff *skb;

		switch (pkt_type) {
		case HCI_COMMAND_PKT:
			cmd = (struct hci_command_hdr *)(fw_ptr + 3);
			opcode = le16_to_cpu(cmd->opcode);

			skb = __hci_cmd_sync(hu->hdev, opcode, cmd->plen,
					     fw_ptr + 3 + HCI_COMMAND_HDR_SIZE,
					     HCI_INIT_TIMEOUT);
			if (IS_ERR(skb)) {
				err = PTR_ERR(skb);
				dev_err(dev, "%s: FW command %04x failed (%d)",
				       hu->hdev->name, opcode, err);
				goto done;
			}
			kfree_skb(skb);
			break;
		case HCI_NOKIA_RADIO_PKT:
		case HCI_NOKIA_NEG_PKT:
		case HCI_NOKIA_ALIVE_PKT:
			break;
		}

		fw_ptr += pkt_size + 2;
		fw_size -= pkt_size + 2;
	}

done:
	release_firmware(fw);
	return err;
}

static int nokia_setup(struct hci_uart *hu)
{
	struct nokia_bt_dev *btdev = hu->priv;
	struct device *dev = &btdev->serdev->dev;
	int err;

	btdev->initialized = false;

	nokia_flow_control(btdev->serdev, false);

	pm_runtime_get_sync(dev);

	if (btdev->tx_enabled) {
		gpiod_set_value_cansleep(btdev->wakeup_bt, 0);
		pm_runtime_put(&btdev->serdev->dev);
		btdev->tx_enabled = false;
	}

	dev_dbg(dev, "protocol setup");

	/* 0. reset connection */
	err = nokia_reset(hu);
	if (err < 0) {
		dev_err(dev, "Reset failed: %d", err);
		goto out;
	}

	/* 1. negotiate speed etc */
	err = nokia_send_negotiation(hu);
	if (err < 0) {
		dev_err(dev, "Negotiation failed: %d", err);
		goto out;
	}

	/* 2. verify correct setup using alive packet */
	err = nokia_send_alive_packet(hu);
	if (err < 0) {
		dev_err(dev, "Alive check failed: %d", err);
		goto out;
	}

	/* 3. send firmware */
	err = nokia_setup_fw(hu);
	if (err < 0) {
		dev_err(dev, "Could not setup FW: %d", err);
		goto out;
	}

	nokia_flow_control(btdev->serdev, false);
	serdev_device_set_baudrate(btdev->serdev, MAX_BAUD_RATE);
	nokia_flow_control(btdev->serdev, true);

	if (btdev->man_id == NOKIA_ID_TI1271) {
		__le32 speed = cpu_to_le32(TI1271_BAUD_RATE);
		struct sk_buff *skb;

		/* answered at the old rate, then the chip switches */
		skb = __hci_cmd_sync(hu->hdev, HCI_VS_UPDATE_UART_HCI_BAUDRATE, sizeof(speed),
				     &speed, HCI_INIT_TIMEOUT);
		if (IS_ERR(skb)) {
			dev_warn(dev, "Baud rate change failed: %ld", PTR_ERR(skb));
		} else {
			kfree_skb(skb);
			nokia_flow_control(btdev->serdev, false);
			serdev_device_set_baudrate(btdev->serdev, TI1271_BAUD_RATE);
			nokia_flow_control(btdev->serdev, true);
		}

		/* The TI1273 LE firmware answers LE Read Supported States with 4 bytes of the 8 */
		hci_set_quirk(hu->hdev, HCI_QUIRK_BROKEN_LE_STATES);
	}

	if (btdev->man_id == NOKIA_ID_BCM2048) {
		hu->hdev->set_bdaddr = btbcm_set_bdaddr;
		hci_set_quirk(hu->hdev, HCI_QUIRK_INVALID_BDADDR);
		dev_dbg(dev, "bcm2048 has invalid bluetooth address!");
	}

	dev_dbg(dev, "protocol setup done!");

	btdev->tx_enabled = false;
	btdev->initialized = true;
	gpiod_set_value_cansleep(btdev->wakeup_bt, 0);
	pm_runtime_put(dev);

	return 0;
out:
	pm_runtime_put(dev);

	return err;
}

static int nokia_open(struct hci_uart *hu)
{
	struct device *dev = &hu->serdev->dev;

	dev_dbg(dev, "protocol open");

	pm_runtime_enable(dev);

	return 0;
}

static int nokia_flush(struct hci_uart *hu)
{
	struct nokia_bt_dev *btdev = hu->priv;

	dev_dbg(&btdev->serdev->dev, "flush device");

	skb_queue_purge(&btdev->txq);

	return 0;
}

static int nokia_close(struct hci_uart *hu)
{
	struct nokia_bt_dev *btdev = hu->priv;
	struct device *dev = &btdev->serdev->dev;

	dev_dbg(dev, "close device");

	btdev->initialized = false;

	skb_queue_purge(&btdev->txq);

	kfree_skb(btdev->rx_skb);

	/* disable module */
	gpiod_set_value(btdev->reset, 1);
	gpiod_set_value(btdev->wakeup_bt, 0);

	pm_runtime_disable(&btdev->serdev->dev);

	return 0;
}

/* Enqueue frame for transmission (padding, crc, etc) */
static int nokia_enqueue(struct hci_uart *hu, struct sk_buff *skb)
{
	struct nokia_bt_dev *btdev = hu->priv;
	int err;

	/* Prepend skb with frame type */
	memcpy(skb_push(skb, 1), &bt_cb(skb)->pkt_type, 1);

	/* Packets must be word aligned */
	if (skb->len % 2) {
		err = skb_pad(skb, 1);
		if (err)
			return err;
		skb_put(skb, 1);
	}

	skb_queue_tail(&btdev->txq, skb);

	return 0;
}

static int nokia_recv_negotiation_packet(struct hci_dev *hdev,
					 struct sk_buff *skb)
{
	struct hci_uart *hu = hci_get_drvdata(hdev);
	struct nokia_bt_dev *btdev = hu->priv;
	struct device *dev = &btdev->serdev->dev;
	struct hci_nokia_neg_hdr *hdr;
	struct hci_nokia_neg_evt *evt;
	int ret = 0;

	hdr = (struct hci_nokia_neg_hdr *)skb->data;
	if (hdr->dlen != sizeof(*evt)) {
		btdev->init_error = -EIO;
		ret = -EIO;
		goto finish_neg;
	}

	evt = skb_pull(skb, sizeof(*hdr));

	if (evt->ack != NOKIA_NEG_ACK) {
		dev_err(dev, "Negotiation received: wrong reply");
		btdev->init_error = -EINVAL;
		ret = -EINVAL;
		goto finish_neg;
	}

	btdev->man_id = evt->man_id;
	btdev->ver_id = evt->ver_id;

	dev_dbg(dev, "Negotiation received: baud=%u:clk=%u:manu=%u:vers=%u",
		evt->baud, evt->sys_clk, evt->man_id, evt->ver_id);

finish_neg:
	complete(&btdev->init_completion);
	kfree_skb(skb);
	return ret;
}

static int nokia_recv_alive_packet(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct hci_uart *hu = hci_get_drvdata(hdev);
	struct nokia_bt_dev *btdev = hu->priv;
	struct device *dev = &btdev->serdev->dev;
	struct hci_nokia_alive_hdr *hdr;
	struct hci_nokia_alive_pkt *pkt;
	int ret = 0;

	hdr = (struct hci_nokia_alive_hdr *)skb->data;
	if (hdr->dlen != sizeof(*pkt)) {
		dev_err(dev, "Corrupted alive message");
		btdev->init_error = -EIO;
		ret = -EIO;
		goto finish_alive;
	}

	pkt = skb_pull(skb, sizeof(*hdr));

	if (pkt->mid != NOKIA_ALIVE_RESP) {
		dev_err(dev, "Alive received: invalid response: 0x%02x!",
			pkt->mid);
		btdev->init_error = -EINVAL;
		ret = -EINVAL;
		goto finish_alive;
	}

	dev_dbg(dev, "Alive received");

finish_alive:
	complete(&btdev->init_completion);
	kfree_skb(skb);
	return ret;
}

static int nokia_recv_radio(struct hci_dev *hdev, struct sk_buff *skb)
{
	/* Packets received on the dedicated radio channel are
	 * HCI events and so feed them back into the core.
	 */
	hci_skb_pkt_type(skb) = HCI_EVENT_PKT;
	return hci_recv_frame(hdev, skb);
}

/* Recv data */
static const struct h4_recv_pkt nokia_recv_pkts[] = {
	{ H4_RECV_ACL,		.recv = hci_recv_frame },
	{ H4_RECV_SCO,		.recv = hci_recv_frame },
	{ H4_RECV_EVENT,	.recv = hci_recv_frame },
	{ NOKIA_RECV_ALIVE,	.recv = nokia_recv_alive_packet },
	{ NOKIA_RECV_NEG,	.recv = nokia_recv_negotiation_packet },
	{ NOKIA_RECV_RADIO,	.recv = nokia_recv_radio },
};

/* ponytail: temporary, every received chunk in the kernel log (rx_dump=1), to find the framing */
static bool rx_dump;
module_param(rx_dump, bool, 0644);

static bool nokia_pkt_type_known(u8 type)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(nokia_recv_pkts); i++)
		if (nokia_recv_pkts[i].type == type)
			return true;
	return false;
}

static int nokia_recv(struct hci_uart *hu, const void *data, int count)
{
	struct nokia_bt_dev *btdev = hu->priv;
	struct device *dev = &btdev->serdev->dev;
	const u8 *p = data;
	int err;

	if (!test_bit(HCI_UART_REGISTERED, &hu->flags))
		return -EUNATCH;

	if (rx_dump)
		print_hex_dump(KERN_INFO, "nokia-bt rx: ", DUMP_PREFIX_NONE, 32, 1, data, count, false);

	/*
	 * A frame begun on a corrupt byte (e.g. a lost packet type, the rest read as a header) waits
	 * for bytes that never come and swallows the next frames into it: command replies (tx
	 * timeouts), the next power-on's set-up (EIO) until a reboot. The chip sends a frame in one
	 * burst, so one still incomplete 500 ms after its last bytes is dropped.
	 */
	if (btdev->rx_skb && time_after(jiffies, btdev->rx_last + msecs_to_jiffies(500))) {
		kfree_skb(btdev->rx_skb);
		btdev->rx_skb = NULL;
		hu->hdev->stat.err_rx++;
	}
	btdev->rx_last = jiffies;

	/*
	 * As hci_h4p, a byte between packets that is no packet type is dropped alone (h4_recv_buf()
	 * would drop the whole chunk with it)
	 */
	while (count && !btdev->rx_skb && !hu->padding && !nokia_pkt_type_known(*p)) {
		hu->hdev->stat.err_rx++;
		p++;
		count--;
	}
	if (!count)
		return 0;

	btdev->rx_skb = h4_recv_buf(hu, btdev->rx_skb, p, count,
				    nokia_recv_pkts, ARRAY_SIZE(nokia_recv_pkts));
	if (IS_ERR(btdev->rx_skb)) {
		err = PTR_ERR(btdev->rx_skb);
		dev_err(dev, "Frame reassembly failed (%d)", err);
		/* ponytail: temporary, to see what derails the stream (remove once known) */
		print_hex_dump(KERN_ERR, "nokia-bt rx: ", DUMP_PREFIX_OFFSET, 16, 1, p,
			       min(count, 48), false);
		btdev->rx_skb = NULL;
		return err;
	}

	return count;
}

/*
 * The end of a transmission (BT wakeup down, the serdev device released) once the UART has sent
 * everything, 100 ms after the queue ran dry, as stock's hci_h4p released its clocks (lazy_release);
 * a frame queued before then carries the burst on. Waiting for the UART in nokia_dequeue() held up
 * the next frame for the serial core's tx_empty polling, a tick or two (~22 ms) after every burst:
 * half the throughput of a ping-pong exchange.
 */
#define NOKIA_TX_RELEASE_MS	100

static void nokia_tx_release(struct work_struct *work)
{
	struct nokia_bt_dev *btdev = container_of(work, struct nokia_bt_dev, tx_release.work);

	mutex_lock(&btdev->tx_lock);
	if (btdev->tx_enabled && btdev->tx_idle) {
		serdev_device_wait_until_sent(btdev->serdev, 0);
		gpiod_set_value_cansleep(btdev->wakeup_bt, 0);
		pm_runtime_put(&btdev->serdev->dev);
		btdev->tx_enabled = false;
	}
	mutex_unlock(&btdev->tx_lock);
}

static struct sk_buff *nokia_dequeue(struct hci_uart *hu)
{
	struct nokia_bt_dev *btdev = hu->priv;
	struct device *dev = &btdev->serdev->dev;
	struct sk_buff *result = skb_dequeue(&btdev->txq);

	if (!btdev->initialized)
		return result;

	mutex_lock(&btdev->tx_lock);
	btdev->tx_idle = !result;
	if (result && !btdev->tx_enabled) {
		pm_runtime_get_sync(dev);
		gpiod_set_value_cansleep(btdev->wakeup_bt, 1);
		btdev->tx_enabled = true;
	} else if (!result && btdev->tx_enabled) {
		mod_delayed_work(system_wq, &btdev->tx_release,
				 msecs_to_jiffies(NOKIA_TX_RELEASE_MS));
	}
	mutex_unlock(&btdev->tx_lock);

	return result;
}

static const struct hci_uart_proto nokia_proto = {
	.id		= HCI_UART_NOKIA,
	.name		= "Nokia",
	.open		= nokia_open,
	.close		= nokia_close,
	.recv		= nokia_recv,
	.enqueue	= nokia_enqueue,
	.dequeue	= nokia_dequeue,
	.flush		= nokia_flush,
	.setup		= nokia_setup,
	.manufacturer	= 1,
};

/*
 * While the chip sleeps the UART is released too (runtime suspend, below): take the port's runtime
 * PM reference back, for good, before the port is shut down or closed (its close drops it).
 */
static void nokia_hold_uart(struct nokia_bt_dev *btdev)
{
	struct device *dev = &btdev->serdev->dev;

	pm_runtime_get_sync(dev);
	btdev->initialized = false;
	pm_runtime_put(dev);
}

/*
 * No transmission left under way, its runtime PM reference put or left to the next set-up (one
 * still waiting for the UART would put it again after the set-up has: an underflow), and nothing
 * left to touch the port once it closes
 */
static void nokia_stop_tx(struct nokia_bt_dev *btdev)
{
	flush_work(&btdev->hu.write_work);
	cancel_delayed_work_sync(&btdev->tx_release);
}

/* Every close (power-off, a failed power-on, unbind: no shutdown then) */
static int nokia_hci_close(struct hci_dev *hdev)
{
	struct hci_uart *hu = hci_get_drvdata(hdev);
	struct nokia_bt_dev *btdev = hu->priv;

	nokia_hold_uart(btdev);
	nokia_stop_tx(btdev);
	return btdev->hci_close(hdev);
}

/*
 * Bluetooth off: the chip held in reset, as stock's hci_h4p (the set-up resets it at power-on).
 * Flow control goes off first: a chip in reset leaves CTS high (pulled up), and bytes still in
 * the UART's FIFO, or written after this, would wait for it until the port's close gives up
 * (30 s, bluetoothd blocked). Stock reset the UART at close and lost them the same way.
 */
static int nokia_shutdown(struct hci_dev *hdev)
{
	struct hci_uart *hu = hci_get_drvdata(hdev);
	struct nokia_bt_dev *btdev = hu->priv;

	nokia_hold_uart(btdev);
	nokia_flow_control(btdev->serdev, false);
	nokia_stop_tx(btdev);	/* flow control off first: nothing can hold the FIFO */
	gpiod_set_value_cansleep(btdev->wakeup_bt, 0);
	gpiod_set_value_cansleep(btdev->reset, 1);
	return 0;
}

static int nokia_bluetooth_serdev_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	struct nokia_bt_dev *btdev;
	struct clk *sysclk;
	int err = 0;

	btdev = devm_kzalloc(dev, sizeof(*btdev), GFP_KERNEL);
	if (!btdev)
		return -ENOMEM;

	btdev->hu.serdev = btdev->serdev = serdev;
	serdev_device_set_drvdata(serdev, btdev);

	btdev->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(btdev->reset)) {
		err = PTR_ERR(btdev->reset);
		dev_err(dev, "could not get reset gpio: %d", err);
		return err;
	}

	btdev->wakeup_host = devm_gpiod_get(dev, "host-wakeup", GPIOD_IN);
	if (IS_ERR(btdev->wakeup_host)) {
		err = PTR_ERR(btdev->wakeup_host);
		dev_err(dev, "could not get host wakeup gpio: %d", err);
		return err;
	}

	btdev->wake_irq = gpiod_to_irq(btdev->wakeup_host);

	err = devm_request_threaded_irq(dev, btdev->wake_irq, NULL,
		wakeup_handler,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		"wakeup", btdev);
	if (err) {
		dev_err(dev, "could request wakeup irq: %d", err);
		return err;
	}

	btdev->wakeup_bt = devm_gpiod_get(dev, "bluetooth-wakeup",
					   GPIOD_OUT_LOW);
	if (IS_ERR(btdev->wakeup_bt)) {
		err = PTR_ERR(btdev->wakeup_bt);
		dev_err(dev, "could not get BT wakeup gpio: %d", err);
		return err;
	}

	sysclk = devm_clk_get(dev, "sysclk");
	if (IS_ERR(sysclk)) {
		err = PTR_ERR(sysclk);
		dev_err(dev, "could not get sysclk: %d", err);
		return err;
	}

	err = clk_prepare_enable(sysclk);
	if (err) {
		dev_err(dev, "could not enable sysclk: %d", err);
		return err;
	}
	btdev->sysclk_speed = clk_get_rate(sysclk);
	clk_disable_unprepare(sysclk);

	skb_queue_head_init(&btdev->txq);
	mutex_init(&btdev->tx_lock);
	INIT_DELAYED_WORK(&btdev->tx_release, nokia_tx_release);

	btdev->hu.priv = btdev;
	btdev->hu.alignment = 2; /* Nokia H4+ is word aligned */

	err = hci_uart_register_device(&btdev->hu, &nokia_proto);
	if (err) {
		dev_err(dev, "could not register bluetooth uart: %d", err);
		return err;
	}

	/*
	 * The UART port closed while Bluetooth is off (open, serdev keeps UART2 and with it the CORE
	 * domain from idling), the chip off; the set-up (firmware) runs at each power-on, as stock
	 */
	hci_set_quirk(btdev->hu.hdev, HCI_QUIRK_NON_PERSISTENT_SETUP);
	btdev->hu.hdev->shutdown = nokia_shutdown;
	btdev->hci_close = btdev->hu.hdev->close;
	btdev->hu.hdev->close = nokia_hci_close;

	return 0;
}

static void nokia_bluetooth_serdev_remove(struct serdev_device *serdev)
{
	struct nokia_bt_dev *btdev = serdev_device_get_drvdata(serdev);

	hci_uart_unregister_device(&btdev->hu);
}

/*
 * While the chip sleeps (no host wakeup, nothing to send): RTS down and the UART released (the
 * port's runtime PM reference serdev_device_open() took), so UART2 and the CORE domain can idle,
 * as stock's hci_h4p let them. The chip waits for RTS before it sends, so a wakeup resumes the UART
 * first (restoring it clears its FIFOs) and then raises RTS. RTS alone: a termios change (flow
 * control) would clear the receive FIFO again (8250_omap's MDR1 erratum path), the chip's first
 * bytes with it. Only between a set-up and the shutdown (the port is open then; it's closed while
 * Bluetooth is off).
 * ponytail: rts_delay_us is a test knob, a late RTS as from a UART woken from CORE idle
 */
static unsigned int rts_delay_us;
module_param(rts_delay_us, uint, 0644);

static int nokia_bluetooth_runtime_suspend(struct device *dev)
{
	struct nokia_bt_dev *btdev = serdev_device_get_drvdata(to_serdev_device(dev));

	if (btdev->initialized && !btdev->uart_released) {
		serdev_device_set_rts(btdev->serdev, false);
		pm_runtime_put(&btdev->serdev->ctrl->dev);
		btdev->uart_released = true;
	}
	return 0;
}

static int nokia_bluetooth_runtime_resume(struct device *dev)
{
	struct nokia_bt_dev *btdev = serdev_device_get_drvdata(to_serdev_device(dev));

	if (btdev->uart_released) {
		pm_runtime_get_sync(&btdev->serdev->ctrl->dev);
		btdev->uart_released = false;
	}
	if (btdev->initialized) {
		if (rts_delay_us)
			usleep_range(rts_delay_us, rts_delay_us + 100);
		serdev_device_set_rts(btdev->serdev, true);
	}
	return 0;
}

static const struct dev_pm_ops nokia_bluetooth_pm_ops = {
	SET_RUNTIME_PM_OPS(nokia_bluetooth_runtime_suspend,
			   nokia_bluetooth_runtime_resume,
			   NULL)
};

#ifdef CONFIG_OF
static const struct of_device_id nokia_bluetooth_of_match[] = {
	{ .compatible = "nokia,h4p-bluetooth", },
	{},
};
MODULE_DEVICE_TABLE(of, nokia_bluetooth_of_match);
#endif

static struct serdev_device_driver nokia_bluetooth_serdev_driver = {
	.probe = nokia_bluetooth_serdev_probe,
	.remove = nokia_bluetooth_serdev_remove,
	.driver = {
		.name = "nokia-bluetooth",
		.pm = &nokia_bluetooth_pm_ops,
		.of_match_table = of_match_ptr(nokia_bluetooth_of_match),
	},
};

module_serdev_device_driver(nokia_bluetooth_serdev_driver);

MODULE_AUTHOR("Sebastian Reichel <sre@kernel.org>");
MODULE_DESCRIPTION("Bluetooth HCI UART Nokia H4+ driver ver " VERSION);
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");
