// SPDX-License-Identifier: GPL-2.0-only
/*
 * dfl61.c -- the Nokia N9's sound cards as its stock (Harmattan) kernel had them, on DT
 *
 * Ported from Nokia's sound/soc/omap/dfl61-twl4030.c and dfl61-tlv320dac33.c (Peter Ujfalusi),
 * audio config 4 (the N9, RM-696):
 * - "dfl61-twl4030": the TWL5031's codec on McBSP3: IHF speaker (its amp on TWL GPIO 7), earpiece,
 *   HAC, vibra, digital mic, headset mic and the FM line-in;
 * - "dfl61-dac33": the TLV320DAC33 on McBSP2 into the TPA6140A2 headphone amplifier, which also
 *   takes the TWL's PREDRIVEL line-out ("interconnect": headset audio during calls).
 * Harmattan's PulseAudio opens the cards by these names and routes by the codecs' control names.
 * The AV jack's buttons (Nokia's twl5031-aci and eci drivers) are here too, beside its plug detection.
 */

#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/consumer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/mfd/twl.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "../codecs/tlv320dac33.h"
#include "../codecs/wl1273.h"

static struct gpio_desc *dfl61_ihf_gpio;
static struct gpio_desc *dfl61_tvout_gpio;	/* TVOUT_EN: the AV jack in video mode when high */
static struct snd_soc_card dfl61_twl_card;
static struct snd_soc_card dfl61_dac33_card;
static struct snd_soc_jack dfl61_jack;
static struct iio_channel *dfl61_hsmic;		/* MADC ADCIN10: the headset mic line */

/*
 * The AV jack: a plug (AvPlugDet, DT jack-gpios) is classified as Nokia's twl5031-aci did, through
 * the TWL5031's accessory block (comparators on the mic line, which the MADC measures), and
 * reported as it did: headphones (and carkits) SND_JACK_HEADPHONE, headsets SND_JACK_HEADSET,
 * TV-out cables SND_JACK_AVOUT, open cables SND_JACK_LINEOUT, anything else, wrong-pinout plugs
 * (CTIA vs OMTP) included, SND_JACK_MECHANICAL: Harmattan's policy calls that incompatible and keeps
 * the speaker. Nokia's ECI accessories (remotes, carkits) come out as headsets; their buttons and a
 * basic headset's are below. As stock, TVOUT_EN goes low (audio) at each plug, before the
 * classification, and high (video) for a TV-out cable, before it's reported; an unplug leaves it.
 */
static int dfl61_jack_detect(void *data);

static struct snd_soc_jack_gpio dfl61_jack_gpio = {
	.name = "jack",
	.report = SND_JACK_HEADSET | SND_JACK_AVOUT | SND_JACK_MECHANICAL,
	.debounce_time = 200,
	.jack_status_check = dfl61_jack_detect,
};

/* TWL5031 accessory block (Nokia's include/linux/mfd/aci.h; ACIIMR/ACIIDR in twl.h) */
#define TWL5031_ACIID			0x00
#define TWL5031_ACICOMR_LSB		0x01
#define TWL5031_ACITXDAR		0x03
#define TWL5031_ACIRXDAR		0x04
#define TWL5031_ECI_DBI_CTRL		0x0b
#define TWL5031_ACI_AV_CTRL		0x0c
#define TWL5031_ACI_AUDIO_CTRL		0x0d
#define AV_COMP1_EN			0x01
#define AV_COMP2_EN			0x02
#define ADC_SW_EN			0x04
#define STATUS_A1_COMP			0x08
#define STATUS_A2_COMP			0x10
#define HOOK_DET_EN			0x20
#define HOOK_DET_EN_SLEEP_MODE		0x40
#define HSM_GROUNDED_EN			0x80
#define HOOK_DEB_BYPASS			0x10	/* ACI_AUDIO_CTRL */
#define ACI_DBI_MODE			0x00
#define ACI_ACCESSORY_MODE		0x01
#define ACI_ENABLE			0x00
#define ACI_DISABLE			0x02

/* On the mic line, mV (twl5031-aci.c) */
#define ACI_HEADPHONE_THRESHOLD		40
#define ACI_AVOUT_THRESHOLD		500
#define ACI_AHJ_NHJ_THRESHOLD		100
#define ACI_BASIC_HEADSET_THRESHOLD	200
#define ACI_BASIC_CARKIT_THRESHOLD	1075
#define ACI_SETTLE_MS			20

static u8 aci_read(u8 reg)
{
	u8 val = 0;

	twl_i2c_read_u8(TWL5031_MODULE_ACCESSORY, &val, reg);
	return val;
}

static void aci_write(u8 reg, u8 val)
{
	twl_i2c_write_u8(TWL5031_MODULE_ACCESSORY, val, reg);
}

/* ADCIN10, 1.5 mV a step (Nokia's MADC_ADCIN10_PRESCALE) */
static int aci_hsmic_mv(void)
{
	int raw;

	return iio_read_channel_raw(dfl61_hsmic, &raw) < 0 ? -1 : raw * 15 / 10;
}

static void aci_disable_pullups(void)
{
	aci_write(TWL5031_ECI_DBI_CTRL, ACI_DISABLE | ACI_DBI_MODE);
	aci_write(TWL5031_ACI_AV_CTRL, aci_read(TWL5031_ACI_AV_CTRL) &
		  ~(AV_COMP2_EN | HOOK_DET_EN_SLEEP_MODE));
}

/*
 * The jack's buttons, as Nokia's twl5031-aci and eci drivers had them, on one input device, "ECI
 * Accessory" (udev's /dev/input/eci, which qmkeyd reads; X and ohmd find it by name):
 * - a basic headset's (or carkit's) button: KEY_PHONE, comparator 1 seeing the mic line below 600 mV;
 * - an ECI accessory's: a chip on the mic line, which the ACI block talks to over a one-wire bus.
 *   Its memory describes its I/O pins and their functions (ohmd reads it from stock's
 *   /sys/devices/platform/ECI_accessory.0/memory); the pins are read as port registers whenever the
 *   accessory raises ACCINT.
 * A bus transaction is fed byte by byte from the ACI interrupt (the TWL's PIH line 6), its steps as
 * Nokia's interrupt thread took them; it runs from process context, not from the interrupt as
 * Nokia's button reads did: the TWL's interrupts are nested in one thread here.
 */
#define ACI_ACCINT			BIT(0)
#define ACI_DREC			BIT(1)
#define ACI_DSENT			BIT(2)
#define ACI_SPDSET			BIT(3)
#define ACI_COMMERR			BIT(4)
#define ACI_FRAERR			BIT(5)
#define ACI_RESERR			BIT(6)
#define ACI_COLL			BIT(7)
#define ACI_NOPINT			BIT(8)
#define ACI_ERRORS			(ACI_COMMERR | ACI_FRAERR | ACI_RESERR | ACI_COLL)
#define ACI_INTERNAL			(ACI_DREC | ACI_DSENT | ACI_SPDSET | ACI_ERRORS | ACI_NOPINT)

#define ACICMD_NOP			0
#define ACICMD_RST_LEARN		1
#define ACICMD_SEND			2
#define ACICMD_RECEIVE_LEARN		4

/* ECI commands: register and memory accesses (Nokia's aci.h, eci.c) */
#define ECI_READ_MEM(addr)		((addr) << 1)
#define ECI_READ_REG(reg)		((reg) << 2 | 1)
#define ECI_WRITE_REG(reg)		((reg) << 2 | 3)
#define ECICMD_MIC_CTRL			0x03
#define ECICMD_MASTER_INT_REG		0x04
#define ECICMD_PORT_DATA_0		0x08
#define ECICMD_LATCHED_PORT_DATA_0	0x0c
#define ECI_MIC_AUTO			0x00
#define ECI_MIC_OFF			0x5a
#define ECI_INT_ENABLE			BIT(0)
#define ECI_INT_DELAY_ENABLE		BIT(1)
#define ECI_INT_LEN_120US		(7 << 5)
#define ECI_EKEY_BLOCK_ID		0xb3
#define ECI_FEATURE_BLOCK_ID		0x02
#define ECI_MAX_MEM_SIZE		0x7c
#define ECI_MAX_FEATURES		31

/* An ECI pin's function to a key, or a switch above KEY_MAX (Nokia's eci_codes) */
static const int eci_codes[] = {
	KEY_UNKNOWN,			/* 0x00 no feature */
	KEY_UNKNOWN,			/* 0x01 ignition sense */
	KEY_UNKNOWN,			/* 0x02 carkit handset hook */
	KEY_BATTERY,			/* 0x03 power supply/car battery */
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_AUDIO,			/* 0x06 external audio in */
	KEY_PHONE,			/* 0x07 send, end, voice recognition */
	KEY_MAX + SW_HEADPHONE_INSERT,	/* 0x08 headphone plug */
	KEY_RESERVED,
	KEY_UNKNOWN,			/* 0x0a device power request */
	KEY_VOLUMEUP,			/* 0x0b */
	KEY_VOLUMEDOWN,			/* 0x0c */
	KEY_PLAYPAUSE,			/* 0x0d */
	KEY_STOP,			/* 0x0e */
	KEY_FORWARD,			/* 0x0f next, fast forward, autosearch up */
	KEY_REWIND,			/* 0x10 previous, rewind, autosearch down */
	KEY_UNKNOWN,			/* 0x11 push to talk */
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_UNKNOWN,			/* 0x14 sync */
	KEY_RADIO,			/* 0x15 music/radio/off selector */
	KEY_UNKNOWN,			/* 0x16 redial */
	KEY_UNKNOWN,			/* 0x17 left soft key */
	KEY_UNKNOWN,			/* 0x18 right soft key */
	KEY_SEND,			/* 0x19 */
	KEY_END,			/* 0x1a */
	KEY_UNKNOWN,			/* 0x1b middle soft key */
	KEY_UP,				/* 0x1c */
	KEY_DOWN,			/* 0x1d */
	KEY_RIGHT,			/* 0x1e */
	KEY_LEFT,			/* 0x1f */
	KEY_UNKNOWN,			/* 0x20 Symbian application key */
	KEY_UNKNOWN,			/* 0x21 terminal application control */
	KEY_RESERVED,
	KEY_UNKNOWN,			/* 0x23 USB class switching */
	KEY_MUTE,			/* 0x24 */
};

enum { ACI_BUTTONS_NONE, ACI_BUTTONS_BASIC, ACI_BUTTONS_ECI };
enum { ACI_IDLE, ACI_RESET, ACI_WRITE, ACI_READ };

static struct {
	int irq;			/* 0: no buttons */
	struct input_dev *input;
	struct regulator *v28;		/* VAUX1: the ACI's pull-ups, held while a plug is in */
	bool powered;
	int buttons;			/* ACI_BUTTONS_* of the plug in */
	struct delayed_work button_work;
	bool down, reported;		/* a basic button: last sampled, last reported */
	struct work_struct mic_work;
	bool mic_on;			/* the headset mic in use (DAPM) */

	/* one transaction at a time; irq_lock: its state and the IMR, between it and the interrupt */
	struct mutex lock, irq_lock;
	struct completion done;
	struct pm_qos_request qos;
	u16 imr;
	int op, count, err, errors;
	int drec, dsent, spdset, nopint;
	u8 cmd, *buf;

	/* the ECI accessory: its memory and the "enhancement features" block in it */
	bool mem_ok;
	u16 mem_size;
	u8 memory[ECI_MAX_MEM_SIZE + 4];	/* + Nokia's 32-bit read of the active states */
	const u8 *io_fun;		/* pin functions, [1..features] */
	u32 active;			/* pin active states, pin p at bit p - 1 */
	int features, port_regs;
} aci;

static void aci_imr(u16 bits, bool mask)
{
	mutex_lock(&aci.irq_lock);
	aci.imr = mask ? aci.imr | bits : aci.imr & ~bits;
	aci_write(TWL5031_ACIIMR_LSB, aci.imr);
	aci_write(TWL5031_ACIIMR_MSB, aci.imr >> 8);
	mutex_unlock(&aci.irq_lock);
}

static void aci_done(int err)
{
	aci.err = err;
	aci.op = ACI_IDLE;
	complete(&aci.done);
}

/* A transaction's next step: its interrupts counted so far decide, in Nokia's order */
static void aci_step(u16 val)
{
	aci.drec += !!(val & ACI_DREC);
	aci.dsent += !!(val & ACI_DSENT);
	aci.spdset += !!(val & ACI_SPDSET);
	aci.nopint += !!(val & ACI_NOPINT);
	aci.errors |= val & ACI_ERRORS;

	switch (aci.op) {
	case ACI_RESET:		/* the accessory's bus speed learnt */
		if (aci.spdset)
			aci_done(0);
		else if (aci.errors & ACI_RESERR)
			aci_done(ACI_RESERR);
		break;
	case ACI_WRITE:		/* command, parameter, NOP */
		if (aci.errors)
			aci_done(aci.errors);
		else if (aci.nopint == 1)
			aci_done(0);
		else if (aci.dsent == 3)
			aci_write(TWL5031_ACICOMR_LSB, ACICMD_NOP);
		else if (aci.dsent == 2)
			aci_write(TWL5031_ACITXDAR, aci.buf[0]);
		else if (aci.dsent == 1)
			aci_write(TWL5031_ACITXDAR, aci.cmd);
		break;
	case ACI_READ:		/* command, NOP, receive: speed, then the bytes */
		if (aci.errors) {
			aci_done(aci.errors);
		} else if (aci.nopint == 2) {
			aci_done(0);
		} else if (aci.drec) {
			u8 b = aci_read(TWL5031_ACIRXDAR);

			if (aci.drec <= aci.count)
				aci.buf[aci.drec - 1] = b;
			if (aci.drec == aci.count)
				aci_write(TWL5031_ACICOMR_LSB, ACICMD_NOP);
		} else if (aci.spdset == 1) {
			/* wait for the data */
		} else if (aci.nopint == 1) {
			aci_write(TWL5031_ACICOMR_LSB, ACICMD_RECEIVE_LEARN);
		} else if (aci.dsent == 2) {
			aci_write(TWL5031_ACICOMR_LSB, ACICMD_NOP);
		} else if (aci.dsent == 1) {
			aci_write(TWL5031_ACITXDAR, aci.cmd);
		}
		break;
	}
}

/* Nokia's twl5031_get_av_button(): the button down pulls the mic line below comparator 1 */
static bool aci_basic_down(void)
{
	aci_write(TWL5031_ACI_AV_CTRL, AV_COMP1_EN);
	usleep_range(100, 200);
	return aci_read(TWL5031_ACI_AV_CTRL) & STATUS_A1_COMP;
}

static irqreturn_t aci_irq(int irq, void *data)
{
	int buttons = READ_ONCE(aci.buttons);
	u16 val;

	val = aci_read(TWL5031_ACIIDR_MSB) << 8 | aci_read(TWL5031_ACIIDR_LSB);
	aci_write(TWL5031_ACIIDR_MSB, val >> 8);
	aci_write(TWL5031_ACIIDR_LSB, val);

	/* The accessory's interrupt: a button; off until it's read (a basic one sampled now) */
	if ((val & ACI_ACCINT) && buttons) {
		aci_imr(ACI_ACCINT, true);
		if (buttons == ACI_BUTTONS_BASIC)
			aci.down = aci_basic_down();
		schedule_delayed_work(&aci.button_work,
				      buttons == ACI_BUTTONS_BASIC ? msecs_to_jiffies(200) : 0);
	}

	/* Only on a transaction's own events: a step on none repeats the last one's write */
	if (val & ACI_INTERNAL) {
		mutex_lock(&aci.irq_lock);
		aci_step(val);
		mutex_unlock(&aci.irq_lock);
	}
	return IRQ_HANDLED;
}

/* Out of the "coma" Nokia saw after a lone COMMERR: the block disabled and enabled again */
static void aci_out_of_coma(void)
{
	msleep(20);
	aci_write(TWL5031_ECI_DBI_CTRL, ACI_DISABLE | ACI_ACCESSORY_MODE);
	aci_write(TWL5031_ECI_DBI_CTRL, ACI_ENABLE | ACI_ACCESSORY_MODE);
	msleep(20);
}

/*
 * One bus transaction (Nokia's twl5031_aci_cmd()): 0, -EAGAIN when the interrupts stopped, or the
 * ACI_ERRORS seen. The CPU must answer each step in time: no deep idle meanwhile.
 */
static int aci_cmd(int op, u8 cmd, u8 *buf, int count)
{
	int err;

	mutex_lock(&aci.lock);
	mutex_lock(&aci.irq_lock);
	aci.drec = aci.dsent = aci.spdset = aci.nopint = aci.errors = 0;
	aci.cmd = cmd;
	aci.buf = buf;
	aci.count = count;
	aci.op = op;
	reinit_completion(&aci.done);
	mutex_unlock(&aci.irq_lock);

	aci_imr(ACI_INTERNAL, false);
	cpu_latency_qos_add_request(&aci.qos, 0);
	aci_write(TWL5031_ACICOMR_LSB, op == ACI_RESET ? ACICMD_RST_LEARN : ACICMD_SEND);
	err = wait_for_completion_timeout(&aci.done, msecs_to_jiffies(100)) ? 0 : -EAGAIN;
	aci_imr(ACI_INTERNAL, true);
	cpu_latency_qos_remove_request(&aci.qos);

	mutex_lock(&aci.irq_lock);
	if (!err)
		err = aci.err;
	aci.op = ACI_IDLE;
	mutex_unlock(&aci.irq_lock);

	if (err != -EAGAIN) {
		usleep_range(1000, 1100);	/* Nokia: "necessary. Reason unknown" */
		if (err) {
			msleep(20);
			if (err == ACI_COMMERR)
				aci_out_of_coma();
		}
	}
	mutex_unlock(&aci.lock);
	return err;
}

static int eci_write(u8 reg, u8 val)
{
	u8 av;
	int ret;

	aci_imr(ACI_ACCINT, true);
	ret = aci_cmd(ACI_WRITE, ECI_WRITE_REG(reg), &val, 1);
	if (reg != ECICMD_MIC_CTRL)
		return ret;

	/* The mic off: the line kept up for hook detection by the ACI's sleep-mode LDO instead */
	av = aci_read(TWL5031_ACI_AV_CTRL);
	if (val == ECI_MIC_OFF)
		av |= HOOK_DET_EN_SLEEP_MODE;
	else
		av &= ~HOOK_DET_EN_SLEEP_MODE;
	aci_write(TWL5031_ACI_AV_CTRL, av);

	if (READ_ONCE(aci.buttons) == ACI_BUTTONS_ECI) {
		/* buttons back on once the mic line settles */
		aci_write(TWL5031_ACIIDR_LSB, ACI_ACCINT);
		msleep(25);
		aci_write(TWL5031_ACIIDR_LSB, ACI_ACCINT);
		aci_imr(ACI_ACCINT, false);
	}
	return ret;
}

/* The accessory's mic on while the headset mic's in use, as Nokia's eci_hsmic_event() */
static void eci_set_mic(void)
{
	u8 val = READ_ONCE(aci.mic_on) ? ECI_MIC_AUTO : ECI_MIC_OFF;
	int ret, err;

	ret = read_poll_timeout(eci_write, err, !err, 0, 200 * USEC_PER_MSEC, false,
				ECICMD_MIC_CTRL, val);
	if (ret)
		dev_err(aci.input->dev.parent, "ECI accessory: mic not switched (%d)\n", err);
}

static void aci_mic_work(struct work_struct *work)
{
	if (aci.mem_ok)
		eci_set_mic();
}

/* The memory's first block, the "ekey", gives the memory's size */
static int eci_get_ekey(u16 *size)
{
	u8 buf[4];
	int ret;

	ret = aci_cmd(ACI_READ, ECI_READ_MEM(0), buf, 4);
	if (ret)
		return ret;
	if (buf[0] != ECI_EKEY_BLOCK_ID)
		return -ENODEV;
	*size = get_unaligned_be16(buf + 2);
	return 0;
}

/* Four bytes a read, from where the last try stopped */
static int eci_get_memory(int *next)
{
	int ret = 0;

	for (; *next < aci.mem_size; *next += 4) {
		ret = aci_cmd(ACI_READ, ECI_READ_MEM(*next), aci.memory + *next, 4);
		if (ret)
			break;
	}
	return ret;
}

/*
 * The "enhancement features" block, after the ekey, as Nokia's eci_parse_enhancement_features() read
 * it (the accessory's data: bounded to its memory): id, length, connector, the number of pins n,
 * DIV_ROUND_UP(n, 8) I/O support bytes, n pin functions, then the pins' active states
 */
static int eci_parse(void)
{
	u8 *end = aci.memory + aci.mem_size, *b = aci.memory;
	const u8 *fun, *active;
	int n, k;

	if (aci.mem_size < 4 || b[0] != ECI_EKEY_BLOCK_ID)
		return -ENODEV;
	do {
		if (!b[1])
			return -EINVAL;
		b += b[1];
		if (b + 4 > end || b + b[1] > end)
			return -EINVAL;
	} while (b[0] != ECI_FEATURE_BLOCK_ID);

	n = b[3];
	if (n > ECI_MAX_FEATURES)
		return -EINVAL;
	k = DIV_ROUND_UP(n, 8);
	fun = b + 4 + k - 1;		/* pins are 1..n: fun[0] unused */
	active = fun + n + 1;
	if (active + k > b + b[1])
		return -EINVAL;

	aci.features = n;
	aci.io_fun = fun;
	/* Nokia's read of four bytes whatever k is: past the block, within the buffer */
	aci.active = get_unaligned_be32(active);
	/* Port data comes as two bytes at least, even for fewer than 8 pins (Nokia's "kludge") */
	aci.port_regs = max(2, DIV_ROUND_UP(n + 1, 8));
	return 0;
}

/* The accessory's interrupt: 120 us pulses, delayed, enabled once set */
static int eci_init_int(void)
{
	u8 buf[4];
	u8 mireg;
	int ret;

	ret = aci_cmd(ACI_READ, ECI_READ_REG(ECICMD_MASTER_INT_REG), buf, 1);
	if (ret)
		return ret;
	mireg = (buf[0] & ~ECI_INT_ENABLE) | ECI_INT_LEN_120US | ECI_INT_DELAY_ENABLE;
	ret = eci_write(ECICMD_MASTER_INT_REG, mireg);
	if (ret)
		return ret;
	usleep_range(20000, 20100);
	ret = eci_write(ECICMD_MASTER_INT_REG, mireg | ECI_INT_ENABLE);
	usleep_range(20000, 20100);
	return ret;
}

/* Nokia's eci_init_accessory(), each step retried as it did */
static int eci_init(void)
{
	int ret, err, next = 0;
	u16 size;

	usleep_range(20000, 20100);
	ret = aci_cmd(ACI_RESET, 0, NULL, 0);
	if (ret)
		return ret;
	usleep_range(20000, 20100);
	ret = eci_write(ECICMD_MIC_CTRL, ECI_MIC_OFF);
	if (ret)
		return ret;

	if (read_poll_timeout(eci_get_ekey, err, !err, 0, 2 * USEC_PER_SEC, false, &size))
		return err;
	if (size > ECI_MAX_MEM_SIZE)
		return -EINVAL;
	aci.mem_size = size;
	memset(aci.memory, 0, sizeof(aci.memory));
	if (read_poll_timeout(eci_get_memory, err, !err, 0, 2 * USEC_PER_SEC, false, &next))
		return err;
	ret = eci_parse();
	if (ret)
		return ret;

	usleep_range(20000, 20100);
	if (read_poll_timeout(eci_init_int, err, !err, 0, 200 * USEC_PER_MSEC, false))
		return err;

	aci.mem_ok = true;
	usleep_range(20000, 20100);
	eci_set_mic();
	return 0;
}

static void aci_release_keys(void)
{
	int code;

	for_each_set_bit(code, aci.input->key, KEY_CNT)
		input_report_key(aci.input, code, 0);
	input_report_switch(aci.input, SW_HEADPHONE_INSERT, 0);
	input_sync(aci.input);
}

/* Pin p is active when bit p of the port data equals bit p - 1 of its active state (Nokia's) */
static bool eci_pin_active(u32 port, int pin)
{
	return !!(port & BIT(pin)) == !!(aci.active & BIT(pin - 1));
}

static void eci_report(u32 port)
{
	int pin, code, active = 0;

	for (pin = 1; pin <= aci.features; pin++)
		active += eci_pin_active(port, pin);
	if (active == aci.features) {
		dev_err(aci.input->dev.parent, "ECI accessory: all buttons down, ignored\n");
		return;
	}

	for (pin = 1; pin <= aci.features; pin++) {
		u8 fun = aci.io_fun[pin] & ~BIT(7);

		if (fun >= ARRAY_SIZE(eci_codes))
			break;
		code = eci_codes[fun];
		if (code >= KEY_MAX)
			input_report_switch(aci.input, code - KEY_MAX, eci_pin_active(port, pin));
		else
			input_report_key(aci.input, code, eci_pin_active(port, pin));
	}
	input_sync(aci.input);
}

/* The port registers (latched: what raised ACCINT), then ACCINT back on */
static void eci_read_buttons(bool latched)
{
	u8 buf[4] = { };
	int i, ret;

	for (i = 0; i < 3; i++) {	/* two re-reads of the latched data after an error */
		ret = aci_cmd(ACI_READ, ECI_READ_REG(latched || i ? ECICMD_LATCHED_PORT_DATA_0 :
						     ECICMD_PORT_DATA_0), buf, aci.port_regs);
		if (!ret)
			break;
	}
	if (ret)
		aci_release_keys();	/* Nokia's watchdog forced them up */
	else
		eci_report(get_unaligned_le32(buf));

	aci_write(TWL5031_ACIIDR_LSB, ACI_ACCINT);
	aci_imr(ACI_ACCINT, false);
}

/*
 * A basic button as Nokia's: a change reported 200 ms after it was sampled (unplugging shorts the
 * mic line: the plug is out by then, and the change is dropped), sampled every 220 ms while it's
 * down, ACCINT back on once it's up
 */
static void aci_button_work(struct work_struct *work)
{
	switch (READ_ONCE(aci.buttons)) {
	case ACI_BUTTONS_ECI:
		eci_read_buttons(true);
		return;
	case ACI_BUTTONS_BASIC:
		break;
	default:
		return;
	}

	if (aci.down != aci.reported && gpiod_get_value_cansleep(dfl61_jack_gpio.desc)) {
		input_report_key(aci.input, KEY_PHONE, aci.down);
		input_sync(aci.input);
		aci.reported = aci.down;
	}
	if (!aci.down) {
		aci_imr(ACI_ACCINT, false);
		return;
	}
	msleep(20);
	aci.down = aci_basic_down();
	schedule_delayed_work(&aci.button_work, msecs_to_jiffies(200));
}

/* Buttons on for a plug just classified */
static void aci_buttons_on(int buttons)
{
	if (buttons == ACI_BUTTONS_ECI) {
		if (eci_init()) {
			aci.mem_ok = false;
			dev_err(aci.input->dev.parent, "ECI accessory not answering: no buttons\n");
			return;
		}
		WRITE_ONCE(aci.buttons, buttons);
		eci_read_buttons(false);
		return;
	}
	/* ACCINT on 200 ms after the classification (Nokia's ACI_WAIT_PLUG_TO_BUTTONS) */
	aci_write(TWL5031_ACIIDR_LSB, ACI_ACCINT);
	aci.down = aci.reported = false;
	WRITE_ONCE(aci.buttons, buttons);
	schedule_delayed_work(&aci.button_work, msecs_to_jiffies(200));
}

/* No plug, or a new one: no buttons, none down */
static void aci_buttons_off(void)
{
	if (!aci.irq)
		return;
	WRITE_ONCE(aci.buttons, ACI_BUTTONS_NONE);
	aci_imr(ACI_ACCINT, true);
	cancel_delayed_work_sync(&aci.button_work);
	aci.mem_ok = false;
	cancel_work_sync(&aci.mic_work);
	aci_release_keys();
}

/* VAUX1 held while a plug is in, as Nokia's rm696_plug_set_state() */
static void aci_power(bool on)
{
	if (on == aci.powered)
		return;
	if (on ? regulator_enable(aci.v28) : regulator_disable(aci.v28))
		return;
	aci.powered = on;
}

static ssize_t memory_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (!aci.mem_ok)
		return -ENXIO;
	memcpy(buf, aci.memory, aci.mem_size);
	return aci.mem_size;
}
static DEVICE_ATTR_RO(memory);

static void aci_unregister_eci(void *pdev)
{
	platform_device_unregister(pdev);
}

/* The ACI block's ID: 0x55 0xaa ... 0x55, its version fifth (Nokia's is_twl5031_aci()) */
static int aci_version(void)
{
	u8 id[10];
	int i;

	aci_write(TWL5031_ACIID, 0);
	for (i = 0; i < ARRAY_SIZE(id); i++)
		id[i] = aci_read(TWL5031_ACIID);
	return id[0] == 0x55 && id[1] == 0xaa && id[9] == 0x55 ? id[4] : -ENODEV;
}

static int dfl61_aci_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct platform_device *eci;
	int i, irq, ver, ret;

	aci.v28 = devm_regulator_get(dev, "v28");
	if (IS_ERR(aci.v28))
		return dev_err_probe(dev, PTR_ERR(aci.v28), "no v28 supply\n");

	irq = platform_get_irq_byname_optional(pdev, "aci");
	if (irq == -EPROBE_DEFER)
		return irq;
	if (irq < 0)
		return 0;	/* no buttons */

	/*
	 * ES1.0 and ES1.1 take writes to 0x74-0x7e at any of their I2C addresses as ACI register
	 * writes (VAUX regulator writes among them); Nokia's twl-core shadowed the ACI registers
	 * around such writes (twl5031_probe_eci_bug()). Not ported: no buttons on those.
	 */
	if ((twl_get_type() & 0xffff) == 0x802f &&
	    (twl_get_version() == 0x0b || twl_get_version() == 0x1b)) {
		dev_warn(dev, "TWL5031 ES1.0/ES1.1: no jack buttons\n");
		return 0;
	}
	ver = aci_version();
	if (ver < 0) {
		dev_warn(dev, "no TWL5031 ACI block: no jack buttons\n");
		return 0;
	}

	mutex_init(&aci.lock);
	mutex_init(&aci.irq_lock);
	init_completion(&aci.done);
	ret = devm_delayed_work_autocancel(dev, &aci.button_work, aci_button_work);
	if (ret)
		return ret;
	ret = devm_work_autocancel(dev, &aci.mic_work, aci_mic_work);
	if (ret)
		return ret;

	aci.input = devm_input_allocate_device(dev);
	if (!aci.input)
		return -ENOMEM;
	aci.input->name = "ECI Accessory";
	for (i = 0; i < ARRAY_SIZE(eci_codes); i++) {
		if (eci_codes[i] >= KEY_MAX)
			__set_bit(eci_codes[i] - KEY_MAX, aci.input->swbit);
		else
			__set_bit(eci_codes[i], aci.input->keybit);
	}
	__set_bit(EV_KEY, aci.input->evbit);
	__set_bit(EV_SW, aci.input->evbit);
	__set_bit(EV_REP, aci.input->evbit);
	ret = input_register_device(aci.input);
	if (ret)
		return ret;
	aci.input->rep[REP_PERIOD] = 400;

	eci = platform_device_register_simple("ECI_accessory", 0, NULL, 0);
	if (IS_ERR(eci))
		return PTR_ERR(eci);
	ret = devm_add_action_or_reset(dev, aci_unregister_eci, eci);
	if (ret)
		return ret;
	ret = device_create_file(&eci->dev, &dev_attr_memory);
	if (ret)
		return ret;

	aci_imr(ACI_INTERNAL | ACI_ACCINT, true);
	ret = devm_request_threaded_irq(dev, irq, NULL, aci_irq, IRQF_ONESHOT, "twl5031_aci", NULL);
	if (ret)
		return ret;
	aci.irq = irq;
	dev_info(dev, "TWL5031 ACI v.%d (IDCODE %06x rev %02x): jack buttons\n", ver,
		 twl_get_type(), twl_get_version());
	return 0;
}

/*
 * Widgets for a measurement: forced on, off, or back under DAPM's control. Back to DAPM is disable
 * then enable: enable_pin alone leaves a widget forced, and DAPM never re-evaluates what follows a
 * widget whose power doesn't change (with the TPA's PGAs left forced, its mute widgets and the
 * headphone endpoint stayed as they were: muted on playback; and later plugs were measured with the
 * amp on)
 */
enum aci_pin { ACI_PIN_ON, ACI_PIN_OFF, ACI_PIN_DAPM };

static void aci_pins(struct snd_soc_card *card, const char * const *pins, int n, enum aci_pin state)
{
	struct snd_soc_dapm_context *dapm = snd_soc_card_to_dapm(card);
	int i;

	if (!snd_soc_card_is_instantiated(card))
		return;
	for (i = 0; i < n; i++) {
		if (state == ACI_PIN_ON) {
			snd_soc_dapm_force_enable_pin(dapm, pins[i]);
			continue;
		}
		snd_soc_dapm_disable_pin(dapm, pins[i]);
		if (state == ACI_PIN_DAPM)
			snd_soc_dapm_enable_pin(dapm, pins[i]);
	}
	snd_soc_dapm_sync(dapm);
}

static const char * const aci_bias[] = { "Headset Mic Bias" };
static const char * const aci_hp[] = { "TPA6140A2 Left PGA", "TPA6140A2 Right PGA" };

/*
 * One pass of Nokia's twl5031_av_detection(): the jack type and what it is, the mic line in mV
 * (alone, with the headphone amp, at the deciding step), and its buttons (ACI_BUTTONS_*: with them,
 * hook detection stays on)
 */
static int aci_detect(struct snd_soc_card *card, const char **what, int mv[3], int *buttons)
{
	int i;

	*buttons = ACI_BUTTONS_NONE;
	mv[2] = -1;

	aci_pins(&dfl61_dac33_card, aci_hp, 2, ACI_PIN_OFF);	/* the headphone amp off */
	aci_write(TWL5031_ACI_AUDIO_CTRL, aci_read(TWL5031_ACI_AUDIO_CTRL) | HOOK_DEB_BYPASS);
	aci_write(TWL5031_ECI_DBI_CTRL, ACI_ENABLE | ACI_DBI_MODE);	/* pull-ups off */
	aci_pins(card, aci_bias, 1, ACI_PIN_ON);
	msleep(ACI_SETTLE_MS);
	aci_write(TWL5031_ACI_AV_CTRL, ADC_SW_EN);

	/* Wrong pinout: with the plug's ground on the mic contact, the headphone amp pulls it down */
	mv[0] = aci_hsmic_mv();
	aci_pins(&dfl61_dac33_card, aci_hp, 2, ACI_PIN_ON);
	msleep(ACI_SETTLE_MS);
	mv[1] = aci_hsmic_mv();
	aci_pins(&dfl61_dac33_card, aci_hp, 2, ACI_PIN_DAPM);
	if (mv[0] - mv[1] > ACI_AHJ_NHJ_THRESHOLD) {
		*what = "wrong pinout (CTIA/OMTP)";
		return SND_JACK_MECHANICAL;
	}

	/* Headphones or a video cable */
	aci_write(TWL5031_ACI_AV_CTRL, ADC_SW_EN | AV_COMP1_EN);
	usleep_range(100, 200);
	if (aci_read(TWL5031_ACI_AV_CTRL) & STATUS_A1_COMP) {
		mv[2] = aci_hsmic_mv();
		if (mv[2] < ACI_HEADPHONE_THRESHOLD) {
			*what = "headphones";
			return SND_JACK_HEADPHONE;
		}
		if (mv[2] < ACI_AVOUT_THRESHOLD) {
			*what = "video cable";
			return SND_JACK_AVOUT;
		}
	}

	/* An ECI accessory: its chip answers a reset on the bus, the pull-ups on (10 tries: it may
	 * still be starting) */
	if (aci.irq) {
		aci_write(TWL5031_ECI_DBI_CTRL, ACI_ENABLE | ACI_ACCESSORY_MODE);
		for (i = 0; i < 10; i++) {
			if (!aci_cmd(ACI_RESET, 0, NULL, 0)) {
				*buttons = ACI_BUTTONS_ECI;
				*what = "ECI accessory";
				return SND_JACK_HEADSET;
			}
		}
		aci_write(TWL5031_ECI_DBI_CTRL, ACI_ENABLE | ACI_DBI_MODE);
	}

	/* Open cable (line out): comparator 2's pull-up only */
	aci_pins(card, aci_bias, 1, ACI_PIN_DAPM);
	aci_write(TWL5031_ACI_AV_CTRL, ADC_SW_EN | AV_COMP2_EN);
	usleep_range(50000, 52000);
	if (!(aci_read(TWL5031_ACI_AV_CTRL) & STATUS_A2_COMP)) {
		*what = "open cable";
		return SND_JACK_LINEOUT;
	}

	/* Basic headset or carkit: the drop across the series resistor, mic line grounded */
	aci_write(TWL5031_ACI_AV_CTRL, ADC_SW_EN | HSM_GROUNDED_EN);
	msleep(ACI_SETTLE_MS);
	mv[2] = aci_hsmic_mv();
	aci_write(TWL5031_ACI_AV_CTRL, AV_COMP1_EN | HOOK_DET_EN);
	aci_write(TWL5031_ECI_DBI_CTRL, ACI_ENABLE | ACI_ACCESSORY_MODE);
	*buttons = ACI_BUTTONS_BASIC;
	if (mv[2] < ACI_BASIC_HEADSET_THRESHOLD) {
		*what = "headset";
		return SND_JACK_HEADSET;
	}
	if (mv[2] > ACI_BASIC_CARKIT_THRESHOLD) {
		*what = "carkit";
		return SND_JACK_HEADPHONE;
	}
	*buttons = ACI_BUTTONS_NONE;
	aci_write(TWL5031_ACI_AV_CTRL, 0);
	*what = "unknown";
	return SND_JACK_MECHANICAL;
}

/*
 * As Nokia's aci_work: passes until two in a row agree, 200 ms after the quick ones (headphones,
 * video cable, unknown) against false results, "unknown" when none agree in 24 (stock's bail-out)
 */
#define ACI_DETECTS_NEEDED		2
#define ACI_REDETECTS_BAIL_OUT		24
#define ACI_QUICK_DETECTION_DELAY_MS	200

static int dfl61_jack_detect(void *data)
{
	struct snd_soc_card *card = data;
	const char *what, *prev = NULL;
	int type, count = 0, tries = 0, mv[3], buttons;

	aci_buttons_off();
	if (!gpiod_get_value_cansleep(dfl61_jack_gpio.desc)) {
		aci_disable_pullups();
		aci_power(false);
		return 0;
	}
	gpiod_set_value_cansleep(dfl61_tvout_gpio, 0);
	if (!dfl61_hsmic)
		return SND_JACK_HEADPHONE;	/* no MADC channel: no classification */

	/* The plug all the way in: Nokia waited 500 ms from AvPlugDet, the mic bias on for the last
	 * 200 (an ECI accessory's chip starts on it) */
	aci_power(true);
	msleep(100);
	aci_pins(card, aci_bias, 1, ACI_PIN_ON);
	msleep(200);

	for (;;) {
		type = aci_detect(card, &what, mv, &buttons);
		count = what == prev ? count + 1 : 0;
		prev = what;
		if (count == ACI_DETECTS_NEEDED - 1)
			break;
		if (++tries > ACI_REDETECTS_BAIL_OUT) {
			type = SND_JACK_MECHANICAL;
			what = "no stable result";
			buttons = ACI_BUTTONS_NONE;
			break;
		}
		if (!buttons && type != SND_JACK_LINEOUT)
			msleep(ACI_QUICK_DETECTION_DELAY_MS);
		/* the bias as the pass left it, as Nokia's: an ECI chip stays up for the next one */
		aci_disable_pullups();
		if (!gpiod_get_value_cansleep(dfl61_jack_gpio.desc)) {
			aci_pins(card, aci_bias, 1, ACI_PIN_DAPM);
			return 0;	/* pulled out meanwhile */
		}
	}
	if (!buttons)
		aci_disable_pullups();
	aci_pins(card, aci_bias, 1, ACI_PIN_DAPM);
	dev_info(card->dev, "AV jack: %s (mic line %d mV, %d with the headphone amp; %d; pass %d)\n",
		 what, mv[0], mv[1], mv[2], tries + 1);
	if (buttons)
		aci_buttons_on(buttons);
	if (type == SND_JACK_AVOUT)
		gpiod_set_value_cansleep(dfl61_tvout_gpio, 1);
	return type;
}

/* The TWL5031 card */

static int dfl61_twl_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);

	switch (params_channels(params)) {
	case 2:	/* stereo I2S */
		return snd_soc_runtime_set_dai_fmt(rtd, SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
						   SND_SOC_DAIFMT_CBP_CFP);
	case 4:	/* four-channel TDM */
		return snd_soc_runtime_set_dai_fmt(rtd, SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_IB_NF |
						   SND_SOC_DAIFMT_CBP_CFP);
	default:
		return -EINVAL;
	}
}

static const struct snd_soc_ops dfl61_twl_ops = {
	.hw_params = dfl61_twl_hw_params,
};

/* "dfl61-twl4030 Jack", with Nokia's report mask: Harmattan's policy (ohm's accessories plugin)
 * finds it by name and follows its switch events */
static int dfl61_twl_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	int ret;

	ret = snd_soc_card_jack_new(card, "Jack",
				    SND_JACK_MECHANICAL | SND_JACK_AVOUT | SND_JACK_HEADSET,
				    &dfl61_jack);
	if (ret)
		return ret;

	dfl61_jack_gpio.gpiod_dev = card->dev;
	dfl61_jack_gpio.data = card;
	return snd_soc_jack_add_gpios(&dfl61_jack, 1, &dfl61_jack_gpio);
}

/* An ECI accessory's own mic follows the headset mic's use, switched over the bus from a work: not
 * under DAPM's lock (Nokia's switched it here) */
static int dfl61_hsmic_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *k, int event)
{
	WRITE_ONCE(aci.mic_on, SND_SOC_DAPM_EVENT_ON(event));
	if (aci.irq)
		schedule_work(&aci.mic_work);
	return 0;
}

static int dfl61_spk_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *k, int event)
{
	gpiod_set_value_cansleep(dfl61_ihf_gpio, SND_SOC_DAPM_EVENT_ON(event));
	return 0;
}

/* PREDRIVEL feeds the DAC33's line-in: open that path on the other card while it's in use */
static int dfl61_interconnect_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *k,
				    int event)
{
	struct snd_soc_dapm_context *dapm = snd_soc_card_to_dapm(&dfl61_dac33_card);

	if (!snd_soc_card_is_instantiated(&dfl61_dac33_card))
		return 0;
	if (SND_SOC_DAPM_EVENT_ON(event))
		snd_soc_dapm_enable_pin(dapm, "twl4030 interconnect");
	else
		snd_soc_dapm_disable_pin(dapm, "twl4030 interconnect");
	return snd_soc_dapm_sync(dapm);
}

static const struct snd_soc_dapm_widget dfl61_twl_widgets[] = {
	SND_SOC_DAPM_SPK("Ext Spk", dfl61_spk_event),
	SND_SOC_DAPM_SPK("Earpiece", NULL),
	SND_SOC_DAPM_SPK("HAC", NULL),
	SND_SOC_DAPM_SPK("Vibra", NULL),
	SND_SOC_DAPM_SPK("DAC33 interconnect", dfl61_interconnect_event),
	SND_SOC_DAPM_MIC("Digital Mic", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", dfl61_hsmic_event),
	SND_SOC_DAPM_LINE("FMRX Left Line-in", NULL),
	SND_SOC_DAPM_LINE("FMRX Right Line-in", NULL),
};

/* Nokia's config 4 map; the mic biases are supplies on mainline's twl4030 */
static const struct snd_soc_dapm_route dfl61_twl_routes[] = {
	{"Ext Spk", NULL, "PREDRIVER"},
	{"Earpiece", NULL, "EARPIECE"},
	{"HAC", NULL, "HFL"},
	{"Vibra", NULL, "VIBRA"},
	{"DAC33 interconnect", NULL, "PREDRIVEL"},

	{"DIGIMIC0", NULL, "Digital Mic"},
	{"Digital Mic", NULL, "Mic Bias 1"},
	{"HSMIC", NULL, "Headset Mic"},
	{"Headset Mic", NULL, "Headset Mic Bias"},

	{"AUXL", NULL, "FMRX Left Line-in"},
	{"AUXR", NULL, "FMRX Right Line-in"},
};

SND_SOC_DAILINK_DEFS(twl,
	DAILINK_COMP_ARRAY(COMP_EMPTY()),
	DAILINK_COMP_ARRAY(COMP_CODEC("twl4030-codec", "twl4030-hifi")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

static struct snd_soc_dai_link dfl61_twl_link = {
	.name = "TWL4030",
	.stream_name = "TWL4030",
	.init = dfl61_twl_init,
	.ops = &dfl61_twl_ops,
	SND_SOC_DAILINK_REG(twl),
};

/* Fully routed: the codec pins config 4 leaves unconnected (main/sub/carkit mics, HSOL/HSOR, HFR,
 * carkit outputs, DIGIMIC1) are disabled, as Nokia's nc_pin list did */
static struct snd_soc_card dfl61_twl_card = {
	.name = "dfl61-twl4030",
	.owner = THIS_MODULE,
	.dai_link = &dfl61_twl_link,
	.num_links = 1,
	.dapm_widgets = dfl61_twl_widgets,
	.num_dapm_widgets = ARRAY_SIZE(dfl61_twl_widgets),
	.dapm_routes = dfl61_twl_routes,
	.num_dapm_routes = ARRAY_SIZE(dfl61_twl_routes),
	.fully_routed = true,
};

/* The DAC33 card */

static int dfl61_dac33_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	int ret;

	/* The DAC33 runs from the 32 kHz sleep clock (its PLL makes the rest) */
	ret = snd_soc_dai_set_sysclk(snd_soc_rtd_to_codec(rtd, 0), TLV320DAC33_SLEEPCLK, 32768,
				     SND_SOC_CLOCK_IN);
	if (ret)
		return ret;

	ret = snd_soc_limit_volume(card, "TPA6140A2 Headphone Playback Volume", 21);
	if (ret)
		return ret;

	snd_soc_dapm_disable_pin(snd_soc_card_to_dapm(card), "twl4030 interconnect");
	return 0;
}

static const struct snd_kcontrol_new dfl61_dac33_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
};

static const struct snd_soc_dapm_widget dfl61_dac33_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_LINE("twl4030 interconnect", NULL),
};

static const struct snd_soc_dapm_route dfl61_dac33_routes[] = {
	{"TPA6140A2 LEFTIN", NULL, "LEFT_LO"},
	{"TPA6140A2 RIGHTIN", NULL, "RIGHT_LO"},
	{"Headphone", NULL, "TPA6140A2 HPLEFT"},
	{"Headphone", NULL, "TPA6140A2 HPRIGHT"},

	{"LINEL", NULL, "twl4030 interconnect"},
	{"LINER", NULL, "twl4030 interconnect"},
};

SND_SOC_DAILINK_DEFS(dac33,
	DAILINK_COMP_ARRAY(COMP_EMPTY()),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "tlv320dac33-hifi")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

static struct snd_soc_dai_link dfl61_dac33_link = {
	.name = "TLV320DAC33",
	.stream_name = "DAC33",
	.dai_fmt = SND_SOC_DAIFMT_LEFT_J | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBP_CFP,
	.init = dfl61_dac33_init,
	SND_SOC_DAILINK_REG(dac33),
};

static struct snd_soc_aux_dev dfl61_dac33_aux;

static struct snd_soc_card dfl61_dac33_card = {
	.name = "dfl61-dac33",
	.owner = THIS_MODULE,
	.dai_link = &dfl61_dac33_link,
	.num_links = 1,
	.aux_dev = &dfl61_dac33_aux,
	.num_aux_devs = 1,
	.controls = dfl61_dac33_controls,
	.num_controls = ARRAY_SIZE(dfl61_dac33_controls),
	.dapm_widgets = dfl61_dac33_widgets,
	.num_dapm_widgets = ARRAY_SIZE(dfl61_dac33_widgets),
	.dapm_routes = dfl61_dac33_routes,
	.num_dapm_routes = ARRAY_SIZE(dfl61_dac33_routes),
};

/*
 * The Bluetooth/FM PCM card, "dfl61-wl1273" as stock: the WL1273's PCM on McBSP4 (NOLO muxes it on
 * the GPMC nCS4-7 pads), with Nokia's WL1273 codec. Bluetooth SCO by default (PulseAudio's
 * sink.hw1/source.hw1, 8 kHz mono): the chip's firmware sets that PCM up (TI's Write CODEC Config:
 * master, 8 kHz frames, one 16-bit slot a clock after the frame sync, i.e. DSP_A). FM ("Codec Mode"
 * FmRx/FmTx, fmrxd's 48 kHz stereo capture) is I2S, the chip master too, as Nokia's dfl61-wl1273.c.
 */
SND_SOC_DAILINK_DEFS(bt,
	DAILINK_COMP_ARRAY(COMP_EMPTY()),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "wl1273-fm")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

static int dfl61_bt_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);

	if (wl1273_get_codec_mode(snd_soc_rtd_to_codec(rtd, 0)->component) == WL1273_MODE_BT)
		return snd_soc_runtime_set_dai_fmt(rtd, SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_IB_NF |
						   SND_SOC_DAIFMT_CBP_CFP);
	return snd_soc_runtime_set_dai_fmt(rtd, SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
					   SND_SOC_DAIFMT_CBP_CFP);
}

static const struct snd_soc_ops dfl61_bt_ops = {
	.startup = dfl61_bt_startup,
};

static struct snd_soc_dai_link dfl61_bt_link = {
	.name = "BT/FM PCM",
	.stream_name = "BT/FM Stream",
	.dai_fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_IB_NF | SND_SOC_DAIFMT_CBP_CFP,
	.ops = &dfl61_bt_ops,
	SND_SOC_DAILINK_REG(bt),
};

static struct snd_soc_card dfl61_bt_card = {
	.name = "dfl61-wl1273",
	.owner = THIS_MODULE,
	.dai_link = &dfl61_bt_link,
	.num_links = 1,
};

static int dfl61_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = (struct snd_soc_card *)of_device_get_match_data(&pdev->dev);
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_dai_link *link = card->dai_link;
	struct device_node *mcbsp;

	mcbsp = of_parse_phandle(np, "nokia,mcbsp", 0);
	if (!mcbsp)
		return dev_err_probe(&pdev->dev, -EINVAL, "no nokia,mcbsp\n");
	link->cpus->of_node = mcbsp;
	link->platforms->of_node = mcbsp;

	if (card == &dfl61_twl_card) {
		dfl61_ihf_gpio = devm_gpiod_get(&pdev->dev, "ihf", GPIOD_OUT_LOW);
		if (IS_ERR(dfl61_ihf_gpio))
			return dev_err_probe(&pdev->dev, PTR_ERR(dfl61_ihf_gpio), "no ihf gpio\n");
		dfl61_tvout_gpio = devm_gpiod_get_optional(&pdev->dev, "tvout", GPIOD_OUT_LOW);
		if (IS_ERR(dfl61_tvout_gpio))
			return dev_err_probe(&pdev->dev, PTR_ERR(dfl61_tvout_gpio), "no tvout gpio\n");
		/* the MADC's headset mic channel, for accessory classification (optional) */
		dfl61_hsmic = devm_iio_channel_get(&pdev->dev, "hsmic");
		if (IS_ERR(dfl61_hsmic)) {
			if (PTR_ERR(dfl61_hsmic) == -EPROBE_DEFER)
				return -EPROBE_DEFER;
			dfl61_hsmic = NULL;
		}
		if (dfl61_hsmic) {
			int ret = dfl61_aci_probe(pdev);

			if (ret)
				return ret;
		}
	} else {
		link->codecs->of_node = of_parse_phandle(np, "nokia,audio-codec", 0);
		if (!link->codecs->of_node)
			return dev_err_probe(&pdev->dev, -EINVAL, "no nokia,audio-codec\n");
	}
	if (card == &dfl61_dac33_card) {
		dfl61_dac33_aux.dlc.of_node = of_parse_phandle(np, "nokia,headphone-amplifier", 0);
		if (!dfl61_dac33_aux.dlc.of_node)
			return dev_err_probe(&pdev->dev, -EINVAL, "no headphone amplifier\n");
	}

	card->dev = &pdev->dev;
	return devm_snd_soc_register_card(&pdev->dev, card);
}

static const struct of_device_id dfl61_of_match[] = {
	{ .compatible = "nokia,dfl61-twl4030", .data = &dfl61_twl_card },
	{ .compatible = "nokia,dfl61-dac33", .data = &dfl61_dac33_card },
	{ .compatible = "nokia,dfl61-wl1273", .data = &dfl61_bt_card },
	{ }
};
MODULE_DEVICE_TABLE(of, dfl61_of_match);

static struct platform_driver dfl61_driver = {
	.probe = dfl61_probe,
	.driver = {
		.name = "dfl61-audio",
		.of_match_table = dfl61_of_match,
	},
};
module_platform_driver(dfl61_driver);

MODULE_DESCRIPTION("ALSA SoC for the Nokia N9 (DFL61), after Nokia's dfl61 machine drivers");
MODULE_LICENSE("GPL");
