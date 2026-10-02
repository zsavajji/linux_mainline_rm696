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
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/consumer.h>
#include <linux/mfd/twl.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "../codecs/tlv320dac33.h"

static struct gpio_desc *dfl61_ihf_gpio;
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
 * the speaker. Not ported: Nokia's ECI accessories (remote buttons), which come out as headsets, and
 * the headset button.
 */
static int dfl61_jack_detect(void *data);

static struct snd_soc_jack_gpio dfl61_jack_gpio = {
	.name = "jack",
	.report = SND_JACK_HEADSET | SND_JACK_AVOUT | SND_JACK_MECHANICAL,
	.debounce_time = 200,
	.jack_status_check = dfl61_jack_detect,
};

/* TWL5031 accessory block (Nokia's include/linux/mfd/aci.h) */
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
 * (alone, with the headphone amp, at the deciding step), and whether hook detection stays on
 */
static int aci_detect(struct snd_soc_card *card, const char **what, int mv[3], bool *keep)
{
	*keep = false;
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
	*keep = true;
	if (mv[2] < ACI_BASIC_HEADSET_THRESHOLD) {
		*what = "headset";
		return SND_JACK_HEADSET;
	}
	if (mv[2] > ACI_BASIC_CARKIT_THRESHOLD) {
		*what = "carkit";
		return SND_JACK_HEADPHONE;
	}
	*keep = false;
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
	int type, count = 0, tries = 0, mv[3];
	bool keep;

	if (!gpiod_get_value_cansleep(dfl61_jack_gpio.desc)) {
		aci_disable_pullups();
		return 0;
	}
	if (!dfl61_hsmic)
		return SND_JACK_HEADPHONE;	/* no MADC channel: no classification */

	msleep(300);	/* the plug all the way in: Nokia waited 500 ms from AvPlugDet */

	for (;;) {
		type = aci_detect(card, &what, mv, &keep);
		count = what == prev ? count + 1 : 0;
		prev = what;
		if (count == ACI_DETECTS_NEEDED - 1)
			break;
		if (++tries > ACI_REDETECTS_BAIL_OUT) {
			type = SND_JACK_MECHANICAL;
			what = "no stable result";
			keep = false;
			break;
		}
		if (!keep && type != SND_JACK_LINEOUT)
			msleep(ACI_QUICK_DETECTION_DELAY_MS);
		aci_disable_pullups();
		aci_pins(card, aci_bias, 1, ACI_PIN_DAPM);
		if (!gpiod_get_value_cansleep(dfl61_jack_gpio.desc))
			return 0;	/* pulled out meanwhile */
	}
	if (!keep)
		aci_disable_pullups();
	aci_pins(card, aci_bias, 1, ACI_PIN_DAPM);
	dev_info(card->dev, "AV jack: %s (mic line %d mV, %d with the headphone amp; %d; pass %d)\n",
		 what, mv[0], mv[1], mv[2], tries + 1);
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
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
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
 * The Bluetooth PCM card, "dfl61-wl1273" as stock (PulseAudio's sink.hw1/source.hw1 for SCO over PCM):
 * the BT chip's PCM on McBSP4 (NOLO muxes it on the GPMC nCS4-7 pads). The chip's firmware sets the
 * PCM up (TI's Write CODEC Config: master, 8 kHz frames, one 16-bit slot a clock after the frame
 * sync), so the generic bt-sco codec stands in for stock's WL1273 one, which also had FM modes
 */
SND_SOC_DAILINK_DEFS(bt,
	DAILINK_COMP_ARRAY(COMP_EMPTY()),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "bt-sco-pcm")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

static struct snd_soc_dai_link dfl61_bt_link = {
	.name = "BT/FM PCM",
	.stream_name = "BT/FM Stream",
	.dai_fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_IB_NF | SND_SOC_DAIFMT_CBP_CFP,
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
		/* the MADC's headset mic channel, for accessory classification (optional) */
		dfl61_hsmic = devm_iio_channel_get(&pdev->dev, "hsmic");
		if (IS_ERR(dfl61_hsmic)) {
			if (PTR_ERR(dfl61_hsmic) == -EPROBE_DEFER)
				return -EPROBE_DEFER;
			dfl61_hsmic = NULL;
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
