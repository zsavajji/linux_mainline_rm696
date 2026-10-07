// SPDX-License-Identifier: GPL-2.0-only
/*
 * ALSA SoC WL1273 codec driver
 *
 * Nokia's from its Harmattan kernel (2.6.32), the codec of the N9's "dfl61-wl1273" card: the
 * chip's PCM on McBSP4, Bluetooth SCO (8 kHz mono) or FM over I2S, and stock's controls, which
 * fmrxd and Nokia's test servers set: "Mode Switch" powers the FM part (Rx/Tx/Off/Suspend), "Codec
 * Mode" picks Bt/FmRx/FmTx, "Audio Switch" Digital (I2S) or Analog (into the TWL5031's AUX).
 *
 * Author:      Matti Aaltonen, <matti.j.aaltonen@nokia.com>
 *
 * Copyright:   (C) 2010 Nokia Corporation
 */

#include <linux/mfd/wl1273-core.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "wl1273.h"

/* codec private data */
struct wl1273_priv {
	enum wl1273_mode mode;
	int ctune;
	int tx_power;
	int rds_mode;
	int pwr_mode;
	int audio_mode;
	int region;
	int preemphasis;
	int spacing;

	struct wl1273_core *core;
};

static int wl1273_get_audio_route(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct wl1273_priv *wl1273 = snd_soc_component_get_drvdata(component);

	ucontrol->value.enumerated.item[0] = wl1273->mode;

	return 0;
}

static int wl1273_set_audio_route(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct wl1273_priv *wl1273 = snd_soc_component_get_drvdata(component);

	if (wl1273->mode == ucontrol->value.enumerated.item[0])
		return 0;

	/* Do not allow changes while stream is running */
	if (snd_soc_component_active(component))
		return -EPERM;

	if (ucontrol->value.enumerated.item[0] > WL1273_MODE_FM_TX)
		return -EINVAL;

	wl1273->mode = ucontrol->value.enumerated.item[0];

	return 1;
}

static const char * const wl1273_audio_route[] = { "Bt", "FmRx", "FmTx" };

static SOC_ENUM_SINGLE_EXT_DECL(wl1273_enum, wl1273_audio_route);

static int snd_wl1273_fm_ctune_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct wl1273_priv *wl1273 = snd_soc_component_get_drvdata(component);

	wl1273->ctune = wl1273_fm_get_tx_ctune(wl1273->core);
	ucontrol->value.integer.value[0] = wl1273->ctune;

	return 0;
}

static int snd_wl1273_fm_power_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct wl1273_priv *wl1273 = snd_soc_component_get_drvdata(component);

	wl1273->tx_power = 31 - wl1273_fm_get_tx_power(wl1273->core);
	ucontrol->value.integer.value[0] = wl1273->tx_power;

	return 0;
}

static int snd_wl1273_fm_power_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct wl1273_priv *wl1273 = snd_soc_component_get_drvdata(component);
	int val, r;

	val = ucontrol->value.integer.value[0];
	if (val < 0 || val > 31)
		return -EDOM;

	if (wl1273->tx_power == val)
		return 0;

	wl1273->tx_power = val;
	r = wl1273_fm_set_tx_power(wl1273->core, 31 - val);
	if (r)
		return r;

	return 1;
}

static int snd_wl1273_fm_rds_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct wl1273_priv *wl1273 = snd_soc_component_get_drvdata(component);

	wl1273->rds_mode = wl1273_fm_get_rds(wl1273->core);
	ucontrol->value.enumerated.item[0] = wl1273->rds_mode;

	return 0;
}

static int snd_wl1273_fm_rds_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct wl1273_priv *wl1273 = snd_soc_component_get_drvdata(component);
	unsigned int val;
	int r;

	val = ucontrol->value.enumerated.item[0];
	if (wl1273->rds_mode == val)
		return 0;

	wl1273->rds_mode = val;
	r = wl1273_fm_set_rds(wl1273->core, val);
	if (r)
		return r;

	return 1;
}

static const char * const wl1273_rds_strings[] = { "Off", "On", "Reset" };

static SOC_ENUM_SINGLE_EXT_DECL(wl1273_rds_enum, wl1273_rds_strings);

static int snd_wl1273_fm_mode_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct wl1273_priv *wl1273 = snd_soc_component_get_drvdata(component);

	wl1273->pwr_mode = wl1273_fm_get_mode(wl1273->core);
	ucontrol->value.enumerated.item[0] = wl1273->pwr_mode;

	return 0;
}

static int snd_wl1273_fm_mode_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct wl1273_priv *wl1273 = snd_soc_component_get_drvdata(component);
	int val, r;

	val = ucontrol->value.enumerated.item[0];
	if (wl1273->pwr_mode == val)
		return 0;

	wl1273->pwr_mode = val;
	r = wl1273_fm_set_mode(wl1273->core, val);
	if (r < 0)
		return r;

	return 1;
}

static const char * const wl1273_mode_strings[] = { "Rx", "Tx", "Off", "Suspend" };

static SOC_ENUM_SINGLE_EXT_DECL(wl1273_mode_enum, wl1273_mode_strings);

static int snd_wl1273_fm_audio_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct wl1273_priv *wl1273 = snd_soc_component_get_drvdata(component);

	wl1273->audio_mode = wl1273_fm_get_audio(wl1273->core);
	ucontrol->value.enumerated.item[0] = wl1273->audio_mode;

	return 0;
}

static int snd_wl1273_fm_audio_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct wl1273_priv *wl1273 = snd_soc_component_get_drvdata(component);
	int val, r;

	val = ucontrol->value.enumerated.item[0];
	if (wl1273->audio_mode == val)
		return 0;

	wl1273->audio_mode = val;
	r = wl1273_fm_set_audio(wl1273->core, val);
	if (r < 0)
		return r;

	return 1;
}

static const char * const wl1273_audio_strings[] = { "Digital", "Analog" };

static SOC_ENUM_SINGLE_EXT_DECL(wl1273_audio_enum, wl1273_audio_strings);

static int snd_wl1273_fm_region_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct wl1273_priv *wl1273 = snd_soc_component_get_drvdata(component);

	wl1273->region = wl1273_fm_get_region(wl1273->core);
	ucontrol->value.enumerated.item[0] = wl1273->region;

	return 0;
}

static int snd_wl1273_fm_region_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct wl1273_priv *wl1273 = snd_soc_component_get_drvdata(component);
	int val, r;

	val = ucontrol->value.enumerated.item[0];
	if (wl1273->region == val)
		return 0;

	wl1273->region = val;
	r = wl1273_fm_set_region(wl1273->core, val);
	if (r < 0)
		return r;

	return 1;
}

static const char * const wl1273_region_strings[] = { "Japan", "USA-Europe" };

static SOC_ENUM_SINGLE_EXT_DECL(wl1273_region_enum, wl1273_region_strings);

static int snd_wl1273_fm_preemphasis_get(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct wl1273_priv *wl1273 = snd_soc_component_get_drvdata(component);

	wl1273->preemphasis = wl1273_fm_get_preemphasis(wl1273->core);
	ucontrol->value.enumerated.item[0] = wl1273->preemphasis;

	return 0;
}

static int snd_wl1273_fm_preemphasis_put(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct wl1273_priv *wl1273 = snd_soc_component_get_drvdata(component);
	int val, r;

	val = ucontrol->value.enumerated.item[0];
	if (wl1273->preemphasis == val)
		return 0;

	wl1273->preemphasis = val;
	r = wl1273_fm_set_preemphasis(wl1273->core, val);
	if (r < 0)
		return 0;

	return 1;
}

static const char * const wl1273_preemphasis_strings[] = { "Off", "50us", "75us" };

static SOC_ENUM_SINGLE_EXT_DECL(wl1273_preemphasis_enum, wl1273_preemphasis_strings);

static int snd_wl1273_fm_spacing_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct wl1273_priv *wl1273 = snd_soc_component_get_drvdata(component);
	int r;

	r = wl1273_fm_tx_get_spacing(wl1273->core);
	if (r < 0)
		return r;

	wl1273->spacing = r;
	ucontrol->value.enumerated.item[0] = wl1273->spacing;

	return 0;
}

static int snd_wl1273_fm_spacing_put(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct wl1273_priv *wl1273 = snd_soc_component_get_drvdata(component);
	int val, r;

	val = ucontrol->value.enumerated.item[0];
	if (wl1273->spacing == val)
		return 0;

	wl1273->spacing = val;
	r = wl1273_fm_tx_set_spacing(wl1273->core, val);
	if (r < 0)
		return r;

	return 1;
}

static const char * const wl1273_spacing_strings[] = { "50kHz", "100kHz", "200kHz" };

static SOC_ENUM_SINGLE_EXT_DECL(wl1273_spacing_enum, wl1273_spacing_strings);

static int snd_wl1273_fm_volume_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct wl1273_priv *wl1273 = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = wl1273->core->volume;

	return 0;
}

static int snd_wl1273_fm_volume_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct wl1273_priv *wl1273 = snd_soc_component_get_drvdata(component);
	int val, r;

	val = ucontrol->value.integer.value[0];
	if (val < 0 || val > WL1273_MAX_VOLUME)
		return -EINVAL;

	r = wl1273_fm_set_volume(wl1273->core, val);
	if (r)
		return r;

	return 1;
}

static const struct snd_kcontrol_new wl1273_controls[] = {
	SOC_ENUM_EXT("Codec Mode", wl1273_enum,
		     wl1273_get_audio_route, wl1273_set_audio_route),
	SOC_SINGLE_EXT("Tx Ctune", 0, 0, 255, 0, snd_wl1273_fm_ctune_get, NULL),
	SOC_SINGLE_EXT("Tx Power", 0, 0, 31, 0, snd_wl1273_fm_power_get,
		       snd_wl1273_fm_power_put),
	SOC_ENUM_EXT("Rds Switch", wl1273_rds_enum,
		     snd_wl1273_fm_rds_get,  snd_wl1273_fm_rds_put),
	SOC_ENUM_EXT("Mode Switch", wl1273_mode_enum,
		     snd_wl1273_fm_mode_get,  snd_wl1273_fm_mode_put),
	SOC_ENUM_EXT("Audio Switch", wl1273_audio_enum,
		     snd_wl1273_fm_audio_get,  snd_wl1273_fm_audio_put),
	SOC_ENUM_EXT("Region Switch", wl1273_region_enum,
		     snd_wl1273_fm_region_get,  snd_wl1273_fm_region_put),
	SOC_ENUM_EXT("Pre-emphasis", wl1273_preemphasis_enum,
		     snd_wl1273_fm_preemphasis_get,
		     snd_wl1273_fm_preemphasis_put),
	SOC_ENUM_EXT("Channel Spacing", wl1273_spacing_enum,
		     snd_wl1273_fm_spacing_get,  snd_wl1273_fm_spacing_put),
	SOC_SINGLE_EXT("Master Capture Volume", 0, 0, WL1273_MAX_VOLUME, 0,
		       snd_wl1273_fm_volume_get, snd_wl1273_fm_volume_put),
};

static int wl1273_startup(struct snd_pcm_substream *substream,
			  struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_pcm *pcm = rtd->pcm;
	struct wl1273_priv *wl1273 = snd_soc_component_get_drvdata(dai->component);

	switch (wl1273->mode) {
	case WL1273_MODE_BT:
		pcm->info_flags &= ~SNDRV_PCM_INFO_HALF_DUPLEX;
		break;
	case WL1273_MODE_FM_RX:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			pr_err("Cannot play in RX mode.\n");
			return -EINVAL;
		}
		pcm->info_flags |= SNDRV_PCM_INFO_HALF_DUPLEX;
		break;
	case WL1273_MODE_FM_TX:
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
			pr_err("Cannot capture in TX mode.\n");
			return -EINVAL;
		}
		pcm->info_flags |= SNDRV_PCM_INFO_HALF_DUPLEX;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int wl1273_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct wl1273_priv *wl1273 = snd_soc_component_get_drvdata(dai->component);
	struct wl1273_core *core = wl1273->core;
	unsigned int rate, width;
	int r;

	if (params_format(params) != SNDRV_PCM_FORMAT_S16_LE) {
		pr_err("Only SNDRV_PCM_FORMAT_S16_LE supported.\n");
		return -EINVAL;
	}

	rate = params_rate(params);
	width = params_width(params);

	if (wl1273->mode == WL1273_MODE_BT) {
		if (rate != 8000) {
			pr_err("Rate %d not supported.\n", params_rate(params));
			return -EINVAL;
		}

		if (params_channels(params) != 1) {
			pr_err("Only mono supported.\n");
			return -EINVAL;
		}

		return 0;
	}

	if (wl1273->mode == WL1273_MODE_FM_TX &&
	    substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		pr_err("Only playback supported with TX.\n");
		return -EINVAL;
	}

	if (wl1273->mode == WL1273_MODE_FM_RX  &&
	    substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		pr_err("Only capture supported with RX.\n");
		return -EINVAL;
	}

	if (wl1273->mode != WL1273_MODE_FM_RX  &&
	    wl1273->mode != WL1273_MODE_FM_TX) {
		pr_err("Unexpected mode: %d.\n", wl1273->mode);
		return -EINVAL;
	}

	r = wl1273_fm_set_i2s_mode(core, rate, width);
	if (r)
		return r;

	r = wl1273_fm_set_channel_number(core, (params_channels(params)));
	if (r)
		return r;

	return 0;
}

static const struct snd_soc_dai_ops wl1273_dai_ops = {
	.startup	= wl1273_startup,
	.hw_params	= wl1273_hw_params,
};

static struct snd_soc_dai_driver wl1273_dai = {
	.name = "wl1273-fm",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE},
	.ops = &wl1273_dai_ops,
};

/* For the machine driver: the McBSP4 format follows the mode (DSP_A for Bluetooth, I2S for FM) */
enum wl1273_mode wl1273_get_codec_mode(struct snd_soc_component *component)
{
	struct wl1273_priv *wl1273 = snd_soc_component_get_drvdata(component);

	return wl1273->mode;
}
EXPORT_SYMBOL_GPL(wl1273_get_codec_mode);

static const struct snd_soc_component_driver soc_component_dev_wl1273 = {
	.controls		= wl1273_controls,
	.num_controls		= ARRAY_SIZE(wl1273_controls),
	.endianness		= 1,
};

static int wl1273_codec_probe(struct platform_device *pdev)
{
	struct wl1273_core **pdata = pdev->dev.platform_data;
	struct wl1273_priv *wl1273;

	if (!pdata)
		return dev_err_probe(&pdev->dev, -EINVAL, "Platform data is missing.\n");

	wl1273 = devm_kzalloc(&pdev->dev, sizeof(*wl1273), GFP_KERNEL);
	if (!wl1273)
		return -ENOMEM;

	wl1273->mode = WL1273_MODE_BT;
	wl1273->ctune = -1;
	wl1273->tx_power = -1;
	wl1273->rds_mode = -1;
	wl1273->pwr_mode = -1;
	wl1273->audio_mode = -1;
	wl1273->region = -1;
	wl1273->preemphasis = -1;
	wl1273->spacing = -1;
	wl1273->core = *pdata;

	platform_set_drvdata(pdev, wl1273);

	return devm_snd_soc_register_component(&pdev->dev, &soc_component_dev_wl1273,
					       &wl1273_dai, 1);
}

MODULE_ALIAS("platform:wl1273_codec_audio");

static struct platform_driver wl1273_codec_driver = {
	.probe		= wl1273_codec_probe,
	.driver		= {
		.name	= "wl1273_codec_audio",
	},
};
module_platform_driver(wl1273_codec_driver);

MODULE_AUTHOR("Matti Aaltonen <matti.j.aaltonen@nokia.com>");
MODULE_DESCRIPTION("ASoC WL1273 codec driver");
MODULE_LICENSE("GPL");
