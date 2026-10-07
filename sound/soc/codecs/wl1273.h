/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * sound/soc/codec/wl1273.h -- ALSA SoC WL1273 codec driver
 *
 * Copyright (C) Nokia Corporation
 * Author: Matti Aaltonen <matti.j.aaltonen@nokia.com>
 */

#ifndef __WL1273_CODEC_H__
#define __WL1273_CODEC_H__

struct snd_soc_component;

/* The "Codec Mode" control's values */
enum wl1273_mode { WL1273_MODE_BT, WL1273_MODE_FM_RX, WL1273_MODE_FM_TX };

enum wl1273_mode wl1273_get_codec_mode(struct snd_soc_component *component);

#endif	/* End of __WL1273_CODEC_H__ */
