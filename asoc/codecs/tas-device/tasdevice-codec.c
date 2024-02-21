/*
 * TAS2563/TAS2871 Linux Driver
 *
 * Copyright (C) 2022 - 2023 Texas Instruments Incorporated
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/crc8.h>
#include <linux/firmware.h>
#include <linux/miscdevice.h>
#include <linux/of_gpio.h>
#include <linux/version.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "tasdevice.h"
#include "tasdevice-dsp.h"
#include "tasdevice-codec.h"
#include "tasdevice-ctl.h"
#include "tasdevice-regbin.h"
#include "tasdevice-rw.h"

#define TASDEVICE_CRC8_POLYNOMIAL	0x4d
#define TASDEVICE_CLK_DIR_IN		(0)
#define TASDEVICE_CLK_DIR_OUT		(1)

static int tasdevice_program_get(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pValue)
{
	struct snd_soc_component *codec
		= snd_soc_kcontrol_component(pKcontrol);
	struct tasdevice_priv *pTAS2781 = snd_soc_component_get_drvdata(codec);

	mutex_lock(&pTAS2781->codec_lock);
	pValue->value.integer.value[0] = pTAS2781->cur_prog;
	mutex_unlock(&pTAS2781->codec_lock);
	return 0;
}

static int tasdevice_program_put(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pValue)
{
	struct snd_soc_component *codec
		= snd_soc_kcontrol_component(pKcontrol);
	struct tasdevice_priv *tas_priv = snd_soc_component_get_drvdata(codec);
	struct tasdevice_fw *tas_fw = tas_priv->fmw;
	int nr_program = pValue->value.integer.value[0];
	int max_val = tas_fw->nr_programs, ret = 0;

	/* 0:		 dsp mode
	 * non-zero:	bypass mode
	 */
	nr_program = (nr_program) ? max_val : 0;
	mutex_lock(&tas_priv->codec_lock);
	if (tas_priv->cur_prog != nr_program) {
		tas_priv->cur_prog = nr_program;
		ret = 1;
	}
	mutex_unlock(&tas_priv->codec_lock);
	return ret;
}

static int tasdevice_configuration_get(
	struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{

	struct snd_soc_component *codec
		= snd_soc_kcontrol_component(pKcontrol);
	struct tasdevice_priv *tas_priv = snd_soc_component_get_drvdata(codec);

	mutex_lock(&tas_priv->codec_lock);
	pValue->value.integer.value[0] = tas_priv->cur_conf;
	mutex_unlock(&tas_priv->codec_lock);
	return 0;
}

static int tasdevice_configuration_put(
	struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
	struct snd_soc_component *codec
		= snd_soc_kcontrol_component(pKcontrol);
	struct tasdevice_priv *tas_priv = snd_soc_component_get_drvdata(codec);
	struct tasdevice_fw *tas_fw = tas_priv->fmw;
	int nConfiguration = pValue->value.integer.value[0];
	int max_val = tas_fw->nr_configurations - 1, ret = 0;

	nConfiguration = clamp(nConfiguration, 0, max_val);
	mutex_lock(&tas_priv->codec_lock);
	if (tas_priv->cur_conf != nConfiguration) {
		tas_priv->cur_conf = nConfiguration;
		ret = 1;
	}
	mutex_unlock(&tas_priv->codec_lock);
	return ret;
}

static int tasdevice_dapm_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *codec = snd_soc_dapm_to_component(w->dapm);
	struct tasdevice_priv *tas_dev = snd_soc_component_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		dev_info(tas_dev->dev, "SND_SOC_DAPM_POST_PMU\n");
		break;
	case SND_SOC_DAPM_PRE_PMD:
		dev_info(tas_dev->dev, "SND_SOC_DAPM_PRE_PMD\n");
		break;
	}

	return 0;
}

static const struct snd_soc_dapm_widget tasdevice_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("ASI", "ASI Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT_E("ASI OUT", "ASI Capture", 0, SND_SOC_NOPM,
		0, 0, tasdevice_dapm_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_SPK("SPK", tasdevice_dapm_event),
	SND_SOC_DAPM_OUTPUT("OUT"),
	SND_SOC_DAPM_INPUT("DMIC")
};

static const struct snd_soc_dapm_route tasdevice_audio_map[] = {
	{"SPK", NULL, "ASI"},
	{"OUT", NULL, "SPK"},
	{"ASI OUT", NULL, "DMIC"}
};


static int tasdevice_startup(struct snd_pcm_substream *substream,
						struct snd_soc_dai *dai)
{
	struct snd_soc_component *codec = dai->component;
	struct tasdevice_priv *tas_dev = snd_soc_component_get_drvdata(codec);
	int ret = 0;

	if (tas_dev->fw_state != TASDEVICE_DSP_FW_ALL_OK) {
		dev_err(tas_dev->dev, "DSP bin file not loaded\n");
		ret = -EINVAL;
	}
	return ret;
}

static int tasdevice_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct tasdevice_priv *tas_dev = snd_soc_dai_get_drvdata(dai);
	unsigned int fsrate;
	unsigned int slot_width;
	int bclk_rate;
	int rc = 0;

	dev_info(tas_dev->dev, "%s: %s\n",
		__func__, substream->stream == SNDRV_PCM_STREAM_PLAYBACK?
		"Playback":"Capture");

	fsrate = params_rate(params);
	switch (fsrate) {
	case 48000:
		break;
	case 44100:
		break;
	default:
		dev_err(tas_dev->dev,
			"%s: incorrect sample rate = %u\n",
			__func__, fsrate);
		rc = -EINVAL;
		goto out;
	}

	slot_width = params_width(params);
	switch (slot_width) {
	case 16:
		break;
	case 20:
		break;
	case 24:
		break;
	case 32:
		break;
	default:
		dev_err(tas_dev->dev,
			"%s: incorrect slot width = %u\n",
			__func__, slot_width);
		rc = -EINVAL;
		goto out;
	}

	bclk_rate = snd_soc_params_to_bclk(params);
	if (bclk_rate < 0) {
		dev_err(tas_dev->dev,
			"%s: incorrect bclk rate = %d\n",
			__func__, bclk_rate);
		rc = bclk_rate;
		goto out;
	}
	dev_info(tas_dev->dev, "%s: BCLK rate = %d Channel = %d"
		"Sample rate = %u slot width = %u\n",
		__func__, bclk_rate, params_channels(params),
		fsrate, slot_width);
out:
	return rc;
}

static int tasdevice_set_dai_sysclk(struct snd_soc_dai *codec_dai,
	int clk_id, unsigned int freq, int dir)
{
	struct tasdevice_priv *tas_dev = snd_soc_dai_get_drvdata(codec_dai);

	dev_info(tas_dev->dev,
		"%s: clk_id = %d, freq = %u, CLK direction %s\n",
		__func__, clk_id, freq,
		dir == TASDEVICE_CLK_DIR_OUT ? "OUT":"IN");

	return 0;
}

void powercontrol_routine(struct work_struct *work)
{
	struct tasdevice_priv *tas_dev =
		container_of(work, struct tasdevice_priv,
		powercontrol_work.work);
	struct tasdevice_fw *fw = tas_dev->fmw;
	int profile_cfg_id = tas_dev->mtRegbin.profile_cfg_id;

	dev_info(tas_dev->dev, "%s: enter\n", __func__);

	if (tas_dev->pstream == 0 && tas_dev->cstream == 0) {
		dev_info(tas_dev->dev,
			"%s: power off has been done before power on has been executed\n",
			__func__);
		goto out;
	}
	mutex_lock(&tas_dev->codec_lock);

	if (fw && tas_dev->cur_prog == 0) {
		/*dsp mode or tuning mode*/
		dev_info(tas_dev->dev, "%s: %s\n", __func__,
			fw->mpConfigurations[tas_dev->cur_conf].mpName);
		tasdevice_select_tuningprm_cfg(tas_dev, tas_dev->cur_prog,
			tas_dev->cur_conf, profile_cfg_id);
	}

	tasdevice_select_cfg_blk(tas_dev, profile_cfg_id,
		TASDEVICE_BIN_BLK_PRE_POWER_UP);

	if (gpio_is_valid(tas_dev->irq_info.irq_gpio))
		tasdevice_enable_irq(tas_dev, true);
	mutex_unlock(&tas_dev->codec_lock);
out:
	dev_info(tas_dev->dev, "%s: leave\n", __func__);
}

void tasdevice_force_dsp_download(struct tasdevice_priv *tas_dev)
{
	struct tasdevice_t *tasdevice;
	int i;

	for (i = 0; i < tas_dev->ndev; i++) {
		tasdevice = &(tas_dev->tasdevice[i]);
		tasdevice->prg_download_cnt = 0;
		tasdevice->mnCurrentProgram = -1;
	}
}

static void tasdevice_set_power_state(
	struct tasdevice_priv *tas_dev, int state)
{
	switch (state) {
	case 0:
		schedule_delayed_work(&tas_dev->powercontrol_work,
			msecs_to_jiffies(20));
		break;
	default:
		if (!(tas_dev->pstream || tas_dev->cstream)) {
			if (gpio_is_valid(tas_dev->irq_info.irq_gpio))
				tasdevice_enable_irq(tas_dev, false);
			tasdevice_select_cfg_blk(tas_dev,
				tas_dev->mtRegbin.profile_cfg_id,
				TASDEVICE_BIN_BLK_PRE_SHUTDOWN);
			if (tas_dev->mtRegbin.profile_cfg_id ==
				TASDEVICE_CALIBRATION_PROFILE)
				tasdevice_force_dsp_download(tas_dev);
		}
		break;
	}
}

static int tasdevice_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	struct snd_soc_component *codec = dai->component;
	struct tasdevice_priv *tas_dev = snd_soc_component_get_drvdata(codec);

	/* Codec Lock Hold */
	mutex_lock(&tas_dev->codec_lock);
	/* misc driver file lock hold */
	mutex_lock(&tas_dev->file_lock);

	if (mute) {
		/* stop DSP only when both playback and capture streams
		* are deactivated
		*/
		if (stream == SNDRV_PCM_STREAM_PLAYBACK)
			tas_dev->pstream = 0;
		else
			tas_dev->cstream = 0;
		if (tas_dev->pstream != 0 || tas_dev->cstream != 0)
			goto out;
	} else
		if (stream == SNDRV_PCM_STREAM_PLAYBACK)
			tas_dev->pstream = 1;
		else
			tas_dev->cstream = 1;

	tasdevice_set_power_state(tas_dev, mute);
out:
	/* Misc driver file lock release */
	mutex_unlock(&tas_dev->file_lock);
	/* Codec Lock Release*/
	mutex_unlock(&tas_dev->codec_lock);

	return 0;
}

static struct snd_soc_dai_ops tasdevice_dai_ops = {
	.startup = tasdevice_startup,
	.hw_params	= tasdevice_hw_params,
	.set_sysclk	= tasdevice_set_dai_sysclk,
	.mute_stream = tasdevice_mute,
};


static struct snd_soc_dai_driver tasdevice_dai_driver[] = {
	{
		.name = "tasdevice_codec",
		.id = 0,
		.playback = {
			.stream_name	= "Playback",
			.channels_min   = 1,
			.channels_max   = 4,
			.rates	 = TASDEVICE_RATES,
			.formats	= TASDEVICE_FORMATS,
		},
		.capture = {
			.stream_name	= "Capture",
			.channels_min   = 1,
			.channels_max   = 4,
			.rates	 = TASDEVICE_RATES,
			.formats	= TASDEVICE_FORMATS,
		},
		.ops = &tasdevice_dai_ops,
		.symmetric_rates = 1,
	},
};

static int tasdevice_codec_probe(
	struct snd_soc_component *codec)
{
	struct tasdevice_priv *tas_priv =
		snd_soc_component_get_drvdata(codec);
	int ret = 0;

	dev_info(tas_priv->dev, "%s, enter\n", __func__);
	/* Codec Lock Hold */
	mutex_lock(&tas_priv->codec_lock);
	tas_priv->codec = codec;

	scnprintf(tas_priv->regbin_binaryname, 64, "%s-%uamp-reg.bin",
		tas_priv->dev_name, tas_priv->ndev);
	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
		tas_priv->regbin_binaryname, tas_priv->dev, GFP_KERNEL,
		tas_priv, tasdevice_regbin_ready);
	if (ret)
		dev_err(tas_priv->dev,
			"%s: request_firmware_nowait error:0x%08x\n",
			__func__, ret);

	crc8_populate_msb(tas_priv->crc8_lkp_tbl, TASDEVICE_CRC8_POLYNOMIAL);
	if (tas_priv->reset)
		tas_priv->hwreset(tas_priv);

	if (tas_priv->set_global_mode != NULL)
		tas_priv->set_global_mode(tas_priv);
	/* Codec Lock Release*/
	mutex_unlock(&tas_priv->codec_lock);
	dev_info(tas_priv->dev, "%s, codec probe success\n", __func__);

	return ret;
}

static void tasdevice_codec_remove(
	struct snd_soc_component *codec)
{
	struct tasdevice_priv *tas_dev =
		snd_soc_component_get_drvdata(codec);
	/* Codec Lock Hold */
	mutex_lock(&tas_dev->codec_lock);
	tasdevice_deinit(tas_dev);
	/* Codec Lock Release*/
	mutex_unlock(&tas_dev->codec_lock);

	return;

}

static const struct snd_soc_component_driver
	soc_codec_driver_tasdevice = {
	.probe			= tasdevice_codec_probe,
	.remove			= tasdevice_codec_remove,
	.dapm_widgets		= tasdevice_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(tasdevice_dapm_widgets),
	.dapm_routes		= tasdevice_audio_map,
	.num_dapm_routes	= ARRAY_SIZE(tasdevice_audio_map),
	.idle_bias_on		= 1,
	.endianness		= 1,
};

static int tas2563_digital_getvol(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec
		= snd_soc_kcontrol_component(kcontrol);
	struct tasdevice_priv *tas_dev =
		snd_soc_component_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned short vol, max_vol = mc->max;
	unsigned char data[4];
	unsigned int tmp;
	int ret = 0;

	/* Read the primary device as the whole */
	ret = tasdevice_dev_bulk_read(tas_dev, 0, mc->reg, data, 4);
	if (ret) {
		dev_err(tas_dev->dev,
		"%s, get digital vol error\n",
		__func__);
		goto out;
	}
	tmp = be32_to_cpup((__be32 *)data);
	vol = tmp >> 16;
	vol = mc->invert ? max_vol - vol : vol;
	ucontrol->value.integer.value[0] = vol;

out:
	return ret;
}

static int tas2563_digital_putvol(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_component *codec =
		snd_soc_kcontrol_component(kcontrol);
	struct tasdevice_priv *tas_dev =
		snd_soc_component_get_drvdata(codec);
	unsigned short val;
	unsigned int vol;
	unsigned char *p;
	int i, ret = 0;
	__be32 mackey;

	val = ucontrol->value.integer.value[0];
	vol = (val << 16) | 0xFFFF;
	mackey = cpu_to_be32p((const unsigned int *) &vol);
	p = (unsigned char *)&mackey;
	p[2] = 0xFF;
	p[3] = 0xff;

	if (tas_dev->set_global_mode != NULL) {
		ret = tasdevice_dev_bulk_write(tas_dev, tas_dev->ndev,
			mc->reg, p, 4);
		if (ret)
			dev_err(tas_dev->dev,
				"%s, set digital vol error in global mode\n",
				__func__);
		goto out;
	}

	for (i = 0; i < tas_dev->ndev; i++) {
		ret = tasdevice_dev_bulk_write(tas_dev, i,
			mc->reg, p, 4);
		if (ret)
			dev_err(tas_dev->dev,
				"%s, set digital vol error in device %d\n",
				__func__, i);
	}

out:
	return ret;
}

static int tasdevice_digital_getvol(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct tasdevice_priv *tas_dev = snd_soc_component_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int val;
	int ret = 0;

	/* Read the primary device as the whole */
	ret = tasdevice_dev_read(tas_dev, 0, mc->reg, &val);
	if (ret) {
		dev_err(tas_dev->dev, "%s, get digital vol error\n", __func__);
		goto out;
	}
	val = (val > mc->max) ? mc->max : val;
	val = mc->invert ? mc->max - val : val;
	ucontrol->value.integer.value[0] = val;

out:
	return ret;
}

static int tasdevice_digital_putvol(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct tasdevice_priv *tas_dev = snd_soc_component_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int val;
	int i, ret = 0;

	val = ucontrol->value.integer.value[0];
	val = (val > mc->max) ? mc->max : val;
	val = mc->invert ? mc->max - val : val;
	val = (val < 0) ? 0 : val;
	if (tas_dev->set_global_mode != NULL) {
		ret = tasdevice_dev_write(tas_dev, tas_dev->ndev, mc->reg, val);
		if (ret)
			dev_err(tas_dev->dev, "%s, set digital vol error in global mode\n",
				__func__);
	} else {
		for (i = 0; i < tas_dev->ndev; i++) {
			ret = tasdevice_dev_write(tas_dev, i, mc->reg, val);
			if (ret)
				dev_err(tas_dev->dev,
					"%s, set digital vol error in device %d\n", __func__, i);
		}
	}

	return ret;
}

static int tasdevice_amp_getvol(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct tasdevice_priv *tas_dev = snd_soc_component_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned char mask = 0;
	unsigned int val;
	int ret = 0;

	/* Read the primary device */
	ret = tasdevice_dev_read(tas_dev, 0, mc->reg, &val);
	if (ret) {
		dev_err(tas_dev->dev, "%s, get AMP vol error\n", __func__);
		goto out;
	}

	mask = (1 << fls(mc->max)) - 1;
	mask <<= mc->shift;
	val = (val & mask) >> mc->shift;
	val = (val > mc->max) ? mc->max : val;
	val = mc->invert ? mc->max - val : val;
	ucontrol->value.integer.value[0] = val;

out:
	return ret;
}

static int tasdevice_amp_putvol(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct tasdevice_priv *tas_dev = snd_soc_component_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned char mask = 0;
	unsigned int val;
	int i, ret = 0;

	mask = (1 << fls(mc->max)) - 1;
	mask <<= mc->shift;
	val = ucontrol->value.integer.value[0];
	val = (val > mc->max) ? mc->max : val;
	val = mc->invert ? mc->max - val : val;
	val = (val < 0) ? 0 : val;

	for (i = 0; i < tas_dev->ndev; i++) {
		ret = tasdevice_dev_update_bits(tas_dev, i, mc->reg, mask,
			val << mc->shift);
		if (ret)
			dev_err(tas_dev->dev, "%s, set AMP vol error in device %d\n",
				__func__, i);
	}

	return ret;
}

static const DECLARE_TLV_DB_SCALE(tas2563_amp_vol_tlv, 800, 50, 0);
static const DECLARE_TLV_DB_SCALE(tas2563_dvc_tlv, -9633, 1, 1);
static const DECLARE_TLV_DB_SCALE(tas2781_dvc_tlv, -10000, 100, 1);
static const DECLARE_TLV_DB_SCALE(tas2781_amp_vol_tlv, 1100, 50, 0);

static const struct snd_kcontrol_new tas2563_snd_controls[] = {
	SOC_SINGLE_RANGE_EXT_TLV("tas2563-amp-gain-volume", TAS2781_AMP_LEVEL,
		1, 0, 0x1C, 0, tasdevice_amp_getvol, tasdevice_amp_putvol,
		tas2563_amp_vol_tlv),
	SOC_SINGLE_RANGE_EXT_TLV("tas2563-digital-volume", TAS2563_DVC_LVL,
		0, 0, 0xFFFF, 0, tas2563_digital_getvol, tas2563_digital_putvol,
		tas2563_dvc_tlv),
};

static const struct snd_kcontrol_new tas2781_snd_controls[] = {
	SOC_SINGLE_RANGE_EXT_TLV("tas2781-amp-gain-volume", TAS2781_AMP_LEVEL,
		1, 0, 0x14, 0, tasdevice_amp_getvol, tasdevice_amp_putvol,
		tas2781_amp_vol_tlv),
	SOC_SINGLE_RANGE_EXT_TLV("tas2781-digital-volume", TAS2781_DVC_LVL,
		0, 0, 200, 1, tasdevice_digital_getvol, tasdevice_digital_putvol,
		tas2781_dvc_tlv),
};

static int tasdevice_info_rotation(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	struct snd_soc_component *codec
		= snd_soc_kcontrol_component(kcontrol);
	struct tasdevice_priv *tas_priv =
		snd_soc_component_get_drvdata(codec);

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	/* Codec Lock Hold*/
	mutex_lock(&tas_priv->codec_lock);
	uinfo->count = 1;
	/* Codec Lock Release*/
	mutex_unlock(&tas_priv->codec_lock);

	uinfo->value.integer.min = tas_priv->mtRegbin.direct_rotation_cfg_id;
	uinfo->value.integer.max = tas_priv->mtRegbin.direct_rotation_cfg_id
		+ tas_priv->mtRegbin.direct_rotation_cfg_total - 1;

	return 0;
}

static int tasdevice_set_rotation_id(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec
		= snd_soc_kcontrol_component(kcontrol);
	struct tasdevice_priv *tas_priv =
		snd_soc_component_get_drvdata(codec);
	int max_val = tas_priv->mtRegbin.direct_rotation_cfg_id
		+ tas_priv->mtRegbin.direct_rotation_cfg_total - 1;
	int min_val = tas_priv->mtRegbin.direct_rotation_cfg_id;
	int val = ucontrol->value.integer.value[0];

	tas_priv->mtRegbin.rotation_id = clamp(val, min_val, max_val);
	/* Codec Lock Hold*/
	mutex_lock(&tas_priv->codec_lock);
	tasdevice_select_cfg_blk(tas_priv, val, TASDEVICE_BIN_BLK_PRE_POWER_UP);
	/* Codec Lock Release*/
	mutex_unlock(&tas_priv->codec_lock);

	return 1;
}

static int tasdevice_get_rotation_id(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec
		= snd_soc_kcontrol_component(kcontrol);
	struct tasdevice_priv *tas_priv
		= snd_soc_component_get_drvdata(codec);

	/* Codec Lock Hold*/
	mutex_lock(&tas_priv->codec_lock);

	ucontrol->value.integer.value[0] = tas_priv->mtRegbin.rotation_id;

	/* Codec Lock Release*/
	mutex_unlock(&tas_priv->codec_lock);

	return 0;
}

static int tasdevice_info_profile(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	struct snd_soc_component *codec
		= snd_soc_kcontrol_component(kcontrol);
	struct tasdevice_priv *tas_priv =
		snd_soc_component_get_drvdata(codec);

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	/* Codec Lock Hold*/
	mutex_lock(&tas_priv->codec_lock);
	uinfo->count = 1;
	/* Codec Lock Release*/
	mutex_unlock(&tas_priv->codec_lock);

	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = tas_priv->mtRegbin.ncfgs - 1;

	return 0;
}

static int tasdevice_get_profile_id(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec
		= snd_soc_kcontrol_component(kcontrol);
	struct tasdevice_priv *tas_priv
		= snd_soc_component_get_drvdata(codec);
	int val = tas_priv->mtRegbin.profile_cfg_id;
	int max_val = tas_priv->mtRegbin.ncfgs - 1;
	int ret = 0;

	val = clamp(val, 0, max_val);
	/* Codec Lock Hold*/
	mutex_lock(&tas_priv->codec_lock);
	if (ucontrol->value.integer.value[0] != val) {
		ucontrol->value.integer.value[0] = val;
		ret = 1;
	}
	/* Codec Lock Release*/
	mutex_unlock(&tas_priv->codec_lock);

	return ret;
}

static int tasdevice_set_profile_id(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec
		= snd_soc_kcontrol_component(kcontrol);
	struct tasdevice_priv *tas_priv =
		snd_soc_component_get_drvdata(codec);
	int max_val = tas_priv->mtRegbin.ncfgs - 1;
	int val = ucontrol->value.integer.value[0];
	int ret = 0;

	val = clamp(val, 0, max_val);
	/* Codec Lock Hold*/
	mutex_lock(&tas_priv->codec_lock);
	if (tas_priv->mtRegbin.profile_cfg_id != val) {
		tas_priv->mtRegbin.profile_cfg_id = val;
		ret = 1;
	}
	/* Codec Lock Release*/
	mutex_unlock(&tas_priv->codec_lock);

	return ret;
}

int tasdevice_create_controls(struct tasdevice_priv *tas_priv)
{
	struct snd_kcontrol_new *tasdevice_profile_controls = NULL;
	int  ret = 0, mix_index = 0;
	char *name;

	tasdevice_profile_controls = devm_kzalloc(tas_priv->dev,
		sizeof(tasdevice_profile_controls[0]), GFP_KERNEL);
	if (tasdevice_profile_controls == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	/* Create a mixer item for selecting the active profile */
	name = devm_kzalloc(tas_priv->dev,
		SNDRV_CTL_ELEM_ID_NAME_MAXLEN, GFP_KERNEL);
	if (!name) {
		ret = -ENOMEM;
		goto out;
	}
	scnprintf(name, SNDRV_CTL_ELEM_ID_NAME_MAXLEN, "TASDEVICE Profile id");
	tasdevice_profile_controls[0].name = name;
	tasdevice_profile_controls[0].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	tasdevice_profile_controls[0].info = tasdevice_info_profile;
	tasdevice_profile_controls[0].get = tasdevice_get_profile_id;
	tasdevice_profile_controls[0].put = tasdevice_set_profile_id;

	ret = snd_soc_add_component_controls(tas_priv->codec,
 		tasdevice_profile_controls, 1);
	if (ret < 0) {
		dev_err(tas_priv->dev, "%s, add regbin ctrl failed\n",
			__func__);
		ret = -ENXIO;
		goto out;
	}
	tas_priv->tas_ctrl.nr_controls++;

	switch (tas_priv->chip_id) {
	case TAS2563:
		mix_index = ARRAY_SIZE(tas2563_snd_controls);
		tasdevice_profile_controls =
			(struct snd_kcontrol_new *)tas2563_snd_controls;
		break;
	case TAS2781:
		mix_index = ARRAY_SIZE(tas2781_snd_controls);
		tasdevice_profile_controls =
			(struct snd_kcontrol_new *)tas2781_snd_controls;
		break;
	}

	if (!mix_index) {
		dev_err(tas_priv->dev, "%s, chip invalid\n",
			__func__);
		goto out;
	}
	ret = snd_soc_add_component_controls(tas_priv->codec,
		tasdevice_profile_controls, mix_index);
	if(ret < 0) {
		dev_err(tas_priv->dev, "%s, add vol ctrl failed\n",
			__func__);
		goto out;
	}
 	tas_priv->tas_ctrl.nr_controls += mix_index;

	if (!tas_priv->mtRegbin.direct_rotation_cfg_total)
		goto out;
	tasdevice_profile_controls = devm_kzalloc(tas_priv->dev,
		sizeof(tasdevice_profile_controls[0]), GFP_KERNEL);
	if (tasdevice_profile_controls == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	/* Create a mixer item for selecting the active profile */
	name = devm_kzalloc(tas_priv->dev,
		SNDRV_CTL_ELEM_ID_NAME_MAXLEN, GFP_KERNEL);
	if (!name) {
		ret = -ENOMEM;
		goto out;
	}

	scnprintf(name, SNDRV_CTL_ELEM_ID_NAME_MAXLEN, "TASDEVICE Rotation id");
	tasdevice_profile_controls[0].name = name;
	tasdevice_profile_controls[0].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	tasdevice_profile_controls[0].info = tasdevice_info_rotation;
	tasdevice_profile_controls[0].put = tasdevice_set_rotation_id;
	tasdevice_profile_controls[0].get = tasdevice_get_rotation_id;

	ret = snd_soc_add_component_controls(tas_priv->codec,
 		tasdevice_profile_controls, 1);
	if (ret < 0) {
		dev_err(tas_priv->dev, "%s, add regbin ctrl failed\n",
			__func__);
		goto out;
	}
	tas_priv->tas_ctrl.nr_controls++;
out:
	return ret;
}

static int tasdevice_info_programs(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	struct snd_soc_component *codec
		= snd_soc_kcontrol_component(kcontrol);
	struct tasdevice_priv *tas_priv =
		snd_soc_component_get_drvdata(codec);
	struct tasdevice_fw *Tfw = tas_priv->fmw;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	/* Codec Lock Hold*/
	mutex_lock(&tas_priv->codec_lock);
	uinfo->count = 1;
	/* Codec Lock Release*/
	mutex_unlock(&tas_priv->codec_lock);

	/* 0:		 dsp mode
	 * non-zero:	bypass mode
	 */
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = (int)Tfw->nr_programs;

	return 0;
}

static int tasdevice_info_configurations(
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	struct snd_soc_component *codec
		= snd_soc_kcontrol_component(kcontrol);
	struct tasdevice_priv *tas_priv =
		snd_soc_component_get_drvdata(codec);
	struct tasdevice_fw *Tfw = tas_priv->fmw;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	/* Codec Lock Hold*/
	mutex_lock(&tas_priv->codec_lock);
	uinfo->count = 1;

	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = (int)Tfw->nr_configurations - 1;

	/* Codec Lock Release*/
	mutex_unlock(&tas_priv->codec_lock);

	return 0;
}

int tasdevice_dsp_create_control(struct tasdevice_priv *tas_priv)
{
	int  nr_controls = 2, ret = 0, mix_index = 0;
	char *program_name = NULL;
	char *configuraton_name = NULL;
	struct snd_kcontrol_new *tasdevice_dsp_controls = NULL;

	tasdevice_dsp_controls = devm_kzalloc(tas_priv->dev,
			nr_controls * sizeof(tasdevice_dsp_controls[0]),
			GFP_KERNEL);
	if (tasdevice_dsp_controls == NULL) {
		dev_err(tas_priv->dev, "%s, allocate mem failed\n",
			__func__);
		ret = -ENOMEM;
		goto out;
	}

	/* Create a mixer item for selecting the active profile */
	program_name = devm_kzalloc(tas_priv->dev,
		SNDRV_CTL_ELEM_ID_NAME_MAXLEN, GFP_KERNEL);
	configuraton_name = devm_kzalloc(tas_priv->dev,
		SNDRV_CTL_ELEM_ID_NAME_MAXLEN, GFP_KERNEL);
	if (!program_name || !configuraton_name) {
		dev_err(tas_priv->dev, "%s, allocate pro or conf failed\n",
			__func__);
		ret = -ENOMEM;
		goto out;
	}

	scnprintf(program_name, SNDRV_CTL_ELEM_ID_NAME_MAXLEN, "Program");
	tasdevice_dsp_controls[mix_index].name = program_name;
	tasdevice_dsp_controls[mix_index].iface =
		SNDRV_CTL_ELEM_IFACE_MIXER;
	tasdevice_dsp_controls[mix_index].info =
		tasdevice_info_programs;
	tasdevice_dsp_controls[mix_index].get =
		tasdevice_program_get;
	tasdevice_dsp_controls[mix_index].put =
		tasdevice_program_put;
	mix_index++;

	scnprintf(configuraton_name, SNDRV_CTL_ELEM_ID_NAME_MAXLEN,
		"Configuration");
	tasdevice_dsp_controls[mix_index].name = configuraton_name;
	tasdevice_dsp_controls[mix_index].iface =
		SNDRV_CTL_ELEM_IFACE_MIXER;
	tasdevice_dsp_controls[mix_index].info =
		tasdevice_info_configurations;
	tasdevice_dsp_controls[mix_index].get =
		tasdevice_configuration_get;
	tasdevice_dsp_controls[mix_index].put =
		tasdevice_configuration_put;
	mix_index++;

	ret = snd_soc_add_component_controls(tas_priv->codec,
		tasdevice_dsp_controls,
		nr_controls < mix_index ? nr_controls : mix_index);
	if(ret < 0) {
		dev_err(tas_priv->dev, "%s, add dsp ctrl failed\n",
			__func__);
		goto out;
	}
	tas_priv->tas_ctrl.nr_controls += nr_controls;
out:
	return ret;
}

int tasdevice_register_codec(struct tasdevice_priv *tas_priv)
{
	int nResult = 0;

	dev_err(tas_priv->dev, "%s, enter\n", __func__);
	nResult = devm_snd_soc_register_component(tas_priv->dev,
		&soc_codec_driver_tasdevice,
		tasdevice_dai_driver, ARRAY_SIZE(tasdevice_dai_driver));

	return nResult;
}

void tasdevice_deregister_codec(struct tasdevice_priv *tas_priv)
{
	snd_soc_unregister_component(tas_priv->dev);
}
