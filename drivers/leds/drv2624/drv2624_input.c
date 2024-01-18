#include <linux/firmware.h>
#include <linux/slab.h>

#include "drv2624.h"
#ifndef LEDS_ARCH
void drv2624_close(struct input_dev *input)
{
	struct drv2624_data *pDRV2624 = input_get_drvdata(input);
	mutex_lock(&pDRV2624->haptic_lock);
	drv2624_haptic_play_go(pDRV2624, false);
	mutex_unlock(&pDRV2624->haptic_lock);
}
int drv2624_haptics_upload_effect(struct input_dev *dev,
	struct ff_effect *effect, struct ff_effect *old)
{
	switch (effect->type){
	case FF_CONSTANT:
		break;
	case FF_PERIODIC:
		break;
	default :
		break;
	}
	return 0;
}
int drv2624_haptics_playback(struct input_dev *dev,
	int effect_id, int val)
{
	struct drv2624_data *pDRV2624 = input_get_drvdata(dev);

	dev_info(pDRV2624->dev, "%s:%u: effect_id(%d)\n",
		__func__, __LINE__, effect_id);

	hrtimer_cancel(&pDRV2624->haptics_timer);
	mutex_lock(&pDRV2624->haptic_lock);
	pDRV2624->state = 1;
	pDRV2624->waveform_id = effect_id;
	mutex_unlock(&pDRV2624->haptic_lock);
	schedule_work(&pDRV2624->vibrator_work);
	return 0;
}

int drv2624_haptics_erase(struct input_dev *dev,
	int effect_id)
{
	struct drv2624_data *pDRV2624 = input_get_drvdata(dev);
	dev_info(pDRV2624->dev, "%s:%u: effect_id(%d)\n",
		__func__, __LINE__, effect_id);
	mutex_lock(&pDRV2624->haptic_lock);
	drv2624_haptic_play_go(pDRV2624, false);
	mutex_unlock(&pDRV2624->haptic_lock);
	return 0;
}

void drv2624_haptics_set_gain(struct input_dev *dev,
	u16 gain)
{

}

void drv2624_rtp_load(const struct firmware *pFW,
	struct drv2624_data *pDRV26xx)
{
	struct drv26xx_RTPwaveforms* wvfm = NULL;
	struct drv26xx_RTPwaveform_info** rtp_wvfm_info = NULL;
	unsigned char* buf = (unsigned char*)pFW->data;
	int cur = 0;
	unsigned short i = 0;

	if (!pFW || !pFW->data || !pDRV26xx) {
		dev_err(pDRV26xx->dev,"%s:%u:Failed to read firmware\n",
			__func__, __LINE__);
		return;
	}
	wvfm = &(pDRV26xx->rtpwvfm);
	if (cur + 4 > pFW->size) {
		dev_err(pDRV26xx->dev,"%s:%u:bin file error!\n",
			__func__, __LINE__);
		goto EXIT;
	}

	wvfm->filetype = SMS_HTONL(buf[cur], buf[cur + 1], buf[cur + 2],
		buf[cur + 3]);
	if (wvfm->filetype != RTP_ID) {
		wvfm->version = 0;
		dev_err(pDRV26xx->dev,"Wrong bin file\n");
		goto EXIT;
	}
	else {
		cur += 4;
		if (cur + 2 > pFW->size) {
			dev_err(pDRV26xx->dev,"%s:%u:bin file error!\n",
				__func__, __LINE__);
			goto EXIT;
		}
		wvfm->version = SMS_HTONS(buf[cur+1], buf[cur]);
		if(wvfm->version != 4) {
			dev_err(pDRV26xx->dev,
				"%s:%u:Error Ver:%u, correct is 4!\n",
				__func__, __LINE__, wvfm->version);
			goto EXIT;
		}
		cur += 2;
	}

	if (cur + 2 > pFW->size) {
		dev_err(pDRV26xx->dev,"%s:%u:bin file error!\n",
			__func__, __LINE__);
		goto EXIT;
	}

	wvfm->mnSampleTimeRegbits =
		drv_get_bf_value(DRV26XX_BF_SAMPLE_TIME, buf[cur]);
	switch (wvfm->mnSampleTimeRegbits) {
		case 0:
			wvfm->sample_time = 150;
			break;
		case 1:
			wvfm->sample_time = 200;
			wvfm->mnSampleTimeRegbits = 4;
			break;
		case 2:
			wvfm->sample_time = 250;
			wvfm->mnSampleTimeRegbits = 8;
			break;
		default:
			wvfm->sample_time = 300;
			wvfm->mnSampleTimeRegbits = 0xc;
			break;
	}
	cur++;

	wvfm->fb_brake_factor =
		drv_get_bf_value(DRV26XX_BF_FB_BRAKE_FACTOR, buf[cur]);
	wvfm->hybrid_loop =
		drv_get_bf_value(DRV26XX_BF_HYBRID_LOOP, buf[cur]);
	wvfm->auto_brake =
		drv_get_bf_value(DRV26XX_BF_AUTO_BRAKE, buf[cur]);
	wvfm->auto_brake_standby =
		drv_get_bf_value(DRV26XX_BF_AUTO_BRAKE_STANDBY, buf[cur]);
	cur++;

	if (cur + 2 > pFW->size) {
		dev_err(pDRV26xx->dev,"%s:%u:bin file error!\n",
			__func__, __LINE__);
		goto EXIT;
	}

	wvfm->rated_Voltage = SMS_HTONS(buf[cur+1], buf[cur]);
	cur += 2;
	if (cur + 2 > pFW->size) {
		dev_err(pDRV26xx->dev,"%s:%u:bin file error!\n",
			__func__, __LINE__);
		goto EXIT;
	}
	wvfm->overDrive_Voltage = SMS_HTONS(buf[cur+1], buf[cur]);
	cur += 2;
	if (cur + 2 > pFW->size) {
		dev_err(pDRV26xx->dev,"%s:%u:bin file error!\n",
			__func__, __LINE__);
		goto EXIT;
	}
	wvfm->lra_f0 = SMS_HTONS(buf[cur+1], buf[cur]);
	cur += 2;
	if (cur + 2 > pFW->size) {
		dev_err(pDRV26xx->dev,"%s:%u:bin file error!\n",
			__func__, __LINE__);
		goto EXIT;
	}
	wvfm->nWaveforms = SMS_HTONS(buf[cur + 1], buf[cur]);;
	cur += 2;

	wvfm->mnCLRatedVoltageRegbits =
		wvfm->rated_Voltage * int_sqrt(1000000-
		(4*wvfm->sample_time+300)*wvfm->lra_f0)/20580;
	wvfm->mnCLOverDriveClampVoltageRegbits =
		(wvfm->overDrive_Voltage * 100) / 2122;
	wvfm->mnOLOverDriveClampVoltageRegbits =
		(wvfm->overDrive_Voltage * 10000) /
		(2132*int_sqrt(10000-8*wvfm->lra_f0));

	dev_info(pDRV26xx->dev, "version = 0x%02x\n", wvfm->version);
	dev_info(pDRV26xx->dev, "fb_brake_factor = 0x%02x\n",
		wvfm->fb_brake_factor);
	dev_info(pDRV26xx->dev, "hybrid_loop = 0x%02x:%s\n",
		wvfm->hybrid_loop,
		(wvfm->hybrid_loop == 1) ? "Enable" : "Disable");
	dev_info(pDRV26xx->dev, "auto_brake = 0x%02x:%s\n",
		wvfm->auto_brake,
		(wvfm->auto_brake == 1) ? "Enable" : "Disable");
	dev_info(pDRV26xx->dev, "auto_brake_standby = 0x%02x:%s\n",
		wvfm->auto_brake_standby,
		(wvfm->auto_brake_standby == 1) ? "Enable" : "Disable");
	dev_info(pDRV26xx->dev, "rated_Voltage = %u mV\n",
		wvfm->rated_Voltage);
	dev_info(pDRV26xx->dev, "overDrive_Voltage = %u mV\n",
		wvfm->overDrive_Voltage);
	dev_info(pDRV26xx->dev, "lra_f0 = %u Hz\n", wvfm->lra_f0);
	dev_info(pDRV26xx->dev, "nWaveforms = %u\n", wvfm->nWaveforms);

	rtp_wvfm_info =
		(struct drv26xx_RTPwaveform_info **)kzalloc(wvfm->nWaveforms,
		sizeof(struct drv26xx_RTPwaveform_info *));

	if (NULL == rtp_wvfm_info) {
		dev_err(pDRV26xx->dev, "%s:%u:wvfm %u kalloc error!\n",
			__func__, __LINE__, i);
		wvfm->nWaveforms = 0;
		goto EXIT;
	}

	for (i = 0; i < wvfm->nWaveforms; i++) {
		unsigned short offset = 0;
		if (cur + 8 > pFW->size) {
			dev_err(pDRV26xx->dev, "%s:%u:wvfm %u bin file error!\n",
				__func__, __LINE__, i);
			break;
		}
		rtp_wvfm_info[i] = (struct drv26xx_RTPwaveform_info*)
			kzalloc(1, sizeof(struct drv26xx_RTPwaveform_info));
		if(NULL == rtp_wvfm_info[i]) {
			dev_err(pDRV26xx->dev,"%s:%u:wvfm %u calloc error!\n",
				__func__, __LINE__, i);
				wvfm->nWaveforms = (i > 0) ? i - 1 : 0;
			break;
		}
		offset = SMS_HTONS(buf[cur+1], buf[cur]);
		cur += 2;
		rtp_wvfm_info[i]->length = SMS_HTONS(buf[cur+1], buf[cur]);
		cur += 2;
		rtp_wvfm_info[i]->duration = SMS_HTONS(buf[cur+1], buf[cur]);
		cur+=2;
		rtp_wvfm_info[i]->brake =
			drv_get_bf_value(DRV26XX_BF_WVFM_BRAKE, buf[cur]);
		rtp_wvfm_info[i]->loop_mod =
			drv_get_bf_value(DRV26XX_BF_WVFM_LOOP_MOD, buf[cur]);
		rtp_wvfm_info[i]->wv_shape =
			drv_get_bf_value(DRV26XX_BF_WVFM_WV_SHAPE, buf[cur]);
		rtp_wvfm_info[i]->actuator_type =
			drv_get_bf_value(DRV26XX_BF_WVFM_ACTUATOR_TYPE, buf[cur]);
		cur += 2;
		rtp_wvfm_info[i]->lra_f0 = SMS_HTONS(buf[cur + 1], buf[cur]);
		cur += 2;

		if(rtp_wvfm_info[i]->actuator_type == 0) {
			rtp_wvfm_info[i]->mnCLRatedVoltageRegbits =
				wvfm->rated_Voltage * int_sqrt(1000000-(4*
				wvfm->sample_time+300)*rtp_wvfm_info[i]->lra_f0)/20580;
			rtp_wvfm_info[i]->mnCLOverDriveClampVoltageRegbits =
				(wvfm->overDrive_Voltage * 100) / 2122;
			rtp_wvfm_info[i]->mnOLOverDriveClampVoltageRegbits =
				(wvfm->overDrive_Voltage * 10000) /
				(2132*int_sqrt(10000-8*rtp_wvfm_info[i]->lra_f0));
		} else {
			rtp_wvfm_info[i]->mnCLRatedVoltageRegbits =
				wvfm->rated_Voltage *100 / 2188;
			rtp_wvfm_info[i]->mnCLOverDriveClampVoltageRegbits =
				wvfm->overDrive_Voltage*(4200 + 150)/(2164*42 - 6492);
			rtp_wvfm_info[i]->mnOLOverDriveClampVoltageRegbits =
				wvfm->overDrive_Voltage*100/2159;
		}
#if 1
		dev_info(pDRV26xx->dev,
			"wvfm = %u dur = %u commands number = %u "
			"f0 = %u\n", i, rtp_wvfm_info[i]->duration,
			rtp_wvfm_info[i]->length, rtp_wvfm_info[i]->lra_f0);
		dev_info(pDRV26xx->dev,
			"rtp_wvfm_info[%u]->brake = 0x%02x:%s\n",
			i, rtp_wvfm_info[i]->brake,
			(rtp_wvfm_info[i]->brake == 1) ? "Enable" : "Disable");
		dev_info(pDRV26xx->dev,
			"rtp_wvfm_info[%u]->loop_mod = 0x%02x:%s\n",
			i, rtp_wvfm_info[i]->loop_mod,
			(rtp_wvfm_info[i]->loop_mod != 1) ?  "Close": "Open");
		dev_info(pDRV26xx->dev,
			"rtp_wvfm_info[%u]->wv_shape = 0x%02x:%s\n",
			i, rtp_wvfm_info[i]->wv_shape,
			(rtp_wvfm_info[i]->wv_shape != 1) ? "Square" : "Sine");
		dev_info(pDRV26xx->dev, "rtp_wvfm_info[%u]->actuator_type = "
			"0x%02x:%s\n", i, rtp_wvfm_info[i]->actuator_type,
			(rtp_wvfm_info[i]->actuator_type == 0) ? "LRA" : "ERM");
#endif
		if (offset + rtp_wvfm_info[i]->length*2 > pFW->size) {
			dev_err(pDRV26xx->dev, "%s:%u:wvfm %u bin file error!\n",
				__func__, __LINE__, i);
			dev_err(pDRV26xx->dev,
				"pFW->size = %ld offset = %u len *2 = %u!\n",
				pFW->size, offset, rtp_wvfm_info[i]->length*2);
			rtp_wvfm_info[i]->length = 0;
			wvfm->nWaveforms = i;
			break;
		}
		rtp_wvfm_info[i]->wvfm_cmds =
			(struct wv_cmds *)kzalloc(rtp_wvfm_info[i]->length,
			sizeof(struct wv_cmds));
		if (rtp_wvfm_info[i]->wvfm_cmds == NULL) {
			dev_err(pDRV26xx->dev,
				"%s:%u:wvfm_cmds %u calloc error!\n",
				__func__, __LINE__, i);
			rtp_wvfm_info[i]->length = 0;
			break;
		}
		memcpy(rtp_wvfm_info[i]->wvfm_cmds, &buf[offset],
			rtp_wvfm_info[i]->length* sizeof(struct wv_cmds));
	}

	wvfm->rtp_wvfm_info = rtp_wvfm_info;
EXIT:
	return;
}

void drv26xx_rtp_waveform_remove(
	struct drv2624_data* pDRV26xx)
{
	int i = 0;
	struct drv26xx_RTPwaveform_info** rtp_wvfm_info =
		pDRV26xx->rtpwvfm.rtp_wvfm_info;
	if (rtp_wvfm_info) {
		for (i = 0; i < pDRV26xx->rtpwvfm.nWaveforms; i++) {
			if (rtp_wvfm_info[i] != NULL) {
				if (rtp_wvfm_info[i]->wvfm_cmds != NULL)
					kfree(rtp_wvfm_info[i]->wvfm_cmds);
				kfree(rtp_wvfm_info[i]);
			}
		}
		kfree(rtp_wvfm_info);
	}
}
#endif
