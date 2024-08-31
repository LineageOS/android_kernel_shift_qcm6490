#ifndef __DRV2624_INPUT_H__
#define __DRV2624_INPUT_H__

#define DRV2624_EFFECT_MAX_NUM		(32)

void drv2624_close(struct input_dev *input);
int drv2624_haptics_upload_effect(struct input_dev *dev,
	struct ff_effect *effect, struct ff_effect *old);
int drv2624_haptics_playback(struct input_dev *dev,
	int effect_id, int val);
int drv2624_haptics_erase(struct input_dev *dev,
	int effect_id);
void drv2624_haptics_set_gain(struct input_dev *dev,
	u16 gain);
void drv2624_rtp_load(const struct firmware *pFW,
	struct drv2624_data *pContext);
void drv26xx_rtp_waveform_remove(
	struct drv2624_data* pDRV26xx);
#endif