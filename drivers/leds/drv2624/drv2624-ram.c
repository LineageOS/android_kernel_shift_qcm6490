/* *******************************************************************
 *       Filename:  drv2624-ram.c
 *    Description:
 *        Version:  1.0
 *        Created:  01/28/21 18:55:42
 *       Revision:  none
 *       Compiler:  gcc
 *         Author:
 *        Company:  Texas Instruments Inc.
 * ******************************************************************/

#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/regmap.h>

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include "drv2624.h"
#include <linux/uaccess.h>

#define LRA_PERIOD_SCALE_NS 		(24390)
#define LRA_PERIOD_SCALE_UNIT		(1000000000)
/* 24.615us * 2000 = 49230ns*/
#define OL_LRA_PERIOD_SCALE_NSX2	(49230)
#define OL_LRA_PERIOD_SCALE_NS		(24615)
#define OL_LRA_PERIOD_FACTOR		(2)
#define OL_LRA_PERIOD_SCALE_UNIT	(2000000000)
#define FRES_FACTOR					(7)

/* Kernel parameter which is got from UEFI to */
/*        distingusih different haptic        */
static char haptic_mode[4];
static char gSysfsCmdLog[256];
static bool g_logEnable = false;
static struct drv2624_data *g_DRV2625data = NULL;

static int drv2624_stop(struct drv2624_data *pDRV2624);
static int drv2624_rtpvibrate(struct drv2624_data *pDRV26xx, unsigned int waveform_id);
module_param_string(haptic, haptic_mode,
	sizeof(haptic_mode), 0600);

static bool drv26xx_volatile(struct device *dev,
	unsigned int reg)
{
	return true;
}

static bool drv26xx_writeable(struct device *dev,
unsigned int reg)
{
	return true;
}

static struct regmap_config drv2624_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = drv26xx_writeable,
	.volatile_reg = drv26xx_volatile,
	.cache_type = REGCACHE_NONE,
	.max_register = 256,
};

static int drv2624_reg_read(struct drv2624_data *pDRV2624,
	unsigned char reg)
{
	unsigned int val;
	int nResult = -1;
	int retry_count = DRV26XX_I2C_RETRY_COUNT;

	while (retry_count--) {
		mutex_lock(&pDRV2624->reg_lock);
		nResult = regmap_read(pDRV2624->mpRegmap, reg, &val);
		mutex_unlock(&pDRV2624->reg_lock);
		if (nResult >= 0) {
			dev_info(pDRV2624->dev, "%s:%u: Reg[0x%02x]=0x%02x\n",
				__func__, __LINE__, reg, val);
			return val;
		} else {
			if(retry_count != 0) msleep(10);
		}
	}
	if(nResult < 0) dev_err(pDRV2624->dev, "%s:%u: I2C error %d\n",
		__func__, __LINE__, nResult);
	return nResult;
}

static int drv2624_reg_write(struct drv2624_data *pDRV2624,
			     unsigned char reg, unsigned char val)
{
	int nResult = -1;
	int retry_count = DRV26XX_I2C_RETRY_COUNT;
	dev_info(pDRV2624->dev, "%s:%u: Reg[0x%02x]=0x%02x\n",
		__func__, __LINE__, reg, val);
	while (retry_count--) {
		mutex_lock(&pDRV2624->reg_lock);
		nResult = regmap_write(pDRV2624->mpRegmap, reg, val);
		mutex_unlock(&pDRV2624->reg_lock);
		if (nResult >= 0) break;
		else {
			if(retry_count != 0) msleep(10);
		}
	}
	if(nResult < 0) dev_err(pDRV2624->dev, "%s:%u: I2C error %d\n",
		__func__, __LINE__, nResult);
	return nResult;
}

static int drv2624_bulk_read(struct drv2624_data *pDRV2624,
	unsigned char reg, unsigned char *buf, unsigned int count)
{
	int nResult;

	mutex_lock(&pDRV2624->reg_lock);
	nResult = regmap_bulk_read(pDRV2624->mpRegmap, reg, buf, count);
	mutex_unlock(&pDRV2624->reg_lock);
	if (nResult < 0)
		dev_err(pDRV2624->dev, "%s reg=0%x, count=%d error %d\n",
			__func__, reg, count, nResult);

	return nResult;
}

static int drv2624_bulk_write(struct drv2624_data *pDRV2624,
	unsigned char reg, const u8 *buf, unsigned int count)
{
	int nResult, i;

	mutex_lock(&pDRV2624->reg_lock);
	nResult = regmap_bulk_write(pDRV2624->mpRegmap, reg, buf, count);
	mutex_unlock(&pDRV2624->reg_lock);
	if (nResult < 0)
		dev_err(pDRV2624->dev, "%s reg=0%x, count=%d error %d\n",
			__func__, reg, count, nResult);
	else if (g_logEnable)
		for(i = 0; i < count; i++)
			dev_dbg(pDRV2624->dev, "%s, Reg[0x%x]=0x%x\n",
				__func__, reg+i, buf[i]);

	return nResult;
}

static int drv2624_set_bits(struct drv2624_data *pDRV2624,
			    unsigned char reg, unsigned char mask,
			    unsigned char val)
{
	int nResult = -1;
	int retry_count = DRV26XX_I2C_RETRY_COUNT;
	dev_info(pDRV2624->dev, "%s:%u: Reg[0x%02x]:M=0x%02x, V=0x%02x\n",
		__func__, __LINE__, reg, mask, val);
	while (retry_count--) {
		mutex_lock(&pDRV2624->reg_lock);
		nResult = regmap_update_bits(pDRV2624->mpRegmap, reg, mask, val);
		mutex_unlock(&pDRV2624->reg_lock);
		if (nResult >= 0) break;
		else {
			if(retry_count != 0) msleep(20);
		}
	}
	if(nResult < 0)	dev_err(pDRV2624->dev, "%s:%u: I2C error %d\n",
		__func__, __LINE__, nResult);
	return nResult;
}

static void drv2624_set_basic_reg(struct drv2624_data *pDRV2624)
{
	drv2624_set_bits(pDRV2624, DRV2624_R0X09,
		DRV2624_R0X09_UVLO_THRES_MSK,
		DRV2624_R0X09_UVLO_THRES_3_2V);
	drv2624_set_bits(pDRV2624, DRV2624_R0X0D,
		DRV2624_R0X0D_PLAYBACK_INTERVAL_MSK,
		(true == pDRV2624->bInternval_1ms)?
		DRV2624_R0X0D_PLAYBACK_INTERVAL_1MS:
		DRV2624_R0X0D_PLAYBACK_INTERVAL_5MS);

	drv2624_set_bits(pDRV2624, DRV2624_R0X23,
		DRV2624_R0X23_BEMF_GAIN_MSK,
		DRV2624_R0X23_BEMF_GAIN_30X_LRA);
	drv2624_set_bits(pDRV2624, DRV2624_R0X29,
		DRV2624_R0X29_SAMPLE_TIME_MSK,
		DRV2624_R0X29_SAMPLE_TIME_250us);
}

static void drv2624_hw_reset(struct drv2624_data *pDRV2624)
{
	dev_dbg(pDRV2624->dev, "%s: %u! \n", __func__, __LINE__);
	gpio_direction_output(pDRV2624->msPlatData.mnGpioNRST, 0);
	usleep_range(5000, 5050);
	gpio_direction_output(pDRV2624->msPlatData.mnGpioNRST, 1);
	usleep_range(2000, 2050);
}

static void drv2624_parse_dt(struct device *dev,
	struct drv2624_data *pDRV2624)
{
	struct device_node *np = dev->of_node;
	struct drv2624_platform_data *pPlatData = &pDRV2624->msPlatData;
	int rc = 0;
	unsigned int value = 0;

	rc = of_property_read_u32(np, "ti,i2c_addr", &value);
	if (rc) {
		dev_err(pDRV2624->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,i2c_addr", np->full_name, rc);
		value = 0x5A;
	}
	dev_err(pDRV2624->dev, "ti,i2c_addr=%x\n", value);
	pDRV2624->client->addr = value;
	pPlatData->mnGpioNRST = of_get_named_gpio(np, "ti,reset-gpio", 0);
	if (!gpio_is_valid(pPlatData->mnGpioNRST)) {
		dev_err(pDRV2624->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,reset-gpio", np->full_name, pPlatData->mnGpioNRST);
	} else {
		dev_info(pDRV2624->dev, "ti,reset-gpio=%d\n",
			pPlatData->mnGpioNRST);
	}

	rc = of_property_read_u32(np, "ti,smart-loop", &value);
	if (rc) {
		dev_info(pDRV2624->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,smart-loop", np->full_name, rc);
		value = 1;
	}

	pPlatData->mnLoop = value & 0x01;
	dev_info(pDRV2624->dev, "ti,smart-loop=%d\n", pPlatData->mnLoop);

	rc = of_property_read_u32(np, "ti,interval-ms", &value);
	if (rc) {
		dev_info(pDRV2624->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,interval", np->full_name, rc);
		value = 1;
	}
	pDRV2624->bInternval_1ms = (value == 5) ? false : true;
	dev_info(pDRV2624->dev, "ti,interval-ms=%d ms\n",
		pDRV2624->bInternval_1ms==true?1:5);

	rc = of_property_read_u32(np, "ti,actuator", &value);
	if (rc) {
		dev_err(pDRV2624->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,actuator", np->full_name, rc);
		value = LRA;
	}

	pPlatData->msActuator.mnActuatorType = value & 0x01;
	dev_info(pDRV2624->dev, "ti,actuator=%d\n",
		pPlatData->msActuator.mnActuatorType);

	rc = of_property_read_u32(np, "ti,odclamp-voltage", &value);
	if (rc) {
		dev_err(pDRV2624->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,odclamp-voltage", np->full_name, rc);
		value = 2540;//2.54V
	}

	pPlatData->msActuator.mnDTSOverDriveClampVoltage = value;
	pPlatData->msActuator.mnDTSOverDriveClampVoltageRegbits =
		(value * 100) / 2122;

	dev_info(pDRV2624->dev, "ti,odclamp-voltage=%d, R=0x%02x\n",
		pPlatData->msActuator.mnDTSOverDriveClampVoltage,
		pPlatData->msActuator.mnDTSOverDriveClampVoltageRegbits);

	if (pPlatData->msActuator.mnActuatorType == LRA) {
		rc = of_property_read_u32(np, "ti,lra-frequency", &value);
		if (rc) {
			dev_err(pDRV2624->dev,
				"Looking up %s property in node %s failed %d\n",
				"ti,lra-frequency", np->full_name, rc);
			value = FRES_DEFAULT;
		}

		if ((value < FRES_MIN) || (value > FRES_MAX)) {
			dev_err(pDRV2624->dev,
				"ERROR, ti,lra-frequency=%d, out of range, "
				"set to default value\n", value);
			value = FRES_DEFAULT;
		}

		pPlatData->msActuator.mnDTSLRAFreq = value;

		rc = of_property_read_u32(np, "ti,lra-fre-min", &value);
		if (rc) {
			dev_err(pDRV2624->dev,
				"Looking up %s property in node %s failed %d\n",
				"ti,lra-fre-min", np->full_name, rc);
			value = FRES_MIN;
		}

		if (value >= pPlatData->msActuator.mnDTSLRAFreq) {
			dev_err(pDRV2624->dev,
				"ERROR, ti,lra-fre-min=%u, invalid, "
				"set to default value\n", value);
			value = FRES_MIN;
		}
		pPlatData->msActuator.mnDTSLRAminFreq = value;

		rc = of_property_read_u32(np, "ti,lra-fre-max", &value);
		if (rc) {
			dev_err(pDRV2624->dev,
				"Looking up %s property in node %s failed %d\n",
				"ti,lra-fre-max", np->full_name, rc);
			value = FRES_MAX;
		}

		if (value <= pPlatData->msActuator.mnDTSLRAFreq) {
			dev_err(pDRV2624->dev,
				"ERROR, ti,lra-fre-min=%u, invalid, "
				"set to default value\n", value);
			value = pPlatData->msActuator.mnDTSLRAFreq << 1;
		}
		pPlatData->msActuator.mnDTSLRAmaxFreq = value;

		pPlatData->msActuator.mnDTSLRAPeriod = 10000 /
			pPlatData->msActuator.mnDTSLRAFreq + 1;
		dev_info(pDRV2624->dev, "ti,lra-frequency = %uHz ti, "
			"lra-period = %ums\n", pPlatData->msActuator.mnDTSLRAFreq,
			pPlatData->msActuator.mnDTSLRAPeriod);

		rc = of_property_read_u32(np, "ti,sample-time",
			&pPlatData->msActuator.mnDTSSampleTime);
		if (rc) {
			dev_err(pDRV2624->dev,
				"Looking up %s property in node %s failed %d\n",
				"ti,sample-time", np->full_name, rc);
		}
		//us
		switch (pPlatData->msActuator.mnDTSSampleTime) {
		case 150:
			pPlatData->msActuator.mnDTSSampleTimeRegbits = 0;
			break;
		case 200:
			pPlatData->msActuator.mnDTSSampleTimeRegbits = 4;
			break;
		case 250:
			pPlatData->msActuator.mnDTSSampleTimeRegbits = 8;
			break;
		default:
			pPlatData->msActuator.mnDTSSampleTime = 300;
			pPlatData->msActuator.mnDTSSampleTimeRegbits = 0xc;
			break;
		}
		dev_info(pDRV2624->dev, "ti,Sampletime=%d, R=0x%02x\n",
			pPlatData->msActuator.mnDTSSampleTime,
			pPlatData->msActuator.mnDTSSampleTimeRegbits);
		rc = of_property_read_u32(np, "ti,rated-voltage", &value);
		if (rc) {
			dev_err(pDRV2624->dev,
				"Looking up %s property in node %s failed %d\n",
				"ti,rated-voltage", np->full_name, rc);
			value = 1800;
		}
		pPlatData->msActuator.mnDTSRatedVoltage = value;
		pPlatData->msActuator.mnDTSRatedVoltageRegbits =
		    pPlatData->msActuator.mnDTSRatedVoltage *
		    int_sqrt(1000000-(4*pPlatData->msActuator.mnDTSSampleTime+300)
		    	*pPlatData->msActuator.mnDTSLRAFreq)/20580;
		dev_info(pDRV2624->dev, "ti,rated-voltage=%d, R=0x%02x\n",
			value, pPlatData->msActuator.mnDTSRatedVoltageRegbits);
	}
	pPlatData->mnGpioINT = of_get_named_gpio(np, "ti,irq-gpio", 0);
	if (!gpio_is_valid(pPlatData->mnGpioINT)) {
		dev_err(pDRV2624->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,irq-gpio", np->full_name, pPlatData->mnGpioINT);
		pDRV2624->mbIRQUsed = false;
	} else{
		dev_info(pDRV2624->dev, "ti,irq-gpio=%d\n",
			pPlatData->mnGpioINT);
		pDRV2624->mbIRQUsed = true;
	}
}

unsigned char drv_get_bf_value(const unsigned char bf,
	const unsigned char reg_value)
{
	unsigned char msk, value;

	/*
	 * bitfield enum:
	 * - 0..3  : len
	 * - 4..7  : pos
	 */
	unsigned char len = bf & 0x0f;
	unsigned char pos = (bf >> 4) & 0x0f;

	msk = ((1 << (len + 1)) - 1) << pos;
	value = (reg_value & msk) >> pos;

	return value;
}

static void drv2624_rtp_load(const struct firmware *pFW,
	void *pContext)
{
	struct drv2624_data *pDRV26xx = (struct drv2624_data*)pContext;
	struct drv26xx_RTPwaveforms* wvfm = NULL;
	struct drv26xx_RTPwaveform_info** rtp_wvfm_info = NULL;
	unsigned char* buf = (unsigned char*)pFW->data;
	int cur = 0;
	unsigned short i = 0;

	
	if (!pFW || !pFW->data || !pDRV26xx) {
		pr_err("%s:%u:Failed to read firmware\n", __func__,
			__LINE__);
		return;
	}
	
	wvfm = &(pDRV26xx->rtpwvfm);
	if (cur + 4 > pFW->size) {
		pr_err("%s:%u:bin file error!\n", __func__, __LINE__);
		goto EXIT;
	}

	
	wvfm->filetype = SMS_HTONL(buf[cur], buf[cur + 1], buf[cur + 2],
		buf[cur + 3]);
	if (wvfm->filetype != RTP_ID) {
		wvfm->version = 0;
		pr_err("Wrong bin file 0x%02x 0x%02x 0x%02x 0x%02x 0x%x\r\n", 
			buf[cur], buf[cur+1], buf[cur+2], buf[cur+3], RTP_ID);
		goto EXIT;
	}
	else {
		cur += 4;
		if (cur + 2 > pFW->size) {
			pr_err("%s:%u:bin file error!\n", __func__,
				__LINE__);
			goto EXIT;
		}
		wvfm->version = SMS_HTONS(buf[cur+1], buf[cur]);
		if(wvfm->version != 4) {
			pr_err("%s:%u:Error Ver:%u, correct is 4!\n",
				__func__, __LINE__, wvfm->version);
			goto EXIT;
		}
		cur += 2;
	}

	if (cur + 2 > pFW->size) {
		pr_err("%s:%u:bin file error!\n", __func__, __LINE__);
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
		pr_err("%s:%u:bin file error!\n", __func__, __LINE__);
		goto EXIT;
	}

	wvfm->rated_Voltage = SMS_HTONS(buf[cur+1], buf[cur]);
	cur += 2;
	if (cur + 2 > pFW->size) {
		pr_err("%s:%u:bin file error!\n", __func__, __LINE__);
		goto EXIT;
	}
	wvfm->overDrive_Voltage = SMS_HTONS(buf[cur+1], buf[cur]);
	cur += 2;
	if (cur + 2 > pFW->size) {
		pr_err("%s:%u:bin file error!\n", __func__, __LINE__);
		goto EXIT;
	}
	wvfm->lra_f0 = SMS_HTONS(buf[cur+1], buf[cur]);
	cur += 2;
	if (cur + 2 > pFW->size) {
		pr_err("%s:%u:bin file error!\n", __func__, __LINE__);
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

	pr_info("version = 0x%02x\n", wvfm->version);
	pr_info("fb_brake_factor = 0x%02x\n",
		wvfm->fb_brake_factor);
	pr_info("hybrid_loop = 0x%02x:%s\n", wvfm->hybrid_loop,
		(wvfm->hybrid_loop == 1) ? "Enable" : "Disable");
	pr_info("auto_brake = 0x%02x:%s\n", wvfm->auto_brake,
		(wvfm->auto_brake == 1) ? "Enable" : "Disable");
	pr_info("auto_brake_standby = 0x%02x:%s\n",
		wvfm->auto_brake_standby,
		(wvfm->auto_brake_standby == 1) ? "Enable" : "Disable");
	pr_info("rated_Voltage = %u mV\n", wvfm->rated_Voltage);
	pr_info("overDrive_Voltage = %u mV\n",
		wvfm->overDrive_Voltage);
	pr_info("lra_f0 = %u Hz\n", wvfm->lra_f0);
	pr_info("nWaveforms = %u\n", wvfm->nWaveforms);

	rtp_wvfm_info =
		(struct drv26xx_RTPwaveform_info **)kzalloc(wvfm->nWaveforms,
		sizeof(struct drv26xx_RTPwaveform_info *));

	if (NULL == rtp_wvfm_info) {
		pr_err( "%s:%u:wvfm %u kalloc error!\n", __func__,
			__LINE__, i);
		wvfm->nWaveforms = 0;
		goto EXIT;
	}

	for (i = 0; i < wvfm->nWaveforms; i++) {
		unsigned short offset = 0;
		if (cur + 8 > pFW->size) {
			pr_err( "%s:%u:wvfm %u bin file error!\n",
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

		pr_info("wvfm = %u dur = %u commands number = %u "
			"f0 = %u\n", i, rtp_wvfm_info[i]->duration,
			rtp_wvfm_info[i]->length, rtp_wvfm_info[i]->lra_f0);
		pr_info("rtp_wvfm_info[%u]->brake = 0x%02x:%s\n",
			i, rtp_wvfm_info[i]->brake,
			(rtp_wvfm_info[i]->brake == 1) ? "Enable" : "Disable");
		pr_info("rtp_wvfm_info[%u]->loop_mod = 0x%02x:%s\n",
			i, rtp_wvfm_info[i]->loop_mod,
			(rtp_wvfm_info[i]->loop_mod != 1) ?  "Close": "Open");
		pr_info("rtp_wvfm_info[%u]->wv_shape = 0x%02x:%s\n",
			i, rtp_wvfm_info[i]->wv_shape,
			(rtp_wvfm_info[i]->wv_shape != 1) ? "Square" : "Sine");
		pr_info("rtp_wvfm_info[%u]->actuator_type = "
			"0x%02x:%s\n", i, rtp_wvfm_info[i]->actuator_type,
			(rtp_wvfm_info[i]->actuator_type == 0) ? "LRA" : "ERM");

		if (offset + rtp_wvfm_info[i]->length*2 > pFW->size) {
			pr_err( "%s:%u:wvfm %u bin file error!\n",
				__func__, __LINE__, i);
			pr_err(
				"pFW->size = %d offset = %u len *2 = %u!\n",
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
static void drv2624_set_mode_reg(struct drv2624_data *pDRV2624, char WorkMode)
{
	if (WorkMode == DRV2624_RAM_MODE) {
		drv2624_reg_write(pDRV2624, 0x23, 0x27);
		drv2624_reg_write(pDRV2624, 0x27, 0x93);
	} else {
		drv2624_reg_write(pDRV2624, 0x23, 0x37);
		drv2624_reg_write(pDRV2624, 0x27, 0x13);
	}
}
static inline int drv2624_change_mode(struct drv2624_data *pDRV2624,
				      drv2624_mode_t work_mode)
{
	pDRV2624->mnWorkMode = work_mode;
	drv2624_set_mode_reg(pDRV2624, work_mode);
	return drv2624_set_bits(pDRV2624, DRV2625_REG_MODE, WORKMODE_MASK,
				work_mode);
}
static inline void drv2624_set_stopflag(struct drv2624_data *pDRV2624)
{
	pDRV2624->mnVibratorPlaying = NO;
	dev_info(pDRV2624->dev, "%s: mnVibratorPlaying=%d\n", __func__,
			 pDRV2624->mnVibratorPlaying);
//      pDRV2624->mnEffectType = 0;
}
static void drv2624_init(struct drv2624_data *pDRV2624)
{
	struct drv2624_platform_data *pDrv2624Platdata =
		&pDRV2624->msPlatData;
	struct actuator_data *pActuator = &(pDrv2624Platdata->msActuator);
	int nResult = 0;
	unsigned int DriveTime = 0;

	drv2624_set_bits(pDRV2624, DRV2624_R0X07,
		DRV2624_R0X07_TRIG_PIN_FUNC_MSK,
		DRV2624_R0X07_TRIG_PIN_FUNC_INT);

	drv2624_set_bits(pDRV2624, DRV2624_R0X08,
		DRV2624_R0X08_LRA_ERM_MSK |
		DRV2624_R0X08_AUTO_BRK_OL_MSK |
		DRV2624_R0X08_CTL_LOOP_MSK,
		(pActuator->mnActuatorType << DRV2624_R0X08_LRA_ERM_SFT) |
		(pDrv2624Platdata->mnLoop << DRV2624_R0X08_CTL_LOOP_SFT) |
		DRV2624_R0X08_AUTO_BRK_OL_EN);

	if (pActuator->mnActuatorType == LRA) {
		/*****************************************/
		/* DriveTime(ms) = 0.5 *(1/LRA) * 1000   */
		/* DriveTime(bit) = DriveTime(ms)*10 - 5 */
		/*****************************************/
		DriveTime = 5 * (1000 - pActuator->mnDTSLRAFreq) /
			pActuator->mnDTSLRAFreq;
		drv2624_set_bits(pDRV2624, DRV2624_R0X27,
			DRV2624_R0X27_DRIVE_TIME_MSK |
			DRV2624_R0X27_LRA_MIN_FREQ_SEL_MSK,
			DriveTime | ((pActuator->mnDTSLRAFreq < 125)?
			DRV2624_R0X27_LRA_MIN_FREQ_SEL_45HZ:0));
		pActuator->openLoopPeriod =
			(unsigned short)((unsigned int)LRA_PERIOD_SCALE_UNIT /
					(OL_LRA_PERIOD_SCALE_NS *
					pActuator->mnDTSLRAFreq));

		nResult = drv2624_set_bits(pDRV2624,
			DRV2624_R0X2E_OL_LRA_PERIOD_H,
			0x03, (pActuator->openLoopPeriod & 0x0300) >> 8);
		if (nResult < 0) {
			dev_err(pDRV2624->dev,
				"%s:%u set bits not Done nResult = %d\n",
				__func__, __LINE__, nResult);
			return;
		}
		nResult = drv2624_reg_write(pDRV2624,
			DRV2624_R0X2F_OL_LRA_PERIOD_L,
			(pActuator->openLoopPeriod & 0x00ff));
		if (nResult < 0) {
			dev_err(pDRV2624->dev,
				"%s:%u set bits not Done nResult = %d\n",
				__func__, __LINE__, nResult);
			return;
		}
	}else {
		drv2624_set_bits(pDRV2624, DRV2624_R0X08,
			DRV2624_R0X08_LRA_ERM_MSK, DRV2624_R0X08_ERM);
	}

	drv2624_set_basic_reg(pDRV2624);
}

static void drv2624_vibrator_enable(
	struct led_classdev *led_cdev,
	enum led_brightness value)
{
	struct drv2624_data *pDRV2624 = container_of(led_cdev,
		struct drv2624_data,
		led_dev);
	hrtimer_cancel(&pDRV2624->haptics_timer);
	mutex_lock(&pDRV2624->haptic_lock);
	pDRV2624->state = value;
	pDRV2624->duration = value;
	mutex_unlock(&pDRV2624->haptic_lock);
	schedule_work(&pDRV2624->vibrator_work);
}
static ssize_t drv2624_rtp_test_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct drv2624_data *pDRV2624 = container_of(led_cdev,
												struct drv2624_data,
												led_dev);
	unsigned int val = 0;
	int rc = kstrtouint(buf, 0, &val);
	if(rc >= 0){
		if(val){
			pDRV2624->mnWorkMode = DRV2624_RTP_MODE;
			drv2624_rtpvibrate(pDRV2624, val%4);
		}
		else{
			drv2624_stop(pDRV2624);
		}
	}

	pr_info("%s:%u!\n", __func__, __LINE__);
	return count;
}

static ssize_t drv2624_rtp_test_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct drv2624_data *pDRV2624 = container_of(led_cdev,
		struct drv2624_data,
		led_dev);

	return snprintf(buf, 16, "%d\n", pDRV2624->state);
}

static ssize_t drv2624_activate_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct drv2624_data *pDRV2624 = container_of(led_cdev,
		struct drv2624_data,
		led_dev);

	/* For now nothing to show */
	return snprintf(buf, 16, "%d\n", pDRV2624->state);
}
static ssize_t drv2624_activate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct drv2624_data *pDRV2624 = container_of(led_cdev,
		struct drv2624_data,
		led_dev);
	unsigned int val = 0;
	int rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val != 0 && val != 1)
		return count;

	pr_err("%s: value=%d\n", __func__, val);

	mutex_lock(&pDRV2624->haptic_lock);
	hrtimer_cancel(&pDRV2624->haptics_timer);

	pDRV2624->state = val;
	mutex_unlock(&pDRV2624->haptic_lock);
	schedule_work(&pDRV2624->vibrator_work);

	return count;
}

static ssize_t drv2624_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct drv2624_data *pDRV2624 = container_of(led_cdev,
		struct drv2624_data,
		led_dev);

	ktime_t time_rem;
	s64 time_ms = 0;

	if (hrtimer_active(&pDRV2624->haptics_timer)) {
		time_rem = hrtimer_get_remaining(&pDRV2624->haptics_timer);
		time_ms = ktime_to_ms(time_rem);
	}

	return snprintf(buf, 16, "%lld\n", time_ms);

}

static ssize_t drv2624_duration_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct drv2624_data *pDRV2624 = container_of(led_cdev,
		struct drv2624_data,
		led_dev);
	int value;
	int rc = kstrtouint(buf, 0, &value);

	if (rc < 0) {
		dev_err(pDRV2624->dev, "%s:%u: rc = %d\n", __func__,
				__LINE__, rc);
		return rc;
	}
	dev_info(pDRV2624->dev, "%s:%u: duration = %d, buf %s", 
		__func__, __LINE__, value, buf);
	/* setting 0 on duration is NOP for now */
	if (value <= 0)
		return count;
	else if (value > 0 && value <= 20)
		pDRV2624->waveform_id = 3;
	else if (value > 20 && value <= 30)
		pDRV2624->waveform_id = 2;
	else if (value > 30 && value <= 60)
		pDRV2624->waveform_id = 1;
	else
		pDRV2624->waveform_id = 4;

	pDRV2624->duration = value;

	return count;
}
static ssize_t drv2624_rtpbininfo_list_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct drv2624_data *pDRV2624 = NULL;
	struct drv26xx_RTPwaveforms* wvfm = NULL;
	int ret = 0;
	char *temp = NULL;
	struct sysfs_cmd *pSysfsCmd = NULL;

	if( led_cdev == NULL ) {
		pr_err("%s:%u: input_dev is NULL.\n", __func__, __LINE__);
		return count;
	}
	pDRV2624 = container_of(led_cdev, struct drv2624_data, led_dev);
	if( pDRV2624 == NULL ) {
		pr_err("%s:%u: pDRV2624 is NULL.\n", __func__, __LINE__);
		return count;
	}
	wvfm = &(pDRV2624->rtpwvfm);
	pSysfsCmd = &pDRV2624->mSysfsCmd;
	pSysfsCmd->bufLen = snprintf(gSysfsCmdLog, 256,
		"command: echo WV > NODE\n"
		"WV is rtp_wave_id, it should be 3-digital decimal\n"
		"eg: echo 021 > NODE\n\r");

	if (count >= 3) {
		temp = kmalloc(count, GFP_KERNEL);
		if (!temp) {
			pSysfsCmd->bCmdErr = true;
			pSysfsCmd->bufLen += snprintf(gSysfsCmdLog,
				15, "No memory!\n");
			goto EXIT;
		}
		memcpy(temp, buf, count);
		ret = sscanf(temp, "%hu", &(pSysfsCmd->rtpwavid));
		if (!ret) {
			pSysfsCmd->bCmdErr = true;
			kfree(temp);
			goto EXIT;
		}
		dev_info(pDRV2624->dev, "%s:%d: RtpWaveId=%2d, cnt=%d\n",
			__func__, __LINE__, pSysfsCmd->rtpwavid, (int)count);
		if(pSysfsCmd->rtpwavid >= wvfm->nWaveforms) {
			pSysfsCmd->bCmdErr = true;
			pSysfsCmd->bufLen +=
				snprintf(gSysfsCmdLog,
					30, "Wrong RtpWaveId!\n\r");
		} else {
			pSysfsCmd->bCmdErr = false;
			gSysfsCmdLog[0] = '\0';
			pSysfsCmd->bufLen = 0;
		}
		kfree(temp);
	} else {
		pSysfsCmd->bCmdErr = true;
		dev_err(pDRV2624->dev, "%s:%u count error.\n",
			__func__, __LINE__);
	}
EXIT:
	return count;
}

static ssize_t drv2624_rtpbininfo_list_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct drv2624_data *pDRV2624 = NULL;
	struct sysfs_cmd *pSysfsCmd = NULL;
	struct drv26xx_RTPwaveforms* wvfm = NULL;
	struct drv26xx_RTPwaveform_info** rtp_wvfm_info = NULL;
	int n = 0;
	unsigned short i = 0, j = 0;

	if(led_cdev == NULL) {
		if(n + 128 < PAGE_SIZE) {
			n += scnprintf(buf+n, PAGE_SIZE -n,
				"%s:%u:ERROR: Can't find led_cdev handle!\n",
				__func__, __LINE__);
		} else {
			scnprintf(buf+PAGE_SIZE-100, 100,
				"\n%s:%u: Out of memory!\n",
				__func__, __LINE__);
			n = PAGE_SIZE;
		}
		goto EXIT;
	}
	pDRV2624 = container_of(led_cdev, struct drv2624_data, led_dev);
	if (pDRV2624 == NULL) {
		if(n + 128 < PAGE_SIZE) {
			n += scnprintf(buf + n, PAGE_SIZE - n,
				"%s:%u:ERROR: Can't find drv2624_data handle!\n",
				__func__, __LINE__);
		} else {
			scnprintf(buf+PAGE_SIZE-100, 100,
				"\n%s:%u: Out of memory!\n",
				__func__, __LINE__);
			n = PAGE_SIZE;
		}
		goto EXIT;
	}

	pSysfsCmd = &pDRV2624->mSysfsCmd;
	wvfm = &(pDRV2624->rtpwvfm);

	if(pSysfsCmd->bCmdErr == true ||
		pSysfsCmd->rtpwavid >= wvfm->nWaveforms) {
		n += scnprintf(buf, pSysfsCmd->bufLen, gSysfsCmdLog);
		goto EXIT;
	}

	if(n + 32 + strlen(wvfm->rtp_binaryname)+1 < PAGE_SIZE) {
		n += scnprintf(buf+n, PAGE_SIZE -n,
			"Rtpbin File name: %s\n",
			wvfm->rtp_binaryname);
	} else {
		scnprintf(buf+PAGE_SIZE-100, 100,
			"\n%s:%u: Out of memory!\n\r",
			__func__, __LINE__);
		n = PAGE_SIZE;
		goto EXIT;
	}

	if(n + 256 < PAGE_SIZE) {
		n += scnprintf(buf + n, PAGE_SIZE - n,
			"fb_brake_factor = 0x%02x\n"
			"hybrid_loop = 0x%02x:%s\n"
			"auto_brake = 0x%02x:%s\n"
			"auto_brake_standby = 0x%02x:%s\n",
			wvfm->fb_brake_factor,
			wvfm->hybrid_loop,
			(wvfm->hybrid_loop == 1)
			? "Enable" : "Disable",
			wvfm->auto_brake,
			(wvfm->auto_brake == 1)
			? "Enable" : "Disable",
			wvfm->auto_brake_standby,
			(wvfm->auto_brake_standby == 1)
			? "Enable" : "Disable");
	} else {
		scnprintf(buf+PAGE_SIZE-100, 100,
			"\n%s:%u: Out of memory!\n\r",
			__func__, __LINE__);
		n = PAGE_SIZE;
		goto EXIT;
	}

	if (wvfm->version != 0) {
		if(n + 256 < PAGE_SIZE) {
			n += scnprintf(buf + n, PAGE_SIZE - n,
				"rated_Voltage = %u mV\n"
				"overDrive_Voltage = %u mV\n"
				"lra_f0 = %u\n"
				"sample_time = %u\n"
				"nWaveforms = %u\n",
				wvfm->rated_Voltage,
				wvfm->overDrive_Voltage,
				wvfm->lra_f0,
				wvfm->sample_time,
				wvfm->nWaveforms);
		} else {
			scnprintf(buf+PAGE_SIZE-100, 100,
				"\n%s:%u: Out of memory!\n\r",
				__func__, __LINE__);
			n = PAGE_SIZE;
			goto EXIT;
		}
	} else {
		if(n + 32 < PAGE_SIZE) {
			n += scnprintf(buf + n, PAGE_SIZE - n,
				"Wrong bin version\n");
		} else {
			scnprintf(buf+PAGE_SIZE-100, 100,
				"\n%s:%u: Out of memory!\n\r",
				__func__, __LINE__);
			n = PAGE_SIZE;
			goto EXIT;
		}
	}

	rtp_wvfm_info = wvfm->rtp_wvfm_info;
	for(i = 0; i < wvfm->nWaveforms; i++) {
		if (pSysfsCmd->rtpwavid == i) {
			if(n + 512 < PAGE_SIZE) {
				n += scnprintf(buf + n, PAGE_SIZE - n,
					"wvfm = %u\n\tlen = %u dur = %u\n"
					"\trtp_wvfm_info[%u]->brake = 0x%02x:%s\n"
					"\trtp_wvfm_info[%u]->loop_mod = 0x%02x:%s\n"
					"\trtp_wvfm_info[%u]->wv_shape = 0x%02x:%s\n",
					i, rtp_wvfm_info[i]->length,
					rtp_wvfm_info[i]->duration,
					i, rtp_wvfm_info[i]->brake,
					(rtp_wvfm_info[i]->brake == 1)
					? "Enable" : "Disable",
					i, rtp_wvfm_info[i]->loop_mod,
					(rtp_wvfm_info[i]->loop_mod == 1)
					? "Close" : "Open",
					i, rtp_wvfm_info[i]->wv_shape,
					(rtp_wvfm_info[i]->wv_shape == 1)
					? "Square" : "Sine");
			} else {
				scnprintf(buf+PAGE_SIZE-100, 100,
					"\n%s:%u: Out of memory!\n\r",
					__func__, __LINE__);
				n = PAGE_SIZE;
				break;
			}
			for (j = 0; j < rtp_wvfm_info[i]->length; j++) {
				if(n + 128 < PAGE_SIZE) {
					n += scnprintf(buf + n, PAGE_SIZE - n,
						"\t\tcmd = %02u: gain = 0x%02x "
						"sleep_time = 0x%02x\n",
						j, rtp_wvfm_info[i]->wvfm_cmds[j].gain,
						rtp_wvfm_info[i]->wvfm_cmds[j].sleep_time);
				} else {
					scnprintf(buf + PAGE_SIZE - 100, 100,
						"\n%s:%u: Out of memory!\n\r",
						__func__, __LINE__);
					n = PAGE_SIZE;
					break;
				}
			}
			break;
		}
	}
EXIT:
	return n;
}
/**
 *
 * bRTP = NO == 0; Enable all interrupt of DRV2624
 * bRTP = 1 == 1; Only Enable critical interrupt,
 *	PROCESS_DONE and PRG_ERROR
 *
 **/
static int drv2624_enableIRQ(struct drv2624_data *pDRV2624, unsigned char bRTP)
{
	int nResult = 0;
	unsigned char bitSet =
		DRV2624_R0X02_INTZ_ENABLE|
		DRV2624_R0X02_INTZ_PROCESS_DONE_DISABLE;

	if(gpio_is_valid(pDRV2624->msPlatData.mnGpioINT)) {
		if (bRTP == 0)
			bitSet = DRV2624_R0X02_INTZ_ENABLE;
		if (pDRV2624->mbIRQEnabled)
			goto end;

		nResult = drv2624_reg_read(pDRV2624, DRV2624_R0X01_STATUS);
		if (nResult < 0) {
			dev_err(pDRV2624->dev, "%s:%u: I2C read error\n",
				__func__, __LINE__);
			goto end;
		}
		nResult = drv2624_set_bits(pDRV2624,
					DRV2624_R0X02_INTZ,
					DRV2624_R0X02_INTZ_MSK, bitSet);
		if (nResult < 0) {
			dev_err(pDRV2624->dev, "%s:%u: I2C set bit error\n",
				__func__, __LINE__);
			goto end;
		}
		enable_irq(pDRV2624->mnIRQ);
		pDRV2624->mbIRQEnabled = true;
	}
end:
	return nResult;
}

static void drv2624_disableIRQ(struct drv2624_data *pDRV2624)
{
	int nResult = 0;
	if(gpio_is_valid(pDRV2624->msPlatData.mnGpioINT)) {
		if(pDRV2624->mbIRQEnabled) {
			disable_irq_nosync(pDRV2624->mnIRQ);
			drv2624_reg_write(pDRV2624, DRV2624_R0X02_INTZ,
				DRV2624_R0X02_INTZ_DISABLE);
			nResult = drv2624_set_bits(pDRV2624,
						DRV2624_R0X02_INTZ,
						DRV2624_R0X02_INTZ_MSK,
						DRV2624_R0X02_INTZ_DISABLE);
			if (nResult < 0) {
				dev_err(pDRV2624->dev,
					"%s:%u: I2C set bit error\n", __func__,
					__LINE__);
			} else pDRV2624->mbIRQEnabled = false;
		}
	}
}

static int drv2624_set_go_bit(struct drv2624_data *pDRV2624,
	unsigned char val)
{
	int nResult = 0, value = 0;
	int retry = DRV26XX_GO_BIT_MAX_RETRY_CNT;

	val &= DRV2624_R0X0C_GO_MSK;
	dev_err(pDRV2624->dev, "%s, go val = %d\n", __func__, val);
	nResult = drv2624_reg_write(pDRV2624, DRV2624_R0X0C_GO, val);
	if (nResult < 0)
		goto end;

	nResult = drv2624_set_bits(pDRV2624, DRV2624_R0X07,
		DRV2624_R0X07_TRIG_PIN_FUNC_MSK,
		(val == 0) ? DRV2624_R0X07_TRIG_PIN_FUNC_EXTERNAL_PULSE:
		DRV2624_R0X07_TRIG_PIN_FUNC_INT);
	if (nResult < 0) {
		dev_err(pDRV2624->dev, "%s:%u: I2C set error\n", __func__,
			__LINE__);
		goto end;
	}
	usleep_range(DRV26XX_GO_BIT_CHECK_INTERVAL,
		DRV26XX_GO_BIT_CHECK_INTERVAL);
	do {
		value = drv2624_reg_read(pDRV2624, DRV2624_R0X0C_GO);
		if (value < 0) {
			nResult = value;
			break;
		}
		if ((value & DRV2624_R0X0C_GO_MSK) == val) break;
		else {
			dev_info(pDRV2624->dev, "%s, GO bit %d\n",
				__func__, value);

			nResult = drv2624_reg_write(pDRV2624,
				DRV2624_R0X0C_GO, val);
 			if (nResult < 0) break;
			usleep_range(DRV26XX_GO_BIT_CHECK_INTERVAL,
				DRV26XX_GO_BIT_CHECK_INTERVAL);
		}
		retry--;
	} while (retry > 0);

	if(nResult == 0) {
		if (retry != 0) {
			pDRV2624->mbWork = (0x0 == val)?false:true;
			if (val) {
				nResult = drv2624_enableIRQ(pDRV2624, YES);
			} else {
				drv2624_disableIRQ(pDRV2624);
			}
			dev_info(pDRV2624->dev, "%s:%u: retry = %d, value = %d "
				"pull go bit success!\n", __func__, __LINE__,
				retry, value);
		} else {
			if(val) {
				dev_err(pDRV2624->dev, "%s:%u: retry = %d, "
					"Set go bit to %d failed!\n",
					__func__, __LINE__, retry, val);
			} else {
				if(pDRV2624->bRTPmode == false) {
					nResult = drv2624_set_bits(pDRV2624,
						DRV2624_R0X07,
						DRV2624_R0X07_MODE_MSK,
						DRV2624_R0X07_MODE_RTP_MODE);
					if (nResult < 0) {
						dev_err(pDRV2624->dev,
							"%s:%u: I2C set error\n",
							__func__, __LINE__);
						goto end;
					}

					nResult = drv2624_reg_write(pDRV2624,
						DRV2624_R0X0C_GO, val);
 					if (nResult < 0) {
						dev_err(pDRV2624->dev,
							"%s:%u: I2C write error\n",
							__func__, __LINE__);
						goto end;
					}
				} else {
					dev_err(pDRV2624->dev, "%s:%u: retry = %d, "
						"Set go bit to %d failed!\n",
						__func__, __LINE__, retry, val);
				}
			}
		}
	}

end:
	return nResult;
}

static int drv262x_auto_calibrate(
	struct drv2624_data *pDRV2624)
{
	int nResult = 0;
	int retry = 9;
	struct drv2624_platform_data *pDrv2624Platdata =
		&pDRV2624->msPlatData;
	struct actuator_data *pActuator = &(pDrv2624Platdata->msActuator);
	struct drv26xx_RTPwaveforms* wvfm = &(pDRV2624->rtpwvfm);

	dev_info(pDRV2624->dev, "%s enter!\n", __func__);
	mutex_lock(&pDRV2624->haptic_lock);
/**
 * Set MODE register to Auto Level Calibration Routine
 * and choose Trigger Function Internal
 **/
 	nResult = drv2624_set_bits(pDRV2624, DRV2624_R0X27,
		DRV2624_R0X27_DRIVE_TIME_MSK |
		DRV2624_R0X27_LRA_MIN_FREQ_SEL_MSK,
		wvfm->DriveTime | ((wvfm->lra_f0 < 125)?
		DRV2624_R0X27_LRA_MIN_FREQ_SEL_45HZ:0));
	if (nResult < 0) {
		dev_err(pDRV2624->dev,
			"%s:%u set bits not Done nResult = %d\n",
			__func__, __LINE__, nResult);
		mutex_unlock(&pDRV2624->haptic_lock);
		goto end;
	}

	nResult = drv2624_set_bits(pDRV2624, DRV2624_R0X08,
		DRV2624_R0X08_LRA_ERM_MSK | DRV2624_R0X08_CTL_LOOP_MSK,
		DRV2624_R0X08_LRA | DRV2624_R0X08_CTL_LOOP_CLOSED_LOOP);
	if (nResult < 0) {
		dev_err(pDRV2624->dev,
			"%s:%u set bits not Done nResult = %d\n",
			__func__, __LINE__, nResult);
		goto end;
	}

	nResult = drv2624_set_bits(pDRV2624,
		DRV2624_R0X07, DRV2624_R0X07_MODE_MSK,
		DRV2624_R0X07_MODE_AUTO_LVL_CALIB_RTN);/*0x4B*/
	if (nResult < 0) {
		dev_err(pDRV2624->dev,
			"%s:%u set bits not Done nResult = %d\n",
			__func__, __LINE__, nResult);
		mutex_unlock(&pDRV2624->haptic_lock);
		goto end;
	}
	nResult = drv2624_set_bits(pDRV2624,
		DRV2624_R0X2E_OL_LRA_PERIOD_H, 0x03,
		(pActuator->openLoopPeriod & 0x0300) >> 8);
	if (nResult < 0) {
		dev_err(pDRV2624->dev,
			"%s:%u set bits not Done nResult = %d\n",
			__func__, __LINE__, nResult);
		mutex_unlock(&pDRV2624->haptic_lock);
		goto end;
	}
	nResult = drv2624_reg_write(pDRV2624,
		DRV2624_R0X2F_OL_LRA_PERIOD_L,
		(pActuator->openLoopPeriod & 0x00ff));
	if (nResult < 0) {
		dev_err(pDRV2624->dev,
			"%s:%u set bits not Done nResult = %d\n",
			__func__, __LINE__, nResult);
		mutex_unlock(&pDRV2624->haptic_lock);
		goto end;
	}

	nResult = drv2624_set_bits(pDRV2624,
		DRV2624_R0X2A, DRV2624_R0X2A_AUTO_CAL_TIME_MSK,
		DRV2624_R0X2A_AUTO_CAL_TIME_TRIGGER_CRTLD);
	if (nResult < 0) {
		dev_err(pDRV2624->dev,
			"%s:%u set bits not Done nResult = %d\n",
			__func__, __LINE__, nResult);
		mutex_unlock(&pDRV2624->haptic_lock);
		goto end;
	}

	nResult = drv2624_reg_write(pDRV2624, DRV2624_R0X1F_RATED_VOLTAGE,
		pActuator->mnDTSRatedVoltageRegbits);
	if (nResult < 0) {
		dev_err(pDRV2624->dev,
			"%s: mnCalRatedVoltage set failed nResult = %d\n",
			__func__, nResult);
		mutex_unlock(&pDRV2624->haptic_lock);
		goto end;
	}

	nResult = drv2624_reg_write(pDRV2624, DRV2624_R0X20_OD_CLAMP,
		pActuator->mnDTSOverDriveClampVoltageRegbits);
	if (nResult < 0) {
		dev_err(pDRV2624->dev,
			"%s: mnCalOverDriveClampVoltage "
			"set failed nResult = %d\n",
			__func__, nResult);
		mutex_unlock(&pDRV2624->haptic_lock);
		goto end;
	}

	nResult = drv2624_set_bits(pDRV2624, DRV2624_R0X29,
		DRV2624_R0X29_SAMPLE_TIME_MSK,
		pActuator->mnDTSSampleTimeRegbits);
	if (nResult < 0) {
		dev_err(pDRV2624->dev,
			"%s: mnDTSSampleTimeRegbits set failed nResult = %d\n",
			__func__, nResult);
		mutex_unlock(&pDRV2624->haptic_lock);
		goto end;
	}


	nResult = drv2624_set_bits(pDRV2624, DRV2624_R0X23,
		DRV2624_R0X23_FB_BRAKE_FACTOR_MSK,
		(FB_BRAKE_FACTOR_CALIBRATION <<
		DRV2624_R0X23_FB_BRAKE_FACTOR_SFT) &
		DRV2624_R0X23_FB_BRAKE_FACTOR_MSK);
	if (nResult < 0) {
		dev_err(pDRV2624->dev,"%s:%u set bits not Done "
			"nResult = %d\n", __func__, __LINE__, nResult);
		goto end;
	}

	nResult = drv2624_set_bits(pDRV2624, DRV2624_R0X23,
		DRV2624_R0X23_LOOP_GAIN_MSK,
		(LOOP_GAIN_CALIBRATION << DRV2624_R0X23_LOOP_GAIN_SFT) &
		DRV2624_R0X23_LOOP_GAIN_MSK);
	if (nResult < 0) {
		dev_err(pDRV2624->dev,"%s:%u set bits not Done "
			"nResult = %d\n", __func__, __LINE__, nResult);
		goto end;
	}

	nResult = drv2624_set_go_bit(pDRV2624, DRV2624_R0X0C_GO_BIT);
	if (nResult < 0) {
		dev_err(pDRV2624->dev,
			"%s: calibrate go bit not Done nResult = %d\n",
			__func__, nResult);
		mutex_unlock(&pDRV2624->haptic_lock);
		goto end;
	}
	mutex_unlock(&pDRV2624->haptic_lock);
	while(retry && pDRV2624->mbWork == true) {
		retry--;
		msleep(200); /* waiting auto calibration finished */
	}

end:
	if (nResult < 0)
		dev_err(pDRV2624->dev, "%s: Calibtion Done nResult = %d\n",
			__func__, nResult);
	return nResult;
}
static int drv262x_update_f0(struct drv2624_data *pDRV2624)
{
	int nResult = -1;
	if(pDRV2624) {
		dev_info(pDRV2624->dev, "%s: enter!\n", __func__);

		nResult =
		    drv2624_reg_write(pDRV2624, DRV2624_R0X2E_OL_LRA_PERIOD_H,
		    	pDRV2624->mAutoCalData.mnOL_LraPeriod.msb);
		if (nResult < 0) {
			dev_err(pDRV2624->dev, "%s:%u: write f0_msb failed\n",
				__func__, __LINE__);
			goto end;
		}

		nResult =
		    drv2624_reg_write(pDRV2624, DRV2624_R0X2F_OL_LRA_PERIOD_L,
		    	pDRV2624->mAutoCalData.mnOL_LraPeriod.lsb);
		if (nResult < 0) {
			dev_err(pDRV2624->dev, "%s:%u: write f0_lsb failed\n",
				__func__, __LINE__);
			goto end;
		}
		nResult = 0;
	} else {
		dev_err(pDRV2624->dev, "%s:%u:drv2624_data is NULL !\n",
			__func__, __LINE__);
	}
end:
	if (nResult < 0) {
		dev_err(pDRV2624->dev, "%s:%u: Failed to update f0 !\n",
			__func__, __LINE__);
	} else {
		dev_info(pDRV2624->dev, "%s:%u: F0 is updated OL_MSB:%d "
			"OL_LSB:%d\n", __func__, __LINE__,
			pDRV2624->mAutoCalData.mnOL_LraPeriod.msb,
			pDRV2624->mAutoCalData.mnOL_LraPeriod.lsb);
	}
	return nResult;
}

static int drv2624_get_calibration_result(
	struct drv2624_data *pDRV2624)
{
	int nResult = -1, retry = 5;
	unsigned int calibrated_f0 = 0, ol_lra_reg = 0, lra_reg = 0;
	struct drv2624_platform_data *pPlatData = &pDRV2624->msPlatData;
	struct drv26xx_RTPwaveforms* wvfm = NULL;

	if(NULL == pDRV2624) {
		pr_err( "%s:%u:drv2624_data is NULL !\n", __func__, __LINE__);
		goto end;
	}
	dev_info(pDRV2624->dev, "%s: enter!\n", __func__);
	wvfm = &(pDRV2624->rtpwvfm);

	do{
		nResult = drv2624_reg_read(pDRV2624, DRV2624_R0X01_STATUS);
		if (nResult < 0) {
			dev_err(pDRV2624->dev, "%s: nResult = %d\n", __func__,
				nResult);
			goto end;
		}
		else{
			if(nResult & DRV2624_R0X01_STATUS_PROCESS_DONE_MSK){
				retry--;
				msleep(200);
				pr_err("%s:%d ---val:0x%02x---", __func__, __LINE__, nResult);
			}
			else{
				break;
			}
		}
	}while(retry > 0);

	pDRV2624->mAutoCalData.mnDoneFlag =
		(nResult & DRV2624_R0X01_STATUS_DIAG_RESULT_MSK);

	if(pDRV2624->mAutoCalData.mnDoneFlag) {
		dev_err(pDRV2624->dev, "%s:%u: calibration failed\n",
			__func__, __LINE__);
		goto end;
	}
	nResult = drv2624_reg_read(pDRV2624, DRV2624_R0X21_CAL_COMP);
	if (nResult < 0) {
		dev_err(pDRV2624->dev, "%s:%u: read cal_comp failed\n",
			__func__, __LINE__);
		goto end;
	}
	pDRV2624->mAutoCalData.mnCalComp = nResult;

	nResult = drv2624_reg_read(pDRV2624, DRV2624_R0X22_CAL_BEMF);
	if (nResult < 0) {
		dev_err(pDRV2624->dev, "%s:%u: read cal_bemf failed\n",
			__func__, __LINE__);
		goto end;
	}
	pDRV2624->mAutoCalData.mnCalBemf = nResult;

	nResult =
		drv2624_reg_read(pDRV2624, DRV2624_R0X23) &
		DRV2624_R0X23_BEMF_GAIN_MSK;
	if (nResult < 0) {
		dev_err(pDRV2624->dev, "%s:%u: read cal_gain failed\n",
			__func__, __LINE__);
		goto end;
	}
	pDRV2624->mAutoCalData.mnCalGain = nResult;
	nResult = drv2624_reg_read(pDRV2624, DRV2624_R0X05_LRA_PERIOD_H);
	if (nResult < 0) {
		dev_err(pDRV2624->dev, "%s:%u: read f0_msb failed\n",
			__func__, __LINE__);
		goto end;
	}
	pDRV2624->mAutoCalData.mnLraPeriod.msb =
		nResult & DRV2624_R0X05_LRA_PERIOD_H_MSK;

	nResult = drv2624_reg_read(pDRV2624, DRV2624_R0X06_LRA_PERIOD_L);
	if (nResult < 0) {
		dev_err(pDRV2624->dev, "%s:%u: read f0_lsb failed\n",
			__func__, __LINE__);
		goto end;
	}
	pDRV2624->mAutoCalData.mnLraPeriod.lsb = nResult;

	lra_reg = pDRV2624->mAutoCalData.mnLraPeriod.msb << 8 |
		pDRV2624->mAutoCalData.mnLraPeriod.lsb;

	if(lra_reg) {
		calibrated_f0 = LRA_PERIOD_SCALE_UNIT/
			(lra_reg * LRA_PERIOD_SCALE_NS);
		if(calibrated_f0 >= pPlatData->msActuator.mnDTSLRAminFreq &&
			calibrated_f0 <= pPlatData->msActuator.mnDTSLRAmaxFreq) {
			ol_lra_reg =
				(OL_LRA_PERIOD_FACTOR*lra_reg*LRA_PERIOD_SCALE_NS)/
				OL_LRA_PERIOD_SCALE_NSX2;
			pDRV2624->mAutoCalData.mnCalibaredF0 = calibrated_f0;
			pDRV2624->mAutoCalData.mnOL_LraPeriod.msb =
				(ol_lra_reg >> 8) & DRV2624_R0X2E_OL_LRA_PERIOD_H_MSK;
			pDRV2624->mAutoCalData.mnOL_LraPeriod.lsb =
				ol_lra_reg & DRV2624_R0X2F_OL_LRA_PERIOD_L_MSK;
			dev_info(pDRV2624->dev, "%s:%u: calibrated_f0 = %u "
				"lra_period_h = 0x%02x lra_period_l = 0x%02x "
				"ol_lra_period_h = 0x%02x ol_lra_period_l = 0x%02x\n",
				__func__, __LINE__, calibrated_f0,
				pDRV2624->mAutoCalData.mnLraPeriod.msb,
				pDRV2624->mAutoCalData.mnLraPeriod.lsb,
				pDRV2624->mAutoCalData.mnOL_LraPeriod.msb,
				pDRV2624->mAutoCalData.mnOL_LraPeriod.lsb);
		} else {
			pDRV2624->mAutoCalData.mnDoneFlag =
				DRV26XX_CALIBRATED_FRES_INVALID;
		}
	}else {
		dev_err(pDRV2624->dev,"%s:%u: f0 is zero\n",
			__func__, __LINE__);
	}
end:
	nResult = drv2624_set_go_bit(pDRV2624, DRV2624_R0X0C_NGO_BIT);
	if (nResult < 0) {
		dev_err(pDRV2624->dev,
			"%s: cali NGO bit set failed, nResult = %d\n",
			__func__, nResult);
	}
	/* updata f0*/
	if(ol_lra_reg) {
		nResult = drv262x_update_f0(pDRV2624);
		pDRV2624->mAutoCalData.mnCnt++;
	}
	return nResult;
}

static int drv2624_get_diag_result(
	struct drv2624_data *pDRV2624)
{
	int nResult = -1;

	if(pDRV2624) {
		nResult = drv2624_reg_read(pDRV2624, DRV2624_R0X01_STATUS);
		if (nResult < 0) {
			dev_err(pDRV2624->dev, "%s:%u: Diagnostic fail\n",
				__func__, __LINE__);
			goto end;
		}
		pDRV2624->mDiagResult.mnResult =
			nResult & DRV2624_R0X01_STATUS_DIAG_RESULT_MSK;
		if (pDRV2624->mDiagResult.mnResult) {
			dev_err(pDRV2624->dev, "%s:%u: Diagnostic fail\n",
				__func__, __LINE__);
			goto end;
		} else {
			nResult = drv2624_reg_read(pDRV2624,
				DRV2624_RX03_DIAG_Z_RESULT);
			if (nResult < 0) {
				dev_err(pDRV2624->dev, "%s:%u: Read DIAG_Z fail\n",
					__func__, __LINE__);
				goto end;
			}
			pDRV2624->mDiagResult.mnDiagZ = nResult;

			nResult = drv2624_reg_read(pDRV2624,
				DRV2624_R0X30_CURRENT_K);
			if (nResult < 0) {
				dev_err(pDRV2624->dev, "%s:%u: Read Current_K fail\n",
					__func__, __LINE__);
				goto end;
			}
			pDRV2624->mDiagResult.mnCurrentK = nResult;

			pDRV2624->mDiagResult.mnRemohm = 478430 *
				pDRV2624->mDiagResult.mnDiagZ /
				(4 * pDRV2624->mDiagResult.mnCurrentK + 719);
			pDRV2624->mDiagResult.mnCnt++;
			dev_info(pDRV2624->dev,
				"%s: ZResult=0x%02x, CurrentK=0x%02x, Re = %d mohm\n",
				__func__, pDRV2624->mDiagResult.mnDiagZ,
				pDRV2624->mDiagResult.mnCurrentK,
				pDRV2624->mDiagResult.mnRemohm);
			nResult = 0;
		}
	}else {
		dev_err(pDRV2624->dev, "%s:%u: drv2624_data is NULL\n",
			__func__, __LINE__);
	}
end:
	return nResult;
}

static int drv262x_run_diagnostics(
	struct drv2624_data *pDRV2624)
{
	int nResult = 0;
	dev_info(pDRV2624->dev, "%s\n", __func__);
	nResult = drv2624_set_bits(pDRV2624,
		DRV2624_R0X07, DRV2624_R0X07_MODE_MSK,
		DRV2624_R0X07_MODE_DIAG_RTN);
	if (nResult < 0) {
		dev_err(pDRV2624->dev, "%s:%u: Diag start failed\n", __func__,
			__LINE__);
		goto end;
	}
	drv2624_reg_read(pDRV2624, DRV2624_R0X07);

	nResult = drv2624_set_go_bit(pDRV2624, DRV2624_R0X0C_GO_BIT);
	if (nResult < 0) {
		dev_err(pDRV2624->dev,
			"%s:%u: calibrate go bit not Done nResult = %d\n",
			__func__, __LINE__, nResult);
		goto end;
	}
end:
	return nResult;
}

static int dev_run_diagnostics(struct drv2624_data *pDRV2624)
{
	int nResult = 0;

	dev_info(pDRV2624->dev, "%s\n", __func__);
#ifdef WKLOCK
	wake_lock(&pDRV2624->wklock);
#endif
	if (g_logEnable)
		dev_dbg(pDRV2624->dev, "wklock lock");

	nResult = drv2624_change_mode(pDRV2624, MODE_DIAGNOSTIC);
	if (nResult < 0) 
		goto end;

	nResult = drv2624_set_go_bit(pDRV2624, GO);
	if (nResult < 0)
		goto end;

	dev_dbg(pDRV2624->dev, "Diag start\n");
	pDRV2624->mnVibratorPlaying = YES;

	if (!pDRV2624->mbIRQUsed) {
		pDRV2624->mnWorkMode |= WORK_DIAGNOSTIC;
		schedule_work(&pDRV2624->vibrator_work);
	}

end:
	if (nResult < 0) {
#ifdef WKLOCK
		wake_unlock(&pDRV2624->wklock);
#endif
		if (g_logEnable)
			dev_dbg(pDRV2624->dev, "wklock unlock");
	}
	return nResult;
}

static int drv2624_playEffect(struct drv2624_data *pDRV2624)
{
	int nResult = 0;

	dev_info(pDRV2624->dev, "%s\n", __func__);
#ifdef WKLOCK
	wake_lock(&pDRV2624->wklock);
#endif
	if (g_logEnable)
		dev_dbg(pDRV2624->dev, "wklock lock");

	nResult = drv2624_change_mode(pDRV2624, MODE_WAVEFORM_SEQUENCER);
	if (nResult < 0) 
		goto end;

	nResult = drv2624_set_go_bit(pDRV2624, GO);
	if (nResult < 0) 
		goto end;

	dev_dbg(pDRV2624->dev, "effects start\n");
	pDRV2624->mnVibratorPlaying = YES;

	if (!pDRV2624->mbIRQUsed) {
		pDRV2624->mnWorkMode |= WORK_EFFECTSEQUENCER;
		schedule_work(&pDRV2624->vibrator_work);
	}

end:
	if (nResult < 0) {
#ifdef WKLOCK
		wake_unlock(&pDRV2624->wklock);
#endif
		dev_dbg(pDRV2624->dev, "wklock unlock");
	}
	return nResult;
}

static int drv2624_stop(struct drv2624_data *pDRV2624)
{
	int nResult = 0;

	if (pDRV2624->mnVibratorPlaying == YES) {
		dev_err(pDRV2624->dev, "%s\n", __func__);
		if (pDRV2624->mbIRQUsed)
			drv2624_disableIRQ(pDRV2624);
		if (hrtimer_active(&pDRV2624->haptics_timer))
			hrtimer_cancel(&pDRV2624->haptics_timer);
		nResult = drv2624_set_go_bit(pDRV2624, STOP);
	}

	return nResult;
}

static int drv2624_rtpvibrate(struct drv2624_data *pDRV26xx,
	unsigned int waveform_id)
{
	int rc = 0, j = 0;
	struct drv26xx_RTPwaveform_info **rtp_wvfm_info =
		pDRV26xx->rtpwvfm.rtp_wvfm_info;
	struct drv26xx_RTPwaveforms* wvfm = &(pDRV26xx->rtpwvfm);

	pr_err("%s:%u enter",__func__,__LINE__);
	if(waveform_id >= pDRV26xx->rtpwvfm.nWaveforms) {
		rc = -1;
		pr_err("L:%u:WavId is out of range rc = %d\n",
			__LINE__, rc);
		goto EXIT;
	}

	drv2624_set_bits(pDRV26xx, DRV2624_R0X08,
		DRV2624_R0X08_LRA_ERM_MSK,
		(rtp_wvfm_info[waveform_id]->actuator_type == 0) ?
		DRV2624_R0X08_LRA : DRV2624_R0X08_ERM);

	drv2624_set_bits(pDRV26xx, DRV2624_R0X07,
		DRV2624_R0X07_TRIG_PIN_FUNC_MSK|DRV2624_R0X07_MODE_MSK,
		DRV2624_R0X07_TRIG_PIN_FUNC_EXTERNAL_PULSE|
		DRV2624_R0X07_MODE_RTP_MODE);

	drv2624_set_bits(pDRV26xx, DRV2624_R0X23,
		DRV2624_R0X23_FB_BRAKE_FACTOR_MSK,
		(wvfm->fb_brake_factor<<DRV2624_R0X23_FB_BRAKE_FACTOR_SFT)&
		DRV2624_R0X23_FB_BRAKE_FACTOR_MSK);
	drv2624_set_bits(pDRV26xx, DRV2624_R0X08,
		DRV2624_R0X08_HYBRID_LOOP_MSK|DRV2624_R0X08_AUTO_BRK_OL_MSK|
		DRV2624_R0X08_AUTO_BRK_INTO_STBY_MSK,
		((wvfm->hybrid_loop<<DRV2624_R0X08_HYBRID_LOOP_SFT)&
		DRV2624_R0X08_HYBRID_LOOP_MSK)|
		((wvfm->auto_brake<<DRV2624_R0X08_AUTO_BRK_OL_SFT)&
		DRV2624_R0X08_AUTO_BRK_OL_MSK)|
		((wvfm->auto_brake_standby<<
		DRV2624_R0X08_AUTO_BRK_INTO_STBY_SFT)&
		DRV2624_R0X08_AUTO_BRK_INTO_STBY_MSK));
	drv2624_set_bits(pDRV26xx, DRV2624_R0X23,
		DRV2624_R0X23_FB_BRAKE_FACTOR_MSK,
		(wvfm->fb_brake_factor<<DRV2624_R0X23_FB_BRAKE_FACTOR_SFT)&
		DRV2624_R0X23_FB_BRAKE_FACTOR_MSK);
	drv2624_set_bits(pDRV26xx, DRV2624_R0X29,
					DRV2624_R0X29_SAMPLE_TIME_MSK,
					wvfm->mnSampleTimeRegbits);

	drv2624_set_bits(pDRV26xx, DRV2624_R0X08,
		DRV2624_R0X08_AUTO_BRK_OL_MSK | DRV2624_R0X08_CTL_LOOP_MSK,
		((rtp_wvfm_info[waveform_id]->brake<<
		DRV2624_R0X08_AUTO_BRK_OL_SFT)&DRV2624_R0X08_AUTO_BRK_OL_MSK)|
		(((rtp_wvfm_info[waveform_id]->loop_mod != 1)<<
		DRV2624_R0X08_CTL_LOOP_SFT)&DRV2624_R0X08_CTL_LOOP_MSK));
	drv2624_set_bits(pDRV26xx, DRV2624_R0X2C,
		DRV2624_R0X2C_LRA_WAVE_SHAPE_MSK,
		(rtp_wvfm_info[waveform_id]->wv_shape != 1) &
		DRV2624_R0X2C_LRA_WAVE_SHAPE_MSK);

	if(rtp_wvfm_info[waveform_id]->loop_mod == 0) {
		//open loop
		drv2624_reg_write(pDRV26xx, DRV2624_R0X20_OD_CLAMP,
			rtp_wvfm_info[waveform_id]->
			mnOLOverDriveClampVoltageRegbits);
	} else {
		drv2624_reg_write(pDRV26xx, DRV2624_R0X1F_RATED_VOLTAGE,
			rtp_wvfm_info[waveform_id]->mnCLRatedVoltageRegbits);
		drv2624_reg_write(pDRV26xx, DRV2624_R0X20_OD_CLAMP,
			rtp_wvfm_info[waveform_id]->
			mnCLOverDriveClampVoltageRegbits);
	}

	drv2624_reg_write(pDRV26xx, DRV2624_R0X0E_RTP_INPUT, 0);
	rc = drv2624_set_go_bit(pDRV26xx, DRV2624_R0X0C_GO_BIT);
	if(rc != 0) {
		pr_err("L:%u:I2C set bit failed rc = %d\n",
			__LINE__, rc);
		goto EXIT;
	}

	for (j = 0; j < rtp_wvfm_info[waveform_id]->length; j++) {
		drv2624_reg_write(pDRV26xx, DRV2624_R0X0E_RTP_INPUT,
			rtp_wvfm_info[waveform_id]->wvfm_cmds[j].gain);
		msleep(rtp_wvfm_info[waveform_id]->wvfm_cmds[j].gain);
	}
	pr_err("%s:%u quit",__func__,__LINE__);
	drv2624_stop(pDRV26xx);

#if 0
	msleep(140);
	rc = drv2624_set_go_bit(pDRV2624, DRV2624_R0X0C_NGO_BIT);;
	if(rc != 0) {
		printf("L:%u:I2C set bit failed rc = %d\n", __LINE__, rc);
		goto EXIT;
	}
#endif

EXIT:
	return rc;

}

static int drv2624_config_waveform(struct drv2624_data *pDRV2624,
	struct drv2624_wave_setting *psetting)
{
	int nResult = 0;
	int value = 0;

	nResult = drv2624_reg_write(pDRV2624,
			DRV2625_REG_MAIN_LOOP, psetting->mnLoop & 0x07);
	if (nResult >= 0) {
		value |= ((psetting->mnInterval & 0x01) << INTERVAL_SHIFT);
		value |= (psetting->mnScale & 0x03);
		nResult = drv2624_set_bits(pDRV2624, DRV2625_REG_CONTROL2,
					INTERVAL_MASK | SCALE_MASK, value);
	}
	return nResult;
}

static int drv2624_set_waveform(struct drv2624_data *pDRV2624,
	struct drv2624_waveform_sequencer *pSequencer)
{
	int nResult = 0;
	int i = 0;
	unsigned char loop[2] = {0};
	unsigned char effects[DRV2625_SEQUENCER_SIZE] = {0};
	unsigned char len = 0;

	for (i = 0; i < DRV2625_SEQUENCER_SIZE; i++) {
		len++;
		if (pSequencer->msWaveform[i].mnEffect != 0) {
			if (i < 4) 
				loop[0] |= (pSequencer->msWaveform[i].mnLoop << (2*i));
			else
				loop[1] |= (pSequencer->msWaveform[i].mnLoop << (2*(i-4)));

			effects[i] = pSequencer->msWaveform[i].mnEffect;
		} else
			break;
	}

	if (len == 1)
		nResult = drv2624_reg_write(pDRV2624, DRV2625_REG_SEQUENCER_1, 0);
	else 
		nResult = drv2624_bulk_write(pDRV2624, DRV2625_REG_SEQUENCER_1, effects, len);

	if (nResult < 0) { 
		dev_err(pDRV2624->dev, "sequence error\n");
		goto end;
	}

	if (len > 1) {
		if ((len-1) <= 4)
			drv2624_reg_write(pDRV2624, DRV2625_REG_SEQ_LOOP_1, loop[0]);
		else
			drv2624_bulk_write(pDRV2624, DRV2625_REG_SEQ_LOOP_1, loop, 2);
	}

end:

	return nResult;
}

static ssize_t drv2624_file_read(struct file* filp, char* buff, size_t length, loff_t* offset)
{
	struct drv2624_data *pDRV2624 = (struct drv2624_data *)filp->private_data;
	int nResult = 0;
	unsigned char value = 0;
	unsigned char *p_kBuf = NULL;

	mutex_lock(&pDRV2624->haptic_lock);

	switch (pDRV2624->mnFileCmd) {
	case HAPTIC_CMDID_REG_READ:
		if (length == 1) {
			nResult = drv2624_reg_read(pDRV2624, pDRV2624->mnCurrentReg);
			if (nResult >= 0) {
				value = nResult;
				nResult = copy_to_user(buff, &value, 1);
				if (0 != nResult) {
					/* Failed to copy all the data, exit */
					dev_err(pDRV2624->dev, "copy to user fail %d\n", nResult);
				}
			}
		} else if (length > 1) {
			p_kBuf = (unsigned char *)kzalloc(length, GFP_KERNEL);
			if (p_kBuf != NULL) {
				nResult = drv2624_bulk_read(pDRV2624,
					pDRV2624->mnCurrentReg, p_kBuf, length);
				if (nResult >= 0) {
					nResult = copy_to_user(buff, p_kBuf, length);
					if (0 != nResult) {
						/* Failed to copy all the data, exit */
						dev_err(pDRV2624->dev, "copy to user fail %d\n", nResult);
					}
				}

				kfree(p_kBuf);
			} else {
				dev_err(pDRV2624->dev, "read no mem\n");
				nResult = -ENOMEM;
			}
		}
		break;

	case HAPTIC_CMDID_RUN_DIAG:
		if (pDRV2624->mnVibratorPlaying)
			length = 0;
		else {
			unsigned char buf[3];

			buf[0] = pDRV2624->mDiagResult.mnResult;
			buf[1] = pDRV2624->mDiagResult.mnDiagZ;
			buf[2] = pDRV2624->mDiagResult.mnDiagK;
			nResult = copy_to_user(buff, buf, 3);
			if (0 != nResult) {
				/* Failed to copy all the data, exit */
				dev_err(pDRV2624->dev, "copy to user fail %d\n", nResult);
			}
		}
		break;

	case HAPTIC_CMDID_RUN_CALIBRATION:
		if (pDRV2624->mnVibratorPlaying)
			length = 0;
		else {
			unsigned char buf[4];

			buf[0] = pDRV2624->mAutoCalResult.mnResult;
			buf[1] = pDRV2624->mAutoCalResult.mnCalComp;
			buf[2] = pDRV2624->mAutoCalResult.mnCalBemf;
			buf[3] = pDRV2624->mAutoCalResult.mnCalGain;
			nResult = copy_to_user(buff, buf, 4);
			if (0 != nResult) {
				/* Failed to copy all the data, exit */
				dev_err(pDRV2624->dev, "copy to user fail %d\n", nResult);
			}
		}
		break;

	case HAPTIC_CMDID_CONFIG_WAVEFORM:
		if (length == sizeof(struct drv2624_wave_setting)) {
			struct drv2624_wave_setting wavesetting;

			value = drv2624_reg_read(pDRV2624, DRV2625_REG_CONTROL2);
			wavesetting.mnLoop = drv2624_reg_read(pDRV2624, DRV2625_REG_MAIN_LOOP)&0x07;
			wavesetting.mnInterval = ((value&INTERVAL_MASK)>>INTERVAL_SHIFT);
			wavesetting.mnScale = (value&SCALE_MASK);
			nResult = copy_to_user(buff, &wavesetting, length);
			if (0 != nResult) {
				/* Failed to copy all the data, exit */
				dev_err(pDRV2624->dev, "copy to user fail %d\n", nResult);
			}
		}
		break;

	case HAPTIC_CMDID_SET_SEQUENCER:
		if (length == sizeof(struct drv2624_waveform_sequencer)) {
			struct drv2624_waveform_sequencer sequencer;
			unsigned char effects[DRV2625_SEQUENCER_SIZE] = {0};
			unsigned char loop[2] = {0};
			int i = 0;

			nResult = drv2624_bulk_read(pDRV2624, DRV2625_REG_SEQUENCER_1,
						effects, DRV2625_SEQUENCER_SIZE);
			if (nResult < 0)
				break;

			nResult = drv2624_bulk_read(pDRV2624, DRV2625_REG_SEQ_LOOP_1, loop, 2);
			if (nResult < 0)
				break;

			for (i = 0; i < DRV2625_SEQUENCER_SIZE; i++) {
				sequencer.msWaveform[i].mnEffect = effects[i];
				if (i < 4)
					sequencer.msWaveform[i].mnLoop = ((loop[0]>>(2*i))&0x03);
				else
					sequencer.msWaveform[i].mnLoop = ((loop[1]>>(2*(i-4)))&0x03);
			}

			nResult = copy_to_user(buff, &sequencer, length);
			if (0 != nResult) {
				/* Failed to copy all the data, exit */
				dev_err(pDRV2624->dev, "copy to user fail %d\n", nResult);
			}
		}
		break;
		
	case HAPTIC_CMDID_REGLOG_ENABLE:
		if (length == 1) {
			nResult = copy_to_user(buff, &g_logEnable, 1);
			if (0 != nResult) {
				/* Failed to copy all the data, exit */
				dev_err(pDRV2624->dev, "copy to user fail %d\n", nResult);
			}
		}
		break;

	default:
		pDRV2624->mnFileCmd = 0;
		break;
	}

	mutex_unlock(&pDRV2624->haptic_lock);

    return length;
}

static ssize_t drv2624_file_write(struct file* filp, const char* buff, size_t len, loff_t* off)
{
	struct drv2624_data *pDRV2624 = (struct drv2624_data *)filp->private_data;
	unsigned char *p_kBuf = NULL;
	int nResult = 0;

	mutex_lock(&pDRV2624->haptic_lock);

	p_kBuf = (unsigned char *)kzalloc(len, GFP_KERNEL);
	if (p_kBuf == NULL) {
		dev_err(pDRV2624->dev, "write no mem\n");
		goto err;
	}

	nResult = copy_from_user(p_kBuf, buff, len);
	if (0 != nResult) {
		dev_err(pDRV2624->dev,"copy_from_user failed.\n");
		goto err;
	}

	pDRV2624->mnFileCmd = p_kBuf[0];

	switch(pDRV2624->mnFileCmd) {
	case HAPTIC_CMDID_REG_READ:
		if (len == 2)
			pDRV2624->mnCurrentReg = p_kBuf[1];
		else
			dev_err(pDRV2624->dev, " read cmd len %d err\n", len);
		break;

	case HAPTIC_CMDID_REG_WRITE:
		if ((len - 1) == 2)
			nResult = drv2624_reg_write(pDRV2624, p_kBuf[1], p_kBuf[2]);
		else if ((len - 1) > 2)
			nResult = drv2624_bulk_write(pDRV2624, p_kBuf[1], &p_kBuf[2], len-2);
		else
			dev_err(pDRV2624->dev, "%s, reg_write len %d error\n", __func__, len);
		break;

	case HAPTIC_CMDID_REG_SETBIT:
		if (len == 4)
			nResult = drv2624_set_bits(pDRV2624, p_kBuf[1], p_kBuf[2], p_kBuf[3]);
		else
			dev_err(pDRV2624->dev, "setbit len %d error\n", len);
		break;

	case HAPTIC_CMDID_RUN_DIAG:
		nResult = drv2624_stop(pDRV2624);
		if (nResult < 0)
			break;
		nResult = dev_run_diagnostics(pDRV2624);
		if ((nResult >= 0) && pDRV2624->mbIRQUsed)
			drv2624_enableIRQ(pDRV2624, NO);
		break;


	case HAPTIC_CMDID_RUN_CALIBRATION:
		nResult = drv2624_stop(pDRV2624);
		if (nResult < 0)
			break;

		nResult = drv262x_auto_calibrate(pDRV2624);
		if ((nResult >= 0) && pDRV2624->mbIRQUsed)
			drv2624_enableIRQ(pDRV2624, NO);
		break;

	case HAPTIC_CMDID_CONFIG_WAVEFORM:
		if (len == (1 + sizeof(struct drv2624_wave_setting))) {
			struct drv2624_wave_setting wavesetting;

			memcpy(&wavesetting, &p_kBuf[1], sizeof(struct drv2624_wave_setting));
			nResult = drv2624_config_waveform(pDRV2624, &wavesetting);
		} else
			dev_dbg(pDRV2624->dev, "pass cmd, prepare for read\n");
		break;

	case HAPTIC_CMDID_SET_SEQUENCER:
		if (len == (1 + sizeof(struct drv2624_waveform_sequencer))) {
			struct drv2624_waveform_sequencer sequencer;

			memcpy(&sequencer, &p_kBuf[1], sizeof(struct drv2624_waveform_sequencer));
			nResult = drv2624_set_waveform(pDRV2624, &sequencer);
		} else
			dev_dbg(pDRV2624->dev, "pass cmd, prepare for read\n");
		break;

	case HAPTIC_CMDID_PLAY_EFFECT_SEQUENCE:
		nResult = drv2624_stop(pDRV2624);
		if (nResult < 0)
			break;

		nResult = drv2624_playEffect(pDRV2624);
		if ((nResult >= 0) && pDRV2624->mbIRQUsed)
			drv2624_enableIRQ(pDRV2624, NO);
		break;

	case HAPTIC_CMDID_STOP:
		nResult = drv2624_stop(pDRV2624);
		break;

	case HAPTIC_CMDID_REGLOG_ENABLE:
		if (len == 2)
			g_logEnable = p_kBuf[1];
		break;

	default:
		dev_err(pDRV2624->dev, "%s, unknown cmd\n", __func__);
		break;
	}

err:
	if (p_kBuf != NULL)
		kfree(p_kBuf);

    mutex_unlock(&pDRV2624->haptic_lock);

    return len;
}

static int drv2624_file_open(struct inode *inode, struct file *file)
{
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	file->private_data = (void*)g_DRV2625data;
	return 0;
}

static int drv2624_file_release(struct inode *inode, struct file *file)
{
	file->private_data = (void*)NULL;
	module_put(THIS_MODULE);
	return 0;
}

static long drv2624_file_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct drv2624_data *pDRV2624 = file->private_data;
	int nResult = 0;

	mutex_lock(&pDRV2624->haptic_lock);

	dev_dbg(pDRV2624->dev, "ioctl 0x%x\n", cmd);

	switch (cmd) {

	}

	mutex_unlock(&pDRV2624->haptic_lock);

	return nResult;
}


static struct file_operations fops =
{
	.owner = THIS_MODULE,
	.read = drv2624_file_read,
	.write = drv2624_file_write,
	.unlocked_ioctl = drv2624_file_unlocked_ioctl,
	.open = drv2624_file_open,
	.release = drv2624_file_release,
};

static struct miscdevice drv2624_misc =
{
	.name = HAPTICS_DEVICE_NAME,
	.fops = &fops,
};

static ssize_t drv2624_calib_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
#ifdef LEDS_ARCH
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
#else
	struct input_dev *led_cdev = dev_get_drvdata(dev);
#endif
	struct drv2624_data *pDRV2624 = NULL;
	ssize_t len = 0;

	if(led_cdev) {
		pDRV2624 = container_of(led_cdev,
			struct drv2624_data, led_dev);
		if(pDRV2624) {
			dev_info(pDRV2624->dev,
				"%s:%u: Cal_Result: 0x%02x, mnCnt: %u, "
				"CalComp: 0x%02x, CalBemf: 0x%02x, "
				"CalGain: 0x%02x, f0_data_msb:0x%02x, "
				"f0_data_lsb:0x%02x\n", __func__, __LINE__,
				pDRV2624->mAutoCalData.mnDoneFlag,
				pDRV2624->mAutoCalData.mnCnt,
				pDRV2624->mAutoCalData.mnCalComp,
				pDRV2624->mAutoCalData.mnCalBemf,
				pDRV2624->mAutoCalData.mnCalGain,
				pDRV2624->mAutoCalData.mnOL_LraPeriod.msb,
				pDRV2624->mAutoCalData.mnOL_LraPeriod.lsb);

			switch (pDRV2624->mAutoCalData.mnDoneFlag) {
			case DRV26XX_CALIBRATION_NOK:
				len = scnprintf(buf, 32, "calibration failed!\n");
				break;
			case DRV26XX_NO_CALIBRATION:
				len = scnprintf(buf, 64, "Kindly run \"echo calib\" "
					"first!\n");
				break;
			case DRV26XX_CALIBRATED_FRES_INVALID:
				len = scnprintf(buf, 64, "calibration failed! "
					"-- F0 is out of range\n");
				break;
			default:
				len = scnprintf(buf, 256, "Calibration data is from "
					"0x%02x, CalComp:0x%02x, CalBemf:0x%02x, "
					"CalGain:0x%02x, f0_data_msb:0x%02x, "
					"f0_data_lsb:0x%02x, f0_data:%u\n",
					pDRV2624->mAutoCalData.mnDoneFlag,
					pDRV2624->mAutoCalData.mnCalComp,
					pDRV2624->mAutoCalData.mnCalBemf,
					pDRV2624->mAutoCalData.mnCalGain,
					pDRV2624->mAutoCalData.mnOL_LraPeriod.msb,
					pDRV2624->mAutoCalData.mnOL_LraPeriod.lsb,
					pDRV2624->mAutoCalData.mnCalibaredF0);
				break;
			}
		}else {
			pr_err("%s:%u: drv2624 is NULL\n", __func__, __LINE__);
		}
	}else {
		pr_err("%s:%u: led_dev is NULL\n", __func__, __LINE__);
	}
	return len;
}

/**
 * store calibration
 *
 **/
static ssize_t drv2624_calib_store (
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
#ifdef LEDS_ARCH
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
#else
	struct input_dev *led_cdev = dev_get_drvdata(dev);
#endif
	struct drv2624_data *pDRV2624 = NULL;
	int nResult = -1;

	if(led_cdev) {
		pDRV2624 = container_of(led_cdev,
			struct drv2624_data, led_dev);
		if(pDRV2624) {
			pDRV2624->bRTPmode = false;
			nResult = drv262x_auto_calibrate(pDRV2624);
			if (nResult<0) {
				dev_err(pDRV2624->dev,
					"%s:%u: run calibrate err nResult = %d\n",
					__func__, __LINE__, nResult);
			} else {
				if(pDRV2624->mbWork == true) {
					mutex_lock(&pDRV2624->haptic_lock);
					nResult =
						drv2624_get_calibration_result(pDRV2624);
					mutex_unlock(&pDRV2624->haptic_lock);
					if (nResult<0) {
						dev_err(pDRV2624->dev,
							"%s: %u: get calibration result "
							"err nResult=%d\n", __func__,
							__LINE__, nResult);
					}
				} else {
					pDRV2624->mAutoCalData.mnDoneFlag =
						DRV2624_R0X01_STATUS_DIAG_RESULT_NOK;
					dev_err(pDRV2624->dev,
						"%s: %u: get calibration failed "
						"probably by interrupt\n",
						__func__,__LINE__);
				}
			}
		} else {
			pr_err("%s:%u: drv2624 is NULL\n",
				__func__, __LINE__);
		}
	} else {
		pr_err("%s:%u: led_dev is NULL\n",
			__func__, __LINE__);
	}
	return count;
}

static ssize_t drv2624_diag_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
#ifdef LEDS_ARCH
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
#else
	struct input_dev *led_cdev = dev_get_drvdata(dev);
#endif
	struct drv2624_data *pDRV2624 = NULL;
	int retry = 9;
	int nResult = -1;

	if(led_cdev) {
		pDRV2624 = container_of(led_cdev,
			struct drv2624_data, led_dev);
		if(pDRV2624) {
			mutex_lock(&pDRV2624->haptic_lock);
			pDRV2624->bRTPmode = false;
			nResult = drv262x_run_diagnostics(pDRV2624);
			if (nResult<0) {
				dev_err(pDRV2624->dev,
					"%s: run diag err nResult=%d\n",__func__,
					nResult);
			}
			mutex_unlock(&pDRV2624->haptic_lock);

			while(retry && pDRV2624->mbWork == true) {
				retry--;
				msleep(200); /* waiting auto calibration finished */
			}
			if(pDRV2624->mbWork == true) {
				mutex_lock(&pDRV2624->haptic_lock);
				nResult = drv2624_get_diag_result(pDRV2624);

				nResult = drv2624_set_go_bit(pDRV2624,
					DRV2624_R0X0C_NGO_BIT);
				if (nResult < 0) {
					dev_err(pDRV2624->dev,
						"%s: diag go bit not Done nResult = %d\n",
						__func__, nResult);
				}
				mutex_unlock(&pDRV2624->haptic_lock);
			} else {
				pDRV2624->mDiagResult.mnResult =
					DRV2624_R0X01_STATUS_DIAG_RESULT_NOK;
				dev_err(pDRV2624->dev,
					"%s: %u: diag failed probably by "
					"interrupt\n", __func__,__LINE__);
			}
		} else {
			pr_err("%s:%u: drv2624 is NULL\n",
				__func__, __LINE__);
		}
	} else {
		pr_err("%s:%u: led_dev is NULL\n",
			__func__, __LINE__);
	}

	return count;
}

static ssize_t
drv2624_diag_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
#ifdef LEDS_ARCH
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
#else
	struct input_dev *led_cdev = dev_get_drvdata(dev);
#endif
	struct drv2624_data *pDRV2624 = NULL;
	ssize_t len = 0;

	if(led_cdev) {
		pDRV2624 = container_of(led_cdev,
			struct drv2624_data, led_dev);
		if(pDRV2624) {
			//mutex_lock(&pDRV2624->haptic_lock);
			dev_info(pDRV2624->dev,
				"%s:%u: mnResult: 0x%x, mnCnt: 0x%x,\
				mnDiagZ: 0x%x, mnCurrentK:0x%x,mnRemohm:%u mohm\n",
				__func__, __LINE__, pDRV2624->mDiagResult.mnResult,
				pDRV2624->mDiagResult.mnCnt,
				pDRV2624->mDiagResult.mnDiagZ,
				pDRV2624->mDiagResult.mnCurrentK,
				pDRV2624->mDiagResult.mnRemohm);
			len = scnprintf(buf, 50, "%x %x %u\n",
				pDRV2624->mDiagResult.mnDiagZ,
				pDRV2624->mDiagResult.mnCurrentK,
				pDRV2624->mDiagResult.mnRemohm);
			//mutex_unlock(&pDRV2624->haptic_lock);
		} else {
			pr_err("%s:%u: drv2624 is NULL\n", __func__, __LINE__);
		}
	} else {
		pr_err("%s:%u: led_dev is NULL\n", __func__, __LINE__);
	}
	return len;
}

static ssize_t
drv2624_reg_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
#ifdef LEDS_ARCH
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
#else
	struct input_dev *led_cdev = dev_get_drvdata(dev);
#endif
	struct drv2624_data *pDRV2624 = NULL;
	ssize_t len = 0;
	const int size = PAGE_SIZE;
	int n_result = 0;
	int i = 0;

	if(led_cdev) {
		pDRV2624 = container_of(led_cdev,
			struct drv2624_data, led_dev);
		if(pDRV2624) {
			for (i = 0; i <= 0x30; i++) {
				n_result = drv2624_reg_read(pDRV2624, i);
				if (n_result < 0) {
					dev_info(pDRV2624->dev,
						"%s:%u: read register failed\n",
						__func__, __LINE__);
					len += scnprintf(buf+len, size-len,
						"Read Register0x%02x failed\n", i);
					break;
				}
				//12 bytes
				if(len + 12 <= size) {
					len += scnprintf(buf+len,
						size-len, "R0x%02x:0x%02x\n",
						i, n_result);
				} else {
					dev_info(pDRV2624->dev,
						"%s:%u: mem is not enough: "
						"PAGE_SIZE = %lu\n", __func__,
						__LINE__, PAGE_SIZE);
					break;
				}
			}
			if(n_result >= 0) {
				for (i = 0xFD; i <= 0xFF; i++) {
					n_result = drv2624_reg_read(pDRV2624, i);
					if (n_result < 0) {
						dev_info(pDRV2624->dev,
							"%s:%u: read register failed\n",
							__func__, __LINE__);
						len += scnprintf(buf+len, size-len,
							"Read Register0x%02x failed\n", i);
						break;
					}
				//12 bytes
					if(len + 12 <= size) {
						len += scnprintf(buf+len, size-len,
							"R0x%02x:0x%02x\n",
							i, n_result);
					} else {
						dev_info(pDRV2624->dev,
							"%s:%u: mem is not enough: "
							"PAGE_SIZE = %lu\n",
							 __func__,__LINE__, PAGE_SIZE);
						break;
					}
				}
			}
	  } else {
		  pr_err("%s:%u: drv2624 is NULL\n", __func__, __LINE__);
	  }
  } else {
	  pr_err("%s:%u: led_dev is NULL\n", __func__, __LINE__);
  }

  return len;
}

static ssize_t drv2624_reg_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
#ifdef LEDS_ARCH
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
#else
	struct input_dev *led_cdev = dev_get_drvdata(dev);
#endif
	struct drv2624_data *pDRV2624 = NULL;
	int nResult = -1;
	unsigned char databuf[2] = {0, 0};

	if(led_cdev && count > 2) {
		pDRV2624 = container_of(led_cdev,
			struct drv2624_data, led_dev);
		if(pDRV2624) {
			if (sscanf(buf, "0x%hhx 0x%hhx", &databuf[0],
				&databuf[1]) == 2) {
				nResult = drv2624_reg_write(pDRV2624,
						(unsigned char)databuf[0],
						(unsigned char)databuf[1]);
				if(nResult) {
					pr_err("%s:%u: I2C error!\n",
						__func__, __LINE__);
				}
			} else {
				pr_err("%s:%u: can't scan the value\n",
					__func__, __LINE__);
			}
		} else {
			pr_err("%s:%u: drv2624 is NULL nResult=%d\n",
				__func__, __LINE__, nResult);
		}
	} else {
		pr_err("%s:%u: led_dev is NULL nResult=%d\n",
			__func__, __LINE__, nResult);
	}

	return nResult;
}

static ssize_t
drv2624_CalComp_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
#ifdef LEDS_ARCH
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
#else
	struct input_dev *led_cdev = dev_get_drvdata(dev);
#endif
	struct drv2624_data *pDRV2624 = NULL;
	ssize_t len = 0;

	if(led_cdev) {
		pDRV2624 = container_of(led_cdev,
			struct drv2624_data, led_dev);
		if(pDRV2624) {
			switch (pDRV2624->mAutoCalData.mnDoneFlag) {
			case DRV26XX_CALIBRATION_NOK:
				len = scnprintf(buf, 32, "calibration failed!\n");
				break;
			case DRV26XX_NO_CALIBRATION:
				len = scnprintf(buf, 64,
					"Kindly run \"echo calib\" first!\n");
				break;
			case DRV26XX_CALIBRATED_FRES_INVALID:
				len = scnprintf(buf, 64, "calibration failed! "
					"-- F0 is out of range\n");
				break;
			default:
				len = scnprintf(buf, 128,
					"Calibration data is from %s\n"
					"CalComp:0x%x\n",
					(DRV2624_R0X01_STATUS_DIAG_RESULT_OK ==
					pDRV2624->mAutoCalData.mnDoneFlag)?
					"calibration":"file",
					pDRV2624->mAutoCalData.mnCalComp);
				break;
			}
		} else {
			pr_err("%s:%u: drv2624 is NULL\n", __func__, __LINE__);
		}
	} else {
		pr_err("%s:%u: led_dev is NULL\n", __func__, __LINE__);
	}
	return len;
}

static ssize_t drv2624_CalComp_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
#ifdef LEDS_ARCH
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
#else
	struct input_dev *led_cdev = dev_get_drvdata(dev);
#endif
	struct drv2624_data *pDRV2624 = NULL;
	int nResult = -1;
	unsigned char data = 0x0;

	if(led_cdev && count > 2) {
		pDRV2624 = container_of(led_cdev,
			struct drv2624_data, led_dev);
		if(pDRV2624) {
			if (sscanf(buf, "0x%hhx", &data) == 1) {
				pDRV2624->mAutoCalData.mnDoneFlag = 0x1;

				nResult = drv2624_reg_write(pDRV2624,
					DRV2624_R0X21_CAL_COMP, data);
				if(nResult) {
					pr_err("%s:%u: I2C error!\n",
						__func__, __LINE__);
				} else {
					pDRV2624->mAutoCalData.mnCalComp = data;
				}
			} else {
				pr_err("%s:%u: can't scan the value\n",
							__func__, __LINE__);
			}
		} else {
			pr_err("%s:%u: drv2624 is NULL nResult=%d\n",
				__func__, __LINE__, nResult);
		}
	} else {
		pr_err("%s:%u: led_dev is NULL nResult=%d\n",
			__func__, __LINE__, nResult);
	}

	return count;
}

static ssize_t
drv2624_CalBemf_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
#ifdef LEDS_ARCH
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
#else
	struct input_dev *led_cdev = dev_get_drvdata(dev);
#endif
	struct drv2624_data *pDRV2624 = NULL;
	ssize_t len = 0;

	if(led_cdev) {
		pDRV2624 = container_of(led_cdev,
			struct drv2624_data, led_dev);
		if(pDRV2624) {
			switch (pDRV2624->mAutoCalData.mnDoneFlag) {
			case DRV26XX_CALIBRATION_NOK:
				len = scnprintf(buf, 32, "calibration failed!\n");
				break;
			case DRV26XX_NO_CALIBRATION:
				len = scnprintf(buf, 64, "Kindly run \"echo calib\" "
					"first!\n");
				break;
			case DRV26XX_CALIBRATED_FRES_INVALID:
				len = scnprintf(buf, 64, "calibration failed! "
					"-- F0 is out of range\n");
				break;
			default:
				len = scnprintf(buf, 128,
					"Calibration data is from %s\n"
					"CalBemf:0x%x\n",
					(DRV2624_R0X01_STATUS_DIAG_RESULT_OK ==
					pDRV2624->mAutoCalData.mnDoneFlag)?
					"calibration":"file",
					pDRV2624->mAutoCalData.mnCalBemf);
				break;
			}
		} else {
			pr_err("%s:%u: drv2624 is NULL\n", __func__, __LINE__);
		}
	} else {
		pr_err("%s:%u: led_dev is NULL\n", __func__, __LINE__);
	}
	return len;
}

static ssize_t drv2624_CalBemf_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
#ifdef LEDS_ARCH
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
#else
	struct input_dev *led_cdev = dev_get_drvdata(dev);
#endif
	struct drv2624_data *pDRV2624 = NULL;
	int nResult = -1;
	unsigned char data = 0x0;

	if(led_cdev && count > 2) {
		pDRV2624 = container_of(led_cdev,
			struct drv2624_data, led_dev);
		if(pDRV2624) {
			if (sscanf(buf, "0x%hhx", &data) == 1) {
				pDRV2624->mAutoCalData.mnDoneFlag = 0x1;

				nResult = drv2624_reg_write(pDRV2624,
					DRV2624_R0X22_CAL_BEMF, data);
				if(nResult) {
					pr_err("%s:%u: I2C error!\n",
						__func__, __LINE__);
				} else {
					pDRV2624->mAutoCalData.mnCalBemf = data;
				}
			} else {
				pr_err("%s:%u: can't scan the value\n",
							__func__, __LINE__);
			}
		} else {
			pr_err("%s:%u: drv2624 is NULL nResult=%d\n",
				__func__, __LINE__, nResult);
		}
	} else {
		pr_err("%s:%u: led_dev is NULL nResult=%d\n",
			__func__, __LINE__, nResult);
	}

	return count;
}

static ssize_t
drv2624_CalGain_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
#ifdef LEDS_ARCH
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
#else
	struct input_dev *led_cdev = dev_get_drvdata(dev);
#endif
	struct drv2624_data *pDRV2624 = NULL;
	ssize_t len = 0;

	if(led_cdev) {
		pDRV2624 = container_of(led_cdev,
			struct drv2624_data, led_dev);
		if(pDRV2624) {
			switch (pDRV2624->mAutoCalData.mnDoneFlag) {
			case DRV26XX_CALIBRATION_NOK:
				len = scnprintf(buf, 32, "calibration failed!\n");
				break;
			case DRV26XX_NO_CALIBRATION:
				len = scnprintf(buf, 64,
					"Kindly run \"echo calib\" first!\n");
				break;
			case DRV26XX_CALIBRATED_FRES_INVALID:
				len = scnprintf(buf, 64,
					"calibration failed! "
					"-- F0 is out of range\n");
				break;
			default:
				len = scnprintf(buf, 128,
					"Calibration data is from %s\n"
					"CalGain:0x%x\n",
					(DRV2624_R0X01_STATUS_DIAG_RESULT_OK ==
					pDRV2624->mAutoCalData.mnDoneFlag)?
					"calibration":"file",
					pDRV2624->mAutoCalData.mnCalGain);
				break;
			}
		} else {
			pr_err("%s:%u: drv2624 is NULL\n", __func__, __LINE__);
		}
	} else {
		pr_err("%s:%u: led_dev is NULL\n", __func__, __LINE__);
	}
	return len;
}

static ssize_t drv2624_CalGain_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
#ifdef LEDS_ARCH
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
#else
	struct input_dev *led_cdev = dev_get_drvdata(dev);
#endif
	struct drv2624_data *pDRV2624 = NULL;
	int nResult = -1;
	unsigned char data = 0x0;

	if(led_cdev && count > 1) {
		pDRV2624 = container_of(led_cdev,
			struct drv2624_data, led_dev);
		if(pDRV2624) {
			if (sscanf(buf, "0x%hhx", &data) == 1) {
				pDRV2624->mAutoCalData.mnDoneFlag = 0x1;

				nResult = drv2624_set_bits(pDRV2624,
					DRV2624_R0X23, DRV2624_R0X23_BEMF_GAIN_MSK, data);
				if(nResult) {
					pr_err("%s:%u: I2C error!\n", __func__, __LINE__);
				} else {
					pDRV2624->mAutoCalData.mnCalGain = data;
				}
			} else {
				pr_err("%s:%u: can't scan the value\n",
							__func__, __LINE__);
			}
		} else {
			pr_err("%s:%u: drv2624 is NULL nResult=%d\n",
				__func__, __LINE__, nResult);
		}
	} else {
		pr_err("%s:%u: led_dev is NULL nResult=%d\n",
			__func__, __LINE__, nResult);
	}

	return count;
}

static ssize_t
drv2624_f0_msb_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
#ifdef LEDS_ARCH
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
#else
	struct input_dev *led_cdev = dev_get_drvdata(dev);
#endif
	struct drv2624_data *pDRV2624 = NULL;
	ssize_t len = 0;

	if(led_cdev) {
		pDRV2624 = container_of(led_cdev,
			struct drv2624_data, led_dev);
		if(pDRV2624) {
			switch (pDRV2624->mAutoCalData.mnDoneFlag) {
			case DRV26XX_CALIBRATION_NOK:
				len = scnprintf(buf, 32, "calibration failed!\n");
				break;
			case DRV26XX_NO_CALIBRATION:
				len = scnprintf(buf, 64, "Kindly run \"echo calib\" "
					"first!\n");
				break;
			case DRV26XX_CALIBRATED_FRES_INVALID:
				len = scnprintf(buf, 64, "calibration failed! "
					"-- F0 is out of range\n");
				break;
			default:
				len = scnprintf(buf, 128,
					"Calibration data is from %s\n"
					"f0_msb:0x%x\n",
					(DRV2624_R0X01_STATUS_DIAG_RESULT_OK ==
					pDRV2624->mAutoCalData.mnDoneFlag)?
					"calibration":"file",
					pDRV2624->mAutoCalData.mnOL_LraPeriod.msb);
				break;
		}
		} else {
			pr_err("%s:%u: drv2624 is NULL\n", __func__, __LINE__);
		}
	} else {
		pr_err("%s:%u: led_dev is NULL\n", __func__, __LINE__);
	}
	return len;
}

static ssize_t drv2624_f0_msb_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
#ifdef LEDS_ARCH
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
#else
	struct input_dev *led_cdev = dev_get_drvdata(dev);
#endif
	struct drv2624_data *pDRV2624 = NULL;
	int nResult = -1;
	unsigned char data = 0x0;

	if(led_cdev && count > 1) {
		pDRV2624 = container_of(led_cdev,
			struct drv2624_data, led_dev);
		if(pDRV2624) {
			if (sscanf(buf, "0x%hhx", &data) == 1) {
				pDRV2624->mAutoCalData.mnDoneFlag = 0x1;

				nResult = drv2624_set_bits(pDRV2624,
					DRV2624_R0X2E_OL_LRA_PERIOD_H,
					DRV2624_R0X2E_OL_LRA_PERIOD_H_MSK, data);
				if(nResult) {
					pr_err("%s:%u: I2C error!\n",
						__func__, __LINE__);
				} else {
					pDRV2624->mAutoCalData.mnOL_LraPeriod.msb
						= data;
				}
			} else {
				pr_err("%s:%u: can't scan the value\n",
							__func__, __LINE__);
			}
		} else {
			pr_err("%s:%u: drv2624 is NULL nResult=%d\n",
				__func__, __LINE__, nResult);
		}
	} else {
		pr_err("%s:%u: led_dev is NULL nResult=%d\n",
			__func__, __LINE__, nResult);
	}

	return count;
}

static ssize_t
drv2624_f0_lsb_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
#ifdef LEDS_ARCH
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
#else
	struct input_dev *led_cdev = dev_get_drvdata(dev);
#endif
	struct drv2624_data *pDRV2624 = NULL;
	ssize_t len = 0;

	if(led_cdev) {
		pDRV2624 = container_of(led_cdev,
			struct drv2624_data, led_dev);
		if(pDRV2624) {
			switch (pDRV2624->mAutoCalData.mnDoneFlag) {
			case DRV26XX_CALIBRATION_NOK:
				len = scnprintf(buf, 32, "calibration failed!\n");
				break;
			case DRV26XX_NO_CALIBRATION:
				len = scnprintf(buf, 64, "Kindly run \"echo calib\" "
					"first!\n");
				break;
			case DRV26XX_CALIBRATED_FRES_INVALID:
				len = scnprintf(buf, 64, "calibration failed! "
					"-- F0 is out of range\n");
				break;
			default:
				len = scnprintf(buf, 128,
					"Calibration data is from %s\n"
					"f0_lsb:0x%x\n",
					(DRV2624_R0X01_STATUS_DIAG_RESULT_OK ==
					pDRV2624->mAutoCalData.mnDoneFlag)?
					"calibration":"file",
					pDRV2624->mAutoCalData.mnOL_LraPeriod.lsb);
				break;
			}
		} else {
			pr_err("%s:%u: drv2624 is NULL\n", __func__, __LINE__);
		}
	} else {
		pr_err("%s:%u: led_dev is NULL\n", __func__, __LINE__);
	}
	return len;
}

static ssize_t drv2624_f0_lsb_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
#ifdef LEDS_ARCH
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
#else
	struct input_dev *led_cdev = dev_get_drvdata(dev);
#endif
	struct drv2624_data *pDRV2624 = NULL;
	int nResult = -1;
	unsigned char data = 0x0;

	if(led_cdev && count > 1) {
		pDRV2624 = container_of(led_cdev,
			struct drv2624_data, led_dev);
		if(pDRV2624) {
			if (sscanf(buf, "0x%hhx", &data) == 1) {
				pDRV2624->mAutoCalData.mnDoneFlag = 0x1;

				nResult = drv2624_reg_write(pDRV2624,
					DRV2624_R0X2F_OL_LRA_PERIOD_L, data);
				if(nResult) {
					pr_err("%s:%u: I2C error!\n",
						__func__, __LINE__);
				} else {
					pDRV2624->mAutoCalData.mnOL_LraPeriod.lsb
						= data;
				}
			} else {
				pr_err("%s:%u: can't scan the value\n",
							__func__, __LINE__);
			}
		} else {
			pr_err("%s:%u: drv2624 is NULL nResult=%d\n",
				__func__, __LINE__, nResult);
		}
	} else {
		pr_err("%s:%u: led_dev is NULL nResult=%d\n",
			__func__, __LINE__, nResult);
	}

	return count;
}

static ssize_t
drv2624_f0_data_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
#ifdef LEDS_ARCH
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
#else
	struct input_dev *led_cdev = dev_get_drvdata(dev);
#endif
	struct drv2624_data *pDRV2624 = NULL;
	ssize_t len = 0;

	if(led_cdev) {
		pDRV2624 = container_of(led_cdev,
			struct drv2624_data, led_dev);
		if(pDRV2624) {
			switch (pDRV2624->mAutoCalData.mnDoneFlag) {
			case DRV26XX_CALIBRATION_NOK:
				len = scnprintf(buf, 32, "calibration failed!\n");
				break;
			case DRV26XX_NO_CALIBRATION:
				len = scnprintf(buf, 64, "Kindly run \"echo calib\" "
					"first!\n");
				break;
			case DRV26XX_CALIBRATED_FRES_INVALID:
				len = scnprintf(buf, 64, "calibration failed! "
					"-- F0 is out of range\n");
				break;
			default: {
					if(pDRV2624->mAutoCalData.mnCalibaredF0 != 0) {
						len = scnprintf(buf, 128,
							"f0 data is from %s, "
							"f0 = %u\n",
							(DRV2624_R0X01_STATUS_DIAG_RESULT_OK ==
							pDRV2624->mAutoCalData.mnDoneFlag)?
							"calibration":"file",
							pDRV2624->mAutoCalData.mnCalibaredF0);
					} else {
						len = scnprintf(buf, 128,
							"f0 data error, "
							"mnOL_LraPeriod is zero!\n");
					}
				}
				break;
			}
		} else {
			pr_err("%s:%u: drv2624 is NULL\n", __func__, __LINE__);
		}
	} else {
		pr_err("%s:%u: led_dev is NULL\n", __func__, __LINE__);
	}
	return len;
}

static ssize_t drv2624_f0_data_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
#ifdef LEDS_ARCH
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
#else
	struct input_dev *led_cdev = dev_get_drvdata(dev);
#endif
	struct drv2624_data *pDRV2624 = NULL;
	int nResult = -1;
	unsigned int data = 0x0;

	if(led_cdev && count > 1) {
		pDRV2624 = container_of(led_cdev,
			struct drv2624_data, led_dev);
		if(pDRV2624) {
			if (sscanf(buf, "%u", &data) == 1) {
				pDRV2624->mAutoCalData.mnDoneFlag = 0x1;
				pDRV2624->mAutoCalData.mnCalibaredF0 = data;
			} else {
				pr_err("%s:%u: can't scan the value\n",
							__func__, __LINE__);
			}
		} else {
			pr_err("%s:%u: drv2624 is NULL nResult=%d\n",
				__func__, __LINE__, nResult);
		}
	} else {
		pr_err("%s:%u: led_dev is NULL nResult=%d\n",
			__func__, __LINE__, nResult);
	}

	return count;
}

static ssize_t
drv2624_gain_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
#ifdef LEDS_ARCH
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
#else
	struct input_dev *led_cdev = dev_get_drvdata(dev);
#endif
	struct drv2624_data *pDRV2624 = NULL;
	ssize_t len = 0;

	if(led_cdev) {
		pDRV2624 = container_of(led_cdev,
			struct drv2624_data, led_dev);
		if(pDRV2624) {
			switch (pDRV2624->gain) {
			case 0:
				len = scnprintf(buf, 32, "100%% strength\n");
				break;
			case 1:
				len = scnprintf(buf, 32, "75%% strength\n");
				break;
			case 2:
				len = scnprintf(buf, 32, "50%% strength\n");
				break;
			case 3:
				len = scnprintf(buf, 32, "25%% strength\n");
				break;
			default:
				len = scnprintf(buf, 32, "Invalid Gain!\n");
				break;
			}
		} else {
			pr_err("%s:%u: drv2624 is NULL\n", __func__, __LINE__);
		}
	} else {
		pr_err("%s:%u: led_dev is NULL\n", __func__, __LINE__);
	}
	return len;
}

static ssize_t i2c_address_show(struct device *dev,
	  struct device_attribute *attr, char *buf)
{
#ifdef LEDS_ARCH
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
#else
	struct input_dev *led_cdev = dev_get_drvdata(dev);
#endif
   struct drv2624_data *pDRV2624 = NULL;
   struct i2c_client *pClient = NULL;
   const int size = 32;
   int n = 0;
   unsigned short addr = 0;
   if( led_cdev != NULL) {
   		pDRV2624 = container_of(led_cdev,
			struct drv2624_data, led_dev);
		if(pDRV2624) {
			pClient = pDRV2624->client;
			addr = pClient->addr;
		}
	}
   n += scnprintf(buf, size, "Active SmartPA-0x%02x\n\r", addr);

   return n;

}

ssize_t fwload_store(struct device *dev,
  struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef LEDS_ARCH
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
#else
	struct input_dev *led_cdev = dev_get_drvdata(dev);
#endif
	struct drv2624_data *pDRV2624 = NULL;
	int ret = 0;
	if(led_cdev) {
		pDRV2624 = container_of(led_cdev, 
			struct drv2624_data, led_dev);
		if(pDRV2624) {
			dev_info(pDRV2624->dev, "fwload: count = %ld\n", count);
			ret = request_firmware_nowait(THIS_MODULE, 
				FW_ACTION_HOTPLUG, RTP_BIN_FILE, pDRV2624->dev,
				GFP_KERNEL, pDRV2624, drv2624_rtp_load);
			if(ret) {
				dev_err(pDRV2624->dev, "load %s error = %d\n",
					RTP_BIN_FILE, ret);
			}
		} else {
			dev_err(pDRV2624->dev, "handle is NULL\n");
		}
	} else {
		dev_err(pDRV2624->dev, "cdev is NULL\n");
	}
	return count;
}

static ssize_t drv2624_gain_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
#ifdef LEDS_ARCH
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
#else
	struct input_dev *led_cdev = dev_get_drvdata(dev);
#endif
	struct drv2624_data *pDRV2624 = NULL;
	int nResult = -1;
	unsigned int val;

	nResult = kstrtouint(buf, 0, &val);
	if (nResult < 0)
		return nResult;

	if(led_cdev && count > 2) {
		pDRV2624 = container_of(led_cdev,
			struct drv2624_data, led_dev);
		if(pDRV2624) {
			switch(val){
			case 0:
				nResult = drv2624_set_bits(pDRV2624,
					DRV2624_R0X0D,
					DRV2624_R0X0D_DIG_MEM_GAIN_MASK,
					DRV2624_R0X0D_DIG_MEM_GAIN_STRENGTH_100);
			break;
			case 1:
				nResult = drv2624_set_bits(pDRV2624,
					DRV2624_R0X0D,
					DRV2624_R0X0D_DIG_MEM_GAIN_MASK,
					DRV2624_R0X0D_DIG_MEM_GAIN_STRENGTH_75);
			break;
			case 2:
				nResult = drv2624_set_bits(pDRV2624,
					DRV2624_R0X0D,
					DRV2624_R0X0D_DIG_MEM_GAIN_MASK,
					DRV2624_R0X0D_DIG_MEM_GAIN_STRENGTH_50);
			break;
			case 3:
				nResult = drv2624_set_bits(pDRV2624,
					DRV2624_R0X0D,
					DRV2624_R0X0D_DIG_MEM_GAIN_MASK,
					DRV2624_R0X0D_DIG_MEM_GAIN_STRENGTH_25);
			break;
			default:
				pr_err("%s:%u: Invalid haptic strength num!\n",
					__func__, __LINE__);
			break;
			}
			if (nResult == 0)
				pDRV2624->gain = val;
		} else {
			pr_err("%s:%u: drv2624 is NULL nResult=%d\n",
				__func__, __LINE__, nResult);
		}
	} else {
		pr_err("%s:%u: led_dev is NULL nResult=%d\n",
			__func__, __LINE__, nResult);
	}

	return count;
}

/*show name*/
static ssize_t
drv2624_NAME_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
#ifdef LEDS_ARCH
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
#else
	struct input_dev *led_cdev = dev_get_drvdata(dev);
#endif
	struct drv2624_data *pDRV2624 = container_of(led_cdev,
			struct drv2624_data, led_dev);
	int nResult = 0;
	nResult = drv2624_reg_read(pDRV2624, DRV2624_R0X00);
	if (nResult == DRV2624_R0X00_CHIP_ID_REV) {
		return snprintf(buf, 100, "TI-DRV2624\n");
	} else {
		return snprintf(buf, 100, "i2c-error\n");
	}
}

static ssize_t
drv2624_state_show(struct device *dev, struct device_attribute *attr,
		      char *buf)
{
	return 1;
}
static ssize_t drv2624_state_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	int nResult = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct drv2624_data *pDRV2624 = container_of(led_cdev,
			struct drv2624_data, led_dev);
	unsigned int val = 0;
	int rc = 0;;
	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val == 0) {
		nResult = drv2624_set_go_bit(pDRV2624, STOP);
		if (hrtimer_active(&pDRV2624->haptics_timer))
			hrtimer_cancel(&pDRV2624->haptics_timer);
		goto end;
	}

	dev_info(pDRV2624->dev, "val = %d\n", val);
	pDRV2624->play.length = val;
	if(val <= 100){
		val = 100;
	}
	drv2624_stop(pDRV2624);
	
	nResult = drv2624_change_mode(pDRV2624, MODE_WAVEFORM_SEQUENCER);
	if (nResult < 0) {
		dev_dbg(pDRV2624->dev, "%s: change_mode nResult = %d\n",
			__func__, nResult);
		goto end;
	}
	nResult = drv2624_reg_write(pDRV2624, DRV2625_REG_RTP_INPUT, 0x7f);
	if (nResult < 0)
		goto end;
	nResult = drv2624_set_go_bit(pDRV2624, GO);
	if (nResult < 0)
		goto end;
	if (pDRV2624->play.length != 0) {
		hrtimer_start(&pDRV2624->haptics_timer,
			      ns_to_ktime((u64) val * NSEC_PER_MSEC),
			      HRTIMER_MODE_REL);
	}
	pDRV2624->mnVibratorPlaying = YES;
end:	return size;
}

// static DEVICE_ATTR(state, 0664, drv2624_state_show, drv2624_state_store);
static DEVICE_ATTR(activate, 0664, drv2624_activate_show,
		   drv2624_activate_store);
static DEVICE_ATTR(duration, 0664, drv2624_duration_show,
		   drv2624_duration_store);
static DEVICE_ATTR(rtp_test, 0664, drv2624_rtp_test_show,
		   drv2624_rtp_test_store);
static DEVICE_ATTR(rtpbininfo_list, 0664, drv2624_rtpbininfo_list_show,
		   drv2624_rtpbininfo_list_store);
static DEVICE_ATTR(calib, 0664, drv2624_calib_show, drv2624_calib_store);
static DEVICE_ATTR(diag, 0664, drv2624_diag_show, drv2624_diag_store);
static DEVICE_ATTR(reg, 0664, drv2624_reg_show, drv2624_reg_store);
static DEVICE_ATTR(CalComp, 0664, drv2624_CalComp_show, drv2624_CalComp_store);
static DEVICE_ATTR(CalBemf, 0664, drv2624_CalBemf_show, drv2624_CalBemf_store);
static DEVICE_ATTR(CalGain, 0664, drv2624_CalGain_show, drv2624_CalGain_store);
static DEVICE_ATTR(f0_msb, 0664, drv2624_f0_msb_show, drv2624_f0_msb_store);
static DEVICE_ATTR(f0_lsb, 0664, drv2624_f0_lsb_show, drv2624_f0_lsb_store);
static DEVICE_ATTR(f0_data, 0664, drv2624_f0_data_show, drv2624_f0_data_store);
static DEVICE_ATTR(gain, 0664, drv2624_gain_show, drv2624_gain_store);
static DEVICE_ATTR(NAME, 0664, drv2624_NAME_show, NULL);
static DEVICE_ATTR(i2caddr, 0664, i2c_address_show, NULL);
static DEVICE_ATTR(fwload, 0664, NULL, fwload_store);
static DEVICE_ATTR(state, 0664, drv2624_state_show, drv2624_state_store);

static struct attribute *drv2624_dev_fs_attrs[] = {
	&dev_attr_state.attr,
	&dev_attr_activate.attr,
	&dev_attr_duration.attr,
	&dev_attr_rtp_test.attr,
	&dev_attr_rtpbininfo_list.attr,
	&dev_attr_gain.attr,
	&dev_attr_calib.attr,
	// &dev_attr_enable.attr,
	&dev_attr_diag.attr,
	&dev_attr_reg.attr,
	&dev_attr_CalComp.attr,
	&dev_attr_CalBemf.attr,
	&dev_attr_CalGain.attr,
	&dev_attr_f0_msb.attr,
	&dev_attr_f0_lsb.attr,
	&dev_attr_NAME.attr,
	&dev_attr_f0_data.attr,
	&dev_attr_i2caddr.attr,
	&dev_attr_fwload.attr,
	NULL,
};

static struct attribute_group drv2624_dev_fs_attr_group = {
	.attrs = drv2624_dev_fs_attrs,
};
#ifdef LEDS_ARCH
	static const struct attribute_group *drv2624_dev_fs_attr_groups[]
		= {
			&drv2624_dev_fs_attr_group,
			NULL,
		};
#endif

static void vibrator_work_routine(struct work_struct *work)
{
	struct drv2624_data *pDRV2624 =
	    container_of(work, struct drv2624_data, vibrator_work);
	unsigned char status;
	int nResult = 0;
	mutex_lock(&pDRV2624->haptic_lock);
	dev_err(pDRV2624->dev, "%s, afer mnWorkMode=0x%x\n", __func__,
		pDRV2624->mnWorkMode);
	if (pDRV2624->mbIRQUsed) {
		pDRV2624->mnIntStatus =
		    drv2624_reg_read(pDRV2624, DRV2625_REG_STATUS);
		if (nResult < 0)
			return;
		drv2624_disableIRQ(pDRV2624);
		status = pDRV2624->mnIntStatus;
		dev_dbg(pDRV2624->dev, "%s, status=0x%x\n", __func__,
			pDRV2624->mnIntStatus);
		if (status & OVERCURRENT_MASK)
			dev_err(pDRV2624->dev,
				"ERROR, Over Current detected!!\n");
		if (status & OVERTEMPRATURE_MASK)
			dev_err(pDRV2624->dev,
				"ERROR, Over Temperature detected!!\n");
		if (status & ULVO_MASK)
			dev_err(pDRV2624->dev, "ERROR, VDD drop observed!!\n");
		if (status & PRG_ERR_MASK)
			dev_err(pDRV2624->dev, "ERROR, PRG error!!\n");
	}
	if (pDRV2624->mnWorkMode == DRV2624_RTP_MODE) {
		dev_err(pDRV2624->dev, "%s: \n", __func__);
		drv2624_stop(pDRV2624);
	} else if (pDRV2624->mnWorkMode == DRV2624_RAM_MODE) {
		dev_err(pDRV2624->dev, "%s: read go bit\n", __func__);
		// status = drv2624_reg_read(pDRV2624, DRV2625_REG_GO);
		// if ((status < 0) || (status == STOP)
		//     || !pDRV2624->mnVibratorPlaying) {
		// 	dev_err(pDRV2624->dev, "%s: status error = %d\n",
		// 		__func__, status);
		// 	//RAM haptic already stoped, just update flag
		// 	drv2624_set_stopflag(pDRV2624);
		// } else {
		// 	if (!hrtimer_active(&pDRV2624->haptics_timer)) {
		// 		dev_dbg(pDRV2624->dev,
		// 			"will check GO bit after %d ms\n",
		// 			GO_BIT_CHECK_INTERVAL);
		// 		hrtimer_start(&pDRV2624->haptics_timer,
		// 			      ns_to_ktime((u64)
		// 					  GO_BIT_CHECK_INTERVAL
		// 					  * NSEC_PER_MSEC),
		// 			      HRTIMER_MODE_REL);
		// 	}
		// }
		drv2624_stop(pDRV2624);
	}
// err:
	mutex_unlock(&pDRV2624->haptic_lock);
}

/**
 * 1. Do work due to pDRV2624->mnWorkMode set before.
 * 2. For WORK_EFFECTSEQUENCER, WORK_CALIBRATION and WORK_DIAGNOSTIC
 *    check the GO bit until the process in DRV2624 has completed.
 * 3. For WORK_VIBRATOR, Stop DRV2624 directly.
 **/
static void vibrator_irq_routine(struct work_struct *work)
{
	struct drv2624_data *pDRV2624 =
	    container_of(work, struct drv2624_data, vibrator_irq_work);
	int nResult = 0, workmode = 0;
	bool bError = false;
	if(pDRV2624 == NULL) {
		dev_err(pDRV2624->dev, "%s:%u: drv2624_data is NULL\n",
			__func__, __LINE__);
		return;
	}

	if(pDRV2624->mbWork == false) {
		dev_info(pDRV2624->dev, "%s:%u: Read Go bit is STOP\n",
			__func__, __LINE__);
		goto EXIT;
	}
	workmode = drv2624_reg_read(pDRV2624, DRV2624_R0X07);
	if(workmode < 0) {
		dev_err(pDRV2624->dev,
			"%s:%u: Read DRV2624_R0X07 error\n", __func__,
			__LINE__);
		goto EXIT;
	} else {
		workmode &= DRV2624_R0X07_MODE_MSK;
		dev_info(pDRV2624->dev, "%s:%u:WorkMode=0x%x\n",
			__func__,__LINE__,
			workmode);
	}
	if(gpio_is_valid(pDRV2624->msPlatData.mnGpioINT) &&
		true == pDRV2624->mbIRQEnabled) {
		nResult =
		    drv2624_reg_read(pDRV2624, DRV2624_R0X01_STATUS);
		if (nResult < 0) {
			dev_err(pDRV2624->dev,
				"%s:%u: Read DRV2624_R0X01_STATUS error\n",
				__func__, __LINE__);
			goto EXIT;
		}
		drv2624_disableIRQ(pDRV2624);
		dev_info(pDRV2624->dev, "%s:%u: status=0x%x\n",
			__func__, __LINE__, nResult);
		if (nResult & DRV2624_R0X01_STATUS_OC_DETECT_MSK) {
			bError = true;
			dev_err(pDRV2624->dev,
				"ERROR, Over Current detected!!\n");
		}
		if (nResult & DRV2624_R0X01_STATUS_OVER_TEMP_MSK) {
			bError = true;
			dev_err(pDRV2624->dev,
				"ERROR, Over Temperature detected!!\n");
		}
		if (nResult & DRV2624_R0X01_STATUS_UVLO_MSK) {
			bError = true;
			dev_err(pDRV2624->dev, "ERROR, VDD drop observed!!\n");
		}
		if (nResult & DRV2624_R0X01_STATUS_PRG_ERROR_MSK) {
			bError = true;
			dev_err(pDRV2624->dev, "ERROR, PRG error!!\n");
		}
	}
	mutex_lock(&pDRV2624->haptic_lock);
	if(true == bError) nResult = drv2624_set_go_bit(pDRV2624,
		DRV2624_R0X0C_NGO_BIT);

	mutex_unlock(&pDRV2624->haptic_lock);
	switch (workmode) {
		case DRV2624_R0X07_MODE_RTP_MODE:
			break;
		case DRV2624_R0X07_MODE_WVFRM_SEQ_MODE:
			break;
		default:
			break;
	}
EXIT:
	return;
}

int drv2624_haptic_play_go(
	struct drv2624_data *pDRV2624,
	bool flag)
{
	int nResult = 0;
	dev_info(pDRV2624->dev,"%s enter, flag = %d\n", __func__, flag);
	if (!flag) {
		pDRV2624->current_ktime = ktime_get();
		dev_info(pDRV2624->dev,"%s:%u:pDRV2624->current_us=%lld\n",
			__func__, __LINE__, ktime_to_us(pDRV2624->current_ktime));

		pDRV2624->interval_us =
			ktime_to_us(ktime_sub(pDRV2624->current_ktime,
			pDRV2624->pre_enter_ktime));
		if (pDRV2624->interval_us < 2000) {
			dev_info(pDRV2624->dev,
				"%s:%u:pDRV2624->interval_us = %u\n",
				__func__, __LINE__, pDRV2624->interval_us);
			usleep_range(2000, 2000);
		}
	}
	if (true == flag) {
		if(pDRV2624->bRTPmode == false) {
			nResult = drv2624_set_bits(pDRV2624, DRV2624_R0X07,
				DRV2624_R0X07_MODE_MSK,
				DRV2624_R0X07_MODE_WVFRM_SEQ_MODE);
			if (nResult < 0) {
				dev_err(pDRV2624->dev, "%s:%u: I2C set error\n",
					__func__, __LINE__);
				goto end;
			}
		}
		nResult = drv2624_set_go_bit(pDRV2624, DRV2624_R0X0C_GO_BIT);
		if (nResult < 0) {
			dev_err(pDRV2624->dev,
				"%s:%u: go bit not Done nResult = %d\n",
				__func__, __LINE__, nResult);
		} else {
			pDRV2624->pre_enter_ktime = ktime_get();
			dev_info(pDRV2624->dev,
				"%s:%u: pDRV2624->current_us = %lld\n",
				__func__, __LINE__,
				ktime_to_us(pDRV2624->pre_enter_ktime));
		}
	} else {
		nResult = drv2624_set_go_bit(pDRV2624, DRV2624_R0X0C_NGO_BIT);
		if (nResult < 0) {
			dev_err(pDRV2624->dev,
				"%s:%u: go bit not Done nResult = %d\n",
				__func__, __LINE__, nResult);
		}
	}
end:
	return nResult;
}

static irqreturn_t drv2624_irq_handler(int irq, void *dev_id)
{
	struct drv2624_data *pDRV2624 = (struct drv2624_data *)dev_id;
	dev_info(pDRV2624->dev, "%s: enter\n", __func__);
	schedule_work(&pDRV2624->vibrator_irq_work);
	return IRQ_HANDLED;
}

static enum hrtimer_restart vibrator_timer_func(
	struct hrtimer *timer)
{
	struct drv2624_data *pDRV2624 =
	    container_of(timer, struct drv2624_data, haptics_timer);
	dev_info(pDRV2624->dev, "%s\n", __func__);
	pDRV2624->state = 0;
	schedule_work(&pDRV2624->vibrator_work);
	return HRTIMER_NORESTART;
}

static int drv2624_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct drv2624_data *pDRV2624 = NULL;
#ifndef LEDS_ARCH
	struct ff_device *ff;
#endif
	int nResult = 0;
	bool bLedRegister = false;
//	const struct firmware *fw_entry;

	dev_info(&client->dev, "%s enter\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s:%u:I2C check failed\n",
			__func__,__LINE__);
		nResult = -ENODEV;
		goto ERROR;
	}

	pDRV2624 = devm_kzalloc(&client->dev,
				sizeof(struct drv2624_data), GFP_KERNEL);
	if (pDRV2624 == NULL) {
		dev_err(&client->dev, "%s:%u:error\n", __func__,__LINE__);
		nResult = -ENOMEM;
		goto ERROR;
	}

	pDRV2624->dev = &client->dev;
	pDRV2624->client = client;
	pDRV2624->mbIRQEnabled = true;
	dev_set_drvdata(&client->dev, pDRV2624);
	pDRV2624->mpRegmap = devm_regmap_init_i2c(client,
		&drv2624_i2c_regmap);
	if (IS_ERR(pDRV2624->mpRegmap)) {
		nResult = PTR_ERR(pDRV2624->mpRegmap);
		dev_err(pDRV2624->dev,
		"%s:Failed to allocate register map: %d\n",
			__func__, nResult);
		goto ERROR;
	}

	if (client->dev.of_node) {
		dev_info(pDRV2624->dev, "of node parse\n");
		drv2624_parse_dt(&client->dev, pDRV2624);
	} else {
		dev_err(pDRV2624->dev, "%s: ERROR no platform data\n",
			__func__);
	}
	mutex_init(&pDRV2624->reg_lock);
	mutex_init(&pDRV2624->haptic_lock);
	if (gpio_is_valid(pDRV2624->msPlatData.mnGpioNRST)) {
		nResult =
		    gpio_request(pDRV2624->msPlatData.mnGpioNRST,
				 "DRV2624-NRST");
		if (nResult < 0) {
			dev_err(pDRV2624->dev,
				"%s:%u: GPIO %d request NRST error\n",
				__func__, __LINE__, pDRV2624->msPlatData.mnGpioNRST);
			goto ERROR;
		}
		drv2624_hw_reset(pDRV2624);
	}
	nResult = drv2624_reg_read(pDRV2624, DRV2624_R0X00);
	if (DRV2625_R0X00_CHIP_ID_REV == nResult) {
		dev_info(pDRV2624->dev, "%s:%u: CHIP_ID & REV = 0x%02x\n",
			__func__, __LINE__, nResult);
		pDRV2624->mnDeviceID = nResult;
	} else {
		dev_err(pDRV2624->dev, "%s:%u: device_id(0x%02x) fail\n",
			__func__, __LINE__,	nResult);
		goto ERROR;
	}
	drv2624_init(pDRV2624);
#ifdef LEDS_ARCH
	pDRV2624->led_dev.name = "drv26xx_haptic";
	pDRV2624->led_dev.brightness_set = drv2624_vibrator_enable;
	pDRV2624->led_dev.groups = drv2624_dev_fs_attr_groups;
	nResult = devm_led_classdev_register(&pDRV2624->client->dev,
		&pDRV2624->led_dev);
	nResult = misc_register(&drv2624_misc);
	if (nResult) {
		dev_err(pDRV2624->dev, "drv2624 misc fail: %d\n", nResult);
		return nResult;
	}
#else
	pDRV2624->led_dev.name = "drv26xx_haptic";
	pDRV2624->led_dev.close = drv2624_close;
	input_set_drvdata(&pDRV2624->led_dev, pDRV2624);
	input_set_capability(&pDRV2624->led_dev, EV_FF, FF_RUMBLE);
	input_set_capability(&pDRV2624->led_dev, EV_FF, FF_CONSTANT);
	input_set_capability(&pDRV2624->led_dev, EV_FF, FF_GAIN);
	input_set_capability(&pDRV2624->led_dev, EV_FF, FF_PERIODIC);
	input_set_capability(&pDRV2624->led_dev, EV_FF, FF_CUSTOM);
	nResult = input_ff_create(&pDRV2624->led_dev,
		DRV2624_EFFECT_MAX_NUM);
	if (nResult) {
		dev_err(pDRV2624->dev, "input_ff_create failed: %d\n",
			nResult);
		goto ERROR;
	}
	nResult = sysfs_create_group(&pDRV2624->dev->kobj,
		&drv2624_dev_fs_attr_group);
	if (nResult) {
		dev_err(pDRV2624->dev, "sysfs_create_group failed: %d\n",
			nResult);
		goto ERROR1;
	}
	ff = pDRV2624->led_dev.ff;
	ff->upload = drv2624_haptics_upload_effect;
	ff->playback = drv2624_haptics_playback;
	ff->erase = drv2624_haptics_erase;
	ff->set_gain = drv2624_haptics_set_gain;
	nResult = input_register_device(&(pDRV2624->led_dev));
#endif
    if (nResult) {
	    dev_err(pDRV2624->dev,"%s:%u:Failed to create haptic "
			"classdev: %d\n", __func__, __LINE__, nResult);
		goto ERROR;
	}
	bLedRegister = true;

	hrtimer_init(&pDRV2624->haptics_timer, CLOCK_MONOTONIC,
	     HRTIMER_MODE_REL);
	pDRV2624->haptics_timer.function = vibrator_timer_func;

	INIT_WORK(&pDRV2624->vibrator_work, vibrator_work_routine);
	INIT_WORK(&pDRV2624->vibrator_irq_work, vibrator_irq_routine);

	if (gpio_is_valid(pDRV2624->msPlatData.mnGpioINT)) {
		nResult =
			gpio_request(pDRV2624->msPlatData.mnGpioINT,
				"DRV2624-IRQ");
		if (nResult == 0) {
			gpio_direction_input(pDRV2624->msPlatData.mnGpioINT);
			pDRV2624->mnIRQ =
				gpio_to_irq(pDRV2624->msPlatData.mnGpioINT);
			dev_info(pDRV2624->dev, "irq = %u \n", pDRV2624->mnIRQ);
			nResult =
				request_threaded_irq(pDRV2624->mnIRQ,
					drv2624_irq_handler, NULL,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					client->name, pDRV2624);
			if (nResult == 0) {
				drv2624_disableIRQ(pDRV2624);
			} else {
				dev_err(pDRV2624->dev, "request_irq failed, %d\n",
					nResult);
			}
		} else {
			dev_err(pDRV2624->dev,
				"%s:%u: GPIO %d request INT error\n", __func__,
				__LINE__, pDRV2624->msPlatData.mnGpioINT);
		}
	}
	pDRV2624->mAutoCalData.mnDoneFlag = 0xFF;
	nResult = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
		RTP_BIN_FILE, &(client->dev),
		GFP_KERNEL, pDRV2624,
		drv2624_rtp_load);
	if (nResult != 0) {
		dev_err(&client->dev,
			"%s: %u: nResult = %d: request drv2624_rtp_load error, "
			"pls check!\n", __func__, __LINE__, nResult);
	}
#ifndef LEDS_ARCH
ERROR1:
	if(nResult) input_ff_destroy(&pDRV2624->led_dev);
#endif
ERROR:
	if(nResult == 0) {
		g_DRV2625data = pDRV2624;
		dev_info(&client->dev, "%s successfully!\n", __func__);
	} else {
		if(pDRV2624 != NULL) {
			mutex_destroy(&pDRV2624->reg_lock);
			mutex_destroy(&pDRV2624->haptic_lock);
			cancel_work_sync(&pDRV2624->vibrator_irq_work);
			cancel_work_sync(&pDRV2624->vibrator_work);
			if(bLedRegister) {
#ifdef LEDS_ARCH
				devm_led_classdev_unregister(&pDRV2624->client->dev,
					&pDRV2624->led_dev);
#else
				input_unregister_device(&pDRV2624->led_dev);
#endif
			}
			if (gpio_is_valid(pDRV2624->msPlatData.mnGpioINT))
				gpio_free(pDRV2624->msPlatData.mnGpioINT);
			if (gpio_is_valid(pDRV2624->msPlatData.mnGpioNRST))
				gpio_free(pDRV2624->msPlatData.mnGpioNRST);

			devm_kfree(&client->dev, pDRV2624);
			pDRV2624 = NULL;
		}
		dev_err(&client->dev, "%s failed!\n", __func__);
	}
	return nResult;
}

static void drv26xx_ram_waveform_remove
	(struct drv2624_data* pDRV26xx)
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

static int drv2624_remove(struct i2c_client *client)
{
	struct drv2624_data *pDRV2624 = i2c_get_clientdata(client);
	if(pDRV2624 != NULL) {
		hrtimer_cancel(&pDRV2624->haptics_timer);
		cancel_work_sync(&pDRV2624->vibrator_irq_work);
		cancel_work_sync(&pDRV2624->vibrator_work);
		mutex_destroy(&pDRV2624->reg_lock);
		mutex_destroy(&pDRV2624->haptic_lock);
#ifdef LEDS_ARCH
		devm_led_classdev_unregister(&pDRV2624->client->dev,
			&pDRV2624->led_dev);
#else
		input_unregister_device(&pDRV2624->led_dev);
		input_ff_destroy(&pDRV2624->led_dev);
		drv26xx_rtp_waveform_remove(pDRV2624);
#endif
		drv26xx_ram_waveform_remove(pDRV2624);
		if (gpio_is_valid(pDRV2624->msPlatData.mnGpioINT))
			gpio_free(pDRV2624->msPlatData.mnGpioINT);
		if (gpio_is_valid(pDRV2624->msPlatData.mnGpioNRST))
			gpio_free(pDRV2624->msPlatData.mnGpioNRST);
	}
	return 0;
}

static const struct i2c_device_id drv2624_id[] = {
	{ "drv2624", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, drv2624_id);

#ifdef CONFIG_OF
static const struct of_device_id of_drv2624_match[] = {
	{ .compatible = "ti,drv2624" },
	{},
};

MODULE_DEVICE_TABLE(of, of_drv2624_match);
#endif

static struct i2c_driver drv2624_driver = {
	.driver = {
		.name	= "drv2624",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(of_drv2624_match),
#endif
	},
	.probe		= drv2624_probe,
	.remove		= drv2624_remove,
	.id_table	= drv2624_id,
};
#if 0
module_i2c_driver(drv2624_driver);
#else
static int __init drv2624_i2c_init(void)
{
	int ret = 0;

	pr_info("%s:%u\n",__func__, __LINE__);
	/*if (haptic_mode[0] == 0) {
		pr_info("%s:%u: no TI haptic!\n",__func__, __LINE__);
		return -ENODEV;
	}*/
	ret = i2c_add_driver(&drv2624_driver);
	if (ret) {
		pr_err("%s:%u:fail to add drv2624 device into"
			" i2c ret = 0x%x\n", __func__, __LINE__, ret);
	}

	return ret;
}

late_initcall_sync(drv2624_i2c_init);

static void __exit drv2624_i2c_exit(void)
{
	i2c_del_driver(&drv2624_driver);
}
module_exit(drv2624_i2c_exit);
#endif

MODULE_DESCRIPTION("Texas Instruments DRV2624 Haptic Driver");
MODULE_LICENSE("GPL");
