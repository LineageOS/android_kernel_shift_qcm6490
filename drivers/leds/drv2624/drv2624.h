/*
** ===================================================================
** Copyright (c)2021-2022  Texas Instruments Inc.
**
** This program is free software; you can redistribute it and/or
** modify it under the terms of the GNU General Public License
** as published by the Free Software Foundation; either version 2
** of the License, or (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
** MA  02110-1301, USA.
**
** File:
**     drv2624.h
**
** Description:
**     Header file for drv2624-ram.c
**
** ===================================================================
*/
#ifndef __DRV2624_H__
#define __DRV2624_H__

#define LEDS_ARCH

#if 0
	#include <linux/leds.h>
#else
	#include <linux/leds.h>
	#include <linux/input.h>
	#include <linux/miscdevice.h>
	#define RTP_BIN_FILE							("drv2624.rtp")
	#define RTP_ID  (SMS_HTONL('r','t','p',0))
	struct wv_cmds {
		unsigned char gain;
		unsigned char sleep_time;
	};

	struct drv26xx_RTPwaveform_info {
		unsigned short length; /*commands numbers*/
		unsigned short duration;
		unsigned char brake;
		unsigned char loop_mod;
		unsigned char wv_shape;
		unsigned char actuator_type;
		unsigned int  lra_f0;
		unsigned char mnCLRatedVoltageRegbits;
		unsigned char mnCLOverDriveClampVoltageRegbits;
		unsigned char mnOLOverDriveClampVoltageRegbits;
		struct wv_cmds* wvfm_cmds;
	};

	struct drv26xx_RTPwaveforms {
		unsigned int   filetype;
		unsigned int  rated_Voltage;
		unsigned int  overDrive_Voltage;
		unsigned int  lra_f0;
		unsigned int  sample_time;
		unsigned int  DriveTime;
		unsigned short version;
		unsigned char mnCLRatedVoltageRegbits;
		unsigned char mnCLOverDriveClampVoltageRegbits;
		unsigned char mnOLOverDriveClampVoltageRegbits;
		unsigned char mnSampleTimeRegbits;
		unsigned char  fb_brake_factor;
		unsigned char  auto_brake_standby;
		unsigned char  auto_brake;
		unsigned char  hybrid_loop;
		unsigned short nWaveforms;
		unsigned char rtp_binaryname[256];
		struct drv26xx_RTPwaveform_info** rtp_wvfm_info;
	};

	struct sysfs_cmd {
		bool bCmdErr;
		unsigned short rtpwavid;
		unsigned short bufLen;
	};
#endif

// #define RAM_BIN_FILE		 						("drv2624.ram")
#define RTP_BIN_FILE								("drv2624.rtp")
#define HAPTICS_DEVICE_NAME 						"drv2625"
#define FRES_MIN									(45)
#define FRES_DEFAULT								(150)
#define FRES_MAX									(FRES_DEFAULT<<1)

#define DRV26XX_I2C_RETRY_COUNT						(3)
#define	DRV26XX_GO_BIT_MAX_RETRY_CNT				(5)
/* 10 ms */
#define	DRV26XX_GO_BIT_CHECK_INTERVAL           	(10)
#define	DRV2624_SEQUENCER_SIZE						(8)

#define	DRV2624_R0X00								(0x00)
#define DRV2624_R0X00_CHIP_ID_REV					(0x03)
#define DRV2625_R0X00_CHIP_ID_REV					(0x13)

#define	DRV2624_R0X01_STATUS						(0x01)
#define	DRV2624_R0X01_STATUS_DIAG_RESULT_MSK		(0x80)
#define	DRV2624_R0X01_STATUS_DIAG_RESULT_OK			(0x00)
#define	DRV2624_R0X01_STATUS_DIAG_RESULT_NOK		(0x80)
#define	DRV2624_R0X01_STATUS_PRG_ERROR_MSK			(0x10)
#define	DRV2624_R0X01_STATUS_PROCESS_DONE_MSK		(0x08)
#define	DRV2624_R0X01_STATUS_UVLO_MSK				(0x04)
#define	DRV2624_R0X01_STATUS_OVER_TEMP_MSK			(0x02)
#define	DRV2624_R0X01_STATUS_OC_DETECT_MSK			(0x01)

#define	DRV2624_R0X02_INTZ							(0x02)
#define	DRV2624_R0X02_INTZ_MSK						(0x1F)
#define	DRV2624_R0X02_INTZ_DISABLE					(0x1F)
#define	DRV2624_R0X02_INTZ_ENABLE					(0x00)
#define	DRV2624_R0X02_INTZ_PROCESS_DONE_DISABLE		(0x08)

#define	DRV2624_RX03_DIAG_Z_RESULT					(0x03)

#define	DRV2624_R0X05_LRA_PERIOD_H					(0x05)
#define	DRV2624_R0X05_LRA_PERIOD_H_MSK				(0x03)
#define	DRV2624_R0X06_LRA_PERIOD_L					(0x06)

#define	DRV2624_R0X07								(0x07)
#define	DRV2624_R0X07_TRIG_PIN_FUNC_SFT				(2)
#define	DRV2624_R0X07_TRIG_PIN_FUNC_MSK				\
	(0x03 << DRV2624_R0X07_TRIG_PIN_FUNC_SFT)
#define	DRV2624_R0X07_TRIG_PIN_FUNC_EXTERNAL_PULSE	(0)
#define	DRV2624_R0X07_TRIG_PIN_FUNC_INT				\
	(0x02 << DRV2624_R0X07_TRIG_PIN_FUNC_SFT)
#define DRV2624_R0X07_MODE_MSK						(0x3)
#define DRV2624_R0X07_MODE_RTP_MODE					(0x0)
#define DRV2624_R0X07_MODE_WVFRM_SEQ_MODE			(0x1)
#define DRV2624_R0X07_MODE_DIAG_RTN					(0x2)
#define DRV2624_R0X07_MODE_AUTO_LVL_CALIB_RTN		(0x3)

#define	DRV2624_R0X08								(0x08)
#define	DRV2624_R0X08_LRA_ERM_MSK					(0x80)
#define	DRV2624_R0X08_LRA_ERM_SFT					(7)
#define	DRV2624_R0X08_LRA							\
	(1<<DRV2624_R0X08_LRA_ERM_SFT)
#define	DRV2624_R0X08_ERM							(0)
#define	DRV2624_R0X08_CTL_LOOP_MSK					(0x40)
#define	DRV2624_R0X08_CTL_LOOP_SFT					(6)
#define	DRV2624_R0X08_CTL_LOOP_CLOSED_LOOP			(0)
#define	DRV2624_R0X08_CTL_LOOP_OPEN_LOOP			\
	(1<<DRV2624_R0X08_CTL_LOOP_SFT)
#define	DRV2624_R0X08_HYBRID_LOOP_MSK				(0x20)
#define	DRV2624_R0X08_HYBRID_LOOP_SFT				(5)
#define	DRV2624_R0X08_AUTO_BRK_OL_MSK				(0x10)
#define	DRV2624_R0X08_AUTO_BRK_OL_SFT				(4)
#define	DRV2624_R0X08_AUTO_BRK_OL_EN				(0x10)
#define	DRV2624_R0X08_AUTO_BRK_INTO_STBY_MSK		(0x08)
#define	DRV2624_R0X08_WITHOUT_AUTO_BRK_INTO_STBY	(0x00)
#define	DRV2624_R0X08_AUTO_BRK_INTO_STBY			(0x04)
#define	DRV2624_R0X08_AUTO_BRK_INTO_STBY_SFT		(3)

#define	DRV2624_R0X09								(0x09)
#define	DRV2624_R0X09_UVLO_THRES_MSK				(0x07)
#define	DRV2624_R0X09_UVLO_THRES_3_2V				(0x07)

#define	DRV2624_R0X0C_GO							(0x0C)
#define	DRV2624_R0X0C_GO_MSK						(0x01)
#define	DRV2624_R0X0C_GO_BIT						(0x01)
#define	DRV2624_R0X0C_NGO_BIT						(0x00)

#define	DRV2624_R0X0D								(0x0D)
#define	DRV2624_R0X0D_PLAYBACK_INTERVAL_MSK			(0x20)
#define	DRV2624_R0X0D_PLAYBACK_INTERVAL_SFT			(5)
#define	DRV2624_R0X0D_PLAYBACK_INTERVAL_1MS			\
	(0x01 << DRV2624_R0X0D_PLAYBACK_INTERVAL_SFT)
#define	DRV2624_R0X0D_PLAYBACK_INTERVAL_5MS			\
		(0x00 << DRV2624_R0X0D_PLAYBACK_INTERVAL_SFT)
#define DRV2624_R0X0D_DIG_MEM_GAIN_MASK 			(0x03)
#define  DRV2624_R0X0D_DIG_MEM_GAIN_STRENGTH_100 (0x0)
#define  DRV2624_R0X0D_DIG_MEM_GAIN_STRENGTH_75	(0x1)
#define  DRV2624_R0X0D_DIG_MEM_GAIN_STRENGTH_50	(0x2)
#define  DRV2624_R0X0D_DIG_MEM_GAIN_STRENGTH_25	(0x3)

#define	DRV2624_R0X0E_RTP_INPUT						(0x0e)
#define	DRV2624_R0X0F_SEQ1							(0x0f)
#define	DRV2624_R0X10_SEQ2							(0x10)
#define	DRV2624_R0X17_WAV1_4_SEQ_LOOP				(0x17)

#define	DRV2624_R0X19_WAV_SEQ_MAIN_LOOP				(0x19)
#define	DRV2624_R0X19_WAV_SEQ_MAIN_LOOP_MSK			(0x07)
#define	DRV2624_R0X19_WAV_SEQ_MAIN_LOOP_ONCE		(0x00)
#define	DRV2624_R0X19_WAV_SEQ_MAIN_LOOP_INFINITE	(0x07)

#define	DRV2624_R0X1F_RATED_VOLTAGE					(0x1f)
#define	DRV2624_R0X20_OD_CLAMP						(0x20)

#define	DRV2624_R0X21_CAL_COMP						(0x21)
#define DRV2624_R0X22_CAL_BEMF						(0x22)
#define	DRV2624_R0X23								(0x23)
#define	DRV2624_R0X23_LOOP_GAIN_MSK					(0x0c)
#define DRV2624_R0X23_LOOP_GAIN_SFT					(2)
#define	DRV2624_R0X23_BEMF_GAIN_MSK					(0x03)
#define	DRV2624_R0X23_BEMF_GAIN_30X_LRA				(0x03)
#define	DRV2624_R0X23_FB_BRAKE_FACTOR_MSK			(0x70)
#define	DRV2624_R0X23_FB_BRAKE_FACTOR_SFT			(4)

#define	DRV2624_R0X27								(0x27)
#define	DRV2624_R0X27_DRIVE_TIME_MSK				(0x1f)
#define	DRV2624_R0X27_LRA_MIN_FREQ_SEL_SFT			(0x07)
#define	DRV2624_R0X27_LRA_MIN_FREQ_SEL_45HZ			\
	(0x01 << DRV2624_R0X27_LRA_MIN_FREQ_SEL_SFT)
#define	DRV2624_R0X27_LRA_MIN_FREQ_SEL_MSK			(0x80)

#define	DRV2624_R0X29								(0x29)
#define	DRV2624_R0X29_SAMPLE_TIME_MSK				(0x0C)
#define	DRV2624_R0X29_SAMPLE_TIME_250us				(0x08)

#define	DRV2624_R0X2A								(0x2A)
#define	DRV2624_R0X2A_AUTO_CAL_TIME_MSK				(0x03)
#define	DRV2624_R0X2A_AUTO_CAL_TIME_TRIGGER_CRTLD	(0x03)

#define	DRV2624_R0X2C								(0x2C)
#define	DRV2624_R0X2C_LRA_WAVE_SHAPE_MSK			(0x01)


#define	DRV2624_R0X2E_OL_LRA_PERIOD_H				(0x2e)
#define	DRV2624_R0X2E_OL_LRA_PERIOD_H_MSK			(0x03)
#define	DRV2624_R0X2F_OL_LRA_PERIOD_L				(0x2f)
#define	DRV2624_R0X2F_OL_LRA_PERIOD_L_MSK			(0xff)

#define	DRV2624_R0X30_CURRENT_K						(0x30)

#define	DRV2624_R0XFD_RAM_ADDR_UPPER				(0xfd)
#define	DRV2624_R0XFE_RAM_ADDR_LOWER				(0xfe)
#define	DRV2624_R0XFF_RAM_DATA						(0xff)

#define DRV26XX_RAM_SIZE        					(1024)

#define FB_BRAKE_FACTOR_CALIBRATION				(3)
#define LOOP_GAIN_CALIBRATION					(2)

#define SMS_HTONS(a,b) (	(((a)&0x00FF)<<8) | \
						((b)&0x00FF) )
#define RAM_ID  (SMS_HTONL('r','a','m',0))
#define SMS_HTONL(a, b, c, d) (	(((a)&0x000000FF)<<24) | \
							(((b)&0x000000FF)<<16)	| \
							(((c)&0x000000FF)<<8)  | \
							((d)&0x000000FF)	)


#define	DRV2625_REG_ID				(0x00)
#define	DRV2625_ID					(0x12&0xf0)

#define	DRV2625_REG_STATUS			0x01
#define	DIAG_MASK					0x80
#define	DIAG_SUCCESS				0x00
#define	DIAG_SHIFT					0x07
#define	INT_MASK					0x1f
#define	PRG_ERR_MASK				0x10
#define	PROCESS_DONE_MASK			0x08
#define	ULVO_MASK					0x04
#define	OVERTEMPRATURE_MASK			0x02
#define	OVERCURRENT_MASK			0x01

#define	DRV2625_REG_INT_ENABLE		0x02
#define	INT_MASK_ALL				0x1f
#define	INT_ENABLE_ALL				0x00
#define	INT_ENABLE_CRITICAL			0x08

#define	DRV2625_REG_DIAG_Z			0x03

 #define DRV26XX_I2C_RETRY_COUNT                     (3)

#define	DRV2625_REG_MODE			0x07
#define	WORKMODE_MASK				0x03
#define	MODE_RTP					0x00
#define	MODE_WAVEFORM_SEQUENCER		0x01
#define	MODE_DIAGNOSTIC				0x02
#define	MODE_CALIBRATION			0x03
#define	PINFUNC_MASK				0x0c
#define	PINFUNC_INT					0x02
#define	PINFUNC_SHIFT				0x02

#define	DRV2625_REG_CONTROL1		0x08
#define	ACTUATOR_MASK				0x80
#define	ACTUATOR_SHIFT				7
#define	LOOP_MASK					0x40
#define	LOOP_SHIFT					6
#define	AUTOBRK_OK_MASK				0x10
#define	AUTOBRK_OK_ENABLE			0x10

#define	DRV2625_REG_GO				0x0c

#define	DRV2625_REG_CONTROL2		0x0d
#define	LIB_LRA						0x00
#define	LIB_ERM						0x01
#define	LIB_MASK					0x80
#define	LIB_SHIFT					0x07
#define	SCALE_MASK					0x03
#define	INTERVAL_MASK				0x20
#define	INTERVAL_SHIFT				0x05

#define	DRV2625_REG_RTP_INPUT		0x0e

#define	DRV2625_REG_SEQUENCER_1		0x0f

#define	DRV2625_REG_SEQ_LOOP_1		0x17

#define	DRV2625_REG_SEQ_LOOP_2		0x18

#define	DRV2625_REG_MAIN_LOOP		0x19

#define	DRV2625_REG_RATED_VOLTAGE	0x1f

#define	DRV2625_REG_OVERDRIVE_CLAMP	0x20

#define	DRV2625_REG_CAL_COMP		0x21

#define	DRV2625_REG_CAL_BEMF		0x22

#define	DRV2625_REG_LOOP_CONTROL	0x23
#define	BEMFGAIN_MASK				0x03

#define	DRV2625_REG_DRIVE_TIME		0x27
#define	DRIVE_TIME_MASK				0x1f
#define	MINFREQ_SEL_45HZ			0x01
#define	MINFREQ_SEL_MASK			0x80
#define	MINFREQ_SEL_SHIFT			0x07

#define	DRV2625_REG_OL_PERIOD_H		0x2e

#define	DRV2625_REG_OL_PERIOD_L		0x2f

#define	DRV2625_REG_DIAG_K			0x30

#define	GO_BIT_POLL_INTERVAL	15
#define	STANDBY_WAKE_DELAY		1
#define	WAKE_STANDBY_DELAY		3


/* Commands */
#define	HAPTIC_CMDID_PLAY_SINGLE_EFFECT		0x01
#define	HAPTIC_CMDID_PLAY_EFFECT_SEQUENCE	0x02
#define	HAPTIC_CMDID_PLAY_TIMED_EFFECT		0x03
#define	HAPTIC_CMDID_GET_DEV_ID				0x04
#define	HAPTIC_CMDID_RUN_DIAG				0x05
#define	HAPTIC_CMDID_AUDIOHAPTIC_ENABLE		0x06
#define	HAPTIC_CMDID_AUDIOHAPTIC_DISABLE	0x07
#define	HAPTIC_CMDID_AUDIOHAPTIC_GETSTATUS	0x08
#define	HAPTIC_CMDID_REG_WRITE				0x09
#define	HAPTIC_CMDID_REG_READ				0x0a
#define	HAPTIC_CMDID_REG_SETBIT				0x0b
#define	HAPTIC_CMDID_PATTERN_RTP			0x0c
#define	HAPTIC_CMDID_RTP_SEQUENCE			0x0d
#define	HAPTIC_CMDID_GET_EFFECT_COUNT		0x10
#define	HAPTIC_CMDID_UPDATE_FIRMWARE		0x11
#define	HAPTIC_CMDID_READ_FIRMWARE			0x12
#define	HAPTIC_CMDID_RUN_CALIBRATION		0x13
#define	HAPTIC_CMDID_CONFIG_WAVEFORM		0x14
#define	HAPTIC_CMDID_SET_SEQUENCER			0x15
#define	HAPTIC_CMDID_REGLOG_ENABLE			0x16

#define	HAPTIC_CMDID_STOP		0xFF

#define	MAX_TIMEOUT		10000 /* 10s */
#define	MAX_READ_BYTES	0xff
#define	DRV2625_SEQUENCER_SIZE	8

#define	WORK_IDLE					0
#define	WORK_VIBRATOR				0x01
#define	WORK_IRQ					0x02
#define	WORK_EFFECTSEQUENCER		0x04
#define	WORK_CALIBRATION			0x08
#define	WORK_DIAGNOSTIC				0x10

#define	YES		1
#define	NO		0
#define	GO		1
#define	STOP	0

#define	POLL_GO_BIT_INTERVAL	5	/* 5 ms */
#define	POLL_GO_BIT_RETRY		20	/* 50 times */

#define	GO_BIT_CHECK_INTERVAL           5	/* 5 ms */
#define	GO_BIT_MAX_RETRY_CNT		20	/* 50 times */

struct drv2625_autocal_result {
	int mnFinished;
	unsigned char mnResult;
	unsigned char mnCalComp;
	unsigned char mnCalBemf;
	unsigned char mnCalGain;
};

struct drv2624_wave_setting {
	unsigned char mnLoop;
	unsigned char mnInterval;
	unsigned char mnScale;
};

typedef enum {
	DRV2624_RTP_MODE = 0x00,
	DRV2624_RAM_MODE,
	DRV2624_WAVE_SEQ_MODE = DRV2624_RAM_MODE,
	DRV2624_DIAG_MODE, 
	DRV2624_CALIBRATION_MODE,
	DRV2624_NEW_RTP_MODE,
} drv2624_mode_t;
struct drv2624_constant_playinfo {
	int effect_count;
	int effect_id;
	int length;
	int magnitude;
	unsigned char rtp_input;
};

typedef enum tiDrv26xxRAM {
	DRV26XX_BF_FB_BRAKE_FACTOR						= 0x02,
	DRV26XX_BF_AUTO_BRAKE_STANDBY					= 0x30,
	DRV26XX_BF_AUTO_BRAKE							= 0x40,
	DRV26XX_BF_HYBRID_LOOP							= 0x50,
	DRV26XX_BF_WVFM_BRAKE							= 0x00,
	DRV26XX_BF_SAMPLE_TIME 							= 0x01,
	DRV26XX_BF_WVFM_LOOP_MOD						= 0x10,
	DRV26XX_BF_WVFM_WV_SHAPE						= 0x20,
	DRV26XX_BF_WVFM_ACTUATOR_TYPE 					= 0x30
} tiDrv26xxBfEnumList_t;

typedef enum tiDrv26xxCaliEnum {
	DRV26XX_CALIBRATION_OK				=
		DRV2624_R0X01_STATUS_DIAG_RESULT_OK,
	DRV26XX_CALIBRATED_FRES_INVALID		= FRES_MIN,
	DRV26XX_CALIBRATION_NOK				=
		DRV2624_R0X01_STATUS_DIAG_RESULT_NOK,
	DRV26XX_NO_CALIBRATION				= 0xFF
} tiDrv26xxCaliEnumList_t;

enum actuator_type {
	ERM = 0,
	LRA
};

struct drv2624_waveform {
	unsigned char mnEffect;
	unsigned char mnLoop;
};

struct drv2624_waveform_sequencer {
	struct drv2624_waveform msWaveform[DRV2624_SEQUENCER_SIZE];
};

struct actuator_data {
	unsigned char mnActuatorType;
	unsigned char mnDTSRatedVoltageRegbits;
	unsigned char mnDTSOverDriveClampVoltageRegbits;
	unsigned char mnDTSSampleTimeRegbits;
	unsigned int  mnDTSSampleTime;
	unsigned int mnDTSLRAFreq;
	unsigned int mnDTSLRAminFreq;
	unsigned int mnDTSLRAmaxFreq;
	unsigned int mnDTSLRAPeriod;
	unsigned int mnDTSRatedVoltage;
	unsigned int mnDTSOverDriveClampVoltage;
	unsigned short openLoopPeriod;
};

struct drv2624_platform_data {
	int mnGpioNRST;
	int mnGpioINT;
	unsigned char mnLoop;
	struct actuator_data msActuator;
};

struct drv2624_lra_period_data {
	unsigned char msb;
	unsigned char lsb;
};

struct drv2624_autocal_data {
	unsigned char mnDoneFlag;
	unsigned char mnCalComp;
	unsigned char mnCalBemf;
	unsigned char mnCalGain;
	unsigned int mnCalibaredF0;
	struct drv2624_lra_period_data mnLraPeriod;
	struct drv2624_lra_period_data mnOL_LraPeriod;
	unsigned char mnCnt;
};

struct drv26xx_RAMwaveform_info {
	unsigned int mnEffectTimems;
	unsigned int  lra_f0;
	unsigned short duration;
	unsigned char brake;
	unsigned char loop_mod;
	unsigned char wv_shape;
	unsigned char actuator_type;
	unsigned char mnCLRatedVoltageRegbits;
	unsigned char mnCLOverDriveClampVoltageRegbits;
	unsigned char mnOLOverDriveClampVoltageRegbits;
};

struct drv26xx_RAMwaveforms {
	unsigned int   filetype;
	unsigned int  rated_Voltage;
	unsigned int  overDrive_Voltage;
	unsigned int  lra_f0;
	unsigned int  sample_time;
	unsigned int  DriveTime;
	unsigned short version;
	unsigned char mnCLRatedVoltageRegbits;
	unsigned char mnCLOverDriveClampVoltageRegbits;
	unsigned char mnOLOverDriveClampVoltageRegbits;
	unsigned char mnSampleTimeRegbits;
	unsigned char  fb_brake_factor;
	unsigned char  auto_brake_standby;
	unsigned char  auto_brake;
	unsigned char  hybrid_loop;
	unsigned char  nWaveforms;
	unsigned char  *rambin_name;
	struct drv26xx_RAMwaveform_info** ram_wvfm_info;
};

struct drv2624_diag_result {
	unsigned char mnResult;
	unsigned char mnDiagZ;
	unsigned char mnCurrentK;
	unsigned char mnCnt;
	unsigned int  mnRemohm;
	int mnFinished;
	unsigned char mnDiagK;
};

struct drv2624_data
{
	struct i2c_client *client;
	struct regmap *mpRegmap;
	struct device *dev;
#ifdef LEDS_ARCH
	struct led_classdev led_dev;
#else
	struct input_dev led_dev;
	struct drv26xx_RTPwaveforms rtpwvfm;
	struct sysfs_cmd mSysfsCmd;
#endif
	struct drv26xx_RTPwaveforms rtpwvfm;
	struct sysfs_cmd mSysfsCmd;
	struct drv2624_platform_data msPlatData;
	struct mutex reg_lock;
	struct mutex haptic_lock;
	struct drv2624_autocal_data mAutoCalData;
	struct drv2624_diag_result mDiagResult;
	struct drv2624_waveform_sequencer msWaveformSequencer;
	struct drv26xx_RAMwaveforms ramwvfm;
	struct work_struct vibrator_irq_work;
	struct work_struct vibrator_work;
	struct hrtimer haptics_timer;
	unsigned char mnDeviceID;
	unsigned int mnIRQ;
	int state;
	int duration;
	unsigned char waveform_id;
	unsigned int gain;
	bool mbIRQEnabled;
	bool mbWork;
	bool bRTPmode;
	bool bInternval_1ms;
	ktime_t current_ktime;
	ktime_t pre_enter_ktime;
	unsigned int interval_us;
	unsigned char mnFileCmd;
	unsigned char mnCurrentReg;
	struct drv2625_autocal_result mAutoCalResult;
	volatile int mnVibratorPlaying;
	bool mbIRQUsed;
	volatile char mnWorkMode;
	unsigned char mnIntStatus;
	struct drv2624_constant_playinfo play;
};

unsigned char drv_get_bf_value(const unsigned char bf,
	const unsigned char reg_value);
int drv2624_haptic_play_go(struct drv2624_data *pDRV2624,
	bool flag);
#endif
