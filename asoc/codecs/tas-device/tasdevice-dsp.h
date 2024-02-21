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

#ifndef __TASDEVICE_DSP_H__
#define __TASDEVICE_DSP_H__

#define MAIN_ALL_DEVICES			0x0d
#define MAIN_DEVICE_A				0x01
#define MAIN_DEVICE_B				0x08
#define MAIN_DEVICE_C				0x10
#define MAIN_DEVICE_D				0x14
#define COEFF_DEVICE_A				0x03
#define COEFF_DEVICE_B				0x0a
#define COEFF_DEVICE_C				0x11
#define COEFF_DEVICE_D				0x15
#define PRE_DEVICE_A				0x04
#define PRE_DEVICE_B				0x0b
#define PRE_DEVICE_C				0x12
#define PRE_DEVICE_D				0x16

#define PPC3_VERSION				0x4100
#define PPC3_VERSION_TAS2781			0x14600

enum tasdevice_dsp_dev_idx {
	TASDEVICE_DSP_TAS_2555 = 0,
	TASDEVICE_DSP_TAS_2555_STEREO,
	TASDEVICE_DSP_TAS_2557_MONO,
	TASDEVICE_DSP_TAS_2557_DUAL_MONO,
	TASDEVICE_DSP_TAS_2559,
	TASDEVICE_DSP_TAS_2563,
	TASDEVICE_DSP_TAS_2563_DUAL_MONO = 7,
	TASDEVICE_DSP_TAS_2563_QUAD,
	TASDEVICE_DSP_TAS_2563_21,
	TASDEVICE_DSP_TAS_2781,
	TASDEVICE_DSP_TAS_2781_DUAL_MONO,
	TASDEVICE_DSP_TAS_2781_21,
	TASDEVICE_DSP_TAS_2781_QUAD,
	TASDEVICE_DSP_TAS_MAX_DEVICE
};

struct tasdevice_fw_fixed_hdr {
	unsigned int mnMagicNumber;
	unsigned int mnFWSize;
	unsigned int mnChecksum;
	unsigned int ppcver;
	unsigned int mnFWVersion;
	unsigned int drv_ver;
	unsigned int mnTimeStamp;
	char mpDDCName[64];
};

struct tasdevice_dspfw_hdr {
	struct tasdevice_fw_fixed_hdr mnFixedHdr;
	unsigned int mnBinFileDocVer;
	char *mpDescription;
	unsigned short mnDeviceFamily;
	unsigned short mnDevice;
	unsigned char ndev;
};

struct TBlock {
	unsigned int type;
	unsigned char mbPChkSumPresent;
	unsigned char mnPChkSum;
	unsigned char mbYChkSumPresent;
	unsigned char mnYChkSum;
	unsigned int mnCommands;
	unsigned int blk_size;
	unsigned int nSublocks;
	unsigned char dev_idx;
	unsigned char *mpData;
};

struct TData {
	char mpName[64];
	char *mpDescription;
	unsigned int mnBlocks;
	struct TBlock *mpBlocks;
};
struct TProgram {
	unsigned int prog_size;
	char mpName[64];
	char *mpDescription;
	unsigned char mnAppMode;
	unsigned char mnPDMI2SMode;
	unsigned char mnISnsPD;
	unsigned char mnVSnsPD;
	unsigned char mnPowerLDG;
	struct TData mData;
};

struct TConfiguration {
	unsigned int cfg_size;
	char mpName[64];
	char *mpDescription;
	unsigned char mnDevice_orientation;
	unsigned char mnDevices;
	unsigned int mProgram;
	unsigned int mnSamplingRate;
	unsigned short mnPLLSrc;
	unsigned int mnPLLSrcRate;
	unsigned int mnFsRate;
	struct TData mData;
};

struct calibration_t {
	char mpName[64];
	char *mpDescription;
	unsigned int mnProgram;
	unsigned int mnConfiguration;
	struct TData mData;
};

struct tasdevice_fw {
	struct tasdevice_dspfw_hdr fw_hdr;
	unsigned int prog_start_offset;
	unsigned short nr_programs;
	struct TProgram *mpPrograms;
	unsigned int cfg_start_offset;
	unsigned short nr_configurations;
	struct TConfiguration *mpConfigurations;
	unsigned short mnCalibrations;
	struct calibration_t *mpCalibrations;
	bool bKernelFormat;
};

extern const char deviceNumber[TASDEVICE_DSP_TAS_MAX_DEVICE];

int tasdevice_dspfw_ready(const void *pVoid, void *pContext);
void tasdevice_dsp_remove(void *pContext);
void tasdevice_calbin_remove(void *pContext);
int tas2781_load_calibration(void *tas_dev, char *pFileName,
	unsigned short i);
int tas2781_set_calibration(void *pContext, unsigned short i,
	int nCalibration);
int tasdevice_select_tuningprm_cfg(void *pContext, int prm,
	int cfg_no, int regbin_conf_no);

#endif
