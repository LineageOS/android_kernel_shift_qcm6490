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
#include <linux/slab.h>

#include "tasdevice-regbin.h"
#include "tasdevice-dsp.h"
#include "tasdevice.h"
#include "tasdevice-dsp_kernel.h"

#define TASDEVICE_MAXPROGRAM_NUM_KERNEL			5
#define TASDEVICE_MAXCONFIG_NUM_KERNEL_MULTIPLE_AMPS	64
#define TASDEVICE_MAXCONFIG_NUM_KERNEL			10
#define MAIN_ALL_DEVICES_1X				0x01
#define MAIN_DEVICE_A_1X				0x02
#define MAIN_DEVICE_B_1X				0x03
#define MAIN_DEVICE_C_1X				0x04
#define MAIN_DEVICE_D_1X				0x05
#define COEFF_DEVICE_A_1X				0x12
#define COEFF_DEVICE_B_1X				0x13
#define COEFF_DEVICE_C_1X				0x14
#define COEFF_DEVICE_D_1X				0x15
#define PRE_DEVICE_A_1X					0x22
#define PRE_DEVICE_B_1X					0x23
#define PRE_DEVICE_C_1X					0x24
#define PRE_DEVICE_D_1X					0x25
#define PRE_SOFTWARE_RESET_DEVICE_A			0x41
#define PRE_SOFTWARE_RESET_DEVICE_B			0x42
#define PRE_SOFTWARE_RESET_DEVICE_C			0x43
#define PRE_SOFTWARE_RESET_DEVICE_D			0x44
#define POST_SOFTWARE_RESET_DEVICE_A			0x45
#define POST_SOFTWARE_RESET_DEVICE_B			0x46
#define POST_SOFTWARE_RESET_DEVICE_C			0x47
#define POST_SOFTWARE_RESET_DEVICE_D			0x48

static unsigned char map_dev_idx(struct tasdevice_fw *tas_fmw,
	struct TBlock *block)
{
	struct tasdevice_dspfw_hdr *pFw_hdr = &(tas_fmw->fw_hdr);
	struct tasdevice_fw_fixed_hdr *fw_fixed_hdr = &(pFw_hdr->mnFixedHdr);
	unsigned char dev_idx = 0;

	if (fw_fixed_hdr->ppcver >= PPC3_VERSION_TAS2781) {
		switch (block->type) {
		case MAIN_ALL_DEVICES_1X:
			dev_idx = 0x80;
			break;
		case MAIN_DEVICE_A_1X:
			dev_idx = 0x81;
			break;
		case COEFF_DEVICE_A_1X:
		case PRE_DEVICE_A_1X:
		case PRE_SOFTWARE_RESET_DEVICE_A:
		case POST_SOFTWARE_RESET_DEVICE_A:
			dev_idx = 0xC1;
			break;
		case MAIN_DEVICE_B_1X:
			dev_idx = 0x82;
			break;
		case COEFF_DEVICE_B_1X:
		case PRE_DEVICE_B_1X:
		case PRE_SOFTWARE_RESET_DEVICE_B:
		case POST_SOFTWARE_RESET_DEVICE_B:
			dev_idx = 0xC2;
			break;
		case MAIN_DEVICE_C_1X:
			dev_idx = 0x83;
			break;
		case COEFF_DEVICE_C_1X:
		case PRE_DEVICE_C_1X:
		case PRE_SOFTWARE_RESET_DEVICE_C:
		case POST_SOFTWARE_RESET_DEVICE_C:
			dev_idx = 0xC3;
			break;
		case MAIN_DEVICE_D_1X:
			dev_idx = 0x84;
			break;
		case COEFF_DEVICE_D_1X:
		case PRE_DEVICE_D_1X:
		case PRE_SOFTWARE_RESET_DEVICE_D:
		case POST_SOFTWARE_RESET_DEVICE_D:
			dev_idx = 0xC4;
			break;
		default:
			pr_info("%s: load block: Other Type = 0x%02x\n", __func__,
				block->type);
			break;
		}
	} else if (fw_fixed_hdr->ppcver >=
	PPC3_VERSION) {
		switch (block->type) {
		case MAIN_ALL_DEVICES_1X:
			dev_idx = 0|0x80;
			break;
		case MAIN_DEVICE_A_1X:
			dev_idx = 1|0x80;
			break;
		case COEFF_DEVICE_A_1X:
		case PRE_DEVICE_A_1X:
			dev_idx = 1|0xC0;
			break;
		case MAIN_DEVICE_B_1X:
			dev_idx = 2|0x80;
			break;
		case COEFF_DEVICE_B_1X:
		case PRE_DEVICE_B_1X:
			dev_idx = 2|0xC0;
			break;
		case MAIN_DEVICE_C_1X:
			dev_idx = 3|0x80;
			break;
		case COEFF_DEVICE_C_1X:
		case PRE_DEVICE_C_1X:
			dev_idx = 3|0xC0;
			break;
		case MAIN_DEVICE_D_1X:
			dev_idx = 4|0x80;
			break;
		case COEFF_DEVICE_D_1X:
		case PRE_DEVICE_D_1X:
			dev_idx = 4|0xC0;
			break;
		default:
			pr_info("%s: TAS2781 load block: Other Type = 0x%02x\n", __func__,
				block->type);
			break;
		}
	} else {
		switch (block->type) {
		case MAIN_ALL_DEVICES:
			dev_idx = 0|0x80;
			break;
		case MAIN_DEVICE_A:
			dev_idx = 1|0x80;
			break;
		case COEFF_DEVICE_A:
		case PRE_DEVICE_A:
			dev_idx = 1|0xC0;
			break;
		case MAIN_DEVICE_B:
			dev_idx = 2|0x80;
			break;
		case COEFF_DEVICE_B:
		case PRE_DEVICE_B:
			dev_idx = 2|0xC0;
			break;
		case MAIN_DEVICE_C:
			dev_idx = 3|0x80;
			break;
		case COEFF_DEVICE_C:
		case PRE_DEVICE_C:
			dev_idx = 3|0xC0;
			break;
		case MAIN_DEVICE_D:
			dev_idx = 4|0x80;
			break;
		case COEFF_DEVICE_D:
		case PRE_DEVICE_D:
			dev_idx = 4|0xC0;
			break;
		default:
			pr_info("%s: TAS2781 load block: Other Type = 0x%02x\n", __func__,
				block->type);
			break;
		}
	}

	return dev_idx;
}

static int fw_parse_block_data_kernel(struct tasdevice_fw *pFirmware,
	struct TBlock *block, const struct firmware *pFW, int offset)
{
	const unsigned char *data = pFW->data;

	if (offset + 4 > pFW->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	block->type = be32_to_cpup((__be32 *)&data[offset]);
	offset  += 4;

	if (offset + 1 > pFW->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	block->mbPChkSumPresent = data[offset];
	offset++;

	if (offset + 1 > pFW->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	block->mnPChkSum = data[offset];
	offset++;

	if (offset + 1 > pFW->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	block->mbYChkSumPresent = data[offset];
	offset++;

	if (offset + 1 > pFW->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	block->mnYChkSum = data[offset];
	offset++;

	if (offset + 4 > pFW->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	block->blk_size = be32_to_cpup((__be32 *)&data[offset]);
	offset  += 4;

	if (offset + 4 > pFW->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	block->nSublocks = be32_to_cpup((__be32 *)&data[offset]);
	offset  += 4;

	/* storing the dev_idx as a member of block can reduce unnecessary time
	 * and system resource comsumption of dev_idx mapping every time the block
	 * data writing to the dsp.
	 */
	block->dev_idx = map_dev_idx(pFirmware, block);

	block->mpData = kzalloc(block->blk_size, GFP_KERNEL);
	if (block->mpData == NULL) {
		pr_err("%s: mpData memory error\n", __func__);
		offset = -1;
		goto out;
	}
	memcpy(block->mpData, &data[offset], block->blk_size);
	offset  += block->blk_size;
out:
	return offset;
}

static int fw_parse_data_kernel(struct tasdevice_fw *pFirmware,
	struct TData *pImageData, const struct firmware *pFW, int offset)
{
	const unsigned char *data = pFW->data;
	unsigned int nBlock;

	if (offset + 4 > pFW->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	pImageData->mnBlocks = be32_to_cpup((__be32 *)&data[offset]);
	offset  += 4;

	pImageData->mpBlocks =
		kcalloc(pImageData->mnBlocks, sizeof(struct TBlock),
			GFP_KERNEL);
	if (pImageData->mpBlocks == NULL) {
		pr_err("%s: FW memory failed!\n", __func__);
		goto out;
	}

	for (nBlock = 0; nBlock < pImageData->mnBlocks; nBlock++) {
		offset = fw_parse_block_data_kernel(pFirmware,
			&(pImageData->mpBlocks[nBlock]), pFW, offset);
		if (offset < 0) {
			offset = -1;
			goto out;
		}
	}
out:
	return offset;
}

int fw_parse_program_data_kernel(struct tasdevice_fw *pFirmware,
	const struct firmware *pFW, int offset)
{
	struct TProgram *pProgram;
	const unsigned char *buf = pFW->data;
	unsigned int  nProgram = 0;

	for (nProgram = 0; nProgram < pFirmware->nr_programs; nProgram++) {
		pProgram = &(pFirmware->mpPrograms[nProgram]);
		if (offset + 64 > pFW->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		memcpy(pProgram->mpName, &buf[offset], 64);
		offset  += 64;

		if (offset + 1 > pFW->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		pProgram->mnAppMode = buf[offset];
		offset++;

		if (offset + 1 > pFW->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		pProgram->mnPDMI2SMode = buf[offset];
		offset++;
		if (offset + 1 > pFW->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		pProgram->mnISnsPD = buf[offset];
		offset++;
		if (offset + 1 > pFW->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		pProgram->mnVSnsPD = buf[offset];
		offset++;
		//skip 3-byte reserved
		offset  += 3;
		if (offset + 1 > pFW->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		pProgram->mnPowerLDG = buf[offset];
		offset++;

		offset = fw_parse_data_kernel(pFirmware, &(pProgram->mData),
			pFW, offset);
		if (offset < 0)
			goto out;
	}
out:
	return offset;
}

int fw_parse_configuration_data_kernel(
	struct tasdevice_fw *pFirmware, const struct firmware *fmw, int offset)
{
	const unsigned char *data = fmw->data;

	unsigned int nConfiguration;
	struct TConfiguration *pConfiguration;

	for (nConfiguration = 0; nConfiguration < pFirmware->nr_configurations;
		nConfiguration++) {
		pConfiguration = &(pFirmware->mpConfigurations[nConfiguration]);
		if (offset + 64 > fmw->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		memcpy(pConfiguration->mpName, &data[offset], 64);
		offset  += 64;

		if (offset + 1 > fmw->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		pConfiguration->mnDevice_orientation = data[offset];
		offset++;
		if (offset + 1 > fmw->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		pConfiguration->mnDevices = data[offset + 1];
		offset  += 1;

		if (offset + 2 > fmw->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		pConfiguration->mProgram =
			be16_to_cpup((__be16 *)&data[offset]);
		offset  += 2;

		if (offset + 4 > fmw->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		pConfiguration->mnSamplingRate =
			be32_to_cpup((__be32 *)&data[offset]);
		offset  += 4;

		if (offset + 2 > fmw->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		pConfiguration->mnPLLSrc =
			be16_to_cpup((__be16 *)&data[offset]);
		offset  += 2;

		if (offset + 2 > fmw->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		pConfiguration->mnFsRate =
			be16_to_cpup((__be16 *)&data[offset]);
		offset  += 2;

		if (offset + 4 > fmw->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		pConfiguration->mnPLLSrcRate =
			be32_to_cpup((__be32 *)&data[offset]);
		offset  += 4;
		offset = fw_parse_data_kernel(pFirmware,
			&(pConfiguration->mData), fmw, offset);
		if (offset < 0)
			goto out;
	}
out:
	return offset;
}

int fw_parse_variable_header_kernel(struct tasdevice_priv *tas_dev,
	const struct firmware *fmw, int offset)
{
	struct tasdevice_fw *pFirmware = tas_dev->fmw;
	struct tasdevice_dspfw_hdr *pFw_hdr = &(pFirmware->fw_hdr);
	const unsigned char *buf = fmw->data;
	struct TProgram *pProgram;
	struct TConfiguration *pConfiguration;
	unsigned int  nProgram = 0, nConfiguration = 0;
	unsigned short maxConf = TASDEVICE_MAXCONFIG_NUM_KERNEL;

	if (offset + 2 > fmw->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	pFw_hdr->mnDeviceFamily = be16_to_cpup((__be16 *)&buf[offset]);
	if (pFw_hdr->mnDeviceFamily != 0) {
		pr_err("ERROR:%s:not TAS device\n", __func__);
		offset = -1;
		goto out;
	}
	offset  += 2;
	if (offset + 2 > fmw->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	pFw_hdr->mnDevice = be16_to_cpup((__be16 *)&buf[offset]);
	if (pFw_hdr->mnDevice >= TASDEVICE_DSP_TAS_MAX_DEVICE ||
		pFw_hdr->mnDevice == 6) {
		pr_err("ERROR:%s: not support device %d\n", __func__,
			pFw_hdr->mnDevice);
		offset = -1;
		goto out;
	}
	offset  += 2;
	pFw_hdr->ndev = deviceNumber[pFw_hdr->mnDevice];

	if (pFw_hdr->ndev != tas_dev->ndev) {
		pr_err("%s: ndev(%u) in dspbin dismatch ndev(%u) in DTS\n",
			__func__, pFw_hdr->ndev, tas_dev->ndev);
		offset = -1;
		goto out;
	}

	if (offset + 4 > fmw->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	pFirmware->nr_programs = be32_to_cpup((__be32 *)&buf[offset]);
	offset  += 4;

	if (pFirmware->nr_programs == 0 || pFirmware->nr_programs >
		TASDEVICE_MAXPROGRAM_NUM_KERNEL) {
		pr_err("%s: mnPrograms is invalid\n", __func__);
		offset = -1;
		goto out;
	}

	if (offset + 4 * TASDEVICE_MAXPROGRAM_NUM_KERNEL > fmw->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}

	pFirmware->mpPrograms = kcalloc(pFirmware->nr_programs,
		sizeof(struct TProgram), GFP_KERNEL);
	if (pFirmware->mpPrograms == NULL) {
		pr_err("%s: mpPrograms memory failed!\n", __func__);
		offset = -1;
		goto out;
	}

	for (nProgram = 0; nProgram < pFirmware->nr_programs; nProgram++) {
		pProgram = &(pFirmware->mpPrograms[nProgram]);
		pProgram->prog_size = be32_to_cpup((__be32 *)&buf[offset]);
		pFirmware->cfg_start_offset  += pProgram->prog_size;
		offset  += 4;
	}
	offset  += (4 * (5 - nProgram));

	if (offset + 4 > fmw->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	pFirmware->nr_configurations = be32_to_cpup((__be32 *)&buf[offset]);
	offset  += 4;
	maxConf = (pFw_hdr->ndev >= 4) ?
		TASDEVICE_MAXCONFIG_NUM_KERNEL_MULTIPLE_AMPS :
		TASDEVICE_MAXCONFIG_NUM_KERNEL;
	if (pFirmware->nr_configurations == 0 ||
		pFirmware->nr_configurations > maxConf) {
		pr_err("%s: mnConfigurations is invalid\n", __func__);
		offset = -1;
		goto out;
	}

	if (offset + 4 * maxConf > fmw->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}

	pFirmware->mpConfigurations = kcalloc(pFirmware->nr_configurations,
		sizeof(struct TConfiguration), GFP_KERNEL);
	if (pFirmware->mpConfigurations == NULL) {
		pr_err("%s: mpPrograms memory failed!\n", __func__);
		offset = -1;
		goto out;
	}

	for (nConfiguration = 0; nConfiguration < pFirmware->nr_programs;
		nConfiguration++) {
		pConfiguration =
			&(pFirmware->mpConfigurations[nConfiguration]);
		pConfiguration->cfg_size =
			be32_to_cpup((__be32 *)&buf[offset]);
		offset  += 4;
	}

	offset  += (4 * (maxConf - nConfiguration));
	pFirmware->prog_start_offset = offset;
	pFirmware->cfg_start_offset  += offset;
out:
	return offset;
}

int tasdevice_load_block_kernel(struct tasdevice_priv *tas_priv,
	struct TBlock *block)
{
	int nResult = 0;

	unsigned char *pData = block->mpData;
	unsigned int i = 0, length = 0;
	const unsigned int blk_size = block->blk_size;

	for (i = 0; i < block->nSublocks; i++) {
		int rc = tasdevice_process_block(tas_priv, pData + length,
			block->dev_idx, blk_size - length);
		if (rc < 0) {
			dev_err(tas_priv->dev, "%s: ERROR:%u %u sublock write "
				"error\n", __func__, length, blk_size);
			break;
		}
		length  += (unsigned int)rc;
		if (blk_size < length) {
			dev_err(tas_priv->dev, "%s: ERROR:%u %u out of "
				"memory\n", __func__, length, blk_size);
			break;
		}
	}

	return nResult;
}
