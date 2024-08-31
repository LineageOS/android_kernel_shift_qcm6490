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
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#ifdef CONFIG_TASDEV_CODEC_SPI
	#include <linux/spi/spi.h>
#else
	#include <linux/i2c.h>
#endif
#include <linux/string.h>
#include <linux/time.h>
#include <linux/uaccess.h>

#include "tasdevice-dsp.h"
#include "tasdevice-regbin.h"
#include "tasdevice.h"
#include "tasdevice-dsp_git.h"
#include "tasdevice-dsp_kernel.h"

#define TAS2781_CAL_BIN_PATH			"/lib/firmware/"

#define ERROR_PRAM_CRCCHK			0x0000000
#define ERROR_YRAM_CRCCHK			0x0000001
#define BINFILEDOCVER				0
#define DRVFWVER				1
#define	PPC_DRIVER_CRCCHK			0x00000200

#define TAS2781_SA_COEFF_SWAP_REG		TASDEVICE_REG(0, 0x35, 0x2c)
#define TAS2781_YRAM_BOOK1			140
#define TAS2781_YRAM1_PAGE			42
#define TAS2781_YRAM1_START_REG			88

#define TAS2781_YRAM2_START_PAGE		43
#define TAS2781_YRAM2_END_PAGE			49
#define TAS2781_YRAM2_START_REG			8
#define TAS2781_YRAM2_END_REG			127

#define TAS2781_YRAM3_PAGE			50
#define TAS2781_YRAM3_START_REG			8
#define TAS2781_YRAM3_END_REG			27

/*should not include B0_P53_R44-R47 */
#define TAS2781_YRAM_BOOK2			0
#define TAS2781_YRAM4_START_PAGE		50
#define TAS2781_YRAM4_END_PAGE			60

#define TAS2781_YRAM5_PAGE			61
#define TAS2781_YRAM5_START_REG			8
#define TAS2781_YRAM5_END_REG			27

struct TYCRC {
	unsigned char mnOffset;
	unsigned char mnLen;
};

const unsigned int BinFileformatVerInfo[][2] = {
	{0x100, 0x100},
	{0x110, 0x200},
	{0x200, 0x300},
	{0x210, 0x310},
	{0x230, 0x320},
	{0x300, 0x400}
};

const char *devicefamily[1] = {
	"TAS Devices" };

const char *devicelist[TASDEVICE_DSP_TAS_MAX_DEVICE] = {
	"TAS2555",
	"TAS2555 Stereo",
	"TAS2557 Mono",
	"TAS2557 Dual Mono",
	"TAS2559",
	"TAS2563",
	NULL,
	"TAS2563 Dual Mono",
	"TAS2563 Quad",
	"TAS2563 2.1",
	"TAS2781",
	"TAS2781 Stereo",
	"TAS2781 2.1",
	"TAS2781 Quad"
};

static inline void tas2781_clear_calfirmware(struct tasdevice_fw
	*mpCalFirmware)
{
	int i = 0;
	unsigned int nBlock = 0;

	if (mpCalFirmware->mpCalibrations) {
		struct calibration_t *cal;

		for (i = 0; i < mpCalFirmware->mnCalibrations; i++) {
			cal = &(mpCalFirmware->mpCalibrations[i]);
			if (cal) {
				struct TData *pImageData = &(cal->mData);

				if (pImageData->mpBlocks) {
					struct TBlock *pBlock;

					for (nBlock = 0; nBlock <
						pImageData->mnBlocks;
						nBlock++) {
						pBlock = &(pImageData->
							mpBlocks[nBlock]);
						kfree(pBlock->mpData);
					}
					kfree(pImageData->mpBlocks);
				}
				if (pImageData->mpDescription)
					kfree(pImageData->mpDescription);
				if (cal->mpDescription)
					kfree(cal->mpDescription);
			}
		}
		kfree(mpCalFirmware->mpCalibrations);
	}
	kfree(mpCalFirmware);
}

static int fw_parse_block_data(struct tasdevice_fw *pFirmware,
	struct TBlock *block, const struct firmware *pFW, int offset)
{
	unsigned char *data = (unsigned char *)pFW->data;
	int n;

	if (offset + 4 > pFW->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	block->type = be32_to_cpup((__be32 *)&data[offset]);
	offset  += 4;

	if (pFirmware->fw_hdr.mnFixedHdr.drv_ver >=
		PPC_DRIVER_CRCCHK) {
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
	} else {
		block->mbPChkSumPresent = 0;
		block->mbYChkSumPresent = 0;
	}
	if (offset + 4 > pFW->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	block->mnCommands = be32_to_cpup((__be32 *)&data[offset]);
	offset  += 4;

	n = block->mnCommands * 4;
	if (offset + n > pFW->size) {
		pr_err("%s: File Size(%u) error offset = %d n = %d\n",
			__func__, pFW->size, offset, n);
		offset = -1;
		goto out;
	}
	block->mpData = kmemdup(&data[offset], n, GFP_KERNEL);
	if (block->mpData == NULL) {
		pr_err("%s: mpData memory error\n", __func__);
		offset = -1;
		goto out;
	}
	offset  += n;
out:
	return offset;
}

static int fw_parse_data(struct tasdevice_fw *pFirmware,
	struct TData *pImageData, const struct firmware *fmw, int offset)
{
	const unsigned char *data = (unsigned char *)fmw->data;
	int n;
	unsigned int nBlock;

	if (offset + 64 > fmw->size) {
		pr_err("%s: File Size error\n", __func__);
		n = -1;
		goto out;
	}
	memcpy(pImageData->mpName, &data[offset], 64);
	offset  += 64;

	n = strlen((char *)&data[offset]);
	n++;
	if (offset + n > fmw->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	pImageData->mpDescription = kmemdup(data, n, GFP_KERNEL);
	if (pImageData->mpDescription == NULL) {
		pr_err("%s: FW memory failed!\n", __func__);
		goto out;
	}
	offset  += n;

	if (offset + 2 > fmw->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	pImageData->mnBlocks = be16_to_cpup((__be16 *)&data[offset]);
	offset  += 2;

	pImageData->mpBlocks = kcalloc(pImageData->mnBlocks,
		sizeof(struct TBlock), GFP_KERNEL);
	if (pImageData->mpBlocks == NULL) {
		pr_err("%s: FW memory failed!\n", __func__);
		goto out;
	}
	for (nBlock = 0; nBlock < pImageData->mnBlocks; nBlock++) {
		offset = fw_parse_block_data(pFirmware,
			&(pImageData->mpBlocks[nBlock]), fmw, offset);
		if (offset < 0) {
			offset = -1;
			goto out;
		}
	}
out:
	return offset;
}

static int fw_parse_calibration_data(struct tasdevice_fw *pFirmware,
	const struct firmware *fmw, int offset)
{
	unsigned char *data = (unsigned char *)fmw->data;
	unsigned int nCalibration = 0;
	struct calibration_t *cal = NULL;

	if (offset + 2 > fmw->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	pFirmware->mnCalibrations = be16_to_cpup((__be16 *)&data[offset]);
	offset  += 2;

	if (pFirmware->mnCalibrations != 1) {
		pr_err("%s: only support one calibraiton(%d)!\n",
			__func__, pFirmware->mnCalibrations);
		goto out;
	}

	pFirmware->mpCalibrations =
		kcalloc(pFirmware->mnCalibrations, sizeof(struct calibration_t),
			GFP_KERNEL);
	if (pFirmware->mpCalibrations == NULL) {
		pr_err("%s: mpCalibrations memory failed!\n", __func__);
		offset = -1;
		goto out;
	}
	for (nCalibration = 0; nCalibration < pFirmware->mnCalibrations;
		nCalibration++) {
		unsigned int n;

		if (offset + 64 > fmw->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		cal = &(pFirmware->mpCalibrations[nCalibration]);
		memcpy(cal->mpName, &data[offset], 64);
		offset  += 64;

		n = strlen((char *)&data[offset]);
		n++;
		if (offset + n > fmw->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		cal->mpDescription = kmemdup(&data[offset], n,
			GFP_KERNEL);
		if (cal->mpDescription == NULL) {
			pr_err("%s: mpPrograms memory failed!\n", __func__);
			offset = -1;
			goto out;
		}
		offset += n;

		if (offset + 1 > fmw->size) {
			pr_err("%s: File Size error, offset = %d\n", __func__,
				offset);
			offset = -1;
			goto out;
		}
		cal->mnProgram = data[offset];
		offset++;

		if (offset + 1 > fmw->size) {
			pr_err("%s: File Size error, offset = %d\n", __func__,
				offset);
			offset = -1;
			goto out;
		}
		cal->mnConfiguration = data[offset];
		offset++;

		offset = fw_parse_data(pFirmware, &(cal->mData), fmw,
			offset);
		if (offset < 0)
			goto out;
	}

out:

	return offset;
}

static int fw_parse_program_data(struct tasdevice_fw *pFirmware,
	const struct firmware *fmw, int offset)
{
	struct TProgram *pProgram;
	unsigned char *buf = (unsigned char *)fmw->data;
	int nProgram = 0;

	if (offset + 2 > fmw->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	pFirmware->nr_programs = be16_to_cpup((__be16 *)&buf[offset]);
	offset  += 2;

	if (pFirmware->nr_programs == 0) {
		pr_info("%s: mnPrograms is null, maybe calbin\n", __func__);
		//Do not "offset = -1;", because of calbin
		goto out;
	}

	pFirmware->mpPrograms =
		kcalloc(pFirmware->nr_programs, sizeof(struct TProgram),
			GFP_KERNEL);
	if (pFirmware->mpPrograms == NULL) {
		pr_err("%s: mpPrograms memory failed!\n", __func__);
		offset = -1;
		goto out;
	}
	for (nProgram = 0; nProgram < pFirmware->nr_programs; nProgram++) {
		int n = 0;

		pProgram = &(pFirmware->mpPrograms[nProgram]);
		if (offset + 64 > fmw->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		memcpy(pProgram->mpName, &buf[offset], 64);
		offset  += 64;

		n = strlen((char *)&buf[offset]);
		n++;
		if (offset + n > fmw->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		pProgram->mpDescription = kmemdup(&buf[offset], n, GFP_KERNEL);
		if (pProgram->mpDescription == NULL) {
			pr_err("%s: mpPrograms memory failed!\n", __func__);
			offset = -1;
			goto out;
		}

		offset  += n;

		if (offset + 1 > fmw->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		pProgram->mnAppMode = buf[offset];
		offset++;

		if (offset + 1 > fmw->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		pProgram->mnPDMI2SMode = buf[offset];
		offset++;
		if (offset + 1 > fmw->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		pProgram->mnISnsPD = buf[offset];
		offset++;
		if (offset + 1 > fmw->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		pProgram->mnVSnsPD = buf[offset];
		offset++;
		if (offset + 1 > fmw->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		pProgram->mnPowerLDG = buf[offset];
		offset++;

		offset = fw_parse_data(pFirmware, &(pProgram->mData), fmw,
			offset);
		if (offset < 0)
			goto out;
	}
out:
	return offset;
}

static int fw_parse_configuration_data(struct tasdevice_fw *pFirmware,
	const struct firmware *fmw, int offset)
{
	unsigned char *data = (unsigned char *)fmw->data;
	unsigned int nConfiguration;
	struct TConfiguration *pConfiguration;

	if (offset + 2 > fmw->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	pFirmware->nr_configurations = be16_to_cpup((__be16 *)&data[offset]);
	offset  += 2;

	if (pFirmware->nr_configurations == 0) {
		pr_err("%s: mnConfigurations is zero\n", __func__);
		//Do not "offset = -1;", because of calbin
		goto out;
	}
	pFirmware->mpConfigurations =
		kcalloc(pFirmware->nr_configurations,
				sizeof(struct TConfiguration), GFP_KERNEL);

	for (nConfiguration = 0; nConfiguration < pFirmware->nr_configurations;
		nConfiguration++) {
		int n;

		pConfiguration =
			&(pFirmware->mpConfigurations[nConfiguration]);
		if (offset + 64 > fmw->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		memcpy(pConfiguration->mpName, &data[offset], 64);
		offset  += 64;

		n = strlen((char *)&data[offset]);
		n++;
		if (offset + n > fmw->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		pConfiguration->mpDescription = kmemdup(&data[offset], n,
			GFP_KERNEL);

		if (pConfiguration->mpDescription == NULL) {
			pr_err("%s: FW memory failed!\n", __func__);
			goto out;
		}
		offset  += n;
		if (offset + 2 > fmw->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		pConfiguration->mnDevice_orientation = data[offset];

		pConfiguration->mnDevices = data[offset + 1];
		offset  += 2;

		if (offset + 1 > fmw->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		pConfiguration->mProgram = data[offset];
		offset++;

		if (offset + 4 > fmw->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		pConfiguration->mnSamplingRate =
			be32_to_cpup((__be32 *)&data[offset]);
		offset  += 4;

		if (offset + 1 > fmw->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		pConfiguration->mnPLLSrc = data[offset];
		offset++;

		if (offset + 4 > fmw->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		pConfiguration->mnPLLSrcRate =
			be32_to_cpup((__be32 *)&data[offset]);
		offset  += 4;

		if (offset + 2 > fmw->size) {
			pr_err("%s: File Size error\n", __func__);
			offset = -1;
			goto out;
		}
		pConfiguration->mnFsRate =
			be16_to_cpup((__be16 *)&data[offset]);
		offset  += 2;

		offset = fw_parse_data(pFirmware, &(pConfiguration->mData),
			fmw, offset);
		if (offset < 0)
			goto out;
	}
out:
	return offset;
}

static int fw_parse_header(struct tasdevice_fw *pFirmware,
	const struct firmware *pFW, int offset)
{
	struct tasdevice_dspfw_hdr *pFw_hdr = &(pFirmware->fw_hdr);
	struct tasdevice_fw_fixed_hdr *fw_fixed_hdr = &(pFw_hdr->mnFixedHdr);
	const unsigned char *buf = (unsigned char *)pFW->data;
	int i = 0;
	unsigned char pMagicNumber[] = { 0x35, 0x35, 0x35, 0x32 };

	if (offset + 4 > pFW->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -EINVAL;
		goto out;
	}
	if (memcmp(&buf[offset], pMagicNumber, 4)) {
		pr_err("Firmware: Magic number doesn't match");
		offset = -EINVAL;
		goto out;
	}
	fw_fixed_hdr->mnMagicNumber = be32_to_cpup((__be32 *)&buf[offset]);
	offset  += 4;
	if (offset + 4 > pFW->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	fw_fixed_hdr->mnFWSize = be32_to_cpup((__be32 *)&buf[offset]);
	offset  += 4;
	if (offset + 4 > pFW->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	if (fw_fixed_hdr->mnFWSize != pFW->size) {
		pr_err("File size not match, %d %d", pFW->size,
			fw_fixed_hdr->mnFWSize);
		offset = -1;
		goto out;
	}
	if (offset + 4 > pFW->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	fw_fixed_hdr->mnChecksum = be32_to_cpup((__be32 *)&buf[offset]);
	offset  += 4;
	if (offset + 4 > pFW->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	fw_fixed_hdr->ppcver = be32_to_cpup((__be32 *)&buf[offset]);
	offset  += 4;
	if (offset + 4 > pFW->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	fw_fixed_hdr->mnFWVersion = be32_to_cpup((__be32 *)&buf[offset]);
	offset  += 4;
	if (offset + 4 > pFW->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	fw_fixed_hdr->drv_ver = be32_to_cpup((__be32 *)&buf[offset]);
	offset  += 4;
	if (offset + 4 > pFW->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	for (i = 0; i < sizeof(BinFileformatVerInfo) /
		sizeof(BinFileformatVerInfo[0]); i++) {
		if (BinFileformatVerInfo[i][DRVFWVER] ==
			fw_fixed_hdr->drv_ver) {
			pFw_hdr->mnBinFileDocVer =
				BinFileformatVerInfo[i][BINFILEDOCVER];
			break;
		}
	}
	fw_fixed_hdr->mnTimeStamp = be32_to_cpup((__be32 *)&buf[offset]);
	offset  += 4;
	if (offset + 64 > pFW->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	memcpy(fw_fixed_hdr->mpDDCName, &buf[offset], 64);
	offset  += 64;

 out:
	return offset;
}

static int isInPageYRAM(struct tasdevice_priv *pTAS2781,
	struct TYCRC *pCRCData,
	unsigned char nBook, unsigned char nPage,
	unsigned char nReg, unsigned char len)
{
	int nResult = 0;

	if (nBook == TAS2781_YRAM_BOOK1) {
		if (nPage == TAS2781_YRAM1_PAGE) {
			if (nReg >= TAS2781_YRAM1_START_REG) {
				pCRCData->mnOffset = nReg;
				pCRCData->mnLen = len;
				nResult = 1;
			} else if ((nReg + len) > TAS2781_YRAM1_START_REG) {
				pCRCData->mnOffset = TAS2781_YRAM1_START_REG;
				pCRCData->mnLen =
				len - (TAS2781_YRAM1_START_REG - nReg);
				nResult = 1;
			} else
				nResult = 0;
		} else if (nPage == TAS2781_YRAM3_PAGE) {
			if (nReg > TAS2781_YRAM3_END_REG) {
				nResult = 0;
			} else if (nReg >= TAS2781_YRAM3_START_REG) {
				if ((nReg + len) > TAS2781_YRAM3_END_REG) {
					pCRCData->mnOffset = nReg;
					pCRCData->mnLen =
					TAS2781_YRAM3_END_REG - nReg + 1;
					nResult = 1;
				} else {
					pCRCData->mnOffset = nReg;
					pCRCData->mnLen = len;
					nResult = 1;
				}
			} else {
				if ((nReg + (len-1)) <
					TAS2781_YRAM3_START_REG)
					nResult = 0;
				else {
					pCRCData->mnOffset =
					TAS2781_YRAM3_START_REG;
					pCRCData->mnLen =
					len - (TAS2781_YRAM3_START_REG - nReg);
					nResult = 1;
				}
			}
		}
	} else if (nBook ==
		TAS2781_YRAM_BOOK2) {
		if (nPage == TAS2781_YRAM5_PAGE) {
			if (nReg > TAS2781_YRAM5_END_REG) {
				nResult = 0;
			} else if (nReg >= TAS2781_YRAM5_START_REG) {
				if ((nReg + len) > TAS2781_YRAM5_END_REG) {
					pCRCData->mnOffset = nReg;
					pCRCData->mnLen =
					TAS2781_YRAM5_END_REG - nReg + 1;
					nResult = 1;
				} else {
					pCRCData->mnOffset = nReg;
					pCRCData->mnLen = len;
					nResult = 1;
				}
			} else {
				if ((nReg + (len-1)) <
					TAS2781_YRAM5_START_REG)
					nResult = 0;
				else {
					pCRCData->mnOffset =
					TAS2781_YRAM5_START_REG;
					pCRCData->mnLen =
					len - (TAS2781_YRAM5_START_REG - nReg);
					nResult = 1;
				}
			}
		}
	} else
		nResult = 0;

	return nResult;
}

static int isInBlockYRAM(struct tasdevice_priv *pTAS2781,
	struct TYCRC *pCRCData,
	unsigned char nBook, unsigned char nPage,
	unsigned char nReg, unsigned char len)
{
	int nResult = 0;

	if (nBook == TAS2781_YRAM_BOOK1) {
		if (nPage < TAS2781_YRAM2_START_PAGE)
			nResult = 0;
		else if (nPage <= TAS2781_YRAM2_END_PAGE) {
			if (nReg > TAS2781_YRAM2_END_REG)
				nResult = 0;
			else if (nReg >= TAS2781_YRAM2_START_REG) {
				pCRCData->mnOffset = nReg;
				pCRCData->mnLen = len;
				nResult = 1;
			} else {
				if ((nReg + (len-1)) <
					TAS2781_YRAM2_START_REG)
					nResult = 0;
				else {
					pCRCData->mnOffset =
					TAS2781_YRAM2_START_REG;
					pCRCData->mnLen =
					nReg + len - TAS2781_YRAM2_START_REG;
					nResult = 1;
				}
			}
		} else
			nResult = 0;
	} else if (nBook ==
		TAS2781_YRAM_BOOK2) {
		if (nPage < TAS2781_YRAM4_START_PAGE)
			nResult = 0;
		else if (nPage <= TAS2781_YRAM4_END_PAGE) {
			if (nReg > TAS2781_YRAM2_END_REG)
				nResult = 0;
			else if (nReg >= TAS2781_YRAM2_START_REG) {
				pCRCData->mnOffset = nReg;
				pCRCData->mnLen = len;
				nResult = 1;
			} else {
				if ((nReg + (len-1))
					< TAS2781_YRAM2_START_REG)
					nResult = 0;
				else {
					pCRCData->mnOffset =
					TAS2781_YRAM2_START_REG;
					pCRCData->mnLen =
					nReg + len - TAS2781_YRAM2_START_REG;
					nResult = 1;
				}
			}
		} else
			nResult = 0;
	} else
		nResult = 0;

	return nResult;
}

static int isYRAM(struct tasdevice_priv *pTAS2781, struct TYCRC *pCRCData,
	unsigned char nBook, unsigned char nPage,
	unsigned char nReg, unsigned char len)
{
	int nResult = 0;

	nResult = isInPageYRAM(pTAS2781, pCRCData, nBook, nPage, nReg, len);
	if (nResult == 0)
		nResult = isInBlockYRAM(pTAS2781, pCRCData, nBook,
				nPage, nReg, len);

	return nResult;
}

static int doSingleRegCheckSum(struct tasdevice_priv *tas_priv,
	unsigned short chl, unsigned char nBook, unsigned char nPage,
	unsigned char nReg, unsigned char nValue)
{
	int nResult = 0;
	struct TYCRC sCRCData;
	unsigned int nData1 = 0;

	if ((nBook == TASDEVICE_BOOK_ID(TAS2781_SA_COEFF_SWAP_REG))
		&& (nPage == TASDEVICE_PAGE_ID(TAS2781_SA_COEFF_SWAP_REG))
		&& (nReg >= TASDEVICE_PAGE_REG(TAS2781_SA_COEFF_SWAP_REG))
		&& (nReg <= (TASDEVICE_PAGE_REG(
		TAS2781_SA_COEFF_SWAP_REG) + 4))) {
		/*DSP swap command, pass */
		nResult = 0;
		goto end;
	}

	nResult = isYRAM(tas_priv, &sCRCData, nBook, nPage, nReg, 1);
	if (nResult == 1) {
		nResult = tas_priv->read(tas_priv, chl,
				TASDEVICE_REG(nBook, nPage, nReg), &nData1);
		if (nResult < 0)
			goto end;

		if (nData1 != nValue) {
			dev_err(tas_priv->dev, "error2, B[0x%x]P[0x%x]R[0x%x] "
				"W[0x%x], R[0x%x]\n", nBook, nPage, nReg,
				nValue, nData1);
			nResult = -EAGAIN;
			tas_priv->tasdevice[chl].mnErrCode |=
				ERROR_YRAM_CRCCHK;
			goto end;
		}

		if (nData1 != nValue) {
			dev_err(tas_priv->dev, "error2, B[0x%x]P[0x%x]R[0x%x] "
				"W[0x%x], R[0x%x]\n", nBook, nPage, nReg,
				nValue, nData1);
			nResult = -EAGAIN;
			goto end;
		}

		nResult = crc8(tas_priv->crc8_lkp_tbl, &nValue, 1, 0);
	}

end:
	return nResult;
}

static int doMultiRegCheckSum(struct tasdevice_priv *tas_priv,
	unsigned short chn, unsigned char nBook, unsigned char nPage,
	unsigned char nReg, unsigned int len)
{
	int nResult = 0, i = 0;
	unsigned char nCRCChkSum = 0;
	unsigned char nBuf1[128] = {0};
	struct TYCRC TCRCData;

	if ((nReg + len-1) > 127) {
		nResult = -EINVAL;
		dev_err(tas_priv->dev, "firmware error\n");
		goto end;
	}

	if ((nBook == TASDEVICE_BOOK_ID(TAS2781_SA_COEFF_SWAP_REG))
		&& (nPage == TASDEVICE_PAGE_ID(TAS2781_SA_COEFF_SWAP_REG))
		&& (nReg == TASDEVICE_PAGE_REG(TAS2781_SA_COEFF_SWAP_REG))
		&& (len == 4)) {
		/*DSP swap command, pass */
		nResult = 0;
		goto end;
	}

	nResult = isYRAM(tas_priv, &TCRCData, nBook, nPage, nReg, len);
	dev_info(tas_priv->dev,
		"isYRAM: nBook 0x%x, nPage 0x%x, nReg 0x%x\n",
		nBook, nPage, nReg);
	dev_info(tas_priv->dev,
		"isYRAM: TCRCData.mnLen 0x%x, len 0x%x, nResult %d\n",
		TCRCData.mnLen, len, nResult);
	dev_info(tas_priv->dev, "TCRCData.mnOffset %x\n", TCRCData.mnOffset);
	if (nResult == 1) {
		if (len == 1) {
			dev_err(tas_priv->dev, "firmware error\n");
			nResult = -EINVAL;
			goto end;
		} else {
			nResult = tas_priv->bulk_read(tas_priv, chn,
				TASDEVICE_REG(nBook, nPage, TCRCData.mnOffset),
				nBuf1, TCRCData.mnLen);
			if (nResult < 0)
				goto end;

			for (i = 0; i < TCRCData.mnLen; i++) {
				if ((nBook == TASDEVICE_BOOK_ID(
					TAS2781_SA_COEFF_SWAP_REG))
					&& (nPage == TASDEVICE_PAGE_ID(
						TAS2781_SA_COEFF_SWAP_REG))
					&& ((i + TCRCData.mnOffset)
					>= TASDEVICE_PAGE_REG(
						TAS2781_SA_COEFF_SWAP_REG))
					&& ((i + TCRCData.mnOffset)
					<= (TASDEVICE_PAGE_REG(
						TAS2781_SA_COEFF_SWAP_REG)
						+ 4))) {
					/*DSP swap command, bypass */
					continue;
				} else
					nCRCChkSum  +=
					crc8(tas_priv->crc8_lkp_tbl, &nBuf1[i],
						1, 0);
			}

			nResult = nCRCChkSum;
		}
	}

end:
	return nResult;
}

static int tasdevice_load_block(struct tasdevice_priv *tas_dev,
				struct TBlock *block)
{
	int nResult = 0;
	unsigned int nCommand = 0;
	unsigned char nBook = 0;
	unsigned char nPage = 0;
	unsigned char nOffset = 0;
	unsigned char nData = 0;
	unsigned int nLength = 0;
	unsigned int nSleep = 0;
	unsigned char nCRCChkSum = 0;
	unsigned int nValue = 0;
	int nRetry = 6;
	unsigned char *pData = block->mpData;
	int chn = 0, chnend = 0;

	dev_info(tas_dev->dev,
		"TAS2781 load block: Type = %d, commands = %d\n",
		block->type, block->mnCommands);
	switch (block->type) {
	case MAIN_ALL_DEVICES:
		chn = 0;
		chnend = tas_dev->ndev;
		break;
	case MAIN_DEVICE_A:
	case COEFF_DEVICE_A:
	case PRE_DEVICE_A:
		chn = 0;
		chnend = 1;
		break;
	case MAIN_DEVICE_B:
	case COEFF_DEVICE_B:
	case PRE_DEVICE_B:
		chn = 1;
		chnend = 2;
		break;
	case MAIN_DEVICE_C:
	case COEFF_DEVICE_C:
	case PRE_DEVICE_C:
		chn = 2;
		chnend = 3;
		break;
	case MAIN_DEVICE_D:
	case COEFF_DEVICE_D:
	case PRE_DEVICE_D:
		chn = 3;
		chnend = 4;
		break;
	default:
		dev_info(tas_dev->dev,
			"TAS2781 load block: Other Type = 0x%02x\n",
			block->type);
		break;
	}

	for (; chn < chnend; chn++) {
		if (tas_dev->tasdevice[chn].bLoading == false)
			continue;
start:
		if (block->mbPChkSumPresent) {
			nResult = tas_dev->write(tas_dev, chn,
				TASDEVICE_I2CChecksum, 0);
			if (nResult < 0)
				goto end;
		}

		if (block->mbYChkSumPresent)
			nCRCChkSum = 0;

		nCommand = 0;

		while (nCommand < block->mnCommands) {
			pData = block->mpData + nCommand * 4;

			nBook = pData[0];
			nPage = pData[1];
			nOffset = pData[2];
			nData = pData[3];

			nCommand++;

			if (nOffset <= 0x7F) {
				nResult = tas_dev->write(tas_dev, chn,
					TASDEVICE_REG(nBook, nPage, nOffset),
					nData);
				if (nResult < 0)
					goto end;
				if (block->mbYChkSumPresent) {
					nResult = doSingleRegCheckSum(tas_dev,
						chn, nBook, nPage, nOffset,
						nData);
					if (nResult < 0)
						goto check;
					nCRCChkSum  += (unsigned char)nResult;
				}
			} else if (nOffset == 0x81) {
				nSleep = (nBook << 8) + nPage;
				msleep(nSleep);
			} else if (nOffset == 0x85) {
				pData  += 4;
				nLength = (nBook << 8) + nPage;
				nBook = pData[0];
				nPage = pData[1];
				nOffset = pData[2];
				if (nLength > 1) {
					nResult = tas_dev->bulk_write(tas_dev,
						chn, TASDEVICE_REG(nBook,
						nPage, nOffset), pData + 3,
						nLength);
					if (nResult < 0)
						goto end;
					if (block->mbYChkSumPresent) {
						nResult = doMultiRegCheckSum(
							tas_dev, chn, nBook,
							nPage, nOffset,
							nLength);
						if (nResult < 0)
							goto check;
						nCRCChkSum  +=
							(unsigned char)nResult;
					}
				} else {
					nResult = tas_dev->write(tas_dev, chn,
						TASDEVICE_REG(nBook, nPage,
						nOffset),
						pData[3]);
					if (nResult < 0)
						goto end;
					if (block->mbYChkSumPresent) {
						nResult = doSingleRegCheckSum(
							tas_dev, chn, nBook,
							nPage, nOffset,
							pData[3]);
						if (nResult < 0)
							goto check;
						nCRCChkSum  +=
							(unsigned char)nResult;
					}
				}

				nCommand++;

				if (nLength >= 2)
					nCommand  += ((nLength - 2) / 4) + 1;
			}
		}
		if (block->mbPChkSumPresent) {
			nResult = tas_dev->read(tas_dev, chn,
				TASDEVICE_I2CChecksum, &nValue);
			if (nResult < 0) {
				dev_err(tas_dev->dev, "%s: Channel %d\n",
					__func__, chn);
				goto check;
			}
			if ((nValue&0xff) != block->mnPChkSum) {
				dev_err(tas_dev->dev,
					"Block PChkSum Channel %d Error: "
					"FW = 0x%x, Reg = 0x%x\n", chn,
					block->mnPChkSum, (nValue&0xff));
				nResult = -EAGAIN;
				tas_dev->tasdevice[chn].mnErrCode |=
					ERROR_PRAM_CRCCHK;
				goto check;
			}
			nResult = 0;
			tas_dev->tasdevice[chn].mnErrCode &=
				~ERROR_PRAM_CRCCHK;
			dev_info(tas_dev->dev, "Block[0x%02x] PChkSum match\n",
				block->type);
		}

		if (block->mbYChkSumPresent) {
			//TBD, open it when FW ready
			dev_err(tas_dev->dev, "Block YChkSum: FW = 0x%x, "
				"YCRC = 0x%x\n", block->mnYChkSum,
				nCRCChkSum);

			tas_dev->tasdevice[chn].mnErrCode &=
				~ERROR_YRAM_CRCCHK;
			nResult = 0;
			dev_info(tas_dev->dev,
				"Block[0x%x] YChkSum match\n", block->type);
		}
check:
		if (nResult == -EAGAIN) {
			nRetry--;
			if (nRetry > 0)
				goto start;
			else {
				//tas_dev->tasdevice[chn].bLoading = false;
				if ((MAIN_ALL_DEVICES == block->type)
					|| (MAIN_DEVICE_A == block->type)
					|| (MAIN_DEVICE_B == block->type)
					|| (MAIN_DEVICE_C == block->type)
					|| (MAIN_DEVICE_D == block->type)) {
					tas_dev->tasdevice[chn].
						mnCurrentProgram = -1;
				} else {
					tas_dev->tasdevice[chn].
						mnCurrentConfiguration = -1;
				}
				nRetry = 6;
			}
		}
	}
end:
	if (nResult < 0) {
		dev_err(tas_dev->dev, "Block (%d) load error\n",
				block->type);
	}
	return nResult;
}


static int tasdevice_load_data(struct tasdevice_priv *tas_dev,
	struct TData *pData)
{
	int nResult = 0;
	unsigned int nBlock = 0;
	struct TBlock *block = NULL;

	dev_info(tas_dev->dev, "%s: TAS2781 load data: %s, Blocks = %d\n",
		__func__,
		pData->mpName, pData->mnBlocks);

	for (nBlock = 0; nBlock < pData->mnBlocks; nBlock++) {
		block = &(pData->mpBlocks[nBlock]);
		nResult = tas_dev->tasdevice_load_block(tas_dev, block);
		if (nResult < 0)
			break;
	}

	return nResult;
}

static int tasdevice_load_calibrated_data(
	struct tasdevice_priv *tas_dev, struct TData *pData)
{
	int nResult = 0;
	unsigned int nBlock = 0;
	struct TBlock *block = NULL;

	dev_info(tas_dev->dev, "%s: TASDEVICE load data: %s, Blocks = %d\n",
		__func__,
		pData->mpName, pData->mnBlocks);

	for (nBlock = 0; nBlock < pData->mnBlocks; nBlock++) {
		block = &(pData->mpBlocks[nBlock]);
		nResult = tasdevice_load_block(tas_dev, block);
		if (nResult < 0)
			break;
	}

	return nResult;
}

int tas2781_load_calibration(void *pContext, char *pFileName,
	unsigned short i)
{
	int ret = 0, offset = 0;
	struct firmware FW;
	const struct firmware *fw_entry = NULL;
	struct tasdevice_priv *tas_dev = (struct tasdevice_priv *)pContext;
	struct tasdevice_t *pTasdev = &(tas_dev->tasdevice[i]);
	struct tasdevice_fw *mpCalFirmware = NULL;

	dev_info(tas_dev->dev, "%s: enter\n", __func__);

	ret = request_firmware(&fw_entry, pFileName, tas_dev->dev);
	if (!ret) {
		if (!fw_entry->size) {
			dev_err(tas_dev->dev,
				"%s: file read error: size = %d\n",
				__func__, fw_entry->size);
			goto out;
		}
		FW.size = fw_entry->size;
		FW.data = fw_entry->data;
		dev_info(tas_dev->dev, "%s: file = %s, file size %zd\n",
			__func__, pFileName, fw_entry->size);
	} else {
		dev_info(tas_dev->dev, "%s: Request firmware failed\n", __func__);
		goto out;
	}

	mpCalFirmware = pTasdev->mpCalFirmware = kzalloc(
		sizeof(struct tasdevice_fw), GFP_KERNEL);
	if (pTasdev->mpCalFirmware == NULL) {
		dev_err(tas_dev->dev, "%s: FW memory failed!\n", __func__);
		ret = -1;
		goto out;
	}
	offset = fw_parse_header(mpCalFirmware, &FW, offset);
	if (offset == -1) {
		dev_err(tas_dev->dev, "%s: EXIT!\n", __func__);
		goto out;
	}
	offset = fw_parse_variable_header_cal(mpCalFirmware, &FW, offset);
	if (offset == -1) {
		dev_err(tas_dev->dev, "%s: EXIT!\n", __func__);
		goto out;
	}
	offset = fw_parse_program_data(mpCalFirmware, &FW, offset);
	if (offset == -1) {
		dev_err(tas_dev->dev, "%s: EXIT!\n", __func__);
		goto out;
	}
	offset = fw_parse_configuration_data(mpCalFirmware, &FW, offset);
	if (offset == -1) {
		dev_err(tas_dev->dev, "%s: EXIT!\n", __func__);
		goto out;
	}
	offset = fw_parse_calibration_data(mpCalFirmware, &FW, offset);
	if (offset == -1) {
		dev_err(tas_dev->dev, "%s: EXIT!\n", __func__);
		goto out;
	}

out:
	if (fw_entry) {
		release_firmware(fw_entry);
		fw_entry = NULL;
	}
	return ret;
}

static int dspfw_default_callback(struct tasdevice_priv *tas_priv,
	unsigned int drv_ver, unsigned int ppcver)
{
	int rc = 0;

	if (drv_ver == 0x100) {
		if (ppcver >= PPC3_VERSION) {
			tas_priv->fw_parse_variable_header =
				fw_parse_variable_header_kernel;
			tas_priv->fw_parse_program_data =
				fw_parse_program_data_kernel;
			tas_priv->fw_parse_configuration_data =
				fw_parse_configuration_data_kernel;
			tas_priv->tasdevice_load_block =
				tasdevice_load_block_kernel;
		} else {
			switch (ppcver) {
			case 0x00:
				tas_priv->fw_parse_variable_header =
					fw_parse_variable_header_git;
				tas_priv->fw_parse_program_data =
					fw_parse_program_data;
				tas_priv->fw_parse_configuration_data =
					fw_parse_configuration_data;
				tas_priv->tasdevice_load_block =
					tasdevice_load_block;
				break;
			default:
				dev_err(tas_priv->dev,
					"%s: PPCVer must be 0x0 or 0x%02x",
					__func__, PPC3_VERSION);
				dev_err(tas_priv->dev, " Current:0x%02x\n",
					ppcver);
				rc = -EINVAL;
				break;
			}
		}
	} else {
		dev_err(tas_priv->dev,
			"DrvVer must be 0x0, 0x230 or above 0x230 ");
		dev_err(tas_priv->dev, "current is 0x%02x\n", drv_ver);
		rc = -EINVAL;
	}

	return rc;
}

int tasdevice_dspfw_ready(const void *pVoid, void *pContext)
{
	struct tasdevice_priv *tas_dev = (struct tasdevice_priv *) pContext;
	const struct firmware *pFW = (const struct firmware *)pVoid;
	struct tasdevice_fw *pFirmware = NULL;
	struct tasdevice_fw_fixed_hdr *fw_fixed_hdr;
	int offset = 0, ret = 0;

	if (!pFW || !pFW->data) {
		dev_err(tas_dev->dev, "%s: Failed to read firmware %s\n",
			__func__, tas_dev->dsp_binaryname);
		ret = -1;
		goto out;
	}

	tas_dev->fmw = kzalloc(sizeof(struct tasdevice_fw), GFP_KERNEL);
	if (tas_dev->fmw == NULL) {
		dev_err(tas_dev->dev, "%s: FW memory failed!\n", __func__);
		ret = -1;
		goto out;
	}
	pFirmware = tas_dev->fmw;

	offset = fw_parse_header(pFirmware, pFW, offset);

	if (offset == -1)
		goto out;
	fw_fixed_hdr = &(pFirmware->fw_hdr.mnFixedHdr);
	switch (fw_fixed_hdr->drv_ver) {
	case 0x301:
	case 0x302:
	case 0x502:
	case 0x503:
		tas_dev->fw_parse_variable_header =
			fw_parse_variable_header_kernel;
		tas_dev->fw_parse_program_data =
			fw_parse_program_data_kernel;
		tas_dev->fw_parse_configuration_data =
			fw_parse_configuration_data_kernel;
		tas_dev->tasdevice_load_block =
			tasdevice_load_block_kernel;
		pFirmware->bKernelFormat = true;
		break;
	case 0x202:
	case 0x400:
	case 0x401:        
		tas_dev->fw_parse_variable_header =
			fw_parse_variable_header_git;
		tas_dev->fw_parse_program_data =
			fw_parse_program_data;
		tas_dev->fw_parse_configuration_data =
			fw_parse_configuration_data;
		tas_dev->tasdevice_load_block =
			tasdevice_load_block;
		pFirmware->bKernelFormat = false;
		break;
	default:
		ret = dspfw_default_callback(tas_dev,
			fw_fixed_hdr->drv_ver, fw_fixed_hdr->ppcver);
		if (ret)
			goto out;
		break;
	}

	offset = tas_dev->fw_parse_variable_header(tas_dev, pFW, offset);
	if (offset == -1)
		goto out;

	offset = tas_dev->fw_parse_program_data(pFirmware, pFW, offset);
	if (offset < 0) {
		ret = -1;
		goto out;
	}
	offset = tas_dev->fw_parse_configuration_data(pFirmware, pFW, offset);
	if (offset < 0)
		ret = -1;

out:
	return ret;
}

void tasdevice_calbin_remove(void *pContext)
{
	struct tasdevice_priv *tas_dev = (struct tasdevice_priv *) pContext;
	struct tasdevice_t *pTasdev = NULL;
	int i = 0;

	if (tas_dev) {
		for (i = 0; i < tas_dev->ndev; i++) {
			pTasdev = &(tas_dev->tasdevice[i]);
			if (pTasdev->mpCalFirmware) {
				tas2781_clear_calfirmware(pTasdev->mpCalFirmware);
				pTasdev->mpCalFirmware = NULL;
			}
		}
	}
}

void tasdevice_dsp_remove(void *pContext)
{
	struct tasdevice_priv *tas_dev = (struct tasdevice_priv *) pContext;
	int i = 0;

	if (tas_dev) {
		if (tas_dev->fmw) {
			struct tasdevice_fw *pFirmware = tas_dev->fmw;

			if (pFirmware->mpPrograms) {
				struct TProgram *pProgram;

				for (i = 0; i < pFirmware->nr_programs; i++) {
					pProgram = &(pFirmware->mpPrograms[i]);
					if (pProgram) {
						struct TData *pImageData =
							&(pProgram->mData);

						if (pImageData->mpBlocks) {
							struct TBlock *pBlock;
							unsigned int nBlock;

							for (nBlock = 0;
								nBlock <
									pImageData->mnBlocks;
								nBlock++) {
								pBlock =
									&(pImageData->mpBlocks[nBlock]);
								if (pBlock) {
									kfree(pBlock->mpData);
								}
							}
							kfree(pImageData->mpBlocks);
						}
						if(pImageData->mpDescription)
							kfree(pImageData->mpDescription);
						if(pProgram->mpDescription)
							kfree(pProgram->mpDescription);
					}
				}
				kfree(pFirmware->mpPrograms);
			}

			if (pFirmware->mpConfigurations) {
				struct TConfiguration *pConfig;

				for (i = 0; i < pFirmware->nr_configurations;
					i++) {
					pConfig = &(pFirmware->
						mpConfigurations[i]);
					if (pConfig) {
						struct TData *pImageData =
							&(pConfig->mData);

						if (pImageData->mpBlocks) {
							struct TBlock *pBlock;
							unsigned int nBlock;

							for (nBlock = 0;
								nBlock <
									pImageData->mnBlocks;
								nBlock++) {
								pBlock =
									&(pImageData->mpBlocks[nBlock]);
								if (pBlock) {
									kfree(pBlock->mpData);
								}
							}
							kfree(pImageData->mpBlocks);
						}
						if(pImageData->mpDescription)
							kfree(pImageData->mpDescription);
						kfree(pConfig->mpDescription);
					}
				}
				kfree(pFirmware->mpConfigurations);
			}
			kfree(pFirmware->fw_hdr.mpDescription);
			kfree(pFirmware);
			tas_dev->fmw = NULL;
		}
	}
}

int tasdevice_select_tuningprm_cfg(void *pContext, int prm_no,
	int cfg_no, int regbin_conf_no)
{
	struct tasdevice_priv *tas_dev = (struct tasdevice_priv *) pContext;
	struct tasdevice_regbin *regbin = &(tas_dev->mtRegbin);
	struct tasdevice_config_info **cfg_info = regbin->cfg_info;
	struct tasdevice_fw *pFirmware = tas_dev->fmw;
	struct TConfiguration *pConfigurations = NULL;
	struct TProgram *pProgram = NULL;
	int i = 0, status = 0, prog_status = 0;

	if (pFirmware == NULL) {
		dev_err(tas_dev->dev, "%s: Firmware is NULL\n", __func__);
		goto out;
	}

	if (cfg_no >= pFirmware->nr_configurations) {
		dev_err(tas_dev->dev,
			"%s: cfg(%d) is not in range of conf %u\n",
			__func__, cfg_no, pFirmware->nr_configurations);
		goto out;
	}

	if (prm_no >= pFirmware->nr_programs) {
		dev_err(tas_dev->dev,
			"%s: prm(%d) is not in range of Programs %u\n",
			__func__,  prm_no, pFirmware->nr_programs);
		goto out;
	}

	if (regbin_conf_no > regbin->ncfgs || regbin_conf_no < 0 ||
		cfg_info == NULL) {
		dev_err(tas_dev->dev,
			"conf_no:%d should be in range from 0 to %u\n",
			regbin_conf_no, regbin->ncfgs-1);
		goto out;
	} else
		dev_info(tas_dev->dev, "%s: regbin_profile_conf_id = %d\n",
			__func__, regbin_conf_no);


	pConfigurations = &(pFirmware->mpConfigurations[cfg_no]);
	for (i = 0; i < tas_dev->ndev; i++) {
		if (cfg_info[regbin_conf_no]->active_dev & (1 << i)) {
			if (tas_dev->tasdevice[i].prg_download_cnt <
				TASDEVICE_MAX_DOWNLOAD_CNT &&
				tas_dev->tasdevice[i].mnCurrentProgram != prm_no) {
				/* After download fw, dsp config must be redownload */
				tas_dev->tasdevice[i].mnCurrentConfiguration = -1;
				tas_dev->tasdevice[i].bLoading = true;
				prog_status++;

				dev_dbg(tas_dev->dev, "%s: dev-%d cnt = %d\n", __func__,
					i, tas_dev->tasdevice[i].prg_download_cnt);
			}
		} else
			tas_dev->tasdevice[i].bLoading = false;
		tas_dev->tasdevice[i].bLoaderr = false;
	}

	if (prog_status) {
		pProgram = &(pFirmware->mpPrograms[prm_no]);
		tasdevice_load_data(tas_dev, &(pProgram->mData));
		for (i = 0; i < tas_dev->ndev; i++) {
			if (tas_dev->tasdevice[i].bLoaderr == true) {
				tas_dev->tasdevice[i].mnCurrentProgram = -1;
				tas_dev->tasdevice[i].prg_download_cnt++;
				continue;
			} else if (tas_dev->tasdevice[i].bLoaderr == false
				&& tas_dev->tasdevice[i].bLoading == true) {
				struct tasdevice_fw *cal_fw =
					tas_dev->tasdevice[i].mpCalFirmware;

				if (cal_fw) {
					struct calibration_t *cal =
						cal_fw->mpCalibrations;

					if (cal)
						tasdevice_load_calibrated_data(
							tas_dev, &(cal->mData));
				}
				tas_dev->tasdevice[i].mnCurrentProgram
					= prm_no;
			}
		}
	}

	for (i = 0; i < tas_dev->ndev; i++) {
		dev_info(tas_dev->dev, "%s,dsp-conf:%d, active-dev:%d, loaderr:%d\n",
			__func__, tas_dev->tasdevice[i].mnCurrentConfiguration,
			cfg_info[regbin_conf_no]->active_dev,
			tas_dev->tasdevice[i].bLoaderr);
		if (tas_dev->tasdevice[i].mnCurrentConfiguration != cfg_no
			&& (cfg_info[regbin_conf_no]->active_dev & (1 << i))
			&& (tas_dev->tasdevice[i].bLoaderr == false)) {
			status++;
			tas_dev->tasdevice[i].bLoading = true;
		} else
			tas_dev->tasdevice[i].bLoading = false;
	}

	if (status) {
		status = 0;
		tasdevice_load_data(tas_dev, &(pConfigurations->mData));
		for (i = 0; i < tas_dev->ndev; i++) {
			if (tas_dev->tasdevice[i].mnCurrentProgram == -1) {
				status |= 1 << (i + 4);
				continue;
			} else if (tas_dev->tasdevice[i].bLoaderr == false
				&& tas_dev->tasdevice[i].bLoading == true)
				tas_dev->tasdevice[i].mnCurrentConfiguration
					= cfg_no;
		}
	} else
		dev_info(tas_dev->dev,
			"%s: No device is in active in conf %d\n",
			__func__, regbin_conf_no);

	dev_info(tas_dev->dev, "%s: DSP mode: load status is %08x\n",
		__func__, status);

	/* After downloading program, global mode will be reset.
	 */
	if (prog_status && tas_dev->set_global_mode)
			tas_dev->set_global_mode(tas_dev);
out:
	return prog_status;
}

int tas2781_set_calibration(void *pContext, unsigned short i,
	int nCalibration)
{
	struct tasdevice_priv *tas_dev = (struct tasdevice_priv *) pContext;
	struct tasdevice_t *tasdevice = &(tas_dev->tasdevice[i]);
	struct tasdevice_fw *cal_fw = tasdevice->mpCalFirmware;
	int nResult = 0;

	dev_info(tas_dev->dev, "%s start\n", __func__);
	if ((!tas_dev->fmw->mpPrograms)
		|| (!tas_dev->fmw->mpConfigurations)) {
		dev_err(tas_dev->dev, "%s, Firmware not found\n\r", __func__);
		goto out;
	}

	if (tasdevice->mnCurrentProgram == -1) {
		dev_dbg(tas_dev->dev, "%s, Firmware prg not loaded\n\r", __func__);
		goto out;
	}

	if (nCalibration == 0xFF || nCalibration == 0x100) {
		if (cal_fw) {
			tas2781_clear_calfirmware(cal_fw);
			cal_fw = NULL;
		}

		scnprintf(tas_dev->cal_binaryname[i], 64, "%s-0x%02x-cal.bin",
			tas_dev->dev_name, tas_dev->tasdevice[i].mnDevAddr);
		nResult = tas2781_load_calibration(tas_dev,
			tas_dev->cal_binaryname[i], i);
		if (nResult != 0) {
			dev_err(tas_dev->dev, "%s: load %s error, "
				"no-side effect for playback\n",
				__func__, tas_dev->cal_binaryname[i]);
			nResult = 0;
		}
	}

	if (cal_fw) {
		struct calibration_t *cal = cal_fw->mpCalibrations;

		if (cal)
			tasdevice_load_calibrated_data(tas_dev, &(cal->mData));
	} else
		dev_err(tas_dev->dev,
			"%s: No calibrated data for device %d\n", __func__, i);

out:
	return nResult;
}
