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

#include "tasdevice-dsp.h"
#include "tasdevice-regbin.h"
#include "tasdevice.h"
#include "tasdevice-dsp_git.h"

const char deviceNumber[TASDEVICE_DSP_TAS_MAX_DEVICE] = {
	1, 2, 1, 2, 1, 1, 0, 2, 4, 3, 1, 2, 3, 4
};

int fw_parse_variable_header_git(struct tasdevice_priv *tas_dev,
	const struct firmware *pFW, int offset)
{
	const unsigned char *buf = pFW->data;
	struct tasdevice_fw *pFirmware = tas_dev->fmw;
	struct tasdevice_dspfw_hdr *pFw_hdr = &(pFirmware->fw_hdr);
	int i = strlen((char *)&buf[offset]);

	i++;

	if (offset + i > pFW->size) {
		dev_err(tas_dev->dev, "%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}

	pFw_hdr->mpDescription = kmemdup(&buf[offset], i, GFP_KERNEL);
	if (pFw_hdr->mpDescription == NULL) {
		dev_err(tas_dev->dev, "%s: mpDescription error!\n", __func__);
		goto out;
	}
	offset  += i;

	if (offset + 4 > pFW->size) {
		dev_err(tas_dev->dev, "%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	pFw_hdr->mnDeviceFamily = be32_to_cpup((__be32 *)&buf[offset]);
	if (pFw_hdr->mnDeviceFamily != 0) {
		dev_err(tas_dev->dev, "ERROR:%s: not TAS device\n", __func__);
		offset = -1;
		goto out;
	}
	offset  += 4;
	if (offset + 4 > pFW->size) {
		dev_err(tas_dev->dev, "%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	pFw_hdr->mnDevice = be32_to_cpup((__be32 *)&buf[offset]);
	if (pFw_hdr->mnDevice >= TASDEVICE_DSP_TAS_MAX_DEVICE ||
		pFw_hdr->mnDevice == 6) {
		dev_err(tas_dev->dev, "ERROR:%s: not support device %d\n",
			__func__, pFw_hdr->mnDevice);
		offset = -1;
		goto out;
	}
	offset  += 4;
	pFw_hdr->ndev = deviceNumber[pFw_hdr->mnDevice];
	if (pFw_hdr->ndev != tas_dev->ndev) {
		dev_err(tas_dev->dev, "%s: ndev(%u) in dspbin dismatch "
			"ndev(%u) in DTS\n", __func__, pFw_hdr->ndev,
			tas_dev->ndev);
		offset = -1;
	}

out:
	return offset;
}

int fw_parse_variable_header_cal(struct tasdevice_fw *pCalFirmware,
	const struct firmware *pFW, int offset)
{
	const unsigned char *buf = pFW->data;
	struct tasdevice_dspfw_hdr *pFw_hdr = &(pCalFirmware->fw_hdr);
	int i = strlen((char *)&buf[offset]);

	i++;

	if (offset + i > pFW->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}

	pFw_hdr->mpDescription = kmemdup(&buf[offset], i, GFP_KERNEL);
	if (pFw_hdr->mpDescription == NULL) {
		pr_err("%s: mpDescription error!\n", __func__);
		goto out;
	}
	offset  += i;

	if (offset + 4 > pFW->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	pFw_hdr->mnDeviceFamily = be32_to_cpup((__be32 *)&buf[offset]);
	if (pFw_hdr->mnDeviceFamily != 0) {
		pr_err("ERROR:%s: not TAS device\n", __func__);
		offset = -1;
		goto out;
	}
	offset  += 4;
	if (offset + 4 > pFW->size) {
		pr_err("%s: File Size error\n", __func__);
		offset = -1;
		goto out;
	}
	pFw_hdr->mnDevice = be32_to_cpup((__be32 *)&buf[offset]);
	if (pFw_hdr->mnDevice >= TASDEVICE_DSP_TAS_MAX_DEVICE ||
		pFw_hdr->mnDevice == 6) {
		pr_err("ERROR:%s: not support device %d\n",
			__func__, pFw_hdr->mnDevice);
		offset = -1;
		goto out;
	}
	offset  += 4;
	pFw_hdr->ndev = deviceNumber[pFw_hdr->mnDevice];
	if (pFw_hdr->ndev != 1) {
		pr_err("%s: calbin must be 1, but currently ndev(%u)\n",
			__func__, pFw_hdr->ndev);
		offset = -1;
	}

out:
	return offset;
}
