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

#ifndef __TASDEVICE_RW_H__
#define __TASDEVICE_RW_H__
int tasdevice_dev_read(struct tasdevice_priv *pPcmdev,
	unsigned short chn, unsigned int reg, unsigned int *pValue);

int tasdevice_dev_write(struct tasdevice_priv *pPcmdev,
	unsigned short chn, unsigned int reg, unsigned int value);

int tasdevice_dev_bulk_write(
	struct tasdevice_priv *pPcmdev, unsigned short chn,
	unsigned int reg, unsigned char *p_data, unsigned int n_length);

int tasdevice_dev_bulk_read(struct tasdevice_priv *pPcmdev,
	unsigned short chn, unsigned int reg, unsigned char *p_data,
	unsigned int n_length);

int tasdevice_dev_update_bits(
	struct tasdevice_priv *pPcmdev, unsigned short chn,
	unsigned int reg, unsigned int mask, unsigned int value);
#endif
