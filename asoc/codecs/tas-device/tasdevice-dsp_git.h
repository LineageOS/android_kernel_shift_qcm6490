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

#ifndef __TASDEVICE_DSP_GIT_H__
#define __TASDEVICE_DSP_GIT_H__
int fw_parse_variable_header_git(struct tasdevice_priv *tas_dev,
	const struct firmware *pFW, int offset);
int fw_parse_variable_header_cal(struct tasdevice_fw *pCalFirmware,
	const struct firmware *pFW, int offset);
#endif
