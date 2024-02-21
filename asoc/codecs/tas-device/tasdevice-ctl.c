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
#include "tasdevice-ctl.h"

void tasdevice_deinit(void *pContext)
{
	struct tasdevice_priv *tas_dev = (struct tasdevice_priv *) pContext;

	tasdevice_config_info_remove(tas_dev);
	tasdevice_dsp_remove(tas_dev);
	tasdevice_calbin_remove(tas_dev);
	tas_dev->fw_state = TASDEVICE_DSP_FW_PENDING;
}
