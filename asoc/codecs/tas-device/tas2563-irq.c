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
#include <sound/soc.h>

#include "tasdevice.h"
#include "tas2563-reg.h"
#include "tasdevice-rw.h"

void tas2563_irq_work_func(struct tasdevice_priv *tas_dev)
{
	int rc = 0;
	unsigned int reg_val = 0, array_size = 0, i = 0, ndev = 0;
	unsigned int int_reg_array[] = {
		TAS2563_REG_INT_LTCH0,
		TAS2563_REG_INT_LTCH1,
		TAS2563_REG_INT_LTCH3,
		TAS2563_REG_INT_LTCH4};

	tasdevice_enable_irq(tas_dev, false);

	array_size = ARRAY_SIZE(int_reg_array);

	for (ndev = 0; ndev < tas_dev->ndev; ndev++) {
		for (i = 0; i < array_size; i++) {
			rc = tasdevice_dev_read(tas_dev,
				ndev, int_reg_array[i], &reg_val);
			if (!rc) {
				dev_info(tas_dev->dev,
					"INT STATUS REG 0x%04x=0x%02x\n",
					int_reg_array[i], reg_val);
			} else {
				dev_err(tas_dev->dev,
					"Read Reg 0x%04x error(rc=%d)\n",
					int_reg_array[i], rc);
			}
		}
	}

}
