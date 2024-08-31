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

#ifndef __TASDEV_CODEC_H__
#define __TASDEV_CODEC_H__
int tasdevice_register_codec(struct tasdevice_priv *tas_priv);
void tasdevice_deregister_codec(struct tasdevice_priv *tas_priv);
int tasdevice_create_controls(struct tasdevice_priv *tas_priv);
int tasdevice_dsp_create_control(struct tasdevice_priv
	*tas_priv);
void powercontrol_routine(struct work_struct *work);
#endif
