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

#ifndef __TASDEVICE_NODE_H__
#define __TASDEVICE_NODE_H__

ssize_t dspfwinfo_list_show(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t active_address_show(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t reg_show(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t reg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
ssize_t regdump_show(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t regdump_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
ssize_t regbininfo_list_show(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t regcfg_list_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
ssize_t regcfg_list_show(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t dspfw_config_show(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t dev_addr_show(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t force_fw_load_chip_show(struct device *dev,
				struct device_attribute *attr, char *buf);
ssize_t force_fw_load_chip_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
#endif
