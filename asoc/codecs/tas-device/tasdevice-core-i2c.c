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
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include "tasdevice.h"
#include "tasdevice-rw.h"
#include "tasdevice-node.h"
#ifndef CONFIG_TASDEV_CODEC_SPI

static const struct regmap_range_cfg tasdevice_ranges[] = {
	{
		.range_min = 0,
		.range_max = 256 * 128,
		.selector_reg = TASDEVICE_PAGE_SELECT,
		.selector_mask = 0xff,
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 128,
	},
};

static bool tas2781_volatile(struct device *dev, unsigned int reg)
{
	return true;
}

static const struct regmap_config tasdevice_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.ranges = tasdevice_ranges,
	.volatile_reg = tas2781_volatile,
	.num_ranges = ARRAY_SIZE(tasdevice_ranges),
	.max_register = 256 * 128,
};

static void tas2781_set_global_mode(struct tasdevice_priv *tas_dev)
{
	int i = 0, ret = 0;

	for (; i < tas_dev->ndev; i++) {
		ret = tasdevice_dev_update_bits(tas_dev, i,
			TAS2871_MISC_CFG2, TASDEVICE_GLOBAL_ADDR_MASK,
			TASDEVICE_GLOBAL_ADDR_ENABLE);
		if (ret < 0) {
			dev_err(tas_dev->dev, "chn %d set global fail, %d\n",
				i, ret);
		}
	}
}

static void tas2563_set_global_mode(struct tasdevice_priv *tas_dev)
{
	int i = 0, ret = 0;

	for (; i < tas_dev->ndev; i++) {
		ret = tasdevice_dev_update_bits(tas_dev, i,
			TAS2563_MISC_CFG2, TASDEVICE_GLOBAL_ADDR_MASK,
			TASDEVICE_GLOBAL_ADDR_ENABLE);
		if (ret < 0) {
			dev_err(tas_dev->dev, "chn %d set global fail, %d\n",
				i, ret);
		}
	}
}

static int tasdevice_i2c_parse_dt(struct tasdevice_priv *tas_priv)
{
	struct i2c_client *client = (struct i2c_client *)tas_priv->client;
	struct device_node *np = tas_priv->dev->of_node;
	unsigned int dev_addrs[TASDEVICE_MAX_CHANNELS];
	int len, sw, aw, i, ndev;
	const __be32 *reg, *reg_end;

	dev_info(tas_priv->dev, "%s, chip_id:%d\n", __func__,
		tas_priv->chip_id);
	strcpy(tas_priv->dev_name, tasdevice_id[tas_priv->chip_id].name);

	aw = of_n_addr_cells(np);
	sw = of_n_size_cells(np);
	if (sw == 0) {
		reg = (const __be32 *)of_get_property(np,
			"reg", &len);
		reg_end = reg + len/sizeof(*reg);
		ndev = 0;
		do {
			dev_addrs[ndev] = of_read_number(reg, aw);
			reg += aw;
			ndev++;
		} while (reg < reg_end);
	} else {
		ndev = 1;
		dev_addrs[0] = client->addr;
	}

	if (ndev > TAS2563_MAX_DEVICE && TAS2563 == tas_priv->chip_id) {
		dev_info(tas_priv->dev, "Do not support more than 4 TAS2563s "
			"on the same I2C bus, force the device number from %d "
			"to %d ", ndev, TAS2563_MAX_DEVICE);
		ndev = TAS2563_MAX_DEVICE;
	}

	tas_priv->ndev = (ndev == 0) ? 1 : ndev;

	for (i = 0; i < ndev; i++)
		tas_priv->tasdevice[i].mnDevAddr = dev_addrs[i];

	if (of_property_read_bool(np, "ti,global-addr-enable"))
		/* Enable I2C broadcast */
		tas_priv->glb_addr.dev_addr = TASDEVICE_GLOBAL_ADDR;
	else
		tas_priv->glb_addr.dev_addr = 0;

	tasdevice_parse_dt_reset_irq_pin(tas_priv, np);

	return 0;
}

static int tasdevice_i2c_probe(struct i2c_client *i2c,
	const struct i2c_device_id *id)
{
	struct tasdevice_priv *tas_dev = NULL;
	int ret = 0;

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev,
			"%s: I2C check failed\n", __func__);
		ret = -ENODEV;
		goto out;
	}

	tas_dev = devm_kzalloc(&i2c->dev, sizeof(*tas_dev), GFP_KERNEL);
	if (!tas_dev) {
		dev_err(&i2c->dev,
			"probe devm_kzalloc failed %d\n", ret);
		ret = -ENOMEM;
		goto out;
	}
	tas_dev->dev = &i2c->dev;
	tas_dev->client = (void *)i2c;
	tas_dev->chip_id = id->driver_data;

	if (i2c->dev.of_node)
		ret = tasdevice_i2c_parse_dt(tas_dev);
	else {
		dev_err(tas_dev->dev, "No DTS info\n");
		goto out;
	}

	tas_dev->regmap = devm_regmap_init_i2c(i2c,
		&tasdevice_i2c_regmap);
	if (IS_ERR(tas_dev->regmap)) {
		ret = PTR_ERR(tas_dev->regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	if (tas_dev->glb_addr.dev_addr != 0
		&& tas_dev->glb_addr.dev_addr < 0x7F) {
		if (tas_dev->chip_id == TAS2781)
			tas_dev->set_global_mode = tas2781_set_global_mode;
		else
			tas_dev->set_global_mode = tas2563_set_global_mode;
	}

	ret = tasdevice_probe_next(tas_dev);

out:
	if (ret < 0 && tas_dev != NULL)
		tasdevice_remove(tas_dev);
	return ret;

}

static int tasdevice_i2c_remove(struct i2c_client *pClient)
{
	struct tasdevice_priv *tas_dev = i2c_get_clientdata(pClient);

	if (tas_dev)
		tasdevice_remove(tas_dev);

	return 0;
}

static struct i2c_driver tasdevice_i2c_driver = {
	.driver = {
		.name	= "tasdevice-codec",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tasdevice_of_match),
		.pm = &tasdevice_pm_ops,
	},
	.probe	= tasdevice_i2c_probe,
	.remove = tasdevice_i2c_remove,
	.id_table	= tasdevice_id,
};

module_i2c_driver(tasdevice_i2c_driver);

MODULE_AUTHOR("Kevin Lu <kevin-lu@ti.com>");
MODULE_DESCRIPTION("ASoC TASDEVICE Driver");
MODULE_LICENSE("GPL");
#endif
