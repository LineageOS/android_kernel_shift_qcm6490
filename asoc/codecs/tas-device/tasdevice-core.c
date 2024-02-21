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

#ifdef CONFIG_COMPAT
	#include <linux/compat.h>
#endif
#include <linux/crc8.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>
#ifdef CONFIG_TASDEV_CODEC_SPI
	#include <linux/spi/spi.h>
#else
	#include <linux/i2c.h>
#endif
#include <linux/uaccess.h>

#include "tasdevice-regbin.h"
#include "tasdevice.h"
#include "tasdevice-rw.h"
#include "tasdevice-ctl.h"
#include "tasdevice-node.h"
#include "tasdevice-codec.h"
#include "tasdevice-dsp.h"
#include "tasdevice-misc.h"

#define TASDEVICE_IRQ_DET_TIMEOUT		(30000)
#define TASDEVICE_IRQ_DET_CNT_LIMIT	(500)

#ifdef CONFIG_TASDEV_CODEC_SPI
const struct spi_device_id tasdevice_id[] = {
	{ "tas2563", TAS2563 },
	{ "tas2781", TAS2781 },
	{}
};
MODULE_DEVICE_TABLE(spi, tasdevice_id);

#else
const struct i2c_device_id tasdevice_id[] = {
	{ "tas2563", TAS2563 },
	{ "tas2781", TAS2781 },
	{}
};

MODULE_DEVICE_TABLE(i2c, tasdevice_id);
#endif

static ssize_t devinfo_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tasdevice_priv *tas_dev = dev_get_drvdata(dev);
	int n = 0, i = 0;

	if (tas_dev != NULL) {
		n  += scnprintf(buf + n, 32, "No.\tDevTyp\tAddr\n");
		for (i = 0; i < tas_dev->ndev; i++) {
			n += scnprintf(buf + n, 16, "%d\t", i);
			n += scnprintf(buf + n, 32, "%s\t",
				tasdevice_id[tas_dev->chip_id].name);
			n += scnprintf(buf + n, 16, "0x%02x\n",
				tas_dev->tasdevice[i].mnDevAddr);
		}
	} else
		n += scnprintf(buf + n, 16, "Invalid data\n");

	return n;
}

const struct of_device_id tasdevice_of_match[] = {
	{ .compatible = "ti,tas2563" },
	{ .compatible = "ti,tas2781" },
	{},
};

MODULE_DEVICE_TABLE(of, tasdevice_of_match);

static DEVICE_ATTR(reg, 0664, reg_show, reg_store);
static DEVICE_ATTR(regdump, 0664, regdump_show, regdump_store);
static DEVICE_ATTR(act_addr, 0664, active_address_show, NULL);
static DEVICE_ATTR(dspfwinfo_list, 0664, dspfwinfo_list_show,
	NULL);
static DEVICE_ATTR(regbininfo_list, 0664, regbininfo_list_show,
	NULL);
static DEVICE_ATTR(regcfg_list, 0664, regcfg_list_show,
	regcfg_list_store);
static DEVICE_ATTR(devinfo, 0664, devinfo_show, NULL);
static DEVICE_ATTR(dspfw_config, 0664, dspfw_config_show, NULL);
static DEVICE_ATTR(force_fw_load_chip, 0664, force_fw_load_chip_show,
	force_fw_load_chip_store);

static struct attribute *sysfs_attrs[] = {
	&dev_attr_reg.attr,
	&dev_attr_regdump.attr,
	&dev_attr_act_addr.attr,
	&dev_attr_dspfwinfo_list.attr,
	&dev_attr_regbininfo_list.attr,
	&dev_attr_regcfg_list.attr,
	&dev_attr_devinfo.attr,
	&dev_attr_dspfw_config.attr,
	&dev_attr_force_fw_load_chip.attr,
	NULL
};
//nodes are in /sys/devices/platform/XXXXXXXX.i2cX/i2c-X/
// Or /sys/bus/i2c/devices/7-004c/
const struct attribute_group tasdevice_attribute_group = {
	.attrs = sysfs_attrs
};

//IRQ
void tasdevice_enable_irq(struct tasdevice_priv *tas_dev,
	bool enable)
{
	if (enable != tas_dev->irq_info.irq_enable &&
		!gpio_is_valid(tas_dev->irq_info.irq_gpio))
		return;

	if (enable)
		enable_irq(tas_dev->irq_info.irq);
	else
		disable_irq_nosync(tas_dev->irq_info.irq);

	tas_dev->irq_info.irq_enable = enable;
}

static void irq_work_routine(struct work_struct *pWork)
{
	struct tasdevice_priv *tas_dev =
		container_of(pWork, struct tasdevice_priv,
		irq_info.irq_work.work);

	dev_info(tas_dev->dev, "%s enter\n", __func__);

	mutex_lock(&tas_dev->codec_lock);
	if (tas_dev->mb_runtime_suspend) {
		dev_info(tas_dev->dev, "%s, Runtime Suspended\n", __func__);
		goto end;
	}
	/*Logical Layer IRQ function, return is ignored*/
	if (tas_dev->irq_work_func)
		tas_dev->irq_work_func(tas_dev);
	else
		dev_info(tas_dev->dev,
			"%s, irq_work_func is NULL\n", __func__);
end:
	mutex_unlock(&tas_dev->codec_lock);
	dev_info(tas_dev->dev, "%s leave\n", __func__);
}

static irqreturn_t tasdevice_irq_handler(int irq,
	void *dev_id)
{
	struct tasdevice_priv *tas_dev = (struct tasdevice_priv *)dev_id;
;
	/* get IRQ status after 100 ms */
	schedule_delayed_work(&tas_dev->irq_info.irq_work,
		msecs_to_jiffies(100));
	return IRQ_HANDLED;
}

void tasdevice_parse_dt_reset_irq_pin(
	struct tasdevice_priv *tas_priv, struct device_node *np)
{
	int rc = 0;

	tas_priv->reset = devm_gpiod_get_optional(tas_priv->dev,
			"reset-gpios", GPIOD_OUT_HIGH);
	if (IS_ERR(tas_priv->reset))
		dev_err(tas_priv->dev, "%s Can't get reset GPIO\n", __func__);

	tas_priv->irq_info.irq_gpio = of_irq_get(np, 0);
	if (gpio_is_valid(tas_priv->irq_info.irq_gpio)) {
		dev_info(tas_priv->dev, "irq-gpio = %d",
			tas_priv->irq_info.irq_gpio);
		INIT_DELAYED_WORK(&tas_priv->irq_info.irq_work,
			irq_work_routine);
		tas_priv->irq_info.irq_enable = false;

		rc = gpio_request(tas_priv->irq_info.irq_gpio, "AUDEV-IRQ");
		if (!rc) {
			gpio_direction_input(tas_priv->irq_info.irq_gpio);

			tas_priv->irq_info.irq =
				gpio_to_irq(tas_priv->irq_info.irq_gpio);
			dev_info(tas_priv->dev, "irq = %d\n",
				tas_priv->irq_info.irq);

			rc = request_threaded_irq(tas_priv->irq_info.irq,
				tasdevice_irq_handler, NULL,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				tas_priv->dev_name, tas_priv);
			if (!rc)
				disable_irq_nosync(tas_priv->irq_info.irq);
			else
				dev_err(tas_priv->dev,
					"request_irq failed, %d\n", rc);
		} else
			dev_err(tas_priv->dev,
				"%s: GPIO %d request error\n",
				__func__,
				tas_priv->irq_info.irq_gpio);
	} else
		dev_err(tas_priv->dev, "Looking up irq gpio property "
			"in node %s failed %d, no side effect on driver running\n",
			np->full_name, tas_priv->irq_info.irq_gpio);

	if (gpio_is_valid(tas_priv->irq_info.irq_gpio)) {
		switch (tas_priv->chip_id) {
		case TAS2563:
			tas_priv->irq_work_func = tas2563_irq_work_func;
			break;
		case TAS2781:
			tas_priv->irq_work_func = tas2781_irq_work_func;
			break;
		default:
			dev_info(tas_priv->dev, "%s: No match irq_work_func\n",
				__func__);
		}
	}
}

static void tasdevice_reset(struct tasdevice_priv *tas_dev)
{
	int ret, i;

	if (tas_dev->reset) {
		gpiod_set_value_cansleep(tas_dev->reset, 0);
		usleep_range(500, 1000);
		gpiod_set_value_cansleep(tas_dev->reset, 1);
	} else {
		for (i = 0; i < tas_dev->ndev; i++) {
			ret = tasdevice_dev_write(tas_dev, i,
				TASDEVICE_REG_SWRESET,
				TASDEVICE_REG_SWRESET_RESET);
			if (ret < 0)
				dev_err(tas_dev->dev, "chn %d rst fail, %d\n",
					i, ret);
		}
	}
}

int tasdevice_probe_next(struct tasdevice_priv *tas_dev)
{
	int nResult, i;

	for (i = 0; i < tas_dev->ndev; i++) {
		tas_dev->tasdevice[i].cur_book = -1;
		tas_dev->tasdevice[i].mnCurrentProgram = -1;
		tas_dev->tasdevice[i].mnCurrentConfiguration = -1;
	}
	mutex_init(&tas_dev->dev_lock);
	mutex_init(&tas_dev->file_lock);
	tas_dev->hwreset = tasdevice_reset;
	tas_dev->read = tasdevice_dev_read;
	tas_dev->write = tasdevice_dev_write;
	tas_dev->bulk_read = tasdevice_dev_bulk_read;
	tas_dev->bulk_write = tasdevice_dev_bulk_write;
	tas_dev->update_bits = tasdevice_dev_update_bits;
	tas_dev->set_calibration = tas2781_set_calibration;

	nResult = tasdevice_misc_register(tas_dev);
	if (nResult < 0) {
		dev_err(tas_dev->dev, "misc dev registration failed\n");
		goto out;
	}

	dev_set_drvdata(tas_dev->dev, tas_dev);
	nResult = sysfs_create_group(&tas_dev->dev->kobj,
				&tasdevice_attribute_group);
	if (nResult < 0) {
		dev_err(tas_dev->dev, "Sysfs registration failed\n");
		goto out;
	}

	INIT_DELAYED_WORK(&tas_dev->powercontrol_work,
		powercontrol_routine);

	mutex_init(&tas_dev->codec_lock);
	nResult = tasdevice_register_codec(tas_dev);
	if (nResult)
		dev_err(tas_dev->dev, "%s: codec register error:0x%08x\n",
			__func__, nResult);

	dev_info(tas_dev->dev, "i2c register success\n");
out:
	return nResult;
}

void tasdevice_remove(struct tasdevice_priv *tas_dev)
{
	if (delayed_work_pending(&tas_dev->irq_info.irq_work)) {
		dev_info(tas_dev->dev, "cancel IRQ work\n");
		cancel_delayed_work(&tas_dev->irq_info.irq_work);
	}
	cancel_delayed_work_sync(&tas_dev->irq_info.irq_work);

	mutex_destroy(&tas_dev->dev_lock);
	mutex_destroy(&tas_dev->file_lock);
	mutex_destroy(&tas_dev->codec_lock);
	misc_deregister(&tas_dev->misc_dev);
	tasdevice_deregister_codec(tas_dev);
	sysfs_remove_group(&tas_dev->dev->kobj, &tasdevice_attribute_group);
}

static int tasdevice_pm_suspend(struct device *dev)
{
	struct tasdevice_priv *tas_dev = dev_get_drvdata(dev);

	if (!tas_dev) {
		dev_err(tas_dev->dev, "%s: drvdata is NULL\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&tas_dev->codec_lock);

	tas_dev->mb_runtime_suspend = true;

	if (gpio_is_valid(tas_dev->irq_info.irq_gpio)) {
		if (delayed_work_pending(&tas_dev->irq_info.irq_work)) {
			dev_dbg(tas_dev->dev, "cancel IRQ work\n");
			cancel_delayed_work_sync(&tas_dev->irq_info.irq_work);
		}
	}
	mutex_unlock(&tas_dev->codec_lock);
	return 0;
}

static int tasdevice_pm_resume(struct device *dev)
{
	struct tasdevice_priv *tas_dev = dev_get_drvdata(dev);

	if (!tas_dev) {
		dev_err(tas_dev->dev, "%s: drvdata is NULL\n",
			__func__);
		return -EINVAL;
	}

	mutex_lock(&tas_dev->codec_lock);
	tas_dev->mb_runtime_suspend = false;
	mutex_unlock(&tas_dev->codec_lock);
	return 0;
}

const struct dev_pm_ops tasdevice_pm_ops = {
	.suspend = tasdevice_pm_suspend,
	.resume = tasdevice_pm_resume
};
