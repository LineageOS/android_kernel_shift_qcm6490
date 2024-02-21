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
#include <linux/regmap.h>
#ifdef CONFIG_TASDEV_CODEC_SPI
	#include <linux/spi/spi.h>
#else
	#include <linux/i2c.h>
#endif

#include "tasdevice.h"
#include "tasdevice-rw.h"

static int tasdevice_regmap_write(
	struct tasdevice_priv *tas_priv,
	unsigned int reg, unsigned int value)
{
	int nResult = 0;
	int retry_count = TASDEVICE_RETRY_COUNT;

	while (retry_count--) {
		nResult = regmap_write(tas_priv->regmap, reg,
			value);
		if (nResult >= 0)
			break;
		usleep_range(5000, 5050);
	}
	if (retry_count == -1)
		return TASDEVICE_ERROR_FAILED;
	else
		return 0;
}

static int tasdevice_regmap_bulk_write(
	struct tasdevice_priv *tas_priv, unsigned int reg,
	unsigned char *pData, unsigned int nLength)
{
	int nResult = 0;
	int retry_count = TASDEVICE_RETRY_COUNT;

	while (retry_count--) {
		nResult = regmap_bulk_write(tas_priv->regmap, reg,
				pData, nLength);
		if (nResult >= 0)
			break;
		usleep_range(5000, 5050);
	}
	if (retry_count == -1)
		return TASDEVICE_ERROR_FAILED;
	else
		return 0;
}

static int tasdevice_regmap_read(
	struct tasdevice_priv *tas_priv,
	unsigned int reg, unsigned int *value)
{
	int nResult = 0;
	int retry_count = TASDEVICE_RETRY_COUNT;

	while (retry_count--) {
		nResult = regmap_read(tas_priv->regmap, reg,
			value);
		if (nResult >= 0)
			break;
		usleep_range(5000, 5050);
	}
	if (retry_count == -1)
		return TASDEVICE_ERROR_FAILED;
	else
		return 0;
}

static int tasdevice_regmap_bulk_read(
	struct tasdevice_priv *tas_priv, unsigned int reg,
	unsigned char *pData, unsigned int nLength)
{
	int nResult = 0;
	int retry_count = TASDEVICE_RETRY_COUNT;

	while (retry_count--) {
		nResult = regmap_bulk_read(tas_priv->regmap, reg,
			pData, nLength);
		if (nResult >= 0)
			break;
		usleep_range(5000, 5050);
	}
	if (retry_count == -1)
		return TASDEVICE_ERROR_FAILED;
	else
		return 0;
}

static int tasdevice_regmap_update_bits(
	struct tasdevice_priv *tas_priv, unsigned int reg,
	unsigned int mask, unsigned int value)
{
	int nResult = 0;
	int retry_count = TASDEVICE_RETRY_COUNT;

	while (retry_count--) {
		nResult = regmap_update_bits(tas_priv->regmap, reg,
			mask, value);
		if (nResult >= 0)
			break;
		usleep_range(5000, 5050);
	}
	if (retry_count == -1)
		return TASDEVICE_ERROR_FAILED;
	else
		return 0;
}

static int tasdevice_change_chn_book(
	struct tasdevice_priv *tas_priv, unsigned short chn, int book)
{
#ifdef CONFIG_TASDEV_CODEC_SPI
	struct spi_device *clnt =
		(struct spi_device *)tas_priv->client;
#else
	struct i2c_client *clnt =
		(struct i2c_client *)tas_priv->client;
#endif
	bool chip_switching = false;
	int ret = 0;

	if (chn < tas_priv->ndev) {
		struct tasdevice_t *tasdev = &tas_priv->tasdevice[chn];

#ifdef CONFIG_TASDEV_CODEC_SPI
		if (clnt->chip_select != tasdev->mnDevAddr) {
			clnt->chip_select = tasdev->mnDevAddr;
			chip_switching = true;
		}
#else
		if (tas_priv->glb_addr.ref_cnt != 0) {
			tas_priv->glb_addr.ref_cnt = 0;
			tas_priv->glb_addr.cur_book = -1;
		}

		if (clnt->addr != tasdev->mnDevAddr) {
			clnt->addr = tasdev->mnDevAddr;
			chip_switching = true;
		}
#endif
		if (tasdev->cur_book == book) {
			if (chip_switching) {
				/* All tas2781s share the same regmap, clear the page
				 * inside regmap once switching to another tas2781.
				 * Register 0 at any pages and any books inside tas2781
				 * is the same one for page-switching. Book has already
				 * inside the current tas2781.
				 */
				ret = tasdevice_regmap_write(tas_priv,
					TASDEVICE_PAGE_SELECT, 0);
				if (ret < 0)
					dev_err(tas_priv->dev, "%s, E=%d\n", __func__, ret);
			}
		} else {
			/* Book switching in other cases */
			ret = tasdevice_regmap_write(tas_priv,
				TASDEVICE_BOOKCTL_REG, book);
			if (ret < 0) {
				dev_err(tas_priv->dev, "%s, E=%d\n", __func__, ret);
				goto out;
			}
			tasdev->cur_book = book;
		}
	} else if (chn == tas_priv->ndev) {
		int i = 0;

		if (tas_priv->glb_addr.ref_cnt == 0)
			for (i = 0; i < tas_priv->ndev; i++)
				tas_priv->tasdevice[i].cur_book = -1;

		clnt->addr = tas_priv->glb_addr.dev_addr;
		if (tas_priv->glb_addr.cur_book != book) {
			ret = tasdevice_regmap_write(tas_priv,
				TASDEVICE_BOOKCTL_REG, book);
			if (ret < 0) {
				dev_err(tas_priv->dev,
					"%s, book%xERROR, E=%d\n",
					__func__, book, ret);
				goto out;
			}
			tas_priv->glb_addr.cur_book = book;
		}

		tas_priv->glb_addr.ref_cnt++;
	} else
		dev_err(tas_priv->dev, "%s, ERROR, no such channel(%d)\n",
			__func__, chn);

out:
	return ret;
}

int tasdevice_dev_read(struct tasdevice_priv *tas_priv,
	unsigned short chn, unsigned int reg, unsigned int *pValue)
{
	int ret = 0;

	mutex_lock(&tas_priv->dev_lock);
	if (chn < tas_priv->ndev) {
		ret = tasdevice_change_chn_book(tas_priv, chn,
			TASDEVICE_BOOK_ID(reg));
		if (ret < 0)
			goto out;

		ret = tasdevice_regmap_read(tas_priv,
			TASDEVICE_PGRG(reg), pValue);
		if (ret < 0)
			dev_err(tas_priv->dev, "%s, ERROR,E=%d\n",
				__func__, ret);
		else
			dev_dbg(tas_priv->dev,
				"%s: chn:0x%02x:BOOK:PAGE:REG 0x%02x:0x%02x:"
				"0x%02x, 0x%02x\n", __func__,
				tas_priv->tasdevice[chn].mnDevAddr,
				TASDEVICE_BOOK_ID(reg), TASDEVICE_PAGE_ID(reg),
				TASDEVICE_PAGE_REG(reg), *pValue);
	} else
		dev_err(tas_priv->dev, "%s, ERROR, no such channel(%d)\n",
			__func__, chn);

out:
	mutex_unlock(&tas_priv->dev_lock);
	return ret;
}


int tasdevice_dev_write(struct tasdevice_priv *tas_priv,
	unsigned short chn, unsigned int reg, unsigned int value)
{
	int ret = 0;

	mutex_lock(&tas_priv->dev_lock);
	if (chn <= tas_priv->ndev) {
		ret = tasdevice_change_chn_book(tas_priv, chn,
			TASDEVICE_BOOK_ID(reg));
		if (ret < 0)
			goto out;

		ret = tasdevice_regmap_write(tas_priv,
			TASDEVICE_PGRG(reg), value);
		if (ret < 0)
			dev_err(tas_priv->dev, "%s, ERROR, E=%d\n",
				__func__, ret);
		else
			dev_dbg(tas_priv->dev,
				"%s: %s-0x%02x:BOOK:PAGE:REG 0x%02x:0x%02x:0x%02x, VAL: 0x%02x\n",
				__func__, (chn == tas_priv->ndev)?"glb":"chn",
				(chn == tas_priv->ndev) ?
				tas_priv->glb_addr.dev_addr
				: tas_priv->tasdevice[chn].mnDevAddr,
				TASDEVICE_BOOK_ID(reg),
				TASDEVICE_PAGE_ID(reg),
				TASDEVICE_PAGE_REG(reg), value);
	} else
		dev_err(tas_priv->dev, "%s, ERROR, no such channel(%d)\n",
			__func__, chn);
out:
	mutex_unlock(&tas_priv->dev_lock);
	return ret;
}


int tasdevice_dev_bulk_write(
	struct tasdevice_priv *tas_priv, unsigned short chn,
	unsigned int reg, unsigned char *p_data,
	unsigned int n_length)
{
	int ret = 0;

	mutex_lock(&tas_priv->dev_lock);
	if (chn <= tas_priv->ndev) {
		ret = tasdevice_change_chn_book(tas_priv, chn,
			TASDEVICE_BOOK_ID(reg));
		if (ret < 0)
			goto out;

		ret = tasdevice_regmap_bulk_write(tas_priv,
			TASDEVICE_PGRG(reg), p_data, n_length);
		if (ret < 0)
			dev_err(tas_priv->dev, "%s, ERROR, E=%d\n",
				__func__, ret);
		else
			dev_dbg(tas_priv->dev,
				"%s: %s-0x%02x:BOOK:PAGE:REG 0x%02x:0x%02x: 0x%02x, len: 0x%02x\n",
				__func__,
				(chn == tas_priv->ndev)?"glb":"chn",
				(chn == tas_priv->ndev) ?
				tas_priv->glb_addr.dev_addr
				: tas_priv->tasdevice[chn].mnDevAddr,
				TASDEVICE_BOOK_ID(reg), TASDEVICE_PAGE_ID(reg),
				TASDEVICE_PAGE_REG(reg), n_length);
	} else
		dev_err(tas_priv->dev, "%s, ERROR, no such channel(%d)\n",
			__func__, chn);
out:
	mutex_unlock(&tas_priv->dev_lock);
	return ret;
}


int tasdevice_dev_bulk_read(struct tasdevice_priv *tas_priv,
	unsigned short chn, unsigned int reg, unsigned char *p_data,
	unsigned int n_length)
{
	int ret = 0;

	mutex_lock(&tas_priv->dev_lock);
	if (chn < tas_priv->ndev) {
		ret = tasdevice_change_chn_book(tas_priv, chn,
			TASDEVICE_BOOK_ID(reg));
		if (ret < 0)
			goto out;

		ret = tasdevice_regmap_bulk_read(tas_priv,
			TASDEVICE_PGRG(reg), p_data, n_length);
		if (ret < 0)
			dev_err(tas_priv->dev, "%s, ERROR, E=%d\n",
				__func__, ret);
		else
			dev_dbg(tas_priv->dev,
				"%s: chn0x%02x:BOOK:PAGE:REG 0x%02x:0x%02x:"
				"0x%02x, len: 0x%02x\n", __func__,
				tas_priv->tasdevice[chn].mnDevAddr,
				TASDEVICE_BOOK_ID(reg), TASDEVICE_PAGE_ID(reg),
				TASDEVICE_PAGE_REG(reg), n_length);
	} else
		dev_err(tas_priv->dev, "%s, ERROR, no such channel(%d)\n",
			__func__, chn);

out:
	mutex_unlock(&tas_priv->dev_lock);
	return ret;
}


int tasdevice_dev_update_bits(struct tasdevice_priv *tas_priv,
	unsigned short chn, unsigned int reg, unsigned int mask,
	unsigned int value)
{
	int ret = 0;

	mutex_lock(&tas_priv->dev_lock);
	if (chn < tas_priv->ndev) {
		ret = tasdevice_change_chn_book(tas_priv, chn,
			TASDEVICE_BOOK_ID(reg));
		if (ret < 0)
			goto out;

		ret = tasdevice_regmap_update_bits(tas_priv,
			TASDEVICE_PGRG(reg), mask, value);
		if (ret < 0)
			dev_err(tas_priv->dev, "%s, ERROR, E=%d\n",
				__func__, ret);
		else
			dev_dbg(tas_priv->dev,
				"%s: chn0x%02x:BOOK:PAGE:REG 0x%02x:0x%02x:0x%02x, "
				"mask: 0x%02x, val: 0x%02x\n",
				__func__, tas_priv->tasdevice[chn].mnDevAddr,
				TASDEVICE_BOOK_ID(reg), TASDEVICE_PAGE_ID(reg),
				TASDEVICE_PAGE_REG(reg), mask, value);
	} else
		dev_err(tas_priv->dev, "%s, ERROR, no such channel(%d)\n",
			__func__, chn);

out:
	mutex_unlock(&tas_priv->dev_lock);
	return ret;
}

