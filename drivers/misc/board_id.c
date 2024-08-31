#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/printk.h>
#include <linux/miscdevice.h>
#include <linux/sysfs.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/string.h>

#define BOARDID_DRIVER_NAME	"boardid"
#define BOARDID_MISC "boardid"

struct boardid_data {
	int BOARD_ID0;
	int BOARD_ID1;
	int BOARD_ID2;
	struct platform_device *pdev;
};
struct boardid_data *boardid_gpiodata = NULL;

static ssize_t version_show(struct device *dev,
		struct device_attribute *attr, char *buf) {
	return sprintf(buf, "%d%d%d\n", gpio_get_value(boardid_gpiodata->BOARD_ID0),
					gpio_get_value(boardid_gpiodata->BOARD_ID1),
					gpio_get_value(boardid_gpiodata->BOARD_ID2));
}

static DEVICE_ATTR(version_boardid, 0644, version_show, NULL);

static struct attribute *boardid_attributes[] = {
	&dev_attr_version_boardid.attr,
	NULL,
};

static struct attribute_group boardid_attr_group = {
	.attrs = boardid_attributes,
};

static struct of_device_id boardid_of_match[] = {
	{.compatible = BOARDID_DRIVER_NAME,},
	{},
};

static const struct file_operations boardid_misc_fops = {
	.owner = THIS_MODULE,
};

static struct miscdevice boardid_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = BOARDID_MISC,
};

static int  boardid_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device_node *node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;

	devm_pinctrl_get_select_default(dev);

	printk(KERN_ERR "[BOARDID] boardid_probe entry \n");

	boardid_gpiodata = devm_kzalloc(&pdev->dev, sizeof(struct boardid_data),
		GFP_KERNEL);

	if (boardid_gpiodata == NULL) {
		dev_err(&pdev->dev, "[BoardId] %s:%d Unable to allocate memory\n",__func__, __LINE__);
	}
	memset(boardid_gpiodata, 0, sizeof(struct boardid_data));

	boardid_gpiodata->pdev = pdev;
	/* get gpio port number */
	boardid_gpiodata->BOARD_ID0 = of_get_named_gpio(node, "boardid1", 0);
	boardid_gpiodata->BOARD_ID1 = of_get_named_gpio(node, "boardid2", 0);
	boardid_gpiodata->BOARD_ID2 = of_get_named_gpio(node, "boardid3", 0);

	if (gpio_is_valid(boardid_gpiodata->BOARD_ID0)) {
		ret = gpio_request(boardid_gpiodata->BOARD_ID0, "boardid1");
		if (ret) {
			dev_err(&boardid_gpiodata->pdev->dev,"[BoardId] %s: Failed to request gpio %d,ret = %d\n",
					__func__, boardid_gpiodata->BOARD_ID0, ret);
			goto error;
		}
	}

	if (gpio_is_valid(boardid_gpiodata->BOARD_ID1)) {
		ret = gpio_request(boardid_gpiodata->BOARD_ID1, "boardid2");
		if (ret) {
			dev_err(&boardid_gpiodata->pdev->dev,"[BoardId] %s: Failed to request gpio %d,ret = %d\n",
					__func__, boardid_gpiodata->BOARD_ID1, ret);
			goto error;
		}
	}

	if (gpio_is_valid(boardid_gpiodata->BOARD_ID2)) {
		ret = gpio_request(boardid_gpiodata->BOARD_ID2, "boardid3");
		if (ret) {
			dev_err(&boardid_gpiodata->pdev->dev,"[BoardId] %s: Failed to request gpio %d,ret = %d\n",
					__func__, boardid_gpiodata->BOARD_ID2, ret);
			goto error;
		}
	}

	/* register misc */
	ret = misc_register(&boardid_misc_device);
	if (ret < 0) {
		dev_err(&pdev->dev, "BoardId register failed\n");
		goto error;
	}

	ret =  sysfs_create_group(&boardid_misc_device.this_device->kobj, &boardid_attr_group);
	if(ret < 0) {
		printk("IN %s : %d : Fail to Create sys attribute.\n",__func__,__LINE__);
	}
	printk(" [BoardId] boardid_probe_end\n");
	return 0;

error:
	return ret;
}

static int boardid_remove(struct platform_device *pdev){
	return 0;
}

static struct platform_driver boardid_driver = {
	.probe = boardid_probe,
	.remove = boardid_remove,
	.driver = {
		.name = BOARDID_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = boardid_of_match,
	}
};

static int __init boardid_init(void) {
	return platform_driver_register(&boardid_driver);
}

static void __exit boardid_exit(void) {
	return platform_driver_unregister(&boardid_driver);
}

late_initcall(boardid_init);
module_exit(boardid_exit);

MODULE_DESCRIPTION("BoardId driver");

