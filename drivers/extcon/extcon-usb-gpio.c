/**
 * drivers/extcon/extcon-usb-gpio.c - USB GPIO extcon driver
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com
 * Author: Roger Quadros <rogerq@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/extcon-provider.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/pinctrl/consumer.h>

#define USB_GPIO_DEBOUNCE_MS	20	/* ms */

struct usb_extcon_info {
	struct device *dev;
	struct extcon_dev *edev;

	struct gpio_desc *id_gpiod;
	struct gpio_desc *vbus_gpiod;
	struct gpio_desc *vbus_out_gpiod;
	int id_irq;
	int vbus_irq;

	unsigned long debounce_jiffies;
	struct delayed_work wq_detcable;
#ifdef CONFIG_NUBIA_USB2_DOCK_SWITCH
	bool is_enabled;
#endif
};

static const unsigned int usb_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

/*
 * "USB" = VBUS and "USB-HOST" = !ID, so we have:
 * Both "USB" and "USB-HOST" can't be set as active at the
 * same time so if "USB-HOST" is active (i.e. ID is 0)  we keep "USB" inactive
 * even if VBUS is on.
 *
 *  State              |    ID   |   VBUS
 * ----------------------------------------
 *  [1] USB            |    H    |    H
 *  [2] none           |    H    |    L
 *  [3] USB-HOST       |    L    |    H
 *  [4] USB-HOST       |    L    |    L
 *
 * In case we have only one of these signals:
 * - VBUS only - we want to distinguish between [1] and [2], so ID is always 1.
 * - ID only - we want to distinguish between [1] and [4], so VBUS = ID.
*/
static void usb_extcon_detect_cable(struct work_struct *work)
{
	int id;
	struct usb_extcon_info *info = container_of(to_delayed_work(work),
						    struct usb_extcon_info,
						    wq_detcable);

	/* check ID and VBUS and update cable state */
	id = info->id_gpiod ?
		gpiod_get_value_cansleep(info->id_gpiod) : 1;
	/* at first we clean states which are no longer active */
	if (id) {
		if (info->vbus_out_gpiod)
			gpiod_set_value_cansleep(info->vbus_out_gpiod, 0);
		extcon_set_state_sync(info->edev, EXTCON_USB_HOST, false);
	}

	if (!id) {
		if (info->vbus_out_gpiod)
			gpiod_set_value_cansleep(info->vbus_out_gpiod, 1);
		extcon_set_state_sync(info->edev, EXTCON_USB_HOST, true);
	}
}

static irqreturn_t usb_irq_handler(int irq, void *dev_id)
{
	struct usb_extcon_info *info = dev_id;

	queue_delayed_work(system_power_efficient_wq, &info->wq_detcable,
			   info->debounce_jiffies);

	return IRQ_HANDLED;
}

#ifdef CONFIG_NUBIA_USB2_DOCK_SWITCH
static int usb_extcon_set_enable(struct usb_extcon_info *info, bool enable)
{
	struct device *dev = info->dev;
	int ret;

	if (enable == info->is_enabled) {
		dev_err(dev, "already %s\n", enable? "enabled": "disabled");
		return 0;
	}

	if (enable) {
		ret = pinctrl_pm_select_default_state(dev);
		if (ret < 0)
			return ret;
		if (info->id_gpiod)
			enable_irq(info->id_irq);
		queue_delayed_work(system_power_efficient_wq,
				   &info->wq_detcable, 0);
		info->is_enabled = true;
		dev_err(dev, "enable usb extcon\n");
	} else {
		extcon_set_state_sync(info->edev, EXTCON_USB_HOST, false);
		extcon_set_state_sync(info->edev, EXTCON_USB, false);
		if (info->id_gpiod)
			disable_irq(info->id_irq);
		ret = pinctrl_pm_select_sleep_state(dev);
		if (ret < 0) {
			if (info->id_gpiod)
				enable_irq(info->id_irq);
			return ret;
		}
		info->is_enabled = false;
		dev_err(dev, "disable usb extcon\n");
	}
	return ret;
}

static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct usb_extcon_info *info = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", info->is_enabled);
}

static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct usb_extcon_info *info = dev_get_drvdata(dev);
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	switch (val) {
	case 0:
		usb_extcon_set_enable(info, false);
		break;
	case 1:
		usb_extcon_set_enable(info, true);
		break;
	default:
		dev_err(dev, "Invalid argument\n");
		return -EINVAL;
	}
	return count;
}
static DEVICE_ATTR_RW(enable);
#endif

static int usb_extcon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct usb_extcon_info *info;
	int ret;

	if (!np)
		return -EINVAL;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = dev;
	info->id_gpiod = devm_gpiod_get_optional(&pdev->dev, "id", GPIOD_IN);
	info->vbus_out_gpiod = devm_gpiod_get_optional(&pdev->dev, "vbus-out",
						   GPIOD_OUT_LOW);

	if (!info->id_gpiod) {
		dev_err(dev, "failed to get gpios\n");
		return -ENODEV;
	}

	if (IS_ERR(info->id_gpiod))
		return PTR_ERR(info->id_gpiod);
		
	if (IS_ERR(info->vbus_out_gpiod))
		return PTR_ERR(info->vbus_out_gpiod);

	info->edev = devm_extcon_dev_allocate(dev, usb_extcon_cable);
	if (IS_ERR(info->edev)) {
		dev_err(dev, "failed to allocate extcon device\n");
		return -ENOMEM;
	}

	ret = devm_extcon_dev_register(dev, info->edev);
	if (ret < 0) {
		dev_err(dev, "failed to register extcon device\n");
		return ret;
	}

#ifdef CONFIG_NUBIA_USB2_DOCK_SWITCH
	device_create_file(&pdev->dev, &dev_attr_enable);
#endif

	if (info->id_gpiod)
		ret = gpiod_set_debounce(info->id_gpiod,
					 USB_GPIO_DEBOUNCE_MS * 1000);

	if (ret < 0)
		info->debounce_jiffies = msecs_to_jiffies(USB_GPIO_DEBOUNCE_MS);

	INIT_DELAYED_WORK(&info->wq_detcable, usb_extcon_detect_cable);

	if (info->id_gpiod) {
		info->id_irq = gpiod_to_irq(info->id_gpiod);
		if (info->id_irq < 0) {
			dev_err(dev, "failed to get ID IRQ\n");
			return info->id_irq;
		}

		ret = devm_request_threaded_irq(dev, info->id_irq, NULL,
						usb_irq_handler,
						IRQF_TRIGGER_RISING |
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						pdev->name, info);
		if (ret < 0) {
			dev_err(dev, "failed to request handler for ID IRQ\n");
			return ret;
		}
	}

	platform_set_drvdata(pdev, info);
#ifdef CONFIG_NUBIA_USB2_DOCK_SWITCH
	device_init_wakeup(&pdev->dev, false);
	info->is_enabled = true;
    msleep(15);
	usb_extcon_set_enable(info, false);
#else
	device_init_wakeup(&pdev->dev, true);

	/* Perform initial detection */
	usb_extcon_detect_cable(&info->wq_detcable.work);
#endif

	return 0;
}

static int usb_extcon_remove(struct platform_device *pdev)
{
	struct usb_extcon_info *info = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&info->wq_detcable);
	device_init_wakeup(&pdev->dev, false);

#ifdef CONFIG_NUBIA_USB2_DOCK_SWITCH
	device_remove_file(&pdev->dev, &dev_attr_enable);
#endif

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int usb_extcon_suspend(struct device *dev)
{
	struct usb_extcon_info *info = dev_get_drvdata(dev);
	int ret = 0;

#ifdef CONFIG_NUBIA_USB2_DOCK_SWITCH
	if (!info->is_enabled) {
		return 0;
	}
#endif

	if (device_may_wakeup(dev)) {
		if (info->id_gpiod) {
			ret = enable_irq_wake(info->id_irq);
			if (ret)
				return ret;
		}
	}

	/*
	 * We don't want to process any IRQs after this point
	 * as GPIOs used behind I2C subsystem might not be
	 * accessible until resume completes. So disable IRQ.
	 */
	if (info->id_gpiod)
		disable_irq(info->id_irq);
		
	if (!device_may_wakeup(dev))
		pinctrl_pm_select_sleep_state(dev);

	return ret;
}

static int usb_extcon_resume(struct device *dev)
{
	struct usb_extcon_info *info = dev_get_drvdata(dev);
	int ret = 0;

#ifdef CONFIG_NUBIA_USB2_DOCK_SWITCH
	if (!info->is_enabled) {
		return 0;
	}
#endif

	if (!device_may_wakeup(dev))
		pinctrl_pm_select_default_state(dev);

	if (device_may_wakeup(dev)) {
		if (info->id_gpiod) {
			ret = disable_irq_wake(info->id_irq);
			if (ret)
				return ret;
		}
	}

	if (info->id_gpiod)
		enable_irq(info->id_irq);

	queue_delayed_work(system_power_efficient_wq,
			   &info->wq_detcable, 0);

	return ret;
}
#endif

static SIMPLE_DEV_PM_OPS(usb_extcon_pm_ops,
			 usb_extcon_suspend, usb_extcon_resume);

static const struct of_device_id usb_extcon_dt_match[] = {
	{ .compatible = "linux,extcon-usb-gpio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, usb_extcon_dt_match);

static const struct platform_device_id usb_extcon_platform_ids[] = {
	{ .name = "extcon-usb-gpio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, usb_extcon_platform_ids);

static struct platform_driver usb_extcon_driver = {
	.probe		= usb_extcon_probe,
	.remove		= usb_extcon_remove,
	.driver		= {
		.name	= "extcon-usb-gpio",
		.pm	= &usb_extcon_pm_ops,
		.of_match_table = usb_extcon_dt_match,
	},
	.id_table = usb_extcon_platform_ids,
};

module_platform_driver(usb_extcon_driver);

MODULE_AUTHOR("Roger Quadros <rogerq@ti.com>");
MODULE_DESCRIPTION("USB GPIO extcon driver");
MODULE_LICENSE("GPL v2");
