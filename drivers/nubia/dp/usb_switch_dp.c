/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/soc/zte/usb_switch_dp.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>


struct dp_switch_priv {
	struct platform_device *pdev;
	struct device *dev;
	unsigned int switch_en;
	unsigned int switch_en_flag;
	unsigned int switch_mode;
	unsigned int switch_mode_flag;
	unsigned int usb_mode;
	bool is_enabled;
}*pswitcher = NULL;


/*
* switcher_function_switch_event - configure FSA switch position based on event
*
* @node - phandle node to fsa4480 device
* @event - fsa_function enum
*
* Returns int on whether the switch happened or not
*/
int switcher_switch_event(struct device_node *node,
		   enum switcher_function event)
{
	if (!pswitcher)
	  return -EINVAL;
	pswitcher->usb_mode = event;
	switch (event) {
	case SWITCHER_MIC_GND_SWAP:
		break;
	case SWITCHER_USBC_ORIENTATION_CC1:		
	case SWITCHER_USBC_ORIENTATION_CC2:
		nubia_usb_switch_dp_enable(1);
		pswitcher->is_enabled = 1;
		break;
	case SWITCHER_USBC_DISPLAYPORT_DISCONNECTED:
		nubia_usb_switch_dp_enable(0);
		pswitcher->is_enabled = 0;
		break;	
	default:
	  break;
	}
	printk("dp_enable_event = %c",pswitcher->is_enabled);
  return 0;
}
EXPORT_SYMBOL(switcher_switch_event);

static int nubia_parse_switcher_dt(struct device_node *node)
{
	int rc = 0;
	u32 value;

	pswitcher->switch_en = of_get_named_gpio_flags(node, "qcom,switch-en-gpio", 0, NULL);
	if (!gpio_is_valid(pswitcher->switch_en)){
		dev_err(pswitcher->dev, "TLMM switch enable gpio not found\n");
		return -EPROBE_DEFER;
	}
	
	rc = gpio_request(pswitcher->switch_en, "USB_SWITCH_DP_EN");
	if (rc < 0){
		dev_err(pswitcher->dev, "parse pswitcher->switch_en error\n");
		rc = -ENODEV;
	}
	rc = of_property_read_u32(node, "qcom,switch-en-flag", &value);
	if (rc < 0){
		dev_err(pswitcher->dev, "parse switch-en-flag error\n");
			return rc;
	}else{
			pswitcher->switch_en_flag = value;
	}
	
	rc = gpio_direction_output(pswitcher->switch_en, pswitcher->switch_en_flag);
	if (rc < 0) {
	    dev_err(pswitcher->dev, "parse switch_en_direction_output error\n");
	    return rc;
	}

	pswitcher->switch_mode = of_get_named_gpio(node, "qcom,switch-mode-gpio", 0);
	if (!gpio_is_valid(pswitcher->switch_mode)){
		dev_err(pswitcher->dev, "TLMM switch mode gpio not found\n");
		return -EPROBE_DEFER;
	}

	rc = gpio_request(pswitcher->switch_mode, "USB_SWITCH_DP_MODE");
	if (rc < 0){
		printk("Failed to request GPIO:%d, ERRNO:%d", (s32)pswitcher->switch_en, rc);
		rc = -ENODEV;
	}

	rc = of_property_read_u32(node, "qcom,switch-mode-flag", &value);
	if (rc < 0){
		dev_err(pswitcher->dev, "parse switch-en-flag error\n");
			return rc;
	}else{
			pswitcher->switch_mode_flag = value;
	}

	rc = gpio_direction_output(pswitcher->switch_mode, pswitcher->switch_mode_flag);
	if (rc) {
		pr_err("%s: Failed to set gpio %d to %d\n", pswitcher->switch_mode, pswitcher->switch_mode_flag);
		return rc;
	}

	printk("%s:pswitcher->switch_en = %d,pswitcher->switch_mode = %d \n", __func__, pswitcher->switch_en,pswitcher->switch_mode);	
	return rc;
}


void nubia_usb_switch_dp_enable(int en)
{	
	if(en){
		gpio_set_value(pswitcher->switch_en, 0);
		if(pswitcher->usb_mode == SWITCHER_USBC_ORIENTATION_CC1)
			gpio_set_value(pswitcher->switch_mode, 0);
		else
			gpio_set_value(pswitcher->switch_mode, 1);
	}else
	{
		gpio_set_value(pswitcher->switch_en, 1);
	}
		
}

static ssize_t dp_cc_statue_show(struct device *dev, struct device_attribute *attr,
                char *buf)
                {
                    switch (pswitcher->usb_mode) {
                    case SWITCHER_USBC_ORIENTATION_CC1:
                        return sprintf(buf, "%d\n", 1);
                        break;
                    case SWITCHER_USBC_ORIENTATION_CC2:
                        return sprintf(buf, "%d\n", 2);
                        break;
                    }
                    return sprintf(buf, "%d\n", 0);
                }

static ssize_t dp_cc_statue_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
                {
                    int val;
                    if (kstrtoint(buf, 0, &val))
                        return -EINVAL;
                    printk("please not set cc_statue!");
                    return count;
                }

static ssize_t dp_enable_show(struct device *dev, struct device_attribute *attr,
                char *buf)
{
        return sprintf(buf, "%d\n", pswitcher->is_enabled);
}

static ssize_t dp_enable_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
        int val;

        if (kstrtoint(buf, 0, &val))
                return -EINVAL;

        switch (val) {
        case 0:
        case 1:
		nubia_usb_switch_dp_enable(0);
                break;
        default:
                dev_err(dev, "Invalid argument\n");
                return -EINVAL;
        }
        return count;
}

static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO, dp_enable_show, dp_enable_store);
static DEVICE_ATTR(cc_statue, S_IWUSR | S_IRUGO, dp_cc_statue_show, dp_cc_statue_store);

static struct attribute *usb_switch_dp_attributes[] = {
        &dev_attr_enable.attr,
        &dev_attr_cc_statue.attr,
        NULL
};

static struct attribute_group usb_switch_dp_attribute_group = {
        .attrs = usb_switch_dp_attributes
};

static int  nubia_usb_switch_dp_probe(struct platform_device *pdev)
{
	int rc = 0;
	int ret;
	struct device_node *node;

	if (!pdev){
		printk(KERN_ERR"pdev is null\n");
		return -EPROBE_DEFER;
	}

	node = pdev->dev.of_node;
	pswitcher = kzalloc(sizeof(struct dp_switch_priv ), GFP_KERNEL);
	if (!pswitcher) {
		dev_err(&pdev->dev, "cannot allocate device memory.\n");
		return -ENOMEM;
	}
	pswitcher->dev = &pdev->dev;
	pswitcher->pdev = pdev;		
	pswitcher->is_enabled = 0;
	rc = nubia_parse_switcher_dt(node);
	if (rc < 0)
		return rc;
	ret = sysfs_create_group(&pswitcher->dev->kobj, &usb_switch_dp_attribute_group);
	if (ret < 0) {
        	dev_err(&pdev->dev, "%s error creating sysfs attr files\n",__func__);
	}	
	return 0;

}

static int nubia_usb_switch_dp_remove(struct platform_device *pdev)
{
        if (!pswitcher)
                return -EINVAL;
        sysfs_remove_group(&pswitcher->dev->kobj, &usb_switch_dp_attribute_group);
        return 0;
}

static const struct of_device_id of_match[] = {
        { .compatible = "nubia,usb_switch_dp" },
        { }
};

static struct platform_driver usb_switch_dp_driver = {
        .probe = nubia_usb_switch_dp_probe,
        .remove = nubia_usb_switch_dp_remove,
        .driver = {
                .name = "nubia,usb_switch_dp",
                .owner = THIS_MODULE,
                .of_match_table = of_match_ptr(of_match),
        },
};

int __init nubia_usb_switch_dp_init(void)
{	
	platform_driver_register(&usb_switch_dp_driver);
	return 0;
}

static void __exit nubia_usb_switch_dp_exit(void)
{
	platform_driver_unregister(&usb_switch_dp_driver);
}

module_init(nubia_usb_switch_dp_init);
module_exit(nubia_usb_switch_dp_exit);

MODULE_DESCRIPTION("nubia_hw_version driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:nubia_hw_version" );

