/*
 * nubia_dp_preference.c - nubia usb display enhancement and temperature setting
 *	      Linux kernel modules for mdss
 *
 * Copyright (c) 2015 nubia <nubia@nubia.com.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 * Supports NUBIA usb display enhancement and color temperature setting
 */

/*------------------------------ header file --------------------------------*/
#include "nubia_usb_test.h"
#include <linux/delay.h>


static struct usb_device *nubia_usb_device=NULL;
static struct usb_gadget *nubia_usb_gadget=NULL;
/*------------------------------- variables ---------------------------------*/
static struct kobject *enhance_kobj = NULL;

static ssize_t otg30_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	if(nubia_usb_device == NULL)
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	if (nubia_usb_device->speed >= USB_SPEED_SUPER)
		return snprintf(buf, PAGE_SIZE, "%d\n", 3);
	else if (nubia_usb_device->speed == USB_SPEED_HIGH)
		return snprintf(buf, PAGE_SIZE, "%d\n", 2);
	else
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);

}

static ssize_t otg30_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t size)
{
	//NUBIA_DISP_INFO("please don't try to set the otg30\n");

	if(nubia_usb_device == NULL)
		return size;

	return size;
}

static ssize_t usb30_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{		
	if(nubia_usb_gadget == NULL)
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	if (nubia_usb_gadget->speed >= USB_SPEED_SUPER)
		return snprintf(buf, PAGE_SIZE, "%d\n", 3);
	else if (nubia_usb_gadget->speed == USB_SPEED_HIGH)
		return snprintf(buf, PAGE_SIZE, "%d\n", 2);
	else
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);

}

static ssize_t usb30_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t size)
{
	//NUBIA_DISP_INFO("please don't try to set the usb30\n");

	if(nubia_usb_gadget == NULL)
		return size;

	return size;
}

static struct kobj_attribute usb_test_attrs[] = {
	__ATTR(otg30,        0664, otg30_show,       otg30_store),
	__ATTR(usb30,        0664, usb30_show,       usb30_store),
};

void nubia_set_usbdev_ctrl(struct usb_device *device)
{
	//NUBIA_DISP_INFO("start\n");

	nubia_usb_device = device;
}

void nubia_set_usbhost_ctrl(struct usb_gadget *gadget)
{
	//NUBIA_DISP_INFO("start\n");

	nubia_usb_gadget = gadget;
}

static int __init nubia_usb_test_init(void)
{
	int retval = 0;
	int attr_count = 0;

	//NUBIA_DISP_INFO("start\n");

	enhance_kobj = kobject_create_and_add("usb_enhance", kernel_kobj);

	if (!enhance_kobj) {
		//NUBIA_DISP_ERROR("failed to create and add kobject\n");
		return -ENOMEM;
	}

	/* Create attribute files associated with this kobject */
	for (attr_count = 0; attr_count < ARRAY_SIZE(usb_test_attrs); attr_count++) {
		retval = sysfs_create_file(enhance_kobj, &usb_test_attrs[attr_count].attr);
		if (retval < 0) {
			//NUBIA_DISP_ERROR("failed to create sysfs attributes\n");
			goto err_sys_creat;
		}
	}
	//NUBIA_DISP_INFO("success\n");

	return retval;

err_sys_creat:
//#ifdef CONFIG_NUBIA_SWITCH_LCD
	//cancel_delayed_work_sync(&nubia_disp_val.lcd_states_work);
//#endif
	for (--attr_count; attr_count >= 0; attr_count--)
		sysfs_remove_file(enhance_kobj, &usb_test_attrs[attr_count].attr);

	kobject_put(enhance_kobj);
	return retval;
}

static void __exit nubia_usb_test_exit(void)
{
	int attr_count = 0;

	for (attr_count = 0; attr_count < ARRAY_SIZE(usb_test_attrs); attr_count++)
		sysfs_remove_file(enhance_kobj, &usb_test_attrs[attr_count].attr);

	kobject_put(enhance_kobj);

}

MODULE_AUTHOR("NUBIA USB Driver Team Software");
MODULE_DESCRIPTION("NUBIA USB3.0 Saturation and Temperature Setting");
MODULE_LICENSE("GPL");
module_init(nubia_usb_test_init);
module_exit(nubia_usb_test_exit);
