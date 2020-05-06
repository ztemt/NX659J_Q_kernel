#ifndef _NUBIA_USB_TEST_
#define _NUBIA_USB_TEST_

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/component.h>
#include <linux/of_irq.h>
#include <linux/extcon.h>

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/completion.h>
#include <linux/list.h>
#include <linux/usb/gadget.h>
#include <linux/usb.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>

#include <linux/uaccess.h>
#include <asm/byteorder.h>


/* ------------------------- General Macro Definition ------------------------*/


/* ----------------------------- Structure ----------------------------------*/

/* ------------------------- Function Declaration ---------------------------*/
void nubia_usb_test(void);
#ifdef CONFIG_USB_DWC3 
void nubia_set_usbdev_ctrl(struct usb_device *device);
void nubia_set_usbhost_ctrl(struct usb_gadget *gadget);
#else
static inline void nubia_set_usbdev_ctrl(struct usb_device *device)
{

}

static inline void nubia_set_usbhost_ctrl(struct usb_gadget *gadget)
{

}
#endif
#endif
