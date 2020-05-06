/*
* This file is part of the nubia sensor common driver.
*Include
*sensor log
*sensor read write


*Reversion
*
====================================================================================
*/
#ifndef __SENSOR_COMMON_H__
#define __SENSOR_COMMON_H__

#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>

#include <linux/module.h>
#include <linux/mutex.h>

#include <linux/input.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/uaccess.h>




#define DRIVER_VERSION "1.0"

#define DEBUG_ON //DEBUG SWITCH
#define LOG_TAG "NUBIA_SENSOR"
#define SENSOR_LOG_LEVEL 1

#define SENSOR_LOG_ERROR(fmt, args...) printk(KERN_ERR "[%s] [%s:%d] " fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG_INFO(fmt, args...)  printk(KERN_INFO "[%s] [%s:%d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG_DEBUG_IF(en, fmt, args...) \
do { \
    if (en>=SENSOR_LOG_LEVEL) { \
        printk(KERN_ERR "[%s] [%s:%d] " fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args); \
    }; \
} while (0)

#define SENSOR_LOG_IF(en, fmt, args...) \
do { \
    if (en>=SENSOR_LOG_LEVEL) { \
        printk(KERN_ERR "[%s] [%s:%d] " fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args); \
    }; \
} while (0)

#ifdef  DEBUG_ON
#define SENSOR_LOG_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s:%d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#else
#define SENSOR_LOG_DEBUG(fmt, args...)
#endif

#ifdef AMS_MUTEX_DEBUG
#define AMS_MUTEX_LOCK(m) { \
        printk(KERN_INFO "%s: Mutex Lock\n", __func__); \
        mutex_lock(m); \
    }
#define AMS_MUTEX_UNLOCK(m) { \
        printk(KERN_INFO "%s: Mutex Unlock\n", __func__); \
        mutex_unlock(m); \
    }
#else
#define AMS_MUTEX_LOCK(m) { \
        mutex_lock(m); \
    }
#define AMS_MUTEX_UNLOCK(m) { \
        mutex_unlock(m); \
    }
#endif

int sensor_create_sysfs_interfaces(struct device *dev, struct device_attribute *dev_attrs, int count);
void sensor_remove_sysfs_interfaces(struct device *dev, struct device_attribute *dev_attrs, int count);
int sensor_write_file(char *file_path, const char *write_buf, int count);
int sensor_read_file(char *file_path, char *read_buf ,int count);


#endif
