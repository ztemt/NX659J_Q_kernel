/*
 *  stk3x3x.c - Linux kernel modules for sensortek stk301x, stk321x, stk331x
 *  , and stk3410 proximity/ambient light sensor
 *
 *  Copyright (C) 2012~2016 Lex Hsieh / sensortek <lex_hsieh@sensortek.com.tw>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/errno.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include   <linux/fs.h>
#include  <asm/uaccess.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
//#include <linux/earlysuspend.h>
#endif

/* Driver Settings */
#define STK_ALS_CHANGE_THD	10	/* The threshold to trigger ALS interrupt, unit: lux */
#define STK_INT_PS_MODE			1	/* 1, 2, or 3	*/
//#define STK_POLL_PS
#define STK_TUNE0
//#define STK_TUNE1
#define CALI_PS_EVERY_TIME
#define STK_DEBUG_PRINTF
#define STK_PS_CALIBRATION
//#define QUALCOMM_PLATFORM
#define STK_QUALCOMM_POWER_CTRL
#include <linux/regulator/consumer.h>

#ifdef QUALCOMM_PLATFORM
	#include <linux/sensors.h>
	#include <linux/regulator/consumer.h>
//	#define STK_QUALCOMM_POWER_CTRL
#endif

#include "stk3x3x.h"
#include "../sensor_common.h"

#undef DRIVER_VERSION
#define DRIVER_VERSION  "3.11.0"
#undef LOG_TAG
#define LOG_TAG "STK3332"

#define DEV_PS_NAME "proximity"

/* Nubia add for ps calibration */
#define DEVICE_CHIP_NAME "pa224:stk3332"
#define PS_UNCOVER_DATA_MIN             1
#define PS_UNCOVER_DATA_MAX             2000
#define PS_THRESH_DATA_MIN              60
#define PS_THRESH_DATA_MAX              2000
#define PS_THRESH_INCREMENT_MIN         60
#define PS_DATA_MAX                     35000
/* Nubia add end */

static dev_t stk_ps_dev_t;
static struct class *stk_ps_class;

/* Define Register Map */
#define STK_STATE_REG 			0x00
#define STK_PSCTRL_REG 			0x01
#define STK_ALSCTRL_REG 			0x02
#define STK_LEDCTRL_REG 			0x03
#define STK_INT_REG 				0x04
#define STK_WAIT_REG 			0x05
#define STK_THDH1_PS_REG 		0x06
#define STK_THDH2_PS_REG 		0x07
#define STK_THDL1_PS_REG 		0x08
#define STK_THDL2_PS_REG 		0x09
#define STK_THDH1_ALS_REG 		0x0A
#define STK_THDH2_ALS_REG 		0x0B
#define STK_THDL1_ALS_REG 		0x0C
#define STK_THDL2_ALS_REG 		0x0D
#define STK_FLAG_REG 			0x10
#define STK_DATA1_PS_REG	 	0x11
#define STK_DATA2_PS_REG 		0x12
#define STK_DATA1_ALS_REG 		0x13
#define STK_DATA2_ALS_REG 		0x14
#define STK_DATA1_OFFSET_REG 	0x15
#define STK_DATA2_OFFSET_REG 	0x16
#define STK_DATA1_IR_REG 		0x17
#define STK_DATA2_IR_REG 		0x18
#define STK_PDT_ID_REG 			0x3E
#define STK_RSRVD_REG 			0x3F
#define STK_SW_RESET_REG		0x80

#define STK_GSCTRL_REG			0x1A
#define STK_FLAG2_REG			0x1C

/* Define state reg */
#define STK_STATE_EN_IRS_SHIFT  	7
#define STK_STATE_EN_AK_SHIFT  	6
#define STK_STATE_EN_ASO_SHIFT  	5
#define STK_STATE_EN_IRO_SHIFT  	4
#define STK_STATE_EN_WAIT_SHIFT  	2
#define STK_STATE_EN_ALS_SHIFT  	1
#define STK_STATE_EN_PS_SHIFT  	0

#define STK_STATE_EN_IRS_MASK	0x80
#define STK_STATE_EN_AK_MASK	0x40
#define STK_STATE_EN_ASO_MASK	0x20
#define STK_STATE_EN_IRO_MASK	0x10
#define STK_STATE_EN_WAIT_MASK	0x04
#define STK_STATE_EN_ALS_MASK	0x02
#define STK_STATE_EN_PS_MASK	0x01

/* Define PS ctrl reg */
#define STK_PS_PRS_SHIFT  		6
#define STK_PS_GAIN_SHIFT  		4
#define STK_PS_IT_SHIFT  			0

#define STK_PS_PRS_MASK			0xC0
#define STK_PS_GAIN_MASK			0x30
#define STK_PS_IT_MASK			0x0F

/* Define ALS ctrl reg */
#define STK_ALS_PRS_SHIFT  		6
#define STK_ALS_GAIN_SHIFT  		4
#define STK_ALS_IT_SHIFT  			0

#define STK_ALS_PRS_MASK		0xC0
#define STK_ALS_GAIN_MASK		0x30
#define STK_ALS_IT_MASK			0x0F

/* Define LED ctrl reg */
#define STK_LED_IRDR_SHIFT  		6
#define STK_LED_DT_SHIFT  		0

#define STK_LED_IRDR_MASK		0xC0
#define STK_LED_DT_MASK			0x3F

/* Define interrupt reg */
#define STK_INT_CTRL_SHIFT  		7
#define STK_INT_OUI_SHIFT  		4
#define STK_INT_ALS_SHIFT  		3
#define STK_INT_PS_SHIFT  			0

#define STK_INT_CTRL_MASK		0x80
#define STK_INT_OUI_MASK			0x10
#define STK_INT_ALS_MASK			0x08
#define STK_INT_PS_MASK			0x07

#define STK_INT_ALS				0x08

/* Define flag reg */
#define STK_FLG_ALSDR_SHIFT  		7
#define STK_FLG_PSDR_SHIFT  		6
#define STK_FLG_ALSINT_SHIFT  		5
#define STK_FLG_PSINT_SHIFT  		4
#define STK_FLG_OUI_SHIFT  		2
#define STK_FLG_IR_RDY_SHIFT  		1
#define STK_FLG_NF_SHIFT  		0

#define STK_FLG_ALSDR_MASK		0x80
#define STK_FLG_PSDR_MASK		0x40
#define STK_FLG_ALSINT_MASK		0x20
#define STK_FLG_PSINT_MASK		0x10
#define STK_FLG_OUI_MASK			0x04
#define STK_FLG_IR_RDY_MASK		0x02
#define STK_FLG_NF_MASK			0x01

/* Define flag2 reg */
#define STK_FLG2_INT_GS_SHIFT		6
#define STK_FLG2_GS10_SHIFT		5
#define STK_FLG2_GS01_SHIFT		4

#define STK_FLG2_INT_GS_MASK	0x40
#define STK_FLG2_GS10_MASK		0x20
#define STK_FLG2_GS01_MASK		0x10

/* misc define */
#define MIN_ALS_POLL_DELAY_NS	60000000

#ifdef STK_TUNE0
	#define STK_MAX_MIN_DIFF	200
	#define STK_LT_N_CT	100
	#define STK_HT_N_CT	150
#endif	/* #ifdef STK_TUNE0 */

#define STK_IRC_MAX_ALS_CODE		20000
#define STK_IRC_MIN_ALS_CODE		25
#define STK_IRC_MIN_IR_CODE		50
#define STK_IRC_ALS_DENOMI		2
#define STK_IRC_ALS_NUMERA		5
#define STK_IRC_ALS_CORREC		850

#define STK_IRS_IT_REDUCE			2
#define STK_ALS_READ_IRS_IT_REDUCE	5
#define STK_ALS_THRESHOLD			30

#define DEVICE_NAME		"stk3332"

#define ALS_NAME "lightsensor-level"

#define PS_NAME "proximity"
#define PS_CAL_FILE_PATH      "/persist/sensors/xtalk_cal"
#define PS_CALI_COUNT       5
#define PS_FAR_DIFFER_VALUE   20

#ifdef STK_QUALCOMM_POWER_CTRL
	/* POWER SUPPLY VOLTAGE RANGE */
	#define STK3X3X_VDD_MIN_UV	2000000
	#define STK3X3X_VDD_MAX_UV	3300000
	#define STK3X3X_VIO_MIN_UV	1750000
	#define STK3X3X_VIO_MAX_UV	1950000
#endif

#define STK3310SA_PID		0x17
#define STK3311SA_PID		0x1E
#define STK3311WV_PID		0x1D
#define STK3311X_PID			0x12
#define STK33119_PID			0x11

#ifdef QUALCOMM_PLATFORM
static struct sensors_classdev sensors_proximity_cdev = {
	.name = "stk3x3x-proximity",
	.vendor = "Sensortek",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "1.0",
	.resolution = "1.0",
	.sensor_power = "0.1",
	.min_delay = 0,
	.max_delay = 0,
	.delay_msec = 200,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.flags = 3,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_calibrate = NULL,
	.sensors_write_cal_params = NULL,
	.params = NULL,
};
#endif

#ifdef SPREADTRUM_PLATFORM
extern int sprd_3rdparty_gpio_pls_irq;

static struct stk3x3x_platform_data stk3x3x_pfdata={
  .state_reg = 0x0,    /* disable all */
  .psctrl_reg = 0x31,    /* ps_persistance=1, ps_gain=64X, PS_IT=0.391ms */
  .ledctrl_reg = 0xFF,   /* 100mA IRDR, 64/64 LED duty */
  .wait_reg = 0xF,    /* 100 ms */
  .ps_thd_h =1700,
  .ps_thd_l = 1500,
  .int_pin = sprd_3rdparty_gpio_pls_irq,
};
#endif

struct stk3x3x_data {
	struct i2c_client *client;
	struct stk3x3x_platform_data *pdata;
	struct device *ps_dev;
#ifdef QUALCOMM_PLATFORM
	struct sensors_classdev ps_cdev;
#endif
#if (!defined(STK_POLL_PS))
    int32_t irq;
    struct work_struct stk_work;
	struct workqueue_struct *stk_wq;
#endif
	uint8_t psctrl_reg;
	uint8_t ledctrl_reg;
	uint8_t state_reg;
	int		int_pin;
	uint8_t wait_reg;
	uint8_t int_reg;
#ifdef CONFIG_HAS_EARLYSUSPEND
	//struct early_suspend stk_early_suspend;
#endif
	uint16_t ps_thd_h;
	uint16_t ps_thd_l;
#ifdef CALI_PS_EVERY_TIME
	uint16_t ps_high_thd_boot;
	uint16_t ps_low_thd_boot;
#endif
	struct mutex io_lock;
	struct input_dev *ps_input_dev;
	int32_t ps_distance_last;
	bool ps_enabled;
	bool re_enable_ps;
	struct wake_lock ps_wakelock;
#ifdef STK_POLL_PS
	struct hrtimer ps_timer;
    struct work_struct stk_ps_work;
	struct workqueue_struct *stk_ps_wq;
	struct wake_lock ps_nosuspend_wl;
#endif
	ktime_t ps_poll_delay;
	bool first_boot;
#ifdef STK_TUNE0
	uint16_t psa;
	uint16_t psi;
	uint16_t psi_set;
	struct hrtimer ps_tune0_timer;
	struct workqueue_struct *stk_ps_tune0_wq;
    struct work_struct stk_ps_tune0_work;
	ktime_t ps_tune0_delay;
	bool tune_zero_init_proc;
	uint32_t ps_stat_data[3];
	int data_count;
	int stk_max_min_diff;
	int stk_lt_n_ct;
	int stk_ht_n_ct;
#endif
	atomic_t	recv_reg;

#ifdef STK_QUALCOMM_POWER_CTRL
	struct regulator *vdd;
	struct regulator *vio;
	bool power_enabled;
#endif
	uint8_t pid;
	uint8_t	p_wv_r_bd_with_co;
	uint32_t als_code_last;
	bool als_en_hal;

	uint32_t ps_code_last;
	uint8_t boot_cali;
	uint8_t	p_1x_r_bd_with_co;
	uint8_t	p_19_r_bc;

#ifdef STK_PS_CALIBRATION
	uint16_t cali_crosstalk;
	uint16_t cali_near;
	uint16_t cali_near_increment;
	uint32_t cali_als;
	uint32_t lux_factor;

	bool ps_cali_timer_run;
	ktime_t ps_cali_delay;
	struct hrtimer ps_cali_timer;
	struct work_struct stk_ps_cali_work;
	struct workqueue_struct *stk_ps_cali_wq;
	struct wake_lock ps_cali_nosuspend_wl;

	uint16_t prox_debug;
	uint8_t  debug_cnt;
#endif
};

static int32_t stk3x3x_enable_ps(struct stk3x3x_data *ps_data, uint8_t enable, uint8_t validate_reg);
static int32_t stk3x3x_set_ps_thd_l(struct stk3x3x_data *ps_data, uint16_t thd_l);
static int32_t stk3x3x_set_ps_thd_h(struct stk3x3x_data *ps_data, uint16_t thd_h);
#ifdef STK_TUNE0
static int stk_ps_tune_zero_func_fae(struct stk3x3x_data *ps_data);
#endif
#ifdef STK_CHK_REG
static int stk3x3x_validate_n_handle(struct i2c_client *client);
#endif
static int stk_ps_val(struct stk3x3x_data *ps_data);
#ifdef STK_QUALCOMM_POWER_CTRL
static int stk3x3x_device_ctl(struct stk3x3x_data *ps_data, bool enable);
#endif

#ifdef STK_PS_CALIBRATION
static void stk_ps_cali_work_func(struct work_struct *work);
static enum hrtimer_restart stk_ps_cali_timer_func(struct hrtimer *timer);
#endif

static int stk3x3x_i2c_read_data(struct i2c_client *client, unsigned char command, int length, unsigned char *values)
{
	uint8_t retry;
	int err;
	struct i2c_msg msgs[] =
	{
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &command,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = values,
		},
	};

	for (retry = 0; retry < 5; retry++)
	{
		err = i2c_transfer(client->adapter, msgs, 2);
		if (err == 2)
			break;
		else
			mdelay(5);
	}

	if (retry >= 5)
	{
		SENSOR_LOG_ERROR("i2c read fail, err=%d\n", err);
		return -EIO;
	}
	return 0;
}

static int stk3x3x_i2c_write_data(struct i2c_client *client, unsigned char command, int length, unsigned char *values)
{
	int retry;
	int err;
	unsigned char data[11];
	struct i2c_msg msg;
	int index;

    if (!client)
		return -EINVAL;
    else if (length >= 10)
	{
        SENSOR_LOG_ERROR("length %d exceeds 10\n", length);
        return -EINVAL;
    }

	data[0] = command;
	for (index=1;index<=length;index++)
		data[index] = values[index-1];

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = length+1;
	msg.buf = data;

	for (retry = 0; retry < 5; retry++)
	{
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			break;
		else
			mdelay(5);
	}

	if (retry >= 5)
	{
		SENSOR_LOG_ERROR("i2c write fail, err=%d\n", err);
		return -EIO;
	}
	return 0;
}

static int stk3x3x_i2c_smbus_read_byte_data(struct i2c_client *client, unsigned char command)
{
	unsigned char value;
	int err;
	err = stk3x3x_i2c_read_data(client, command, 1, &value);
	if(err < 0)
		return err;
	return value;
}

static int stk3x3x_i2c_smbus_write_byte_data(struct i2c_client *client, unsigned char command, unsigned char value)
{
	int err;
	err = stk3x3x_i2c_write_data(client, command, 1, &value);
	return err;
}

static void stk3x3x_proc_plat_data(struct stk3x3x_data *ps_data, struct stk3x3x_platform_data *plat_data)
{
	uint8_t w_reg;

	ps_data->state_reg = plat_data->state_reg;
	ps_data->psctrl_reg = plat_data->psctrl_reg;
#ifdef STK_POLL_PS
	ps_data->psctrl_reg &= 0x3F;
#endif
	ps_data->ledctrl_reg = plat_data->ledctrl_reg;
	if(ps_data->pid == STK3310SA_PID || ps_data->pid == STK3311SA_PID)
		ps_data->ledctrl_reg &= 0x3F;
	ps_data->wait_reg = plat_data->wait_reg;
	if(ps_data->wait_reg < 2)
	{
		SENSOR_LOG_ERROR("wait_reg should be larger than 2, force to write 2\n");
		ps_data->wait_reg = 2;
	}
	else if (ps_data->wait_reg > 0xFF)
	{
		SENSOR_LOG_ERROR("wait_reg should be less than 0xFF, force to write 0xFF\n");
		ps_data->wait_reg = 0xFF;
	}

	if(ps_data->ps_thd_h == 0 && ps_data->ps_thd_l == 0)
	{
		ps_data->ps_thd_h = plat_data->ps_thd_h;
		ps_data->ps_thd_l = plat_data->ps_thd_l;
	}

#ifdef CALI_PS_EVERY_TIME
	ps_data->ps_high_thd_boot = plat_data->ps_thd_h;
	ps_data->ps_low_thd_boot = plat_data->ps_thd_l;
#endif
	w_reg = 0;
#ifndef STK_POLL_PS
	#ifdef STK_TUNE1
	w_reg |= 0x07;
	#else
	w_reg |= STK_INT_PS_MODE;
	#endif
#else
	w_reg |= 0x01;
#endif

	ps_data->int_reg = w_reg;
	return;
}

static int32_t stk3x3x_init_all_reg(struct stk3x3x_data *ps_data)
{
	int32_t ret;

    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, ps_data->state_reg);
    if (ret < 0)
    {
        SENSOR_LOG_ERROR("write i2c error\n");
        return ret;
    }
    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_PSCTRL_REG, ps_data->psctrl_reg);
    if (ret < 0)
    {
        SENSOR_LOG_ERROR("write i2c error\n");
        return ret;
    }
    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_LEDCTRL_REG, ps_data->ledctrl_reg);
    if (ret < 0)
    {
        SENSOR_LOG_ERROR("write i2c error\n");
        return ret;
    }
    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_WAIT_REG, ps_data->wait_reg);
    if (ret < 0)
    {
        SENSOR_LOG_ERROR("write i2c error\n");
        return ret;
    }
#ifdef STK_TUNE0
	ps_data->psa = 0x0;
	ps_data->psi = 0xFFFF;
#endif
	stk3x3x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
	stk3x3x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);

    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_INT_REG, ps_data->int_reg);
    if (ret < 0)
	{
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}

	//INTEL PEERS
    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, 0x4F, 0x3F);
    if (ret < 0)
	{
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}

	//PS INT mode
    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, 0xFA, 0x01);
    if (ret < 0)
	{
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}

    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, 0xA0, 0x10);
    if (ret < 0)
	{
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}

    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, 0xAA, 0x64);
    if (ret < 0)
	{
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}

    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, 0xDB, 0x15);
    if (ret < 0)
	{
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}

	return 0;
}

static int stk3x3x_otp_read_byte_data(struct i2c_client *client, unsigned char command)
{
	unsigned char value;
	int err;

    err = stk3x3x_i2c_smbus_write_byte_data(client, 0x0, 0x2);
    if (err < 0)
    {
        SENSOR_LOG_ERROR("write i2c error\n");
        return err;
    }

	err = stk3x3x_i2c_smbus_write_byte_data(client, 0x90, command);
	if(err)
		return err;

	err = stk3x3x_i2c_smbus_write_byte_data(client, 0x92, 0x82);
	if(err)
		return err;

	usleep_range(2000, 4000);

	err = stk3x3x_i2c_smbus_read_byte_data(client, 0x91);
	if(err < 0)
		return err;
	value = err;
	SENSOR_LOG_INFO("read OTP 0x%x=0x%x", command, value);

    err = stk3x3x_i2c_smbus_write_byte_data(client, 0x0, 0x0);
    if (err < 0)
    {
        SENSOR_LOG_ERROR("write i2c error\n");
        return err;
    }

	return value;
}

static int32_t stk3x3x_check_pid(struct stk3x3x_data *ps_data)
{
	unsigned char value[3], pid_msb;
	int err;
	int otp25;

	ps_data->p_wv_r_bd_with_co = 0;
	ps_data->p_1x_r_bd_with_co = 0;
	ps_data->p_19_r_bc = 0;

	err = stk3x3x_i2c_read_data(ps_data->client, STK_PDT_ID_REG, 2, &value[0]);
	if(err < 0)
	{
		SENSOR_LOG_ERROR("fail, ret=%d\n", err);
		return err;
	}
	err = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, 0xE0);
	if(err < 0)
		return err;
	value[2] = err;

	SENSOR_LOG_INFO("PID=0x%x, RID=0x%x, 0x90=0x%x\n", value[0], value[1], value[2]);
	ps_data->pid = value[0];

	if(value[0] == STK3311WV_PID)
		ps_data->p_wv_r_bd_with_co |= 0b100;
	else if(value[0] == STK3311X_PID)
		ps_data->p_1x_r_bd_with_co |= 0b100;
	else if(value[0] == STK33119_PID)
		ps_data->p_19_r_bc |= 0b10;

	if(value[1] == 0xC3)
	{
		ps_data->p_wv_r_bd_with_co |= 0b010;
		ps_data->p_1x_r_bd_with_co |= 0b010;
	}
	else if(value[1] == 0xC2)
	{
		ps_data->p_19_r_bc |= 0b01;
	}

	err = stk3x3x_otp_read_byte_data(ps_data->client, 0x25);
	if(err < 0)
		return err;

	otp25 = err;
	if(otp25 & 0x80)
		ps_data->p_wv_r_bd_with_co |= 0b001;
	SENSOR_LOG_INFO(" p_wv_r_bd_with_co = 0x%x\n", ps_data->p_wv_r_bd_with_co);

	if(otp25 & 0x40)
		ps_data->p_1x_r_bd_with_co |= 0b001;
	SENSOR_LOG_INFO(" p_1x_r_bd_with_co = 0x%x\n", ps_data->p_1x_r_bd_with_co);

	SENSOR_LOG_INFO(" p_19_r_bc = 0x%x\n", ps_data->p_19_r_bc);

	if(value[0] == 0)
	{
		SENSOR_LOG_ERROR("PID=0x0, please make sure the chip is stk3x3x!\n");
		return -2;
	}

	pid_msb = value[0] & 0xF0;
	switch(pid_msb)
	{
	case 0x10:
	case 0x20:
	case 0x30:
	case 0x50:
		return 0;
	default:
		SENSOR_LOG_ERROR("invalid PID(%#x)\n", value[0]);
		return -1;
	}
	return 0;
}


static int32_t stk3x3x_software_reset(struct stk3x3x_data *ps_data)
{
    int32_t r;
    uint8_t w_reg;

    w_reg = 0x7F;
    r = stk3x3x_i2c_smbus_write_byte_data(ps_data->client,STK_WAIT_REG,w_reg);
    if (r<0)
    {
        SENSOR_LOG_ERROR("software reset: write i2c error, ret=%d\n", r);
        return r;
    }
    r = stk3x3x_i2c_smbus_read_byte_data(ps_data->client,STK_WAIT_REG);
    if (w_reg != r)
    {
        SENSOR_LOG_ERROR("software reset: read-back value is not the same\n");
        return -1;
    }

    r = stk3x3x_i2c_smbus_write_byte_data(ps_data->client,STK_SW_RESET_REG,0);
    if (r<0)
    {
        SENSOR_LOG_ERROR("software reset: read error after reset\n");
        return r;
    }
	usleep_range(30000, 50000);
    return 0;
}


static int32_t stk3x3x_set_ps_thd_l(struct stk3x3x_data *ps_data, uint16_t thd_l)
{
	unsigned char val[2];
	int ret;
	val[0] = (thd_l & 0xFF00) >> 8;
	val[1] = thd_l & 0x00FF;
	ret = stk3x3x_i2c_write_data(ps_data->client, STK_THDL1_PS_REG, 2, val);
	if(ret < 0)
		SENSOR_LOG_ERROR("fail, ret=%d\n", ret);
	return ret;
}
static int32_t stk3x3x_set_ps_thd_h(struct stk3x3x_data *ps_data, uint16_t thd_h)
{
	unsigned char val[2];
	int ret;
	val[0] = (thd_h & 0xFF00) >> 8;
	val[1] = thd_h & 0x00FF;
	ret = stk3x3x_i2c_write_data(ps_data->client, STK_THDH1_PS_REG, 2, val);
	if(ret < 0)
		SENSOR_LOG_ERROR("fail, ret=%d\n", ret);
	return ret;
}

static uint32_t stk3x3x_get_ps_reading(struct stk3x3x_data *ps_data)
{
	unsigned char value[2];
	int err;
	err = stk3x3x_i2c_read_data(ps_data->client, STK_DATA1_PS_REG, 2, &value[0]);
	if(err < 0)
	{
		SENSOR_LOG_ERROR("fail, ret=%d\n", err);
		return err;
	}
	return ((value[0]<<8) | value[1]);
}


static int32_t stk3x3x_set_flag(struct stk3x3x_data *ps_data, uint8_t org_flag_reg, uint8_t clr)
{
	uint8_t w_flag;
	int ret;

	w_flag = org_flag_reg | (STK_FLG_ALSINT_MASK | STK_FLG_PSINT_MASK | STK_FLG_OUI_MASK | STK_FLG_IR_RDY_MASK);
	w_flag &= (~clr);
	//SENSOR_LOG_INFO(" org_flag_reg=0x%x, w_flag = 0x%x\n", org_flag_reg, w_flag);
    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client,STK_FLAG_REG, w_flag);
	if(ret < 0)
		SENSOR_LOG_ERROR("fail, ret=%d\n", ret);
	return ret;
}

static int32_t stk3x3x_get_flag(struct stk3x3x_data *ps_data)
{
	int ret;
    ret = stk3x3x_i2c_smbus_read_byte_data(ps_data->client,STK_FLAG_REG);
	if(ret < 0)
		SENSOR_LOG_ERROR("fail, ret=%d\n", ret);
	return ret;
}

static int32_t stk3x3x_set_state(struct stk3x3x_data *ps_data, uint8_t state)
{
	int ret;
    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client,STK_STATE_REG, state);
	if(ret < 0)
		SENSOR_LOG_ERROR("fail, ret=%d\n", ret);
	return ret;
}

static int32_t stk3x3x_get_state(struct stk3x3x_data *ps_data)
{
	int ret;
    ret = stk3x3x_i2c_smbus_read_byte_data(ps_data->client,STK_STATE_REG);
	if(ret < 0)
		SENSOR_LOG_ERROR("fail, ret=%d\n", ret);
	return ret;
}

static void stk_ps_report(struct stk3x3x_data *ps_data, int nf)
{
#ifdef QUALCOMM_PLATFORM
	ktime_t	timestamp = ktime_get_boottime();
#endif

	ps_data->ps_distance_last = nf;
//	input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, nf);
	input_report_rel(ps_data->ps_input_dev, REL_RZ, nf? 10:3);

#ifdef QUALCOMM_PLATFORM
	input_event(ps_data->ps_input_dev, EV_SYN, SYN_TIME_SEC, ktime_to_timespec(timestamp).tv_sec);
	input_event(ps_data->ps_input_dev, EV_SYN, SYN_TIME_NSEC, ktime_to_timespec(timestamp).tv_nsec);
#endif
	input_sync(ps_data->ps_input_dev);
	wake_lock_timeout(&ps_data->ps_wakelock, 3*HZ);
}

static int32_t stk3x3x_enable_ps(struct stk3x3x_data *ps_data, uint8_t enable, uint8_t validate_reg)
{
    int32_t ret;
	uint8_t w_state_reg;
	uint8_t curr_ps_enable;
	uint32_t reading;
	int32_t near_far_state;

#ifdef STK_QUALCOMM_POWER_CTRL
	if (enable) {
		ret = stk3x3x_device_ctl(ps_data, enable);
		if (ret)
			return ret;
	}
#endif

#ifdef STK_CHK_REG
	if(validate_reg)
	{
		ret = stk3x3x_validate_n_handle(ps_data->client);
		if(ret < 0)
			SENSOR_LOG_ERROR("stk3x3x_validate_n_handle fail: %d\n", ret);
	}
#endif /* #ifdef STK_CHK_REG */

	curr_ps_enable = ps_data->ps_enabled?1:0;
	if(curr_ps_enable == enable)
		return 0;

#ifdef STK_TUNE0
	if (!(ps_data->psi_set) && !enable)
	{
		hrtimer_cancel(&ps_data->ps_tune0_timer);
		cancel_work_sync(&ps_data->stk_ps_tune0_work);
	}
#endif
	if(ps_data->first_boot == true)
	{
		ps_data->first_boot = false;
	}
    if(enable)
	{
		stk3x3x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
		stk3x3x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);
	}

	ret = stk3x3x_get_state(ps_data);
	if(ret < 0)
		return ret;
	w_state_reg = ret;


	w_state_reg &= ~(STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK | STK_STATE_EN_AK_MASK);
	if(enable)
	{
		w_state_reg |= STK_STATE_EN_PS_MASK;
		w_state_reg |= STK_STATE_EN_WAIT_MASK;
	}
	ret = stk3x3x_set_state(ps_data, w_state_reg);
	if(ret < 0)
		return ret;
	ps_data->state_reg = w_state_reg;

    if(enable)
	{
#ifdef STK_TUNE0
	#ifdef CALI_PS_EVERY_TIME
		ps_data->psi_set = 0;
		ps_data->psa = 0;
		ps_data->psi = 0xFFFF;
		ps_data->ps_thd_h = ps_data->ps_high_thd_boot;
		ps_data->ps_thd_l = ps_data->ps_low_thd_boot;
		hrtimer_start(&ps_data->ps_tune0_timer, ps_data->ps_tune0_delay, HRTIMER_MODE_REL);
	#else
		if (!(ps_data->psi_set))
			hrtimer_start(&ps_data->ps_tune0_timer, ps_data->ps_tune0_delay, HRTIMER_MODE_REL);
	#endif	/* #ifdef CALI_PS_EVERY_TIME */
#endif

#ifdef STK_CHK_REG
		if(!validate_reg)
		{
			reading = stk3x3x_get_ps_reading(ps_data);
			stk_ps_report(ps_data, 1);
			SENSOR_LOG_INFO(" force report ps input event=1, ps code = %d\n", reading);
		}
		else
#endif /* #ifdef STK_CHK_REG */
		{
			usleep_range(4000, 5000);
			reading = stk3x3x_get_ps_reading(ps_data);
			if (reading < 0)
				return reading;

			ret = stk3x3x_get_flag(ps_data);
			if (ret < 0)
				return ret;
			near_far_state = ret & STK_FLG_NF_MASK;
			stk_ps_report(ps_data, near_far_state);
			SENSOR_LOG_INFO(" ps input event=%d, ps=%d\n",near_far_state, reading);
		}
#ifdef STK_POLL_PS
		hrtimer_start(&ps_data->ps_timer, ps_data->ps_poll_delay, HRTIMER_MODE_REL);
		ps_data->ps_distance_last = -1;
#endif

		enable_irq(ps_data->irq);
		ps_data->ps_enabled = true;

#ifdef CALI_PS_EVERY_TIME
		if(ps_data->boot_cali == 1 && ps_data->ps_low_thd_boot < 1000 )
		{
			ps_data->ps_thd_h = ps_data->ps_high_thd_boot;
			ps_data->ps_thd_l = ps_data->ps_low_thd_boot;
		}
		else
#endif
			SENSOR_LOG_INFO(" HT=%d, LT=%d\n", ps_data->ps_thd_h, ps_data->ps_thd_l);
	}
	else
	{
		disable_irq(ps_data->irq);
		ps_data->ps_enabled = false;
#ifdef STK_QUALCOMM_POWER_CTRL
		ret = stk3x3x_device_ctl(ps_data, enable);
		if (ret)
			return ret;
#endif
	}
	return ret;
}

#ifdef STK_CHK_REG
static int stk3x3x_chk_reg_valid(struct stk3x3x_data *ps_data)
{
	unsigned char value[9];
	int err;
	/*
	uint8_t cnt;

	for(cnt=0;cnt<9;cnt++)
	{
		value[cnt] = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, (cnt+1));
		if(value[cnt] < 0)
		{
			SENSOR_LOG_ERROR("fail, ret=%d", value[cnt]);
			return value[cnt];
		}
	}
	*/
	err = stk3x3x_i2c_read_data(ps_data->client, STK_PSCTRL_REG, 9, &value[0]);
	if(err < 0)
	{
		SENSOR_LOG_ERROR("fail, ret=%d\n", err);
		return err;
	}

	if(value[0] != ps_data->psctrl_reg)
	{
		SENSOR_LOG_ERROR("invalid reg 0x01=0x%2x\n", value[0]);
		return 0xFF;
	}

	if((value[1] != ps_data->alsctrl_reg) && (value[1] != (ps_data->alsctrl_reg - STK_ALS_READ_IRS_IT_REDUCE)))
	{
		SENSOR_LOG_ERROR("invalid reg 0x02=0x%2x\n", value[1]);
		return 0xFF;
	}
	if(value[2] != ps_data->ledctrl_reg)
	{
		SENSOR_LOG_ERROR("invalid reg 0x03=0x%2x\n", value[2]);
		return 0xFF;
	}
	if(value[3] != ps_data->int_reg)
	{
		SENSOR_LOG_ERROR("invalid reg 0x04=0x%2x\n", value[3]);
		return 0xFF;
	}
	if(value[4] != ps_data->wait_reg)
	{
		SENSOR_LOG_ERROR("invalid reg 0x05=0x%2x\n", value[4]);
		return 0xFF;
	}
	if(value[5] != ((ps_data->ps_thd_h & 0xFF00) >> 8))
	{
		SENSOR_LOG_ERROR("invalid reg 0x06=0x%2x\n", value[5]);
		return 0xFF;
	}
	if(value[6] != (ps_data->ps_thd_h & 0x00FF))
	{
		SENSOR_LOG_ERROR("invalid reg 0x07=0x%2x\n", value[6]);
		return 0xFF;
	}
	if(value[7] != ((ps_data->ps_thd_l & 0xFF00) >> 8))
	{
		SENSOR_LOG_ERROR("invalid reg 0x08=0x%2x\n", value[7]);
		return 0xFF;
	}
	if(value[8] != (ps_data->ps_thd_l & 0x00FF))
	{
		SENSOR_LOG_ERROR("invalid reg 0x09=0x%2x\n", value[8]);
		return 0xFF;
	}

	return 0;
}

static int stk3x3x_validate_n_handle(struct i2c_client *client)
{
	struct stk3x3x_data *ps_data = i2c_get_clientdata(client);
	int err;

	err = stk3x3x_chk_reg_valid(ps_data);
	if(err < 0)
	{
		SENSOR_LOG_ERROR("stk3x3x_chk_reg_valid fail: %d\n", err);
		return err;
	}

	if(err == 0xFF)
	{
		SENSOR_LOG_ERROR("Re-init chip\n");
		err = stk3x3x_software_reset(ps_data);
		if(err < 0)
			return err;
		err = stk3x3x_init_all_reg(ps_data);
		if(err < 0)
			return err;

		//ps_data->psa = 0;
		//ps_data->psi = 0xFFFF;
		stk3x3x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
		stk3x3x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);
		return 0xFF;
	}
	return 0;
}
#endif /* #ifdef STK_CHK_REG */

static ssize_t stk_ps_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
    uint32_t reading;
    reading = stk3x3x_get_ps_reading(ps_data);
    return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
}

#ifdef QUALCOMM_PLATFORM
static int stk_ps_enable_set(struct sensors_classdev *sensors_cdev,
						unsigned int enabled)
{
	struct stk3x3x_data *ps_data = container_of(sensors_cdev,
						struct stk3x3x_data, ps_cdev);
	int err;

	mutex_lock(&ps_data->io_lock);
	err = stk3x3x_enable_ps(ps_data, enabled, 0);
	mutex_unlock(&ps_data->io_lock);

	if (err < 0)
		return err;
	return 0;
}
#endif

#ifdef STK_PS_CALIBRATION
static ssize_t stk_prox_dev_init_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return 1;
}

static ssize_t stk_prox_dev_init_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int val = 0;
	int rc;
	uint16_t data[2] = {0};
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
	rc = kstrtoint(buf, 0, &val);
	if (val) {
		rc = sensor_read_file(PS_CAL_FILE_PATH, (char*)data, sizeof(data));
		if (rc < 0) {
			SENSOR_LOG_ERROR("read file error\n");
			return -1;
		}

		ps_data->cali_near_increment = data[0];
		SENSOR_LOG_ERROR("chip->cali_near_increment = %d\n",
		            ps_data->cali_near_increment);
	}

    ps_data->stk_ht_n_ct = ps_data->cali_near_increment;
    ps_data->stk_lt_n_ct = ps_data->stk_ht_n_ct - PS_FAR_DIFFER_VALUE;
	SENSOR_LOG_ERROR("stk_ht_n_ct = %d, stk_lt_n_ct = %d\n",
				ps_data->stk_ht_n_ct, ps_data->stk_lt_n_ct);

	return size;
}
#endif

static ssize_t stk_ps_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ret;
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);

	ret = stk3x3x_get_state(ps_data);
	if(ret < 0)
		return ret;
    ret = (ret & STK_STATE_EN_PS_MASK)?1:0;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t stk_ps_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
	uint8_t en;
	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else
	{
		SENSOR_LOG_ERROR(", invalid value %d\n", *buf);
		return -EINVAL;
	}
    SENSOR_LOG_INFO(" Enable PS : %d\n", en);
    mutex_lock(&ps_data->io_lock);
    stk3x3x_enable_ps(ps_data, en, 0);
    mutex_unlock(&ps_data->io_lock);
    return size;
}

static ssize_t stk_ps_enable_aso_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ret;
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);

    ret = stk3x3x_i2c_smbus_read_byte_data(ps_data->client,STK_STATE_REG);
    ret = (ret & STK_STATE_EN_ASO_MASK)?1:0;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t stk_ps_enable_aso_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
	uint8_t en;
    int32_t ret;
	uint8_t w_state_reg;

	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else
	{
		SENSOR_LOG_ERROR(", invalid value %d\n", *buf);
		return -EINVAL;
	}
    SENSOR_LOG_INFO(" Enable PS ASO : %d\n", en);

    ret = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, STK_STATE_REG);
    if (ret < 0)
    {
        SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
    }
	w_state_reg = (uint8_t)(ret & (~STK_STATE_EN_ASO_MASK));
	if(en)
		w_state_reg |= STK_STATE_EN_ASO_MASK;

    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, w_state_reg);
    if (ret < 0)
	{
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}

	return size;
}


static ssize_t stk_ps_offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
    int32_t word_data;
	unsigned char value[2];
	int ret;

	ret = stk3x3x_i2c_read_data(ps_data->client, STK_DATA1_OFFSET_REG, 2, &value[0]);
	if(ret < 0)
	{
		SENSOR_LOG_ERROR("fail, ret=0x%x", ret);
		return ret;
	}
	word_data = (value[0]<<8) | value[1];

	return scnprintf(buf, PAGE_SIZE, "%d\n", word_data);
}

static ssize_t stk_ps_offset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
	unsigned long offset = 0;
	int ret;
	unsigned char val[2];

	ret = kstrtoul(buf, 10, &offset);
	if(ret < 0)
	{
		SENSOR_LOG_ERROR("kstrtoul failed, ret=0x%x\n", ret);
		return ret;
	}
	if(offset > 65535)
	{
		SENSOR_LOG_ERROR("invalid value, offset=%ld\n", offset);
		return -EINVAL;
	}

	val[0] = (offset & 0xFF00) >> 8;
	val[1] = offset & 0x00FF;
	ret = stk3x3x_i2c_write_data(ps_data->client, STK_DATA1_OFFSET_REG, 2, val);
	if(ret < 0)
	{
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}

	return size;
}


static ssize_t stk_ps_distance_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
    int32_t dist=1;
	int32_t ret;

    ret = stk3x3x_get_flag(ps_data);
	if(ret < 0)
		return ret;
    dist = (ret & STK_FLG_NF_MASK)?1:0;
	stk_ps_report(ps_data, dist);
	SENSOR_LOG_INFO(" ps input event=%d\n", dist);
    return scnprintf(buf, PAGE_SIZE, "%d\n", dist);
}


static ssize_t stk_ps_distance_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;

	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		SENSOR_LOG_ERROR("kstrtoul failed, ret=0x%x\n", ret);
		return ret;
	}
	stk_ps_report(ps_data, value);
	SENSOR_LOG_INFO(" ps input event=%d\n", (int)value);
    return size;
}


static ssize_t stk_ps_code_thd_l_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ps_thd_l1_reg, ps_thd_l2_reg;
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
    ps_thd_l1_reg = stk3x3x_i2c_smbus_read_byte_data(ps_data->client,STK_THDL1_PS_REG);
    if(ps_thd_l1_reg < 0)
	{
		SENSOR_LOG_ERROR("fail, err=0x%x", ps_thd_l1_reg);
		return -EINVAL;
	}
    ps_thd_l2_reg = stk3x3x_i2c_smbus_read_byte_data(ps_data->client,STK_THDL2_PS_REG);
    if(ps_thd_l2_reg < 0)
	{
		SENSOR_LOG_ERROR("fail, err=0x%x", ps_thd_l2_reg);
		return -EINVAL;
	}
	ps_thd_l1_reg = ps_thd_l1_reg<<8 | ps_thd_l2_reg;
    return scnprintf(buf, PAGE_SIZE, "%d\n", ps_thd_l1_reg);
}


static ssize_t stk_ps_code_thd_l_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		SENSOR_LOG_ERROR("kstrtoul failed, ret=0x%x\n", ret);
		return ret;
	}
    stk3x3x_set_ps_thd_l(ps_data, value);
    return size;
}

static ssize_t stk_ps_code_thd_h_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ps_thd_h1_reg, ps_thd_h2_reg;
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
    ps_thd_h1_reg = stk3x3x_i2c_smbus_read_byte_data(ps_data->client,STK_THDH1_PS_REG);
    if(ps_thd_h1_reg < 0)
	{
		SENSOR_LOG_ERROR("fail, err=0x%x", ps_thd_h1_reg);
		return -EINVAL;
	}
    ps_thd_h2_reg = stk3x3x_i2c_smbus_read_byte_data(ps_data->client,STK_THDH2_PS_REG);
    if(ps_thd_h2_reg < 0)
	{
		SENSOR_LOG_ERROR("fail, err=0x%x", ps_thd_h2_reg);
		return -EINVAL;
	}
	ps_thd_h1_reg = ps_thd_h1_reg<<8 | ps_thd_h2_reg;
    return scnprintf(buf, PAGE_SIZE, "%d\n", ps_thd_h1_reg);
}

static ssize_t stk_ps_code_thd_h_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		SENSOR_LOG_ERROR("kstrtoul failed, ret=0x%x\n", ret);
		return ret;
	}
    stk3x3x_set_ps_thd_h(ps_data, value);
    return size;
}

static ssize_t stk_all_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ps_reg[0x23];
	uint8_t cnt;
	int len = 0;
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);

	for(cnt=0;cnt<0x20;cnt++)
	{
		ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, (cnt));
		if(ps_reg[cnt] < 0)
		{
			SENSOR_LOG_ERROR("fail, ret=%d", ps_reg[cnt]);
			return -EINVAL;
		}
		else
		{
			SENSOR_LOG_INFO("reg[0x%2X]=0x%2X\n", cnt, ps_reg[cnt]);
			len += scnprintf(buf+len, PAGE_SIZE-len, "[%2X]%2X,", cnt, ps_reg[cnt]);
		}
	}
	ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, STK_PDT_ID_REG);
	if(ps_reg[cnt] < 0)
	{
		SENSOR_LOG_ERROR("fail, ret=%d", ps_reg[cnt]);
		return -EINVAL;
	}
	SENSOR_LOG_INFO("reg[0x%x]=0x%2X\n", STK_PDT_ID_REG, ps_reg[cnt]);

	cnt++;
	ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, STK_RSRVD_REG);
	if(ps_reg[cnt] < 0)
	{
		SENSOR_LOG_ERROR("fail, ret=%d", ps_reg[cnt]);
		return -EINVAL;
	}
	SENSOR_LOG_INFO("reg[0x%x]=0x%2X\n", STK_RSRVD_REG, ps_reg[cnt]);

	cnt++;
	ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, 0xE0);
	if(ps_reg[cnt] < 0)
	{
		SENSOR_LOG_ERROR("fail, ret=%d", ps_reg[cnt]);
		return -EINVAL;
	}
	SENSOR_LOG_INFO("reg[0xE0]=0x%2X\n", ps_reg[cnt]);
	len += scnprintf(buf+len, PAGE_SIZE-len, "[3E]%2X,[3F]%2X,[E0]%2X\n", ps_reg[cnt-2], ps_reg[cnt-1], ps_reg[cnt]);
	return len;
/*
    return scnprintf(buf, PAGE_SIZE, "[0]%2X [1]%2X [2]%2X [3]%2X [4]%2X [5]%2X [6/7 HTHD]%2X,%2X [8/9 LTHD]%2X, %2X [A]%2X [B]%2X [C]%2X [D]%2X [E/F Aoff]%2X,%2X,[10]%2X [11/12 PS]%2X,%2X [13]%2X [14]%2X [15/16 Foff]%2X,%2X [17]%2X [18]%2X [3E]%2X [3F]%2X\n",
		ps_reg[0], ps_reg[1], ps_reg[2], ps_reg[3], ps_reg[4], ps_reg[5], ps_reg[6], ps_reg[7], ps_reg[8],
		ps_reg[9], ps_reg[10], ps_reg[11], ps_reg[12], ps_reg[13], ps_reg[14], ps_reg[15], ps_reg[16], ps_reg[17],
		ps_reg[18], ps_reg[19], ps_reg[20], ps_reg[21], ps_reg[22], ps_reg[23], ps_reg[24], ps_reg[25], ps_reg[26]);
		*/
}

static ssize_t stk_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ps_reg[27];
	uint8_t cnt;
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
	for(cnt=0;cnt<25;cnt++)
	{
		ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, (cnt));
		if(ps_reg[cnt] < 0)
		{
			SENSOR_LOG_ERROR("fail, ret=%d", ps_reg[cnt]);
			return -EINVAL;
		}
		else
		{
			SENSOR_LOG_INFO("reg[0x%2X]=0x%2X\n", cnt, ps_reg[cnt]);
		}
	}
	ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, STK_PDT_ID_REG);
	if(ps_reg[cnt] < 0)
	{
		SENSOR_LOG_ERROR("fail, ret=%d", ps_reg[cnt]);
		return -EINVAL;
	}
	SENSOR_LOG_INFO("reg[0x%x]=0x%2X\n", STK_PDT_ID_REG, ps_reg[cnt]);
	cnt++;
	ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, STK_RSRVD_REG);
	if(ps_reg[cnt] < 0)
	{
		SENSOR_LOG_ERROR("fail, ret=%d", ps_reg[cnt]);
		return -EINVAL;
	}
	SENSOR_LOG_INFO("reg[0x%x]=0x%2X\n", STK_RSRVD_REG, ps_reg[cnt]);

    return scnprintf(buf, PAGE_SIZE, "[PS=%2X] [ALS=%2X] [WAIT=0x%4Xms] [EN_ASO=%2X] [EN_AK=%2X] [NEAR/FAR=%2X] [FLAG_OUI=%2X] [FLAG_PSINT=%2X] [FLAG_ALSINT=%2X]\n",
		ps_reg[0]&0x01,(ps_reg[0]&0x02)>>1,((ps_reg[0]&0x04)>>2)*ps_reg[5]*6,(ps_reg[0]&0x20)>>5,
		(ps_reg[0]&0x40)>>6,ps_reg[16]&0x01,(ps_reg[16]&0x04)>>2,(ps_reg[16]&0x10)>>4,(ps_reg[16]&0x20)>>5);
}

static ssize_t stk_recv_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&ps_data->recv_reg));
}


static ssize_t stk_recv_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned long value = 0;
	int ret;
	int32_t recv_data;
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);

	if((ret = kstrtoul(buf, 16, &value)) < 0)
	{
		SENSOR_LOG_ERROR("kstrtoul failed, ret=0x%x\n", ret);
		return ret;
	}
	recv_data = stk3x3x_i2c_smbus_read_byte_data(ps_data->client,value);
//	SENSOR_LOG_INFO("reg 0x%x=0x%x\n", (int)value, recv_data);
	atomic_set(&ps_data->recv_reg, recv_data);
	return size;
}


static ssize_t stk_send_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}


static ssize_t stk_send_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int addr, cmd;
	int32_t ret, i;
	char *token[10];
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);

	for (i = 0; i < 2; i++)
		token[i] = strsep((char **)&buf, " ");
	if((ret = kstrtoul(token[0], 16, (unsigned long *)&(addr))) < 0)
	{
		SENSOR_LOG_ERROR("kstrtoul failed, ret=0x%x\n", ret);
		return ret;
	}
	if((ret = kstrtoul(token[1], 16, (unsigned long *)&(cmd))) < 0)
	{
		SENSOR_LOG_ERROR("kstrtoul failed, ret=0x%x\n", ret);
		return ret;
	}
	SENSOR_LOG_INFO(" write reg 0x%x=0x%x\n", addr, cmd);

	ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, (unsigned char)addr, (unsigned char)cmd);
	if (0 != ret)
	{
		SENSOR_LOG_ERROR("stk3x3x_i2c_smbus_write_byte_data fail\n");
		return ret;
	}

	return size;
}

#ifdef STK_TUNE0
static ssize_t stk_ps_cali_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
	int32_t word_data;
	unsigned char value[2];
	int ret;

	ret = stk3x3x_i2c_read_data(ps_data->client, 0x20, 2, &value[0]);
	if(ret < 0)
	{
		SENSOR_LOG_ERROR("fail, ret=0x%x", ret);
		return ret;
	}
	word_data = (value[0]<<8) | value[1];

	ret = stk3x3x_i2c_read_data(ps_data->client, 0x22, 2, &value[0]);
	if(ret < 0)
	{
		SENSOR_LOG_ERROR("fail, ret=0x%x", ret);
		return ret;
	}
	word_data += ((value[0]<<8) | value[1]);

	SENSOR_LOG_INFO("psi_set=%d, psa=%d,psi=%d, word_data=%d\n",
		ps_data->psi_set, ps_data->psa, ps_data->psi, word_data);
#ifdef CALI_PS_EVERY_TIME
	SENSOR_LOG_INFO("boot HT=%d, LT=%d\n", ps_data->ps_high_thd_boot, ps_data->ps_low_thd_boot);
#endif
	return 0;
}

static ssize_t stk_ps_maxdiff_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
    unsigned long value = 0;
	int ret;

	if((ret = kstrtoul(buf, 10, &value)) < 0)
	{
		SENSOR_LOG_ERROR("kstrtoul failed, ret=0x%x\n", ret);
		return ret;
	}
	ps_data->stk_max_min_diff = (int) value;
	return size;
}


static ssize_t stk_ps_maxdiff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%d\n", ps_data->stk_max_min_diff);
}

static ssize_t stk_ps_ltnct_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
    unsigned long value = 0;
	int ret;

	if((ret = kstrtoul(buf, 10, &value)) < 0)
	{
		SENSOR_LOG_ERROR("kstrtoul failed, ret=0x%x\n", ret);
		return ret;
	}
	ps_data->stk_lt_n_ct = (int) value;
	return size;
}

static ssize_t stk_ps_ltnct_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%d\n", ps_data->stk_lt_n_ct);
}

static ssize_t stk_ps_htnct_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
    unsigned long value = 0;
	int ret;

	if((ret = kstrtoul(buf, 10, &value)) < 0)
	{
		SENSOR_LOG_ERROR("kstrtoul failed, ret=0x%x\n", ret);
		return ret;
	}
	ps_data->stk_ht_n_ct = (int) value;
	return size;
}

static ssize_t stk_ps_htnct_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%d\n", ps_data->stk_ht_n_ct);
}
#endif	/* #ifdef STK_TUNE0 */


/* Nubia add for ps calibration */
#ifdef STK_PS_CALIBRATION
static int32_t stk3x3x_ps_calibration_uncover(struct stk3x3x_data *ps_data){
    //step01: cali crosstalk
    uint32_t reading, ps_sum = 0, counter = 0;
#if 0
    if(ps_data->ps_enabled == false){
        //enable ps
        ret = stk3x3x_get_state(ps_data);
        if(ret < 0)
            return ret;
        w_state_reg = ret;

        SENSOR_LOG_INFO("cur state=0x%x\n", w_state_reg);
        w_state_reg &= ~(STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK | STK_STATE_EN_AK_MASK);
        w_state_reg |= STK_STATE_EN_PS_MASK;
        w_state_reg |= STK_STATE_EN_WAIT_MASK;

        ret = stk3x3x_set_state(ps_data, w_state_reg);
        if(ret < 0)
            return ret;
    }
#endif
    while (counter < PS_CALI_COUNT)
    {
        msleep(60);
        reading = stk3x3x_get_ps_reading(ps_data);
        SENSOR_LOG_INFO("ps %d = %d\n", counter, reading);
        ps_sum += reading;
        counter++;
    }

    ps_data->cali_crosstalk = (ps_sum / PS_CALI_COUNT);
	
	SENSOR_LOG_ERROR("prox crosstalk = %d\n", ps_data->cali_crosstalk);
	return 0;
}

static int32_t stk3x3x_ps_calibration_thres(struct stk3x3x_data *ps_data){
    //step01: cali near
    uint32_t reading, ps_sum = 0, counter = 0;
    uint16_t old_ct = 0;
	int rc;	
	uint16_t prox_buf[2] = {0};

    while (counter < PS_CALI_COUNT)
    {
        msleep(60);
        reading = stk3x3x_get_ps_reading(ps_data);
        SENSOR_LOG_INFO("ps %d = %d\n", counter, reading);
        ps_sum += reading;
        counter++;
    }

    ps_data->cali_near = (ps_sum / PS_CALI_COUNT);
    ps_data->cali_near_increment = ps_data->cali_near - ps_data->cali_crosstalk;

	if(ps_data->cali_near_increment < PS_THRESH_INCREMENT_MIN)
	{
		SENSOR_LOG_ERROR("crosstalk = %d, near = %d, increment = %d\n", ps_data->cali_crosstalk, ps_data->cali_near, ps_data->cali_near_increment);
		return -2;
	}

    old_ct = ps_data->ps_thd_h - ps_data->stk_ht_n_ct;

    //update ht, lt after calibration
    ps_data->stk_ht_n_ct = ps_data->cali_near_increment;
    ps_data->stk_lt_n_ct = ps_data->stk_ht_n_ct - PS_FAR_DIFFER_VALUE;

    ps_data->ps_thd_h = old_ct + ps_data->stk_ht_n_ct;
    ps_data->ps_thd_l = old_ct + ps_data->stk_lt_n_ct;

    stk3x3x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
    stk3x3x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);

    SENSOR_LOG_INFO("HT=%d, LT=%d\n", ps_data->ps_thd_h, ps_data->ps_thd_l);

	prox_buf[0] = ps_data->cali_near_increment;
	rc = sensor_write_file(PS_CAL_FILE_PATH, (char *)prox_buf, sizeof(prox_buf));
	if (rc < 0) {
		SENSOR_LOG_ERROR("PS CAL write file fail, rc = %d\n", rc);
		return -1;
	}
	
	SENSOR_LOG_INFO("stk_ht_n_ct:%d, stk_lt_n_ct:%d \n",
		 ps_data->stk_ht_n_ct, ps_data->stk_lt_n_ct);
	return 0;

}

static ssize_t stk_prox_chip_id_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", DEVICE_CHIP_NAME);
}

static ssize_t stk_prox_debug_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", ps_data->prox_debug);
}


static int stk_prox_set_debug_mode(struct stk3x3x_data *ps_data, uint8_t enable)
{
	int reading;
	uint8_t org_flag_reg;

	if(ps_data->prox_debug == enable)
	{
		SENSOR_LOG_INFO(" prox_debug = %d, debug_enable = %d\n", ps_data->prox_debug, enable);
		return 0;
	}

	if (enable) {
		ps_data->prox_debug = true;
		ps_data->debug_cnt = 0;
//		disable_irq(ps_data->irq);
		if(ps_data->ps_cali_timer_run == false)
		{
			ps_data->ps_cali_timer_run = true;
			hrtimer_start(&ps_data->ps_cali_timer, ps_data->ps_cali_delay, HRTIMER_MODE_REL);
		}
		SENSOR_LOG_INFO("debuging...\n");

		if(ps_data->ps_enabled)
		{
			org_flag_reg = stk3x3x_get_flag(ps_data);
			if(org_flag_reg < 0)
				return -EINVAL;

			if(!(org_flag_reg&STK_FLG_PSDR_MASK))
			{
				//SENSOR_LOG_INFO(" ps is not ready\n");
				return -EINVAL;
			}
		
			reading = stk3x3x_get_ps_reading(ps_data);
			if(reading >= 0)
			{
				input_report_rel(ps_data->ps_input_dev, REL_RZ, reading);
				input_sync(ps_data->ps_input_dev);
			}

		}
	}
	else
	{
		ps_data->prox_debug = false;
		if(ps_data->ps_cali_timer_run == true)
		{
			ps_data->ps_cali_timer_run = false;
			hrtimer_cancel(&ps_data->ps_cali_timer);
			cancel_work_sync(&ps_data->stk_ps_cali_work);
		}
//		enable_irq(ps_data->irq);
	}

	return 0;
}

static ssize_t stk_prox_debug_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
	uint8_t en;
	int ret = 0;
	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else
	{
		SENSOR_LOG_ERROR(", invalid value %d\n", *buf);
		return -EINVAL;
	}
    SENSOR_LOG_INFO(" Enable PS Debug: %d\n", en);
    mutex_lock(&ps_data->io_lock);
    ret = stk_prox_set_debug_mode(ps_data, en);
    mutex_unlock(&ps_data->io_lock);
	if(ret < 0)
	{
		SENSOR_LOG_INFO(", [debug mode]ps is not ready!\n");
	}
    return size;
}

static ssize_t stk_prox_thres_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
	ret = stk3x3x_ps_calibration_thres(ps_data);

	if(ret < 0)
	{
		return sprintf(buf, "%d\n", ret);
	}
	else
	{
		return sprintf(buf, "%d\n", ps_data->cali_near);
	}
}

static ssize_t stk_prox_thres_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int val;
	int rc;
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
	rc = kstrtoint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val) {
		rc = stk3x3x_ps_calibration_thres(ps_data);
	}
	return rc;
}

static ssize_t stk_prox_uncover_cal_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{

	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
	//SENSOR_LOG_INFO("enter\n");
	//chip->params.prox_raw = tmd2725_mean_prox_calc(chip);
	//SENSOR_LOG_ERROR("chip->params.prox_raw=%d", chip->params.prox_raw);
	stk3x3x_ps_calibration_uncover(ps_data);
	return sprintf(buf, "%d\n", ps_data->cali_crosstalk);
}
static ssize_t stk_prox_uncover_cal_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int val;
	int rc;
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
	//SENSOR_LOG_INFO("enter\n");
	rc = kstrtoint(buf, 0, &val);
	if (val) {
		/*@ not use, uncover_cal_show will called */
		rc = stk3x3x_ps_calibration_uncover(ps_data);
		if (rc < 0) {
			//SENSOR_LOG_ERROR("*#777# calibrate fail\n");
			return size;
		}
	}
	return size;
}

static ssize_t stk_prox_uncover_data_min_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", PS_UNCOVER_DATA_MIN);
}
static ssize_t stk_prox_uncover_data_max_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", PS_UNCOVER_DATA_MAX);
}
static ssize_t stk_prox_min_thres_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", PS_THRESH_DATA_MIN);
}
static ssize_t stk_prox_max_thres_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", PS_THRESH_DATA_MAX);
}
static ssize_t stk_prox_max_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", PS_DATA_MAX);
}

static ssize_t stk_prox_thres_to_persist_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
#if 0
	int val;
	int rc;	
	uint16_t prox_buf[2] = {0};
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
	rc = kstrtoint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val) {
		prox_buf[0] = ps_data->cali_near_increment;
		rc = sensor_write_file(PS_CAL_FILE_PATH, (char *)prox_buf, sizeof(prox_buf));
		if (rc < 0) {
			SENSOR_LOG_ERROR("write file fail\n");
			return rc;
		}
		
		SENSOR_LOG_INFO("stk_ht_n_ct:%d, stk_lt_n_ct:%d \n",
			 ps_data->stk_ht_n_ct, ps_data->stk_lt_n_ct);
	}
	return rc;
#else
	return 0;
#endif
}


static void stk_ps_cali_work_func(struct work_struct *work)
{
	struct stk3x3x_data *ps_data = container_of(work, struct stk3x3x_data, stk_ps_cali_work);
	uint32_t reading;
	uint8_t org_flag_reg;
	ps_data->debug_cnt++;

	if(ps_data->ps_enabled)
	{
		org_flag_reg = stk3x3x_get_flag(ps_data);
		if(org_flag_reg < 0)
			return;

		if(!(org_flag_reg&STK_FLG_PSDR_MASK))
		{
			SENSOR_LOG_ERROR(" ps is not ready\n");
			return;
		}

		reading = stk3x3x_get_ps_reading(ps_data);
		if(ps_data->prox_debug)
		{
			if(ps_data->debug_cnt %10 == 9)
				SENSOR_LOG_INFO(" ps debug value: %d\n", reading);
			input_report_rel(ps_data->ps_input_dev, REL_RZ, reading);
			input_sync(ps_data->ps_input_dev);
		}
	}

	return;
}

static enum hrtimer_restart stk_ps_cali_timer_func(struct hrtimer *timer)
{
	struct stk3x3x_data *ps_data = container_of(timer, struct stk3x3x_data, ps_cali_timer);
	queue_work(ps_data->stk_ps_cali_wq, &ps_data->stk_ps_cali_work);
	hrtimer_forward_now(&ps_data->ps_cali_timer, ps_data->ps_cali_delay);
	return HRTIMER_RESTART;
}

#endif
/* Nubia add end */
static struct device_attribute ps_enable_attribute = __ATTR(enable,0664,stk_ps_enable_show,stk_ps_enable_store);
static struct device_attribute ps_enable_aso_attribute = __ATTR(enableaso,0664,stk_ps_enable_aso_show,stk_ps_enable_aso_store);
static struct device_attribute ps_distance_attribute = __ATTR(distance,0664,stk_ps_distance_show, stk_ps_distance_store);
static struct device_attribute ps_offset_attribute = __ATTR(offset,0664,stk_ps_offset_show, stk_ps_offset_store);
static struct device_attribute ps_code_attribute = __ATTR(code, 0444, stk_ps_code_show, NULL);
static struct device_attribute ps_code_thd_l_attribute = __ATTR(codethdl,0664,stk_ps_code_thd_l_show,stk_ps_code_thd_l_store);
static struct device_attribute ps_code_thd_h_attribute = __ATTR(codethdh,0664,stk_ps_code_thd_h_show,stk_ps_code_thd_h_store);
static struct device_attribute ps_recv_attribute = __ATTR(recv,0664,stk_recv_show,stk_recv_store);
static struct device_attribute ps_send_attribute = __ATTR(send,0664,stk_send_show, stk_send_store);
static struct device_attribute all_reg_attribute = __ATTR(allreg, 0444, stk_all_reg_show, NULL);
static struct device_attribute status_attribute = __ATTR(status, 0444, stk_status_show, NULL);
#ifdef STK_TUNE0
static struct device_attribute ps_cali_attribute = __ATTR(cali,0444,stk_ps_cali_show, NULL);
static struct device_attribute ps_maxdiff_attribute = __ATTR(maxdiff,0664,stk_ps_maxdiff_show, stk_ps_maxdiff_store);
static struct device_attribute ps_ltnct_attribute = __ATTR(ltnct,0664,stk_ps_ltnct_show, stk_ps_ltnct_store);
static struct device_attribute ps_htnct_attribute = __ATTR(htnct,0664,stk_ps_htnct_show, stk_ps_htnct_store);
#endif

static struct attribute *stk_ps_attrs [] =
{
    &ps_enable_attribute.attr,
    &ps_enable_aso_attribute.attr,
    &ps_distance_attribute.attr,
	&ps_offset_attribute.attr,
    &ps_code_attribute.attr,
	&ps_code_thd_l_attribute.attr,
	&ps_code_thd_h_attribute.attr,
	&ps_recv_attribute.attr,
	&ps_send_attribute.attr,
	&all_reg_attribute.attr,
	&status_attribute.attr,
#ifdef STK_TUNE0
	&ps_cali_attribute.attr,
	&ps_maxdiff_attribute.attr,
	&ps_ltnct_attribute.attr,
	&ps_htnct_attribute.attr,
#endif
    NULL
};

static struct attribute_group stk_ps_attribute_group = {
#ifndef QUALCOMM_PLATFORM
	.name = "driver",
#endif
	.attrs = stk_ps_attrs,
};

struct device_attribute stk_prox_attrs[] = {
	__ATTR(enable,0664,stk_ps_enable_show,stk_ps_enable_store),
	__ATTR(enableaso,0664,stk_ps_enable_aso_show,stk_ps_enable_aso_store),
	__ATTR(distance,0664,stk_ps_distance_show, stk_ps_distance_store),
	__ATTR(offset,0664,stk_ps_offset_show, stk_ps_offset_store),
	__ATTR(code, 0444, stk_ps_code_show, NULL),
	__ATTR(codethdl,0664,stk_ps_code_thd_l_show,stk_ps_code_thd_l_store),
	__ATTR(codethdh,0664,stk_ps_code_thd_h_show,stk_ps_code_thd_h_store),
	__ATTR(recv,0664,stk_recv_show,stk_recv_store),
	__ATTR(send,0664,stk_send_show, stk_send_store),
	__ATTR(allreg, 0444, stk_all_reg_show, NULL),
	__ATTR(status, 0444, stk_status_show, NULL),
#ifdef STK_TUNE0
	__ATTR(cali,0444,stk_ps_cali_show, NULL),
	__ATTR(maxdiff,0664,stk_ps_maxdiff_show, stk_ps_maxdiff_store),
	__ATTR(ltnct,0664,stk_ps_ltnct_show, stk_ps_ltnct_store),
	__ATTR(htnct,0664,stk_ps_htnct_show, stk_ps_htnct_store),
#endif

#ifdef STK_PS_CALIBRATION
	__ATTR(prox_init, 0664, stk_prox_dev_init_show, stk_prox_dev_init_store),
	__ATTR(chip_name, 0440, stk_prox_chip_id_show, NULL),
	__ATTR(prox_debug, 0644, stk_prox_debug_show,  stk_prox_debug_store),
	__ATTR(prox_thres, 0644, stk_prox_thres_show, stk_prox_thres_store),
	__ATTR(prox_offset_cal, 0644, stk_prox_uncover_cal_show, stk_prox_uncover_cal_store),
	__ATTR(prox_thres_max,0440, stk_prox_max_thres_show, NULL),
	__ATTR(prox_thres_min, 0440, stk_prox_min_thres_show, NULL),
//	__ATTR(prox_init, 0644, tmd2725_prox_dev_init_show, tmd2725_prox_dev_init_store),
	__ATTR(prox_data_max, 0440, stk_prox_max_data_show, NULL),
	__ATTR(prox_uncover_min, 0440, stk_prox_uncover_data_min_show, NULL),
	__ATTR(prox_uncover_max, 0440, stk_prox_uncover_data_max_show, NULL),
	__ATTR(prox_thres_to_persist, 0220, NULL, stk_prox_thres_to_persist_store),
	__ATTR(prox_offset, 0440, stk_ps_offset_show, NULL),
#endif
};
static struct i2c_driver stk_ps_driver;

static int stk_ps_val(struct stk3x3x_data *ps_data)
{
	int32_t lii = 100;
	unsigned char value[4];
	int ret, i;
	bool out_of_range = false;

	ret = stk3x3x_i2c_read_data(ps_data->client, 0x34, 4, value);
	if(ret < 0)
	{
		SENSOR_LOG_ERROR("fail, ret=0x%x", ret);
		return ret;
	}

	for (i = 0; i < 4; i++)
    {
        if (value[i] >= lii)
        {
            out_of_range = true;
            break;
        }
    }
	if(out_of_range == true)
	{
		SENSOR_LOG_INFO(" bgir:%d, %d, %d, %d\n", value[0], value[1], value[2], value[3]);
		return 0xFFFF;
	}
	return 0;
}

#ifdef STK_TUNE0
static int stk_ps_tune_zero_final(struct stk3x3x_data *ps_data)
{
	int ret;

	ps_data->tune_zero_init_proc = false;
	ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_INT_REG, ps_data->int_reg);
	if (ret < 0)
	{
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}

	ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, 0);
	if (ret < 0)
	{
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}

	if(ps_data->data_count == -1)
	{
		SENSOR_LOG_INFO(" exceed limit\n");
		hrtimer_cancel(&ps_data->ps_tune0_timer);
		return 0;
	}

	ps_data->psa = ps_data->ps_stat_data[0];
	ps_data->psi = ps_data->ps_stat_data[2];

#ifdef CALI_PS_EVERY_TIME
	ps_data->ps_high_thd_boot = ps_data->ps_stat_data[1] + ps_data->stk_ht_n_ct*3;
	ps_data->ps_low_thd_boot = ps_data->ps_stat_data[1] + ps_data->stk_lt_n_ct*3;
	ps_data->ps_thd_h = ps_data->ps_high_thd_boot ;
	ps_data->ps_thd_l = ps_data->ps_low_thd_boot ;
#else
	ps_data->ps_thd_h = ps_data->ps_stat_data[1] + ps_data->stk_ht_n_ct;
	ps_data->ps_thd_l = ps_data->ps_stat_data[1] + ps_data->stk_lt_n_ct;
#endif
	stk3x3x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
	stk3x3x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);
	ps_data->boot_cali = 1;
	SENSOR_LOG_INFO(" set HT=%d,LT=%d\n", ps_data->ps_thd_h,  ps_data->ps_thd_l);
	hrtimer_cancel(&ps_data->ps_tune0_timer);
	return 0;
}

static int32_t stk_tune_zero_get_ps_data(struct stk3x3x_data *ps_data)
{
	uint32_t ps_adc;
	int ret;

	ret = stk_ps_val(ps_data);
	if(ret == 0xFFFF)
	{
		ps_data->data_count = -1;
		stk_ps_tune_zero_final(ps_data);
		return 0;
	}

	ps_adc = stk3x3x_get_ps_reading(ps_data);
	SENSOR_LOG_INFO(" ps_adc #%d=%d\n", ps_data->data_count, ps_adc);
	if(ps_adc < 0)
		return ps_adc;

	ps_data->ps_stat_data[1]  +=  ps_adc;
	if(ps_adc > ps_data->ps_stat_data[0])
		ps_data->ps_stat_data[0] = ps_adc;
	if(ps_adc < ps_data->ps_stat_data[2])
		ps_data->ps_stat_data[2] = ps_adc;
	ps_data->data_count++;

	if(ps_data->data_count == 5)
	{
		ps_data->ps_stat_data[1]  /= ps_data->data_count;
		stk_ps_tune_zero_final(ps_data);
	}

	return 0;
}

#ifndef QUALCOMM_PLATFORM
static int stk_ps_tune_zero_init(struct stk3x3x_data *ps_data)
{
	int32_t ret = 0;
	uint8_t w_state_reg;

#ifdef CALI_EVERY_TIME
	ps_data->ps_high_thd_boot = ps_data->ps_thd_h;
	ps_data->ps_low_thd_boot = ps_data->ps_thd_l;
	if(ps_data->ps_high_thd_boot <= 0)
	{
		ps_data->ps_high_thd_boot = ps_data->stk_ht_n_ct*3;
		ps_data->ps_low_thd_boot = ps_data->stk_lt_n_ct*3;
	}
#endif

	ps_data->psi_set = 0;
	ps_data->ps_stat_data[0] = 0;
	ps_data->ps_stat_data[2] = 9999;
	ps_data->ps_stat_data[1] = 0;
	ps_data->data_count = 0;
	ps_data->boot_cali = 0;
	ps_data->tune_zero_init_proc = true;

	ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_INT_REG, 0);
	if (ret < 0)
	{
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}

	w_state_reg = (STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK);
	ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, w_state_reg);
	if (ret < 0)
	{
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}
	hrtimer_start(&ps_data->ps_tune0_timer, ps_data->ps_tune0_delay, HRTIMER_MODE_REL);

	return 0;
}
#endif

static int stk_ps_tune_zero_func_fae(struct stk3x3x_data *ps_data)
{
	int32_t word_data;
	int ret, diff;
	unsigned char value[2];

#ifdef CALI_PS_EVERY_TIME
	if(!(ps_data->ps_enabled))
#else
	if(ps_data->psi_set || !(ps_data->ps_enabled))
#endif
	{
		return 0;
	}

	ret = stk3x3x_get_flag(ps_data);
	if(ret < 0)
		return ret;
	if(!(ret&STK_FLG_PSDR_MASK))
	{
		//SENSOR_LOG_INFO(" ps data is not ready yet\n");
		return 0;
	}

	ret = stk_ps_val(ps_data);
	if(ret == 0)
	{
		ret = stk3x3x_i2c_read_data(ps_data->client, 0x11, 2, &value[0]);
		if(ret < 0)
		{
			SENSOR_LOG_ERROR("fail, ret=0x%x", ret);
			return ret;
		}
		word_data = (value[0]<<8) | value[1];
		//SENSOR_LOG_INFO(" word_data=%d\n", word_data);

		if(word_data == 0)
		{
			//SENSOR_LOG_ERROR("incorrect word data (0)\n");
			return 0xFFFF;
		}

		if(word_data > ps_data->psa)
		{
			ps_data->psa = word_data;
			SENSOR_LOG_INFO(" update psa: psa=%d,psi=%d\n", ps_data->psa, ps_data->psi);
		}
		if(word_data < ps_data->psi)
		{
			ps_data->psi = word_data;
			SENSOR_LOG_INFO(" update psi: psa=%d,psi=%d\n", ps_data->psa, ps_data->psi);
		}
	}
	diff = ps_data->psa - ps_data->psi;
	if(diff > ps_data->stk_max_min_diff)
	{
		ps_data->psi_set = ps_data->psi;

	#ifdef CALI_PS_EVERY_TIME
		if(((ps_data->psi + ps_data->stk_ht_n_ct) > (ps_data->ps_thd_h + 500)) && (ps_data->ps_thd_h != 0))
		{
		//	ps_data->ps_thd_h = ps_data->ps_thd_h;
		//	ps_data->ps_thd_l = ps_data->ps_thd_l;
			SENSOR_LOG_INFO(" no update thd, HT=%d, LT=%d\n", ps_data->ps_thd_h, ps_data->ps_thd_l);
		}
		else
		{
			ps_data->ps_thd_h = ps_data->psi + ps_data->stk_ht_n_ct;
			ps_data->ps_thd_l = ps_data->psi + ps_data->stk_lt_n_ct;
			SENSOR_LOG_INFO(" update thd, HT=%d, LT=%d\n", ps_data->ps_thd_h, ps_data->ps_thd_l);
		}
	#else
		ps_data->ps_thd_h = ps_data->psi + ps_data->stk_ht_n_ct;
		ps_data->ps_thd_l = ps_data->psi + ps_data->stk_lt_n_ct;
		SENSOR_LOG_INFO(" update thd, HT=%d, LT=%d\n", ps_data->ps_thd_h, ps_data->ps_thd_l);
	#endif

/*
#ifdef CALI_PS_EVERY_TIME
		if(ps_data->ps_thd_h > (ps_data->ps_high_thd_boot))
		{
			ps_data->ps_high_thd_boot = ps_data->ps_thd_h;
			ps_data->ps_low_thd_boot = ps_data->ps_thd_l;
			SENSOR_LOG_INFO(" update boot HT=%d, LT=%d\n", ps_data->ps_high_thd_boot, ps_data->ps_low_thd_boot);
		}
#endif		*/
		stk3x3x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
		stk3x3x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);

		SENSOR_LOG_INFO("FAE tune0 psa-psi(%d) > STK_DIFF found\n", diff);
		hrtimer_cancel(&ps_data->ps_tune0_timer);
	}

	return 0;
}

static void stk_ps_tune0_work_func(struct work_struct *work)
{
	struct stk3x3x_data *ps_data = container_of(work, struct stk3x3x_data, stk_ps_tune0_work);
	if(ps_data->prox_debug)
		return;

	if(ps_data->tune_zero_init_proc)
		stk_tune_zero_get_ps_data(ps_data);
	else
		stk_ps_tune_zero_func_fae(ps_data);
	return;
}

static enum hrtimer_restart stk_ps_tune0_timer_func(struct hrtimer *timer)
{
	struct stk3x3x_data *ps_data = container_of(timer, struct stk3x3x_data, ps_tune0_timer);
	queue_work(ps_data->stk_ps_tune0_wq, &ps_data->stk_ps_tune0_work);
	hrtimer_forward_now(&ps_data->ps_tune0_timer, ps_data->ps_tune0_delay);
	return HRTIMER_RESTART;
}
#endif

#ifdef STK_POLL_PS
static enum hrtimer_restart stk_ps_timer_func(struct hrtimer *timer)
{
	struct stk3x3x_data *ps_data = container_of(timer, struct stk3x3x_data, ps_timer);
	queue_work(ps_data->stk_ps_wq, &ps_data->stk_ps_work);
	hrtimer_forward_now(&ps_data->ps_timer, ps_data->ps_poll_delay);
	return HRTIMER_RESTART;
}

static void stk_ps_poll_work_func(struct work_struct *work)
{
	struct stk3x3x_data *ps_data = container_of(work, struct stk3x3x_data, stk_ps_work);
	uint32_t reading;
	int32_t near_far_state;
    uint8_t org_flag_reg;

	if(ps_data->ps_enabled)
	{
#ifdef STK_TUNE0
		// if(!(ps_data->psi_set))
			// return;
#endif
		org_flag_reg = stk3x3x_get_flag(ps_data);
		if(org_flag_reg < 0)
			return;

		if(!(org_flag_reg&STK_FLG_PSDR_MASK))
		{
			//SENSOR_LOG_INFO(" ps is not ready\n");
			return;
		}

		near_far_state = (org_flag_reg & STK_FLG_NF_MASK)?1:0;
		reading = stk3x3x_get_ps_reading(ps_data);
		ps_data->ps_code_last = reading;

		if(ps_data->ps_distance_last != near_far_state)
		{
			stk_ps_report(ps_data, near_far_state);
			SENSOR_LOG_INFO(" ps input event=%d, ps=%d\n", near_far_state, reading);
		}
	}
}
#endif

#if (!defined(STK_POLL_PS))
#if ((STK_INT_PS_MODE == 0x03) || (STK_INT_PS_MODE	== 0x02))
static void stk_ps_int_handle_int_mode_2_3(struct stk3x3x_data *ps_data)
{
	uint32_t reading;
	int32_t near_far_state;

#if (STK_INT_PS_MODE	== 0x03)
	near_far_state = gpio_get_value(ps_data->int_pin);
#elif	(STK_INT_PS_MODE == 0x02)
	near_far_state = !(gpio_get_value(ps_data->int_pin));
#endif
	reading = stk3x3x_get_ps_reading(ps_data);
	stk_ps_report(ps_data, near_far_state);
	SENSOR_LOG_INFO(" ps input event=%d, ps code=%d\n", near_far_state, reading);
}
#endif

static void stk_ps_int_handle(struct stk3x3x_data *ps_data, uint32_t ps_reading, int32_t nf_state)
{
	stk_ps_report(ps_data, nf_state);
	SENSOR_LOG_INFO(" ps input event=%d, ps code=%d\n", nf_state, ps_reading);
}

static void stk_work_func(struct work_struct *work)
{
    int32_t ret;
	int32_t near_far_state;

#if ((STK_INT_PS_MODE != 0x03) && (STK_INT_PS_MODE != 0x02))
    uint8_t disable_flag = 0;
    int32_t org_flag_reg;
#endif	/* #if ((STK_INT_PS_MODE != 0x03) && (STK_INT_PS_MODE != 0x02)) */
	struct stk3x3x_data *ps_data = container_of(work, struct stk3x3x_data, stk_work);
	uint32_t reading;

#if ((STK_INT_PS_MODE == 0x03) || (STK_INT_PS_MODE	== 0x02))
	stk_ps_int_handle_int_mode_2_3(ps_data);
#else
	/* mode 0x01 or 0x04 */
	org_flag_reg = stk3x3x_get_flag(ps_data);
	if(org_flag_reg < 0)
		goto err_i2c_rw;

#ifdef STK_DEBUG_PRINTF
	SENSOR_LOG_INFO(" flag=0x%x\n", org_flag_reg);
#endif

    if (org_flag_reg & STK_FLG_PSINT_MASK)
    {
		disable_flag |= STK_FLG_PSINT_MASK;
		if(ps_data->prox_debug == false)
		{
			reading = stk3x3x_get_ps_reading(ps_data);
			if(reading < 0)
				goto err_i2c_rw;
			ps_data->ps_code_last = reading;

			near_far_state = (org_flag_reg & STK_FLG_NF_MASK)?1:0;
				stk_ps_int_handle(ps_data, reading, near_far_state);
		}
	}

	if(disable_flag)
	{
		ret = stk3x3x_set_flag(ps_data, org_flag_reg, disable_flag);
		if(ret < 0)
			goto err_i2c_rw;
	}

#endif
	usleep_range(1000, 2000);
    enable_irq(ps_data->irq);
	return;

err_i2c_rw:
	msleep(30);
	enable_irq(ps_data->irq);
	return;
}

static irqreturn_t stk_oss_irq_handler(int irq, void *data)
{
	struct stk3x3x_data *pData = data;
	disable_irq_nosync(irq);
	queue_work(pData->stk_wq,&pData->stk_work);
	return IRQ_HANDLED;
}
#endif	/*	#if (!defined(STK_POLL_PS) || !defined(STK_POLL_ALS))	*/

static int32_t stk3x3x_init_all_setting(struct i2c_client *client, struct stk3x3x_platform_data *plat_data)
{
	int32_t ret;
	struct stk3x3x_data *ps_data = i2c_get_clientdata(client);

	ret = stk3x3x_software_reset(ps_data);
	if(ret < 0)
		return ret;

	ret = stk3x3x_check_pid(ps_data);
	if(ret < 0)
		return ret;
	stk3x3x_proc_plat_data(ps_data, plat_data);
	ret = stk3x3x_init_all_reg(ps_data);
	if(ret < 0)
		return ret;

	ps_data->ps_enabled = false;
	ps_data->re_enable_ps = false;
	ps_data->first_boot = true;

#ifdef STK_TUNE0
	#ifdef QUALCOMM_PLATFORM
	ps_data->tune_zero_init_proc = false;
	ps_data->psi_set = 0;
	#else
	stk_ps_tune_zero_init(ps_data);
	#endif
#endif
	atomic_set(&ps_data->recv_reg, 0);
	ps_data->ps_distance_last = -1;
    return 0;
}

#if (!defined(STK_POLL_PS))
struct pinctrl_state *stk_pins_active;
struct pinctrl *stk_pinctrl;
static int stk_pinctrl_active(struct device *dev)
{
    int err;
    stk_pinctrl = devm_pinctrl_get(dev);
    if (IS_ERR_OR_NULL(stk_pinctrl)) {
        dev_err(dev, "Failed to get pin ctrl\n");
        return PTR_ERR(stk_pinctrl);
    }
    stk_pins_active = pinctrl_lookup_state(stk_pinctrl,
                "stk_irq_active");
    if (IS_ERR_OR_NULL(stk_pins_active)) {
        dev_err(dev, "Failed to lookup stk_pinctrl default state\n");
        return PTR_ERR(stk_pins_active);
    }

    err = pinctrl_select_state(stk_pinctrl, stk_pins_active);
    return 0;
}

static int stk3x3x_setup_irq(struct i2c_client *client)
{
	int irq, err = -EIO;
	struct stk3x3x_data *ps_data = i2c_get_clientdata(client);

	stk_pinctrl_active(&client->dev);

#ifdef SPREADTRUM_PLATFORM
	irq = sprd_alloc_gpio_irq(ps_data->int_pin);
#else
	irq = gpio_to_irq(ps_data->int_pin);
#endif
#ifdef STK_DEBUG_PRINTF
	SENSOR_LOG_INFO(" int pin #=%d, irq=%d\n", ps_data->int_pin, irq);
#endif
	if (irq <= 0)
	{
		SENSOR_LOG_ERROR("irq number is not specified, irq # = %d, int pin=%d\n",irq, ps_data->int_pin);
		return irq;
	}
	ps_data->irq = irq;
	err = gpio_request(ps_data->int_pin,"stk-int");
	if(err < 0)
	{
		SENSOR_LOG_ERROR("gpio_request, err=%d", err);
		return err;
	}
	// gpio_tlmm_config(GPIO_CFG(ps_data->int_pin, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE);

	err = gpio_direction_input(ps_data->int_pin);
	if(err < 0)
	{
		SENSOR_LOG_ERROR("gpio_direction_input, err=%d", err);
		return err;
	}
#if ((STK_INT_PS_MODE == 0x03) || (STK_INT_PS_MODE	== 0x02))
	err = request_any_context_irq(irq, stk_oss_irq_handler, IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, DEVICE_NAME, ps_data);
#else
	err = request_any_context_irq(irq, stk_oss_irq_handler, IRQF_TRIGGER_LOW, DEVICE_NAME, ps_data);
#endif
	if (err < 0)
	{
		SENSOR_LOG_DEBUG("request_any_context_irq(%d) failed for (%d)\n", irq, err);
		goto err_request_any_context_irq;
	}
	disable_irq(irq);

	return 0;
err_request_any_context_irq:
#ifdef SPREADTRUM_PLATFORM
	sprd_free_gpio_irq(ps_data->int_pin);
#else
	gpio_free(ps_data->int_pin);
#endif
	return err;
}
#endif


static int stk3x3x_suspend(struct device *dev)
{
	struct stk3x3x_data *ps_data = dev_get_drvdata(dev);
#if (defined(STK_CHK_REG) || !defined(STK_POLL_PS))
	int err;
#endif

#ifndef STK_POLL_PS
    struct i2c_client *client = to_i2c_client(dev);
#endif

	SENSOR_LOG_INFO("enter\n");
#ifndef SPREADTRUM_PLATFORM
	mutex_lock(&ps_data->io_lock);
#endif
#ifdef STK_CHK_REG
	err = stk3x3x_validate_n_handle(ps_data->client);
	if(err < 0)
	{
		SENSOR_LOG_ERROR("stk3x3x_validate_n_handle fail: %d\n", err);
	}
	else if (err == 0xFF)
	{
		if(ps_data->ps_enabled)
			stk3x3x_enable_ps(ps_data, 1, 0);
	}
#endif /* #ifdef STK_CHK_REG */
	if(ps_data->ps_enabled)
	{
#ifdef STK_POLL_PS
		wake_lock(&ps_data->ps_nosuspend_wl);
#else
		if(device_may_wakeup(&client->dev))
		{
			err = enable_irq_wake(ps_data->irq);
			if (err)
				SENSOR_LOG_DEBUG("set_irq_wake(%d) failed, err=(%d)\n", ps_data->irq, err);
		}
		else
		{
			SENSOR_LOG_ERROR("not support wakeup source\n");
		}
#endif
	}
#ifndef SPREADTRUM_PLATFORM
	mutex_unlock(&ps_data->io_lock);
#endif
	return 0;
}

static int stk3x3x_resume(struct device *dev)
{
	struct stk3x3x_data *ps_data = dev_get_drvdata(dev);
#if (defined(STK_CHK_REG) || !defined(STK_POLL_PS))
	int err;
#endif
#ifndef STK_POLL_PS
    struct i2c_client *client = to_i2c_client(dev);
#endif

	SENSOR_LOG_INFO("enter\n");
#ifndef SPREADTRUM_PLATFORM
	mutex_lock(&ps_data->io_lock);
#endif
#ifdef STK_CHK_REG
	err = stk3x3x_validate_n_handle(ps_data->client);
	if(err < 0)
	{
		SENSOR_LOG_ERROR("stk3x3x_validate_n_handle fail: %d\n", err);
	}
	else if (err == 0xFF)
	{
		if(ps_data->ps_enabled)
			stk3x3x_enable_ps(ps_data, 1, 0);
	}
#endif /* #ifdef STK_CHK_REG */

	if(ps_data->ps_enabled)
	{
#ifdef STK_POLL_PS
		wake_unlock(&ps_data->ps_nosuspend_wl);
#else
		if(device_may_wakeup(&client->dev))
		{
			err = disable_irq_wake(ps_data->irq);
			if (err)
				SENSOR_LOG_DEBUG("disable_irq_wake(%d) failed, err=(%d)\n", ps_data->irq, err);
		}
#endif
	}
#ifndef SPREADTRUM_PLATFORM
	mutex_unlock(&ps_data->io_lock);
#endif
	return 0;
}

static const struct dev_pm_ops stk3x3x_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(stk3x3x_suspend, stk3x3x_resume)
};

#ifdef STK_QUALCOMM_POWER_CTRL
static int stk3x3x_power_ctl(struct stk3x3x_data *data, bool on)
{
	int ret = 0;

	if (!on && data->power_enabled) {
		ret = regulator_disable(data->vdd);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_disable(data->vio);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vio disable failed ret=%d\n", ret);
			ret = regulator_enable(data->vdd);
			if (ret) {
				dev_err(&data->client->dev,
					"Regulator vdd enable failed ret=%d\n",
					ret);
			}
			return ret;
		}
		data->power_enabled = on;
		SENSOR_LOG_INFO(" disable stk3x3x power");
		dev_dbg(&data->client->dev, "stk3x3x_power_ctl on=%d\n",
				on);
	} else if (on && !data->power_enabled) {
		ret = regulator_set_mode(data->vdd,REGULATOR_MODE_NORMAL);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vdd set mode failed ret=%d\n", ret);
			return ret;
		}
		ret = regulator_enable(data->vdd);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_enable(data->vio);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vio enable failed ret=%d\n", ret);
			regulator_disable(data->vdd);
			return ret;
		}
		data->power_enabled = on;
		SENSOR_LOG_INFO(" enable stk3x3x power");
		dev_dbg(&data->client->dev, "stk3x3x_power_ctl on=%d\n",
				on);
	} else {
		dev_warn(&data->client->dev,
				"Power on=%d. enabled=%d\n",
				on, data->power_enabled);
	}

	return ret;
}

static int stk3x3x_power_init(struct stk3x3x_data *data, bool on)
{
	int ret;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd,
					0, STK3X3X_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio,
					0, STK3X3X_VIO_MAX_UV);

		regulator_put(data->vio);
	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			ret = PTR_ERR(data->vdd);
			dev_err(&data->client->dev,
				"Regulator get failed vdd ret=%d\n", ret);
			return ret;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			ret = regulator_set_voltage(data->vdd,
					STK3X3X_VDD_MIN_UV,
					STK3X3X_VDD_MAX_UV);
			if (ret) {
				dev_err(&data->client->dev,
					"Regulator set failed vdd ret=%d\n",
					ret);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			ret = PTR_ERR(data->vio);
			dev_err(&data->client->dev,
				"Regulator get failed vio ret=%d\n", ret);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			ret = regulator_set_voltage(data->vio,
					STK3X3X_VIO_MIN_UV,
					STK3X3X_VIO_MAX_UV);
			if (ret) {
				dev_err(&data->client->dev,
				"Regulator set failed vio ret=%d\n", ret);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, STK3X3X_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return ret;
}

static int stk3x3x_device_ctl(struct stk3x3x_data *ps_data, bool enable)
{
	int ret;
	struct device *dev = &ps_data->client->dev;

	if (enable && !ps_data->power_enabled) {
		ret = stk3x3x_power_ctl(ps_data, true);
		if (ret) {
			dev_err(dev, "Failed to enable device power\n");
			goto err_exit;
		}
		/*it will be init setting when probe*/
		//ret = stk3x3x_init_all_setting(ps_data->client, ps_data->pdata);
		//if (ret < 0) {
		//	stk3x3x_power_ctl(ps_data, false);
		//	dev_err(dev, "Failed to re-init device setting\n");
		//	goto err_exit;
		//}
	} else if (!enable && ps_data->power_enabled) {

		if (!ps_data->ps_enabled) {
			ret = stk3x3x_power_ctl(ps_data, false);
			if (ret) {
				dev_err(dev, "Failed to disable device power\n");
				goto err_exit;
			}
		} else {
			dev_dbg(dev, "device control: als_enabled=%d, ps_enabled=%d\n",
				0, ps_data->ps_enabled);
		}
	} else {
		dev_dbg(dev, "device control: enable=%d, power_enabled=%d\n",
			enable, ps_data->power_enabled);
	}
	return 0;

err_exit:
	return ret;
}
#endif	/* #ifdef STK_QUALCOMM_POWER_CTRL */

#ifdef CONFIG_OF
static int stk3x3x_parse_dt(struct device *dev,
			struct stk3x3x_platform_data *pdata)
{
#if 1
	int rc;
	struct device_node *np = dev->of_node;
	u32 temp_val;

	pdata->int_pin = of_get_named_gpio_flags(np, "stk,irq-gpio",
				0, &pdata->int_flags);
	if (pdata->int_pin < 0) {
		dev_err(dev, "Unable to read irq-gpio\n");
		return pdata->int_pin;
	}

	rc = of_property_read_u32(np, "stk,state-reg", &temp_val);
	if (!rc)
		pdata->state_reg = temp_val;
	else {
		dev_err(dev, "Unable to read state-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,psctrl-reg", &temp_val);
	if (!rc)
		pdata->psctrl_reg = (u8)temp_val;
	else {
		dev_err(dev, "Unable to read psctrl-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,ledctrl-reg", &temp_val);
	if (!rc)
		pdata->ledctrl_reg = (u8)temp_val;
	else {
		dev_err(dev, "Unable to read ledctrl-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,wait-reg", &temp_val);
	if (!rc)
		pdata->wait_reg = (u8)temp_val;
	else {
		dev_err(dev, "Unable to read wait-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,ps-thdh", &temp_val);
	if (!rc)
		pdata->ps_thd_h = (u16)temp_val;
	else {
		dev_err(dev, "Unable to read ps-thdh\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,ps-thdl", &temp_val);
	if (!rc)
		pdata->ps_thd_l = (u16)temp_val;
	else {
		dev_err(dev, "Unable to read ps-thdl\n");
		return rc;
	}

	//pdata->use_fir = of_property_read_bool(np, "stk,use-fir");
#endif
	return 0;
}
#else
static int stk3x3x_parse_dt(struct device *dev,
			struct stk3x3x_platform_data *pdata)
{
	return -ENODEV;
}
#endif /* !CONFIG_OF */

static int stk3x3x_set_wq(struct stk3x3x_data *ps_data)
{
#ifdef STK_POLL_PS
	ps_data->stk_ps_wq = create_singlethread_workqueue("stk_ps_wq");
	INIT_WORK(&ps_data->stk_ps_work, stk_ps_poll_work_func);
	hrtimer_init(&ps_data->ps_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ps_data->ps_poll_delay = ns_to_ktime(60 * NSEC_PER_MSEC);
	ps_data->ps_timer.function = stk_ps_timer_func;
#endif

#ifdef STK_TUNE0
	ps_data->stk_ps_tune0_wq = create_singlethread_workqueue("stk_ps_tune0_wq");
	INIT_WORK(&ps_data->stk_ps_tune0_work, stk_ps_tune0_work_func);
	hrtimer_init(&ps_data->ps_tune0_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ps_data->ps_tune0_delay = ns_to_ktime(60 * NSEC_PER_MSEC);
	ps_data->ps_tune0_timer.function = stk_ps_tune0_timer_func;
#endif

#ifdef STK_PS_CALIBRATION
	ps_data->stk_ps_cali_wq = create_singlethread_workqueue("stk_ps_cali_wq");
	INIT_WORK(&ps_data->stk_ps_cali_work, stk_ps_cali_work_func);
	hrtimer_init(&ps_data->ps_cali_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ps_data->ps_cali_delay = ns_to_ktime(180 * NSEC_PER_MSEC);
	ps_data->ps_cali_timer.function = stk_ps_cali_timer_func;
	ps_data->ps_cali_timer_run = false;
#endif

#if (!defined(STK_POLL_PS))
	ps_data->stk_wq = create_singlethread_workqueue("stk_wq");
	INIT_WORK(&ps_data->stk_work, stk_work_func);
#endif
	return 0;
}


static int stk3x3x_set_input_devices(struct stk3x3x_data *ps_data)
{
	int err;

	ps_data->ps_input_dev = input_allocate_device();
	if (ps_data->ps_input_dev==NULL)
	{
		SENSOR_LOG_ERROR("could not allocate ps device\n");
		err = -ENOMEM;
		goto err_ps_input_allocate;
	}

	ps_data->ps_input_dev->name = PS_NAME;

	//set_bit(EV_ABS, ps_data->ps_input_dev->evbit);
	set_bit(EV_REL, ps_data->ps_input_dev->evbit);
	set_bit(REL_RZ,  ps_data->ps_input_dev->relbit);
	set_bit(REL_MISC, ps_data->ps_input_dev->relbit);

	//input_set_abs_params(ps_data->ps_input_dev, ABS_DISTANCE, 0,1, 0, 0);
	err = input_register_device(ps_data->ps_input_dev);
	if (err<0)
	{
		SENSOR_LOG_ERROR("can not register ps input device\n");
		goto err_ps_input_register;
	}

	err = sysfs_create_group(&ps_data->ps_input_dev->dev.kobj, &stk_ps_attribute_group);
	if (err < 0)
	{
		SENSOR_LOG_ERROR("could not create sysfs group for ps\n");
		goto err_ps_create_group;
	}
	input_set_drvdata(ps_data->ps_input_dev, ps_data);

	/* create sysfs */
	stk_ps_class = class_create(THIS_MODULE, DEV_PS_NAME);
	alloc_chrdev_region(&stk_ps_dev_t, 0, 1, DEV_PS_NAME);
	ps_data->ps_dev = device_create(stk_ps_class, 0, stk_ps_dev_t, &stk_ps_driver, DEV_PS_NAME);
	if (IS_ERR_OR_NULL(ps_data->ps_dev)) {
		SENSOR_LOG_ERROR("ps device create fail\n");
		err = -ENOMEM;
		goto exit_create_ps_dev;
	}

	err = sensor_create_sysfs_interfaces(ps_data->ps_dev, stk_prox_attrs, ARRAY_SIZE(stk_prox_attrs));
	if (err < 0) {
		SENSOR_LOG_ERROR("ps create sysfs interfaces fail\n");
		err = -ENOMEM;
		goto err_create_ps_sysfs;
	}
	dev_set_drvdata(ps_data->ps_dev, ps_data);

	return 0;

err_create_ps_sysfs:
	device_destroy(stk_ps_class, stk_ps_dev_t);
	class_destroy(stk_ps_class);
exit_create_ps_dev:
	sysfs_remove_group(&ps_data->ps_input_dev->dev.kobj, &stk_ps_attribute_group);
err_ps_create_group:
	input_unregister_device(ps_data->ps_input_dev);
	return err;
err_ps_input_register:
	input_free_device(ps_data->ps_input_dev);
	return err;
err_ps_input_allocate:
	return err;
}

static int stk3x3x_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
    int err = -ENODEV;
    struct stk3x3x_data *ps_data;
	struct stk3x3x_platform_data *plat_data;
    SENSOR_LOG_INFO(" driver version = %s\n", DRIVER_VERSION);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        SENSOR_LOG_ERROR("No Support for I2C_FUNC_I2C\n");
        return -ENODEV;
    }

	ps_data = kzalloc(sizeof(struct stk3x3x_data),GFP_KERNEL);
	if(!ps_data)
	{
		SENSOR_LOG_ERROR("failed to allocate stk3x3x_data\n");
		return -ENOMEM;
	}
	ps_data->client = client;
	i2c_set_clientdata(client,ps_data);
	mutex_init(&ps_data->io_lock);
	wake_lock_init(&ps_data->ps_wakelock,WAKE_LOCK_SUSPEND, "stk_input_wakelock");

#ifdef STK_POLL_PS
	wake_lock_init(&ps_data->ps_nosuspend_wl,WAKE_LOCK_SUSPEND, "stk_nosuspend_wakelock");
#endif

	if (client->dev.of_node) {
		SENSOR_LOG_INFO(" probe with device tree\n");
		plat_data = devm_kzalloc(&client->dev,
			sizeof(struct stk3x3x_platform_data), GFP_KERNEL);
		if (!plat_data) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = stk3x3x_parse_dt(&client->dev, plat_data);
		if (err)
		{
			dev_err(&client->dev,
				"%s: stk3x3x_parse_dt ret=%d\n", __func__, err);
			return err;
		}
	} else {
		SENSOR_LOG_INFO(" probe with platform data\n");
#ifdef SPREADTRUM_PLATFORM
		plat_data = &stk3x3x_pfdata;
#else
		plat_data = client->dev.platform_data;
#endif
	}
	if (!plat_data) {
		dev_err(&client->dev,
			"%s: no stk3x3x platform data!\n", __func__);
		goto err_stk_input_allocate;
	}
	ps_data->int_pin = plat_data->int_pin;
	ps_data->pdata = plat_data;

	stk3x3x_set_wq(ps_data);
#ifdef QUALCOMM_PLATFORM
	ps_data->ps_thd_h = 0;
	ps_data->ps_thd_l = 0;
#endif

#ifdef STK_TUNE0
	ps_data->stk_max_min_diff = STK_MAX_MIN_DIFF;
	ps_data->stk_lt_n_ct = STK_LT_N_CT;
	ps_data->stk_ht_n_ct = STK_HT_N_CT;
#endif

#ifdef STK_QUALCOMM_POWER_CTRL
    SENSOR_LOG_ERROR("STK_QUALCOMM_POWER_CTRL IN\n");
	err = stk3x3x_power_init(ps_data, true);
	if (err)
		goto err_power_on;

	err = stk3x3x_power_ctl(ps_data, true);
	if (err)
		goto err_power_on;
	msleep(3);
	err = stk3x3x_check_pid(ps_data);
	if(err < 0)
		goto err_init_all_setting;

	ps_data->ps_enabled = false;
#endif

	err = stk3x3x_init_all_setting(client, plat_data);
	if(err < 0)
		goto err_init_all_setting;

	err = stk3x3x_set_input_devices(ps_data);
	if(err < 0)
		goto err_setup_input_device;

#if (!defined(STK_POLL_PS))
	err = stk3x3x_setup_irq(client);
	if(err < 0)
		goto err_stk3x3x_setup_irq;
#endif
	device_init_wakeup(&client->dev, true);

#ifdef QUALCOMM_PLATFORM

	ps_data->ps_cdev = sensors_proximity_cdev;
	ps_data->ps_cdev.sensors_enable = stk_ps_enable_set;
	err = sensors_classdev_register(&ps_data->ps_input_dev->dev, &ps_data->ps_cdev);
	if (err)
		goto err_class_sysfs;
#endif

#ifdef STK_QUALCOMM_POWER_CTRL
	/* enable device power only when it is enabled */
	err = stk3x3x_power_ctl(ps_data, false);
	if (err)
		goto err_power_ctl;
#endif

	SENSOR_LOG_INFO(" probe successfully");
	return 0;

	//device_init_wakeup(&client->dev, false);
#ifdef STK_QUALCOMM_POWER_CTRL
err_power_ctl:
#endif
#ifdef QUALCOMM_PLATFORM
	sensors_classdev_unregister(&ps_data->ps_cdev);
err_class_sysfs:
#endif
#if (!defined(STK_POLL_PS))
err_stk3x3x_setup_irq:
	free_irq(ps_data->irq, ps_data);
	#ifdef SPREADTRUM_PLATFORM
		sprd_free_gpio_irq(ps_data->int_pin);
	#else
		gpio_free(ps_data->int_pin);
	#endif
#endif

	sysfs_remove_group(&ps_data->ps_input_dev->dev.kobj, &stk_ps_attribute_group);
	input_unregister_device(ps_data->ps_input_dev);

err_setup_input_device:
err_init_all_setting:
#ifdef STK_QUALCOMM_POWER_CTRL
	stk3x3x_power_ctl(ps_data, false);
err_power_on:
	stk3x3x_power_init(ps_data, false);
#endif

#ifdef STK_TUNE0
	destroy_workqueue(ps_data->stk_ps_tune0_wq);
#endif
#ifdef STK_POLL_PS
	hrtimer_try_to_cancel(&ps_data->ps_timer);
	destroy_workqueue(ps_data->stk_ps_wq);
#endif
#if (!defined(STK_POLL_PS))
	destroy_workqueue(ps_data->stk_wq);
#endif
err_stk_input_allocate:
#ifdef STK_POLL_PS
    wake_lock_destroy(&ps_data->ps_nosuspend_wl);
#endif
    wake_lock_destroy(&ps_data->ps_wakelock);
    mutex_destroy(&ps_data->io_lock);
	kfree(ps_data);
    return err;
}


static int stk3x3x_remove(struct i2c_client *client)
{
	struct stk3x3x_data *ps_data = i2c_get_clientdata(client);

	device_init_wakeup(&client->dev, false);
#ifdef STK_QUALCOMM_POWER_CTRL
	stk3x3x_power_ctl(ps_data, false);
#endif
#ifdef QUALCOMM_PLATFORM
	sensors_classdev_unregister(&ps_data->ps_cdev);
	sensors_classdev_unregister(&ps_data->als_cdev);
#endif
#ifdef STK_QUALCOMM_POWER_CTRL
	stk3x3x_power_init(ps_data, false);
#endif
#if (!defined(STK_POLL_PS))
	free_irq(ps_data->irq, ps_data);
	#ifdef SPREADTRUM_PLATFORM
		sprd_free_gpio_irq(ps_data->int_pin);
	#else
		gpio_free(ps_data->int_pin);
	#endif
#endif	/* #if (!defined(STK_POLL_PS)) */

//	sysfs_remove_group(&ps_data->ps_input_dev->dev.kobj, &stk_ps_attribute_group);
//	input_unregister_device(ps_data->ps_input_dev);

	sensor_remove_sysfs_interfaces(ps_data->ps_dev, stk_prox_attrs, ARRAY_SIZE(stk_prox_attrs));
    device_destroy(stk_ps_class, stk_ps_dev_t);
    class_destroy(stk_ps_class);

#ifdef STK_TUNE0
	destroy_workqueue(ps_data->stk_ps_tune0_wq);
#endif
#ifdef STK_POLL_PS
	hrtimer_try_to_cancel(&ps_data->ps_timer);
	destroy_workqueue(ps_data->stk_ps_wq);
#if (!defined(STK_POLL_PS))
	destroy_workqueue(ps_data->stk_wq);
#endif
	wake_lock_destroy(&ps_data->ps_nosuspend_wl);
#endif
	wake_lock_destroy(&ps_data->ps_wakelock);
    mutex_destroy(&ps_data->io_lock);
	kfree(ps_data);

    return 0;
}

static const struct i2c_device_id stk_ps_id[] =
{
    { "DEVICE_NAME", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, stk_ps_id);

static struct of_device_id stk_match_table[] = {
	{ .compatible = "stk,stk3x3x", },
	{ },
};

static struct i2c_driver stk_ps_driver =
{
    .driver = {
        .name = DEVICE_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = stk_match_table,
#endif
		.pm = &stk3x3x_pm_ops,
    },
    .probe = stk3x3x_probe,
    .remove = stk3x3x_remove,
    .id_table = stk_ps_id,
};


static int __init stk3x3x_init(void)
{
	int ret;
    ret = i2c_add_driver(&stk_ps_driver);
    if (ret)
	{
		i2c_del_driver(&stk_ps_driver);
        return ret;
	}
    return 0;
}

static void __exit stk3x3x_exit(void)
{
    i2c_del_driver(&stk_ps_driver);
}

module_init(stk3x3x_init);
module_exit(stk3x3x_exit);
MODULE_AUTHOR("Lex Hsieh <lex_hsieh@sensortek.com.tw>");
MODULE_DESCRIPTION("Sensortek stk3x3x Proximity Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
