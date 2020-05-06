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
#define CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
#define STK_ALS_CHANGE_THD  3  /* The threshold to trigger ALS interrupt, unit: lux */
#define STK_INT_PS_MODE         1   /* 1, 2, or 3   */
//#define STK_POLL_PS
#define STK_POLL_ALS        /* ALS interrupt is valid only when STK_INT_PS_MODE = 1 or 4*/
#define STK_TUNE0
#define CALI_PS_EVERY_TIME
#define STK_ALS_FIR
// #define STK_IRS
#define STK_ALSPS_CALIBRATION
#define ALS_CALI_COUNT   3

#ifdef QUALCOMM_PLATFORM
#include <linux/sensors.h>
#include <linux/regulator/consumer.h>
//  #define STK_QUALCOMM_POWER_CTRL
#endif

#include "stk3x3x.h"
#include "../sensor_common.h"

#undef DRIVER_VERSION
#define DRIVER_VERSION  "3.11.0"

#undef LOG_TAG
#define LOG_TAG "STK3337"


#define CTTRACKING

/* Nubia add for ps calibration */
#define DEVICE_CHIP_NAME "pa224:stk3337"
#define PS_UNCOVER_DATA_MIN             0
#define PS_UNCOVER_DATA_MAX             1000
#define PS_THRESH_DATA_MIN              60
#define PS_THRESH_DATA_MAX              4000
#define PS_THRESH_INCREMENT_MIN         80
#define PS_THRESH_INCREMENT_MAX         2000
#define PS_DATA_MAX                     35000
/* Nubia add end */

int stk_increment_init_flag = false;

/* Define Register Map */
#define STK_STATE_REG           0x00
#define STK_PSCTRL_REG          0x01
#define STK_ALSCTRL_REG             0x02
#define STK_LEDCTRL_REG             0x03
#define STK_INT_REG                 0x04
#define STK_WAIT_REG            0x05
#define STK_THDH1_PS_REG        0x06
#define STK_THDH2_PS_REG        0x07
#define STK_THDL1_PS_REG        0x08
#define STK_THDL2_PS_REG        0x09
#define STK_THDH1_ALS_REG       0x0A
#define STK_THDH2_ALS_REG       0x0B
#define STK_THDL1_ALS_REG       0x0C
#define STK_THDL2_ALS_REG       0x0D
#define STK_FLAG_REG            0x10
#define STK_DATA1_PS_REG        0x11
#define STK_DATA2_PS_REG        0x12
#define STK_DATA1_ALS_REG       0x13
#define STK_DATA2_ALS_REG       0x14
#define STK_DATA1_OFFSET_REG    0x15
#define STK_DATA2_OFFSET_REG    0x16
#define STK_DATA1_IR_REG        0x17
#define STK_DATA2_IR_REG        0x18
#define STK_PDT_ID_REG          0x3E
#define STK_RSRVD_REG           0x3F
#define STK_SW_RESET_REG        0x80

#define STK_GSCTRL_REG          0x1A
#define STK_FLAG2_REG           0x1C

/* Define state reg */
#define STK_STATE_EN_IRS_SHIFT      7
#define STK_STATE_EN_AK_SHIFT   6
#define STK_STATE_EN_ASO_SHIFT      5
#define STK_STATE_EN_IRO_SHIFT      4
#define STK_STATE_EN_WAIT_SHIFT     2
#define STK_STATE_EN_ALS_SHIFT      1
#define STK_STATE_EN_PS_SHIFT   0

#define STK_STATE_EN_IRS_MASK   0x80
#define STK_STATE_EN_AK_MASK    0x40
#define STK_STATE_EN_ASO_MASK   0x20
#define STK_STATE_EN_IRO_MASK   0x10
#define STK_STATE_EN_WAIT_MASK  0x04
#define STK_STATE_EN_ALS_MASK   0x02
#define STK_STATE_EN_PS_MASK    0x01

/* Define PS ctrl reg */
#define STK_PS_PRS_SHIFT        6
#define STK_PS_GAIN_SHIFT       4
#define STK_PS_IT_SHIFT             0

#define STK_PS_PRS_MASK         0xC0
#define STK_PS_GAIN_MASK            0x30
#define STK_PS_IT_MASK          0x0F

/* Define ALS ctrl reg */
#define STK_ALS_PRS_SHIFT       6
#define STK_ALS_GAIN_SHIFT          4
#define STK_ALS_IT_SHIFT            0

#define STK_ALS_PRS_MASK        0xC0
#define STK_ALS_GAIN_MASK       0x30
#define STK_ALS_IT_MASK         0x0F

/* Define LED ctrl reg */
#define STK_LED_IRDR_SHIFT          6
#define STK_LED_DT_SHIFT        0

#define STK_LED_IRDR_MASK       0xC0
#define STK_LED_DT_MASK         0x3F

/* Define interrupt reg */
#define STK_INT_CTRL_SHIFT          7
#define STK_INT_OUI_SHIFT       4
#define STK_INT_ALS_SHIFT       3
#define STK_INT_PS_SHIFT            0

#define STK_INT_CTRL_MASK       0x80
#define STK_INT_OUI_MASK            0x10
#define STK_INT_ALS_MASK            0x08
#define STK_INT_PS_MASK         0x07

#define STK_INT_ALS             0x08

/* Define flag reg */
#define STK_FLG_ALSDR_SHIFT         7
#define STK_FLG_PSDR_SHIFT          6
#define STK_FLG_ALSINT_SHIFT        5
#define STK_FLG_PSINT_SHIFT         4
#define STK_FLG_OUI_SHIFT       2
#define STK_FLG_IR_RDY_SHIFT        1
#define STK_FLG_NF_SHIFT        0

#define STK_FLG_ALSDR_MASK      0x80
#define STK_FLG_PSDR_MASK       0x40
#define STK_FLG_ALSINT_MASK     0x20
#define STK_FLG_PSINT_MASK      0x10
#define STK_FLG_OUI_MASK            0x04
#define STK_FLG_IR_RDY_MASK     0x02
#define STK_FLG_NF_MASK         0x01

/* Define flag2 reg */
#define STK_FLG2_INT_GS_SHIFT       6
#define STK_FLG2_GS10_SHIFT     5
#define STK_FLG2_GS01_SHIFT     4

#define STK_FLG2_INT_GS_MASK    0x40
#define STK_FLG2_GS10_MASK      0x20
#define STK_FLG2_GS01_MASK      0x10

/* misc define */
#define MIN_ALS_POLL_DELAY_NS   60000000

#define STK_IT_ALS_25MS   0x00
#define STK_IT_ALS_50MS   0x01
#define STK_IT_ALS_100MS  0x02
#define STK_IT_ALS_200MS  0x03
#define STK_IT_ALS_400MS  0x04
#define STK_IT_ALS_800MS  0x05
#define STK_IT_ALS_1600MS 0x06

#define STK_ALS_GAIN_1    0x00 
#define STK_ALS_GAIN_4    0x10 
#define STK_ALS_GAIN_16   0x20 
#define STK_ALS_GAIN_64   0x30

#ifdef STK_TUNE0
#define STK_MAX_MIN_DIFF    160
#define STK_LT_N_CT 350
#define STK_HT_N_CT 400
#endif  /* #ifdef STK_TUNE0 */

#define STK_H_PS        3000
#define STK_H_LT        1300
#define STK_H_HT        1350

#define STK_DEFAULT_LT  8000
#define STK_DEFAULT_HT  8200

#define STK_IRC_MAX_ALS_CODE        20000
#define STK_IRC_MIN_ALS_CODE        25
#define STK_IRC_MIN_IR_CODE     50
#define STK_IRC_ALS_DENOMI      2
#define STK_IRC_ALS_NUMERA      5
#define STK_IRC_ALS_CORREC      850

#define STK_IRS_IT_REDUCE           2
#define STK_ALS_READ_IRS_IT_REDUCE  5
#define STK_ALS_THRESHOLD           30

#define DEVICE_NAME     "stk3337"

#define ALS_NAME "light"
#define ALS_CAL_FILE_PATH        "/persist/sensors/als_cal_data"
#define ALS_FAC_CAL_PATH        "/persist/sensors/als_fac_cal"

#define PS_NAME "proximity"
#define PS_CAL_FILE_PATH      "/persist/sensors/xtalk_cal"

#define PS_CALI_COUNT       5
#define PS_FAR_DIFFER_VALUE   50

#ifdef STK_QUALCOMM_POWER_CTRL
/* POWER SUPPLY VOLTAGE RANGE */
#define STK3X3X_VDD_MIN_UV  2000000
#define STK3X3X_VDD_MAX_UV  3300000
#define STK3X3X_VIO_MIN_UV  1750000
#define STK3X3X_VIO_MAX_UV  1950000
#endif

#define STK3310SA_PID       0x17
#define STK3311SA_PID       0x1E
#define STK3311WV_PID       0x1D
#define STK3311X_PID            0x12
#define STK33119_PID            0x11

#define LIGHT_SLOPE_CWF     1000
#define LIGHT_SLOPE_A       125
#define LIGHT_SLOPE_CLOUDY1 435 
#define LIGHT_SLOPE_CLOUDY2 295 

#define ALS_GAIN_COUNT      15 

#ifdef QUALCOMM_PLATFORM

static struct sensors_classdev sensors_light_cdev = {
	.name = "stk3x3x-light",
	.vendor = "Sensortek",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "6500",
	.resolution = "0.0625",
	.sensor_power = "0.09",
	.min_delay = 0, /* us */
	.max_delay = 0,
	.delay_msec = 200,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.flags = 2,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_calibrate = NULL,
	.sensors_write_cal_params = NULL,
	.params = NULL,
};

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

static struct stk3x3x_platform_data stk3x3x_pfdata= {
	.state_reg = 0x0,    /* disable all */
	.psctrl_reg = 0x31,    /* ps_persistance=1, ps_gain=64X, PS_IT=0.391ms */
	.alsctrl_reg = 0x39,    /* als_persistance=1, als_gain=64X, ALS_IT=100ms */
	.ledctrl_reg = 0xFF,   /* 100mA IRDR, 64/64 LED duty */
	.wait_reg = 0xF,    /* 100 ms */
	.ps_thd_h =1700,
	.ps_thd_l = 1500,
	.int_pin = sprd_3rdparty_gpio_pls_irq,
	.transmittance = 500,
};
#endif

#define STK_FIR_LEN 2
#define MAX_FIR_LEN 32

struct data_filter {
	u16 raw[MAX_FIR_LEN];
	int sum;
	int number;
	int idx;
};

struct stk3x3x_data {
	struct i2c_client *client;
	struct stk3x3x_platform_data *pdata;
	struct device *ps_dev;
	struct device *als_dev;
#ifdef QUALCOMM_PLATFORM
	struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;
#endif
#if (!defined(STK_POLL_PS) || !defined(STK_POLL_ALS))
	int32_t irq;
	struct work_struct stk_work;
	struct workqueue_struct *stk_wq;
#endif
	uint16_t ir_code;
	uint16_t als_correct_factor;
	uint8_t alsctrl_reg;
	uint8_t psctrl_reg;
	uint8_t ledctrl_reg;
	uint8_t state_reg;
	int     int_pin;
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
	struct input_dev *als_input_dev;
	int32_t als_lux_last;
	uint32_t als_transmittance;
	bool als_enabled;
	bool re_enable_als;
	ktime_t ps_poll_delay;
	ktime_t als_poll_delay;
    bool als_fast_report;
    int als_fast_report_cnt;
#ifdef STK_POLL_ALS
	struct work_struct stk_als_work;
	struct hrtimer als_timer;
	struct workqueue_struct *stk_als_wq;
#endif
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
#ifdef STK_ALS_FIR
	struct data_filter      fir;
	atomic_t                firlength;
#endif
	atomic_t    recv_reg;

#ifdef STK_IRS
	int als_data_index;
#endif
#ifdef STK_QUALCOMM_POWER_CTRL
	struct regulator *vdd;
	struct regulator *vio;
	bool power_enabled;
#endif
	uint8_t pid;
	uint8_t p_wv_r_bd_with_co;
	uint32_t als_code_last;
	bool als_en_hal;

	uint32_t ps_code_last;
	uint8_t boot_cali;
	uint8_t p_1x_r_bd_with_co;
	uint8_t p_19_r_bc;

#ifdef STK_ALSPS_CALIBRATION
	uint16_t cali_crosstalk;
	uint16_t cali_near;
	uint16_t cali_near_increment;
	uint32_t lux_factor;
	uint32_t als_fac_cal;

	bool ps_cali_timer_run;
	ktime_t ps_cali_delay;
	struct hrtimer ps_cali_timer;
	struct work_struct stk_ps_cali_work;
	struct workqueue_struct *stk_ps_cali_wq;
	struct wake_lock ps_cali_nosuspend_wl;

	uint16_t prox_debug;
	uint8_t  debug_cnt;
#endif

#ifdef CTTRACKING
	bool ps_thd_update;
	uint16_t tracking_count;
#endif
	int stk_h_ht;
	int stk_h_lt;
    uint16_t als_debug_cnt;
    
    uint16_t slope;
    uint16_t als_ctrl;
    uint16_t code_debug;
    uint16_t als_gain_count;
    uint16_t als_gain_changing;
    bool entry_suspend;
};
static struct i2c_driver stk_ps_driver;

static struct class *ps_class;
static dev_t stk3337_ps_dev_t;

static struct class *als_class;
static dev_t stk3337_als_dev_t;

static int32_t stk3x3x_enable_ps(struct stk3x3x_data *stk_data, uint8_t enable, uint8_t validate_reg);
static int32_t stk3x3x_enable_als(struct stk3x3x_data *stk_data, uint8_t enable);
static int32_t stk3x3x_set_ps_thd_l(struct stk3x3x_data *stk_data, uint16_t thd_l);
static int32_t stk3x3x_set_ps_thd_h(struct stk3x3x_data *stk_data, uint16_t thd_h);
static int32_t stk3x3x_set_als_thd_l(struct stk3x3x_data *stk_data, uint16_t thd_l);
static int32_t stk3x3x_set_als_thd_h(struct stk3x3x_data *stk_data, uint16_t thd_h);
static int32_t stk3x3x_get_ir_reading(struct stk3x3x_data *stk_data, int32_t als_it_reduce);
static int32_t stk3x3x_get_state(struct stk3x3x_data *stk_data);
static int32_t stk3x3x_set_state(struct stk3x3x_data *stk_data, uint8_t state);
static int32_t stk3x3x_get_als_reading(struct stk3x3x_data *stk_data);
static int32_t stk3x3x_als_fast_report_init(struct stk3x3x_data *stk_data);
static void stk3x3x_als_fast_report_enable(struct stk3x3x_data *stk_data, int enable);

#ifdef STK_TUNE0
static int stk_ps_tune_zero_func_fae(struct stk3x3x_data *stk_data);
#endif
#ifdef STK_CHK_REG
static int stk3x3x_validate_n_handle(struct i2c_client *client);
#endif
static int stk_ps_val(struct stk3x3x_data *stk_data);
#ifdef STK_QUALCOMM_POWER_CTRL
static int stk3x3x_device_ctl(struct stk3x3x_data *stk_data, bool enable);
#endif

#ifdef STK_ALSPS_CALIBRATION
static void stk_ps_cali_work_func(struct work_struct *work);
static enum hrtimer_restart stk_ps_cali_timer_func(struct hrtimer *timer);
#endif

static int stk3x3x_i2c_read_data(struct i2c_client *client, unsigned char command, int length, unsigned char *values)
{
	uint8_t retry;
	int err;
	struct i2c_msg msgs[] = {
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

	for (retry = 0; retry < 5; retry++) {
		err = i2c_transfer(client->adapter, msgs, 2);
		if (err == 2)
			break;
		else
			mdelay(5);
	}

	if (retry >= 5) {
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
	else if (length >= 10) {
		SENSOR_LOG_ERROR("length %d exceeds 10\n", length);
		return -EINVAL;
	}

	data[0] = command;
	for (index=1; index<=length; index++)
		data[index] = values[index-1];

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = length+1;
	msg.buf = data;

	for (retry = 0; retry < 5; retry++) {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			break;
		else
			mdelay(5);
	}

	if (retry >= 5) {
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

#ifdef STK_ALSPS_CALIBRATION
static int32_t stk3x3x_als_calibration(struct stk3x3x_data *stk_data, const int source_cal_data)
{
	int32_t ret = 0;
	uint8_t w_state_reg;
	uint32_t cali_als;
	uint32_t reading, als_sum = 0, counter = 0;

	stk_data->als_fac_cal = 0;
	if(stk_data->als_enabled == false) {
		/*enable als*/
		ret = stk3x3x_get_state(stk_data);
		if(ret < 0)
			return ret;
		w_state_reg = ret;
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "cur state=0x%x\n", w_state_reg);
		w_state_reg |= STK_STATE_EN_ALS_MASK;

		ret = stk3x3x_set_state(stk_data, w_state_reg);
		if(ret < 0)
			return ret;
	}

	while (counter < ALS_CALI_COUNT) {
		msleep(110);
		reading = stk3x3x_get_als_reading(stk_data);
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug,"als %d = %d\n", counter, reading);
		als_sum += reading;
		counter++;
	}

	cali_als = (als_sum / ALS_CALI_COUNT);
	if(cali_als == 0) {
		SENSOR_LOG_ERROR("cali als = 0\n");
		return -1;
	}

	if(cali_als == 0){
		SENSOR_LOG_INFO("cali failed, als = 0\n");
	}else{
		stk_data->lux_factor = (source_cal_data * 1000) / cali_als;
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "cali factor = %d, source:%d, als:%d\n", stk_data->lux_factor, source_cal_data, cali_als);
		sensor_write_file(ALS_CAL_FILE_PATH, (char *)(&stk_data->lux_factor), sizeof(stk_data->lux_factor));
		stk_data->als_fac_cal = 1;
		sensor_write_file(ALS_FAC_CAL_PATH, (char *)(&stk_data->als_fac_cal), sizeof(stk_data->als_fac_cal));
	}
	
	if(stk_data->als_enabled == false) {
		//disable als
		ret = stk3x3x_get_state(stk_data);
		if(ret < 0)
			return ret;
		w_state_reg = ret;
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "cur state=0x%x\n", w_state_reg);
		w_state_reg &= ~(STK_STATE_EN_ALS_MASK);
		ret = stk3x3x_set_state(stk_data, w_state_reg);
	}
	return ret;
}

static int32_t stk3x3x_load_als_calibration_config(struct stk3x3x_data *stk_data)
{

	uint32_t lux_factor = 0;
	uint32_t als_fac_cal = 0;
	int ret = 0;

	ret = sensor_read_file(ALS_CAL_FILE_PATH, (char *)(&lux_factor), sizeof(lux_factor));
	if (ret < 0) {
		SENSOR_LOG_ERROR("read als cal file error\n");
		stk_data->lux_factor = 1000;
		return -1;
	}
	if (lux_factor == 0) {
		stk_data->lux_factor = 1000;
	} else {
		stk_data->lux_factor  = lux_factor;
	}

	ret = sensor_read_file(ALS_FAC_CAL_PATH, (char *)(&als_fac_cal), sizeof(als_fac_cal));
	if (ret < 0) {
		SENSOR_LOG_ERROR("read als_fac_cal error\n");
		return -1;
	}
    stk_data->als_fac_cal = als_fac_cal;
    
	return 0;
}
#endif /*endif STK_ALSPS_CALIBRATION*/
uint32_t stk_alscode2lux(struct stk3x3x_data *stk_data, uint64_t alscode)
{
#ifdef STK_ALSPS_CALIBRATION
	uint64_t lux;

	if(stk_data->als_fast_report == true){
        lux = 4*(alscode * stk_data->lux_factor * stk_data->slope) / (1000*1000);
        if((stk_data->als_debug_cnt%10) == 10){
            SENSOR_LOG_ERROR("fast 1_ctrl=%x, lux=%ld, alscode=%ld, lux_factor=%d, slope=%d\n", stk_data->als_ctrl, lux, alscode, stk_data->lux_factor, stk_data->slope);
        }
	}else if(stk_data->als_ctrl == 0x22){
        lux = 1*(alscode * stk_data->lux_factor * stk_data->slope) / (1000*1000);
        if((stk_data->als_debug_cnt%10) == 10){
            SENSOR_LOG_ERROR("1_ctrl=%x, lux=%ld, alscode=%ld, lux_factor=%d, slope=%d\n", stk_data->als_ctrl, lux, alscode, stk_data->lux_factor, stk_data->slope);
        }
    }else{
        lux = 4*(alscode * stk_data->lux_factor * stk_data->slope) / (1000*1000);
        if((stk_data->als_debug_cnt%10) == 10){
            SENSOR_LOG_ERROR("2_ctrl= %x, lux=%ld, alscode=%ld, lux_factor=%d, slope=%d\n", stk_data->als_ctrl, lux, alscode, stk_data->lux_factor, stk_data->slope);
        }
    }
    if(alscode <= 3){
        return alscode;
    }else{
	    return (lux + 3);
    }
#else
	alscode += ((alscode<<7)+(alscode<<3)+(alscode>>1));
	alscode<<=3;
	alscode/=stk_data->als_transmittance;
	return alscode;
#endif
}

uint32_t stk_lux2alscode(struct stk3x3x_data *stk_data, uint32_t lux)
{
#ifdef STK_ALSPS_CALIBRATION
	uint32_t alscode;
	alscode = (lux * 1000) / stk_data->lux_factor;
	if (unlikely(alscode>=(1<<16)))
		alscode = (1<<16) -1;
	return alscode;
#else
	lux*=stk_data->als_transmittance;
	lux/=1100;
	if (unlikely(lux>=(1<<16)))
		lux = (1<<16) -1;
	return lux;
#endif
}

void stk_als_set_new_thd(struct stk3x3x_data *stk_data, uint16_t alscode)
{
	int32_t high_thd,low_thd;
	high_thd = alscode + stk_lux2alscode(stk_data, STK_ALS_CHANGE_THD);
	low_thd = alscode - stk_lux2alscode(stk_data, STK_ALS_CHANGE_THD);
	if (high_thd >= (1<<16))
		high_thd = (1<<16) -1;
	if (low_thd <0)
		low_thd = 0;
	stk3x3x_set_als_thd_h(stk_data, (uint16_t)high_thd);
	stk3x3x_set_als_thd_l(stk_data, (uint16_t)low_thd);
}
static void stk3x3x_proc_plat_data(struct stk3x3x_data *stk_data, struct stk3x3x_platform_data *plat_data)
{
	uint8_t w_reg;

	stk_data->state_reg = plat_data->state_reg;
	stk_data->psctrl_reg = plat_data->psctrl_reg;
#ifdef STK_POLL_PS
	stk_data->psctrl_reg &= 0x3F;
#endif
	stk_data->alsctrl_reg = plat_data->alsctrl_reg;
	stk_data->ledctrl_reg = plat_data->ledctrl_reg;
	if(stk_data->pid == STK3310SA_PID || stk_data->pid == STK3311SA_PID)
		stk_data->ledctrl_reg &= 0x3F;
	stk_data->wait_reg = plat_data->wait_reg;
	if(stk_data->wait_reg < 2) {
		SENSOR_LOG_ERROR("wait_reg should be larger than 2, force to write 2\n");
		stk_data->wait_reg = 2;
	} else if (stk_data->wait_reg > 0xFF) {
		SENSOR_LOG_ERROR("wait_reg should be less than 0xFF, force to write 0xFF\n");
		stk_data->wait_reg = 0xFF;
	}

	if(stk_data->ps_thd_h == 0 && stk_data->ps_thd_l == 0) {
		stk_data->ps_thd_h = plat_data->ps_thd_h;
		stk_data->ps_thd_l = plat_data->ps_thd_l;
	}

#ifdef CALI_PS_EVERY_TIME
	stk_data->ps_high_thd_boot = plat_data->ps_thd_h;
	stk_data->ps_low_thd_boot = plat_data->ps_thd_l;
#endif
	w_reg = 0;
#ifndef STK_POLL_PS
	w_reg |= STK_INT_PS_MODE;
#else
	w_reg |= 0x01;
#endif

#if (!defined(STK_POLL_ALS) && (STK_INT_PS_MODE != 0x02) && (STK_INT_PS_MODE != 0x03))
	w_reg |= STK_INT_ALS;
#endif
	stk_data->int_reg = w_reg;
	return;
}

static int32_t stk3x3x_init_all_reg(struct stk3x3x_data *stk_data)
{
	int32_t ret;

	ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, STK_STATE_REG, stk_data->state_reg);
	if (ret < 0) {
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}
	ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, STK_PSCTRL_REG, stk_data->psctrl_reg);
	if (ret < 0) {
		SENSOR_LOG_ERROR("%write i2c error\n");
		return ret;
	}
	ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, STK_ALSCTRL_REG, stk_data->alsctrl_reg);
	if (ret < 0) {
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}
	ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, STK_LEDCTRL_REG, stk_data->ledctrl_reg);
	if (ret < 0) {
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}
	ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, STK_WAIT_REG, stk_data->wait_reg);
	if (ret < 0) {
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}
#ifdef STK_TUNE0
	stk_data->psa = 0x0;
	stk_data->psi = 0xFFFF;
#endif
	stk3x3x_set_ps_thd_h(stk_data, stk_data->ps_thd_h);
	stk3x3x_set_ps_thd_l(stk_data, stk_data->ps_thd_l);

	ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, STK_INT_REG, stk_data->int_reg);
	if (ret < 0) {
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}

	//INTEL PEERS
	ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, 0x4F, 0x3F);
	if (ret < 0) {
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}

	ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, 0x4D, 0x01);
	if (ret < 0) {
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}
	//PS INT mode
	ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, 0xFA, 0x01);
	if (ret < 0) {
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}

	ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, 0xA0, 0x10);
	if (ret < 0) {
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}

	ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, 0xAA, 0x64);
	if (ret < 0) {
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}

	ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, 0xDB, 0x15);
	if (ret < 0) {
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
	if (err < 0) {
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
	if (err < 0) {
		SENSOR_LOG_ERROR("write i2c error\n");
		return err;
	}

	return value;
}

static int32_t stk3x3x_check_pid(struct stk3x3x_data *stk_data)
{
	unsigned char value[3], pid_msb;
	int err;
	int otp25;

	stk_data->p_wv_r_bd_with_co = 0;
	stk_data->p_1x_r_bd_with_co = 0;
	stk_data->p_19_r_bc = 0;

	err = stk3x3x_i2c_read_data(stk_data->client, STK_PDT_ID_REG, 2, &value[0]);
	if(err < 0) {
		SENSOR_LOG_ERROR("fail, ret=%d\n", err);
		return err;
	}
	err = stk3x3x_i2c_smbus_read_byte_data(stk_data->client, 0xE0);
	if(err < 0)
		return err;
	value[2] = err;

	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "PID=0x%x, RID=0x%x, 0x90=0x%x\n", value[0], value[1], value[2]);
	stk_data->pid = value[0];

	if(value[0] == STK3311WV_PID)
		stk_data->p_wv_r_bd_with_co |= 0b100;
	else if(value[0] == STK3311X_PID)
		stk_data->p_1x_r_bd_with_co |= 0b100;
	else if(value[0] == STK33119_PID)
		stk_data->p_19_r_bc |= 0b10;

	if(value[1] == 0xC3) {
		stk_data->p_wv_r_bd_with_co |= 0b010;
		stk_data->p_1x_r_bd_with_co |= 0b010;
	} else if(value[1] == 0xC2) {
		stk_data->p_19_r_bc |= 0b01;
	}

	err = stk3x3x_otp_read_byte_data(stk_data->client, 0x25);
	if(err < 0)
		return err;

	otp25 = err;
	if(otp25 & 0x80)
		stk_data->p_wv_r_bd_with_co |= 0b001;
	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " p_wv_r_bd_with_co = 0x%x\n", stk_data->p_wv_r_bd_with_co);

	if(otp25 & 0x40)
		stk_data->p_1x_r_bd_with_co |= 0b001;
	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " p_1x_r_bd_with_co = 0x%x\n", stk_data->p_1x_r_bd_with_co);

	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " p_19_r_bc = 0x%x\n", stk_data->p_19_r_bc);

	if(value[0] == 0) {
		SENSOR_LOG_ERROR("PID=0x0, please make sure the chip is stk3x3x!\n");
		return -2;
	}

	pid_msb = value[0] & 0xF0;
	switch(pid_msb) {
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

static void stk3x3x_dump_reg(struct stk3x3x_data *stk_data)
{
	int ret;
	unsigned char reg[13]= {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x4F, 0xA0, 0xA1, 0xDB, 0xA4, 0x3E, 0x3F};
	uint8_t cnt;
	for(cnt = 0; cnt < 13; cnt++) {
		ret = stk3x3x_i2c_smbus_read_byte_data(stk_data->client, reg[cnt]);
		if(ret < 0) {
			SENSOR_LOG_ERROR("fail, ret=%d\n", ret);
		}

		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "reg[0x%2X]=0x%2X\n", reg[cnt], ret);
	}
}

static int32_t stk3x3x_software_reset(struct stk3x3x_data *stk_data)
{
	int32_t r;
	uint8_t w_reg;

	w_reg = 0x7F;
	r = stk3x3x_i2c_smbus_write_byte_data(stk_data->client,STK_WAIT_REG,w_reg);
	if (r<0) {
		SENSOR_LOG_ERROR("software reset: write i2c error, ret=%d\n", r);
		return r;
	}
	r = stk3x3x_i2c_smbus_read_byte_data(stk_data->client,STK_WAIT_REG);
	if (w_reg != r) {
		SENSOR_LOG_ERROR("software reset: read-back value is not the same\n");
		return -1;
	}

	r = stk3x3x_i2c_smbus_write_byte_data(stk_data->client,STK_SW_RESET_REG,0);
	if (r<0) {
		SENSOR_LOG_ERROR("software reset: read error after reset\n");
		return r;
	}
	usleep_range(30000, 50000);
	return 0;
}


static int32_t stk3x3x_set_als_thd_l(struct stk3x3x_data *stk_data, uint16_t thd_l)
{
	unsigned char val[2];
	int ret;
	val[0] = (thd_l & 0xFF00) >> 8;
	val[1] = thd_l & 0x00FF;
	ret = stk3x3x_i2c_write_data(stk_data->client, STK_THDL1_ALS_REG, 2, val);
	if(ret < 0)
		SENSOR_LOG_ERROR("fail, ret=%d\n", ret);

	return ret;
}
static int32_t stk3x3x_set_als_thd_h(struct stk3x3x_data *stk_data, uint16_t thd_h)
{
	unsigned char val[2];
	int ret;
	val[0] = (thd_h & 0xFF00) >> 8;
	val[1] = thd_h & 0x00FF;
	ret = stk3x3x_i2c_write_data(stk_data->client, STK_THDH1_ALS_REG, 2, val);
	if(ret < 0)
		SENSOR_LOG_ERROR("fail, ret=%d\n", ret);
	return ret;
}

static int32_t stk3x3x_set_ps_thd_l(struct stk3x3x_data *stk_data, uint16_t thd_l)
{
	unsigned char val[2];
	int ret;
	val[0] = (thd_l & 0xFF00) >> 8;
	val[1] = thd_l & 0x00FF;
	ret = stk3x3x_i2c_write_data(stk_data->client, STK_THDL1_PS_REG, 2, val);
	if(ret < 0)
		SENSOR_LOG_ERROR("fail, ret=%d\n", ret);
	return ret;
}
static int32_t stk3x3x_set_ps_thd_h(struct stk3x3x_data *stk_data, uint16_t thd_h)
{
	unsigned char val[2];
	int ret;
	val[0] = (thd_h & 0xFF00) >> 8;
	val[1] = thd_h & 0x00FF;
	ret = stk3x3x_i2c_write_data(stk_data->client, STK_THDH1_PS_REG, 2, val);
	if(ret < 0)
		SENSOR_LOG_ERROR("fail, ret=%d\n", ret);
	return ret;
}

static uint32_t stk3x3x_get_ps_thd_h(struct stk3x3x_data *stk_data)
{
	unsigned char value[2];
	int err;
	err = stk3x3x_i2c_read_data(stk_data->client, STK_THDH1_PS_REG, 2, &value[0]);
	if(err < 0) {
		SENSOR_LOG_ERROR("fail, ret=%d\n", err);
		return err;
	}
	return ((value[0]<<8) | value[1]);
}

static uint32_t stk3x3x_get_ps_thd_l(struct stk3x3x_data *stk_data)
{
	unsigned char value[2];
	int err;
	err = stk3x3x_i2c_read_data(stk_data->client, STK_THDL1_PS_REG, 2, &value[0]);
	if(err < 0) {
		SENSOR_LOG_ERROR("fail, ret=%d\n", err);
		return err;
	}
	return ((value[0]<<8) | value[1]);
}

static uint32_t stk3x3x_get_ps_reading(struct stk3x3x_data *stk_data)
{
	unsigned char value[2];
	int err;
	err = stk3x3x_i2c_read_data(stk_data->client, STK_DATA1_PS_REG, 2, &value[0]);
	if(err < 0) {
		SENSOR_LOG_ERROR("fail, ret=%d\n", err);
		return err;
	}
	return ((value[0]<<8) | value[1]);
}


static int32_t stk3x3x_set_flag(struct stk3x3x_data *stk_data, uint8_t org_flag_reg, uint8_t clr)
{
	uint8_t w_flag;
	int ret;

	w_flag = org_flag_reg | (STK_FLG_ALSINT_MASK | STK_FLG_PSINT_MASK | STK_FLG_OUI_MASK | STK_FLG_IR_RDY_MASK);
	w_flag &= (~clr);
	//SENSOR_LOG_INFO(" org_flag_reg=0x%x, w_flag = 0x%x\n", org_flag_reg, w_flag);
	ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client,STK_FLAG_REG, w_flag);
	if(ret < 0)
		SENSOR_LOG_ERROR("fail, ret=%d\n", ret);
	return ret;
}

static int32_t stk3x3x_get_flag(struct stk3x3x_data *stk_data)
{
	int ret;
	ret = stk3x3x_i2c_smbus_read_byte_data(stk_data->client,STK_FLAG_REG);
	if(ret < 0)
		SENSOR_LOG_ERROR("fail, ret=%d\n", ret);
	return ret;
}

static int32_t stk3x3x_set_state(struct stk3x3x_data *stk_data, uint8_t state)
{
	int ret;
	ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client,STK_STATE_REG, state);
	if(ret < 0)
		SENSOR_LOG_ERROR("fail, ret=%d\n", ret);
	return ret;
}

static int32_t stk3x3x_get_state(struct stk3x3x_data *stk_data)
{
	int ret;
	ret = stk3x3x_i2c_smbus_read_byte_data(stk_data->client,STK_STATE_REG);
	if(ret < 0)
		SENSOR_LOG_ERROR("fail, ret=%d\n", ret);
	return ret;
}

static void stk_ps_report(struct stk3x3x_data *stk_data, int nf)
{
#ifdef QUALCOMM_PLATFORM
	ktime_t timestamp = ktime_get_boottime();
#endif

	stk_data->ps_distance_last = nf;
//  input_report_abs(stk_data->ps_input_dev, ABS_DISTANCE, nf);
	input_report_rel(stk_data->ps_input_dev, REL_RZ, nf? 10:3);

#ifdef QUALCOMM_PLATFORM
	input_event(stk_data->ps_input_dev, EV_SYN, SYN_TIME_SEC, ktime_to_timespec(timestamp).tv_sec);
	input_event(stk_data->ps_input_dev, EV_SYN, SYN_TIME_NSEC, ktime_to_timespec(timestamp).tv_nsec);
#endif
	input_sync(stk_data->ps_input_dev);
	wake_lock_timeout(&stk_data->ps_wakelock, 3*HZ);
}

static void stk_als_report(struct stk3x3x_data *stk_data, int als)
{
#ifdef QUALCOMM_PLATFORM
	ktime_t timestamp = ktime_get_boottime();
#endif

	stk_data->als_lux_last = als;
    if(als == 0){
	    input_report_rel(stk_data->als_input_dev, REL_X, (als + stk_data->als_debug_cnt%2));
    }else{
	    input_report_rel(stk_data->als_input_dev, REL_X, als);
    }
#ifdef QUALCOMM_PLATFORM
	input_event(stk_data->als_input_dev, EV_SYN, SYN_TIME_SEC, ktime_to_timespec(timestamp).tv_sec);
	input_event(stk_data->als_input_dev, EV_SYN, SYN_TIME_NSEC, ktime_to_timespec(timestamp).tv_nsec);
#endif
	input_sync(stk_data->als_input_dev);

    if((stk_data->als_debug_cnt%10) == 10){
	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "als input event %d lux\n", als);
    }

}

static int stk_prox_increment_init(struct stk3x3x_data *stk_data)
{
	int rc = 0;
	uint16_t data[2] = {0};
	uint32_t high_thd;
	uint16_t ct_value = 0;

	SENSOR_LOG_ERROR("enter\n");
	rc = sensor_read_file(PS_CAL_FILE_PATH, (char*)data, sizeof(data));
	if (rc < 0) {
		SENSOR_LOG_ERROR("read file error\n");
		return -1;
	}

	stk_data->cali_near_increment = data[0];
	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "chip->cali_near_increment = %d\n",
	                 stk_data->cali_near_increment);

	if((stk_data->cali_near_increment > PS_THRESH_INCREMENT_MAX) || (stk_data->cali_near_increment == 0 ))  {
		SENSOR_LOG_ERROR("read file value exception [%d]\n", stk_data->cali_near_increment);
		return -1;
	}

	high_thd = stk3x3x_get_ps_thd_h(stk_data);
	if (high_thd < 0) {
		SENSOR_LOG_DEBUG("fail, error: %d\n", high_thd);
		return high_thd;
	}
	ct_value = high_thd - stk_data->stk_ht_n_ct;
	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "old high = %d, delta ht=%d, lt=%d\n",
	                 high_thd, stk_data->stk_ht_n_ct, stk_data->stk_lt_n_ct);

	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "old smudge delta ht = %d, lt=%d\n",
	                 stk_data->stk_h_ht, stk_data->stk_h_lt);

	stk_data->stk_ht_n_ct = stk_data->cali_near_increment;
	stk_data->stk_lt_n_ct = stk_data->stk_ht_n_ct - PS_FAR_DIFFER_VALUE;
	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "new delta ht=%d, lt=%d\n", stk_data->stk_ht_n_ct, stk_data->stk_lt_n_ct);

	if(stk_data->stk_ht_n_ct > stk_data->stk_h_ht) {
		stk_data->stk_h_ht = stk_data->stk_ht_n_ct;
		stk_data->stk_h_lt = stk_data->stk_lt_n_ct;
		SENSOR_LOG_ERROR("new smudge delta ht = %d, lt=%d\n", stk_data->stk_h_ht, stk_data->stk_h_lt);
	}

	stk_data->ps_thd_h = ct_value + stk_data->stk_ht_n_ct;
	stk_data->ps_thd_l = ct_value + stk_data->stk_lt_n_ct;

	stk_data->ps_high_thd_boot = ct_value + stk_data->stk_h_ht;
	stk_data->ps_low_thd_boot = ct_value + stk_data->stk_h_lt;

	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "thd(%d, %d), smudge thd(%d, %d)\n",
	                 stk_data->ps_thd_h, stk_data->ps_thd_l, stk_data->ps_high_thd_boot, stk_data->ps_low_thd_boot);

	return 0;
}

static int32_t stk3x3x_enable_ps(struct stk3x3x_data *stk_data, uint8_t enable, uint8_t validate_reg)
{
	int32_t ret;
	uint8_t w_state_reg;
	uint8_t curr_ps_enable;
	uint32_t reading;
	int32_t near_far_state;

#ifdef STK_QUALCOMM_POWER_CTRL
	if (enable) {
		ret = stk3x3x_device_ctl(stk_data, enable);
		if (ret)
			return ret;
	}
#endif

#ifdef STK_CHK_REG
	if(validate_reg) {
		ret = stk3x3x_validate_n_handle(stk_data->client);
		if(ret < 0)
			SENSOR_LOG_ERROR("stk3x3x_validate_n_handle fail: %d\n", ret);
	}
#endif /* #ifdef STK_CHK_REG */

	curr_ps_enable = stk_data->ps_enabled?1:0;
	if(curr_ps_enable == enable)
		return 0;

#ifdef STK_TUNE0
#ifndef CTTRACKING
	if (!(stk_data->psi_set) && !enable) {
		hrtimer_cancel(&stk_data->ps_tune0_timer);
		cancel_work_sync(&stk_data->stk_ps_tune0_work);
	}
#endif
#endif
	if(stk_data->first_boot == true) {
		stk_data->first_boot = false;
	}

	ret = stk3x3x_get_state(stk_data);
	if(ret < 0)
		return ret;
	w_state_reg = ret;


	w_state_reg &= ~(STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK | STK_STATE_EN_AK_MASK);
	if(enable) {
		w_state_reg |= STK_STATE_EN_PS_MASK;
		if(!(stk_data->als_enabled))
			w_state_reg |= STK_STATE_EN_WAIT_MASK;
	}
	ret = stk3x3x_set_state(stk_data, w_state_reg);
	if(ret < 0)
		return ret;
	stk_data->state_reg = w_state_reg;

	if(enable) {
#ifdef STK_TUNE0
#ifdef CALI_PS_EVERY_TIME
		stk_data->psi_set = 0;
		stk_data->psa = 0;
		stk_data->psi = 0xFFFF;
		stk_data->data_count = 0;
#ifdef CTTRACKING
		stk_data->ps_thd_update = false;
		stk_data->tracking_count = 0;
#endif


		if(stk_increment_init_flag == false) {
			ret = stk_prox_increment_init(stk_data);
			if(ret < 0) {
				stk_increment_init_flag = false;
			} else {
				stk_increment_init_flag = true;
			}
		}
		if(stk_data->boot_cali == 1 && stk_data->ps_low_thd_boot < 8000 ) {
			stk_data->ps_thd_h = stk_data->ps_high_thd_boot;
			stk_data->ps_thd_l = stk_data->ps_low_thd_boot;
		}

		stk3x3x_set_ps_thd_h(stk_data, stk_data->ps_thd_h);
		stk3x3x_set_ps_thd_l(stk_data, stk_data->ps_thd_l);
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "cur delta ht=%d, lt=%d\n", stk_data->stk_ht_n_ct, stk_data->stk_lt_n_ct);
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "smudge delta ht=%d, lt=%d\n", stk_data->stk_h_ht, stk_data->stk_h_lt);
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "cali crosstalk=%d, increment=%d\n", stk_data->cali_crosstalk, stk_data->cali_near_increment);
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "set ht=%d, lt=%d\n", stk_data->ps_thd_h, stk_data->ps_thd_l);

		if((stk_data->stk_ht_n_ct == 0) || (stk_data->stk_ht_n_ct < 0)
			|| (stk_data->stk_lt_n_ct == 0) || (stk_data->stk_lt_n_ct < 0)) {
			stk_data->stk_ht_n_ct = STK_HT_N_CT;
			stk_data->stk_lt_n_ct = STK_LT_N_CT;
			SENSOR_LOG_ERROR("reset delta ht=%d, lt=%d\n", stk_data->stk_ht_n_ct, stk_data->stk_lt_n_ct);
		}

		if((stk_data->stk_ht_n_ct < STK_H_HT) && (stk_data->stk_h_ht > STK_H_HT)) {
			stk_data->stk_h_ht = STK_H_HT;
			stk_data->stk_h_lt = STK_H_LT;
			SENSOR_LOG_ERROR("reset smudge delta ht=%d, lt=%d\n", stk_data->stk_h_ht, stk_data->stk_h_lt);
		}

		hrtimer_start(&stk_data->ps_tune0_timer, stk_data->ps_tune0_delay, HRTIMER_MODE_REL);
#else
		if (!(stk_data->psi_set))
			hrtimer_start(&stk_data->ps_tune0_timer, stk_data->ps_tune0_delay, HRTIMER_MODE_REL);
#endif  /* #ifdef CALI_PS_EVERY_TIME */
#endif
		/*
		    if(stk_increment_init_flag == false)
		    {
		        ret = stk_prox_increment_init(stk_data);
		        if(ret < 0)
		        {
		            stk_increment_init_flag = false;
		        }
		        else
		        {
		            stk_increment_init_flag = true;
		        }
		    }

		    SENSOR_LOG_ERROR("smudge delta ht=%d, lt=%d\n", stk_data->stk_h_ht, stk_data->stk_h_lt);
		*/
#ifdef STK_CHK_REG
		if(!validate_reg) {
			reading = stk3x3x_get_ps_reading(stk_data);
			stk_ps_report(stk_data, 1);
			SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " force report ps input event=1, ps code = %d\n",reading);
		} else
#endif /* #ifdef STK_CHK_REG */
		{
			usleep_range(4000, 5000);
			reading = stk3x3x_get_ps_reading(stk_data);
			if (reading < 0)
				return reading;

			ret = stk3x3x_get_flag(stk_data);
			if (ret < 0)
				return ret;
			near_far_state = ret & STK_FLG_NF_MASK;
			stk_ps_report(stk_data, near_far_state);
			SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " ps input event=%d, ps=%d\n",near_far_state, reading);
		}
#ifdef STK_POLL_PS
		hrtimer_start(&stk_data->ps_timer, stk_data->ps_poll_delay, HRTIMER_MODE_REL);
		stk_data->ps_distance_last = -1;
#endif
#ifndef STK_POLL_PS
#ifndef STK_POLL_ALS
		if(!(stk_data->als_enabled))
#endif  /* #ifndef STK_POLL_ALS */
			enable_irq(stk_data->irq);
#endif  /* #ifndef STK_POLL_PS */
		stk_data->ps_enabled = true;
        
        stk_data->entry_suspend = false;
		//dump reg info
		stk3x3x_dump_reg(stk_data);
	} else {
#ifdef STK_POLL_PS
		hrtimer_cancel(&stk_data->ps_timer);
		cancel_work_sync(&stk_data->stk_ps_work);
#else
#ifndef STK_POLL_ALS
		if(!(stk_data->als_enabled))
#endif
			disable_irq(stk_data->irq);
#endif
		stk_data->ps_enabled = false;

#ifdef CTTRACKING
		hrtimer_cancel(&stk_data->ps_tune0_timer);
		cancel_work_sync(&stk_data->stk_ps_tune0_work);
#endif

#ifdef STK_QUALCOMM_POWER_CTRL
		ret = stk3x3x_device_ctl(stk_data, enable);
		if (ret)
			return ret;
#endif
	}
	return ret;
}

static int32_t stk3x3x_enable_als(struct stk3x3x_data *stk_data, uint8_t enable)
{
	int32_t ret;
	uint8_t w_state_reg;
	uint8_t curr_als_enable = (stk_data->als_enabled)?1:0;

	if(curr_als_enable == enable)
		return 0;
#ifdef STK_QUALCOMM_POWER_CTRL
	if (enable) {
		ret = stk3x3x_device_ctl(stk_data, enable);
		if (ret)
			return ret;
	}
#endif
#ifndef STK_POLL_ALS
#ifdef STK_IRS
	if(enable && !(stk_data->ps_enabled)) {
		ret = stk3x3x_get_ir_reading(stk_data, STK_IRS_IT_REDUCE );
		if(ret > 0)
			stk_data->ir_code = ret;
	}
#endif

	if (enable) {
		stk3x3x_set_als_thd_h(stk_data, 0x0000);
		stk3x3x_set_als_thd_l(stk_data, 0xFFFF);
	}
#endif

	ret = stk3x3x_get_state(stk_data);
	if(ret < 0)
		return ret;

	w_state_reg = (uint8_t)(ret & (~(STK_STATE_EN_ALS_MASK | STK_STATE_EN_WAIT_MASK)));
	if(enable){
		w_state_reg |= STK_STATE_EN_ALS_MASK;
        stk3x3x_als_fast_report_enable(stk_data,1);}
	else if (stk_data->ps_enabled)
		w_state_reg |= STK_STATE_EN_WAIT_MASK;

	ret = stk3x3x_set_state(stk_data, w_state_reg);
	if(ret < 0)
		return ret;
	stk_data->state_reg = w_state_reg;

	if (enable) {
        stk_data->als_debug_cnt = 0;
		stk_data->als_enabled = true;
#ifdef STK_POLL_ALS
		hrtimer_start(&stk_data->als_timer, stk_data->als_poll_delay, HRTIMER_MODE_REL);
#else
#ifndef STK_POLL_PS
		if(!(stk_data->ps_enabled))
#endif
			enable_irq(stk_data->irq);
#endif
#ifdef STK_IRS
		stk_data->als_data_index = 0;
#endif
        stk_data->slope = LIGHT_SLOPE_CWF;
        stk_data->als_ctrl = 0x22;
        stk_data->als_gain_changing = 0; 
        stk_data->als_gain_count = 0;
        memset(&stk_data->fir, 0x00, sizeof(stk_data->fir));
        stk_data->fir.number = 0;
        SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "cali factor=%d\n", stk_data->lux_factor);
	} else {
		stk_data->als_enabled = false;
#ifdef STK_ALS_FIR
        memset(&stk_data->fir, 0x00, sizeof(stk_data->fir));
#endif
#ifdef STK_POLL_ALS
		hrtimer_cancel(&stk_data->als_timer);
		cancel_work_sync(&stk_data->stk_als_work);
        stk3x3x_als_fast_report_enable(stk_data, 0);
#else
#ifndef STK_POLL_PS
		if(!(stk_data->ps_enabled))
#endif
			disable_irq(stk_data->irq);
#endif
#ifdef STK_QUALCOMM_POWER_CTRL
		ret = stk3x3x_device_ctl(stk_data, enable);
		if (ret)
			return ret;
#endif
	}
	return ret;
}


static int32_t stk3x3x_get_als_reading(struct stk3x3x_data *stk_data)
{
    int32_t r_data, g_data, b_data, c_data, ir_data = 0;
	int64_t als_data = 0;
#ifdef STK_ALS_FIR
	int index;
	int firlen = atomic_read(&stk_data->firlength);
#endif
	unsigned char value[6];
	int ret;
#ifdef STK_IRS
	const int ir_enlarge = 1 << (STK_ALS_READ_IRS_IT_REDUCE - STK_IRS_IT_REDUCE);
#endif

    uint16_t cg_ratio = 0;
	uint16_t cur_als_ctrl;

	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "ALS:reading als_ctrl=0x%x, mode=%d, cnt=%d\n", stk_data->als_ctrl, stk_data->als_gain_changing, stk_data->als_gain_count);

	cur_als_ctrl = stk3x3x_i2c_smbus_read_byte_data(stk_data->client, STK_ALSCTRL_REG);
	if(ret < 0) {
		SENSOR_LOG_ERROR("fail, ret=%d\n", ret);
	}

	if(stk_data->als_fast_report == false){
		stk_data->als_ctrl = cur_als_ctrl;
	}else{
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "fast:%d, cnt%d, als_ctrl:0x%x, cur=0x%x\n", stk_data->als_fast_report, stk_data->als_fast_report_cnt, stk_data->als_ctrl, cur_als_ctrl);
	}

	ret = stk3x3x_i2c_read_data(stk_data->client, STK_DATA1_ALS_REG, 6, &value[0]);
	if(ret < 0) {
		SENSOR_LOG_ERROR("fail, ret=0x%x\n", ret);
		return ret;
	}
	als_data = (value[0]<<8) | value[1];
	r_data = (value[2]<<8) | value[3];
	g_data = (value[4]<<8) | value[5];

	ret = stk3x3x_i2c_read_data(stk_data->client, 0x19, 4, &value[0]);
	if(ret < 0) {
		SENSOR_LOG_ERROR("fail, ret=0x%x\n", ret);
		return ret;
	}
	b_data = (value[0]<<8) | value[1];
	c_data = (value[2]<<8) | value[3];

    if((stk_data->als_debug_cnt%10) == 10){
        SENSOR_LOG_ERROR("ALS:als=%d, c=%d, gain=0x%x\n", als_data, c_data, stk_data->als_ctrl);
        SENSOR_LOG_ERROR("ALS:r=%d, g=%d, b=%d\n", r_data, g_data, b_data);
		SENSOR_LOG_ERROR("fast:%d, cnt%d, als_ctrl:0x%x, cur=0x%x\n", stk_data->als_fast_report, stk_data->als_fast_report_cnt, stk_data->als_ctrl, cur_als_ctrl);
    }   

	if(stk_data->als_fast_report == false){
		if(g_data > 60000){
			if(stk_data->als_ctrl != 0x12){
				SENSOR_LOG_ERROR("ALS:als_ctrl11=%d, mode=%d, cnt=%d\n", stk_data->als_ctrl, stk_data->als_gain_changing, stk_data->als_gain_count);
				if(stk_data->als_gain_changing != 1){
					stk_data->als_gain_changing = 1;//gainx4
					stk_data->als_gain_count = 0;
				}else if(stk_data->als_gain_changing == 1){
					stk_data->als_gain_count++;
				}
				
				if(stk_data->als_gain_count > ALS_GAIN_COUNT){
					SENSOR_LOG_ERROR("ALS:set dgainx4, [%d, %d]\n", stk_data->als_gain_changing, stk_data->als_gain_count);
					ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, STK_ALSCTRL_REG, 0x12);
					if (ret < 0) {
						SENSOR_LOG_ERROR("write i2c error\n");
						return ret;
					}

					stk_data->als_gain_changing = 0; 
					stk_data->als_gain_count = 0;

					return (stk_data->als_code_last);
				}
			}else{
				stk_data->als_gain_changing = 0; 
				stk_data->als_gain_count = 0;				
			}
		}else if(g_data < 3000){
			if(stk_data->als_ctrl != 0x22){
				SENSOR_LOG_ERROR("ALS:als_ctrl22=%d, mode=%d, cnt=%d\n", stk_data->als_ctrl, stk_data->als_gain_changing, stk_data->als_gain_count);
				if(stk_data->als_gain_changing != 2){
					stk_data->als_gain_changing = 2;//gainx16
					stk_data->als_gain_count = 0;
				}else if(stk_data->als_gain_changing == 2){
					stk_data->als_gain_count++;
				}
				if(stk_data->als_gain_count > ALS_GAIN_COUNT){
					SENSOR_LOG_ERROR("ALS:set dgainx16, [%d, %d]\n", stk_data->als_gain_changing, stk_data->als_gain_count);
					ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, STK_ALSCTRL_REG, 0x22);
					if (ret < 0) {
						SENSOR_LOG_ERROR("write i2c error\n");
						return ret;
					}	
					stk_data->als_gain_changing = 0; 
					stk_data->als_gain_count = 0;

					return (stk_data->als_code_last);		
				}
			}else{
				stk_data->als_gain_changing = 0; 
				stk_data->als_gain_count = 0;				
			}
		}else{
			stk_data->als_gain_changing = 0; 
			stk_data->als_gain_count = 0;	
		}
	}

    if(g_data < 100){
	   stk_data->slope = LIGHT_SLOPE_CWF;
    }else{
        if(stk_data->als_ctrl == 0x22){
            cg_ratio = (c_data * 1000)/g_data;
        }else{
            cg_ratio = (c_data * 1000)/(g_data*4);
        }
    }

    if(cg_ratio >= 20){
        if(stk_data->lux_factor == 1000){
            stk_data->slope = 335;
		}else{
            stk_data->slope = LIGHT_SLOPE_A;
        }
        if((stk_data->als_debug_cnt%10) == 10){
		    SENSOR_LOG_ERROR("ALS: cg=%d, a slope=%d\n", cg_ratio, stk_data->slope);
        }
    }else if(cg_ratio >= 10){
		if(c_data > 900){
			stk_data->slope = LIGHT_SLOPE_CLOUDY2;
            if((stk_data->als_debug_cnt%10) == 10){
			    SENSOR_LOG_ERROR("ALS: cg=%d, cloudy2 slope=%d\n", cg_ratio, stk_data->slope);
		    }
        }else{
			stk_data->slope = LIGHT_SLOPE_CLOUDY1;
            if((stk_data->als_debug_cnt%10) == 10){
			    SENSOR_LOG_ERROR("ALS: cg=%d, cloudy1 slope=%d\n", cg_ratio, stk_data->slope);
            }
        }
    }else{
        if(stk_data->lux_factor == 1000){
            stk_data->slope = 335;
        }else{
            stk_data->slope = LIGHT_SLOPE_CWF;
        }
        if((stk_data->als_debug_cnt%10) == 10){
            SENSOR_LOG_ERROR("ALS: cg=%d, cwf slope=%d\n", cg_ratio, stk_data->slope);
        }
    }

	if(stk_data->p_1x_r_bd_with_co == 0x07 || stk_data->p_19_r_bc == 0x03) {
		als_data = als_data * 16 / 10;
		if(als_data > 65535)
			als_data = 65535;
	}

	if(stk_data->p_wv_r_bd_with_co & 0b010) {
		if(als_data < STK_ALS_THRESHOLD && stk_data->als_code_last > 10000) {
			ir_data = stk3x3x_get_ir_reading(stk_data, STK_ALS_READ_IRS_IT_REDUCE);
#ifdef STK_IRS
			if(ir_data > 0)
				stk_data->ir_code = ir_data * ir_enlarge;
#endif

			if(ir_data > (STK_ALS_THRESHOLD*3)) {
				als_data = stk_data->als_code_last;
			}
		}
#ifdef STK_IRS
		else {
			stk_data->ir_code = 0;
		}
#endif
	}

	stk_data->als_code_last = als_data;

#ifdef STK_ALS_FIR
	if(stk_data->fir.number < firlen) {
		stk_data->fir.raw[stk_data->fir.number] = als_data;
		stk_data->fir.sum += als_data;
		stk_data->fir.number++;
		stk_data->fir.idx++;
	} else {
		index = stk_data->fir.idx % firlen;
		stk_data->fir.sum -= stk_data->fir.raw[index];
		stk_data->fir.raw[index] = als_data;
		stk_data->fir.sum += als_data;
		stk_data->fir.idx++;
		als_data = stk_data->fir.sum/firlen;
	}
#endif

	return als_data;
}


#if (defined(STK_IRS) && defined(STK_POLL_ALS))
static int stk_als_ir_skip_als(struct stk3x3x_data *stk_data)
{
	int ret;
	unsigned char value[2];

	if(stk_data->als_data_index < 60000)
		stk_data->als_data_index++;
	else
		stk_data->als_data_index = 0;

	if( stk_data->als_data_index % 10 == 1) {
		ret = stk3x3x_i2c_read_data(stk_data->client, STK_DATA1_ALS_REG, 2, &value[0]);
		if(ret < 0) {
			SENSOR_LOG_ERROR("fail, ret=0x%x\n", ret);
			return ret;
		}
		return 1;
	}
	return 0;
}

static void stk_als_ir_get_corr(struct stk3x3x_data *stk_data, int32_t als)
{
	int32_t als_comperator;

	if(stk_data->ir_code) {
		stk_data->als_correct_factor = 1000;
		if(als < STK_IRC_MAX_ALS_CODE && als > STK_IRC_MIN_ALS_CODE &&
		   stk_data->ir_code > STK_IRC_MIN_IR_CODE) {
			als_comperator = als * STK_IRC_ALS_NUMERA / STK_IRC_ALS_DENOMI;
			if(stk_data->ir_code > als_comperator)
				stk_data->als_correct_factor = STK_IRC_ALS_CORREC;
		}

		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "als=%d, ir=%d, als_correct_factor=%d\n", als, stk_data->ir_code, stk_data->als_correct_factor);
		stk_data->ir_code = 0;
	}
	return;
}

static int stk_als_ir_run(struct stk3x3x_data *stk_data)
{
	int ret;

	if( stk_data->als_data_index % 10 == 0) {
		if(stk_data->ps_distance_last != 0 && stk_data->ir_code == 0) {
			ret = stk3x3x_get_ir_reading(stk_data, STK_IRS_IT_REDUCE);
			if(ret > 0)
				stk_data->ir_code = ret;
		}
		return ret;
	}
	return 0;
}
#endif  /* #if (defined(STK_IRS) && defined(STK_POLL_ALS)) */


static int32_t stk3x3x_set_irs_it_slp(struct stk3x3x_data *stk_data, uint16_t *slp_time, int32_t ials_it_reduce)
{
	uint8_t irs_alsctrl;
	int32_t ret;

	irs_alsctrl = (stk_data->alsctrl_reg & 0x0F) - ials_it_reduce;
	switch(irs_alsctrl) {
	case 2:
		*slp_time = 1;
		break;
	case 3:
		*slp_time = 2;
		break;
	case 4:
		*slp_time = 3;
		break;
	case 5:
		*slp_time = 6;
		break;
	case 6:
		*slp_time = 12;
		break;
	case 7:
		*slp_time = 24;
		break;
	case 8:
		*slp_time = 48;
		break;
	case 9:
		*slp_time = 96;
		break;
	case 10:
		*slp_time = 192;
		break;
	default:
		SENSOR_LOG_ERROR("unknown ALS IT=0x%x\n", irs_alsctrl);
		ret = -EINVAL;
		return ret;
	}
	irs_alsctrl |= (stk_data->alsctrl_reg & 0xF0);
	ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, STK_ALSCTRL_REG, irs_alsctrl);
	if (ret < 0) {
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}
	return 0;
}

static int32_t stk3x3x_get_ir_reading(struct stk3x3x_data *stk_data, int32_t als_it_reduce)
{
	int32_t word_data, ret;
	uint8_t w_reg, retry = 0;
	uint16_t irs_slp_time = 100;
	unsigned char value[2];

	ret = stk3x3x_set_irs_it_slp(stk_data, &irs_slp_time, als_it_reduce);
	if(ret < 0)
		goto irs_err_i2c_rw;

	ret = stk3x3x_get_state(stk_data);
	if(ret < 0)
		goto irs_err_i2c_rw;

	w_reg = ret | STK_STATE_EN_IRS_MASK;
	ret = stk3x3x_set_state(stk_data, w_reg);
	if(ret < 0)
		goto irs_err_i2c_rw;
	msleep(irs_slp_time);

	do {
		usleep_range(3000, 4000);
		ret = stk3x3x_get_flag(stk_data);
		if (ret < 0)
			goto irs_err_i2c_rw;
		retry++;
	} while(retry < 10 && ((ret&STK_FLG_IR_RDY_MASK) == 0));

	if(retry == 10) {
		SENSOR_LOG_ERROR("ir data is not ready for a long time\n");
		ret = -EINVAL;
		goto irs_err_i2c_rw;
	}

	ret = stk3x3x_set_flag(stk_data, ret, STK_FLG_IR_RDY_MASK);
	if (ret < 0)
		goto irs_err_i2c_rw;

	ret = stk3x3x_i2c_read_data(stk_data->client, STK_DATA1_IR_REG, 2, &value[0]);
	if(ret < 0) {
		SENSOR_LOG_ERROR("fail, ret=0x%x\n", ret);
		goto irs_err_i2c_rw;
	}
	word_data = ((value[0]<<8) | value[1]);

	ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, STK_ALSCTRL_REG, stk_data->alsctrl_reg );
	if (ret < 0) {
		SENSOR_LOG_ERROR("write i2c error\n");
		goto irs_err_i2c_rw;
	}

	return word_data;

irs_err_i2c_rw:
	return ret;
}

#ifdef STK_CHK_REG
static int stk3x3x_chk_reg_valid(struct stk3x3x_data *stk_data)
{
	unsigned char value[9];
	int err;
	/*
	uint8_t cnt;

	for(cnt=0;cnt<9;cnt++)
	{
	    value[cnt] = stk3x3x_i2c_smbus_read_byte_data(stk_data->client, (cnt+1));
	    if(value[cnt] < 0)
	    {
	        SENSOR_LOG_ERROR("fail, ret=%d", value[cnt]);
	        return value[cnt];
	    }
	}
	*/
	err = stk3x3x_i2c_read_data(stk_data->client, STK_PSCTRL_REG, 9, &value[0]);
	if(err < 0) {
		SENSOR_LOG_ERROR("fail, ret=%d\n", err);
		return err;
	}

	if(value[0] != stk_data->psctrl_reg) {
		SENSOR_LOG_ERROR("invalid reg 0x01=0x%2x\n", value[0]);
		return 0xFF;
	}
#ifdef STK_IRS
	if((value[1] != stk_data->alsctrl_reg) && (value[1] != (stk_data->alsctrl_reg - STK_IRS_IT_REDUCE))
	   && (value[1] != (stk_data->alsctrl_reg - STK_ALS_READ_IRS_IT_REDUCE)))
#else
	if((value[1] != stk_data->alsctrl_reg) && (value[1] != (stk_data->alsctrl_reg - STK_ALS_READ_IRS_IT_REDUCE)))
#endif
	{
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "invalid reg 0x02=0x%2x\n", value[1]);
		return 0xFF;
	}
	if(value[2] != stk_data->ledctrl_reg) {
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "invalid reg 0x03=0x%2x\n", value[2]);
		return 0xFF;
	}
	if(value[3] != stk_data->int_reg) {
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "invalid reg 0x04=0x%2x\n", value[3]);
		return 0xFF;
	}
	if(value[4] != stk_data->wait_reg) {
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "invalid reg 0x05=0x%2x\n", value[4]);
		return 0xFF;
	}
	if(value[5] != ((stk_data->ps_thd_h & 0xFF00) >> 8)) {
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "invalid reg 0x06=0x%2x\n", value[5]);
		return 0xFF;
	}
	if(value[6] != (stk_data->ps_thd_h & 0x00FF)) {
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "invalid reg 0x07=0x%2x\n", value[6]);
		return 0xFF;
	}
	if(value[7] != ((stk_data->ps_thd_l & 0xFF00) >> 8)) {
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "invalid reg 0x08=0x%2x\n", value[7]);
		return 0xFF;
	}
	if(value[8] != (stk_data->ps_thd_l & 0x00FF)) {
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "invalid reg 0x09=0x%2x\n", value[8]);
		return 0xFF;
	}

	return 0;
}

static int stk3x3x_validate_n_handle(struct i2c_client *client)
{
	struct stk3x3x_data *stk_data = i2c_get_clientdata(client);
	int err;

	err = stk3x3x_chk_reg_valid(stk_data);
	if(err < 0) {
		SENSOR_LOG_ERROR("stk3x3x_chk_reg_valid fail: %d\n", err);
		return err;
	}

	if(err == 0xFF) {
 		SENSOR_LOG_INFO("Re-init chip\n");
		err = stk3x3x_software_reset(stk_data);
		if(err < 0)
			return err;
		err = stk3x3x_init_all_reg(stk_data);
		if(err < 0)
			return err;

		//stk_data->psa = 0;
		//stk_data->psi = 0xFFFF;
		stk3x3x_set_ps_thd_h(stk_data, stk_data->ps_thd_h);
		stk3x3x_set_ps_thd_l(stk_data, stk_data->ps_thd_l);
#ifdef STK_ALS_FIR
		memset(&stk_data->fir, 0x00, sizeof(stk_data->fir));
#endif
		return 0xFF;
	}
	return 0;
}
#endif /* #ifdef STK_CHK_REG */


static ssize_t stk_als_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	int32_t reading;
#ifdef STK_POLL_ALS
	reading = stk_data->als_code_last;
#else
	unsigned char value[2];
	int ret;
	ret = stk3x3x_i2c_read_data(stk_data->client, STK_DATA1_ALS_REG, 2, &value[0]);
	if(ret < 0) {
		SENSOR_LOG_ERROR("fail, ret=0x%x\n", ret);
		return ret;
	}
	reading = (value[0]<<8) | value[1];
#endif
	return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
}

#ifdef QUALCOMM_PLATFORM
static int stk_als_enable_set(struct sensors_classdev *sensors_cdev,
                              unsigned int enabled)
{
	struct stk3x3x_data *stk_data = container_of(sensors_cdev,
	                                struct stk3x3x_data, als_cdev);
	int err = 0;

	mutex_lock(&stk_data->io_lock);
	err = stk3x3x_enable_als(stk_data, enabled);
	mutex_unlock(&stk_data->io_lock);
	stk_data->als_en_hal = enabled?true:false;
	if (err < 0)
		return err;
	return 0;
}
#endif

static ssize_t stk_als_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	int32_t ret;

	ret = stk3x3x_get_state(stk_data);
	if(ret < 0)
		return ret;
	ret = (ret & STK_STATE_EN_ALS_MASK)?1:0;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t stk_als_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *stk_data = dev_get_drvdata(dev);
	uint8_t en;
	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else {
		SENSOR_LOG_ERROR("invalid value %d\n", *buf);
		return -EINVAL;
	}
	SENSOR_LOG_INFO("Enable ALS : %d\n",en);
	mutex_lock(&stk_data->io_lock);
	stk3x3x_enable_als(stk_data, en);
	mutex_unlock(&stk_data->io_lock);
	stk_data->als_en_hal = en?true:false;
	return size;
}
static ssize_t stk_als_dev_init_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
	return 1;
}

static ssize_t stk_als_dev_init_store(struct device *dev,
                                      struct device_attribute *attr,
                                      const char *buf, size_t size)
{
	int err;
	struct stk3x3x_data *stk_data = dev_get_drvdata(dev);

#ifdef STK_ALSPS_CALIBRATION
	err = stk3x3x_load_als_calibration_config(stk_data);
	if (err < 0) {
		SENSOR_LOG_ERROR("read factory cal parameters failed\n");
	}
	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "cali factor = %d\n", stk_data->lux_factor);
#endif

	return size;
}

static ssize_t stk_als_lux_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *stk_data = dev_get_drvdata(dev);
	int32_t als_reading;
	uint32_t als_lux;
	als_reading = stk3x3x_get_als_reading(stk_data);
	als_lux = stk_alscode2lux(stk_data, als_reading);
	return scnprintf(buf, PAGE_SIZE, "%d lux\n", als_lux);
}

static ssize_t stk_als_lux_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 16, &value);
	if(ret < 0) {
		SENSOR_LOG_ERROR("kstrtoul failed, ret=0x%x\n", ret);
		return ret;
	}
	stk_als_report(stk_data, value);
	return size;
}


static ssize_t stk_als_transmittance_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	int32_t transmittance;
	transmittance = stk_data->als_transmittance;
	return scnprintf(buf, PAGE_SIZE, "%d\n", transmittance);
}


static ssize_t stk_als_transmittance_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0) {
		SENSOR_LOG_ERROR("kstrtoul failed, ret=0x%x\n", ret);
		return ret;
	}
	stk_data->als_transmittance = value;
	return size;
}

static ssize_t stk_als_ir_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	int32_t reading;
	reading = stk3x3x_get_ir_reading(stk_data, STK_IRS_IT_REDUCE);
	return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
}

static ssize_t stk_als_chip_id_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", DEVICE_CHIP_NAME);
}

#ifdef STK_ALSPS_CALIBRATION
static ssize_t stk_code_debug_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
    return sprintf(buf, "%d\n", stk_data->code_debug);
}

static ssize_t stk_code_debug_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t size)
{
	int ret ;
	int val = 0;
	struct stk3x3x_data *stk_data = dev_get_drvdata(dev);
	ret = kstrtoint(buf, 10, &val);
	stk_data->code_debug = val;
	return size;
}

static ssize_t stk_als_fac_cal_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", stk_data->als_fac_cal);
}

static ssize_t stk_als_fac_calibrate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", stk_data->lux_factor);

}

static ssize_t stk_als_fac_calibrate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{

	int err;
	int value = 0;

	struct stk3x3x_data *stk_data = dev_get_drvdata(dev);
	if (IS_ERR_OR_NULL(buf)) {
		SENSOR_LOG_ERROR("buf NULL.\n");
		return -EINVAL;
	}
	memcpy(&value, buf, sizeof(value));
	err = stk3x3x_als_calibration(stk_data, value);
	if (err < 0) {
		SENSOR_LOG_ERROR("stk als calibrate: failed.\n");
		return err;
	}
	return size;
}

static ssize_t stk_als_calibrate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{

	int err;
	struct stk3x3x_data *stk_data = dev_get_drvdata(dev);
	int value = 0;

	if (IS_ERR_OR_NULL(buf)) {
		SENSOR_LOG_ERROR("NULL.\n");
		return -EINVAL;
	}

	err = kstrtoint(buf, 0, &value);
	if (err < 0) {
		SENSOR_LOG_ERROR("kstrtoint failed\n");
		return err;
	}

	err = stk3x3x_als_calibration(stk_data, value);
	if (err < 0) {
		SENSOR_LOG_ERROR("stk als calibrate: failed.\n");
		return err;
	}

	return size;
}

static ssize_t stk_als_calibrate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", stk_data->lux_factor);
}
#endif  /*STK_ALSPS_CALIBRATION*/

static ssize_t stk_als_factory_lux_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *stk_data = dev_get_drvdata(dev);
	int count;
	int32_t als_prev_lux,flag_reg,retry_times = 10;
	uint32_t als_cal_lux;

	SENSOR_LOG_INFO("Enable ALS");
	mutex_lock(&stk_data->io_lock);
	stk3x3x_enable_als(stk_data, 1);
	mutex_unlock(&stk_data->io_lock);

	do {
		msleep(100);
		flag_reg = stk3x3x_get_flag(stk_data);
		retry_times--;
	} while(retry_times && !(flag_reg&STK_FLG_ALSDR_MASK));
	if(retry_times <= 0) {
		SENSOR_LOG_ERROR("get flag not als mask");
		count = -ENXIO;
		goto read_value_err;
	}

	als_prev_lux = stk3x3x_get_als_reading(stk_data);
	SENSOR_LOG_INFO("als_prev_lux = %d\n", als_prev_lux);
	if(als_prev_lux < 0) {
		SENSOR_LOG_ERROR("read lux failed");
		count = -ENXIO;
		goto read_value_err;
	}

	als_cal_lux = stk_alscode2lux(stk_data, als_prev_lux);
	SENSOR_LOG_INFO("als_cal_lux = %d\n", als_cal_lux);
	if(stk_data->als_fac_cal == 0) {
		SENSOR_LOG_ERROR("Factory lux not cailbrate\n");
	}

	count = sprintf(buf, "%d", als_cal_lux);
	SENSOR_LOG_INFO("als_cal_lux = %d ,als_lux =%d",als_cal_lux, als_prev_lux);

read_value_err:
	SENSOR_LOG_INFO("Disable ALS\n");
	mutex_lock(&stk_data->io_lock);
	stk3x3x_enable_als(stk_data, 0);
	mutex_unlock(&stk_data->io_lock);
	
	return count;
	
}

#ifdef STK_ALS_FIR
static ssize_t stk_als_firlen_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	int len = atomic_read(&stk_data->firlength);

	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "len = %2d, idx = %2d\n", len, stk_data->fir.idx);
	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "sum = %5d, ave = %5d\n", stk_data->fir.sum, stk_data->fir.sum/len);
	return scnprintf(buf, PAGE_SIZE, "%d\n", len);
}


static ssize_t stk_als_firlen_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	uint64_t value = 0;
	int ret;
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	ret = kstrtoull(buf, 10, &value);
	if(ret < 0) {
		SENSOR_LOG_ERROR("kstrtoull failed, ret=0x%x\n", ret);
		return ret;
	}

	if(value > MAX_FIR_LEN) {
		SENSOR_LOG_ERROR("firlen exceed maximum filter length\n");
	} else if (value < 1) {
		atomic_set(&stk_data->firlength, 1);
		memset(&stk_data->fir, 0x00, sizeof(stk_data->fir));
	} else {
		atomic_set(&stk_data->firlength, value);
		memset(&stk_data->fir, 0x00, sizeof(stk_data->fir));
	}
	return size;
}
#endif  /* #ifdef STK_ALS_FIR */
static ssize_t stk_als_flush_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	int count;
	int32_t reading = 0, reading_lux;
	mutex_lock(&stk_data->io_lock);

	reading = stk3x3x_get_als_reading(stk_data);
	if(reading < 0)
		return -1;

	reading_lux = stk_alscode2lux(stk_data, reading);
	stk_als_report(stk_data, reading_lux);
	count = sprintf(buf, "%d", stk_data->als_lux_last);
	mutex_unlock(&stk_data->io_lock);
	return count;
}

struct device_attribute stk_als_attrs[] = {
	__ATTR(debug, 0664,stk_code_debug_show,stk_code_debug_store),
	__ATTR(enable,0664,stk_als_enable_show,stk_als_enable_store),
	__ATTR(dev_init,0664, stk_als_dev_init_show,stk_als_dev_init_store),
	__ATTR(lux,0664,stk_als_lux_show,stk_als_lux_store),
	__ATTR(code, 0444, stk_als_code_show, NULL),
	__ATTR(transmittance,0664,stk_als_transmittance_show,stk_als_transmittance_store),
	__ATTR(ircode,0444,stk_als_ir_code_show,NULL),
	__ATTR(chip_name, 0440, stk_als_chip_id_show, NULL),
	__ATTR(flush, 0664, stk_als_flush_show, NULL),
#ifdef STK_ALSPS_CALIBRATION
	__ATTR(fac_calibrate, 0664, stk_als_fac_calibrate_show, stk_als_fac_calibrate_store),
	__ATTR(als_calibrate, 0664,stk_als_calibrate_show, stk_als_calibrate_store),
	__ATTR(calibrate, 0664, stk_als_fac_cal_show, NULL),
	__ATTR(light_value, 0664, stk_als_factory_lux_show, NULL),
#endif
#ifdef STK_ALS_FIR
	__ATTR(firlen,0664,stk_als_firlen_show,stk_als_firlen_store),
#endif
};


static ssize_t stk_ps_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	uint32_t reading;
	reading = stk3x3x_get_ps_reading(stk_data);
	return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
}

#ifdef QUALCOMM_PLATFORM
static int stk_ps_enable_set(struct sensors_classdev *sensors_cdev,
                             unsigned int enabled)
{
	struct stk3x3x_data *stk_data = container_of(sensors_cdev,
	                                struct stk3x3x_data, ps_cdev);
	int err;

	mutex_lock(&stk_data->io_lock);
	err = stk3x3x_enable_ps(stk_data, enabled, 0);
	mutex_unlock(&stk_data->io_lock);

	if (err < 0)
		return err;
	return 0;
}
#endif


#ifdef STK_ALSPS_CALIBRATION
static ssize_t stk_prox_dev_init_show(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{
	return 1;
}

static ssize_t stk_prox_dev_init_store(struct device *dev,
                                       struct device_attribute *attr,
                                       const char *buf, size_t size)
{
#if 0
	int val = 0;
	int rc;
	uint16_t data[2] = {0};
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	rc = kstrtoint(buf, 0, &val);
	if (val) {
		rc = sensor_read_file(PS_CAL_FILE_PATH, (char*)data, sizeof(data));
		if (rc < 0) {
			SENSOR_LOG_ERROR("read file error\n");
			return -1;
		}

		stk_data->cali_near_increment = data[0];
		SENSOR_LOG_ERROR("chip->cali_near_increment = %d\n",
		                 stk_data->cali_near_increment);
	}

	stk_data->stk_ht_n_ct = stk_data->cali_near_increment;
	stk_data->stk_lt_n_ct = stk_data->stk_ht_n_ct - PS_FAR_DIFFER_VALUE;
	SENSOR_LOG_ERROR("stk_ht_n_ct = %d, stk_lt_n_ct = %d\n",
	                 stk_data->stk_ht_n_ct, stk_data->stk_lt_n_ct);
	return rc;
#else
	return size;
#endif
}
#endif

static ssize_t stk_ps_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int32_t ret;
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);

	ret = stk3x3x_get_state(stk_data);
	if(ret < 0)
		return ret;
	ret = (ret & STK_STATE_EN_PS_MASK)?1:0;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t stk_ps_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	uint8_t en;
	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else {
		SENSOR_LOG_ERROR(", invalid value %d\n", *buf);
		return -EINVAL;
	}
	SENSOR_LOG_INFO(" Enable PS : %d\n", en);
	mutex_lock(&stk_data->io_lock);
	stk3x3x_enable_ps(stk_data, en, 0);
	mutex_unlock(&stk_data->io_lock);
	return size;
}


static ssize_t stk_ps_enable_aso_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int32_t ret;
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);

	ret = stk3x3x_i2c_smbus_read_byte_data(stk_data->client,STK_STATE_REG);
	ret = (ret & STK_STATE_EN_ASO_MASK)?1:0;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t stk_ps_enable_aso_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	uint8_t en;
	int32_t ret;
	uint8_t w_state_reg;

	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else {
		SENSOR_LOG_ERROR(", invalid value %d\n", *buf);
		return -EINVAL;
	}
	SENSOR_LOG_INFO(" Enable PS ASO : %d\n", en);

	ret = stk3x3x_i2c_smbus_read_byte_data(stk_data->client, STK_STATE_REG);
	if (ret < 0) {
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}
	w_state_reg = (uint8_t)(ret & (~STK_STATE_EN_ASO_MASK));
	if(en)
		w_state_reg |= STK_STATE_EN_ASO_MASK;

	ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, STK_STATE_REG, w_state_reg);
	if (ret < 0) {
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}

	return size;
}


static ssize_t stk_ps_offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	int32_t word_data;
	unsigned char value[2];
	int ret;

	ret = stk3x3x_i2c_read_data(stk_data->client, STK_DATA1_OFFSET_REG, 2, &value[0]);
	if(ret < 0) {
		SENSOR_LOG_ERROR("fail, ret=0x%x", ret);
		return ret;
	}
	word_data = (value[0]<<8) | value[1];

	return scnprintf(buf, PAGE_SIZE, "%d\n", word_data);
}

static ssize_t stk_ps_offset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	unsigned long offset = 0;
	int ret;
	unsigned char val[2];

	ret = kstrtoul(buf, 10, &offset);
	if(ret < 0) {
		SENSOR_LOG_ERROR("kstrtoul failed, ret=0x%x\n", ret);
		return ret;
	}
	if(offset > 65535) {
		SENSOR_LOG_ERROR("invalid value, offset=%ld\n", offset);
		return -EINVAL;
	}

	val[0] = (offset & 0xFF00) >> 8;
	val[1] = offset & 0x00FF;
	ret = stk3x3x_i2c_write_data(stk_data->client, STK_DATA1_OFFSET_REG, 2, val);
	if(ret < 0) {
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}

	return size;
}


static ssize_t stk_ps_distance_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	int32_t dist=1;
	int32_t ret;

	ret = stk3x3x_get_flag(stk_data);
	if(ret < 0)
		return ret;
	dist = (ret & STK_FLG_NF_MASK)?1:0;
	stk_ps_report(stk_data, dist);
	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " ps input event=%d\n", dist);
	return scnprintf(buf, PAGE_SIZE, "%d\n", dist);
}


static ssize_t stk_ps_distance_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;

	ret = kstrtoul(buf, 10, &value);
	if(ret < 0) {
		SENSOR_LOG_ERROR("kstrtoul failed, ret=0x%x\n", ret);
		return ret;
	}
	stk_ps_report(stk_data, value);
	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " ps input event=%d\n",(int)value);
	return size;
}


static ssize_t stk_ps_code_thd_l_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int32_t ps_thd_l1_reg, ps_thd_l2_reg;
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	ps_thd_l1_reg = stk3x3x_i2c_smbus_read_byte_data(stk_data->client,STK_THDL1_PS_REG);
	if(ps_thd_l1_reg < 0) {
		SENSOR_LOG_ERROR("fail, err=0x%x", ps_thd_l1_reg);
		return -EINVAL;
	}
	ps_thd_l2_reg = stk3x3x_i2c_smbus_read_byte_data(stk_data->client,STK_THDL2_PS_REG);
	if(ps_thd_l2_reg < 0) {
		SENSOR_LOG_ERROR("fail, err=0x%x", ps_thd_l2_reg);
		return -EINVAL;
	}
	ps_thd_l1_reg = ps_thd_l1_reg<<8 | ps_thd_l2_reg;
	return scnprintf(buf, PAGE_SIZE, "%d\n", ps_thd_l1_reg);
}


static ssize_t stk_ps_code_thd_l_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0) {
		SENSOR_LOG_ERROR("kstrtoul failed, ret=0x%x\n", ret);
		return ret;
	}
	stk3x3x_set_ps_thd_l(stk_data, value);
	return size;
}

static ssize_t stk_ps_code_thd_h_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int32_t ps_thd_h1_reg, ps_thd_h2_reg;
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	ps_thd_h1_reg = stk3x3x_i2c_smbus_read_byte_data(stk_data->client,STK_THDH1_PS_REG);
	if(ps_thd_h1_reg < 0) {
		SENSOR_LOG_ERROR("fail, err=0x%x", ps_thd_h1_reg);
		return -EINVAL;
	}
	ps_thd_h2_reg = stk3x3x_i2c_smbus_read_byte_data(stk_data->client,STK_THDH2_PS_REG);
	if(ps_thd_h2_reg < 0) {
		SENSOR_LOG_ERROR("fail, err=0x%x", ps_thd_h2_reg);
		return -EINVAL;
	}
	ps_thd_h1_reg = ps_thd_h1_reg<<8 | ps_thd_h2_reg;
	return scnprintf(buf, PAGE_SIZE, "%d\n", ps_thd_h1_reg);
}

static ssize_t stk_ps_code_thd_h_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0) {
		SENSOR_LOG_ERROR("kstrtoul failed, ret=0x%x\n", ret);
		return ret;
	}
	stk3x3x_set_ps_thd_h(stk_data, value);
	return size;
}

static ssize_t stk_all_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int32_t ps_reg[0x23];
	uint8_t cnt;
	int len = 0;
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);

	for(cnt=0; cnt<0x20; cnt++) {
		ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(stk_data->client, (cnt));
		if(ps_reg[cnt] < 0) {
			SENSOR_LOG_ERROR("fail, ret=%d", ps_reg[cnt]);
			return -EINVAL;
		} else {
			SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "reg[0x%2X]=0x%2X\n", cnt, ps_reg[cnt]);
			len += scnprintf(buf+len, PAGE_SIZE-len, "[%2X]%2X,", cnt, ps_reg[cnt]);
		}
	}
	ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(stk_data->client, STK_PDT_ID_REG);
	if(ps_reg[cnt] < 0) {
		SENSOR_LOG_ERROR("fail, ret=%d", ps_reg[cnt]);
		return -EINVAL;
	}
	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "reg[0x%x]=0x%2X\n", STK_PDT_ID_REG, ps_reg[cnt]);

	cnt++;
	ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(stk_data->client, STK_RSRVD_REG);
	if(ps_reg[cnt] < 0) {
		SENSOR_LOG_ERROR("fail, ret=%d", ps_reg[cnt]);
		return -EINVAL;
	}
	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "reg[0x%x]=0x%2X\n", STK_RSRVD_REG, ps_reg[cnt]);

	cnt++;
	ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(stk_data->client, 0xE0);
	if(ps_reg[cnt] < 0) {
		SENSOR_LOG_ERROR("fail, ret=%d", ps_reg[cnt]);
		return -EINVAL;
	}
	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "reg[0xE0]=0x%2X\n", ps_reg[cnt]);
	len += scnprintf(buf+len, PAGE_SIZE-len, "[3E]%2X,[3F]%2X,[E0]%2X\n", ps_reg[cnt-2], ps_reg[cnt-1], ps_reg[cnt]);
	return len;
}

static ssize_t stk_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int32_t ps_reg[27];
	uint8_t cnt;
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	for(cnt=0; cnt<25; cnt++) {
		ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(stk_data->client, (cnt));
		if(ps_reg[cnt] < 0) {
			SENSOR_LOG_ERROR("fail, ret=%d", ps_reg[cnt]);
			return -EINVAL;
		} else {
			SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "reg[0x%2X]=0x%2X\n", cnt, ps_reg[cnt]);
		}
	}
	ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(stk_data->client, STK_PDT_ID_REG);
	if(ps_reg[cnt] < 0) {
		SENSOR_LOG_ERROR("fail, ret=%d", ps_reg[cnt]);
		return -EINVAL;
	}
	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "reg[0x%x]=0x%2X\n", STK_PDT_ID_REG, ps_reg[cnt]);
	cnt++;
	ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(stk_data->client, STK_RSRVD_REG);
	if(ps_reg[cnt] < 0) {
		SENSOR_LOG_ERROR("fail, ret=%d", ps_reg[cnt]);
		return -EINVAL;
	}
	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "reg[0x%x]=0x%2X\n", STK_RSRVD_REG, ps_reg[cnt]);

	return scnprintf(buf, PAGE_SIZE, "[PS=%2X] [ALS=%2X] [WAIT=0x%4Xms] [EN_ASO=%2X] [EN_AK=%2X] [NEAR/FAR=%2X] [FLAG_OUI=%2X] [FLAG_PSINT=%2X] [FLAG_ALSINT=%2X]\n",
	                 ps_reg[0]&0x01,(ps_reg[0]&0x02)>>1,((ps_reg[0]&0x04)>>2)*ps_reg[5]*6,(ps_reg[0]&0x20)>>5,
	                 (ps_reg[0]&0x40)>>6,ps_reg[16]&0x01,(ps_reg[16]&0x04)>>2,(ps_reg[16]&0x10)>>4,(ps_reg[16]&0x20)>>5);
}

static ssize_t stk_recv_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&stk_data->recv_reg));
}


static ssize_t stk_recv_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long value = 0;
	int ret;
	int32_t recv_data;
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);

	if((ret = kstrtoul(buf, 16, &value)) < 0) {
		SENSOR_LOG_ERROR("kstrtoul failed, ret=0x%x\n", ret);
		return ret;
	}
	recv_data = stk3x3x_i2c_smbus_read_byte_data(stk_data->client,value);
    SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "reg 0x%x=0x%x\n", (int)value, recv_data);
	atomic_set(&stk_data->recv_reg, recv_data);
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
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);

	for (i = 0; i < 2; i++)
		token[i] = strsep((char **)&buf, " ");
	if((ret = kstrtoul(token[0], 16, (unsigned long *)&(addr))) < 0) {
		SENSOR_LOG_ERROR("kstrtoul failed, ret=0x%x\n", ret);
		return ret;
	}
	if((ret = kstrtoul(token[1], 16, (unsigned long *)&(cmd))) < 0) {
		SENSOR_LOG_ERROR("kstrtoul failed, ret=0x%x\n", ret);
		return ret;
	}
	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " write reg 0x%x=0x%x\n", addr, cmd);

	ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, (unsigned char)addr, (unsigned char)cmd);
	if (0 != ret) {
		SENSOR_LOG_ERROR("stk3x3x_i2c_smbus_write_byte_data fail\n");
		return ret;
	}

	return size;
}

#ifdef STK_TUNE0
static ssize_t stk_ps_cali_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	int32_t word_data;
	unsigned char value[2];
	int ret;

	ret = stk3x3x_i2c_read_data(stk_data->client, 0x20, 2, &value[0]);
	if(ret < 0) {
		SENSOR_LOG_ERROR("fail, ret=0x%x", ret);
		return ret;
	}
	word_data = (value[0]<<8) | value[1];

	ret = stk3x3x_i2c_read_data(stk_data->client, 0x22, 2, &value[0]);
	if(ret < 0) {
		SENSOR_LOG_ERROR("fail, ret=0x%x", ret);
		return ret;
	}
	word_data += ((value[0]<<8) | value[1]);

	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "psi_set=%d, psa=%d,psi=%d, word_data=%d\n",
	                stk_data->psi_set, stk_data->psa, stk_data->psi, word_data);
#ifdef CALI_PS_EVERY_TIME
	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "boot HT=%d, LT=%d\n", stk_data->ps_high_thd_boot, stk_data->ps_low_thd_boot);
#endif
	return 0;
}

static ssize_t stk_ps_maxdiff_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;

	if((ret = kstrtoul(buf, 10, &value)) < 0) {
		SENSOR_LOG_ERROR("kstrtoul failed, ret=0x%x\n", ret);
		return ret;
	}
	stk_data->stk_max_min_diff = (int) value;
	return size;
}


static ssize_t stk_ps_maxdiff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%d\n", stk_data->stk_max_min_diff);
}

static ssize_t stk_ps_ltnct_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;

	if((ret = kstrtoul(buf, 10, &value)) < 0) {
		SENSOR_LOG_ERROR("kstrtoul failed, ret=0x%x\n", ret);
		return ret;
	}
	stk_data->stk_lt_n_ct = (int) value;
	return size;
}

static ssize_t stk_ps_ltnct_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%d\n", stk_data->stk_lt_n_ct);
}

static ssize_t stk_ps_htnct_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;

	if((ret = kstrtoul(buf, 10, &value)) < 0) {
		SENSOR_LOG_ERROR("kstrtoul failed, ret=0x%x\n", ret);
		return ret;
	}
	stk_data->stk_ht_n_ct = (int) value;
	return size;
}

static ssize_t stk_ps_htnct_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%d\n", stk_data->stk_ht_n_ct);
}
#endif  /* #ifdef STK_TUNE0 */

/* Nubia add for ps calibration */
#ifdef STK_ALSPS_CALIBRATION

static int32_t stk3x3x_ps_calibration_uncover(struct stk3x3x_data *stk_data)
{
	//step01: cali crosstalk
	uint32_t reading, ps_sum = 0, counter = 0;
	uint32_t err_cnt = 0;
#if 0
	if(stk_data->ps_enabled == false) {
		//enable ps
		ret = stk3x3x_get_state(stk_data);
		if(ret < 0)
			return ret;
		w_state_reg = ret;

		SENSOR_LOG_INFO("cur state=0x%x\n", w_state_reg);
		w_state_reg &= ~(STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK | STK_STATE_EN_AK_MASK);
		w_state_reg |= STK_STATE_EN_PS_MASK;
		w_state_reg |= STK_STATE_EN_WAIT_MASK;

		ret = stk3x3x_set_state(stk_data, w_state_reg);
		if(ret < 0)
			return ret;
	}
#endif
	while (counter < PS_CALI_COUNT) {
		msleep(60);
		reading = stk3x3x_get_ps_reading(stk_data);
		if(reading < 0) {
			SENSOR_LOG_ERROR("[read err]:ps%d = %d\n", counter, reading);
			err_cnt++;

			if(err_cnt > 5) {
				stk_data->cali_crosstalk = 0xFFFF;
				return -1;
			}
			continue;
		}

		SENSOR_LOG_ERROR("ps%d = %d\n", counter, reading);
		ps_sum += reading;
		counter++;
	}

	stk_data->cali_crosstalk = (ps_sum / PS_CALI_COUNT);

	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "prox crosstalk = %d\n", stk_data->cali_crosstalk);
	return 0;
}

static int32_t stk3x3x_ps_calibration_thres(struct stk3x3x_data *stk_data)
{
	//step01: cali near
	uint32_t reading, ps_sum = 0, counter = 0;
	uint16_t old_ct = 0;
	int rc;
	uint16_t prox_buf[2] = {0};
	int diff = 0;

	while (counter < PS_CALI_COUNT) {
		msleep(60);
		reading = stk3x3x_get_ps_reading(stk_data);
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "ps%d = %d\n", counter, reading);
		ps_sum += reading;
		counter++;
	}

	stk_data->cali_near = (ps_sum / PS_CALI_COUNT);
	diff = stk_data->cali_near - stk_data->cali_crosstalk;
	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "crosstalk = %d, near = %d, increment = %d\n", stk_data->cali_crosstalk, stk_data->cali_near, diff);

	if(diff < PS_THRESH_INCREMENT_MIN) {
		SENSOR_LOG_ERROR("cali_near_increment too little!\n");
		return -2;
	}

	if(diff > PS_THRESH_INCREMENT_MAX) {
		SENSOR_LOG_ERROR("cali_near_increment too big!\n");
		return -2;
	}

	stk_data->cali_near_increment = diff;
	old_ct = stk_data->cali_crosstalk;
#if 0
	//update ht, lt after calibration
	stk_data->stk_ht_n_ct = stk_data->cali_near_increment;
	stk_data->stk_lt_n_ct = stk_data->stk_ht_n_ct - PS_FAR_DIFFER_VALUE;

	stk_data->ps_thd_h = old_ct + stk_data->stk_ht_n_ct;
	stk_data->ps_thd_l = old_ct + stk_data->stk_lt_n_ct;

	stk3x3x_set_ps_thd_h(stk_data, stk_data->ps_thd_h);
	stk3x3x_set_ps_thd_l(stk_data, stk_data->ps_thd_l);

	SENSOR_LOG_INFO("HT=%d, LT=%d\n", stk_data->ps_thd_h, stk_data->ps_thd_l);
#endif
	stk_data->ps_high_thd_boot = old_ct + stk_data->stk_h_ht;
	stk_data->ps_low_thd_boot = old_ct + stk_data->stk_h_lt;

	prox_buf[0] = stk_data->cali_near_increment;
	rc = sensor_write_file(PS_CAL_FILE_PATH, (char *)prox_buf, sizeof(prox_buf));
	if (rc < 0) {
		SENSOR_LOG_ERROR("PS CAL write file fail, rc = %d\n", rc);
		return -1;
	}

	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "stk_ht_n_ct:%d, stk_lt_n_ct:%d \n",
	                stk_data->stk_ht_n_ct, stk_data->stk_lt_n_ct);


	stk_increment_init_flag = false;
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
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", stk_data->prox_debug);
}


static int stk_prox_set_debug_mode(struct stk3x3x_data *stk_data, uint8_t enable)
{
	int reading;
	uint8_t org_flag_reg;

	if(stk_data->prox_debug == enable) {
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " prox_debug = %d, debug_enable = %d\n", stk_data->prox_debug, enable);
		return 0;
	}

	if (enable) {
		stk_data->prox_debug = true;
		stk_data->debug_cnt = 0;
//      disable_irq(stk_data->irq);
		if(stk_data->ps_cali_timer_run == false) {
			stk_data->ps_cali_timer_run = true;
			hrtimer_start(&stk_data->ps_cali_timer, stk_data->ps_cali_delay, HRTIMER_MODE_REL);
		}
		SENSOR_LOG_INFO("debuging...\n");

		if(stk_data->ps_enabled) {
			org_flag_reg = stk3x3x_get_flag(stk_data);
			if(org_flag_reg < 0)
				return -EINVAL;

			if(!(org_flag_reg&STK_FLG_PSDR_MASK)) {
				//SENSOR_LOG_INFO(" ps is not ready\n");
				return -EINVAL;
			}

			reading = stk3x3x_get_ps_reading(stk_data);
			if(reading >= 0) {
				input_report_rel(stk_data->ps_input_dev, REL_RZ, reading);
				input_sync(stk_data->ps_input_dev);
			}

		}
	} else {
		stk_data->prox_debug = false;
		if(stk_data->ps_cali_timer_run == true) {
			stk_data->ps_cali_timer_run = false;
			hrtimer_cancel(&stk_data->ps_cali_timer);
			cancel_work_sync(&stk_data->stk_ps_cali_work);
		}
//      enable_irq(stk_data->irq);
	}

	return 0;
}

static ssize_t stk_prox_debug_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t size)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	uint8_t en;
	int ret = 0;
	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else {
		SENSOR_LOG_ERROR(", invalid value %d\n", *buf);
		return -EINVAL;
	}
	SENSOR_LOG_INFO(" Enable PS Debug: %d\n", en);
	mutex_lock(&stk_data->io_lock);
	ret = stk_prox_set_debug_mode(stk_data, en);
	mutex_unlock(&stk_data->io_lock);
	if(ret < 0) {
		SENSOR_LOG_INFO(", [debug mode]ps is not ready!\n");
	}
	return size;
}

static ssize_t stk_prox_thres_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	ret = stk3x3x_ps_calibration_thres(stk_data);

	if(ret < 0) {
		return sprintf(buf, "%d\n", ret);
	} else {
		return sprintf(buf, "%d\n", stk_data->cali_near);
	}
}

static ssize_t stk_prox_thres_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t size)
{
	int val;
	int rc;
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	rc = kstrtoint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val) {
		rc = stk3x3x_ps_calibration_thres(stk_data);
	}
	return rc;
}

static ssize_t stk_prox_uncover_cal_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{

	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	stk3x3x_ps_calibration_uncover(stk_data);
	return sprintf(buf, "%d\n", stk_data->cali_crosstalk);
}
static ssize_t stk_prox_uncover_cal_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t size)
{
	int val;
	int rc;
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	SENSOR_LOG_INFO("uncover enter\n");
	rc = kstrtoint(buf, 0, &val);
	if (val) {
		/*@ not use, uncover_cal_show will called */
		rc = stk3x3x_ps_calibration_uncover(stk_data);
		if (rc < 0) {
			SENSOR_LOG_ERROR("*#777# calibrate fail\n");
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
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	rc = kstrtoint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val) {
		prox_buf[0] = stk_data->cali_near_increment;
		rc = sensor_write_file(PS_CAL_FILE_PATH, (char *)prox_buf, sizeof(prox_buf));
		if (rc < 0) {
			SENSOR_LOG_ERROR("write file fail\n");
			return rc;
		}

		SENSOR_LOG_INFO("stk_ht_n_ct:%d, stk_lt_n_ct:%d \n",
		                stk_data->stk_ht_n_ct, stk_data->stk_lt_n_ct);
	}
#else
	return size;
#endif
}


static void stk_ps_cali_work_func(struct work_struct *work)
{
	struct stk3x3x_data *stk_data = container_of(work, struct stk3x3x_data, stk_ps_cali_work);
	uint32_t reading;
	uint8_t org_flag_reg;
	stk_data->debug_cnt++;

	if(stk_data->ps_enabled) {
		org_flag_reg = stk3x3x_get_flag(stk_data);
		if(org_flag_reg < 0)
			return;

		if(!(org_flag_reg&STK_FLG_PSDR_MASK)) {
			SENSOR_LOG_ERROR(" ps is not ready\n");
			return;
		}

		reading = stk3x3x_get_ps_reading(stk_data);
		if(stk_data->prox_debug) {
			if(stk_data->debug_cnt %10 == 9)
				SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " ps debug value: %d\n", reading);
			input_report_rel(stk_data->ps_input_dev, REL_RZ, reading);
			input_sync(stk_data->ps_input_dev);
		}
	}

	return;
}

static enum hrtimer_restart stk_ps_cali_timer_func(struct hrtimer *timer)
{
	struct stk3x3x_data *stk_data = container_of(timer, struct stk3x3x_data, ps_cali_timer);
	queue_work(stk_data->stk_ps_cali_wq, &stk_data->stk_ps_cali_work);
	hrtimer_forward_now(&stk_data->ps_cali_timer, stk_data->ps_cali_delay);
	return HRTIMER_RESTART;
}

static ssize_t stk_ps_distance_flush_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *stk_data =  dev_get_drvdata(dev);
	int32_t dist=1;
	int32_t ret;

	ret = stk3x3x_get_flag(stk_data);
	if(ret < 0)
		return ret;
	dist = (ret & STK_FLG_NF_MASK)?1:0;

	stk_data->ps_distance_last = dist;
	input_report_rel(stk_data->ps_input_dev, REL_RZ, dist? 10:3);
	input_sync(stk_data->ps_input_dev);

	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " ps flush input event=%d\n", dist);
	return scnprintf(buf, PAGE_SIZE, "%d\n", dist);
}

#endif
/* Nubia add end */

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

#ifdef STK_ALSPS_CALIBRATION
	__ATTR(prox_init, 0664, stk_prox_dev_init_show, stk_prox_dev_init_store),
	__ATTR(chip_name, 0440, stk_prox_chip_id_show, NULL),
	__ATTR(prox_debug, 0644, stk_prox_debug_show,  stk_prox_debug_store),
	__ATTR(prox_thres, 0644, stk_prox_thres_show, stk_prox_thres_store),
	__ATTR(prox_offset_cal, 0644, stk_prox_uncover_cal_show, stk_prox_uncover_cal_store),
	__ATTR(prox_thres_max,0440, stk_prox_max_thres_show, NULL),
	__ATTR(prox_thres_min, 0440, stk_prox_min_thres_show, NULL),
	__ATTR(prox_data_max, 0440, stk_prox_max_data_show, NULL),
	__ATTR(prox_uncover_min, 0440, stk_prox_uncover_data_min_show, NULL),
	__ATTR(prox_uncover_max, 0440, stk_prox_uncover_data_max_show, NULL),
	__ATTR(prox_thres_to_persist, 0220, NULL, stk_prox_thres_to_persist_store),
	__ATTR(prox_offset, 0440, stk_ps_offset_show, NULL),
#endif
	__ATTR(prox_value,0440,stk_ps_distance_flush_show, NULL),
};

static int stk_ps_val(struct stk3x3x_data *stk_data)
{
	int32_t lii = 12, psoff_thd = 1000;
	unsigned char value[4]= {0};
	int ret, i;
	bool out_of_range = false;
	unsigned char buf[2];
	uint16_t psoff[4]= {0};


	ret = stk3x3x_i2c_read_data(stk_data->client, 0x34, 4, value);
	if(ret < 0) {
		SENSOR_LOG_ERROR("fail, ret=0x%x", ret);
		return ret;
	}

	for (i = 0; i < 4; i++) {
		if (value[i] >= lii) {
			out_of_range = true;
			break;
		}
	}

	for(i = 0; i < 4; i++) {
		ret = stk3x3x_i2c_read_data(stk_data->client, (0x24 + i*2), 2, &buf[0]);
		if(ret < 0) {
			SENSOR_LOG_ERROR("fail, ret=0x%x", ret);
			return ret;
		}
		psoff[i] = (buf[0]<<8) | buf[1];
		if(psoff[i] >= psoff_thd) {
			out_of_range = true;
			break;
		}
	}

	if((stk_data->data_count%20)==5) {
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "bgir:%d, %d, %d, %d\n", value[0], value[1], value[2], value[3]);
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "psoff:%d, %d, %d, %d\n", psoff[0], psoff[1], psoff[2], psoff[3]);
	}

	if(out_of_range == true) {
		return 0xFFFF;
	}
	return 0;
}

#ifdef STK_TUNE0
static int stk_ps_tune_zero_final(struct stk3x3x_data *stk_data)
{
	int ret;

	stk_data->tune_zero_init_proc = false;
	ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, STK_INT_REG, stk_data->int_reg);
	if (ret < 0) {
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}

	ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, STK_STATE_REG, 0);
	if (ret < 0) {
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}

	if(stk_data->data_count == -1) {
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " exceed limit\n");
		hrtimer_cancel(&stk_data->ps_tune0_timer);
		return 0;
	}

	stk_data->psa = stk_data->ps_stat_data[0];
	stk_data->psi = stk_data->ps_stat_data[2];

#ifdef CALI_PS_EVERY_TIME
	stk_data->ps_high_thd_boot = stk_data->ps_stat_data[1] + stk_data->stk_h_ht;
	stk_data->ps_low_thd_boot = stk_data->ps_stat_data[1] + stk_data->stk_h_lt;
	stk_data->ps_thd_h = stk_data->ps_high_thd_boot ;
	stk_data->ps_thd_l = stk_data->ps_low_thd_boot ;
#else
	stk_data->ps_thd_h = stk_data->ps_stat_data[1] + stk_data->stk_ht_n_ct;
	stk_data->ps_thd_l = stk_data->ps_stat_data[1] + stk_data->stk_lt_n_ct;
#endif
	stk3x3x_set_ps_thd_h(stk_data, stk_data->ps_thd_h);
	stk3x3x_set_ps_thd_l(stk_data, stk_data->ps_thd_l);
	stk_data->boot_cali = 1;
	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " set HT=%d,LT=%d\n", stk_data->ps_thd_h,  stk_data->ps_thd_l);
	hrtimer_cancel(&stk_data->ps_tune0_timer);
	return 0;
}

static int32_t stk_tune_zero_get_ps_data(struct stk3x3x_data *stk_data)
{
	uint32_t ps_adc;
	int ret;

	ret = stk_ps_val(stk_data);
	if(ret == 0xFFFF) {
		stk_data->data_count = -1;
		stk_ps_tune_zero_final(stk_data);
		return 0;
	}

	ps_adc = stk3x3x_get_ps_reading(stk_data);
	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " ps_adc #%d=%d\n", stk_data->data_count, ps_adc);
	if(ps_adc < 0)
		return ps_adc;

	stk_data->ps_stat_data[1]  +=  ps_adc;
	if(ps_adc > stk_data->ps_stat_data[0])
		stk_data->ps_stat_data[0] = ps_adc;
	if(ps_adc < stk_data->ps_stat_data[2])
		stk_data->ps_stat_data[2] = ps_adc;
	stk_data->data_count++;

	if(stk_data->data_count == 5) {
		stk_data->ps_stat_data[1]  /= stk_data->data_count;
		stk_ps_tune_zero_final(stk_data);
	}

	return 0;
}

#ifndef QUALCOMM_PLATFORM
static int stk_ps_tune_zero_init(struct stk3x3x_data *stk_data)
{
	int32_t ret = 0;
	uint8_t w_state_reg;

#ifdef CALI_PS_EVERY_TIME
	stk_data->ps_high_thd_boot = stk_data->ps_thd_h;
	stk_data->ps_low_thd_boot = stk_data->ps_thd_l;
	if(stk_data->ps_high_thd_boot <= 0) {
		stk_data->ps_high_thd_boot = STK_DEFAULT_HT;
		stk_data->ps_low_thd_boot = STK_DEFAULT_LT;
	}
#endif

	stk_data->psi_set = 0;
	stk_data->ps_stat_data[0] = 0;
	stk_data->ps_stat_data[2] = 0xFFFF;
	stk_data->ps_stat_data[1] = 0;
	stk_data->data_count = 0;
	stk_data->boot_cali = 0;
	stk_data->tune_zero_init_proc = true;
	stk_data->cali_crosstalk = 0xFFFF;

	ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, STK_INT_REG, 0);
	if (ret < 0) {
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}

	w_state_reg = (STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK);
	ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, STK_STATE_REG, w_state_reg);
	if (ret < 0) {
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}
	hrtimer_start(&stk_data->ps_tune0_timer, stk_data->ps_tune0_delay*2, HRTIMER_MODE_REL);

	return 0;
}
#endif

static int stk_ps_tune_zero_func_fae(struct stk3x3x_data *stk_data)
{
	int32_t word_data, flag;
	int ret, diff;
	unsigned char value[2];

#ifdef CTTRACKING
	uint16_t ct_value = 0;
#endif
	uint32_t high_thd, low_thd;


#ifdef CALI_PS_EVERY_TIME
	if(!(stk_data->ps_enabled))
#else
	if(stk_data->psi_set || !(stk_data->ps_enabled))
#endif
	{
		return 0;
	}


	stk_data->data_count++;
	if(stk_data->data_count < 3) {
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "count=%d\n", stk_data->data_count);
		return 0;
	}

	if(stk_data->data_count > 10000) {
		stk_data->data_count = 10;
	} else if((stk_data->data_count%20)==6) {
		flag = stk3x3x_get_flag(stk_data);
		if(flag < 0) {
			return flag;
		}

		ret = stk3x3x_i2c_read_data(stk_data->client, 0x11, 2, &value[0]);
		if(ret < 0) {
			SENSOR_LOG_ERROR("fail, ret=0x%x", ret);
			return ret;
		}
		word_data = (value[0]<<8) | value[1];

		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "cur ps=%d, flag=0x%x, dis=%d\n", word_data, flag, stk_data->ps_distance_last);
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "psi_set=%d, psi=%d, smudge=%d\n", stk_data->psi_set, stk_data->psi, stk_data->ps_thd_update);
	} else if((stk_data->data_count%200)==9) {
		high_thd = stk3x3x_get_ps_thd_h(stk_data);
		if (high_thd < 0) {
			SENSOR_LOG_DEBUG("fail, error: %d\n", high_thd);
			return high_thd;
		}

		low_thd = stk3x3x_get_ps_thd_l(stk_data);
		if (low_thd < 0) {
			SENSOR_LOG_DEBUG("fail, error: %d\n", low_thd);
			return low_thd;
		}
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "cur ht=%d, lt=%d\n", high_thd, low_thd);
	} else if((stk_data->data_count%500)==8) {
		stk3x3x_dump_reg(stk_data);
	}


#ifdef CTTRACKING
	if ((stk_data->psi_set != 0)) {
		if (stk_data->ps_distance_last == 1) {
			ret = stk_ps_val(stk_data);

			if (ret == 0) {
				ret = stk3x3x_i2c_read_data(stk_data->client, 0x11, 2, &value[0]);
				if(ret < 0) {
					SENSOR_LOG_ERROR("fail, ret=0x%x", ret);
					return ret;
				}
				word_data = (value[0]<<8) | value[1];

				if (word_data > 0) {
					ct_value = stk_data->ps_thd_h - stk_data->stk_ht_n_ct;
					flag = stk3x3x_get_flag(stk_data);
					if ((word_data < ct_value) && ((ct_value - word_data) > 10)
					    && ((flag & STK_FLG_PSINT_MASK) == 0)
					    && (stk_data->ps_distance_last == 1)
					    && (stk_data->ps_thd_update == false)) {
						stk_data->tracking_count++;
						SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "CTTRACKING pre ps=%d, count=%d\n", word_data, stk_data->tracking_count);
						if(stk_data->tracking_count > 3) {
							stk_data->psi = word_data;
							stk_data->ps_thd_h = stk_data->psi + stk_data->stk_ht_n_ct;
							stk_data->ps_thd_l = stk_data->psi + stk_data->stk_lt_n_ct;
							stk3x3x_set_ps_thd_h(stk_data, stk_data->ps_thd_h);
							stk3x3x_set_ps_thd_l(stk_data, stk_data->ps_thd_l);
							SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "CTTRACKING set HT=%d, LT=%d, psi=%d\n", stk_data->ps_thd_h, stk_data->ps_thd_l, stk_data->psi);
							stk_data->tracking_count = 0;
							if ((stk_data->ps_thd_h + STK_H_LT - STK_HT_N_CT) < stk_data->ps_high_thd_boot) {
								stk_data->ps_high_thd_boot =
								    word_data + stk_data->stk_h_ht;
								stk_data->ps_low_thd_boot =
								    word_data + stk_data->stk_h_lt;
								stk_data->boot_cali = 1;
								SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "update boot2 HT=%d, LT=%d\n",
								                stk_data->ps_high_thd_boot, stk_data->ps_low_thd_boot);
							}
						}
					} else {
						stk_data->tracking_count = 0;
					}
				}
			}
		} else if(stk_data->ps_thd_update == false) {
			ret = stk_ps_val(stk_data);
			if (ret == 0) {
				ret = stk3x3x_i2c_read_data(stk_data->client, 0x11, 2, &value[0]);
				if(ret < 0) {
					SENSOR_LOG_ERROR("fail, ret=0x%x", ret);
					return ret;
				}
				word_data = (value[0]<<8) | value[1];

				if((stk_data->data_count%15)==9) {
					SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "judge ps=%d\n", word_data);
				}

				if(word_data > (stk_data->psi + STK_H_PS)) {
					stk_data->ps_thd_h = stk_data->psi + stk_data->stk_h_ht;
					stk_data->ps_thd_l = stk_data->psi + stk_data->stk_h_lt;
					stk3x3x_set_ps_thd_h(stk_data, stk_data->ps_thd_h);
					stk3x3x_set_ps_thd_l(stk_data, stk_data->ps_thd_l);
					stk_data->ps_thd_update = true;
					SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "smudge set HT=%d, LT=%d, psi=%d, ps=%d\n", stk_data->ps_thd_h, stk_data->ps_thd_l, stk_data->psi, word_data);
				}
			}
		}

		return 0;
	}
#endif

	ret = stk3x3x_get_flag(stk_data);
	if(ret < 0)
		return ret;
	if(!(ret&STK_FLG_PSDR_MASK)) {
		return 0;
	}

	if((ret & STK_FLG_NF_MASK)==0) {
		SENSOR_LOG_INFO(" nearby state\n");
		return 0;
	}

	ret = stk_ps_val(stk_data);
	if(ret == 0) {
		ret = stk3x3x_i2c_read_data(stk_data->client, 0x11, 2, &value[0]);
		if(ret < 0) {
			SENSOR_LOG_ERROR("fail, ret=0x%x", ret);
			return ret;
		}
		word_data = (value[0]<<8) | value[1];
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " word_data=%d\n", word_data);

		if(word_data == 0) {
			//SENSOR_LOG_ERROR("incorrect word data (0)\n");
			return 0xFFFF;
		}

		if(word_data > stk_data->psa) {
			stk_data->psa = word_data;
			SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " update psa: psa=%d,psi=%d\n", stk_data->psa, stk_data->psi);
		}
		if(word_data < stk_data->psi) {
			stk_data->psi = word_data;
			SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " update psi: psa=%d,psi=%d\n", stk_data->psa, stk_data->psi);
		}
	}
	diff = stk_data->psa - stk_data->psi;
	if(diff > stk_data->stk_max_min_diff) {
		stk_data->psi_set = stk_data->psi;

#ifdef CALI_PS_EVERY_TIME
		if(((stk_data->psi + stk_data->stk_ht_n_ct) > (stk_data->ps_thd_h + 2000)) && (stk_data->ps_thd_h != 0)) {
			//  stk_data->ps_thd_h = stk_data->ps_thd_h;
			//  stk_data->ps_thd_l = stk_data->ps_thd_l;
			SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " no update thd, HT=%d, LT=%d\n", stk_data->ps_thd_h, stk_data->ps_thd_l);
		} else {
			stk_data->ps_thd_h = stk_data->psi + stk_data->stk_ht_n_ct;
			stk_data->ps_thd_l = stk_data->psi + stk_data->stk_lt_n_ct;
			SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " update thd, HT=%d, LT=%d\n", stk_data->ps_thd_h, stk_data->ps_thd_l);
		}
#else
		stk_data->ps_thd_h = stk_data->psi + stk_data->stk_ht_n_ct;
		stk_data->ps_thd_l = stk_data->psi + stk_data->stk_lt_n_ct;
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " update thd, HT=%d, LT=%d\n", stk_data->ps_thd_h, stk_data->ps_thd_l);
#endif

		/*
		#ifdef CALI_PS_EVERY_TIME
		        if(stk_data->ps_thd_h > (stk_data->ps_high_thd_boot))
		        {
		            stk_data->ps_high_thd_boot = stk_data->ps_thd_h;
		            stk_data->ps_low_thd_boot = stk_data->ps_thd_l;
		            SENSOR_LOG_INFO(" update boot HT=%d, LT=%d\n", stk_data->ps_high_thd_boot, stk_data->ps_low_thd_boot);
		        }
		#endif      */
		stk3x3x_set_ps_thd_h(stk_data, stk_data->ps_thd_h);
		stk3x3x_set_ps_thd_l(stk_data, stk_data->ps_thd_l);
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "FAE tune0 psa-psi(%d) > STK_DIFF found\n", diff);

		if ((stk_data->ps_thd_h + STK_H_HT - STK_HT_N_CT) < stk_data->ps_high_thd_boot) {
			stk_data->ps_high_thd_boot =
			    stk_data->psi + stk_data->stk_h_ht;
			stk_data->ps_low_thd_boot =
			    stk_data->psi + stk_data->stk_h_lt;
			stk_data->boot_cali = 1;
			SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "update boot1 HT=%d, LT=%d\n",
			                stk_data->ps_high_thd_boot, stk_data->ps_low_thd_boot);
		}

#ifndef CTTRACKING
		hrtimer_cancel(&stk_data->ps_tune0_timer);
#endif

	}

	return 0;
}

static void stk_ps_tune0_work_func(struct work_struct *work)
{
	struct stk3x3x_data *stk_data = container_of(work, struct stk3x3x_data, stk_ps_tune0_work);
	if(stk_data->prox_debug)
		return;

    if(stk_data->entry_suspend == true)
        return;

	if(stk_data->tune_zero_init_proc)
		stk_tune_zero_get_ps_data(stk_data);
	else
		stk_ps_tune_zero_func_fae(stk_data);
	return;
}

static enum hrtimer_restart stk_ps_tune0_timer_func(struct hrtimer *timer)
{
	struct stk3x3x_data *stk_data = container_of(timer, struct stk3x3x_data, ps_tune0_timer);
	queue_work(stk_data->stk_ps_tune0_wq, &stk_data->stk_ps_tune0_work);
	hrtimer_forward_now(&stk_data->ps_tune0_timer, stk_data->ps_tune0_delay);
	return HRTIMER_RESTART;
}
#endif

#ifdef STK_POLL_ALS
static enum hrtimer_restart stk_als_timer_func(struct hrtimer *timer)
{
	struct stk3x3x_data *stk_data = container_of(timer, struct stk3x3x_data, als_timer);
	queue_work(stk_data->stk_als_wq, &stk_data->stk_als_work);
	hrtimer_forward_now(&stk_data->als_timer, stk_data->als_poll_delay);
	return HRTIMER_RESTART;
}

static void stk_als_poll_work_func(struct work_struct *work)
{
	struct stk3x3x_data *stk_data = container_of(work, struct stk3x3x_data, stk_als_work);
	int32_t reading = 0, reading_lux, flag_reg;
	int ret;
    unsigned char value[2];
    uint16_t als_data;

	if((stk_data->als_fast_report == false) && (stk_data->als_fast_report_cnt == stk_data->als_debug_cnt)){
		
		ret = stk3x3x_i2c_read_data(stk_data->client, STK_DATA1_ALS_REG, 2, &value[0]);
		if(ret < 0) {
			SENSOR_LOG_ERROR("fail, ret=0x%x\n", ret);
			return ;
		}		
		als_data = (value[0]<<8) | value[1];

		SENSOR_LOG_ERROR("skip slow first data cnt=%d, als=%d\n", stk_data->als_debug_cnt, als_data);
		stk_data->als_fast_report_cnt = 60000;
		return;
	}else if((stk_data->als_fast_report == false) && (stk_data->als_fast_report_cnt == 60000)){
		ret = stk3x3x_i2c_read_data(stk_data->client, STK_DATA1_ALS_REG, 2, &value[0]);
		if(ret < 0) {
			SENSOR_LOG_ERROR("fail, ret=0x%x\n", ret);
			return ;
		}		
		als_data = (value[0]<<8) | value[1];

		SENSOR_LOG_ERROR("skip slow two data als=%d\n", als_data);
		stk_data->als_fast_report_cnt = 60001;
		return;
	}	

	if((stk_data->als_fast_report == true) && (stk_data->als_fast_report_cnt == 20) && (stk_data->als_debug_cnt == 0)){
		ret = stk3x3x_i2c_read_data(stk_data->client, STK_DATA1_ALS_REG, 2, &value[0]);
		if(ret < 0) {
			SENSOR_LOG_ERROR("fail, ret=0x%x\n", ret);
			return ;
		}		
		als_data = (value[0]<<8) | value[1];		
		SENSOR_LOG_ERROR("skip fast first data cnt=%d, als=%d\n", stk_data->als_fast_report_cnt, als_data);
		stk_data->als_debug_cnt++;
		return;
	}else if((stk_data->als_fast_report == true) && (stk_data->als_fast_report_cnt == 20) && (stk_data->als_debug_cnt == 1)){
		ret = stk3x3x_i2c_read_data(stk_data->client, STK_DATA1_ALS_REG, 2, &value[0]);
		if(ret < 0) {
			SENSOR_LOG_ERROR("fail, ret=0x%x\n", ret);
			return ;
		}		
		als_data = (value[0]<<8) | value[1];		
		SENSOR_LOG_ERROR("skip fast two data cnt=%d, als=%d\n", stk_data->als_fast_report_cnt, als_data);
		stk_data->als_debug_cnt++;
        
        #ifdef STK_ALS_FIR
        SENSOR_LOG_ERROR("skip fast two data reset fir\n");
        memset(&stk_data->fir, 0x00, sizeof(stk_data->fir));
        stk_data->fir.number = 0;
        #endif
        
        return;		
	}	

    stk_data->als_debug_cnt++;
    if(stk_data->als_debug_cnt > 50000){
        stk_data->als_debug_cnt = 0;
    }

	flag_reg = stk3x3x_get_flag(stk_data);
	if(flag_reg < 0)
		return;
	if(!(flag_reg&STK_FLG_ALSDR_MASK)) {
		return;
	}

#ifdef STK_IRS
	ret = stk_als_ir_skip_als(stk_data);
	if(ret == 1)
		return;
#endif

	reading = stk3x3x_get_als_reading(stk_data);
	if(reading < 0)
		return;

#ifdef STK_IRS
	stk_als_ir_get_corr(stk_data, reading);
	reading = reading * stk_data->als_correct_factor / 1000;
#endif
	reading_lux = stk_alscode2lux(stk_data, reading);
	stk_als_report(stk_data, reading_lux);

#ifdef STK_IRS
	stk_als_ir_run(stk_data);
#endif
    if(stk_data->als_fast_report == true && !stk_data->als_fast_report_cnt--){
        stk3x3x_als_fast_report_enable(stk_data, 0);
        SENSOR_LOG_ERROR("fast report disable cnt=%d\n", stk_data->als_debug_cnt);
        stk_data->als_fast_report_cnt = stk_data->als_debug_cnt;
    }
}
#endif /* #ifdef STK_POLL_ALS */

#ifdef STK_POLL_PS
static enum hrtimer_restart stk_ps_timer_func(struct hrtimer *timer)
{
	struct stk3x3x_data *stk_data = container_of(timer, struct stk3x3x_data, ps_timer);
	queue_work(stk_data->stk_ps_wq, &stk_data->stk_ps_work);
	hrtimer_forward_now(&stk_data->ps_timer, stk_data->ps_poll_delay);
	return HRTIMER_RESTART;
}

static void stk_ps_poll_work_func(struct work_struct *work)
{
	struct stk3x3x_data *stk_data = container_of(work, struct stk3x3x_data, stk_ps_work);
	uint32_t reading;
	int32_t near_far_state;
	uint8_t org_flag_reg;

	if(stk_data->ps_enabled) {
#ifdef STK_TUNE0
		// if(!(stk_data->psi_set))
		// return;
#endif
		org_flag_reg = stk3x3x_get_flag(stk_data);
		if(org_flag_reg < 0)
			return;

		if(!(org_flag_reg&STK_FLG_PSDR_MASK)) {
			//SENSOR_LOG_INFO(" ps is not ready\n");
			return;
		}

		near_far_state = (org_flag_reg & STK_FLG_NF_MASK)?1:0;
		reading = stk3x3x_get_ps_reading(stk_data);
		stk_data->ps_code_last = reading;

		if(stk_data->ps_distance_last != near_far_state) {
			stk_ps_report(stk_data, near_far_state);
			SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " ps input event=%d, ps=%d\n", near_far_state, reading);
		}
	}
}
#endif

#if (!defined(STK_POLL_PS) || !defined(STK_POLL_ALS))
#if ((STK_INT_PS_MODE == 0x03) || (STK_INT_PS_MODE  == 0x02))
static void stk_ps_int_handle_int_mode_2_3(struct stk3x3x_data *stk_data)
{
	uint32_t reading;
	int32_t near_far_state;

#if (STK_INT_PS_MODE    == 0x03)
	near_far_state = gpio_get_value(stk_data->int_pin);
#elif   (STK_INT_PS_MODE == 0x02)
	near_far_state = !(gpio_get_value(stk_data->int_pin));
#endif
	reading = stk3x3x_get_ps_reading(stk_data);
	stk_ps_report(stk_data, near_far_state);
	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " ps input event=%d, ps code=%d\n", near_far_state, reading);
}
#endif

static void stk_ps_int_handle(struct stk3x3x_data *stk_data, uint32_t ps_reading, int32_t nf_state)
{
	stk_ps_report(stk_data, nf_state);
	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " ps input event=%d, ps code=%d\n", nf_state, ps_reading);
}

static int stk_als_int_handle(struct stk3x3x_data *stk_data, uint32_t als_reading)
{
	int32_t als_comperator;
#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
	uint32_t nLuxIndex;
#endif
	int lux;

#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
	nLuxIndex = stk_get_lux_interval_index(als_reading);
	stk3x3x_set_als_thd_h(stk_data, code_threshold_table[nLuxIndex]);
	stk3x3x_set_als_thd_l(stk_data, code_threshold_table[nLuxIndex-1]);
#else
	stk_als_set_new_thd(stk_data, als_reading);
#endif //CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD

	if(stk_data->ir_code) {
		if(als_reading < STK_IRC_MAX_ALS_CODE && als_reading > STK_IRC_MIN_ALS_CODE &&
		   stk_data->ir_code > STK_IRC_MIN_IR_CODE) {
			als_comperator = als_reading * STK_IRC_ALS_NUMERA / STK_IRC_ALS_DENOMI;
			if(stk_data->ir_code > als_comperator)
				stk_data->als_correct_factor = STK_IRC_ALS_CORREC;
			else
				stk_data->als_correct_factor = 1000;
		}
		stk_data->ir_code = 0;
	}

	als_reading = als_reading * stk_data->als_correct_factor / 1000;
	stk_data->als_code_last = als_reading;

	lux = stk_alscode2lux(stk_data, als_reading);
	stk_als_report(stk_data, lux);
	return 0;
}

static void stk_work_func(struct work_struct *work)
{
	int32_t ret;
	int32_t near_far_state;

#if ((STK_INT_PS_MODE != 0x03) && (STK_INT_PS_MODE != 0x02))
	uint8_t disable_flag = 0;
	int32_t org_flag_reg;
#endif  /* #if ((STK_INT_PS_MODE != 0x03) && (STK_INT_PS_MODE != 0x02)) */
	struct stk3x3x_data *stk_data = container_of(work, struct stk3x3x_data, stk_work);
	uint32_t reading;

#if ((STK_INT_PS_MODE == 0x03) || (STK_INT_PS_MODE  == 0x02))
	stk_ps_int_handle_int_mode_2_3(stk_data);
#else
	/* mode 0x01 or 0x04 */
	org_flag_reg = stk3x3x_get_flag(stk_data);
	if(org_flag_reg < 0)
		goto err_i2c_rw;

	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " flag=0x%x\n", org_flag_reg);
	if(org_flag_reg & STK_FLG_ALSINT_MASK) {
		disable_flag |= STK_FLG_ALSINT_MASK;
		reading = stk3x3x_get_als_reading(stk_data);
		if(reading < 0)
			goto err_i2c_rw;
		ret = stk_als_int_handle(stk_data, reading);
		if(ret < 0)
			goto err_i2c_rw;
	}
	if (org_flag_reg & STK_FLG_PSINT_MASK) {
		disable_flag |= STK_FLG_PSINT_MASK;
		if(stk_data->prox_debug == false) {
			reading = stk3x3x_get_ps_reading(stk_data);
			if(reading < 0)
				goto err_i2c_rw;
			stk_data->ps_code_last = reading;

			near_far_state = (org_flag_reg & STK_FLG_NF_MASK)?1:0;

#ifdef CTTRACKING
			if (near_far_state == 0) {
				if(reading > (stk_data->psi + STK_H_PS)) {
					stk_data->ps_thd_h = stk_data->psi + stk_data->stk_h_ht;
					stk_data->ps_thd_l = stk_data->psi + stk_data->stk_h_lt;
					stk3x3x_set_ps_thd_h(stk_data, stk_data->ps_thd_h);
					stk3x3x_set_ps_thd_l(stk_data, stk_data->ps_thd_l);
					stk_data->ps_thd_update = true;
					SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " near update HT=%d, LT=%d\n", stk_data->ps_thd_h, stk_data->ps_thd_l);
				}
			} else {
				SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " far ps_update=%d\n", stk_data->ps_thd_update);
				if (stk_data->ps_thd_update) {
					ret = stk_ps_val(stk_data);
					if (ret == 0) {
						if((reading + stk_data->stk_ht_n_ct) < stk_data->ps_thd_h) {
							stk_data->ps_thd_h = reading + stk_data->stk_ht_n_ct;
							stk_data->ps_thd_l = reading + stk_data->stk_lt_n_ct;
							stk3x3x_set_ps_thd_h(stk_data, stk_data->ps_thd_h);
							stk3x3x_set_ps_thd_l(stk_data, stk_data->ps_thd_l);
							SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " far update HT=%d, LT=%d, ps=%d\n", stk_data->ps_thd_h, stk_data->ps_thd_l, reading);
						}
					}
					stk_data->ps_thd_update = false;
				}
			}
#endif

			stk_ps_int_handle(stk_data, reading, near_far_state);
		}
	}

	if(disable_flag) {
		ret = stk3x3x_set_flag(stk_data, org_flag_reg, disable_flag);
		if(ret < 0)
			goto err_i2c_rw;
	}

#endif
	usleep_range(1000, 2000);
	enable_irq(stk_data->irq);
	return;

err_i2c_rw:
	msleep(30);
	enable_irq(stk_data->irq);
	return;
}

static irqreturn_t stk_oss_irq_handler(int irq, void *data)
{
	struct stk3x3x_data *pData = data;
	disable_irq_nosync(irq);
	queue_work(pData->stk_wq,&pData->stk_work);
	return IRQ_HANDLED;
}
#endif  /*  #if (!defined(STK_POLL_PS) || !defined(STK_POLL_ALS))   */

#ifdef STK_POLL_ALS
static void stk3x3x_als_set_poll_delay(struct stk3x3x_data *stk_data)
{
	uint8_t als_it = stk_data->alsctrl_reg & STK_ALS_IT_MASK;

	if(als_it == STK_IT_ALS_25MS) {
		stk_data->als_poll_delay = ns_to_ktime(35 * NSEC_PER_MSEC);
	} else if(als_it == STK_IT_ALS_50MS) {
		stk_data->als_poll_delay = ns_to_ktime(60 * NSEC_PER_MSEC);
	} else if(als_it == STK_IT_ALS_100MS) {
		stk_data->als_poll_delay = ns_to_ktime(110 * NSEC_PER_MSEC);
	} else if(als_it == STK_IT_ALS_200MS) {
		stk_data->als_poll_delay = ns_to_ktime(220 * NSEC_PER_MSEC);
	} else if(als_it == STK_IT_ALS_400MS) {
		stk_data->als_poll_delay = ns_to_ktime(440 * NSEC_PER_MSEC);
	} else if(als_it == STK_IT_ALS_800MS) {
		stk_data->als_poll_delay = ns_to_ktime(880 * NSEC_PER_MSEC);
	} else {
		stk_data->als_poll_delay = ns_to_ktime(110 * NSEC_PER_MSEC);
		SENSOR_LOG_DEBUG_IF(stk_data->code_debug, "unknown ALS_IT=%d, set als_poll_delay=110ms\n", als_it);
	}
	SENSOR_LOG_INFO("ALS_IT=%d, set als_poll_delay %lld ns\n", als_it, stk_data->als_poll_delay);
}

static int32_t stk3x3x_set_als_ctrl(struct stk3x3x_data *stk_data, uint8_t als_it, uint8_t als_gain)
{
	/*
	 *althouth set als_ctrl reg need poweroff als, we not poweroff to set alsctrl_reg for speed.
	 *if not poweroff to set als_ctrl, it will read small als value.but data_filter will average als value.
	 *so set it
	 */
	int ret;
	stk_data->alsctrl_reg &=~ (STK_ALS_IT_MASK|STK_ALS_GAIN_MASK);
	stk_data->alsctrl_reg |= (als_it|als_gain);

	SENSOR_LOG_INFO("set alsctrl_reg 0x%x\n", stk_data->alsctrl_reg);
	stk3x3x_als_set_poll_delay(stk_data);

	ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, STK_ALSCTRL_REG, stk_data->alsctrl_reg);
	if (ret < 0) {
		SENSOR_LOG_ERROR("write i2c error\n");
		return ret;
	}
	return ret;

}
static void stk3x3x_als_fast_report_enable(struct stk3x3x_data *stk_data, int enable)
{
	if(enable && stk_data->als_fast_report != true) {
		stk_data->als_fast_report_cnt = 20;
		stk_data->als_fast_report = true;
		stk3x3x_als_fast_report_init(stk_data);
	} else if(!enable && stk_data->als_fast_report != false) {
		stk_data->als_fast_report_cnt = 0;
		stk_data->als_fast_report = false;
		stk3x3x_als_fast_report_init(stk_data);
	}
}

static int stk3x3x_als_fast_report_init(struct stk3x3x_data *stk_data)
{
	static uint8_t old_als_ctrl_reg = 0;
	uint8_t als_it, als_gain;
	int ret;
	unsigned char value[6];

	if(stk_data->als_fast_report == true) {
		ret = stk3x3x_i2c_smbus_read_byte_data(stk_data->client,STK_ALSCTRL_REG);//backup old als reg
		if(old_als_ctrl_reg < 0) {
			SENSOR_LOG_ERROR("Fail, ret=%d\n", ret);
			return ret;
		}
		old_als_ctrl_reg = (uint8_t)ret;
		SENSOR_LOG_INFO("Backup old alsctrl_reg 0x%x\n", old_als_ctrl_reg);
		stk3x3x_set_als_ctrl(stk_data, STK_IT_ALS_25MS, STK_ALS_GAIN_16);
		msleep(50);
		stk3x3x_i2c_read_data(stk_data->client, STK_DATA1_ALS_REG, 6, &value[0]);
	}
	if(stk_data->als_fast_report == false) { //recovery old als reg
		als_it = old_als_ctrl_reg & STK_ALS_IT_MASK;
		als_gain = old_als_ctrl_reg & STK_ALS_GAIN_MASK;
		SENSOR_LOG_INFO("set old alsctrl_reg 0x%x\n", old_als_ctrl_reg);
		stk3x3x_set_als_ctrl(stk_data, als_it, als_gain);
		msleep(100);
		stk3x3x_i2c_read_data(stk_data->client, STK_DATA1_ALS_REG, 6, &value[0]);
	}
	return ret;
}
#endif

static int32_t stk3x3x_init_all_setting(struct i2c_client *client, struct stk3x3x_platform_data *plat_data)
{
	int32_t ret;
	struct stk3x3x_data *stk_data = i2c_get_clientdata(client);

	ret = stk3x3x_software_reset(stk_data);
	if(ret < 0)
		return ret;

	ret = stk3x3x_check_pid(stk_data);
	if(ret < 0)
		return ret;
	stk3x3x_proc_plat_data(stk_data, plat_data);
	ret = stk3x3x_init_all_reg(stk_data);
	if(ret < 0)
		return ret;
#ifdef STK_POLL_ALS
	stk3x3x_als_set_poll_delay(stk_data);
#endif
	stk_data->als_enabled = false;
	stk_data->ps_enabled = false;
	stk_data->re_enable_als = false;
	stk_data->re_enable_ps = false;
	stk_data->ir_code = 0;
	stk_data->als_correct_factor = 1000;
	stk_data->first_boot = true;
#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
	stk_init_code_threshold_table(stk_data);
#endif
#ifdef STK_TUNE0
#ifdef QUALCOMM_PLATFORM
	stk_data->tune_zero_init_proc = false;
	stk_data->psi_set = 0;
#else
	stk_ps_tune_zero_init(stk_data);
#endif
#endif
#ifdef STK_ALS_FIR
	memset(&stk_data->fir, 0x00, sizeof(stk_data->fir));
	atomic_set(&stk_data->firlength, STK_FIR_LEN);
#endif
	atomic_set(&stk_data->recv_reg, 0);
#ifdef STK_IRS
	stk_data->als_data_index = 0;
#endif
	stk_data->ps_distance_last = -1;
	stk_data->als_code_last = 100;
	return 0;
}

#if (!defined(STK_POLL_PS) || !defined(STK_POLL_ALS))
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
	struct stk3x3x_data *stk_data = i2c_get_clientdata(client);

	stk_pinctrl_active(&client->dev);

#ifdef SPREADTRUM_PLATFORM
	irq = sprd_alloc_gpio_irq(stk_data->int_pin);
#else
	irq = gpio_to_irq(stk_data->int_pin);
#endif
	SENSOR_LOG_DEBUG_IF(stk_data->code_debug, " int pin #=%d, irq=%d\n",stk_data->int_pin, irq);
	if (irq <= 0) {
		SENSOR_LOG_ERROR("irq number is not specified, irq # = %d, int pin=%d\n",irq, stk_data->int_pin);
		return irq;
	}
	stk_data->irq = irq;
	err = gpio_request(stk_data->int_pin,"stk-int");
	if(err < 0) {
		SENSOR_LOG_ERROR("gpio_request, err=%d", err);
		return err;
	}
	// gpio_tlmm_config(GPIO_CFG(stk_data->int_pin, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE);

	err = gpio_direction_input(stk_data->int_pin);
	if(err < 0) {
		SENSOR_LOG_ERROR("gpio_direction_input, err=%d", err);
		return err;
	}
#if ((STK_INT_PS_MODE == 0x03) || (STK_INT_PS_MODE  == 0x02))
	err = request_any_context_irq(irq, stk_oss_irq_handler, IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, DEVICE_NAME, stk_data);
#else
	err = request_any_context_irq(irq, stk_oss_irq_handler, IRQF_TRIGGER_LOW, DEVICE_NAME, stk_data);
#endif
	if (err < 0) {
		SENSOR_LOG_DEBUG("request_any_context_irq(%d) failed for (%d)\n", irq, err);
		goto err_request_any_context_irq;
	}
	disable_irq(irq);

	return 0;
err_request_any_context_irq:
#ifdef SPREADTRUM_PLATFORM
	sprd_free_gpio_irq(stk_data->int_pin);
#else
	gpio_free(stk_data->int_pin);
#endif
	return err;
}
#endif


static int stk3x3x_suspend(struct device *dev)
{
	struct stk3x3x_data *stk_data = dev_get_drvdata(dev);
#if (defined(STK_CHK_REG) || !defined(STK_POLL_PS))
	int err;
#endif

#ifndef STK_POLL_PS
	struct i2c_client *client = to_i2c_client(dev);
#endif

	SENSOR_LOG_INFO("enter\n");
    stk_data->entry_suspend = true;
#ifndef SPREADTRUM_PLATFORM
	mutex_lock(&stk_data->io_lock);
#endif
#ifdef STK_CHK_REG
	err = stk3x3x_validate_n_handle(stk_data->client);
	if(err < 0) {
		SENSOR_LOG_ERROR("stk3x3x_validate_n_handle fail: %d\n", err);
	} else if (err == 0xFF) {
		if(stk_data->ps_enabled)
			stk3x3x_enable_ps(stk_data, 1, 0);
	}
#endif /* #ifdef STK_CHK_REG */

#ifndef SPREADTRUM_PLATFORM
	if(stk_data->als_enabled) {
		SENSOR_LOG_ERROR("Enable ALS : 0\n");
		stk3x3x_enable_als(stk_data, 0);
		stk_data->re_enable_als = true;
	}
#endif

	if(stk_data->ps_enabled) {
#ifdef STK_POLL_PS
		wake_lock(&stk_data->ps_nosuspend_wl);
#else
		if(device_may_wakeup(&client->dev)) {
			err = enable_irq_wake(stk_data->irq);
			if (err)
				SENSOR_LOG_DEBUG("set_irq_wake(%d) failed, err=(%d)\n", stk_data->irq, err);
		} else {
			SENSOR_LOG_ERROR("not support wakeup source\n");
		}
#endif
	}
#ifndef SPREADTRUM_PLATFORM
	mutex_unlock(&stk_data->io_lock);
#endif
	return 0;
}

static int stk3x3x_resume(struct device *dev)
{
	struct stk3x3x_data *stk_data = dev_get_drvdata(dev);
#if (defined(STK_CHK_REG) || !defined(STK_POLL_PS))
	int err;
#endif
#ifndef STK_POLL_PS
	struct i2c_client *client = to_i2c_client(dev);
#endif

	SENSOR_LOG_INFO("enter\n");
    stk_data->entry_suspend = false;
#ifndef SPREADTRUM_PLATFORM
	mutex_lock(&stk_data->io_lock);
#endif
#ifdef STK_CHK_REG
	err = stk3x3x_validate_n_handle(stk_data->client);
	if(err < 0) {
		SENSOR_LOG_ERROR("stk3x3x_validate_n_handle fail: %d\n", err);
	} else if (err == 0xFF) {
		if(stk_data->ps_enabled)
			stk3x3x_enable_ps(stk_data, 1, 0);
	}
#endif /* #ifdef STK_CHK_REG */

#ifndef SPREADTRUM_PLATFORM
	if(stk_data->re_enable_als) {
		SENSOR_LOG_ERROR("Enable ALS : 1\n");
		stk3x3x_enable_als(stk_data, 1);
		stk_data->re_enable_als = false;
	}
#endif

	if(stk_data->ps_enabled) {
#ifdef STK_POLL_PS
		wake_unlock(&stk_data->ps_nosuspend_wl);
#else
		if(device_may_wakeup(&client->dev)) {
			err = disable_irq_wake(stk_data->irq);
			if (err)
				SENSOR_LOG_DEBUG("disable_irq_wake(%d) failed, err=(%d)\n", stk_data->irq, err);
		}
#endif
	}
#ifndef SPREADTRUM_PLATFORM
	mutex_unlock(&stk_data->io_lock);
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

static int stk3x3x_device_ctl(struct stk3x3x_data *stk_data, bool enable)
{
	int ret;
	struct device *dev = &stk_data->client->dev;

	if (enable && !stk_data->power_enabled) {
		ret = stk3x3x_power_ctl(stk_data, true);
		if (ret) {
			dev_err(dev, "Failed to enable device power\n");
			goto err_exit;
		}
		ret = stk3x3x_init_all_setting(stk_data->client, stk_data->pdata);
		if (ret < 0) {
			stk3x3x_power_ctl(stk_data, false);
			dev_err(dev, "Failed to re-init device setting\n");
			goto err_exit;
		}
	} else if (!enable && stk_data->power_enabled) {

		if (!stk_data->ps_enabled) {
			ret = stk3x3x_power_ctl(stk_data, false);
			if (ret) {
				dev_err(dev, "Failed to disable device power\n");
				goto err_exit;
			}
		} else {
			dev_dbg(dev, "device control: als_enabled=%d, ps_enabled=%d\n",
			        0, stk_data->ps_enabled);
		}
	} else {
		dev_dbg(dev, "device control: enable=%d, power_enabled=%d\n",
		        enable, stk_data->power_enabled);
	}
	return 0;

err_exit:
	return ret;
}
#endif  /* #ifdef STK_QUALCOMM_POWER_CTRL */

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
	rc = of_property_read_u32(np, "stk,transmittance", &temp_val);
	if (!rc)
		pdata->transmittance = temp_val;
	else {
		dev_err(dev, "Unable to read transmittance\n");
		return rc;
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

	rc = of_property_read_u32(np, "stk,alsctrl-reg", &temp_val);
	if (!rc)
		pdata->alsctrl_reg = (u8)temp_val;
	else {
		dev_err(dev, "Unable to read alsctrl-reg\n");
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

static int stk3x3x_set_wq(struct stk3x3x_data *stk_data)
{
#ifdef STK_POLL_ALS
	stk_data->stk_als_wq = create_singlethread_workqueue("stk_als_wq");
	INIT_WORK(&stk_data->stk_als_work, stk_als_poll_work_func);
	hrtimer_init(&stk_data->als_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stk_data->als_poll_delay = ns_to_ktime(110 * NSEC_PER_MSEC);
	stk_data->als_timer.function = stk_als_timer_func;
#endif

#ifdef STK_POLL_PS
	stk_data->stk_ps_wq = create_singlethread_workqueue("stk_ps_wq");
	INIT_WORK(&stk_data->stk_ps_work, stk_ps_poll_work_func);
	hrtimer_init(&stk_data->ps_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stk_data->ps_poll_delay = ns_to_ktime(60 * NSEC_PER_MSEC);
	stk_data->ps_timer.function = stk_ps_timer_func;
#endif

#ifdef STK_TUNE0
	stk_data->stk_ps_tune0_wq = create_singlethread_workqueue("stk_ps_tune0_wq");
	INIT_WORK(&stk_data->stk_ps_tune0_work, stk_ps_tune0_work_func);
	hrtimer_init(&stk_data->ps_tune0_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stk_data->ps_tune0_delay = ns_to_ktime(60 * NSEC_PER_MSEC);
	stk_data->ps_tune0_timer.function = stk_ps_tune0_timer_func;
#endif

#ifdef STK_ALSPS_CALIBRATION
	stk_data->stk_ps_cali_wq = create_singlethread_workqueue("stk_ps_cali_wq");
	INIT_WORK(&stk_data->stk_ps_cali_work, stk_ps_cali_work_func);
	hrtimer_init(&stk_data->ps_cali_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stk_data->ps_cali_delay = ns_to_ktime(180 * NSEC_PER_MSEC);
	stk_data->ps_cali_timer.function = stk_ps_cali_timer_func;
	stk_data->ps_cali_timer_run = false;
#endif

#if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS))
	stk_data->stk_wq = create_singlethread_workqueue("stk_wq");
	INIT_WORK(&stk_data->stk_work, stk_work_func);
#endif
	return 0;
}


static int stk3x3x_set_input_devices(struct stk3x3x_data *stk_data)
{
	int err;
    
	stk_data->als_input_dev = input_allocate_device();
	if (stk_data->als_input_dev==NULL) {
		SENSOR_LOG_ERROR("could not allocate als device\n");
		err = -ENOMEM;
		goto err_als_input_allocate;
	}
	stk_data->code_debug = 0;
    stk_data->als_input_dev->name = ALS_NAME;
	//set_bit(EV_ABS, stk_data->als_input_dev->evbit);
	//input_set_abs_params(stk_data->als_input_dev, ABS_MISC, 0, stk_alscode2lux(stk_data, (1<<16)-1), 0, 0);
	set_bit(EV_REL, stk_data->als_input_dev->evbit);
	set_bit(REL_X,  stk_data->als_input_dev->relbit);
	err = input_register_device(stk_data->als_input_dev);
	if (err<0) {
		SENSOR_LOG_ERROR("can not register als input device\n");
		goto err_als_input_register;
	}
	input_set_drvdata(stk_data->als_input_dev, stk_data);

	stk_data->ps_input_dev = input_allocate_device();
	if (stk_data->ps_input_dev==NULL) {
		SENSOR_LOG_ERROR("could not allocate ps device\n");
		err = -ENOMEM;
		goto err_ps_input_allocate;
	}
	stk_data->ps_input_dev->name = PS_NAME;
	//set_bit(EV_ABS, stk_data->ps_input_dev->evbit);
	set_bit(EV_REL, stk_data->ps_input_dev->evbit);
	set_bit(REL_RZ,  stk_data->ps_input_dev->relbit);
	set_bit(REL_MISC, stk_data->ps_input_dev->relbit);
	err = input_register_device(stk_data->ps_input_dev);
	if (err<0) {
		SENSOR_LOG_ERROR("can not register ps input device\n");
		goto err_ps_input_register;
	}
	input_set_drvdata(stk_data->ps_input_dev, stk_data);

	als_class = class_create(THIS_MODULE, ALS_NAME);
	alloc_chrdev_region(&stk3337_als_dev_t, 0, 1, ALS_NAME);
	stk_data->als_dev = device_create(als_class, 0, stk3337_als_dev_t, &stk_ps_driver, ALS_NAME);
	if (IS_ERR_OR_NULL(stk_data->als_dev)) {
		SENSOR_LOG_ERROR("als device create fail\n");
		err = -PTR_ERR(stk_data->als_dev);
		goto err_remove_als_device;
	}
	err = sensor_create_sysfs_interfaces(stk_data->als_dev, stk_als_attrs, ARRAY_SIZE(stk_als_attrs));
	if (err < 0) {
		SENSOR_LOG_ERROR("Als create sysfs interfaces fail\n");
		goto err_remove_als_interfaces;
	}
	dev_set_drvdata(stk_data->als_dev, stk_data);

	ps_class = class_create(THIS_MODULE, PS_NAME);
	alloc_chrdev_region(&stk3337_ps_dev_t, 0, 1, PS_NAME);
	stk_data->ps_dev = device_create(ps_class, 0, stk3337_ps_dev_t, &stk_ps_driver, PS_NAME);
	if (IS_ERR_OR_NULL(stk_data->ps_dev)) {
		SENSOR_LOG_ERROR("als device create fail\n");
		err = -PTR_ERR(stk_data->ps_dev);
		goto err_remove_ps_device;
	}
	err = sensor_create_sysfs_interfaces(stk_data->ps_dev, stk_prox_attrs, ARRAY_SIZE(stk_prox_attrs));
	if (err < 0) {
		SENSOR_LOG_ERROR("ps create sysfs interfaces fail\n");
		goto err_remove_ps_interfaces;
	}
	dev_set_drvdata(stk_data->ps_dev, stk_data);

	return 0;

err_als_input_allocate:
	input_unregister_device(stk_data->als_input_dev);
	return err;

err_ps_input_allocate:
	input_unregister_device(stk_data->ps_input_dev);
	return err;

err_als_input_register:
	input_unregister_device(stk_data->als_input_dev);
	input_free_device(stk_data->als_input_dev);
	return err;

err_ps_input_register:
	input_unregister_device(stk_data->ps_input_dev);
	input_free_device(stk_data->ps_input_dev);
	return err;

err_remove_als_interfaces:
	sensor_remove_sysfs_interfaces(stk_data->als_dev,
	                               stk_als_attrs, ARRAY_SIZE(stk_als_attrs));
err_remove_als_device:
	device_destroy(als_class, stk3337_als_dev_t);
	class_destroy(als_class);
	return err;

err_remove_ps_interfaces:
	sensor_remove_sysfs_interfaces(stk_data->ps_dev,
	                               stk_prox_attrs, ARRAY_SIZE(stk_prox_attrs));
err_remove_ps_device:
	device_destroy(ps_class, stk3337_ps_dev_t);
	class_destroy(ps_class);
	return err;
}

static int stk3x3x_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
	int err = -ENODEV;
	struct stk3x3x_data *stk_data;
	struct stk3x3x_platform_data *plat_data;
	SENSOR_LOG_INFO(" driver version = %s\n", DRIVER_VERSION);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		SENSOR_LOG_ERROR("No Support for I2C_FUNC_I2C\n");
		return -ENODEV;
	}

	stk_data = kzalloc(sizeof(struct stk3x3x_data),GFP_KERNEL);
	if(!stk_data) {
		SENSOR_LOG_ERROR("failed to allocate stk3x3x_data\n");
		return -ENOMEM;
	}
	stk_data->client = client;
	i2c_set_clientdata(client,stk_data);
	mutex_init(&stk_data->io_lock);
	wake_lock_init(&stk_data->ps_wakelock,WAKE_LOCK_SUSPEND, "stk_input_wakelock");

#ifdef STK_POLL_PS
	wake_lock_init(&stk_data->ps_nosuspend_wl,WAKE_LOCK_SUSPEND, "stk_nosuspend_wakelock");
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
		if (err) {
			dev_err(&client->dev,
			        "stk3x3x_parse_dt ret=%d\n", err);
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
		        "no stk3x3x platform data!\n");
		goto err_stk_input_allocate;
	}
	stk_data->als_transmittance = plat_data->transmittance;
	stk_data->int_pin = plat_data->int_pin;
	stk_data->pdata = plat_data;

	if (stk_data->als_transmittance == 0) {
		dev_err(&client->dev, "%s: Please set als_transmittance\n", __func__);
		goto err_als_input_allocate;
	}

	stk3x3x_set_wq(stk_data);
#ifdef QUALCOMM_PLATFORM
	stk_data->ps_thd_h = 0;
	stk_data->ps_thd_l = 0;
#endif

#ifdef STK_TUNE0
	stk_data->stk_max_min_diff = STK_MAX_MIN_DIFF;
	stk_data->stk_lt_n_ct = STK_LT_N_CT;
	stk_data->stk_ht_n_ct = STK_HT_N_CT;
	stk_data->stk_h_ht = STK_H_HT;
	stk_data->stk_h_lt = STK_H_LT;
#endif

#ifdef STK_QUALCOMM_POWER_CTRL

	err = stk3x3x_power_init(stk_data, true);
	if (err)
		goto err_power_on;

	err = stk3x3x_power_ctl(stk_data, true);
	if (err)
		goto err_power_on;
	msleep(3);
	err = stk3x3x_check_pid(stk_data);
	if(err < 0)
		goto err_init_all_setting;

	stk_data->als_enabled = false;
	stk_data->ps_enabled = false;
#endif
	err = stk3x3x_init_all_setting(client, plat_data);
	if(err < 0)
		goto err_init_all_setting;


	err = stk3x3x_set_input_devices(stk_data);
	if(err < 0)
		goto err_setup_input_device;

#if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS))
	err = stk3x3x_setup_irq(client);
	if(err < 0)
		goto err_stk3x3x_setup_irq;
#endif
	device_init_wakeup(&client->dev, true);

#ifdef QUALCOMM_PLATFORM

	stk_data->ps_cdev = sensors_proximity_cdev;
	stk_data->ps_cdev.sensors_enable = stk_ps_enable_set;
	err = sensors_classdev_register(&stk_data->ps_input_dev->dev, &stk_data->ps_cdev);
	if (err)
		goto err_class_sysfs;
#endif

#ifdef STK_QUALCOMM_POWER_CTRL
	/* enable device power only when it is enabled */
	err = stk3x3x_power_ctl(stk_data, false);
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
	sensors_classdev_unregister(&stk_data->ps_cdev);
err_class_sysfs:
	sensors_classdev_unregister(&stk_data->als_cdev);
#endif
#if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS))
err_stk3x3x_setup_irq:
	free_irq(stk_data->irq, stk_data);
#ifdef SPREADTRUM_PLATFORM
	sprd_free_gpio_irq(stk_data->int_pin);
#else
	gpio_free(stk_data->int_pin);
#endif
#endif
	sensor_remove_sysfs_interfaces(stk_data->als_dev,
	                               stk_als_attrs, ARRAY_SIZE(stk_als_attrs));
	sensor_remove_sysfs_interfaces(stk_data->ps_dev,
	                               stk_prox_attrs, ARRAY_SIZE(stk_prox_attrs));

	input_unregister_device(stk_data->ps_input_dev);
	input_unregister_device(stk_data->als_input_dev);
err_setup_input_device:
err_init_all_setting:
#ifdef STK_QUALCOMM_POWER_CTRL
	stk3x3x_power_ctl(stk_data, false);
err_power_on:
	stk3x3x_power_init(stk_data, false);
#endif
#ifdef STK_POLL_ALS
	hrtimer_try_to_cancel(&stk_data->als_timer);
	destroy_workqueue(stk_data->stk_als_wq);
#endif
#ifdef STK_TUNE0
	destroy_workqueue(stk_data->stk_ps_tune0_wq);
#endif
#ifdef STK_POLL_PS
	hrtimer_try_to_cancel(&stk_data->ps_timer);
	destroy_workqueue(stk_data->stk_ps_wq);
#endif
#if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS))
	destroy_workqueue(stk_data->stk_wq);
#endif
err_stk_input_allocate:
err_als_input_allocate:
#ifdef STK_POLL_PS
	wake_lock_destroy(&stk_data->ps_nosuspend_wl);
#endif
	wake_lock_destroy(&stk_data->ps_wakelock);
	mutex_destroy(&stk_data->io_lock);
	kfree(stk_data);
	return err;
}


static int stk3x3x_remove(struct i2c_client *client)
{
	struct stk3x3x_data *stk_data = i2c_get_clientdata(client);

	device_init_wakeup(&client->dev, false);
#ifdef STK_QUALCOMM_POWER_CTRL
	stk3x3x_power_ctl(stk_data, false);
#endif
#ifdef QUALCOMM_PLATFORM
	sensors_classdev_unregister(&stk_data->ps_cdev);
	sensors_classdev_unregister(&stk_data->als_cdev);
#endif
#ifdef STK_QUALCOMM_POWER_CTRL
	stk3x3x_power_init(stk_data, false);
#endif
#if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS))
	free_irq(stk_data->irq, stk_data);
#ifdef SPREADTRUM_PLATFORM
	sprd_free_gpio_irq(stk_data->int_pin);
#else
	gpio_free(stk_data->int_pin);
#endif
#endif  /* #if (!defined(STK_POLL_PS)) */

	sensor_remove_sysfs_interfaces(stk_data->als_dev,
	                               stk_als_attrs, ARRAY_SIZE(stk_als_attrs));
	sensor_remove_sysfs_interfaces(stk_data->ps_dev,
	                               stk_prox_attrs, ARRAY_SIZE(stk_prox_attrs));

	input_unregister_device(stk_data->ps_input_dev);
	input_unregister_device(stk_data->als_input_dev);

#ifdef STK_POLL_ALS
	hrtimer_try_to_cancel(&stk_data->als_timer);
	destroy_workqueue(stk_data->stk_als_wq);
#endif
#ifdef STK_TUNE0
	destroy_workqueue(stk_data->stk_ps_tune0_wq);
#endif
#ifdef STK_POLL_PS
	hrtimer_try_to_cancel(&stk_data->ps_timer);
	destroy_workqueue(stk_data->stk_ps_wq);
#if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS))
	destroy_workqueue(stk_data->stk_wq);
#endif
	wake_lock_destroy(&stk_data->ps_nosuspend_wl);
#endif
	wake_lock_destroy(&stk_data->ps_wakelock);
	mutex_destroy(&stk_data->io_lock);
	kfree(stk_data);

	return 0;
}

static const struct i2c_device_id stk_ps_id[] = {
	{ "DEVICE_NAME", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, stk_ps_id);

static struct of_device_id stk_match_table[] = {
	{ .compatible = "stk,stk3x3x", },
	{ },
};

static struct i2c_driver stk_ps_driver = {
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
	if (ret) {
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
