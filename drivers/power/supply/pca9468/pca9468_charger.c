/*
 * Driver for the NXP PCA9468 battery charger.
 *
 * Copyright (C) 2018 NXP Semiconductor.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/power_supply.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/rtc.h>
#include <linux/debugfs.h>
#include "pca9468_charger.h"
#include "pca9468-step-chg.h"

#if defined (CONFIG_OF)
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif /* CONFIG_OF */

#include <linux/pm_wakeup.h>
#ifdef CONFIG_USBPD_PHY_QCOM
#include <linux/usb/usbpd.h>		// Use Qualcomm USBPD PHY
#endif

#define BITS(_end, _start) ((BIT(_end) - BIT(_start)) + BIT(_end))
#define MASK2SHIFT(_mask)	__ffs(_mask)
#define MIN(a, b)	((a < b) ? (a):(b))
#define MAX(a, b)	((a > b) ? (a):(b))

//
// Register Map
//
#define PCA9468_REG_DEVICE_INFO 		0x00	// Device ID, revision
#define PCA9468_BIT_DEV_REV				BITS(7,4)
#define PCA9468_BIT_DEV_ID				BITS(3,0)
#define PCA9468_DEVICE_ID				0x18	// Default ID

#define PCA9468_REG_INT1				0x01	// Interrupt register
#define PCA9468_BIT_V_OK_INT			BIT(7)
#define PCA9468_BIT_NTC_TEMP_INT		BIT(6)
#define PCA9468_BIT_CHG_PHASE_INT		BIT(5)
#define PCA9468_BIT_CTRL_LIMIT_INT		BIT(3)
#define PCA9468_BIT_TEMP_REG_INT		BIT(2)
#define PCA9468_BIT_ADC_DONE_INT		BIT(1)
#define PCA9468_BIT_TIMER_INT			BIT(0)

#define PCA9468_REG_INT1_MSK			0x02	// INT mask for INT1 register
#define PCA9468_BIT_V_OK_M				BIT(7)
#define PCA9468_BIT_NTC_TEMP_M			BIT(6)
#define PCA9468_BIT_CHG_PHASE_M			BIT(5)
#define PCA9468_BIT_RESERVED_M			BIT(4)
#define PCA9468_BIT_CTRL_LIMIT_M		BIT(3)
#define PCA9468_BIT_TEMP_REG_M			BIT(2)
#define PCA9468_BIT_ADC_DONE_M			BIT(1)
#define PCA9468_BIT_TIMER_M				BIT(0)

#define PCA9468_REG_INT1_STS			0x03	// INT1 status register
#define PCA9468_BIT_V_OK_STS			BIT(7)
#define PCA9468_BIT_NTC_TEMP_STS		BIT(6)
#define PCA9468_BIT_CHG_PHASE_STS		BIT(5)
#define PCA9468_BIT_CTRL_LIMIT_STS		BIT(3)
#define PCA9468_BIT_TEMP_REG_STS		BIT(2)
#define PCA9468_BIT_ADC_DONE_STS		BIT(1)
#define PCA9468_BIT_TIMER_STS			BIT(0)

#define PCA9468_REG_STS_A				0x04	// INT1 status register
#define PCA9468_BIT_IIN_LOOP_STS		BIT(7)
#define PCA9468_BIT_CHG_LOOP_STS		BIT(6)
#define PCA9468_BIT_VFLT_LOOP_STS		BIT(5)
#define PCA9468_BIT_CFLY_SHORT_STS		BIT(4)
#define PCA9468_BIT_VOUT_UV_STS			BIT(3)
#define PCA9468_BIT_VBAT_OV_STS			BIT(2)
#define PCA9468_BIT_VIN_OV_STS			BIT(1)
#define PCA9468_BIT_VIN_UV_STS			BIT(0)

#define PCA9468_REG_STS_B				0x05	// INT1 status register
#define PCA9468_BIT_BATT_MISS_STS		BIT(7)
#define PCA9468_BIT_OCP_FAST_STS		BIT(6)
#define PCA9468_BIT_OCP_AVG_STS			BIT(5)
#define PCA9468_BIT_ACTIVE_STATE_STS	BIT(4)
#define PCA9468_BIT_SHUTDOWN_STATE_STS	BIT(3)
#define PCA9468_BIT_STANDBY_STATE_STS	BIT(2)
#define PCA9468_BIT_CHARGE_TIMER_STS	BIT(1)
#define PCA9468_BIT_WATCHDOG_TIMER_STS	BIT(0)

#define PCA9468_REG_STS_C				0x06	// IIN status
#define PCA9468_BIT_IIN_STS				BITS(7,2)

#define PCA9468_REG_STS_D				0x07	// ICHG status
#define PCA9468_BIT_ICHG_STS			BITS(7,1)

#define PCA9468_REG_STS_ADC_1			0x08	// ADC register
#define PCA9468_BIT_ADC_IIN7_0			BITS(7,0)

#define PCA9468_REG_STS_ADC_2			0x09	// ADC register
#define PCA9468_BIT_ADC_IOUT5_0			BITS(7,2)
#define PCA9468_BIT_ADC_IIN9_8			BITS(1,0)

#define PCA9468_REG_STS_ADC_3			0x0A	// ADC register
#define PCA9468_BIT_ADC_VIN3_0			BITS(7,4)
#define PCA9468_BIT_ADC_IOUT9_6			BITS(3,0)

#define PCA9468_REG_STS_ADC_4			0x0B	// ADC register
#define PCA9468_BIT_ADC_VOUT1_0			BITS(7,6)
#define PCA9468_BIT_ADC_VIN9_4			BITS(5,0)

#define PCA9468_REG_STS_ADC_5			0x0C	// ADC register
#define PCA9468_BIT_ADC_VOUT9_2			BITS(7,0)

#define PCA9468_REG_STS_ADC_6			0x0D	// ADC register
#define PCA9468_BIT_ADC_VBAT7_0			BITS(7,0)

#define PCA9468_REG_STS_ADC_7			0x0E	// ADC register
#define PCA9468_BIT_ADC_DIETEMP5_0		BITS(7,2)
#define PCA9468_BIT_ADC_VBAT9_8			BITS(1,0)

#define PCA9468_REG_STS_ADC_8			0x0F	// ADC register
#define PCA9468_BIT_ADC_NTCV3_0			BITS(7,4)
#define PCA9468_BIT_ADC_DIETEMP9_6		BITS(3,0)

#define PCA9468_REG_STS_ADC_9			0x10	// ADC register
#define PCA9468_BIT_ADC_NTCV9_4			BITS(5,0)

#define PCA9468_REG_ICHG_CTRL			0x20	// Change current configuration
#define PCA9468_BIT_ICHG_SS				BIT(7)
#define PCA9468_BIT_ICHG_CFG			BITS(6,0)

#define PCA9468_REG_IIN_CTRL			0x21	// Input current configuration
#define PCA9468_BIT_LIMIT_INCREMENT_EN	BIT(7)
#define PCA9468_BIT_IIN_SS				BIT(6)
#define PCA9468_BIT_IIN_CFG				BITS(5,0)

#define PCA9468_REG_START_CTRL			0x22	// Device initialization configuration
#define PCA9468_BIT_SNSRES				BIT(7)
#define PCA9468_BIT_EN_CFG				BIT(6)
#define PCA9468_BIT_STANDBY_EN			BIT(5)
#define PCA9468_BIT_REV_IIN_DET			BIT(4)
#define PCA9468_BIT_FSW_CFG				BITS(3,0)

#define PCA9468_REG_ADC_CTRL			0x23	// ADC configuration
#define PCA9468_BIT_FORCE_ADC_MODE		BITS(7,6)
#define PCA9468_BIT_ADC_SHUTDOWN_CFG	BIT(5)
#define PCA9468_BIT_HIBERNATE_DELAY		BITS(4,3)

#define PCA9468_REG_ADC_CFG				0x24	// ADC channel configuration
#define PCA9468_BIT_CH7_EN				BIT(7)
#define PCA9468_BIT_CH6_EN				BIT(6)
#define PCA9468_BIT_CH5_EN				BIT(5)
#define PCA9468_BIT_CH4_EN				BIT(4)
#define PCA9468_BIT_CH3_EN				BIT(3)
#define PCA9468_BIT_CH2_EN				BIT(2)
#define PCA9468_BIT_CH1_EN				BIT(1)

#define PCA9468_REG_TEMP_CTRL			0x25	// Temperature configuration
#define PCA9468_BIT_TEMP_REG			BITS(7,6)
#define PCA9468_BIT_TEMP_DELTA			BITS(5,4)
#define PCA9468_BIT_TEMP_REG_EN			BIT(3)
#define PCA9468_BIT_NTC_PROTECTION_EN	BIT(2)
#define PCA9468_BIT_TEMP_MAX_EN			BIT(1)

#define PCA9468_REG_PWR_COLLAPSE		0x26	// Power collapse configuration
#define PCA9468_BIT_UV_DELTA			BITS(7,6)
#define PCA9468_BIT_IIN_FORCE_COUNT		BIT(4)
#define PCA9468_BIT_BAT_MISS_DET_EN		BIT(3)

#define PCA9468_REG_V_FLOAT				0x27	// Voltage regulation configuration
#define PCA9468_BIT_V_FLOAT				BITS(7,0)

#define PCA9468_REG_SAFETY_CTRL			0x28	// Safety configuration
#define PCA9468_BIT_WATCHDOG_EN			BIT(7)
#define PCA9468_BIT_WATCHDOG_CFG		BITS(6,5)
#define PCA9468_BIT_CHG_TIMER_EN		BIT(4)
#define PCA9468_BIT_CHG_TIMER_CFG		BITS(3,2)
#define PCA9468_BIT_OV_DELTA			BITS(1,0)

#define PCA9468_REG_NTC_TH_1			0x29	// Thermistor threshold configuration
#define PCA9468_BIT_NTC_THRESHOLD7_0	BITS(7,0)

#define PCA9468_REG_NTC_TH_2			0x2A	// Thermistor threshold configuration
#define PCA9468_BIT_NTC_THRESHOLD9_8	BITS(1,0)

#define PCA9468_REG_ADC_ACCESS			0x30

#define PCA9468_REG_ADC_ADJUST			0x31
#define PCA9468_BIT_ADC_GAIN			BITS(7,4)

#define PCA9468_REG_ADC_IMPROVE			0x3D
#define PCA9468_BIT_ADC_IIN_IMP			BIT(3)

#define PCA9468_REG_ADC_MODE			0x3F
#define PCA9468_BIT_ADC_MODE			BIT(4)

#define PCA9468_MAX_REGISTER			0x4F


#define PCA9468_IIN_CFG_STEP			100000	// input current step, unit - uA
#define PCA9468_IIN_CFG(_input_current)	(_input_current/PCA9468_IIN_CFG_STEP)	// input current, unit - uA
#define PCA9468_ICHG_CFG(_chg_current)	(_chg_current/100000)					// charging current, uint - uA
#define PCA9468_V_FLOAT(_v_float)		((_v_float/1000 - 3725)/5)				// v_float voltage, unit - uV

#define PCA9468_SNSRES_5mOhm			0x00
#define PCA9468_SNSRES_10mOhm			PCA9468_BIT_SNSRES

#define PCA9468_NTC_TH_STEP				2346	// 2.346mV, unit - uV

/* VIN over voltage setting from 2*VOUT */
enum {
	OV_DELTA_10P,
	OV_DELTA_30P,
	OV_DELTA_20P,
	OV_DELTA_40P,
};

/* Switching frequency */
enum {
	FSW_CFG_833KHZ,
	FSW_CFG_893KHZ,
	FSW_CFG_935KHZ,
	FSW_CFG_980KHZ,
	FSW_CFG_1020KHZ,
	FSW_CFG_1080KHZ,
	FSW_CFG_1120KHZ,
	FSW_CFG_1160KHZ,
	FSW_CFG_440KHZ,
	FSW_CFG_490KHZ,
	FSW_CFG_540KHZ,
	FSW_CFG_590KHZ,
	FSW_CFG_630KHZ,
	FSW_CFG_680KHZ,
	FSW_CFG_730KHZ,
	FSW_CFG_780KHZ
};

/* Enable pin polarity selection */
#define PCA9468_EN_ACTIVE_H		0x00
#define PCA9468_EN_ACTIVE_L		PCA9468_BIT_EN_CFG
#define PCA9468_STANDBY_FORCED	PCA9468_BIT_STANDBY_EN
#define PCA9468_STANDBY_DONOT	0

/* ADC Channel */
enum {
	ADCCH_VOUT = 1,
	ADCCH_VIN,
	ADCCH_VBAT,
	ADCCH_ICHG,
	ADCCH_IIN,
	ADCCH_DIETEMP,
	ADCCH_NTC,
	ADCCH_MAX
};

/* ADC step */
#define VIN_STEP		16000	// 16mV(16000uV) LSB, Range(0V ~ 16.368V)
#define VBAT_STEP		5000	// 5mV(5000uV) LSB, Range(0V ~ 5.115V)
#define IIN_STEP		4890 	// 4.89mA(4890uA) LSB, Range(0A ~ 5A)
#define ICHG_STEP		9780 	// 9.78mA(9780uA) LSB, Range(0A ~ 10A)
#define DIETEMP_STEP  	435		// 0.435C LSB, Range(-25C ~ 160C)
#define DIETEMP_DENOM	1000	// 1000, denominator
#define DIETEMP_MIN 	-25  	// -25C
#define DIETEMP_MAX		160		// 160C
#define VOUT_STEP		5000 	// 5mV(5000uV) LSB, Range(0V ~ 5.115V)
#define NTCV_STEP		2346 	// 2.346mV(2346uV) LSB, Range(0V ~ 2.4V)
#define ADC_IIN_OFFSET	900000	// 900mA

/* adc_gain bit[7:4] of reg 0x31 - 2's complement */
static int adc_gain[16] = { 0, 1, 2, 3, 4, 5, 6, 7, -8, -7, -6, -5, -4, -3, -2, -1 };

/* Timer definition */
#define PCA9468_VBATMIN_CHECK_T	1000	// 1000ms
#define PCA9468_CCMODE_CHECK1_T	5000	// 10000ms -> 5000ms
#define PCA9468_CCMODE_CHECK2_T	5000	// 5000ms
#define PCA9468_CVMODE_CHECK_T 	10000	// 10000ms
#define PCA9468_PDMSG_WAIT_T	200		// 200ms
#define PCA4968_ENABLE_DELAY_T	150		// 150ms
#define PCA9468_PPS_PERIODIC_T	10000	// 10000ms

/* Battery Threshold */
#define PCA9468_DC_VBAT_MIN		3500000	// 3500000uV
/* Input Current Limit default value */
#define PCA9468_IIN_CFG_DFT		2500000	// 2500000uA
/* Charging Current default value */
#define PCA9468_ICHG_CFG_DFT	8000000	// 8000000uA
/* Charging Float Voltage default value */
#define PCA9468_VFLOAT_DFT		4350000	// 4350000uV
/* Charging Sub Float Voltage default value */
#define PCA9468_VFLOAT_SUB_DFT	5000000	// 5000000uV
/* Sense Resistance default value */
#define PCA9468_SENSE_R_DFT		1		// 10mOhm
/* Switching Frequency default value */
#define PCA9468_FSW_CFG_DFT		3		// 980KHz
/* NTC threshold voltage default value */
#define PCA9468_NTC_TH_DFT		0		// 0uV

/* Charging Done Condition */
#define PCA9468_ICHG_DONE_DFT	1000000	// 1000mA
#define PCA9468_IIN_DONE_DFT	500000	// 500mA
/* parallel charging done conditoin */
#define PCA9468_IIN_P_DONE		1000000	// 1000mA
/* Parallel charging default threshold */
#define PCA9468_IIN_P_TH_DFT	4000000	// 4000mA
/* Single charging default threshold */
#define PCA9468_IIN_S_TH_DFT	10000000	// 10000mA

/* Maximum TA voltage threshold */
#define PCA9468_TA_MAX_VOL		9800000 // 9800000uV
/* Maximum TA current threshold */
#define PCA9468_TA_MAX_CUR		2500000	// 2500000uA
/* Minimum TA current threshold */
#define PCA9468_TA_MIN_CUR		1000000	// 1000000uA - PPS minimum current

/* Minimum TA voltage threshold in Preset mode */
#define PCA9468_TA_MIN_VOL_PRESET	8000000	// 8000000uV
/* TA voltage threshold starting Adjust CC mode */
#define PCA9468_TA_MIN_VOL_CCADJ	8500000	// 8000000uV-->8500000uV

#define PCA9468_TA_VOL_PRE_OFFSET	600000	// 600000uV
/* Adjust CC mode TA voltage step */
#define PCA9468_TA_VOL_STEP_ADJ_CC	40000	// 40000uV
/* Pre CV mode TA voltage step */
#define PCA9468_TA_VOL_STEP_PRE_CV	20000	// 20000uV
/* Pre CC mode TA voltage step */
#define PCA9468_TA_VOL_STEP_PRE_CC	100000	// 100000uV
/* IIN_CC adc offset for accuracy */
#define PCA9468_IIN_ADC_OFFSET		20000	// 20000uA
/* IIN_CC compensation offset */
#define PCA9468_IIN_CC_COMP_OFFSET	50000	// 50000uA
/* IIN_CC compensation offset in Power Limit Mode(Constant Power) TA */
#define PCA9468_IIN_CC_COMP_OFFSET_CP	20000	// 20000uA
/* TA maximum voltage that can support constant current in Constant Power Mode */
#define PCA9468_TA_MAX_VOL_CP		9800000	// 9760000uV --> 9800000uV
/* maximum retry counter for restarting charging */
#define PCA9468_MAX_RETRY_CNT		3		// 3times
/* TA IIN tolerance */
#define PCA9468_TA_IIN_OFFSET		100000	// 100mA
/* IIN_CC upper protection offset in Power Limit Mode TA */
#define PCA9468_IIN_CC_UPPER_OFFSET	50000	// 50mA

/* PD Message Voltage and Current Step */
#define PD_MSG_TA_VOL_STEP			20000	// 20mV
#define PD_MSG_TA_CUR_STEP			50000	// 50mA

#define SWCHG_ICL_MIN				50000	// 50mA
#define SWCHG_ICL_NORMAL		    3000000 // 3000mA


/* INT1 Register Buffer */
enum {
	REG_INT1,
	REG_INT1_MSK,
	REG_INT1_STS,
	REG_INT1_MAX
};

/* STS Register Buffer */
enum {
	REG_STS_A,
	REG_STS_B,
	REG_STS_C,
	REG_STS_D,
	REG_STS_MAX
};

/* Direct Charging State */
enum {
	DC_STATE_NO_CHARGING,	/* No charging */
	DC_STATE_CHECK_VBAT,	/* Check min battery level */
	DC_STATE_PRESET_DC, 	/* Preset TA voltage/current for the direct charging */
	DC_STATE_CHECK_ACTIVE,	/* Check active status before entering Adjust CC mode */
	DC_STATE_ADJUST_CC,		/* Adjust CC mode */
	DC_STATE_START_CC,		/* Start CC mode */
	DC_STATE_CC_MODE,		/* Check CC mode status */
	DC_STATE_START_CV,		/* Start CV mode */
	DC_STATE_CV_MODE,		/* Check CV mode status */
	DC_STATE_CHARGING_DONE,	/* Charging Done */
	DC_STATE_ADJUST_TAVOL,	/* Adjust TA voltage to set new TA current under 1000mA input */
	DC_STATE_ADJUST_TACUR,	/* Adjust TA current to set new TA current over 1000mA input */
	DC_STATE_MAX,
};


/* CC Mode Status */
enum {
	CCMODE_CHG_LOOP,
	CCMODE_VFLT_LOOP,
	CCMODE_IIN_LOOP,
	CCMODE_LOOP_INACTIVE,
	CCMODE_VIN_UVLO,
};

/* CV Mode Status */
enum {
	CVMODE_CHG_LOOP,
	CVMODE_VFLT_LOOP,
	CVMODE_IIN_LOOP,
	CVMODE_LOOP_INACTIVE,
	CVMODE_CHG_DONE,
	CVMODE_VIN_UVLO,
};

/* Timer ID */
enum {
	TIMER_ID_NONE,
	TIMER_VBATMIN_CHECK,
	TIMER_PRESET_DC,
	TIMER_PRESET_CONFIG,
	TIMER_CHECK_ACTIVE,
	TIMER_ADJUST_CCMODE,
	TIMER_ENTER_CCMODE,
	TIMER_CHECK_CCMODE,
	TIMER_ENTER_CVMODE,
	TIMER_CHECK_CVMODE,
	TIMER_PDMSG_SEND,
	TIMER_ADJUST_TAVOL,
	TIMER_ADJUST_TACUR,
};

/* PD Message Type */
enum {
	PD_MSG_REQUEST_APDO,
	PD_MSG_REQUEST_FIXED_PDO,
};

/* TA increment Type */
enum {
	INC_NONE,	/* No increment */
	INC_TA_VOL, /* TA voltage increment */
	INC_TA_CUR, /* TA current increment */
};

/* TA Mode for the direct charging */
enum {
	TA_NO_DC_MODE,
	TA_2TO1_DC_MODE,
	TA_4TO1_DC_MODE,
	WC_DC_MODE,
};

/* IIN offset as the switching frequency in uA*/
static int iin_fsw_cfg[16] = { 9990, 10540, 11010, 11520, 12000, 12520, 12990, 13470,
								5460, 6050, 6580, 7150, 7670, 8230, 8720, 9260 };

/**
 * struct pca9468_charger - pca9468 charger instance
 * @monitor_wake_lock: lock to enter the suspend mode
 * @lock: protects concurrent access to online variables
 * @dev: pointer to device
 * @regmap: pointer to driver regmap
 * @mains: power_supply instance for AC/DC power
 * @dc_wq: work queue for the algorithm and monitor timer
 * @timer_work: timer work for charging
 * @timer_id: timer id for timer_work
 * @timer_period: timer period for timer_work
 * @last_update_time: last update time after sleep
 * @pps_work: pps work for PPS periodic time
 * @pd: phandle for qualcomm PMI usbpd-phy
 * @mains_online: is AC/DC input connected
 * @charging_state: direct charging state
 * @ret_state: return direct charging state after DC_STATE_ADJUST_TAVOL is done
 * @iin_cc: input current for the direct charging in cc mode, uA
 * @ta_cur: AC/DC(TA) current, uA
 * @ta_vol: AC/DC(TA) voltage, uV
 * @ta_objpos: AC/DC(TA) PDO object position
 * @ta_target_vol: TA target voltage before any compensation
 * @ta_max_cur: TA maximum current of APDO, uA
 * @ta_max_vol: TA maximum voltage for the direct charging, uV
 * @ta_max_pwr: TA maximum power, uW
 * @prev_iin: Previous IIN ADC of PCA9468, uA
 * @prev_inc: Previous TA voltage or current increment factor
 * @req_new_iin: Request for new input current limit, true or false
 * @req_new_vfloat: Request for new vfloat, true or false
 * @new_iin: New request input current limit, uA
 * @new_vfloat: New request vfloat, uV
 * @adc_comp_gain: adc gain for compensation
 * @retry_cnt: retry counter for re-starting charging if charging stop happens
 * @chg_mode: charging mode that TA can support for the direct charging, 2:1 or 4:1 mode
 * @parallel_chg: parallel charging with sub charger
 * @i2c_sub: sub charger i2c client
 * @regmap2: pointer to driver regmap2, sub charger
 * @sub_chg: power_supply instance for sub charger
 * @adc_comp_gain_sub: adc gain for compensation in sub charger
 * @pdata: pointer to platform data
 * @debug_root: debug entry
 * @debug_address: debug register address
 */
struct pca9468_charger {
	struct wakeup_source	monitor_wake_lock;
	struct mutex		lock;
	struct device		*dev;
	struct regmap		*regmap;
	struct power_supply	*mains;
	
	struct workqueue_struct *dc_wq;
	struct delayed_work timer_work;
	unsigned int		timer_id;
	unsigned long      	timer_period;
	unsigned long		last_update_time;

	struct delayed_work	pps_work;
#ifdef CONFIG_USBPD_PHY_QCOM
	struct usbpd 		*pd;
	int nubia_fast_charge;
#endif

	bool				mains_online;
	unsigned int 		charging_state;
	unsigned int		ret_state;

	unsigned int		iin_cc;
	
	unsigned int		ta_cur;
	unsigned int		ta_vol;
	unsigned int		ta_objpos;

	unsigned int		ta_target_vol;

	unsigned int		ta_max_cur;
	unsigned int		ta_max_vol;
	unsigned int		ta_max_pwr;

	unsigned int		prev_iin;
	unsigned int		prev_inc;

	bool				req_new_iin;
	bool				req_new_vfloat;
	unsigned int		new_iin;
	unsigned int		new_vfloat;
	
	int					adc_comp_gain;

	int					retry_cnt;

	int					chg_mode;

	bool				parallel_chg;

	struct i2c_client 	*i2c_sub;
	struct regmap		*regmap2;
	struct power_supply	*sub_chg;
	int					adc_comp_gain_sub;

	struct pca9468_platform_data *pdata;

	/* debug */
	struct dentry		*debug_root;
	u32					debug_address;
};


#ifdef CONFIG_USBPD_PHY_QCOM
static int pca9468_usbpd_setup(struct pca9468_charger *pca9468);
#endif


/*******************************/
/* Switching charger control function */
/*******************************/
/* This function needs some modification by a customer */
static int pca9468_set_switching_charger( bool enable, 
					unsigned int input_current, 
					unsigned int charging_current, 
					unsigned int vfloat)
{
	int ret;
	struct power_supply *psy_swcharger;
	union power_supply_propval val;

	pr_info("%s: enable=%d, iin=%d, ichg=%d, vfloat=%d\n", 
		__func__, enable, input_current, charging_current, vfloat);
	
	/* Insert Code */

	/* Get power supply name */
#ifdef CONFIG_USBPD_PHY_QCOM
	psy_swcharger = power_supply_get_by_name("usb");
#else
	/* Change "sw-charger" to the customer's switching charger name */
	psy_swcharger = power_supply_get_by_name("sw-charger");
#endif
	if (psy_swcharger == NULL) {
		pr_err("%s: cannot get power_supply_name-usb\n", __func__);
		ret = -ENODEV;
		goto error;
	}
	
	if (enable == true)	{
		/* Set Switching charger */
#ifdef CONFIG_USBPD_PHY_QCOM
		/* input current */
		val.intval = input_current;
		ret = power_supply_set_property(psy_swcharger, POWER_SUPPLY_PROP_CURRENT_MAX, &val);
		if (ret < 0)
			goto error;

		/* it depends on customer's code to enable charger */
		val.intval = enable;
		ret = power_supply_set_property(psy_swcharger, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
		if (ret < 0)
			goto error;
#else
		/* input current */
		val.intval = input_current;
		ret = power_supply_set_property(psy_swcharger, POWER_SUPPLY_PROP_CURRENT_MAX, &val);
		if (ret < 0)
			goto error;
		/* charging current */
		val.intval = charging_current;
		ret = power_supply_set_property(psy_swcharger, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, &val);
		if (ret < 0)
			goto error;
		/* vfloat voltage */
		val.intval = vfloat;
		ret = power_supply_set_property(psy_swcharger, POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE, &val);
		if (ret < 0)
			goto error;

		/* it depends on customer's code to enable charger */
		val.intval = enable;
		ret = power_supply_set_property(psy_swcharger, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
		if (ret < 0)
			goto error;
#endif
	} else {
		/* disable charger */
		/* it depends on customer's code to disable charger */
		val.intval = enable;
		ret = power_supply_set_property(psy_swcharger, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
		if (ret < 0)
			goto error;
		
		/* input_current */
		val.intval = input_current;
		ret = power_supply_set_property(psy_swcharger, POWER_SUPPLY_PROP_CURRENT_MAX, &val);
		if (ret < 0)
			goto error;
	}

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

static int pca9468_get_switching_charger_property(enum power_supply_property prop,
						  union power_supply_propval *val)
{
	int ret;
	struct power_supply *psy_swcharger;

	/* Get power supply name */
#ifdef CONFIG_USBPD_PHY_QCOM
	psy_swcharger = power_supply_get_by_name("usb");
#else
	psy_swcharger = power_supply_get_by_name("sw-charger");
#endif
	if (psy_swcharger == NULL) {
		ret = -EINVAL;
		goto error;
	}
	ret = power_supply_get_property(psy_swcharger, prop, val);

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/******************/
/* Send PD message */
/******************/
/* Send Request message to the source */
/* This function needs some modification by a customer */
static int pca9468_send_pd_message(struct pca9468_charger *pca9468, unsigned int msg_type)
{
#ifdef CONFIG_USBPD_PHY_QCOM
#else
	u8 msg_buf[4];	/* Data Buffer for raw PD message */
	unsigned int max_cur;
	unsigned int op_cur, out_vol;
#endif
	int ret = 0;

	/* Cancel pps request timer */
	cancel_delayed_work(&pca9468->pps_work);

	mutex_lock(&pca9468->lock);

	if ((pca9468->charging_state == DC_STATE_NO_CHARGING) &&
		(msg_type == PD_MSG_REQUEST_APDO)) {
		/* Vbus reset happened in the previous PD communication */
		goto out;
	}
	
#ifdef CONFIG_USBPD_PHY_QCOM
	/* check the phandle */
	if (pca9468->pd == NULL) {
		pr_info("%s: get phandle\n", __func__);
		ret = pca9468_usbpd_setup(pca9468);
		if (ret != 0) {
			dev_err(pca9468->dev, "Error usbpd setup!\n");
			pca9468->pd = NULL;
			goto out;
		}
	}
#endif
	
	pr_info("%s: msg_type=%d, ta_cur=%d, ta_vol=%d, ta_objpos=%d\n", 
		__func__, msg_type, pca9468->ta_cur, pca9468->ta_vol, pca9468->ta_objpos);
		
	switch (msg_type) {
	case PD_MSG_REQUEST_APDO:
#ifdef CONFIG_USBPD_PHY_QCOM
		ret = usbpd_request_pdo(pca9468->pd, pca9468->ta_objpos, pca9468->ta_vol, pca9468->ta_cur);
		if (ret == -EBUSY) {
			/* wait 100ms */
			msleep(100);
			/* try again */
			ret = usbpd_request_pdo(pca9468->pd, pca9468->ta_objpos, pca9468->ta_vol, pca9468->ta_cur);
		}
#else
		op_cur = pca9468->ta_cur/50000; 	// Operating Current 50mA units
		out_vol = pca9468->ta_vol/20000;	// Output Voltage in 20mV units
		msg_buf[0] = op_cur & 0x7F; 		// Operating Current 50mA units - B6...0
		msg_buf[1] = (out_vol<<1) & 0xFE;	// Output Voltage in 20mV units - B19..(B15)..B9
		msg_buf[2] = (out_vol>>7) & 0x0F;	// Output Voltage in 20mV units - B19..(B16)..B9,
		msg_buf[3] = pca9468->ta_objpos<<4; // Object Position - B30...B28

		/* Send the PD message to CC/PD chip */
		/* Todo - insert code */
#endif
		/* Start pps request timer */
		if (ret == 0) {
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->pps_work,
								msecs_to_jiffies(PCA9468_PPS_PERIODIC_T));
		}
		break;
		
	case PD_MSG_REQUEST_FIXED_PDO:
#ifdef CONFIG_USBPD_PHY_QCOM
		ret = usbpd_request_pdo(pca9468->pd, pca9468->ta_objpos, pca9468->ta_vol, pca9468->ta_cur);
		if (ret == -EBUSY) {
			/* wait 100ms */
			msleep(100);
			/* try again */
			ret = usbpd_request_pdo(pca9468->pd, pca9468->ta_objpos, pca9468->ta_vol, pca9468->ta_cur);
		}
#else
		max_cur = pca9468->ta_cur/10000;	// Maximum Operation Current 10mA units
		op_cur = max_cur;					// Operating Current 10mA units
		msg_buf[0] = max_cur & 0xFF;		// Maximum Operation Current -B9..(7)..0
		msg_buf[1] = ((max_cur>>8) & 0x03) | ((op_cur<<2) & 0xFC);	// Operating Current - B19..(15)..10
		msg_buf[2] = ((op_cur>>6) & 0x0F);	// Operating Current - B19..(16)..10, Unchunked Extended Messages Supported  - not support
		msg_buf[3] = pca9468->ta_objpos<<4; // Object Position - B30...B28

		/* Send the PD message to CC/PD chip */
		/* Todo - insert code */
#endif
		break;

	default:
		break;
	}

out:
	if ((pca9468->charging_state == DC_STATE_NO_CHARGING) &&
		(msg_type == PD_MSG_REQUEST_APDO)) {
		/* Even though PD communication success, Vbus reset might happen
			So, check the charging state again */
		ret = -EINVAL;
	}
	
	pr_info("%s: ret=%d\n", __func__, ret);
	mutex_unlock(&pca9468->lock);
	return ret;
}

/************************/
/* Get APDO max power    */
/************************/
/* Get the max current/voltage/power of APDO from the CC/PD driver */
/* This function needs some modification by a customer */
static int pca9468_get_apdo_max_power(struct pca9468_charger *pca9468)
{
	int ret = 0;

#ifdef CONFIG_USBPD_PHY_QCOM
	/* check the phandle */
	if (pca9468->pd == NULL) {
		pr_info("%s: get phandle\n", __func__);
		ret = pca9468_usbpd_setup(pca9468);
		if (ret != 0) {
			dev_err(pca9468->dev, "Error usbpd setup!\n");
			pca9468->pd = NULL;
			goto out;
		}
	}

	/* Put ta_max_vol to the desired ta maximum value, ex) 9800mV */
	/* Get new ta_max_vol and ta_max_cur, ta_max_power and proper object position by CC/PD IC */
	ret = usbpd_get_apdo_max_power(pca9468->pd, &pca9468->ta_objpos, 
									&pca9468->ta_max_vol, &pca9468->ta_max_cur, &pca9468->ta_max_pwr);
#else
	/* Put ta_max_vol to the desired ta maximum value, ex) 9800mV */
	/* Get new ta_max_vol and ta_max_cur, ta_max_power and proper object position by CC/PD IC */

	/* insert code */
	goto out;
#endif

out:
	return ret;
}


/* ADC Read function */
static int pca9468_read_adc(struct pca9468_charger *pca9468, u8 adc_ch)
{
	u8 reg_data[2];
	u16 raw_adc;	// raw ADC value
	int conv_adc;	// conversion ADC value
	int ret;
	u8 rsense; /* sense resistance */
	
	switch (adc_ch)
	{
	case ADCCH_VOUT:
		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_4, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_VOUT9_2)<<2) | ((reg_data[0] & PCA9468_BIT_ADC_VOUT1_0)>>6);
		conv_adc = raw_adc * VOUT_STEP;	// unit - uV
		break;
	
	case ADCCH_VIN:
		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_3, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_VIN9_4)<<4) | ((reg_data[0] & PCA9468_BIT_ADC_VIN3_0)>>4);
		conv_adc = raw_adc * VIN_STEP;	// unit - uV
		break;

	case ADCCH_VBAT:
		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_6, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_VBAT9_8)<<8) | ((reg_data[0] & PCA9468_BIT_ADC_VBAT7_0)>>0);
		conv_adc = raw_adc * VBAT_STEP;		// unit - uV
		break;

	case ADCCH_ICHG:
		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_2, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC
		rsense = (pca9468->pdata->snsres == 0) ? 5 : 10;	// snsres : 0 - 5mOhm, 1 - 10mOhm
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_IOUT9_6)<<6) | ((reg_data[0] & PCA9468_BIT_ADC_IOUT5_0)>>2);
		conv_adc = raw_adc * ICHG_STEP;	// unit - uA
		break;

	case ADCCH_IIN:
		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_1, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC - iin = rawadc*4.89 + (rawadc*4.89 - 900)*adc_comp_gain/100, unit - uA
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_IIN9_8)<<8) | ((reg_data[0] & PCA9468_BIT_ADC_IIN7_0)>>0);
		conv_adc = raw_adc * IIN_STEP + (raw_adc * IIN_STEP - ADC_IIN_OFFSET)*pca9468->adc_comp_gain/100;		// unit - uA
		if (conv_adc < 0)
			conv_adc = 0;	// If ADC raw value is 0, convert value will be minus value because of compensation gain, so in this case conv_adc is 0
		break;

	
	case ADCCH_DIETEMP:
		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_7, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_DIETEMP9_6)<<6) | ((reg_data[0] & PCA9468_BIT_ADC_DIETEMP5_0)>>2);
		conv_adc = (935 - raw_adc)*DIETEMP_STEP/DIETEMP_DENOM; 	// Temp = (935-rawadc)*0.435, unit - C
		if (conv_adc > DIETEMP_MAX) {
			conv_adc = DIETEMP_MAX;
		} else if (conv_adc < DIETEMP_MIN) {
			conv_adc = DIETEMP_MIN;
		}
		break;

	case ADCCH_NTC:
		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_8, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_NTCV9_4)<<4) | ((reg_data[0] & PCA9468_BIT_ADC_NTCV3_0)>>4);
		conv_adc = raw_adc * NTCV_STEP;		// unit - uV
		break;

	default:
		conv_adc = -EINVAL;
		break;
	}

error:
	pr_info("%s: adc_ch=%d, convert_val=%d\n", __func__, adc_ch, conv_adc);

	return conv_adc;
}


static int pca9468_set_vfloat(struct pca9468_charger *pca9468, unsigned int v_float)
{
	int ret, val;

	pr_info("%s: vfloat=%d\n", __func__, v_float);

	/* v float voltage */
	val = PCA9468_V_FLOAT(v_float);
	ret = regmap_write(pca9468->regmap, PCA9468_REG_V_FLOAT, val);
	return ret;
}

static int pca9468_set_charging_current(struct pca9468_charger *pca9468, unsigned int ichg)
{
	int ret, val;

	pr_info("%s: ichg=%d\n", __func__, ichg);

	/* charging current */
	if (ichg > 8000000)
		ichg = 8000000;
	val = PCA9468_ICHG_CFG(ichg);
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_ICHG_CTRL, PCA9468_BIT_ICHG_CFG, val);
	return ret;
}

static int pca9468_set_input_current(struct pca9468_charger *pca9468, unsigned int iin)
{
	int ret, val;

	pr_info("%s: iin=%d\n", __func__, iin);


	/* input current */
	/* round-up and increase one step */
	iin = iin + PD_MSG_TA_CUR_STEP;
	val = PCA9468_IIN_CFG(iin);
	/* Set IIN_CFG to one step higher */
	val = val + 1;

	if (val > 0x32)
		val = 0x32;	/* maximum value is 5A */
	
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_IIN_CTRL, PCA9468_BIT_IIN_CFG, val);

	pr_info("%s: real iin_cfg=%d\n", __func__, val*PCA9468_IIN_CFG_STEP);
	return ret;
}

static int pca9468_set_charging(struct pca9468_charger *pca9468, bool enable)
{
	int ret, val;

	pr_info("%s: enable=%d\n", __func__, enable);

	if (enable == true) {
		/* Improve adc */
		val = 0x5B;
		ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_ACCESS, val);
		if (ret < 0)
			goto error;
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_ADC_IMPROVE, PCA9468_BIT_ADC_IIN_IMP, 0);
		if (ret < 0)
			goto error;

		/* For fixing input current error */
		/* Overwrite 0x00 in 0x41 register */
		val = 0x00;
		ret = regmap_write(pca9468->regmap, 0x41, val);
		if (ret < 0)
			goto error;
		/* Overwrite 0x01 in 0x43 register */
		val = 0x01;
		ret = regmap_write(pca9468->regmap, 0x43, val);
		if (ret < 0)
			goto error;
		/* Overwrite 0x00 in 0x4B register */
		val = 0x00;
		ret = regmap_write(pca9468->regmap, 0x4B, val);
		if (ret < 0)
			goto error;
		/* End for fixing input current error */

	} else {
		/* Disable NTC_PROTECTION_EN */
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_TEMP_CTRL, 
				PCA9468_BIT_NTC_PROTECTION_EN, 0);
	}
	
	/* Enable PCA9468 */
	val = (enable == true) ? PCA9468_STANDBY_DONOT: PCA9468_STANDBY_FORCED;
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL, PCA9468_BIT_STANDBY_EN, val);
	if (ret < 0)
		goto error;
	
	if (enable == true) {
		/* Wait 50ms, first to keep the start-up sequence */
		mdelay(50);
		/* Wait 150ms */
		msleep(150);
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_ADC_IMPROVE, PCA9468_BIT_ADC_IIN_IMP, PCA9468_BIT_ADC_IIN_IMP);
		if (ret  < 0)
			goto error;
		
		val = 0x00;
		ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_ACCESS, val);

		/* Enable NTC_PROTECTION_EN */
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_TEMP_CTRL, 
				PCA9468_BIT_NTC_PROTECTION_EN, PCA9468_BIT_NTC_PROTECTION_EN);
	} else {
		/* Wait 5ms to keep the shutdown sequence */
		mdelay(5);
	}

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/* Check Active status */
static int pca9468_check_error(struct pca9468_charger *pca9468)
{
	int ret;
	unsigned int reg_val;
	int vbatt;
	
	/* Read STS_B */
	ret = regmap_read(pca9468->regmap, PCA9468_REG_STS_B, &reg_val);
	if (ret < 0)
		goto error;

	/* Read VBAT_ADC */
	vbatt = pca9468_read_adc(pca9468, ADCCH_VBAT);
	
	/* Check Active status */
	if (reg_val & PCA9468_BIT_ACTIVE_STATE_STS) {
		/* PCA9468 is active state */
		/* PCA9468 is in charging */
		/* Check whether the battery voltage is over the minimum voltage level or not */
		if (vbatt > PCA9468_DC_VBAT_MIN) {
			/* Normal charging battery level */
			/* Check temperature regulation loop */
			/* Read INT1_STS register */
			ret = regmap_read(pca9468->regmap, PCA9468_REG_INT1_STS, &reg_val);
			if (reg_val & PCA9468_BIT_TEMP_REG_STS) {				
				/* Over temperature protection */
				pr_err("%s: Device is in temperature regulation", __func__);
				ret = -EINVAL;
			} else {
				/* Normal temperature */
				ret = 0;
			}
		} else {
			/* Abnormal battery level */
			pr_err("%s: Error abnormal battery voltage=%d\n", __func__, vbatt);
			ret = -EINVAL;
		}
	} else {
		/* PCA9468 is not active state  - standby or shutdown */
		/* Stop charging in timer_work */
		
		/* Read all status register for debugging */
		u8 val[8];
		u8 test_val[16];	/* Dump for test register */
		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_INT1, &val[PCA9468_REG_INT1], 7);
		pr_err("%s: Error reg[1]=0x%x,[2]=0x%x,[3]=0x%x,[4]=0x%x,[5]=0x%x,[6]=0x%x,[7]=0x%x\n",
			__func__, val[1], val[2], val[3], val[4], val[5], val[6], val[7]);
		/* Read test register for debugging */
		ret = regmap_bulk_read(pca9468->regmap, 0x40, test_val, 16);
		pr_err("%s: Error reg[0x40]=0x%x,[0x41]=0x%x,[0x42]=0x%x,[0x43]=0x%x,[0x44]=0x%x,[0x45]=0x%x,[0x46]=0x%x,[0x47]=0x%x\n",
			__func__, test_val[0], test_val[1], test_val[2], test_val[3], test_val[4], test_val[5], test_val[6], test_val[7]);
		pr_err("%s: Error reg[0x48]=0x%x,[0x49]=0x%x,[0x4A]=0x%x,[0x4B]=0x%x,[0x4C]=0x%x,[0x4D]=0x%x,[0x4E]=0x%x,[0x4F]=0x%x\n",
			__func__, test_val[8], test_val[9], test_val[10], test_val[11], test_val[12], test_val[13], test_val[14], test_val[15]);

		/* Check INT1_STS first */
		if ((val[PCA9468_REG_INT1_STS] & PCA9468_BIT_V_OK_STS) != PCA9468_BIT_V_OK_STS) {
			/* VBUS is invalid */
			pr_err("%s: VOK is invalid", __func__);
			/* Check STS_A */
			if (val[PCA9468_REG_STS_A] & PCA9468_BIT_CFLY_SHORT_STS)
				pr_err("%s: Flying Cap is shorted to GND", __func__);	/* Flying cap is short to GND */
			else if (val[PCA9468_REG_STS_A] & PCA9468_BIT_VOUT_UV_STS)
				pr_err("%s: VOUT UV", __func__);	/* VOUT < VOUT_OK */
			else if (val[PCA9468_REG_STS_A] & PCA9468_BIT_VBAT_OV_STS)
				pr_err("%s: VBAT OV", __func__);	/* VBAT > VBAT_OV */
			else if (val[PCA9468_REG_STS_A] & PCA9468_BIT_VIN_OV_STS)
				pr_err("%s: VIN OV", __func__);		/* VIN > V_OV_FIXED or V_OV_TRACKING */
			else if (val[PCA9468_REG_STS_A] & PCA9468_BIT_VIN_UV_STS)
				pr_err("%s: VIN UV", __func__);		/* VIN < V_UVTH */
			else
				pr_err("%s: Invalid VIN or VOUT", __func__);

			ret = -EINVAL;
		} else if (val[PCA9468_REG_INT1_STS] & PCA9468_BIT_NTC_TEMP_STS) {
			/* NTC protection */
			int ntc_adc, ntc_th;
			/* NTC threshold */
			u8 reg_data[2];
			regmap_bulk_read(pca9468->regmap, PCA9468_REG_NTC_TH_1, reg_data, 2);
			ntc_th = ((reg_data[1] & PCA9468_BIT_NTC_THRESHOLD9_8)<<8) | reg_data[0];	/* uV unit */
			/* Read NTC ADC */
			ntc_adc = pca9468_read_adc(pca9468, ADCCH_NTC);	/* uV unit */
			pr_err("%s: NTC Protection, NTC_TH=%d(uV), NTC_ADC=%d(uV)", __func__, ntc_th, ntc_adc);
			
			ret = -EINVAL;
		} else if (val[PCA9468_REG_INT1_STS] & PCA9468_BIT_CTRL_LIMIT_STS) {
			/* OCP event happens */
			/* Check STS_B */
			if (val[PCA9468_REG_STS_B] & PCA9468_BIT_OCP_FAST_STS)
				pr_err("%s: IIN is over OCP_FAST", __func__); 	/* OCP_FAST happened */
			else if (val[PCA9468_REG_STS_B] & PCA9468_BIT_OCP_AVG_STS)
				pr_err("%s: IIN is over OCP_AVG", __func__);	/* OCP_AVG happened */
			else
				pr_err("%s: No Loop active", __func__);

			ret = -EINVAL;
		} else if (val[PCA9468_REG_INT1_STS] & PCA9468_BIT_TEMP_REG_STS) {
			/* Over temperature protection */
			pr_err("%s: Device is in temperature regulation", __func__);
			
			ret = -EINVAL;
		} else if (val[PCA9468_REG_INT1_STS] & PCA9468_BIT_TIMER_STS) {
			/* Check STS_B */
			if (val[PCA9468_REG_STS_B] & PCA9468_BIT_CHARGE_TIMER_STS)
				pr_err("%s: Charger timer is expired", __func__);
			else if (val[PCA9468_REG_STS_B] & PCA9468_BIT_WATCHDOG_TIMER_STS)
				pr_err("%s: Watchdog timer is expired", __func__);
			else
				pr_err("%s: Timer INT, but no timer STS", __func__);

			ret = -EINVAL;
		}
		else if (val[PCA9468_REG_STS_A] & PCA9468_BIT_CFLY_SHORT_STS) {
			/* Flying cap short */
			pr_err("%s: Flying Cap is shorted to GND", __func__);	/* Flying cap is short to GND */
			ret = -EINVAL;
		} else {
			/* There is no error, but not active state */
			if (reg_val & PCA9468_BIT_STANDBY_STATE_STS) {
				/* Standby state */
				/* Check the RCP condition, T_REVI_DET is 300ms */
				/* Wait 200ms */
				msleep(200);

				/* Check the charging state */
				/* Sometimes battery driver might call set_property function to stop charging during msleep
				    At this case, charging state would change DC_STATE_NO_CHARGING.
				    PCA9468 should stop checking RCP condition and exit timer_work
				*/
				if (pca9468->charging_state == DC_STATE_NO_CHARGING) {
					pr_err("%s: other driver forced to stop direct charging\n", __func__);
					ret = -EINVAL;
				} else {
					/* Keep the current charging state */
					/* Check PCA948 state again */
					/* Read STS_B */
					ret = regmap_read(pca9468->regmap, PCA9468_REG_STS_B, &reg_val);
					if (ret < 0)
						goto error;

					pr_info("%s:RCP check, STS_B=0x%x\n",	__func__, reg_val);

					/* Check Active status */
					if (reg_val & PCA9468_BIT_ACTIVE_STATE_STS) {
						/* RCP condition happened, but VIN is still valid */
						/* If VIN is increased, input current will increase over IIN_LOW level */
						/* Normal charging */
						pr_info("%s: RCP happened before, but VIN is valid\n", __func__);
						ret = 0;
					} else if (reg_val & PCA9468_BIT_STANDBY_STATE_STS) {
						/* It is not RCP condition */
						/* Need to retry if DC is in starting state */
						pr_err("%s: Any abnormal condition, retry\n", __func__);
						/* Dump register */
						ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_INT1, &val[PCA9468_REG_INT1], 7);
						pr_err("%s: Error reg[1]=0x%x,[2]=0x%x,[3]=0x%x,[4]=0x%x,[5]=0x%x,[6]=0x%x,[7]=0x%x\n",
								__func__, val[1], val[2], val[3], val[4], val[5], val[6], val[7]);
						ret = regmap_bulk_read(pca9468->regmap, 0x48, test_val, 3);
						pr_err("%s: Error reg[0x48]=0x%x,[0x49]=0x%x,[0x4a]=0x%x\n",
								__func__, test_val[0], test_val[1], test_val[2]);
						ret = -EAGAIN;
					} else {
						/* Shutdown State */
						pr_err("%s: Shutdown state\n", __func__);
						/* Dump register */
						ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_INT1, &val[PCA9468_REG_INT1], 7);
						pr_err("%s: Error reg[1]=0x%x,[2]=0x%x,[3]=0x%x,[4]=0x%x,[5]=0x%x,[6]=0x%x,[7]=0x%x\n",
								__func__, val[1], val[2], val[3], val[4], val[5], val[6], val[7]);
						ret = regmap_bulk_read(pca9468->regmap, 0x48, test_val, 3);
						pr_err("%s: Error reg[0x48]=0x%x,[0x49]=0x%x,[0x4a]=0x%x\n",
								__func__, test_val[0], test_val[1], test_val[2]);
						ret = -EINVAL;
					}
				}
			} else {
				/* PCA9468 is in shutdown state */
				pr_err("%s: PCA9468 is in shutdown state\n", __func__);
				ret = -EINVAL;
			}
		}
	}

error:
	pr_info("%s: Active Status=%d\n", __func__, ret);
	return ret;
}


/* Check CC Mode status */
static int pca9468_check_ccmode_status(struct pca9468_charger *pca9468)
{
	unsigned int reg_val;
	int ret;
	
	/* Read STS_A */
	ret = regmap_read(pca9468->regmap, PCA9468_REG_STS_A, &reg_val);
	if (ret < 0)
		goto error;

	/* Check STS_A */
	if (reg_val & PCA9468_BIT_VIN_UV_STS) {
		ret = CCMODE_VIN_UVLO;
	} else if (reg_val & PCA9468_BIT_CHG_LOOP_STS) {
		ret = CCMODE_CHG_LOOP;
	} else if (reg_val & PCA9468_BIT_VFLT_LOOP_STS) {
		ret = CCMODE_VFLT_LOOP;
	} else if (reg_val & PCA9468_BIT_IIN_LOOP_STS) {
		ret = CCMODE_IIN_LOOP;
	} else {
		ret = CCMODE_LOOP_INACTIVE;
	}

error:
	pr_info("%s: CCMODE Status=%d\n", __func__, ret);
	return ret;
}


/* Check CVMode Status */
static int pca9468_check_cvmode_status(struct pca9468_charger *pca9468)
{
	unsigned int val;
	int ret;

	/* Read STS_A */
	ret = regmap_read(pca9468->regmap, PCA9468_REG_STS_A, &val);
	if (ret < 0)
		goto error;

	/* Check STS_A */
	if (val & PCA9468_BIT_CHG_LOOP_STS)	{
		ret = CVMODE_CHG_LOOP;
	} else if (val & PCA9468_BIT_VFLT_LOOP_STS) {
		ret = CVMODE_VFLT_LOOP;
	} else if (val & PCA9468_BIT_IIN_LOOP_STS) {
		ret = CVMODE_IIN_LOOP;
	} else if (val & PCA9468_BIT_VIN_UV_STS) {
		ret = CVMODE_VIN_UVLO;
	} else {
		/* Any LOOP is inactive */
		ret = CVMODE_LOOP_INACTIVE;
	}
	
error:
	pr_info("%s: CVMODE Status=%d\n", __func__, ret);
	return ret;
}


/* ADC Read function */
static int pca9468_read_adc_sub(struct pca9468_charger *pca9468, u8 adc_ch)
{
	u8 reg_data[2];
	u16 raw_adc;	// raw ADC value
	int conv_adc;	// conversion ADC value
	int ret;
	u8 rsense; /* sense resistance */
	
	switch (adc_ch)
	{
	case ADCCH_VOUT:
		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap2, PCA9468_REG_STS_ADC_4, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_VOUT9_2)<<2) | ((reg_data[0] & PCA9468_BIT_ADC_VOUT1_0)>>6);
		conv_adc = raw_adc * VOUT_STEP;	// unit - uV
		break;
	
	case ADCCH_VIN:
		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap2, PCA9468_REG_STS_ADC_3, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_VIN9_4)<<4) | ((reg_data[0] & PCA9468_BIT_ADC_VIN3_0)>>4);
		conv_adc = raw_adc * VIN_STEP;	// uint - uV
		break;

	case ADCCH_VBAT:
		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap2, PCA9468_REG_STS_ADC_6, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_VBAT9_8)<<8) | ((reg_data[0] & PCA9468_BIT_ADC_VBAT7_0)>>0);
		conv_adc = raw_adc * VBAT_STEP;		// unit - uV
		break;

	case ADCCH_ICHG:
		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap2, PCA9468_REG_STS_ADC_2, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC
		rsense = (pca9468->pdata->snsres == 0) ? 5 : 10;	// snsres : 0 - 5mOhm, 1 - 10mOhm
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_IOUT9_6)<<6) | ((reg_data[0] & PCA9468_BIT_ADC_IOUT5_0)>>2);
		conv_adc = raw_adc * ICHG_STEP;	// unit - uA
		break;

	case ADCCH_IIN:
		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap2, PCA9468_REG_STS_ADC_1, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC - iin = rawadc*4.89 + (rawadc*4.89 - 900)*adc_comp_gain_sub/100, unit - uA
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_IIN9_8)<<8) | ((reg_data[0] & PCA9468_BIT_ADC_IIN7_0)>>0);
		conv_adc = raw_adc * IIN_STEP + (raw_adc * IIN_STEP - ADC_IIN_OFFSET)*pca9468->adc_comp_gain_sub/100;		// unit - uA
		if (conv_adc < 0)
			conv_adc = 0;	// If ADC raw value is 0, convert value will be minus value becasue of compensation gain, so in this case conv_adc is 0
		break;

	
	case ADCCH_DIETEMP:
		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap2, PCA9468_REG_STS_ADC_7, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_DIETEMP9_6)<<6) | ((reg_data[0] & PCA9468_BIT_ADC_DIETEMP5_0)>>2);
		conv_adc = (935 - raw_adc)*DIETEMP_STEP/DIETEMP_DENOM; 	// Temp = (935-rawadc)*0.435, unit - C
		if (conv_adc > DIETEMP_MAX) {
			conv_adc = DIETEMP_MAX;
		} else if (conv_adc < DIETEMP_MIN) {
			conv_adc = DIETEMP_MIN;
		}
		break;

	case ADCCH_NTC:
		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap2, PCA9468_REG_STS_ADC_8, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_NTCV9_4)<<4) | ((reg_data[0] & PCA9468_BIT_ADC_NTCV3_0)>>4);
		conv_adc = raw_adc * NTCV_STEP;		// unit - uV
		break;

	default:
		conv_adc = -EINVAL;
		break;
	}

error:
	pr_info("%s: adc_ch=%d, convert_val=%d\n", __func__, adc_ch, conv_adc);

	return conv_adc;
}


static int pca9468_set_vfloat_sub(struct pca9468_charger *pca9468, unsigned int v_float)
{
	int ret, val;

	pr_info("%s: vfloat=%d\n", __func__, v_float);

	/* v float voltage */
	val = PCA9468_V_FLOAT(v_float);
	ret = regmap_write(pca9468->regmap2, PCA9468_REG_V_FLOAT, val);
	return ret;
}

static int pca9468_set_charging_current_sub(struct pca9468_charger *pca9468, unsigned int ichg)
{
	int ret, val;

	pr_info("%s: ichg=%d\n", __func__, ichg);

	/* charging current */
	if (ichg > 8000000)
		ichg = 8000000;
	val = PCA9468_ICHG_CFG(ichg);
	ret = regmap_update_bits(pca9468->regmap2, PCA9468_REG_ICHG_CTRL, PCA9468_BIT_ICHG_CFG, val);
	return ret;
}

static int pca9468_set_input_current_sub(struct pca9468_charger *pca9468, unsigned int iin)
{
	int ret, val;

	pr_info("%s: iin=%d\n", __func__, iin);


	/* input current */
	/* round off and increase one step */
	iin = iin + PD_MSG_TA_CUR_STEP;
	val = PCA9468_IIN_CFG(iin);
	/* Set IIN_CFG to one step higher */
	val = val + 1;

	if (val > 0x32)
		val = 0x32;	/* maximum value is 5A */
	
	ret = regmap_update_bits(pca9468->regmap2, PCA9468_REG_IIN_CTRL, PCA9468_BIT_IIN_CFG, val);

	pr_info("%s: real iin_cfg=%d\n", __func__, val*PCA9468_IIN_CFG_STEP);
	return ret;
}

static int pca9468_set_charging_sub(struct pca9468_charger *pca9468, bool enable)
{
	int ret, val;

	pr_info("%s: enable=%d\n", __func__, enable);

	if (enable == true) {
		/* Improve adc */
		val = 0x5B;
		ret = regmap_write(pca9468->regmap2, PCA9468_REG_ADC_ACCESS, val);
		if (ret < 0)
			goto error;
		ret = regmap_update_bits(pca9468->regmap2, PCA9468_REG_ADC_IMPROVE, PCA9468_BIT_ADC_IIN_IMP, 0);
		if (ret < 0)
			goto error;
		
		/* For fixing input current error */
		/* Overwirte 0x00 in 0x41 register */
		val = 0x00;
		ret = regmap_write(pca9468->regmap2, 0x41, val);
		if (ret < 0)
			goto error;
		/* Overwirte 0x01 in 0x43 register */
		val = 0x01;
		ret = regmap_write(pca9468->regmap2, 0x43, val);
		if (ret < 0)
			goto error;

		/* Overwirte 0x00 in 0x4B register */
		val = 0x00;
		ret = regmap_write(pca9468->regmap2, 0x4B, val);
		if (ret < 0)
			goto error;
		/* End for fixing input current error */

	} else {
		/* Disable NTC_PROTECTION_EN */
		ret = regmap_update_bits(pca9468->regmap2, PCA9468_REG_TEMP_CTRL, 
				PCA9468_BIT_NTC_PROTECTION_EN, 0);
	}
	
	/* Enable PCA9468 */
	val = (enable == true) ? PCA9468_STANDBY_DONOT: PCA9468_STANDBY_FORCED;
	ret = regmap_update_bits(pca9468->regmap2, PCA9468_REG_START_CTRL, PCA9468_BIT_STANDBY_EN, val);
	if (ret < 0)
		goto error;
	
	if (enable == true) {
		/* Wait 50ms, first to keep the start-up sequence */
		mdelay(50);
		/* Wait 150ms */
		msleep(150);
		ret = regmap_update_bits(pca9468->regmap2, PCA9468_REG_ADC_IMPROVE, PCA9468_BIT_ADC_IIN_IMP, PCA9468_BIT_ADC_IIN_IMP);
		if (ret  < 0)
			goto error;
		
		val = 0x00;
		ret = regmap_write(pca9468->regmap2, PCA9468_REG_ADC_ACCESS, val);

		/* Enable NTC_PROTECTION_EN */
		ret = regmap_update_bits(pca9468->regmap2, PCA9468_REG_TEMP_CTRL, 
				PCA9468_BIT_NTC_PROTECTION_EN, PCA9468_BIT_NTC_PROTECTION_EN);
	} else {
		/* Wait 5ms to keep the shutdown sequence */
		mdelay(5);
	}

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/* Check Acitve status */
static int pca9468_check_error_sub(struct pca9468_charger *pca9468)
{
	int ret;
	unsigned int reg_val;
	int vbatt;
	
	/* Read STS_B */
	ret = regmap_read(pca9468->regmap2, PCA9468_REG_STS_B, &reg_val);
	if (ret < 0)
		goto error;

	/* Read VBAT_ADC */
	vbatt = pca9468_read_adc_sub(pca9468, ADCCH_VBAT);
	
	/* Check Active status */
	if (reg_val & PCA9468_BIT_ACTIVE_STATE_STS) {
		/* PCA9468 is active state */
		/* PCA9468 is in charging */
		/* Check whether the battery voltage is over the minimum voltage level or not */
		if (vbatt > PCA9468_DC_VBAT_MIN) {
			/* Normal charging battery level */
			/* Check temperature regulation loop */
			/* Read INT1_STS register */
			ret = regmap_read(pca9468->regmap2, PCA9468_REG_INT1_STS, &reg_val);
			if (reg_val & PCA9468_BIT_TEMP_REG_STS) {				
				/* Over temperature protection */
				pr_err("%s: Device is in temperature regulation", __func__);
				ret = -EINVAL;
			} else {
				/* Normal temperature */
				ret = 0;
			}
		} else {
			/* Abnormal battery level */
			pr_err("%s: Error abnormal battery voltage=%d\n", __func__, vbatt);
			ret = -EINVAL;
		}
	} else {
		/* PCA9468 is not active state  - stanby or shutdown */
		/* Stop charging in timer_work */
		
		/* Read all status register for debugging */
		u8 val[8];
		u8 test_val[16];	/* Dump for test register */
		ret = regmap_bulk_read(pca9468->regmap2, PCA9468_REG_INT1, &val[PCA9468_REG_INT1], 7);
		pr_err("%s: Error reg[1]=0x%x,[2]=0x%x,[3]=0x%x,[4]=0x%x,[5]=0x%x,[6]=0x%x,[7]=0x%x\n",
			__func__, val[1], val[2], val[3], val[4], val[5], val[6], val[7]);
		/* Read test register for debugging */
		ret = regmap_bulk_read(pca9468->regmap2, 0x40, test_val, 16);
		pr_err("%s: Error reg[0x40]=0x%x,[0x41]=0x%x,[0x42]=0x%x,[0x43]=0x%x,[0x44]=0x%x,[0x45]=0x%x,[0x46]=0x%x,[0x47]=0x%x\n",
			__func__, test_val[0], test_val[1], test_val[2], test_val[3], test_val[4], test_val[5], test_val[6], test_val[7]);
		pr_err("%s: Error reg[0x48]=0x%x,[0x49]=0x%x,[0x4A]=0x%x,[0x4B]=0x%x,[0x4C]=0x%x,[0x4D]=0x%x,[0x4E]=0x%x,[0x4F]=0x%x\n",
			__func__, test_val[8], test_val[9], test_val[10], test_val[11], test_val[12], test_val[13], test_val[14], test_val[15]);

		/* Check INT1_STS first */
		if ((val[PCA9468_REG_INT1_STS] & PCA9468_BIT_V_OK_STS) != PCA9468_BIT_V_OK_STS) {
			/* VBUS is invalid */
			pr_err("%s: VOK is invalid", __func__);
			/* Check STS_A */
			if (val[PCA9468_REG_STS_A] & PCA9468_BIT_CFLY_SHORT_STS)
				pr_err("%s: Flying Cap is shorted to GND", __func__);	/* Flying cap is short to GND */
			else if (val[PCA9468_REG_STS_A] & PCA9468_BIT_VOUT_UV_STS)
				pr_err("%s: VOUT UV", __func__);	/* VOUT < VOUT_OK */
			else if (val[PCA9468_REG_STS_A] & PCA9468_BIT_VBAT_OV_STS)
				pr_err("%s: VBAT OV", __func__);	/* VBAT > VBAT_OV */
			else if (val[PCA9468_REG_STS_A] & PCA9468_BIT_VIN_OV_STS)
				pr_err("%s: VIN OV", __func__);		/* VIN > V_OV_FIXED or V_OV_TRACKING */
			else if (val[PCA9468_REG_STS_A] & PCA9468_BIT_VIN_UV_STS)
				pr_err("%s: VIN UV", __func__);		/* VIN < V_UVTH */
			else
				pr_err("%s: Invalid VIN or VOUT", __func__);

			ret = -EINVAL;
		} else if (val[PCA9468_REG_INT1_STS] & PCA9468_BIT_NTC_TEMP_STS) {
			/* NTC protection */
			int ntc_adc, ntc_th;
			/* NTC threshold */
			u8 reg_data[2];
			regmap_bulk_read(pca9468->regmap2, PCA9468_REG_NTC_TH_1, reg_data, 2);
			ntc_th = ((reg_data[1] & PCA9468_BIT_NTC_THRESHOLD9_8)<<8) | reg_data[0];	/* uV unit */
			/* Read NTC ADC */
			ntc_adc = pca9468_read_adc_sub(pca9468, ADCCH_NTC);	/* uV unit */
			pr_err("%s: NTC Protection, NTC_TH=%d(uV), NTC_ADC=%d(uV)", __func__, ntc_th, ntc_adc);
			
			ret = -EINVAL;
		} else if (val[PCA9468_REG_INT1_STS] & PCA9468_BIT_CTRL_LIMIT_STS) {
			/* OCP event happens */
			/* Check STS_B */
			if (val[PCA9468_REG_STS_B] & PCA9468_BIT_OCP_FAST_STS)
				pr_err("%s: IIN is over OCP_FAST", __func__); 	/* OCP_FAST happened */
			else if (val[PCA9468_REG_STS_B] & PCA9468_BIT_OCP_AVG_STS)
				pr_err("%s: IIN is over OCP_AVG", __func__);	/* OCP_AVG happened */
			else
				pr_err("%s: No Loop active", __func__);

			ret = -EINVAL;
		} else if (val[PCA9468_REG_INT1_STS] & PCA9468_BIT_TEMP_REG_STS) {
			/* Over temperature protection */
			pr_err("%s: Device is in temperature regulation", __func__);
			
			ret = -EINVAL;
		} else if (val[PCA9468_REG_INT1_STS] & PCA9468_BIT_TIMER_STS) {
			/* Check STS_B */
			if (val[PCA9468_REG_STS_B] & PCA9468_BIT_CHARGE_TIMER_STS)
				pr_err("%s: Charger timer is expired", __func__);
			else if (val[PCA9468_REG_STS_B] & PCA9468_BIT_WATCHDOG_TIMER_STS)
				pr_err("%s: Watchdog timer is expired", __func__);
			else
				pr_err("%s: Timer INT, but no timer STS", __func__);

			ret = -EINVAL;
		}
		else if (val[PCA9468_REG_STS_A] & PCA9468_BIT_CFLY_SHORT_STS) {
			/* Flying cap short */
			pr_err("%s: Flying Cap is shorted to GND", __func__);	/* Flying cap is short to GND */
			ret = -EINVAL;
		} else {
			/* There is no error, but not active state */
			if (reg_val & PCA9468_BIT_STANDBY_STATE_STS) {
				/* Standby state */
				/* Check the RCP condition, T_REVI_DET is 300ms */
				/* Wait 200ms */
				msleep(200);

				/* Check the charging state */
				/* Sometimes battery driver might call set_property function to stop charging during msleep
				    At this case, charging state would change DC_STATE_NO_CHARGING.
				    PCA9468 should stop checking RCP condition and exit timer_work
				*/
				if (pca9468->charging_state == DC_STATE_NO_CHARGING) {
					pr_err("%s: other driver forced to stop direct charing\n", __func__);
					ret = -EINVAL;
				} else {
					/* Keep the current charging state */
					/* Check PCA948 state again */
					/* Read STS_B */
					ret = regmap_read(pca9468->regmap2, PCA9468_REG_STS_B, &reg_val);
					if (ret < 0)
						goto error;

					pr_info("%s:RCP check, STS_B=0x%x\n",	__func__, reg_val);

					/* Check Active status */
					if (reg_val & PCA9468_BIT_ACTIVE_STATE_STS) {
						/* RCP condition happened, but VIN is still valid */
						/* If VIN is increased, input current will increase over IIN_LOW level */
						/* Normal charging */
						pr_info("%s: RCP happened before, but VIN is valid\n", __func__);
						ret = 0;
					} else if (reg_val & PCA9468_BIT_STANDBY_STATE_STS) {
						/* It is not RCP condition */
						/* Need to retry if DC is in starting state */
						pr_err("%s: Any abnormal condition, retry\n", __func__);
						/* Dump register */
						ret = regmap_bulk_read(pca9468->regmap2, PCA9468_REG_INT1, &val[PCA9468_REG_INT1], 7);
						pr_err("%s: Error reg[1]=0x%x,[2]=0x%x,[3]=0x%x,[4]=0x%x,[5]=0x%x,[6]=0x%x,[7]=0x%x\n",
								__func__, val[1], val[2], val[3], val[4], val[5], val[6], val[7]);
						ret = regmap_bulk_read(pca9468->regmap2, 0x48, test_val, 3);
						pr_err("%s: Error reg[0x48]=0x%x,[0x49]=0x%x,[0x4a]=0x%x\n",
								__func__, test_val[0], test_val[1], test_val[2]);
						ret = -EAGAIN;
					} else {
						/* Shutdown State */
						pr_err("%s: Shutdown state\n", __func__);
						/* Dump register */
						ret = regmap_bulk_read(pca9468->regmap2, PCA9468_REG_INT1, &val[PCA9468_REG_INT1], 7);
						pr_err("%s: Error reg[1]=0x%x,[2]=0x%x,[3]=0x%x,[4]=0x%x,[5]=0x%x,[6]=0x%x,[7]=0x%x\n",
								__func__, val[1], val[2], val[3], val[4], val[5], val[6], val[7]);
						ret = regmap_bulk_read(pca9468->regmap2, 0x48, test_val, 3);
						pr_err("%s: Error reg[0x48]=0x%x,[0x49]=0x%x,[0x4a]=0x%x\n",
								__func__, test_val[0], test_val[1], test_val[2]);
						ret = -EINVAL;
					}
				}
			} else {
				/* PCA9468 is in shutdown state */
				pr_err("%s: PCA9468 is in shutdown state\n", __func__);
				ret = -EINVAL;
			}
		}
	}

error:
	pr_info("%s: Active Status=%d\n", __func__, ret);
	return ret;
}


/* Check CC Mode status */
static int pca9468_check_ccmode_status_sub(struct pca9468_charger *pca9468)
{
	unsigned int reg_val;
	int ret;
	
	/* Read STS_A */
	ret = regmap_read(pca9468->regmap2, PCA9468_REG_STS_A, &reg_val);
	if (ret < 0)
		goto error;

	/* Check STS_A */
	if (reg_val & PCA9468_BIT_VIN_UV_STS) {
		ret = CCMODE_VIN_UVLO;
	} else if (reg_val & PCA9468_BIT_CHG_LOOP_STS) {
		ret = CCMODE_CHG_LOOP;
	} else if (reg_val & PCA9468_BIT_VFLT_LOOP_STS) {
		ret = CCMODE_VFLT_LOOP;
	} else if (reg_val & PCA9468_BIT_IIN_LOOP_STS) {
		ret = CCMODE_IIN_LOOP;
	} else {
		ret = CCMODE_LOOP_INACTIVE;
	}

error:
	pr_info("%s: CCMODE Status=%d\n", __func__, ret);
	return ret;
}


/* Check CVMode Status */
static int pca9468_check_cvmode_status_sub(struct pca9468_charger *pca9468)
{
	unsigned int val;
	int ret;

	/* Read STS_A */
	ret = regmap_read(pca9468->regmap2, PCA9468_REG_STS_A, &val);
	if (ret < 0)
		goto error;

	/* Check STS_A */
	if (val & PCA9468_BIT_CHG_LOOP_STS) {
		ret = CVMODE_CHG_LOOP;
	} else if (val & PCA9468_BIT_VFLT_LOOP_STS) {
		ret = CVMODE_VFLT_LOOP;
	} else if (val & PCA9468_BIT_IIN_LOOP_STS) {
		ret = CVMODE_IIN_LOOP;
	} else if (val & PCA9468_BIT_VIN_UV_STS) {
		ret = CVMODE_VIN_UVLO;
	} else {
		/* Any LOOP is inactive */
		ret = CVMODE_LOOP_INACTIVE;
	}

error:
	pr_info("%s: CVMODE Status=%d\n", __func__, ret);
	return ret;
}


/* Stop Charging */
static int pca9468_stop_charging(struct pca9468_charger *pca9468)
{
	int ret = 0;

	/* Check the current state */
	if (pca9468->charging_state != DC_STATE_NO_CHARGING) {
		/* Recover switching charger ICL */		
		ret = pca9468_set_switching_charger(true, SWCHG_ICL_NORMAL, 
											pca9468->pdata->ichg_cfg, pca9468->pdata->v_float);
		if (ret < 0) {
			pr_err("%s: Error-set_switching charger ICL\n", __func__);
			goto error;
		}

		// Stop Direct charging 
		cancel_delayed_work(&pca9468->timer_work);
		cancel_delayed_work(&pca9468->pps_work);
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_ID_NONE;
		pca9468->timer_period = 0;
		mutex_unlock(&pca9468->lock);
		__pm_relax(&pca9468->monitor_wake_lock);

		/* Clear parameter */
		pca9468->charging_state = DC_STATE_NO_CHARGING;
		pca9468->ret_state = DC_STATE_NO_CHARGING;
		pca9468->ta_target_vol = PCA9468_TA_MAX_VOL;
		pca9468->prev_iin = 0;
		pca9468->prev_inc = INC_NONE;
		pca9468->req_new_iin = false;
		pca9468->req_new_vfloat = false;
		pca9468->chg_mode = TA_NO_DC_MODE;

		/* Set IIN_CFG and VFLOAT to the default value */
		pca9468->pdata->iin_cfg = PCA9468_IIN_CFG_DFT;
		pca9468->pdata->v_float = PCA9468_VFLOAT_DFT;

		/* Clear new Vfloat and new IIN */
		pca9468->new_vfloat = pca9468->pdata->v_float;
		pca9468->new_iin = pca9468->pdata->iin_cfg;

		/* Clear retry counter */
		pca9468->retry_cnt = 0;

		/* Check the charging type */
		if (pca9468->parallel_chg == true) {
			/* parallel charging - use main and sub charger */
			/* Stop main charger */
			ret = pca9468_set_charging(pca9468, false);
			if (ret < 0) {
				pr_err("%s: Error-set_charging(main)\n", __func__);
				goto error;
			}
			/* Stop sub charger */
			ret = pca9468_set_charging_sub(pca9468, false);				
			if (ret < 0) {
				pr_err("%s: Error-set_charging(main)\n", __func__);
				goto error;
			}
		} else {
			/* single charging - use only main charger */
			ret = pca9468_set_charging(pca9468, false);
			if (ret < 0) {
				pr_err("%s: Error-set_charging(main)\n", __func__);
				goto error;
			}
		}

		/* Recover TA voltage */
		/* Set TA voltage to fixed 5V */
		pca9468->ta_vol = 5000000;
		/* Set TA current to maximum 3A */
		pca9468->ta_cur = 3000000;
		
		/* Send PD Message */
		pca9468->ta_objpos = 1; // PDO1 - fixed 5V
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_FIXED_PDO);
		if (ret < 0) {
			pr_err("%s: Error-send_pd_message\n", __func__);
		}
		power_supply_changed(pca9468->mains);
	}
	

error:
	pr_info("%s: END, ret=%d\n", __func__, ret);

	return ret;
}


/* Compensate TA current for the target input current */
static int pca9468_set_ta_current_comp(struct pca9468_charger *pca9468)
{
	int iin;
	int iin_main, iin_sub;

	/* Check the charging type */
	if (pca9468->parallel_chg == true) {		
		/* use main and sub charger */
		/* Read IIN ADC */
		iin_main = pca9468_read_adc(pca9468, ADCCH_IIN);
		iin_sub = pca9468_read_adc_sub(pca9468, ADCCH_IIN);
		iin = iin_main + iin_sub;
	} else {
		/* only use main charger */
		/* Read IIN ADC */
		iin = pca9468_read_adc(pca9468, ADCCH_IIN);
	}
	pr_info("%s: iin=%d\n", __func__, iin);

	/* Compare IIN ADC with target input current */
	if (iin > (pca9468->iin_cc + PCA9468_IIN_CC_COMP_OFFSET)) {
		/* TA current is higher than the target input current */
		/* Compare TA current with IIN_CC */
		if (pca9468->ta_cur > pca9468->iin_cc) {
			/* TA current is over than IIN_CC */
			/* Decrease TA current (50mA) */
			pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;
			pr_info("%s: Comp. Cont1: ta_cur=%d\n", __func__, pca9468->ta_cur);

		} else {
			/* TA current is already less than IIN_CC */
			/* Compara IIN_ADC with the previous IIN_ADC */
			if (iin < (pca9468->prev_iin - PCA9468_IIN_ADC_OFFSET)) {
				/* Assume that TA operation mode is CV mode, so decrease TA voltage */
				/* Decrease TA voltage (20mA) */
				pca9468->ta_vol = pca9468->ta_vol - PD_MSG_TA_VOL_STEP;
				pr_info("%s: Comp. Cont2-1: ta_vol=%d\n", __func__, pca9468->ta_vol);				
			} else {
				/* Assume TA operation mode is CL mode, so decrease TA current */
				/* Decrease TA current (50mA) */
				pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;
				pr_info("%s: Comp. Cont2-2: ta_cur=%d\n", __func__, pca9468->ta_cur);
			}
		}	
		/* Send PD Message */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
		mutex_unlock(&pca9468->lock);
	} else {
		if (iin < (pca9468->iin_cc - PCA9468_IIN_CC_COMP_OFFSET)) {
			/* TA current is lower than the target input current */
			/* compare IIN ADC with previous IIN ADC + 20mA */
			if (iin > (pca9468->prev_iin + PCA9468_IIN_ADC_OFFSET)) {
				/* In this case, TA voltage is not enough to supply the operating current of RDO.
				    So, increase TA voltage */
				/* Compare TA max voltage */
				if (pca9468->ta_vol == pca9468->ta_max_vol) {
					/* TA voltage is already the maximum voltage */
					/* Compare TA max current */
					if (pca9468->ta_cur == pca9468->ta_max_cur) {
						/* Both of TA voltage and current are the maximum value */
						/* Update TA target voltage to TA voltage */
						pca9468->ta_target_vol = pca9468->ta_vol;
						pr_info("%s: Comp. End1: ta_target_vol=%d\n", __func__, pca9468->ta_target_vol);
						/* Set timer */
						mutex_lock(&pca9468->lock);
						pca9468->timer_id = TIMER_CHECK_CCMODE;
						pca9468->timer_period = PCA9468_CCMODE_CHECK1_T;
						mutex_unlock(&pca9468->lock);
					} else {					
						/* Increase TA current (50mA) */
						pca9468->ta_cur = pca9468->ta_cur + PD_MSG_TA_CUR_STEP;
						pr_info("%s: Comp. Cont3: ta_cur=%d\n", __func__, pca9468->ta_cur);
						/* Send PD Message */
						mutex_lock(&pca9468->lock);
						pca9468->timer_id = TIMER_PDMSG_SEND;
						pca9468->timer_period = 0;
						mutex_unlock(&pca9468->lock);
						
						/* Set TA increment flag */
						pca9468->prev_inc = INC_TA_CUR;
					}
				} else {
					/* Increase TA voltage (20mV) */
					pca9468->ta_vol = pca9468->ta_vol + PD_MSG_TA_VOL_STEP;
					pr_info("%s: Comp. Cont4: ta_vol=%d\n", __func__, pca9468->ta_vol);
					/* Send PD Message */
					mutex_lock(&pca9468->lock);
					pca9468->timer_id = TIMER_PDMSG_SEND;
					pca9468->timer_period = 0;
					mutex_unlock(&pca9468->lock);
					
					/* Set TA increment flag */
					pca9468->prev_inc = INC_TA_VOL;
				}
			} else {			
				/* TA current is lower than the target input current */
				/* Check the previous TA increment */
				if (pca9468->prev_inc == INC_TA_VOL) {
					/* The previous increment is TA voltage, but input current does not increase */
					/* Try to increase TA current */
					/* Compare TA max current */
					if (pca9468->ta_cur == pca9468->ta_max_cur) {
						/* TA current is already the maximum current */
						/* Compare TA max voltage */
						if (pca9468->ta_vol == pca9468->ta_max_vol) {
							/* TA voltage and current are already the maximum values */							
							/* Update TA target voltage to TA voltage */
							pca9468->ta_target_vol = pca9468->ta_vol;
							pr_info("%s: Comp. End2: ta_target_vol=%d\n", __func__, pca9468->ta_target_vol);
							/* Set timer */
							mutex_lock(&pca9468->lock);
							pca9468->timer_id = TIMER_CHECK_CCMODE;
							pca9468->timer_period = PCA9468_CCMODE_CHECK1_T;
							mutex_unlock(&pca9468->lock);
						} else {
							/* Increase TA voltage (20mV) */
							pca9468->ta_vol = pca9468->ta_vol + PD_MSG_TA_VOL_STEP;
							pr_info("%s: Comp. Cont5: ta_vol=%d\n", __func__, pca9468->ta_vol);

							/* Send PD Message */
							mutex_lock(&pca9468->lock);
							pca9468->timer_id = TIMER_PDMSG_SEND;
							pca9468->timer_period = 0;
							mutex_unlock(&pca9468->lock);
							
							/* Set TA increment flag */
							pca9468->prev_inc = INC_TA_VOL;
						}						
					} else {
						/* Increase TA current (50mA) */
						pca9468->ta_cur = pca9468->ta_cur + PD_MSG_TA_CUR_STEP;
						pr_info("%s: Comp. Cont6: ta_cur=%d\n", __func__, pca9468->ta_cur);
						/* Send PD Message */
						mutex_lock(&pca9468->lock);
						pca9468->timer_id = TIMER_PDMSG_SEND;
						pca9468->timer_period = 0;
						mutex_unlock(&pca9468->lock);
						
						/* Set TA increment flag */
						pca9468->prev_inc = INC_TA_CUR;
					}
				} else {				
					/* The previous increment is TA current, but input current does not increase */
					/* Try to increase TA voltage */
					/* Compare TA max voltage */
					if (pca9468->ta_vol == pca9468->ta_max_vol) {
						/* TA voltage is already the maximum voltage */
						/* Compare TA maximum current */
						if (pca9468->ta_cur == pca9468->ta_max_cur) {
							/* TA voltage and current are already the maximum values */
							/* Update TA target voltage to TA voltage */
							pca9468->ta_target_vol = pca9468->ta_vol;
							pr_info("%s: Comp. End3: ta_target_vol=%d\n", __func__, pca9468->ta_target_vol);
							/* Set timer */
							mutex_lock(&pca9468->lock);
							pca9468->timer_id = TIMER_CHECK_CCMODE;
							pca9468->timer_period = PCA9468_CCMODE_CHECK1_T;
							mutex_unlock(&pca9468->lock);
						} else {
							/* Increase TA current (50mA) */
							pca9468->ta_cur = pca9468->ta_cur + PD_MSG_TA_CUR_STEP;
							pr_info("%s: Comp. Cont7: ta_cur=%d\n", __func__, pca9468->ta_cur);
							/* Send PD Message */
							mutex_lock(&pca9468->lock);
							pca9468->timer_id = TIMER_PDMSG_SEND;
							pca9468->timer_period = 0;
							mutex_unlock(&pca9468->lock);
							
							/* Set TA increment flag */
							pca9468->prev_inc = INC_TA_CUR;
						}
					} else {
						/* Increase TA voltage (20mV) */
						pca9468->ta_vol = pca9468->ta_vol + PD_MSG_TA_VOL_STEP;
						pr_info("%s: Comp. Cont8: ta_vol=%d\n", __func__, pca9468->ta_vol);

						/* Send PD Message */
						mutex_lock(&pca9468->lock);
						pca9468->timer_id = TIMER_PDMSG_SEND;
						pca9468->timer_period = 0;
						mutex_unlock(&pca9468->lock);

						/* Set TA increment flag */
						pca9468->prev_inc = INC_TA_VOL;
					}
				}
			}
		} else {
			/* IIN ADC is in valid range */
			/* IIN_CC - 50mA < IIN ADC < IIN_CC + 50mA  */			
			/* Update TA target voltage to TA voltage */
			pca9468->ta_target_vol = pca9468->ta_vol;
			pr_info("%s: Comp. End4(valid): ta_target_vol=%d\n", __func__, pca9468->ta_target_vol);
			/* Set timer */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_CHECK_CCMODE;
			pca9468->timer_period = PCA9468_CCMODE_CHECK1_T;
			mutex_unlock(&pca9468->lock);
		}
	}

	/* Save previous iin adc */
	pca9468->prev_iin = iin;
	
	queue_delayed_work(pca9468->dc_wq,
						&pca9468->timer_work,
						msecs_to_jiffies(pca9468->timer_period));

	return 0;
}

/* Compensate TA current for constant power mode */
static int pca9468_set_ta_current_comp2(struct pca9468_charger *pca9468)
{
	int iin;
	int iin_main, iin_sub;

	unsigned int val;
	unsigned int iin_apdo;

	/* Check the charging type */
	if (pca9468->parallel_chg == true) {
		/* use main and sub charger */
		/* Read IIN ADC */
		iin_main = pca9468_read_adc(pca9468, ADCCH_IIN);
		iin_sub = pca9468_read_adc_sub(pca9468, ADCCH_IIN);
		iin = iin_main + iin_sub;
	} else {
		/* use only main charger */
		/* Read IIN ADC */
		iin = pca9468_read_adc(pca9468, ADCCH_IIN);
	}
	pr_info("%s: iin=%d\n", __func__, iin);

	/* Compare IIN ADC with target input current */
	if (iin > (pca9468->pdata->iin_cfg + PCA9468_IIN_CC_UPPER_OFFSET)) {
		/* TA current is higher than the target input current limit */
		/* Decrease TA current (50mA) */
		pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;

		/* Send PD Message */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
		mutex_unlock(&pca9468->lock);
	} else if (iin < (pca9468->iin_cc - PCA9468_IIN_CC_COMP_OFFSET_CP)) {
		/* TA current is lower than the target input current */
		/* IIN_ADC < IIN_CC -20mA */
		if (pca9468->ta_vol == pca9468->ta_max_vol) {
			/* Check IIN_ADC < IIN_CC - 50mA */
			if (iin < (pca9468->iin_cc - PCA9468_IIN_CC_COMP_OFFSET)) {
				/* Set new IIN_CC to IIN_CC - 50mA */
				pca9468->iin_cc = pca9468->iin_cc - PCA9468_IIN_CC_COMP_OFFSET;
				/* Set new TA_MAX_VOL to TA_MAX_PWR/IIN_CC */
				/* Adjust new IIN_CC with APDO resolution */
				iin_apdo = pca9468->iin_cc/PD_MSG_TA_CUR_STEP;
				iin_apdo = iin_apdo*PD_MSG_TA_CUR_STEP;
				val = pca9468->ta_max_pwr/(iin_apdo/pca9468->chg_mode/1000);	/* mV */
				val = val*1000/PD_MSG_TA_VOL_STEP;	/* Adjust values with APDO resolution(20mV) */
				val = val*PD_MSG_TA_VOL_STEP; /* uV */
				/* Set new TA_MAX_VOL */
				pca9468->ta_max_vol = MIN(val, PCA9468_TA_MAX_VOL*pca9468->chg_mode);
				/* Increase TA voltage(40mV) */
				pca9468->ta_vol = pca9468->ta_vol + PD_MSG_TA_VOL_STEP*2;
				
				pr_info("%s: Comp. Cont1: ta_vol=%d\n", __func__, pca9468->ta_vol);
				/* Send PD Message */
				mutex_lock(&pca9468->lock);
				pca9468->timer_id = TIMER_PDMSG_SEND;
				pca9468->timer_period = 0;
				mutex_unlock(&pca9468->lock);
			} else {
				/* Wait for next current step compensation */
				/* IIN_CC - 50mA < IIN ADC < IIN_CC - 20mA*/				
				/* Update TA target voltage to TA voltage */
				pca9468->ta_target_vol = pca9468->ta_vol;
				pr_info("%s: Comp.(wait): Update ta_target_vol=%d\n", __func__, pca9468->ta_target_vol);
				/* Set timer */
				mutex_lock(&pca9468->lock);
				pca9468->timer_id = TIMER_CHECK_CCMODE;
				pca9468->timer_period = PCA9468_CCMODE_CHECK2_T;
				mutex_unlock(&pca9468->lock);			
			}
		} else {
			/* Increase TA voltage(40mV) */
			pca9468->ta_vol = pca9468->ta_vol + PD_MSG_TA_VOL_STEP*2;
			if (pca9468->ta_vol > pca9468->ta_max_vol)
				pca9468->ta_vol = pca9468->ta_max_vol;
			
			pr_info("%s: Comp. Cont2: ta_vol=%d\n", __func__, pca9468->ta_vol);
			/* Send PD Message */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_PDMSG_SEND;
			pca9468->timer_period = 0;
			mutex_unlock(&pca9468->lock);
		}
	} else {
		/* IIN ADC is in valid range */
		/* IIN_CC - 50mA < IIN ADC < IIN_CFG + 50mA */		
		/* Update TA target voltage to TA voltage */
		pca9468->ta_target_vol = pca9468->ta_vol;
		pr_info("%s: Comp.(valid): Update ta_target_vol=%d\n", __func__, pca9468->ta_target_vol);
		/* Set timer */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_CHECK_CCMODE;
		pca9468->timer_period = PCA9468_CCMODE_CHECK2_T;
		mutex_unlock(&pca9468->lock);
	}

	/* Save previous iin adc */
	pca9468->prev_iin = iin;
	
	queue_delayed_work(pca9468->dc_wq,
						&pca9468->timer_work,
						msecs_to_jiffies(pca9468->timer_period));	

	return 0;
}


/* Compensate TA voltage for the target input current */
static int pca9468_set_ta_voltage_comp(struct pca9468_charger *pca9468)
{
	int iin;
	int iin_main, iin_sub;

	pr_info("%s: ======START=======\n", __func__);

	if (pca9468->parallel_chg == true) {
		/* use main and sub charger */
		/* Read IIN ADC */
		iin_main = pca9468_read_adc(pca9468, ADCCH_IIN);
		iin_sub = pca9468_read_adc_sub(pca9468, ADCCH_IIN);
		iin = iin_main + iin_sub;
	} else {
		/* use only main charger */
		/* Read IIN ADC */
		iin = pca9468_read_adc(pca9468, ADCCH_IIN);
	}
	pr_info("%s: iin=%d\n", __func__, iin);

	/* Compare IIN ADC with target input current */
	if (iin > (pca9468->iin_cc + PCA9468_IIN_CC_COMP_OFFSET)) {
		/* TA current is higher than the target input current */
		/* Decrease TA voltage (20mV) */
		pca9468->ta_vol = pca9468->ta_vol - PD_MSG_TA_VOL_STEP;
		pr_info("%s: Decrease: ta_vol=%d\n", __func__, pca9468->ta_vol);

		/* Send PD Message */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
		mutex_unlock(&pca9468->lock);
	} else {
		if (iin < (pca9468->iin_cc - PCA9468_IIN_CC_COMP_OFFSET)) {
			/* TA current is lower than the target input current */
			/* Compare TA max voltage */
			if (pca9468->ta_vol == pca9468->ta_max_vol) {
				/* TA current is already the maximum voltage */				
				/* Update TA target voltage to TA voltage */
				pca9468->ta_target_vol = pca9468->ta_vol;
				pr_info("%s: Comp.(max TA vol): Update ta_target_vol=%d\n", __func__, pca9468->ta_target_vol);
				/* Set timer */
				/* Check the current charging state */
				if (pca9468->charging_state == DC_STATE_CC_MODE) {
					/* CC mode */
					mutex_lock(&pca9468->lock);
					pca9468->timer_id = TIMER_CHECK_CCMODE;
					pca9468->timer_period = PCA9468_CCMODE_CHECK1_T;
					mutex_unlock(&pca9468->lock);
				} else {
					/* CV mode */
					mutex_lock(&pca9468->lock);
					pca9468->timer_id = TIMER_CHECK_CVMODE;
					pca9468->timer_period = PCA9468_CVMODE_CHECK_T;
					mutex_unlock(&pca9468->lock);
				}
			} else {
				/* Increase TA voltage (20mV) */
				pca9468->ta_vol = pca9468->ta_vol + PD_MSG_TA_VOL_STEP;
				pr_info("%s: Increase: ta_vol=%d\n", __func__, pca9468->ta_vol);

				/* Send PD Message */
				mutex_lock(&pca9468->lock);
				pca9468->timer_id = TIMER_PDMSG_SEND;
				pca9468->timer_period = 0;
				mutex_unlock(&pca9468->lock);
			}
		} else {
			/* IIN ADC is in valid range */
			/* IIN_CC - 50mA < IIN ADC < IIN_CC + 50mA  */
			/* Update TA target voltage to TA voltage */
			pca9468->ta_target_vol = pca9468->ta_vol;
			pr_info("%s: Comp.(valid): Update ta_target_vol=%d\n", __func__, pca9468->ta_target_vol);
			/* Set timer */
			/* Check the current charging state */
			if (pca9468->charging_state == DC_STATE_CC_MODE) {
				/* CC mode */
				mutex_lock(&pca9468->lock);
				pca9468->timer_id = TIMER_CHECK_CCMODE;
				pca9468->timer_period = PCA9468_CCMODE_CHECK1_T;
				mutex_unlock(&pca9468->lock);
			} else {
				/* CV mode */
				mutex_lock(&pca9468->lock);
				pca9468->timer_id = TIMER_CHECK_CVMODE;
				pca9468->timer_period = PCA9468_CVMODE_CHECK_T;
				mutex_unlock(&pca9468->lock);
			}
		}
	}

	queue_delayed_work(pca9468->dc_wq,
						&pca9468->timer_work,
						msecs_to_jiffies(pca9468->timer_period));

	return 0;
}


/* Set TA current for target current */
static int pca9468_adjust_ta_current (struct pca9468_charger *pca9468)
{
	int ret = 0;
	int vbat;
	unsigned int val;

	/* Set charging state to ADJUST_TACUR */
	pca9468->charging_state = DC_STATE_ADJUST_TACUR;

	if (pca9468->ta_cur == pca9468->iin_cc/pca9468->chg_mode) {
		/* finish sending PD message */
		/* Recover IIN_CC to the original value(new_iin) */
		pca9468->iin_cc = pca9468->new_iin;
		
		/* Clear req_new_iin */
		mutex_lock(&pca9468->lock);
		pca9468->req_new_iin = false;
		mutex_unlock(&pca9468->lock);

		/* Compare new iin_cc with parallel threshold */
		if (pca9468->iin_cc > pca9468->pdata->parallel_th) {
			/* new input current is higher than the parallel charging threshold */
			if (pca9468->parallel_chg == false) {
				/* Update iin_cfg and turn on sub charger */
				/* Set IIN_CFG_S to iin_cfg/2 */
				ret = pca9468_set_input_current_sub(pca9468, pca9468->pdata->iin_cfg/2);
				if (ret < 0)
					goto error;
				/* enable sub charger to use parallel charging */
				ret = pca9468_set_charging_sub(pca9468, true);
				if (ret < 0)
					goto error;
				/* Set IIN_CFG_M to iin_cfg/2 */
				ret = pca9468_set_input_current(pca9468, pca9468->pdata->iin_cfg/2);
				if (ret < 0)
					goto error;
				pca9468->parallel_chg = true;
				pr_info("%s: Use parallel charging, new_iin_cc=%d\n", __func__, pca9468->iin_cc);
			} else {
				/* Already use parallel charging */
				pr_info("%s: Already use parallel charging\n", __func__);
			}
		} else {
			/* new input current is less than the parallel charging threshold */
			if (pca9468->parallel_chg == true) {
				/* Update iin_cfg and turn off sub charger */
				if (pca9468->pdata->iin_cfg/2 < pca9468->iin_cc) {
					/* Update IIN_CFG_M */
					/* Set IIN_CFG_M to iin_cfg */
					ret = pca9468_set_input_current(pca9468, pca9468->pdata->iin_cfg);
					if (ret < 0)
						goto error;
				}
				/* disable sub charger to use single charging */
				ret = pca9468_set_charging_sub(pca9468, false);
				if (ret < 0)
					goto error;
				pca9468->parallel_chg = false;
				pr_info("%s: Use single charging, new_iin_cc=%d\n", __func__, pca9468->iin_cc);
			} else {
				/* Already use single charging */
				pr_info("%s: Already use single charging\n", __func__);
			}
		}

		/* Go to return state  */
		pca9468->charging_state = pca9468->ret_state;
		/* Set timer */
		mutex_lock(&pca9468->lock);
		if (pca9468->charging_state == DC_STATE_CC_MODE)
			pca9468->timer_id = TIMER_CHECK_CCMODE;
		else
			pca9468->timer_id = TIMER_CHECK_CVMODE;
		pca9468->timer_period = 1000;	/* Wait 1000ms */
		mutex_unlock(&pca9468->lock);
	} else {
		/* Compare new IIN with IIN_CFG */
		if (pca9468->iin_cc > pca9468->pdata->iin_cfg) {
			/* New iin is higher than the current iin_cfg */
			/* Update iin_cfg */
			pca9468->pdata->iin_cfg = pca9468->iin_cc;
			if (pca9468->iin_cc > pca9468->pdata->parallel_th) {
				/* new input current is higher than the parallel charging threshold */
				/* Update iin_cfg */
				/* Set IIN_CFG_M/S to New_IIN/2 */
				ret = pca9468_set_input_current(pca9468, pca9468->iin_cc/2);
				if (ret < 0)
					goto error;
				ret = pca9468_set_input_current_sub(pca9468, pca9468->iin_cc/2);
				if (ret < 0)
					goto error;
				if (pca9468->parallel_chg == false) {
					/* enable sub charger to use parallel charging */				
					ret = pca9468_set_charging_sub(pca9468, true);
					if (ret < 0)
						goto error;
					pca9468->parallel_chg = true;
					pr_info("%s: Use parallel charging, new_iin_cc=%d\n", __func__, pca9468->iin_cc);
				} else {
					/* Already use parallel charging */
					pr_info("%s: Already use parallel charging, just change iin\n", __func__);					
				}
			} else {
				/* new input current is less than the parallel charging threshold */
				/* Set IIN_CFG to new IIN */
				ret = pca9468_set_input_current(pca9468, pca9468->iin_cc);
				if (ret < 0)
					goto error;
				if (pca9468->parallel_chg == true) {
					/* disable sub charger to use single charging */					
					ret = pca9468_set_charging_sub(pca9468, false);
					if (ret < 0)
						goto error;
					pca9468->parallel_chg = false;
					pr_info("%s: Use single charging, new_iin_cc=%d\n", __func__, pca9468->iin_cc);
				} else {
					/* Already use single charging */
					pr_info("%s: Already use single charging, just change iin\n", __func__);	
				}
			}				

			/* Clear Request flag */
			mutex_lock(&pca9468->lock);
			pca9468->req_new_iin = false;
			mutex_unlock(&pca9468->lock);

			/* Set new TA voltage and current */
			/* Read VBAT ADC */
			vbat = pca9468_read_adc(pca9468, ADCCH_VBAT);

			/* Calculate new TA maximum current and voltage that used in the direct charging */
			/* Set IIN_CC to MIN[IIN, TA_MAX_CUR*chg_mode]*/
			pca9468->iin_cc = MIN(pca9468->pdata->iin_cfg, pca9468->ta_max_cur*pca9468->chg_mode);
			/* Set the current IIN_CC to iin_cfg for recovering it after resolution adjustment */
			pca9468->pdata->iin_cfg = pca9468->iin_cc;
			/* Calculate new TA max voltage */
			/* Adjust IIN_CC with APDO resoultion(50mA) - It will recover to the original value after max voltage calculation */
			val = pca9468->iin_cc/(PD_MSG_TA_CUR_STEP*pca9468->chg_mode);
			pca9468->iin_cc = val*(PD_MSG_TA_CUR_STEP*pca9468->chg_mode);
			/* Set TA_MAX_VOL to MIN[PCA9468_TA_MAX_VOL, (TA_MAX_PWR/IIN_CC)] */
			val = pca9468->ta_max_pwr/(pca9468->iin_cc/pca9468->chg_mode/1000);	/* mV */
			val = val*1000/PD_MSG_TA_VOL_STEP;	/* Adjust values with APDO resolution(20mV) */
			val = val*PD_MSG_TA_VOL_STEP; /* uV */
			pca9468->ta_max_vol = MIN(val, PCA9468_TA_MAX_VOL*pca9468->chg_mode);
			
			/* Set TA voltage to MAX[8000mV, (2*VBAT_ADC + 500 mV)] */
			pca9468->ta_vol = max(PCA9468_TA_MIN_VOL_PRESET*pca9468->chg_mode, (2*vbat*pca9468->chg_mode + PCA9468_TA_VOL_PRE_OFFSET));
			val = pca9468->ta_vol/PD_MSG_TA_VOL_STEP;	/* PPS voltage resolution is 20mV */
			pca9468->ta_vol = val*PD_MSG_TA_VOL_STEP;
			/* Set TA voltage to MIN[TA voltage, TA_MAX_VOL] */
			pca9468->ta_vol = MIN(pca9468->ta_vol, pca9468->ta_max_vol);
			/* Set TA current to IIN_CC */
			pca9468->ta_cur = pca9468->iin_cc/pca9468->chg_mode;
			/* Recover IIN_CC to the original value(iin_cfg) */
			pca9468->iin_cc = pca9468->pdata->iin_cfg;

			pr_info("%s: New IIN, ta_max_vol=%d, ta_max_cur=%d, ta_max_pwr=%d, iin_cc=%d, chg_mode=%d\n", 
				__func__, pca9468->ta_max_vol, pca9468->ta_max_cur, pca9468->ta_max_pwr, pca9468->iin_cc, pca9468->chg_mode);

			/* Clear previous IIN ADC */
			pca9468->prev_iin = 0;
			/* Clear TA increment flag */
			pca9468->prev_inc = INC_NONE;

			/* Send PD Message and go to Adjust CC mode */
			pca9468->charging_state = DC_STATE_ADJUST_CC;
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_PDMSG_SEND;
			pca9468->timer_period = 0;
			mutex_unlock(&pca9468->lock);
		} else {
			/* Set TA voltage to TA target voltage */
			pca9468->ta_vol = pca9468->ta_target_vol;
			/* Adjust IIN_CC with APDO resoultion(50mA) - It will recover to the original value after sending PD message */
			val = pca9468->iin_cc/PD_MSG_TA_CUR_STEP;
			pca9468->iin_cc = val*PD_MSG_TA_CUR_STEP;
			/* Set TA current to IIN_CC */
			pca9468->ta_cur = pca9468->iin_cc/pca9468->chg_mode;
			/* Send PD Message */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_PDMSG_SEND;
			pca9468->timer_period = 0;
			mutex_unlock(&pca9468->lock);
		}
	}
	queue_delayed_work(pca9468->dc_wq,
					&pca9468->timer_work,
					msecs_to_jiffies(pca9468->timer_period));

error:
	return ret;
}


/* Set TA voltage for target current */
static int pca9468_adjust_ta_voltage(struct pca9468_charger *pca9468)
{
	int iin;
	int iin_main, iin_sub;

	pca9468->charging_state = DC_STATE_ADJUST_TAVOL;

	/* Check the charging type */
	if (pca9468->parallel_chg == true) {		
		/* Read IIN ADC */
		iin_main = pca9468_read_adc(pca9468, ADCCH_IIN);
		iin_sub = pca9468_read_adc_sub(pca9468, ADCCH_IIN);
		iin = iin_main + iin_sub;
	} else {	
		/* Read IIN ADC */
		iin = pca9468_read_adc(pca9468, ADCCH_IIN);
	}

	/* Compare IIN ADC with targer input current */
	if (iin > (pca9468->iin_cc + PD_MSG_TA_CUR_STEP)) {
		/* TA current is higher than the target input current */
		/* Decrease TA voltage (20mV) */
		pca9468->ta_vol = pca9468->ta_vol - PD_MSG_TA_VOL_STEP;
		
		/* Send PD Message */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
		mutex_unlock(&pca9468->lock);
	} else {
		if (iin < (pca9468->iin_cc - PD_MSG_TA_CUR_STEP)) {
			/* TA current is lower than the target input current */
			/* Compare TA max voltage */
			if (pca9468->ta_vol == pca9468->ta_max_vol) {
				/* TA current is already the maximum voltage */
				/* Clear req_new_iin */
				mutex_lock(&pca9468->lock);
				pca9468->req_new_iin = false;
				mutex_unlock(&pca9468->lock);
				/* Return charging state to the previous state */
				pca9468->charging_state = pca9468->ret_state;

				/* Set TA current and voltage to the same value */
				/* Send PD Message */
				mutex_lock(&pca9468->lock);
				pca9468->timer_id = TIMER_PDMSG_SEND;
				pca9468->timer_period = 0;
				mutex_unlock(&pca9468->lock);
			} else {
				/* Increase TA voltage (20mV) */
				pca9468->ta_vol = pca9468->ta_vol + PD_MSG_TA_VOL_STEP;
				
				/* Send PD Message */
				mutex_lock(&pca9468->lock);
				pca9468->timer_id = TIMER_PDMSG_SEND;
				pca9468->timer_period = 0;
				mutex_unlock(&pca9468->lock);
			}
		} else {
			/* IIN ADC is in valid range */
			/* Clear req_new_iin */
			mutex_lock(&pca9468->lock);
			pca9468->req_new_iin = false;
			mutex_unlock(&pca9468->lock);

			/* IIN_CC - 50mA < IIN ADC < IIN_CC + 50mA  */
			/* Return charging state to the previous state */
			pca9468->charging_state = pca9468->ret_state;
			/* Set TA current and voltage to the same value */
			/* Send PD Message */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_PDMSG_SEND;
			pca9468->timer_period = 0;
			mutex_unlock(&pca9468->lock);
		}
	}
	queue_delayed_work(pca9468->dc_wq,
					&pca9468->timer_work,
					msecs_to_jiffies(pca9468->timer_period));

	return 0;
}


/* Set new input current  */
static int pca9468_set_new_iin(struct pca9468_charger *pca9468)
{
	int ret = 0;

	pr_info("%s: new_iin=%d\n", __func__, pca9468->new_iin);

	/* Check the charging state */
	if ((pca9468->charging_state == DC_STATE_NO_CHARGING) ||
		(pca9468->charging_state == DC_STATE_CHECK_VBAT)) {
		/* Apply new iin when the direct charging is started */
		pca9468->pdata->iin_cfg = pca9468->new_iin;
	} else {
		/* Check whether the previous request is done or not */
		if (pca9468->req_new_iin == true) {
			/* The previous request is not done yet */
			pr_err("%s: There is the previous request for New iin\n", __func__);
			ret = -EBUSY;
		} else {
			/* Set request flag */
			mutex_lock(&pca9468->lock);
			pca9468->req_new_iin = true;
			mutex_unlock(&pca9468->lock);

			/* Check the charging state */
			if ((pca9468->charging_state == DC_STATE_CC_MODE) ||
				(pca9468->charging_state == DC_STATE_CV_MODE) ||
				(pca9468->charging_state == DC_STATE_CHARGING_DONE)) {
				/* cancel delayed_work */
				cancel_delayed_work(&pca9468->timer_work);

				/* Set new IIN to IIN_CC */
				pca9468->iin_cc = pca9468->new_iin;
				/* Save return state */
				pca9468->ret_state = pca9468->charging_state;

				/* Check new IIN with the minimum TA current */
				if (pca9468->iin_cc < (PCA9468_TA_MIN_CUR * pca9468->chg_mode)) {
					/* Set the TA current to PCA9468_TA_MIN_CUR(1.0A) */
					pca9468->ta_cur = PCA9468_TA_MIN_CUR;
					/* Need to control TA voltage for request current */
					ret = pca9468_adjust_ta_voltage(pca9468);
				} else {
					/* Need to control TA current for request current */
					ret = pca9468_adjust_ta_current(pca9468);
				}
			} else {
				/* Wait for next valid state */
				pr_info("%s: Not support new iin yet in charging state=%d\n", __func__, pca9468->charging_state);
			}
		}
	}

	pr_info("%s: ret=%d\n", __func__, ret);
	return ret;
}


/* Set new float voltage */
static int pca9468_set_new_vfloat(struct pca9468_charger *pca9468)
{
	int ret = 0;
	int vbat;
	unsigned int val;

	/* Check the charging state */
	if ((pca9468->charging_state == DC_STATE_NO_CHARGING) ||
		(pca9468->charging_state == DC_STATE_CHECK_VBAT)) {
		/* Apply new vfloat when the direct charging is started */
		pca9468->pdata->v_float = pca9468->new_vfloat;
	} else {
		/* Check whether the previous request is done or not */
		if (pca9468->req_new_vfloat == true) {
			/* The previous request is not done yet */
			pr_err("%s: There is the previous request for New vfloat\n", __func__);
			ret = -EBUSY;
		} else {
			/* Set request flag */
			mutex_lock(&pca9468->lock);
			pca9468->req_new_vfloat = true;
			mutex_unlock(&pca9468->lock);

			/* Check the charging state */
			if ((pca9468->charging_state == DC_STATE_CC_MODE) ||
				(pca9468->charging_state == DC_STATE_CV_MODE) ||
				(pca9468->charging_state == DC_STATE_CHARGING_DONE)) {
				/* Read VBAT ADC */
				vbat = pca9468_read_adc(pca9468, ADCCH_VBAT);

				/* Compare the new VBAT with the current VBAT */
				if (pca9468->new_vfloat > vbat) {
					/* cancel delayed_work */
					cancel_delayed_work(&pca9468->timer_work);
					
					/* Set VFLOAT to new vfloat */
					pca9468->pdata->v_float = pca9468->new_vfloat;
					ret = pca9468_set_vfloat(pca9468, pca9468->pdata->v_float);
					if (ret < 0)
						goto error;
					/* Set IIN_CFG to the current IIN_CC */
					/* save the current iin_cc in iin_cfg */
					pca9468->pdata->iin_cfg = pca9468->iin_cc;	
					pca9468->pdata->iin_cfg = MIN(pca9468->pdata->iin_cfg, pca9468->ta_max_cur*pca9468->chg_mode);
					if (pca9468->parallel_chg == true) {
						ret = pca9468_set_input_current(pca9468, pca9468->pdata->iin_cfg/2);
						if (ret < 0)
							goto error;
						ret = pca9468_set_input_current_sub(pca9468, pca9468->pdata->iin_cfg/2);
						if (ret < 0)
							goto error;
					} else {
						ret = pca9468_set_input_current(pca9468, pca9468->pdata->iin_cfg);
						if (ret < 0)
							goto error;
					}
					pca9468->iin_cc = pca9468->pdata->iin_cfg;

					/* Clear req_new_vfloat */					
					mutex_lock(&pca9468->lock);
					pca9468->req_new_vfloat = false;
					mutex_unlock(&pca9468->lock);

					/* Calculate new TA maximum voltage that used in the direct charging */
					/* Calculate new TA max voltage */
					/* Adjust IIN_CC with APDO resoultion(50mA) - It will recover to the original value after max voltage calculation */
					val = pca9468->iin_cc/(PD_MSG_TA_CUR_STEP*pca9468->chg_mode);
					pca9468->iin_cc = val*(PD_MSG_TA_CUR_STEP*pca9468->chg_mode);
					/* Set TA_MAX_VOL to MIN[PCA9468_TA_MAX_VOL, (TA_MAX_PWR/IIN_CC)] */
					val = pca9468->ta_max_pwr/(pca9468->iin_cc/pca9468->chg_mode/1000);	/* mV */
					val = val*1000/PD_MSG_TA_VOL_STEP;	/* uV */
					val = val*PD_MSG_TA_VOL_STEP; /* Adjust values with APDO resolution(20mV) */
					pca9468->ta_max_vol = MIN(val, PCA9468_TA_MAX_VOL*pca9468->chg_mode);
								
					/* Set TA voltage to MAX[8000mV*chg_mode, (2*VBAT_ADC*chg_mode + 500 mV)] */
					pca9468->ta_vol = max(PCA9468_TA_MIN_VOL_PRESET*pca9468->chg_mode, (2*vbat*pca9468->chg_mode + PCA9468_TA_VOL_PRE_OFFSET));
					val = pca9468->ta_vol/PD_MSG_TA_VOL_STEP;	/* PPS voltage resolution is 20mV */
					pca9468->ta_vol = val*PD_MSG_TA_VOL_STEP;
					/* Set TA voltage to MIN[TA voltage, TA_MAX_VOL] */
					pca9468->ta_vol = MIN(pca9468->ta_vol, pca9468->ta_max_vol);
					/* Set TA current to IIN_CC */
					pca9468->ta_cur = pca9468->iin_cc/pca9468->chg_mode;
					/* Recover IIN_CC to the original value(iin_cfg) */
					pca9468->iin_cc = pca9468->pdata->iin_cfg;

					pr_info("%s: New VFLOAT, ta_max_vol=%d, ta_max_cur=%d, ta_max_pwr=%d, iin_cc=%d, chg_mode=%d\n", 
						__func__, pca9468->ta_max_vol, pca9468->ta_max_cur, pca9468->ta_max_pwr, pca9468->iin_cc, pca9468->chg_mode);

					/* Clear previous IIN ADC */
					pca9468->prev_iin = 0;
					/* Clear TA increment flag */
					pca9468->prev_inc = INC_NONE;

					/* Send PD Message and go to Adjust CC mode */
					pca9468->charging_state = DC_STATE_ADJUST_CC;
					mutex_lock(&pca9468->lock);
					pca9468->timer_id = TIMER_PDMSG_SEND;
					pca9468->timer_period = 0;
					mutex_unlock(&pca9468->lock);
					queue_delayed_work(pca9468->dc_wq,
										&pca9468->timer_work,
										msecs_to_jiffies(pca9468->timer_period));
				} else {
					/* The new VBAT is lower than the current VBAT */
					/* return invalid error */
					pr_err("%s: New vfloat is lower than VBAT ADC\n", __func__);
					ret = -EINVAL;
				}
			} else {
				/* Wait for next valid state */
				pr_info("%s: Not support new vfloat yet in charging state=%d\n", __func__, pca9468->charging_state);
			}
		}
	}
error:
	pr_info("%s: ret=%d\n", __func__, ret);
	return ret;
}

/* 2:1 Direct Charging Adjust CC MODE control */
static int pca9468_charge_adjust_ccmode(struct pca9468_charger *pca9468)
{
	int iin, ccmode;
	int vbatt;
	int iin_main = 0;
	int iin_sub = 0;

	int vin_vol;
	int ret = 0;

	pr_info("%s: ======START=======\n", __func__);

	pca9468->charging_state = DC_STATE_ADJUST_CC;

	/* Check the charging type */
	if (pca9468->parallel_chg == true) {
		/* use main and sub charger */
		ret = pca9468_check_error(pca9468);
		if (ret != 0)
			goto error; // This is not active mode. 

		ret = pca9468_check_error_sub(pca9468);
		if (ret != 0)
			goto error; // This is not active mode.

		/* Check the main charger status first */			
		ccmode = pca9468_check_ccmode_status(pca9468);
		if (ccmode < 0) {
			ret = ccmode;
			goto error;
		}
		if (ccmode == CCMODE_LOOP_INACTIVE) {
			/* main charger has no loop status */
			/* Check the sub charger status */
			ccmode = pca9468_check_ccmode_status_sub(pca9468);
			if (ccmode < 0) {
				ret = ccmode;
				goto error;
			}
		}
	} else {
		/* use only main charger */
		ret = pca9468_check_error(pca9468);
		if (ret != 0)
			goto error; // This is not active mode.
		/* Check the status */
		ccmode = pca9468_check_ccmode_status(pca9468);
		if (ccmode < 0) {
			ret = ccmode;
			goto error;
		}
	}

	switch(ccmode) {
	case CCMODE_IIN_LOOP:
	case CCMODE_CHG_LOOP:
		/* Check TA current */
		if (pca9468->ta_cur > PCA9468_TA_MIN_CUR) {
			/* TA current is higher than 1.0A */
			/* Decrease TA current (50mA) */
			pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;
		}
		pr_info("%s: CC adjust End(LOOP): ta_cur=%d, ta_vol=%d\n", __func__, pca9468->ta_cur, pca9468->ta_vol);

		/* TA target voltage = TA voltage */
		pca9468->ta_target_vol = pca9468->ta_vol;

		/* Read IIN ADC */
		if (pca9468->parallel_chg == true) {
			iin_main = pca9468_read_adc(pca9468, ADCCH_IIN);
			iin_sub = pca9468_read_adc_sub(pca9468, ADCCH_IIN);
			iin = iin_main + iin_sub;
		} else {
			iin = pca9468_read_adc(pca9468, ADCCH_IIN);
		}
		
		/* End TA voltage and current adjustment */
		pr_info("%s: CC adjust End(LOOP): iin=%d, ta_target_vol=%d\n", __func__, iin, pca9468->ta_target_vol);
		/* Clear TA increment flag */
		pca9468->prev_inc = INC_NONE;
		/* Send PD Message and then go to CC mode */
		pca9468->charging_state = DC_STATE_CC_MODE;
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;
		
	case CCMODE_VFLT_LOOP:
		/* Read VBAT ADC */
		vbatt = pca9468_read_adc(pca9468, ADCCH_VBAT);
		/* Save TA target and voltage*/
		pca9468->ta_target_vol = pca9468->ta_vol;
		pr_info("%s: CC adjust End(VFLOAT): vbatt=%d, ta_target_vol=%d\n", __func__, vbatt, pca9468->ta_target_vol);

		/* Clear TA increment flag */
		pca9468->prev_inc = INC_NONE;
		/* Go to Pre-CV mode */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_ENTER_CVMODE;
		pca9468->timer_period = 0;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;

	case CCMODE_LOOP_INACTIVE:
		/* Check IIN ADC with IIN */
		if (pca9468->parallel_chg == true) {
			iin_main = pca9468_read_adc(pca9468, ADCCH_IIN);
			iin_sub = pca9468_read_adc_sub(pca9468, ADCCH_IIN);
			iin = iin_main + iin_sub;
		} else {
			iin = pca9468_read_adc(pca9468, ADCCH_IIN);
		}
		
		/* IIN_ADC > IIN_CC -20mA ? */
		if (iin > (pca9468->iin_cc - PCA9468_IIN_ADC_OFFSET)) 
		{
			/* Input current is already over IIN_CC */
			/* End TA voltage and current adjustment */
			/* change charging state to CC mode */
			pca9468->charging_state = DC_STATE_CC_MODE;

			/* TA target voltage = TA voltage */
			pca9468->ta_target_vol = pca9468->ta_vol;
			pr_info("%s: CC adjust End: IIN_ADC=%d, ta_target_vol=%d\n", __func__, iin, pca9468->ta_target_vol);
			/* Clear TA increment flag */
			pca9468->prev_inc = INC_NONE;
			/* Go to CC mode */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_CHECK_CCMODE;
			pca9468->timer_period = 0;
			mutex_unlock(&pca9468->lock);
		} else {
			/* Check TA voltage */
			if (pca9468->ta_vol == pca9468->ta_max_vol) {
				/* TA voltage is already max value */
				pr_info("%s: CC adjust End: MAX value, ta_vol=%d, ta_cur=%d\n", 
					__func__, pca9468->ta_vol, pca9468->ta_cur);
				/* Clear TA increment flag */
				pca9468->prev_inc = INC_NONE;
				/* Save TA target voltage */
				pca9468->ta_target_vol = pca9468->ta_vol;
				/* Go to CC mode */
				mutex_lock(&pca9468->lock);
				pca9468->timer_id = TIMER_CHECK_CCMODE;
				pca9468->timer_period = 0;
				mutex_unlock(&pca9468->lock);
			} else {
				/* Check TA tolerance */
				/* The current input current compares the final input current(IIN_CC) with 100mA offset */
				/* PPS current tolerance has +/-150mA, so offset defined 100mA(tolerance +50mA) */
				if (iin < (pca9468->iin_cc - PCA9468_TA_IIN_OFFSET)) {
					/* TA voltage too low to enter TA CC mode, so we should increae TA voltage */
					pca9468->ta_vol = pca9468->ta_vol + PCA9468_TA_VOL_STEP_ADJ_CC*pca9468->chg_mode;
					if (pca9468->ta_vol > pca9468->ta_max_vol)
						pca9468->ta_vol = pca9468->ta_max_vol;
					pr_info("%s: CC adjust Cont: ta_vol=%d\n", __func__, pca9468->ta_vol);
					/* Set TA increment flag */
					pca9468->prev_inc = INC_TA_VOL;
					/* Send PD Message */
					mutex_lock(&pca9468->lock);
					pca9468->timer_id = TIMER_PDMSG_SEND;
					pca9468->timer_period = 0;
					mutex_unlock(&pca9468->lock);
				} else {
					/* compare IIN ADC with previous IIN ADC + 20mA */
					if (iin > (pca9468->prev_iin + PCA9468_IIN_ADC_OFFSET)) {
						/* TA can supply more current if TA voltage is high */
						/* TA voltage too low to enter TA CC mode, so we should increae TA voltage */
						pca9468->ta_vol = pca9468->ta_vol + PCA9468_TA_VOL_STEP_ADJ_CC*pca9468->chg_mode;
						if (pca9468->ta_vol > pca9468->ta_max_vol)
							pca9468->ta_vol = pca9468->ta_max_vol;
						pr_info("%s: CC adjust Cont: ta_vol=%d\n", __func__, pca9468->ta_vol);
						/* Set TA increment flag */
						pca9468->prev_inc = INC_TA_VOL;
						/* Send PD Message */
						mutex_lock(&pca9468->lock);
						pca9468->timer_id = TIMER_PDMSG_SEND;
						pca9468->timer_period = 0;
						mutex_unlock(&pca9468->lock);
					} else {
						/* Check the previous increment */
						if (pca9468->prev_inc == INC_TA_CUR) {
							/* The previous increment is TA current, but input current does not increase */
							/* Try to increase TA voltage(40mV) */
							pca9468->ta_vol = pca9468->ta_vol + PCA9468_TA_VOL_STEP_ADJ_CC*pca9468->chg_mode;
							if (pca9468->ta_vol > pca9468->ta_max_vol)
								pca9468->ta_vol = pca9468->ta_max_vol;
							pr_info("%s: CC adjust(flag) Cont: ta_vol=%d\n", __func__, pca9468->ta_vol);
							/* Set TA increment flag */
							pca9468->prev_inc = INC_TA_VOL;
							/* Send PD Message */
							mutex_lock(&pca9468->lock);
							pca9468->timer_id = TIMER_PDMSG_SEND;
							pca9468->timer_period = 0;
							mutex_unlock(&pca9468->lock);
						} else {
							/* The previous increment is TA voltage, but input current does not increase */
							/* Try to increase TA current */
							/* Check APDO max current */
							if (pca9468->ta_cur == pca9468->ta_max_cur) {
								/* TA current is maximum current */

								/* TA target voltage = TA voltage */
								pca9468->ta_target_vol = pca9468->ta_vol;
								pr_info("%s: CC adjust End(MAX_CUR): IIN_ADC=%d, ta_target_vol=%d\n", __func__, iin, pca9468->ta_target_vol);
								/* Clear TA increment flag */
								pca9468->prev_inc = INC_NONE;
								/* Go to CC mode */
								mutex_lock(&pca9468->lock);
								pca9468->timer_id = TIMER_CHECK_CCMODE;
								pca9468->timer_period = 0;
								mutex_unlock(&pca9468->lock);
							} else {
								/* TA has tolerance and compensate it as real current */
								/* Increase TA current(50mA) */
								pca9468->ta_cur = pca9468->ta_cur + PD_MSG_TA_CUR_STEP;
								if (pca9468->ta_cur > pca9468->ta_max_cur)
									pca9468->ta_cur = pca9468->ta_max_cur;
								pr_info("%s: CC adjust Cont: ta_cur=%d\n", __func__, pca9468->ta_cur);
								/* Set TA increment flag */
								pca9468->prev_inc = INC_TA_CUR;
								/* Send PD Message */
								mutex_lock(&pca9468->lock);
								pca9468->timer_id = TIMER_PDMSG_SEND;
								pca9468->timer_period = 0;
								mutex_unlock(&pca9468->lock);
							}
						}
					}
				}
			}
		}
		/* Save previous iin adc */
		pca9468->prev_iin = iin;
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;

	case CCMODE_VIN_UVLO:
		/* VIN UVLO - just notification , it works by hardware */
		vin_vol = pca9468_read_adc(pca9468, ADCCH_VIN);
		pr_info("%s: CC adjust VIN_UVLO: ta_vol=%d, vin_vol=%d\n", __func__, pca9468->ta_cur, vin_vol);
		/* Check VIN after 1sec */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_ADJUST_CCMODE;
		pca9468->timer_period = 1000;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;

	default:
		break;
	}
	
error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


/* 2:1 Direct Charging Start CC MODE control - Pre CC MODE */
/* Increase TA voltage to TA target voltage */
static int pca9468_charge_start_ccmode(struct pca9468_charger *pca9468)
{
	int ret = 0;
	
	pr_info("%s: ======START=======\n", __func__);

	pca9468->charging_state = DC_STATE_START_CC;

	/* Increase TA voltage */
	pca9468->ta_vol = pca9468->ta_vol + PCA9468_TA_VOL_STEP_PRE_CC * pca9468->chg_mode;
	/* Check TA target voltage */
	if (pca9468->ta_vol >= pca9468->ta_target_vol) {
		pca9468->ta_vol = pca9468->ta_target_vol;
		pr_info("%s: PreCC END: ta_vol=%d\n", __func__, pca9468->ta_vol);

		/* Change to DC state to CC mode */
		pca9468->charging_state = DC_STATE_CC_MODE;
	} else {
		pr_info("%s: PreCC: ta_vol=%d\n", __func__, pca9468->ta_vol);
	}

	/* Send PD Message */
	mutex_lock(&pca9468->lock);
	pca9468->timer_id = TIMER_PDMSG_SEND;
	pca9468->timer_period = 0;
	mutex_unlock(&pca9468->lock);
	queue_delayed_work(pca9468->dc_wq,
						&pca9468->timer_work,
						msecs_to_jiffies(pca9468->timer_period));

	return ret;
}


/* 2:1 Direct Charging CC MODE control */
static int pca9468_charge_ccmode(struct pca9468_charger *pca9468)
{
	int ret = 0;
	int ccmode;
	int vin_vol, iin;
	int iin_main, iin_sub;
	
	pr_info("%s: ======START=======\n", __func__);

	pca9468->charging_state = DC_STATE_CC_MODE;

	/* Check the charging type */
	if (pca9468->parallel_chg == true) {
		/* use main and sub charger */
		ret = pca9468_check_error(pca9468);
		if (ret != 0)
			goto error; // This is not active mode. 

		ret = pca9468_check_error_sub(pca9468);
		if (ret != 0)
			goto error; // This is not active mode.
	} else {
		/* use only main charger */
		ret = pca9468_check_error(pca9468);
		if (ret != 0)
			goto error; // This is not active mode.
	}

	/* Check new vfloat request and new iin request */
	if (pca9468->req_new_vfloat == true) {
		/* Clear request flag */
		mutex_lock(&pca9468->lock);
		pca9468->req_new_vfloat = false;
		mutex_unlock(&pca9468->lock);
		ret = pca9468_set_new_vfloat(pca9468);
	} else if (pca9468->req_new_iin == true) {
		/* Clear request flag */
		mutex_lock(&pca9468->lock);
		pca9468->req_new_iin = false;
		mutex_unlock(&pca9468->lock);
		ret = pca9468_set_new_iin(pca9468);
	} else {
		/* Check the charging type */
		if (pca9468->parallel_chg == true) {
			/* use main and sub charger */
			/* Check the main charger status first */			
			ccmode = pca9468_check_ccmode_status(pca9468);
			if (ccmode < 0) {
				ret = ccmode;
				goto error;
			}
			if (ccmode == CCMODE_LOOP_INACTIVE) {
				/* main charger has no loop status */
				/* Check the sub charger status */
				ccmode = pca9468_check_ccmode_status_sub(pca9468);
				if (ccmode < 0) {
					ret = ccmode;
					goto error;
				}
			}
		} else {
			/* use only main charger */
			ccmode = pca9468_check_ccmode_status(pca9468);
			if (ccmode < 0) {
				ret = ccmode;
				goto error;
			}
		}

		switch(ccmode) {
		case CCMODE_LOOP_INACTIVE:
			/* Set input current compensation */
			/* Check the current TA current with TA_MIN_CUR */
			if (pca9468->ta_cur <= PCA9468_TA_MIN_CUR) {
				/* Set TA current to PCA9468_TA_MIN_CUR(1.0A) */
				pca9468->ta_cur = PCA9468_TA_MIN_CUR;
				/* Need input voltage compensation */
				ret = pca9468_set_ta_voltage_comp(pca9468);
			} else {
				if (pca9468->ta_max_vol >= PCA9468_TA_MAX_VOL_CP) {
					/* Need input current compensation */
					ret = pca9468_set_ta_current_comp(pca9468);
				} else {
					/* Need input current compensation in constant power mode */
					ret = pca9468_set_ta_current_comp2(pca9468);
				}
			}
			pr_info("%s: CC INACTIVE: ta_cur=%d, ta_vol=%d\n", __func__, pca9468->ta_cur, pca9468->ta_vol);
			break;

		case CCMODE_VFLT_LOOP:
			/* Read IIN_ADC */
			if (pca9468->parallel_chg == true) {
				/* Read IIN_ADC */
				iin_main = pca9468_read_adc(pca9468, ADCCH_IIN);
				iin_sub = pca9468_read_adc_sub(pca9468, ADCCH_IIN);
				iin = iin_main + iin_sub;
			} else {
				/* Read IIN_ADC */
				iin = pca9468_read_adc(pca9468, ADCCH_IIN);
			}
			pr_info("%s: CC VFLOAT: iin=%d\n", __func__, iin);
			/* go to Pre-CV mode */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_ENTER_CVMODE;
			pca9468->timer_period = 0;
			mutex_unlock(&pca9468->lock);
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->timer_work,
								msecs_to_jiffies(pca9468->timer_period));
			break;

		case CCMODE_IIN_LOOP:
		case CCMODE_CHG_LOOP:
			/* Read IIN_ADC */
			if (pca9468->parallel_chg == true) {
				/* Read IIN_ADC */
				iin_main = pca9468_read_adc(pca9468, ADCCH_IIN);
				iin_sub = pca9468_read_adc_sub(pca9468, ADCCH_IIN);
				iin = iin_main + iin_sub;
			} else {
				/* Read IIN_ADC */
				iin = pca9468_read_adc(pca9468, ADCCH_IIN);
			}

			/* Check the current TA current with TA_MIN_CUR */
			if (pca9468->ta_cur <= PCA9468_TA_MIN_CUR) {
				/* Decrease TA voltage (20mV) */
				pca9468->ta_vol = pca9468->ta_vol - PD_MSG_TA_VOL_STEP;
				pr_info("%s: CC LOOP:iin=%d, next_ta_vol=%d\n", __func__, iin, pca9468->ta_vol);
			} else {
				/* Decrease TA current (50mA) */
				pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;
				pr_info("%s: CC LOOP:iin=%d, next_ta_cur=%d\n", __func__, iin, pca9468->ta_cur);
			}
			/* Send PD Message */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_PDMSG_SEND;
			pca9468->timer_period = 0;
			mutex_unlock(&pca9468->lock);
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->timer_work,
								msecs_to_jiffies(pca9468->timer_period));
			break;

		case CCMODE_VIN_UVLO:
			/* VIN UVLO - just notification , it works by hardware */
			vin_vol = pca9468_read_adc(pca9468, ADCCH_VIN);
			pr_info("%s: CC VIN_UVLO: ta_vol=%d, vin_vol=%d\n", __func__, pca9468->ta_cur, vin_vol);
			/* Check VIN after 1sec */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_CHECK_CCMODE;
			pca9468->timer_period = 1000;
			mutex_unlock(&pca9468->lock);
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->timer_work,
								msecs_to_jiffies(pca9468->timer_period));
			break;

		default:
			break;
		}
	}
error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


/* 2:1 Direct Charging Start CV MODE control - Pre CV MODE */
static int pca9468_charge_start_cvmode(struct pca9468_charger *pca9468)
{
	int ret = 0;
	int cvmode;
	int vin_vol;

	pr_info("%s: ======START=======\n", __func__);

	pca9468->charging_state = DC_STATE_START_CV;

	/* Check the charging type */
	if (pca9468->parallel_chg == true) {
		/* use main and sub charger */
		ret = pca9468_check_error(pca9468);
		if (ret != 0)
			goto error; // This is not active mode. 

		ret = pca9468_check_error_sub(pca9468);
		if (ret != 0)
			goto error; // This is not active mode.

		/* Check the main charger status first */			
		cvmode = pca9468_check_cvmode_status(pca9468);
		if (cvmode < 0) {
			ret = cvmode;
			goto error;
		}
		if (cvmode == CVMODE_LOOP_INACTIVE) {
			/* main charger has no loop status */
			/* Check the sub charger status */
			cvmode = pca9468_check_cvmode_status_sub(pca9468);
			if (cvmode < 0) {
				ret = cvmode;
				goto error;
			}
		}
	} else {
		/* use only main charger */
		ret = pca9468_check_error(pca9468);
		if (ret != 0)
			goto error; // This is not active mode.
		/* Check the status */
		cvmode = pca9468_check_cvmode_status(pca9468);
		if (cvmode < 0) {
			ret = cvmode;
			goto error;
		}
	}

	switch(cvmode) {
	case CVMODE_CHG_LOOP:
	case CVMODE_IIN_LOOP:
		/* Check TA current */
		if (pca9468->ta_cur > PCA9468_TA_MIN_CUR) {
			/* TA current is higher than 1.0A */
			/* Decrease TA current (50mA) */
			pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;
			pr_info("%s: PreCV Cont: ta_cur=%d\n", __func__, pca9468->ta_cur);
		} else {
			/* TA current is less than 1.0A */
			/* Decrease TA voltage (20mV) */
			pca9468->ta_vol = pca9468->ta_vol - PD_MSG_TA_VOL_STEP;
			pr_info("%s: PreCV Cont(IIN_LOOP): ta_vol=%d\n", __func__, pca9468->ta_vol);
		}
		/* Send PD Message */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;

	case CVMODE_VFLT_LOOP:
		/* Decrease TA voltage (20mV) */
		pca9468->ta_vol = pca9468->ta_vol - PCA9468_TA_VOL_STEP_PRE_CV * pca9468->chg_mode;
		pr_info("%s: PreCV Cont: ta_vol=%d\n", __func__, pca9468->ta_vol);
		/* Send PD Message */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = 0;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;

	case CVMODE_LOOP_INACTIVE:
		/* Exit Pre CV mode */
		pr_info("%s: PreCV End: ta_vol=%d, ta_cur=%d\n", __func__, pca9468->ta_vol, pca9468->ta_cur);

		/* Set TA target voltage to TA voltage */
		pca9468->ta_target_vol = pca9468->ta_vol;
		
		/* Need to implement notification to other driver */
		/* To do here */
		
		/* Go to CV mode */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_CHECK_CVMODE;
		pca9468->timer_period = 0;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;

	case CVMODE_VIN_UVLO:
		/* VIN UVLO - just notification , it works by hardware */
		vin_vol = pca9468_read_adc(pca9468, ADCCH_VIN);
		pr_info("%s: PreCV VIN_UVLO: ta_vol=%d, vin_vol=%d\n", __func__, pca9468->ta_cur, vin_vol);
		/* Check VIN after 1sec */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_ENTER_CVMODE;
		pca9468->timer_period = 1000;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;
		
	default:
		break;
	}

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/* 2:1 Direct Charging CV MODE control */
static int pca9468_charge_cvmode(struct pca9468_charger *pca9468)
{
	int ret = 0;
	int cvmode;
	int vin_vol;
	int iin = 0;
	int iin_main, iin_sub;

	pr_info("%s: ======START=======\n", __func__);

	pca9468->charging_state = DC_STATE_CV_MODE;

	if (pca9468->parallel_chg == true) {
		/* use main and sub charger */
		ret = pca9468_check_error(pca9468);
		if (ret != 0)
			goto error; // This is not active mode. 

		ret = pca9468_check_error_sub(pca9468);
		if (ret != 0)
			goto error; // This is not active mode.
	} else {
		/* use only main charger */
		ret = pca9468_check_error(pca9468);
		if (ret != 0)
			goto error; // This is not active mode.
	}

	/* Check new vfloat request and new iin request */
	if (pca9468->req_new_vfloat == true) {
		/* Clear request flag */
		mutex_lock(&pca9468->lock);
		pca9468->req_new_vfloat = false;
		mutex_unlock(&pca9468->lock);
		ret = pca9468_set_new_vfloat(pca9468);
	} else if (pca9468->req_new_iin == true) {
		/* Clear request flag */
		mutex_lock(&pca9468->lock);
		pca9468->req_new_iin = false;
		mutex_unlock(&pca9468->lock);
		ret = pca9468_set_new_iin(pca9468);
	} else {
		if (pca9468->parallel_chg == true) {
			/* use main and sub charger */
			/* Check the main charger status first */			
			cvmode = pca9468_check_cvmode_status(pca9468);
			if (cvmode < 0) {
				ret = cvmode;
				goto error;
			}
			if (cvmode == CVMODE_LOOP_INACTIVE) {
				/* main charger has no loop status */
				/* Check the sub charger status */
				cvmode = pca9468_check_cvmode_status_sub(pca9468);
				if (cvmode < 0) {
					ret = cvmode;
					goto error;
				}
			}
		} else {
			cvmode = pca9468_check_cvmode_status(pca9468);
			if (cvmode < 0) {
				ret = cvmode;
				goto error;
			}		
		}

		/* Check charging done state */
		if (cvmode == CVMODE_LOOP_INACTIVE) {
			if (pca9468->parallel_chg == true) {
				/* Read IIN_ADC */
				iin_main = pca9468_read_adc(pca9468, ADCCH_IIN);
				iin_sub = pca9468_read_adc_sub(pca9468, ADCCH_IIN);
				iin = iin_main + iin_sub;
			} else {
				/* Read IIN_ADC */
				iin = pca9468_read_adc(pca9468, ADCCH_IIN);
			}
			/* Compare iin with input topoff current */
			pr_info("%s: iin=%d, iin_topoff=%d\n", __func__, iin, pca9468->pdata->iin_topoff);
			if (iin < pca9468->pdata->iin_topoff) {
				/* Change cvmode status to charging done */
				cvmode = CVMODE_CHG_DONE;
				pr_info("%s: CVMODE Status=%d\n", __func__, cvmode);
			}
		}
		
		switch(cvmode) {
		case CVMODE_CHG_DONE:
			/* Charging Done */
			/* Keep CV mode until battery driver send stop charging */
			pca9468->charging_state = DC_STATE_CHARGING_DONE;
			power_supply_changed(pca9468->mains);
			
			/* Need to implement notification function */
			/* A customer should insert code */

			pr_info("%s: CV Done\n", __func__);

			/* Check the charging status after notification function */
			if(pca9468->charging_state != DC_STATE_NO_CHARGING) {
				/* Notification function does not stop timer work yet */
				/* Keep the charging done state */
				/* Set timer */
				mutex_lock(&pca9468->lock);
				pca9468->timer_id = TIMER_CHECK_CVMODE;
				pca9468->timer_period = PCA9468_CVMODE_CHECK_T;
				mutex_unlock(&pca9468->lock);
				queue_delayed_work(pca9468->dc_wq,
									&pca9468->timer_work,
									msecs_to_jiffies(pca9468->timer_period));
			} else {
				/* Already called stop charging by notification function */
				pr_info("%s: Already stop DC\n", __func__);
			}
			break;

		case CVMODE_CHG_LOOP:
		case CVMODE_IIN_LOOP:
			/* Check TA current */
			if (pca9468->ta_cur > PCA9468_TA_MIN_CUR) {
				/* TA current is higher than (1.0A*chg_mode) */
				/* Decrease TA current (50mA) */
				pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;
				pr_info("%s: CV LOOP, Cont: ta_cur=%d\n", __func__, pca9468->ta_cur);
			} else {
				/* TA current is less than (1.0A*chg_mode) */
				/* Decrease TA Voltage (20mV) */
				pca9468->ta_vol = pca9468->ta_vol - PD_MSG_TA_VOL_STEP;
				pr_info("%s: CV LOOP, Cont: ta_vol=%d\n", __func__, pca9468->ta_vol);
			}
			/* Send PD Message */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_PDMSG_SEND;
			pca9468->timer_period = 0;
			mutex_unlock(&pca9468->lock);
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->timer_work,
								msecs_to_jiffies(pca9468->timer_period));
			break;

		case CVMODE_VFLT_LOOP:
			/* Decrease TA voltage */
			pca9468->ta_vol = pca9468->ta_vol - PD_MSG_TA_VOL_STEP;
			pr_info("%s: CV VFLOAT, Cont: ta_vol=%d\n", __func__, pca9468->ta_vol);

			/* Set TA target voltage to TA voltage */
			pca9468->ta_target_vol = pca9468->ta_vol;
			
			/* Send PD Message */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_PDMSG_SEND;
			pca9468->timer_period = 0;
			mutex_unlock(&pca9468->lock);
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->timer_work,
								msecs_to_jiffies(pca9468->timer_period));
			break;

		case CVMODE_LOOP_INACTIVE:			
			if (pca9468->parallel_chg == true) {
				/* Check single chargng condition */
				if (iin < PCA9468_IIN_P_DONE) {
					/* input current is less than 1000mA */
					/* change to single charging */
					pca9468->parallel_chg = false;
					/* disable sub charger */					
					ret = pca9468_set_charging_sub(pca9468, false);
					if (ret < 0)
						goto error;
				}
			}
			
			/* Set timer */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_CHECK_CVMODE;
			pca9468->timer_period = PCA9468_CVMODE_CHECK_T;
			mutex_unlock(&pca9468->lock);
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->timer_work,
								msecs_to_jiffies(pca9468->timer_period));
			break;

		case CVMODE_VIN_UVLO:
			/* VIN UVLO - just notification , it works by hardware */
			vin_vol = pca9468_read_adc(pca9468, ADCCH_VIN);
			pr_info("%s: CC VIN_UVLO: ta_vol=%d, vin_vol=%d\n", __func__, pca9468->ta_cur, vin_vol);
			/* Check VIN after 1sec */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_CHECK_CVMODE;
			pca9468->timer_period = 1000;
			mutex_unlock(&pca9468->lock);
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->timer_work,
								msecs_to_jiffies(pca9468->timer_period));
			break;

		default:
			break;
		}
	}

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


/* Preset TA voltage and current for Direct Charging Mode */
static int pca9468_preset_dcmode(struct pca9468_charger *pca9468)
{
	int vbat;
	unsigned int val;
	int ret = 0;
	int chg_mode;

	pr_info("%s: ======START=======\n", __func__);

	pca9468->charging_state = DC_STATE_PRESET_DC;

	/* Read VBAT ADC */
	vbat = pca9468_read_adc(pca9468, ADCCH_VBAT);
	if (vbat < 0) {
	  ret = vbat;
	  goto error;
	}

	/* Compare VBAT with VBAT ADC */
	if (vbat > pca9468->pdata->v_float)	{
		/* Invalid battery voltage to start direct charging */
		pr_err("%s: vbat adc is higher than VFLOAT\n", __func__);
		ret = -EINVAL;
		goto error;
	}

	/* Check the TA mode for wireless charging */
	if (pca9468->chg_mode == WC_DC_MODE) {
		/* Wireless Charger DC mode */
		/* Set IIN_CC to IIN */
		pca9468->iin_cc = pca9468->pdata->iin_cfg;
		/* Go to preset config */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PRESET_CONFIG;
		pca9468->timer_period = 0;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		goto error;
	}

	/* Set the TA max current to input request current(iin_cfg) initially 
		 to get TA maximum current from PD IC */
	pca9468->ta_max_cur = pca9468->pdata->iin_cfg;
	/* Set the TA max voltage to enough high value to find TA maximum voltage initially */
	pca9468->ta_max_vol = PCA9468_TA_MAX_VOL * pca9468->pdata->chg_mode;
	/* Search the proper object position of PDO */
	pca9468->ta_objpos = 0;
	/* Get the APDO max current/voltage(TA_MAX_CUR/VOL) */
	ret = pca9468_get_apdo_max_power(pca9468);
	if (ret < 0) {
		/* TA does not have the desired APDO */
		/* Check the desired mode */
		if (pca9468->pdata->chg_mode == TA_4TO1_DC_MODE) {
			/* TA doesn't have any APDO to support 4:1 mode */
			/* Get the APDO max current/voltage with 2:1 mode */
			pca9468->ta_max_vol = PCA9468_TA_MAX_VOL;
			pca9468->ta_objpos = 0;
			ret = pca9468_get_apdo_max_power(pca9468);
			if (ret < 0) {
				pr_err("%s: TA doesn't have any APDO to support 2:1 or 4:1\n", __func__);
				pca9468->chg_mode = TA_NO_DC_MODE;
				goto error;
			} else {
				/* TA has APDO to support 2:1 mode */
				pca9468->chg_mode = TA_2TO1_DC_MODE;
			}
		} else {
			/* The desired TA mode is 2:1 mode */
			/* TA doesn't have any APDO to support 2:1 mode*/
			pr_err("%s: TA doesn't have any APDO to support 2:1\n", __func__);
			pca9468->chg_mode = TA_NO_DC_MODE;
			goto error;
		}
	} else {
		/* TA has the desired APDO */
		pca9468->chg_mode = pca9468->pdata->chg_mode;
	}
	
	chg_mode = pca9468->chg_mode;
	
	/* Calculate new TA maximum current and voltage that used in the direct charging */
	/* Set IIN_CC to MIN[IIN, (TA_MAX_CUR by APDO)*chg_mode]*/
	pca9468->iin_cc = MIN(pca9468->pdata->iin_cfg, (pca9468->ta_max_cur*chg_mode));
	/* Set the current IIN_CC to iin_cfg for recovering it after resolution adjustment */
	pca9468->pdata->iin_cfg = pca9468->iin_cc;
	/* Calculate new TA max voltage */
	/* Adjust IIN_CC with APDO resoultion(50mA) - It will recover to the original value after max voltage calculation */
	val = pca9468->iin_cc/PD_MSG_TA_CUR_STEP;
	pca9468->iin_cc = val*PD_MSG_TA_CUR_STEP;
	/* Set TA_MAX_VOL to MIN[PCA9468_TA_MAX_VOL*chg_mode, TA_MAX_PWR/(IIN_CC/chg_mode)] */
	val = pca9468->ta_max_pwr/(pca9468->iin_cc/chg_mode/1000);	/* mV */
	val = val*1000/PD_MSG_TA_VOL_STEP;	/* Adjust values with APDO resolution(20mV) */
	val = val*PD_MSG_TA_VOL_STEP; /* uV */
	pca9468->ta_max_vol = MIN(val, PCA9468_TA_MAX_VOL*chg_mode);
	
	/* Set TA voltage to MAX[8000mV*chg_mode, (2*VBAT_ADC*chg_mode + 500 mV)] */
	pca9468->ta_vol = max(PCA9468_TA_MIN_VOL_PRESET*chg_mode, (2*vbat*chg_mode + PCA9468_TA_VOL_PRE_OFFSET));
	val = pca9468->ta_vol/PD_MSG_TA_VOL_STEP;	/* PPS voltage resolution is 20mV */
	pca9468->ta_vol = val*PD_MSG_TA_VOL_STEP;
	/* Set TA voltage to MIN[TA voltage, TA_MAX_VOL*chg_mode] */
	pca9468->ta_vol = MIN(pca9468->ta_vol, pca9468->ta_max_vol);
	/* Set the initial TA current to IIN_CC/chg_mode */
	pca9468->ta_cur = pca9468->iin_cc/chg_mode;
	/* Recover IIN_CC to the original value(iin_cfg) */
	pca9468->iin_cc = pca9468->pdata->iin_cfg;

	pr_info("%s: Preset DC, ta_max_vol=%d, ta_max_cur=%d, ta_max_pwr=%d, iin_cc=%d, chg_mode=%d\n", 
		__func__, pca9468->ta_max_vol, pca9468->ta_max_cur, pca9468->ta_max_pwr, pca9468->iin_cc, pca9468->chg_mode);

	pr_info("%s: Preset DC, ta_vol=%d, ta_cur=%d\n", 
		__func__, pca9468->ta_vol, pca9468->ta_cur);

	/* Send PD Message */
	mutex_lock(&pca9468->lock);
	pca9468->timer_id = TIMER_PDMSG_SEND;
	pca9468->timer_period = 0;
	mutex_unlock(&pca9468->lock);
	queue_delayed_work(pca9468->dc_wq,
						&pca9468->timer_work,
						msecs_to_jiffies(pca9468->timer_period));

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


/* Preset direct charging configuration */
static int pca9468_preset_config(struct pca9468_charger *pca9468)
{
	int ret = 0;
	
	pr_info("%s: ======START=======\n", __func__);
	
	pca9468->charging_state = DC_STATE_PRESET_DC;

	/* Check parallel charging threshold */
	if (pca9468->iin_cc > pca9468->pdata->parallel_th) {
		/* parallel charging */
		/* Set IIN_CFG to IIN_CC */
		ret = pca9468_set_input_current(pca9468, pca9468->iin_cc/2);
		if (ret < 0)
			goto error;	
		ret = pca9468_set_input_current_sub(pca9468, pca9468->iin_cc/2);
		if (ret < 0)
			goto error;
		
		/* Set ICHG_CFG to enough high value */
		ret = pca9468_set_charging_current(pca9468, pca9468->pdata->ichg_cfg);
		if (ret < 0)
			goto error;
		ret = pca9468_set_charging_current_sub(pca9468, pca9468->pdata->ichg_cfg);
		if (ret < 0)
			goto error;
		
		/* Set VFLOAT */
		ret = pca9468_set_vfloat(pca9468, pca9468->pdata->v_float);
		if (ret < 0)
			goto error;
		ret = pca9468_set_vfloat_sub(pca9468, PCA9468_VFLOAT_SUB_DFT);
		if (ret < 0)
			goto error;
		
		/* Enable PCA9468 */	
		ret = pca9468_set_charging(pca9468, true);
		if (ret < 0)
			goto error;		
		ret = pca9468_set_charging_sub(pca9468, true);
		if (ret < 0)
			goto error;

		/* Set the parallel charging flag */
		pca9468->parallel_chg = true;
	} else {
		/* single charging */
		/* Set IIN_CFG to IIN_CC */
		ret = pca9468_set_input_current(pca9468, pca9468->iin_cc);
		if (ret < 0)
			goto error;

		/* Set ICHG_CFG to enough high value */
		ret = pca9468_set_charging_current(pca9468, pca9468->pdata->ichg_cfg);
		if (ret < 0)
			goto error;

		/* Set VFLOAT */
		ret = pca9468_set_vfloat(pca9468, pca9468->pdata->v_float);
		if (ret < 0)
			goto error;

		/* Enable PCA9468 */	
		ret = pca9468_set_charging(pca9468, true);
		if (ret < 0)
			goto error;

		/* Disable the parallel charging */
		pca9468->parallel_chg = false;
	}

	/* Clear previous iin adc */
	pca9468->prev_iin = 0;

	/* Clear TA increment flag */
	pca9468->prev_inc = INC_NONE;

	/* Go to CHECK_ACTIVE state after 150ms*/
	mutex_lock(&pca9468->lock);
	pca9468->timer_id = TIMER_CHECK_ACTIVE;
	pca9468->timer_period = PCA4968_ENABLE_DELAY_T;
	mutex_unlock(&pca9468->lock);
	queue_delayed_work(pca9468->dc_wq,
						&pca9468->timer_work,
						msecs_to_jiffies(pca9468->timer_period));
	ret = 0;
	
error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret; 
}

/* Check the charging status before entering the adjust cc mode */
static int pca9468_check_active_state(struct pca9468_charger *pca9468)
{
	int ret = 0;

	pr_info("%s: ======START=======\n", __func__);

	pca9468->charging_state = DC_STATE_CHECK_ACTIVE;

	/* Check the charging type */
	if (pca9468->parallel_chg == true) {
		/* use main and sub charger */
		/* Check main charger first */
		ret = pca9468_check_error(pca9468);
		if (ret == 0) {
			/* Check sub charger */
			ret = pca9468_check_error_sub(pca9468);
		}
	} else {
		/* use only main charger */
		ret = pca9468_check_error(pca9468);
	}

	if (ret == 0) {
		/* PCA9468 is active state */
		/* Clear retry counter */
		pca9468->retry_cnt = 0;
		/* Check TA mode */
		if (pca9468->chg_mode == WC_DC_MODE) {
			/* Go to WC CV mode */
			/* Implement later */			
			pca9468->timer_id = TIMER_ID_NONE;
			pca9468->timer_period = 0;
			/* It depends on customer's WC algorithm */
			mutex_unlock(&pca9468->lock);
		} else {
			/* Go to Adjust CC mode */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_ADJUST_CCMODE;
			pca9468->timer_period = 0;
			mutex_unlock(&pca9468->lock);
		}
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		ret = 0;
	} else if (ret == -EAGAIN) {
		/* It is the retry condition */
		/* Check the retry counter */
		if (pca9468->retry_cnt < PCA9468_MAX_RETRY_CNT) {
			/* Disable charging */
			ret = pca9468_set_charging(pca9468, false);
			/* Increase retry counter */
			pca9468->retry_cnt++;
			pr_err("%s: retry charging start - retry_cnt=%d\n", __func__, pca9468->retry_cnt);
			/* Go to DC_STATE_PRESET_DC */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_PRESET_DC;
			pca9468->timer_period = 0;
			mutex_unlock(&pca9468->lock);
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->timer_work,
								msecs_to_jiffies(pca9468->timer_period));
			ret = 0;
		} else {
			pr_err("%s: retry fail\n", __func__);
			/* Notify maximum retry error */
			ret = -EINVAL;
		}
	} else {
		/* Implement error handler function if it is needed */
		/* Stop charging in timer_work */
	}
	
	return ret;
}


/* Enter direct charging algorithm */
static int pca9468_start_direct_charging(struct pca9468_charger *pca9468)
{
	int ret;
	unsigned int val;
	struct power_supply *psy;
	//union power_supply_propval pro_val;

	pr_info("%s: =========START=========\n", __func__);

	/* Check the charging state */
	if (pca9468->pdata->parallel == true) {
		/* Support parallel charging by HW */
		/* Set OV_DELTA to 40% */
		val = OV_DELTA_40P << MASK2SHIFT(PCA9468_BIT_OV_DELTA);
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_SAFETY_CTRL,
								PCA9468_BIT_OV_DELTA, val);
		if (ret < 0)
				return ret;
		ret = regmap_update_bits(pca9468->regmap2, PCA9468_REG_SAFETY_CTRL,
								PCA9468_BIT_OV_DELTA, val);
		if (ret < 0)
				return ret;

		/* Set Switching Frequency */
		val = pca9468->pdata->fsw_cfg;
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
								PCA9468_BIT_FSW_CFG, val);
		if (ret < 0)
			return ret;
		ret = regmap_update_bits(pca9468->regmap2, PCA9468_REG_START_CTRL,
								PCA9468_BIT_FSW_CFG, val);
		if (ret < 0)
			return ret;

		/* current sense resistance */
		val = pca9468->pdata->snsres << MASK2SHIFT(PCA9468_BIT_SNSRES);
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
								PCA9468_BIT_SNSRES, val);
		if (ret < 0)
			return ret;
		val = pca9468->pdata->snsres << MASK2SHIFT(PCA9468_BIT_SNSRES);
		ret = regmap_update_bits(pca9468->regmap2, PCA9468_REG_START_CTRL,
								PCA9468_BIT_SNSRES, val);
		if (ret < 0)
			return ret;
		
		/* Set EN_CFG to active low */
		val =  PCA9468_EN_ACTIVE_L;
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
								PCA9468_BIT_EN_CFG, val);
		if (ret < 0)
			return ret;
		ret = regmap_update_bits(pca9468->regmap2, PCA9468_REG_START_CTRL,
								PCA9468_BIT_EN_CFG, val);
		if (ret < 0)
			return ret;

		/* Set NTC voltage threshold */
		val = pca9468->pdata->ntc_th / PCA9468_NTC_TH_STEP;
		ret = regmap_write(pca9468->regmap, PCA9468_REG_NTC_TH_1, (val & 0xFF));	
		if (ret < 0)
			return ret;
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_NTC_TH_2, 
							PCA9468_BIT_NTC_THRESHOLD9_8, (val >> 8));
		if (ret < 0)
			return ret;
		ret = regmap_write(pca9468->regmap2, PCA9468_REG_NTC_TH_1, (val & 0xFF));	
		if (ret < 0)
			return ret;
		ret = regmap_update_bits(pca9468->regmap2, PCA9468_REG_NTC_TH_2, 
							PCA9468_BIT_NTC_THRESHOLD9_8, (val >> 8));
		if (ret < 0)
			return ret;
	} else {
		/* Not support parallel charging - use only main charger */
		/* Set OV_DELTA to 40% */
		val = OV_DELTA_40P << MASK2SHIFT(PCA9468_BIT_OV_DELTA);
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_SAFETY_CTRL,
								PCA9468_BIT_OV_DELTA, val);
		if (ret < 0)
				return ret;

		/* Set Switching Frequency */
		val = pca9468->pdata->fsw_cfg;
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
								PCA9468_BIT_FSW_CFG, val);
		if (ret < 0)
			return ret;

		/* current sense resistance */
		val = pca9468->pdata->snsres << MASK2SHIFT(PCA9468_BIT_SNSRES);
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
								PCA9468_BIT_SNSRES, val);
		if (ret < 0)
			return ret;

		/* Set EN_CFG to active low */
		val =  PCA9468_EN_ACTIVE_L;
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
								PCA9468_BIT_EN_CFG, val);
		if (ret < 0)
			return ret;

		/* Set NTC voltage threshold */
		val = pca9468->pdata->ntc_th / PCA9468_NTC_TH_STEP;
		ret = regmap_write(pca9468->regmap, PCA9468_REG_NTC_TH_1, (val & 0xFF));	
		if (ret < 0)
			return ret;
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_NTC_TH_2, 
							PCA9468_BIT_NTC_THRESHOLD9_8, (val >> 8));
		if (ret < 0)
			return ret;
	}

	/* Check the current power supply type whether it is the wireless charger or USBPD TA */
	/* The below code should be modified by the customer */
	/* Get power supply name */
	psy = power_supply_get_by_name("battery");
	if (!psy) {
		dev_err(pca9468->dev, "Cannot find battery power supply\n");
		ret = -ENODEV;
		return ret;
	}

	/* Get the current power supply type */
	//ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &pro_val);
/*
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_TYPE, &pro_val);
	pro_val.intval = POWER_SUPPLY_TYPE_USB_PD;
	power_supply_put(psy);
*/
/*	
	if (ret < 0) {
		pr_err("====pca9468 ret < 0====\n");
		dev_err(pca9468->dev, "Cannot get power supply type from battery driver\n");
		return ret;
	}
*/
	/* Check the current power supply type */
/*
	if (pro_val.intval == POWER_SUPPLY_TYPE_WIRELESS) {
		// The current power supply type is wireless charger 
		pca9468->chg_mode = WC_DC_MODE;
		pr_info("%s: The current power supply type is WC, chg_mode=%d\n", __func__, pca9468->chg_mode);
	}
	pca9468->chg_mode = TA_2TO1_DC_MODE;
*/	
	/* wake lock */
	__pm_stay_awake(&pca9468->monitor_wake_lock);

	/* Preset charging configuration and TA condition */
	ret = pca9468_preset_dcmode(pca9468);

	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


/* Check Vbat minimum level to start direct charging */
static int pca9468_check_vbatmin(struct pca9468_charger *pca9468)
{
	int vbat;
	int ret;
	union power_supply_propval val;

	pr_info("%s: =========START=========\n", __func__);

	pca9468->charging_state = DC_STATE_CHECK_VBAT;

	/* Check Vbat */
	vbat = pca9468_read_adc(pca9468, ADCCH_VBAT);
	if (vbat < 0) {
		ret = vbat;
	}

#ifdef CONFIG_USBPD_PHY_QCOM
	/* Read switching charger status */
	ret = pca9468_get_switching_charger_property(POWER_SUPPLY_PROP_CURRENT_MAX, &val);
	if (ret < 0) {
		pr_info("%s: Error: does not get switching charger current\n", __func__);
		/* Start Direct Charging again after 1sec */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_VBATMIN_CHECK;
		pca9468->timer_period = PCA9468_VBATMIN_CHECK_T;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		goto error;
	}

	if (val.intval <= SWCHG_ICL_MIN) {
		/* already disabled switching charger */
		/* Clear retry counter */
		pca9468->retry_cnt = 0;
		/* Preset TA voltage and PCA9468 parameters */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PRESET_DC;
		pca9468->timer_period = 0;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
	} else {
		/* Switching charger is enabled */
		if (vbat > PCA9468_DC_VBAT_MIN) {
			/* Start Direct Charging */
			/* now switching charger is enabled */
			/* disable switching charger first */
			ret = pca9468_set_switching_charger(true, SWCHG_ICL_MIN, 0, 0);
		}

		/* Wait 1sec for stopping switching charger or Start 1sec timer for battery check */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_VBATMIN_CHECK;
		pca9468->timer_period = PCA9468_VBATMIN_CHECK_T;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
	}
#else
	/* Read switching charger status */
	ret = pca9468_get_switching_charger_property(POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (ret < 0) {
		/* Start Direct Charging again after 1sec */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_VBATMIN_CHECK;
		pca9468->timer_period = PCA9468_VBATMIN_CHECK_T;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		goto error;
	}

	if (val.intval == 0) {
		/* already disabled switching charger */
		/* Clear retry counter */
		pca9468->retry_cnt = 0;
		/* Preset TA voltage and PCA9468 parameters */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PRESET_DC;
		pca9468->timer_period = 0;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
	} else {
		/* Switching charger is enabled */
		if (vbat > PCA9468_DC_VBAT_MIN) {
			/* Start Direct Charging */
			/* now switching charger is enabled */
			/* disable switching charger first */
			ret = pca9468_set_switching_charger(false, 0, 0, 0);
		}

		/* Wait 1sec for stopping switching charger or Start 1sec timer for battery check */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_VBATMIN_CHECK;
		pca9468->timer_period = PCA9468_VBATMIN_CHECK_T;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
	}
#endif

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


/* delayed work function for charging timer */
static void pca9468_timer_work(struct work_struct *work)
{
	struct pca9468_charger *pca9468 = container_of(work, struct pca9468_charger,
						 timer_work.work);
	int ret = 0;
	unsigned int val;

	pr_info("%s: timer id=%d, charging_state=%d\n", 
		__func__, pca9468->timer_id, pca9468->charging_state);

	switch (pca9468->timer_id) {
	case TIMER_VBATMIN_CHECK:
		ret = pca9468_check_vbatmin(pca9468);
		if (ret < 0)
			goto error;
		break;

	case TIMER_PRESET_DC:
		ret = pca9468_start_direct_charging(pca9468);
		if (ret < 0)
			goto error;
		break;
		
	case TIMER_PRESET_CONFIG:
		ret = pca9468_preset_config(pca9468);
		if (ret < 0)
			goto error;
		break;

	case TIMER_CHECK_ACTIVE:
		ret = pca9468_check_active_state(pca9468);
		if (ret < 0)
			goto error;
		break;
		
	case TIMER_ADJUST_CCMODE:
		ret = pca9468_charge_adjust_ccmode(pca9468);
		if (ret < 0)
			goto error;
		break;			

	case TIMER_ENTER_CCMODE:
		ret = pca9468_charge_start_ccmode(pca9468);
		if (ret < 0)
			goto error;
		break;

	case TIMER_CHECK_CCMODE:
		ret = pca9468_charge_ccmode(pca9468);
		if (ret < 0)
			goto error;
		break;

	case TIMER_ENTER_CVMODE:
		/* Enter Pre-CV mode */
		ret = pca9468_charge_start_cvmode(pca9468);
		if (ret < 0)
			goto error;
		break;

	case TIMER_CHECK_CVMODE:
		ret = pca9468_charge_cvmode(pca9468);
		if (ret < 0)
			goto error;
		break;

	case TIMER_PDMSG_SEND:
		/* Adjust TA current and voltage step */
		val = pca9468->ta_vol/PD_MSG_TA_VOL_STEP;	/* PPS voltage resolution is 20mV */
		pca9468->ta_vol = val*PD_MSG_TA_VOL_STEP;
		val = pca9468->ta_cur/PD_MSG_TA_CUR_STEP;	/* PPS current resolution is 50mA */
		pca9468->ta_cur = val*PD_MSG_TA_CUR_STEP;
		if (pca9468->ta_cur < PCA9468_TA_MIN_CUR)	/* PPS minimum current is 1000mA */
			pca9468->ta_cur = PCA9468_TA_MIN_CUR;
		/* Send PD Message */
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			goto error;

		/* Go to the next state */
		mutex_lock(&pca9468->lock);
		switch (pca9468->charging_state) {
		case DC_STATE_PRESET_DC:
			pca9468->timer_id = TIMER_PRESET_CONFIG;
			break;
		case DC_STATE_ADJUST_CC:
			pca9468->timer_id = TIMER_ADJUST_CCMODE;
			break;
		case DC_STATE_START_CC:
			pca9468->timer_id = TIMER_ENTER_CCMODE;
			break;
		case DC_STATE_CC_MODE:
			pca9468->timer_id = TIMER_CHECK_CCMODE;
			break;
		case DC_STATE_START_CV:
			pca9468->timer_id = TIMER_ENTER_CVMODE;
			break;
		case DC_STATE_CV_MODE:
			pca9468->timer_id = TIMER_CHECK_CVMODE;
			break;
		case DC_STATE_ADJUST_TAVOL:
			pca9468->timer_id = TIMER_ADJUST_TAVOL;
			break;
		case DC_STATE_ADJUST_TACUR:
			pca9468->timer_id = TIMER_ADJUST_TACUR;
			break;
		default:
			ret = -EINVAL;
			break;
		}
		pca9468->timer_period = PCA9468_PDMSG_WAIT_T;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;

	case TIMER_ADJUST_TAVOL:
		ret = pca9468_adjust_ta_voltage(pca9468);
		if (ret < 0)
			goto error;
		break;

	case TIMER_ADJUST_TACUR:
		ret = pca9468_adjust_ta_current(pca9468);
		if (ret < 0)
			goto error;
		break;

	case TIMER_ID_NONE:
		ret = pca9468_stop_charging(pca9468);
		if (ret < 0)
			goto error;
		break;

	default:
		break;
	}

	/* Check the charging state again */
	if (pca9468->charging_state == DC_STATE_NO_CHARGING) {
		/* Cancel work queue again */
		cancel_delayed_work(&pca9468->timer_work);
		cancel_delayed_work(&pca9468->pps_work);
	}
	return;
	
error:
	pca9468_stop_charging(pca9468);
	return;
}


/* delayed work function for pps periodic timer */
static void pca9468_pps_request_work(struct work_struct *work)
{
	struct pca9468_charger *pca9468 = container_of(work, struct pca9468_charger,
						 pps_work.work);

	int ret = 0;
	
	pr_info("%s: pps_work_start\n", __func__);

	/* Send PD message */
	ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
	pr_info("%s: End, ret=%d\n", __func__, ret);
}

static int pca9468_hw_init(struct pca9468_charger *pca9468)
{
	unsigned int val;
	int ret;

	pr_info("%s: =========START=========\n", __func__);

	/* Read Device info register to check the incomplete I2C operation */
	ret = regmap_read(pca9468->regmap, PCA9468_REG_DEVICE_INFO, &val);
	if ((ret < 0) || (val != PCA9468_DEVICE_ID)) {
		/* There is the incomplete I2C operation or I2C communication error */
		/* Read Device info register again */
		ret = regmap_read(pca9468->regmap, PCA9468_REG_DEVICE_INFO, &val);
		if ((ret < 0) || (val != PCA9468_DEVICE_ID)) {
			dev_err(pca9468->dev, "reading DEVICE_INFO failed, val=0x%x\n", val);
			ret = -EINVAL;
			return ret;
		}
	}

	/*
	 * Program the platform specific configuration values to the device
	 * first.
	 */

	/* Set OV_DELTA to 40% */
	val = OV_DELTA_40P << MASK2SHIFT(PCA9468_BIT_OV_DELTA);
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_SAFETY_CTRL,
						 	PCA9468_BIT_OV_DELTA, val);
	if (ret < 0)
		return ret;

	/* Set Switching Frequencey */
	val = pca9468->pdata->fsw_cfg << MASK2SHIFT(PCA9468_BIT_FSW_CFG);
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
						 	PCA9468_BIT_FSW_CFG, val);
	if (ret < 0)
		return ret;

	/* current sense resistance */
	val = pca9468->pdata->snsres << MASK2SHIFT(PCA9468_BIT_SNSRES);
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
						 	PCA9468_BIT_SNSRES, val);
	if (ret < 0)
		return ret;

	/* Set Reverse Current Detection and standby mode*/
	val = PCA9468_BIT_REV_IIN_DET | PCA9468_EN_ACTIVE_L | PCA9468_STANDBY_FORCED;
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
							(PCA9468_BIT_REV_IIN_DET | PCA9468_BIT_EN_CFG | PCA9468_BIT_STANDBY_EN), 
							val);
	if (ret < 0)
		return ret;

	/* clear LIMIT_INCREMENT_EN */
	val = 0;
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_IIN_CTRL,
						 	PCA9468_BIT_LIMIT_INCREMENT_EN, val);
	if (ret < 0)
		return ret;
	
	/* Set the ADC channel */
	val = (PCA9468_BIT_CH7_EN |	/* NTC voltage ADC */
		   PCA9468_BIT_CH6_EN | /* DIETEMP ADC */
		   PCA9468_BIT_CH5_EN | /* IIN ADC */
		   PCA9468_BIT_CH3_EN | /* VBAT ADC */
 		   PCA9468_BIT_CH2_EN | /* VIN ADC */
 		   PCA9468_BIT_CH1_EN); /* VOUT ADC */

	ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_CFG, val);
	if (ret < 0)
		return ret;

	/* ADC Mode change */
	val = 0x5B;
	ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_ACCESS, val);
	if (ret < 0)
		return ret;
	val = 0x10;
	ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_MODE, val);
	if (ret < 0)
		return ret;
	val = 0x00;
	ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_ACCESS, val);	
	if (ret < 0)
		return ret;

	/* Read ADC compesation gain */
	ret = regmap_read(pca9468->regmap, PCA9468_REG_ADC_ADJUST, &val);
	if (ret < 0)
		return ret;
	pca9468->adc_comp_gain = adc_gain[(val>>MASK2SHIFT(PCA9468_BIT_ADC_GAIN))];
	
	/* input current - uA*/
	ret = pca9468_set_input_current(pca9468, pca9468->pdata->iin_cfg);
	if (ret < 0)
		return ret;

	/* charging current */
	ret = pca9468_set_charging_current(pca9468, pca9468->pdata->ichg_cfg);
	if (ret < 0)
		return ret;

	/* v float voltage */
	ret = pca9468_set_vfloat(pca9468, pca9468->pdata->v_float);
	if (ret < 0)
		return ret;

	/* Check the charging type */
	if (pca9468->pdata->parallel == true) {
		/* Can support parallel charging - use main and sub charger */
		/* Initialize sub charger */
		/* Read Device info register to check the incomplete I2C operation */
		ret = regmap_read(pca9468->regmap2, PCA9468_REG_DEVICE_INFO, &val);
		if ((ret < 0) || (val != PCA9468_DEVICE_ID)) {
			/* There is the incomplete I2C operation or I2C communication error */
			/* Read Device info register again */
			ret = regmap_read(pca9468->regmap2, PCA9468_REG_DEVICE_INFO, &val);
			if ((ret < 0) || (val != PCA9468_DEVICE_ID)) {
				dev_err(pca9468->dev, "reading Sub charger DEVICE_INFO failed, val=0x%x\n", val);
				ret = -EINVAL;
				return ret;
			}
		}

		/*
		 * Program the platform specific configuration values to the device
		 * first.
		 */

		/* Set OV_DELTA to 40% */
		val = OV_DELTA_40P << MASK2SHIFT(PCA9468_BIT_OV_DELTA);
		ret = regmap_update_bits(pca9468->regmap2, PCA9468_REG_SAFETY_CTRL,
							 	PCA9468_BIT_OV_DELTA, val);
		if (ret < 0)
			return ret;

		/* Set Switching Frequencey */
		val = pca9468->pdata->fsw_cfg << MASK2SHIFT(PCA9468_BIT_FSW_CFG);
		ret = regmap_update_bits(pca9468->regmap2, PCA9468_REG_START_CTRL,
							 	PCA9468_BIT_FSW_CFG, val);
		if (ret < 0)
			return ret;

		/* current sense resistance */
		val = pca9468->pdata->snsres << MASK2SHIFT(PCA9468_BIT_SNSRES);
		ret = regmap_update_bits(pca9468->regmap2, PCA9468_REG_START_CTRL,
							 	PCA9468_BIT_SNSRES, val);
		if (ret < 0)
			return ret;

		/* Set Reverse Current Detection and standby mode*/
		val = PCA9468_BIT_REV_IIN_DET | PCA9468_EN_ACTIVE_L | PCA9468_STANDBY_FORCED;
		ret = regmap_update_bits(pca9468->regmap2, PCA9468_REG_START_CTRL,
								(PCA9468_BIT_REV_IIN_DET | PCA9468_BIT_EN_CFG | PCA9468_BIT_STANDBY_EN), 
								val);
		if (ret < 0)
			return ret;

		/* clear LIMIT_INCREMENT_EN */
		val = 0;
		ret = regmap_update_bits(pca9468->regmap2, PCA9468_REG_IIN_CTRL,
							 	PCA9468_BIT_LIMIT_INCREMENT_EN, val);
		if (ret < 0)
			return ret;
		
		/* Set the ADC channel */
		val = (PCA9468_BIT_CH7_EN |	/* NTC voltage ADC */
			   PCA9468_BIT_CH6_EN | /* DIETEMP ADC */
			   PCA9468_BIT_CH5_EN | /* IIN ADC */
			   PCA9468_BIT_CH3_EN | /* VBAT ADC */
	 		   PCA9468_BIT_CH2_EN | /* VIN ADC */
	 		   PCA9468_BIT_CH1_EN); /* VOUT ADC */

		ret = regmap_write(pca9468->regmap2, PCA9468_REG_ADC_CFG, val);
		if (ret < 0)
			return ret;

		/* ADC Mode change */
		val = 0x5B;
		ret = regmap_write(pca9468->regmap2, PCA9468_REG_ADC_ACCESS, val);
		if (ret < 0)
			return ret;
		val = 0x10;
		ret = regmap_write(pca9468->regmap2, PCA9468_REG_ADC_MODE, val);
		if (ret < 0)
			return ret;
		val = 0x00;
		ret = regmap_write(pca9468->regmap2, PCA9468_REG_ADC_ACCESS, val);	
		if (ret < 0)
			return ret;

		/* Read ADC compesation gain */
		ret = regmap_read(pca9468->regmap2, PCA9468_REG_ADC_ADJUST, &val);
		if (ret < 0)
			return ret;
		pca9468->adc_comp_gain_sub = adc_gain[(val>>MASK2SHIFT(PCA9468_BIT_ADC_GAIN))];
		
		/* input current - uA*/
		ret = pca9468_set_input_current_sub(pca9468, pca9468->pdata->iin_cfg);
		if (ret < 0)
			return ret;

		/* charging current */
		ret = pca9468_set_charging_current_sub(pca9468, pca9468->pdata->ichg_cfg);
		if (ret < 0)
			return ret;

		/* v float voltage */
		ret = pca9468_set_vfloat_sub(pca9468, pca9468->pdata->v_float);
		if (ret < 0)
			return ret;
	} else {
		/* This phone cannot support parallel charging */
		pca9468->parallel_chg = false;
	}
	return ret;
}


static irqreturn_t pca9468_interrupt_handler(int irq, void *data)
{
	struct pca9468_charger *pca9468 = data;
	u8 int1[REG_INT1_MAX], sts[REG_STS_MAX];	/* INT1, INT1_MSK, INT1_STS, STS_A, B, C, D */
	u8 masked_int;	/* masked int */
	bool handled = false;
	int ret;

	/* Read INT1, INT1_MSK, INT1_STS */
	ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_INT1, int1, 3);
	if (ret < 0) {
		dev_warn(pca9468->dev, "reading INT1_X failed\n");
		return IRQ_NONE;
	}

	/* Read STS_A, B, C, D */
	ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_A, sts, 4);
	if (ret < 0) {
		dev_warn(pca9468->dev, "reading STS_X failed\n");
		return IRQ_NONE;
	}

	pr_info("%s: int1=0x%2x, int1_sts=0x%2x, sts_a=0x%2x\n", __func__, 
			int1[REG_INT1], int1[REG_INT1_STS], sts[REG_STS_A]);

	/* Check Interrupt */
	masked_int = int1[REG_INT1] & !int1[REG_INT1_MSK];
	if (masked_int & PCA9468_BIT_V_OK_INT) {
		/* V_OK interrupt happened */
		mutex_lock(&pca9468->lock);
		pca9468->mains_online = (int1[REG_INT1_STS] & PCA9468_BIT_V_OK_STS) ? true : false;
		mutex_unlock(&pca9468->lock);
		power_supply_changed(pca9468->mains);
		handled = true;
	}

	if (masked_int & PCA9468_BIT_NTC_TEMP_INT) {
		/* NTC_TEMP interrupt happened */
		if (int1[REG_INT1_STS] & PCA9468_BIT_NTC_TEMP_STS) {
			/* above NTC_THRESHOLD */
			dev_err(pca9468->dev, "charging stopped due to NTC threshold voltage\n");
		}
		handled = true;
	}

	if (masked_int & PCA9468_BIT_CHG_PHASE_INT) {
		/* CHG_PHASE interrupt happened */
		if (int1[REG_INT1_STS] & PCA9468_BIT_CHG_PHASE_STS) {
			/* Any of loops is active*/
			if (sts[REG_STS_A] & PCA9468_BIT_VFLT_LOOP_STS)	{
				/* V_FLOAT loop is in regulation */
				pr_info("%s: V_FLOAT loop interrupt\n", __func__);
				/* Disable CHG_PHASE_M */
				ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_INT1_MSK, 
										PCA9468_BIT_CHG_PHASE_M, PCA9468_BIT_CHG_PHASE_M);
				if (ret < 0) {
					handled = false;
					return handled;
				}
				/* Go to Pre CV Mode */
				pca9468->timer_id = TIMER_ENTER_CVMODE;
				pca9468->timer_period = 10;
				queue_delayed_work(pca9468->dc_wq,
									&pca9468->timer_work,
									msecs_to_jiffies(pca9468->timer_period));
			} else if (sts[REG_STS_A] & PCA9468_BIT_IIN_LOOP_STS) {
				/* IIN loop or ICHG loop is in regulation */
				pr_info("%s: IIN loop interrupt\n", __func__);
			} else if (sts[REG_STS_A] & PCA9468_BIT_CHG_LOOP_STS) {
				/* ICHG loop is in regulation */
				pr_info("%s: ICHG loop interrupt\n", __func__);
			}
		}
		handled = true;
	}

	if (masked_int & PCA9468_BIT_CTRL_LIMIT_INT) {
		/* CTRL_LIMIT interrupt happened */
		if (int1[REG_INT1_STS] & PCA9468_BIT_CTRL_LIMIT_STS) {
			/* No Loop is active or OCP */
			if (sts[REG_STS_B] & PCA9468_BIT_OCP_FAST_STS) {
				/* Input fast over current */
				dev_err(pca9468->dev, "IIN > 50A instantaneously\n");
			}
			if (sts[REG_STS_B] & PCA9468_BIT_OCP_AVG_STS) {
				/* Input average over current */
				dev_err(pca9468->dev, "IIN > IIN_CFG*150percent\n");
			}
		}
		handled = true;
	}

	if (masked_int & PCA9468_BIT_TEMP_REG_INT) {
		/* TEMP_REG interrupt happened */
		if (int1[REG_INT1_STS] & PCA9468_BIT_TEMP_REG_STS) {
			/* Device is in temperature regulation */
			dev_err(pca9468->dev, "Device is in temperature regulation\n");
		}
		handled = true;
	}

	if (masked_int & PCA9468_BIT_ADC_DONE_INT) {
		/* ADC complete interrupt happened */
		dev_dbg(pca9468->dev, "ADC has been completed\n");
		handled = true;
	}

	if (masked_int & PCA9468_BIT_TIMER_INT) {
		/* Timer falut interrupt happened */
		if (int1[REG_INT1_STS] & PCA9468_BIT_TIMER_STS) {
			if (sts[REG_STS_B] & PCA9468_BIT_CHARGE_TIMER_STS) {
				/* Charger timer is expired */
				dev_err(pca9468->dev, "Charger timer is expired\n");
			}
			if (sts[REG_STS_B] & PCA9468_BIT_WATCHDOG_TIMER_STS) {
				/* Watchdog timer is expired */
				dev_err(pca9468->dev, "Watchdog timer is expired\n");
			}
		}
		handled = true;
	}

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static int pca9468_irq_init(struct pca9468_charger *pca9468,
			   struct i2c_client *client)
{
	const struct pca9468_platform_data *pdata = pca9468->pdata;
	int ret, msk, irq;

	pr_info("%s: =========START=========\n", __func__);

	irq = gpio_to_irq(pdata->irq_gpio);

	ret = gpio_request_one(pdata->irq_gpio, GPIOF_IN, client->name);
	if (ret < 0)
		goto fail;

	ret = request_threaded_irq(irq, NULL, pca9468_interrupt_handler,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   client->name, pca9468);
	if (ret < 0)
		goto fail_gpio;

	/*
	 * Configure the Mask Register for interrupts: disable all interrupts by default.
	 */
	msk = (PCA9468_BIT_V_OK_M | 
		   PCA9468_BIT_NTC_TEMP_M | 
		   PCA9468_BIT_CHG_PHASE_M |
		   PCA9468_BIT_RESERVED_M |
		   PCA9468_BIT_CTRL_LIMIT_M |
		   PCA9468_BIT_TEMP_REG_M |
		   PCA9468_BIT_ADC_DONE_M |
		   PCA9468_BIT_TIMER_M);
	ret = regmap_write(pca9468->regmap, PCA9468_REG_INT1_MSK, msk);
	if (ret < 0)
		goto fail_wirte;
	
	client->irq = irq;
	return 0;

fail_wirte:
	free_irq(irq, pca9468);
fail_gpio:
	gpio_free(pdata->irq_gpio);
fail:
	client->irq = 0;
	return ret;
}


/*
 * Returns the input current limit programmed
 * into the charger in uA.
 */
static int get_input_current_limit(struct pca9468_charger *pca9468)
{
	int ret, intval;
	unsigned int val;

	if (!pca9468->mains_online)
		return -ENODATA;

	ret = regmap_read(pca9468->regmap, PCA9468_REG_IIN_CTRL, &val);
	if (ret < 0)
		return ret;

	intval = (val & PCA9468_BIT_IIN_CFG) * 100000;

	if (intval < 500000)
		intval = 500000;
	
	return intval;
}

/*
 * Returns the constant charge current programmed
 * into the charger in uA.
 */
static int get_const_charge_current(struct pca9468_charger *pca9468)
{
	int ret, intval;
	unsigned int val;

	if (!pca9468->mains_online)
		return -ENODATA;

	ret = regmap_read(pca9468->regmap, PCA9468_REG_ICHG_CTRL, &val);
	if (ret < 0)
		return ret;

	intval = (val & PCA9468_BIT_ICHG_CFG) * 100000;

	return intval;
}

/*
 * Returns the constant charge voltage programmed
 * into the charger in uV.
 */
static int get_const_charge_voltage(struct pca9468_charger *pca9468)
{
	int ret, intval;
	unsigned int val;

	if (!pca9468->mains_online)
		return -ENODATA;

	ret = regmap_read(pca9468->regmap, PCA9468_REG_V_FLOAT, &val);
	if (ret < 0)
		return ret;

	intval = (val * 5 + 3725) * 1000;

	return intval;
}

/*
 * Returns the enable or disable value.
 * into 1 or 0.
 */
static int get_charging_enabled(struct pca9468_charger *pca9468)
{
	int ret, intval;
	unsigned int val;
	
	ret = regmap_read(pca9468->regmap, PCA9468_REG_START_CTRL, &val);
	if (ret < 0)
		return ret;

	intval = (val & PCA9468_BIT_STANDBY_EN) ? 0 : 1;

	return intval;
}

static int pca9468_mains_set_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     const union power_supply_propval *val)
{
	struct pca9468_charger *pca9468 = power_supply_get_drvdata(psy);
	int ret = 0;

	pr_info("%s: =========START=========\n", __func__);
	pr_info("%s: prop=%d, val=%d\n", __func__, prop, val->intval);

	switch (prop) {
	/* Todo - Insert code */
	/* It needs modification by a customer */
	/* The customer make a decision to start charging and stop charging property */
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		if (val->intval == 0) {			
			/* Cancel delayed work */
			cancel_delayed_work(&pca9468->timer_work);
			cancel_delayed_work(&pca9468->pps_work);
			// Stop Direct Charging		
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_ID_NONE;
			pca9468->timer_period = 0;
			mutex_unlock(&pca9468->lock);
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->timer_work,
								msecs_to_jiffies(pca9468->timer_period));
			ret = 0;
		} else {
			if (pca9468->charging_state == DC_STATE_NO_CHARGING) {
				// Start Direct Charging
				/* Start 1sec timer for battery check */
				mutex_lock(&pca9468->lock);
				pca9468->charging_state = DC_STATE_CHECK_VBAT;
				pca9468->timer_id = TIMER_VBATMIN_CHECK;
				pca9468->timer_period = 1000;	/* The delay time for PD state goes to PE_SNK_STATE */
				mutex_unlock(&pca9468->lock);
				queue_delayed_work(pca9468->dc_wq,
									&pca9468->timer_work,
									msecs_to_jiffies(pca9468->timer_period));
				/* Set the initial charging step */
				power_supply_changed(pca9468->mains);
				ret = 0;
			} else {
				pr_info("%s: already enabled\n", __func__);
			}
		}
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		if (val->intval != pca9468->new_vfloat) {
			/* reqeust new float voltage */
			pca9468->new_vfloat = val->intval;
			ret = pca9468_set_new_vfloat(pca9468);
		}
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (val->intval != pca9468->new_iin) {
			/* reqeust new input current */
			pca9468->new_iin = val->intval;
			ret = pca9468_set_new_iin(pca9468);
		}
		break;

	case POWER_SUPPLY_PROP_BATT_FULL_CURRENT:
		if (val->intval != pca9468->pdata->iin_topoff) {
			/* request new topoff current */
			pca9468->pdata->iin_topoff = val->intval;
		}
		break;

	case POWER_SUPPLY_PROP_CHARGE_DONE:
		if ((val->intval == 1) && 
			(pca9468->charging_state == DC_STATE_CHARGING_DONE)) {
			/* Set charging done from battery manager */			
			/* Stop direct charginig */
			ret = pca9468_stop_charging(pca9468);
			if (ret < 0)
				goto error;
		}
		break;
	#if defined(CONFIG_USBPD_PHY_QCOM)
	case POWER_SUPPLY_PROP_NUBIA_FAST_CHARGE:
		if(val->intval == 1){
			pca9468->nubia_fast_charge = 1;
		}else{
			pca9468->nubia_fast_charge = 0;
		}
		break;
	#endif
	default:
		ret = -EINVAL;
		break;
	}

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


static int pca9468_mains_get_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     union power_supply_propval *val)
{
	int ret = 0;
	struct pca9468_charger *pca9468 = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (pca9468->timer_id == TIMER_ID_NONE)
			val->intval = 0;
		else
			val->intval = 1;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = get_const_charge_voltage(pca9468);
		if (ret < 0)
			return ret;
		else
			val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = get_const_charge_current(pca9468);
		if (ret < 0)
			return ret;
		else
			val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		ret = get_charging_enabled(pca9468);
		if (ret < 0)
			return ret;
		else
			val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = get_input_current_limit(pca9468);
		if (ret < 0)
			return ret;
		else
			val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		/* return NTC voltage  - uV unit */
		ret = pca9468_read_adc(pca9468, ADCCH_NTC);
		if (ret < 0)
			return ret;
		else
			val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* return the output current - uA unit */
		/* check charging status */
		if (pca9468->charging_state == DC_STATE_NO_CHARGING) {
			/* return invalid */
			val->intval = 0;
			ret = -EINVAL;
		} else {
			int iin;
			/* get input current */
			iin = pca9468_read_adc(pca9468, ADCCH_IIN);
			if (ret < 0) {
				dev_err(pca9468->dev, "Invalid IIN ADC\n");
				return ret;
			}
		
			/* calculate the output current */
			/* Iout = (Iin - Ifsw_cfg)*2 */
			val->intval = (iin - iin_fsw_cfg[pca9468->pdata->fsw_cfg])*2;	
		}
		break;

	case POWER_SUPPLY_PROP_BATT_FULL_CURRENT:
		val->intval = pca9468->pdata->iin_topoff;
		break;

	case POWER_SUPPLY_PROP_CHARGE_DONE:
		/* Check the charging state */
		if (pca9468->charging_state == DC_STATE_CHARGING_DONE) {
			val->intval = 1;
		} else {
			val->intval = 0;
		}
		break;
	#if defined(CONFIG_USBPD_PHY_QCOM)
	case POWER_SUPPLY_PROP_NUBIA_FAST_CHARGE:
		val->intval = pca9468->nubia_fast_charge;		
		break;
	#endif	
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property pca9468_mains_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	#if defined(CONFIG_USBPD_PHY_QCOM)
	POWER_SUPPLY_PROP_NUBIA_FAST_CHARGE,
	#endif
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_BATT_FULL_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_DONE,
};


static const struct regmap_config pca9468_regmap = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= PCA9468_MAX_REGISTER,
};

static const struct power_supply_desc pca9468_mains_desc = {
	.name		= "pca9468-mains",
	.type		= POWER_SUPPLY_TYPE_MAINS,
	.get_property	= pca9468_mains_get_property,
	.set_property 	= pca9468_mains_set_property,
	.properties	= pca9468_mains_properties,
	.num_properties	= ARRAY_SIZE(pca9468_mains_properties),
};


/*
 * Returns the input current limit programmed
 * into the charger in uA.
 */
static int get_input_current_limit_sub(struct pca9468_charger *pca9468)
{
	int ret, intval;
	unsigned int val;

	if (!pca9468->mains_online)
		return -ENODATA;

	ret = regmap_read(pca9468->regmap2, PCA9468_REG_IIN_CTRL, &val);
	if (ret < 0)
		return ret;

	intval = (val & PCA9468_BIT_IIN_CFG) * 100000;

	if (intval < 500000)
		intval = 500000;
	
	return intval;
}

/*
 * Returns the constant charge current programmed
 * into the charger in uA.
 */
static int get_const_charge_current_sub(struct pca9468_charger *pca9468)
{
	int ret, intval;
	unsigned int val;

	if (!pca9468->mains_online)
		return -ENODATA;

	ret = regmap_read(pca9468->regmap2, PCA9468_REG_ICHG_CTRL, &val);
	if (ret < 0)
		return ret;

	intval = (val & PCA9468_BIT_ICHG_CFG) * 100000;

	return intval;
}

/*
 * Returns the constant charge voltage programmed
 * into the charger in uV.
 */
static int get_const_charge_voltage_sub(struct pca9468_charger *pca9468)
{
	int ret, intval;
	unsigned int val;

	if (!pca9468->mains_online)
		return -ENODATA;

	ret = regmap_read(pca9468->regmap2, PCA9468_REG_V_FLOAT, &val);
	if (ret < 0)
		return ret;

	intval = (val * 5 + 3725) * 1000;

	return intval;
}

/*
 * Returns the enable or disable value.
 * into 1 or 0.
 */
static int get_charging_enabled_sub(struct pca9468_charger *pca9468)
{
	int ret, intval;
	unsigned int val;
	
	ret = regmap_read(pca9468->regmap2, PCA9468_REG_START_CTRL, &val);
	if (ret < 0)
		return ret;

	intval = (val & PCA9468_BIT_STANDBY_EN) ? 0 : 1;

	return intval;
}

static int pca9468_sub_set_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     const union power_supply_propval *val)
{
	int ret = 0;

	pr_info("%s: =========START=========\n", __func__);
	pr_info("%s: prop=%d, val=%d\n", __func__, prop, val->intval);

	switch (prop) {
	/* Todo - Insert code */
	/* It needs modification by a customer */
	/* The customer make a decision to start charging and stop charging property */
	default:
		ret = -EINVAL;
		break;
	}

	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


static int pca9468_sub_get_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     union power_supply_propval *val)
{
	int ret = 0;
	struct pca9468_charger *pca9468 = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = get_const_charge_voltage_sub(pca9468);
		if (ret < 0)
			return ret;
		else
			val->intval = ret;
		break;
		
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = get_const_charge_current_sub(pca9468);
		if (ret < 0)
			return ret;
		else
			val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		ret = get_charging_enabled_sub(pca9468);
		if (ret < 0)
			return ret;
		else
			val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = get_input_current_limit_sub(pca9468);
		if (ret < 0)
			return ret;
		else
			val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		/* return NTC voltage  - uV unit */
		ret = pca9468_read_adc_sub(pca9468, ADCCH_NTC);
		if (ret < 0)
			return ret;
		else
			val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* return the output current - uA unit */
		/* check charging status */
		if (pca9468->charging_state == DC_STATE_NO_CHARGING) {
			/* return invalid */
			val->intval = 0;
			ret = -EINVAL;
		} else {
			int iin;
			/* get input current */
			iin = pca9468_read_adc_sub(pca9468, ADCCH_IIN);
			if (ret < 0) {
				dev_err(pca9468->dev, "Invalid IIN ADC\n");
				return ret;
			}
		
			/* calculate the output current */
			/* Iout = (Iin - Ifsw_cfg)*2 */
			val->intval = (iin - iin_fsw_cfg[pca9468->pdata->fsw_cfg])*2;	
		}
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property pca9468_sub_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static const struct power_supply_desc pca9468_sub_desc = {
	.name		= "pca9468-sub",
	.type		= POWER_SUPPLY_TYPE_UNKNOWN,
	.get_property	= pca9468_sub_get_property,
	.set_property 	= pca9468_sub_set_property,
	.properties	= pca9468_sub_properties,
	.num_properties	= ARRAY_SIZE(pca9468_sub_properties),
};

static const struct regmap_config pca9468_regmap2 = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= PCA9468_MAX_REGISTER,
};


#if defined(CONFIG_OF)
static int of_pca9468_dt(struct device *dev, struct pca9468_platform_data *pdata)
{
	struct device_node *np_pca9468 = dev->of_node;
	int ret;
	if(!np_pca9468)
		return -EINVAL;

	/* irq gpio */
	pdata->irq_gpio = of_get_named_gpio(np_pca9468, "pca9468,irq-gpio", 0);
	pr_info("%s: irq-gpio: %u \n", __func__, pdata->irq_gpio);

	/* input current limit */
	ret = of_property_read_u32(np_pca9468, "pca9468,input-current-limit",
						   &pdata->iin_cfg);
	if (ret) {
		pr_info("%s: pca9468,input-current-limit is Empty\n", __func__);
		pdata->iin_cfg = PCA9468_IIN_CFG_DFT;
	}
	pr_info("%s: pca9468,iin_cfg is %d\n", __func__, pdata->iin_cfg);

	/* charging current */
	ret = of_property_read_u32(np_pca9468, "pca9468,charging-current",
							   &pdata->ichg_cfg);
	if (ret) {
		pr_info("%s: pca9468,charging-current is Empty\n", __func__);
		pdata->ichg_cfg = PCA9468_ICHG_CFG_DFT;
	}
	pr_info("%s: pca9468,ichg_cfg is %d\n", __func__, pdata->ichg_cfg);

	/* charging float voltage */
	ret = of_property_read_u32(np_pca9468, "pca9468,float-voltage",
							   &pdata->v_float);
	if (ret) {
		pr_info("%s: pca9468,float-voltage is Empty\n", __func__);
		pdata->v_float = PCA9468_VFLOAT_DFT;
	}
	pr_info("%s: pca9468,v_float is %d\n", __func__, pdata->v_float);

	/* input topoff current */
	ret = of_property_read_u32(np_pca9468, "pca9468,input-itopoff",
							   &pdata->iin_topoff);
	if (ret) {
		pr_info("%s: pca9468,input-itopoff is Empty\n", __func__);
		pdata->iin_topoff = PCA9468_IIN_DONE_DFT;
	}
	pr_info("%s: pca9468,iin_topoff is %d\n", __func__, pdata->iin_topoff);

	/* sense resistance */
	ret = of_property_read_u32(np_pca9468, "pca9468,sense-resistance",
							   &pdata->snsres);
	if (ret) {
		pr_info("%s: pca9468,sense-resistance is Empty\n", __func__);
		pdata->snsres = PCA9468_SENSE_R_DFT;
	}
	pr_info("%s: pca9468,snsres is %d\n", __func__, pdata->snsres);

	/* switching frequency */
	ret = of_property_read_u32(np_pca9468, "pca9468,switching-frequency",
							   &pdata->fsw_cfg);
	if (ret) {
		pr_info("%s: pca9468,switching frequency is Empty\n", __func__);
		pdata->fsw_cfg = PCA9468_FSW_CFG_DFT;
	}
	pr_info("%s: pca9468,fsw_cfg is %d\n", __func__, pdata->fsw_cfg);

	/* NTC threshold voltage */
	ret = of_property_read_u32(np_pca9468, "pca9468,ntc-threshold",
							   &pdata->ntc_th);
	if (ret) {
		pr_info("%s: pca9468,ntc threshold voltage is Empty\n", __func__);
		pdata->ntc_th = PCA9468_NTC_TH_DFT;
	}
	pr_info("%s: pca9468,ntc_th is %d\n", __func__, pdata->ntc_th);

	/* Charging mode */
	ret = of_property_read_u32(np_pca9468, "pca9468,chg-mode",
							   &pdata->chg_mode);
	if (ret) {
		pr_info("%s: pca9468,charging mode is Empty\n", __func__);
		pdata->chg_mode = TA_2TO1_DC_MODE;
	}
	pr_info("%s: pca9468,chg_mode is %d\n", __func__, pdata->chg_mode);

	/* Charging Type */
	ret = of_property_read_u32(np_pca9468, "pca9468,parallel",
							   &pdata->parallel);
	if (ret) {
		pr_info("%s: pca9468,parallel is Empty\n", __func__);
		pdata->parallel = 0;
	}
	pr_info("%s: pca9468,parallel is %d\n", __func__, pdata->parallel);


	if (pdata->parallel == true) {
		/* Sub I2C slave address */
		ret = of_property_read_u32(np_pca9468, "pca9468,sub-i2c-addr",
								   &pdata->sub_i2c_addr);
		if (ret) {
			pr_info("%s: pca9468,sub i2c address is Empty\n", __func__);
			pdata->sub_i2c_addr = 0;
		}
		pr_info("%s: pca9468,sub-i2c-addr is 0x%x\n", __func__, pdata->sub_i2c_addr);

		/* Parallel charging threshold  */
		ret = of_property_read_u32(np_pca9468, "pca9468,parallel-threshold",
								   &pdata->parallel_th);
		if (ret) {
			pr_info("%s: pca9468,parallel threshold is Empty\n", __func__);
			pdata->parallel_th = PCA9468_IIN_P_TH_DFT;
		}
		pr_info("%s: pca9468,parallel-threshold is %d\n", __func__, pdata->parallel_th);
	} else {
		/* Only support single charging and the threshold is the maximum value */
		pdata->parallel_th = PCA9468_IIN_S_TH_DFT;
	}

	return 0;
}
#else
static int of_pca9468_dt(struct device *dev, struct pca9468_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */

#ifdef CONFIG_USBPD_PHY_QCOM
static int pca9468_usbpd_setup(struct pca9468_charger *pca9468)
{
	int ret = 0;
	const char *pd_phandle = "usbpd-phy";

	pca9468->pd = devm_usbpd_get_by_phandle(pca9468->dev, pd_phandle);

	if (IS_ERR(pca9468->pd)) {
		pr_err("get_usbpd phandle failed (%ld)\n",
				PTR_ERR(pca9468->pd));
		return PTR_ERR(pca9468->pd);
	}

	return ret;
}
#endif

static int read_reg(void *data, u64 *val)
{
	struct pca9468_charger *chip = data;
	int rc;
	unsigned int temp;

	rc = regmap_read(chip->regmap, chip->debug_address, &temp); 
	if (rc) {
		pr_err("Couldn't read reg %x rc = %d\n",
			chip->debug_address, rc);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int write_reg(void *data, u64 val)
{
	struct pca9468_charger *chip = data;
	int rc;
	u8 temp;

	temp = (u8) val;
	rc = regmap_write(chip->regmap, chip->debug_address, temp);
	if (rc) {
		pr_err("Couldn't write 0x%02x to 0x%02x rc= %d\n",
			temp, chip->debug_address, rc);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(register_debug_ops, read_reg, write_reg, "0x%02llx\n");

static int pca9468_create_debugfs_entries(struct pca9468_charger *chip)
{
	struct dentry *ent;
	int rc = 0;
	
	chip->debug_root = debugfs_create_dir("charger-pca9468", NULL);
	if (!chip->debug_root) {
		dev_err(chip->dev, "Couldn't create debug dir\n");
		rc = -ENOENT;
	} else {
		ent = debugfs_create_x32("address", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root,
					  &(chip->debug_address));
		if (!ent) {
			dev_err(chip->dev,
				"Couldn't create address debug file\n");
			rc = -ENOENT;
		}

		ent = debugfs_create_file("data", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root, chip,
					  &register_debug_ops);
		if (!ent) {
			dev_err(chip->dev,
				"Couldn't create data debug file\n");
			rc = -ENOENT;
		}
	}

	return rc;
}


static int pca9468_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{	
	static char *battery[] = { "pca9468-battery" };
	struct power_supply_config mains_cfg = {};
	struct power_supply_config sub_cfg = {};

	struct pca9468_platform_data *pdata;
	struct device *dev = &client->dev;
	struct pca9468_charger *pca9468_chg;
	int ret;

	pr_info("%s: =========START=========\n", __func__);

	pca9468_chg = devm_kzalloc(dev, sizeof(*pca9468_chg), GFP_KERNEL);
	if (!pca9468_chg)
		return -ENOMEM;

#if defined(CONFIG_OF)
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev, sizeof(struct pca9468_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory \n");
			return -ENOMEM;
		}

		ret = of_pca9468_dt(&client->dev, pdata);
		if (ret < 0){
			dev_err(&client->dev, "Failed to get device of_node \n");
			return -ENOMEM;
		}

		client->dev.platform_data = pdata;
	} else {
		pdata = client->dev.platform_data;
	}
#else
	pdata = dev->platform_data;
#endif
	if (!pdata)
		return -EINVAL;

	i2c_set_clientdata(client, pca9468_chg);

	mutex_init(&pca9468_chg->lock);
	pca9468_chg->dev = &client->dev;
	pca9468_chg->pdata = pdata;
	pca9468_chg->charging_state = DC_STATE_NO_CHARGING;

	/* Create a work queue for the direct charger */
	pca9468_chg->dc_wq = alloc_ordered_workqueue("pca9468_dc_wq", WQ_MEM_RECLAIM);
	if (pca9468_chg->dc_wq == NULL) {
		dev_err(pca9468_chg->dev, "failed to create work queue\n");
		return -ENOMEM;
	}

	wakeup_source_init(&pca9468_chg->monitor_wake_lock, "pca9468-charger-monitor");

	/* initialize work */
	INIT_DELAYED_WORK(&pca9468_chg->timer_work, pca9468_timer_work);
	pca9468_chg->timer_id = TIMER_ID_NONE;
	pca9468_chg->timer_period = 0;

	INIT_DELAYED_WORK(&pca9468_chg->pps_work, pca9468_pps_request_work);

	pca9468_chg->regmap = devm_regmap_init_i2c(client, &pca9468_regmap);
	if (IS_ERR(pca9468_chg->regmap)) {
		ret = -EINVAL;
		goto error;
	}

	/* Check the parallel support */
	if (pca9468_chg->pdata->parallel == true) {
		/* Set Sub charger I2C client and regmap */
		pca9468_chg->i2c_sub = i2c_new_dummy(client->adapter, pca9468_chg->pdata->sub_i2c_addr);
		i2c_set_clientdata(pca9468_chg->i2c_sub, pca9468_chg);

		pca9468_chg->regmap2 = devm_regmap_init_i2c(pca9468_chg->i2c_sub, &pca9468_regmap2);
		if (IS_ERR(pca9468_chg->regmap2)) {
			ret = -EINVAL;
			goto error;
		}
	}

#ifdef CONFIG_USBPD_PHY_QCOM
	pca9468_chg->nubia_fast_charge = 0;
	if (pca9468_usbpd_setup(pca9468_chg)) {
		dev_err(dev, "Error usbpd setup!\n");
		pca9468_chg->pd = NULL;
	}
#endif

	ret = pca9468_hw_init(pca9468_chg);
	if (ret < 0)
		goto error;

	mains_cfg.supplied_to = battery;
	mains_cfg.num_supplicants = ARRAY_SIZE(battery);
	mains_cfg.drv_data = pca9468_chg;
	pca9468_chg->mains = power_supply_register(dev, &pca9468_mains_desc,
					   &mains_cfg);
	if (IS_ERR(pca9468_chg->mains)) {
		ret = -ENODEV;
		goto error;
	}

	if (pca9468_chg->pdata->parallel == true) {
		/* register sub charger */
		sub_cfg.supplied_to = battery;
		sub_cfg.num_supplicants = ARRAY_SIZE(battery);
		sub_cfg.drv_data = pca9468_chg;
		pca9468_chg->sub_chg = power_supply_register(dev, &pca9468_sub_desc,
						   &sub_cfg);
		if (IS_ERR(pca9468_chg->sub_chg)) {
			ret = -ENODEV;
			goto unregister_main_psy;
		}
	}

	/*
	 * Interrupt pin is optional. If it is connected, we setup the
	 * interrupt support here.
	 */
	if (pdata->irq_gpio >= 0) {
		ret = pca9468_irq_init(pca9468_chg, client);
		if (ret < 0) {
			dev_warn(dev, "failed to initialize IRQ: %d\n", ret);
			dev_warn(dev, "disabling IRQ support\n");
		}
		/* disable interrupt */
		disable_irq(client->irq);
	}

	ret = pca9468_create_debugfs_entries(pca9468_chg);
	if (ret < 0)
		goto unregister_sub_psy;

	ret = pca9468_step_chg_init(pca9468_chg->dev);
	if (ret < 0) {
		dev_err(dev, "Couldn't init pca9468_step_chg_init ret=%d\n",
			ret);
		goto unregister_sub_psy;
	}

	pr_info("%s: =========END=========\n", __func__);

	return 0;

unregister_sub_psy:
	power_supply_unregister(pca9468_chg->sub_chg);
unregister_main_psy:
	power_supply_unregister(pca9468_chg->mains);
error:
	destroy_workqueue(pca9468_chg->dc_wq);
	mutex_destroy(&pca9468_chg->lock);
	wakeup_source_trash(&pca9468_chg->monitor_wake_lock);
	return ret;
}

static int pca9468_remove(struct i2c_client *client)
{
	struct pca9468_charger *pca9468_chg = i2c_get_clientdata(client);

	/* stop charging if it is active */
	pca9468_stop_charging(pca9468_chg);
	
	if (client->irq) {
		free_irq(client->irq, pca9468_chg);
		gpio_free(pca9468_chg->pdata->irq_gpio);
	}

	/* Delete the work queue */
	destroy_workqueue(pca9468_chg->dc_wq);

	wakeup_source_trash(&pca9468_chg->monitor_wake_lock);
	if (pca9468_chg->mains)
		power_supply_put(pca9468_chg->mains);
	power_supply_unregister(pca9468_chg->mains);

	if (pca9468_chg->sub_chg)
		power_supply_put(pca9468_chg->sub_chg);
	power_supply_unregister(pca9468_chg->sub_chg);

	pca9468_step_chg_deinit();

	return 0;
}

static const struct i2c_device_id pca9468_id[] = {
	{ "pca9468", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pca9468_id);

#if defined(CONFIG_OF)
static struct of_device_id pca9468_i2c_dt_ids[] = {
	{ .compatible = "nxp,pca9468" },
	{ },
};
MODULE_DEVICE_TABLE(of, pca9468_i2c_dt_ids);
#endif /* CONFIG_OF */

#if defined(CONFIG_PM)
static int pca9468_suspend(struct device *dev)
{
	struct pca9468_charger *pca9468 = dev_get_drvdata(dev);

	pr_debug("%s: cancel delayed work\n", __func__);

	/* cancel delayed_work */
	cancel_delayed_work(&pca9468->timer_work);
	return 0;
}

static int pca9468_resume(struct device *dev)
{
	struct pca9468_charger *pca9468 = dev_get_drvdata(dev);

	pr_debug("%s: update_timer\n", __func__);

	/* Update the current timer */
	if (pca9468->timer_id != TIMER_ID_NONE) {
		mutex_lock(&pca9468->lock);
		pca9468->timer_period = 0;	// ms unit
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
	}
	return 0;
}
#else
#define pca9468_suspend		NULL
#define pca9468_resume		NULL
#endif

const struct dev_pm_ops pca9468_pm_ops = {
	.suspend = pca9468_suspend,
	.resume = pca9468_resume,
};

static struct i2c_driver pca9468_driver = {
	.driver = {
		.name = "pca9468",
#if defined(CONFIG_OF)
		.of_match_table = pca9468_i2c_dt_ids,
#endif /* CONFIG_OF */
#if defined(CONFIG_PM)
		.pm = &pca9468_pm_ops,
#endif
	},
	.probe        = pca9468_probe,
	.remove       = pca9468_remove,
	.id_table     = pca9468_id,
};

module_i2c_driver(pca9468_driver);

MODULE_AUTHOR("Clark Kim <clark.kim@nxp.com>");
MODULE_DESCRIPTION("PCA9468 charger driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("3.6.12N");
