/* Copyright (c) 2014-2016 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt) "NEO: %s: " fmt, __func__

#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/spmi.h>
//#include <linux/wakelock.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/debugfs.h>
#include <linux/ktime.h>
#include <linux/pmic-voter.h>

struct neo_dt_props {
	int	batt_capacity_uah;
	int batt_cutoff_mv;
	int batt_soc_increase_per_sec;
	int batt_soc_decrease_per_sec;
};

struct neo_params {
	int batt_raw_soc;
	int batt_soc;
	int batt_mv;
	int batt_ma;
	int batt_full;
	int batt_full_design;
	int batt_temp;
	int batt_health;
	int batt_status;
	int usb_present;
	int charge_counter;
	int cycle_count;
	int vbus_mv;
	int ibus_ma;
};


struct neo_battery_data {
	struct device			*dev;
	struct platform_device	*pdev;
	struct regmap			*regmap;

	struct dentry			*debug_root;
	struct neo_dt_props		dt;
	struct neo_params		param;

	bool					update_now;

	struct power_supply		*neo_psy;
	struct power_supply		*batt_psy;
	struct power_supply		*bms_psy;
	struct power_supply		*usb_psy;
#if defined(CONFIG_NEO_EXTERNAL_FG_SUPPORT)
	struct power_supply		*ex_batt_psy;
#endif

	struct mutex			write_lock;
	struct wakeup_source	*debug_lock;
	struct votable			*debug_awake_votable;
	struct votable			*soc_monitor_work_votable;
	struct delayed_work		soc_monitor_work;
	//struct wake_lock		monitor_wake_lock;
	struct timespec			last_soc_change_time;
#ifdef CONFIG_NUBIA_POWEROFFCHARGE_NODE
	int 					poweroffcharge;
#endif
};


static int neo_battery_debug_mask = 0x01;
module_param_named(
	debug_mask, neo_battery_debug_mask, int, S_IRUSR | S_IWUSR
);

enum print_reason {
	PR_INFO			= BIT(0),
	PR_REGISTER		= BIT(1),
	PR_INTERRUPT	= BIT(2),
	PR_STATUS		= BIT(3),
	PR_DUMP			= BIT(4),
};

#define PON_SOC_BACKUP_REG				0x88D
#define CHGR_BASE						0x1000
#define BATTERY_CHARGER_STATUS_1_REG	0x1006
#define BATTERY_CHARGER_STATUS_MASK		GENMASK(2, 0)
#define BATTERY_CHARGER_STATUS_2_REG	0x1007
#define BATTERY_JEITA_STATUS_MASK		GENMASK(3, 2)
#define CHARGER_ERROR_STATUS_BAT_OV_BIT	BIT(5)
#define BATTERY_CHARGER_STATUS_5_REG	0x100B
#define FAST_CHARGE_CURRENT_CFG_REG		0x1061
#define FLOAT_VOLTAGE_CFG_REG			0x1070
#define USBIN_APSD_RESULT_STATUS_REG	0x1308
#define SDP_CHARGER_BIT					BIT(0)
#define USBIN_CURRENT_LIMIT_CFG_REG		0x1370
#define MISC_ICL_STATUS_REG				0x1107
#define MISC_AICL_STATUS_REG			0x110A
#define USBIN_LOAD_CFG_REG			0x1365
#define USBIN_ICL_OPTIONS_REG			0x1366
#define USBIN_INT_LATCHED_STS			0x1318
#define TYPEC_INT_LATCHED_STS			0x1518
#define IBATT_RSENSE_LOW				0x30f6
#define IBATT_RSENSE_HIGH				0x30f7


struct pmi_regs_table{
	char *name;
	int  reg;
};

struct pmi_regs_table regs_table[] = {
	{"cs2", BATTERY_CHARGER_STATUS_2_REG},
	{"cs5", BATTERY_CHARGER_STATUS_5_REG},
	{"cc", FAST_CHARGE_CURRENT_CFG_REG},
	{"fv", FLOAT_VOLTAGE_CFG_REG},
	{"ui", USBIN_CURRENT_LIMIT_CFG_REG},
	{"is", MISC_ICL_STATUS_REG},
	{"as", MISC_AICL_STATUS_REG},
	{"lc", USBIN_LOAD_CFG_REG},
	{"io", USBIN_ICL_OPTIONS_REG},
	{"uin", USBIN_INT_LATCHED_STS},
	{"cin", TYPEC_INT_LATCHED_STS},
	{"irl", IBATT_RSENSE_LOW},
	{"irh", IBATT_RSENSE_HIGH}
};

enum {
	TRICKLE_CHARGE = 0,
	PRE_CHARGE,
	FAST_CHARGE,
	FULLON_CHARGE,
	TAPER_CHARGE,
	TERMINATE_CHARGE,
	INHIBIT_CHARGE,
	DISABLE_CHARGE,
};

#define MAX_TIMEOUT_COUNT		3
#define STANDARD_USB_CURRENT	500
#define RECHARGE_SOC_THRESHOLD	99
#define REGISTER_DUMP_VOTER		"REGISTER_DUMP_VOTER"
//static int timeout = MAX_TIMEOUT_COUNT;

#define NEO_DEB(reason, fmt, ...)				\
do {											\
		if (neo_battery_debug_mask & (reason))	\
			pr_info(fmt, ##__VA_ARGS__);		\
} while (0)

static enum power_supply_property neo_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
};

static int spmi_bus_read(struct neo_battery_data *chip, u16 addr, u8 *val)
{
	unsigned int temp;
	int rc = 0;

	rc = regmap_read(chip->regmap, addr, &temp);
	if (rc >= 0)
		*val = (u8)temp;

	return rc;
}

static void spmi_dump_regs(struct neo_battery_data *chip)
{
	u8 reg = 0;
	int table_size = ARRAY_SIZE(regs_table);
	char buf[100]={0},pbuf[200]={0};
	int i;
#if 0
	u8 stat1, stat2, stat3;
	int i, rc;

	rc = spmi_bus_read(chip, BATTERY_CHARGER_STATUS_2_REG, &stat2);
	if (rc < 0) {
		pr_err("Couldn't read BATTERY_CHARGER_STATUS_2_REG rc=%d\n", rc);
		return;
	}

	if(chip->param.usb_present &&
			chip->param.batt_raw_soc < RECHARGE_SOC_THRESHOLD &&
			chip->param.batt_ma > -STANDARD_USB_CURRENT){
		/* USB present and current less than 500mA*/
		rc = spmi_bus_read(chip, BATTERY_CHARGER_STATUS_1_REG, &stat1);
		if (rc < 0) {
			pr_err("Couldn't read BATTERY_CHARGER_STATUS_1_REG rc=%d\n", rc);
			return;
		}

		if(((stat1 & BATTERY_CHARGER_STATUS_MASK) != TERMINATE_CHARGE ||
			(stat1 & BATTERY_CHARGER_STATUS_MASK) != DISABLE_CHARGE ||
			(stat1 & BATTERY_CHARGER_STATUS_MASK) != INHIBIT_CHARGE) &&
			(stat2 & BATTERY_JEITA_STATUS_MASK) == 0) {
			/* Battery in normal temperature */
			rc = spmi_bus_read(chip, USBIN_APSD_RESULT_STATUS_REG, &stat3);
			if (rc < 0) {
				pr_err("Couldn't read USBIN_APSD_RESULT_STATUS_REG rc=%d\n", rc);
				return;
			}
			/*Battery OV status*/
			if(stat2 & CHARGER_ERROR_STATUS_BAT_OV_BIT){
				if(timeout > 0){
					timeout--;
					vote(chip->debug_awake_votable, REGISTER_DUMP_VOTER, true, 0);
					pr_err("Battery OV, dump all the registers. [%d]\n", timeout);
				}
			}
			else if((stat1 & BATTERY_CHARGER_STATUS_MASK) == DISABLE_CHARGE){
				/* Discharging status*/
				if(timeout > 0){
					timeout--;
					vote(chip->debug_awake_votable, REGISTER_DUMP_VOTER, true, 0);
					pr_err("Charging error, dump all the registers. [%d]\n", timeout);
				}
			}
			else if((stat3 & SDP_CHARGER_BIT) != 1){
				/* Not USB connected*/
				if(timeout > 0){
					timeout--;
					vote(chip->debug_awake_votable, REGISTER_DUMP_VOTER, true, 0);
					pr_err("Battery current too small, dump all the registers. [%d]\n", timeout);
				}
			}
		}
	}
	else if (timeout != MAX_TIMEOUT_COUNT) {
		timeout = MAX_TIMEOUT_COUNT;
		vote(chip->debug_awake_votable, REGISTER_DUMP_VOTER, false, 0);
	}

	if(timeout == 0){
		timeout = -1;
		for(i = 0; i < 0x700; i++){
			spmi_bus_read(chip, CHGR_BASE + i, &reg);
			printk("[%x 0x%02x]\n", CHGR_BASE + i, reg);
		}
		vote(chip->debug_awake_votable, REGISTER_DUMP_VOTER, false, 0);
	}
#endif
	for(i = 0; i < table_size; i++){
		spmi_bus_read(chip, regs_table[i].reg, &reg);
		sprintf(buf,"%s[%x %02x] ", regs_table[i].name, regs_table[i].reg, reg);
		strcat(pbuf,buf);
	}
	printk("CHG:%s\n",pbuf);
}

static ssize_t neo_battery_show_charge_full_design(struct device *dev,
			    struct device_attribute *attr, char *buf)
{

	struct power_supply *neo = power_supply_get_by_name("neo-battery");
	struct neo_battery_data *chip = power_supply_get_drvdata(neo);

	if(!chip){
		return 0;
	}

	return sprintf(buf, "%d\n", chip->dt.batt_capacity_uah);
}
static DEVICE_ATTR(charge_full_design, S_IRUGO, neo_battery_show_charge_full_design, NULL);

static ssize_t neo_battery_update_battery_information(struct device *dev,
			    struct device_attribute *attr, const char *buf, size_t count)
{
	int i = 0;
	struct power_supply *neo = power_supply_get_by_name("neo-battery");
	struct neo_battery_data *chip = power_supply_get_drvdata(neo);
	struct power_supply *psy =  chip->neo_psy;
	union power_supply_propval ret;

	/* Go through all properties for the psy */
	for (i = 0; i < 9; i++) {
		enum power_supply_property prop;
		prop = psy->desc->properties[i];

		if (power_supply_get_property(psy, prop, &ret))
			continue;
	};
	power_supply_changed(psy);

	return count;
}
static DEVICE_ATTR(update_now, S_IWUSR, NULL, neo_battery_update_battery_information);

#ifdef CONFIG_NUBIA_POWEROFFCHARGE_NODE
static ssize_t poweroffcharge_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *neo = power_supply_get_by_name("neo-battery");
	struct neo_battery_data *chip = power_supply_get_drvdata(neo);

	pr_info("Show battery poweroffcharge node value:%d\n", chip->poweroffcharge);

	return sprintf(buf, "%d\n", chip->poweroffcharge);
}

static ssize_t poweroffcharge_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct power_supply *neo = power_supply_get_by_name("neo-battery");
	struct neo_battery_data *chip = power_supply_get_drvdata(neo);

	chip->poweroffcharge = 1;
	return count;
}
static DEVICE_ATTR(poweroffcharge, 0644, poweroffcharge_show, poweroffcharge_store);
#endif

static int neo_battery_awake_vote_callback(struct votable *votable,
			void *data, int awake, const char *client)
{
	struct neo_battery_data *chip = data;

	if (awake)
		__pm_stay_awake(chip->debug_lock);
	else
		__pm_relax(chip->debug_lock);

	NEO_DEB(PR_INFO, "awake: %d\n", awake);
	return 0;
}

static int soc_monitor_work_vote_callback(struct votable *votable,
			void *data, int awake, const char *client)
{
	struct neo_battery_data *chip = data;

	pr_err("NEO:re-schedule soc_monitor_work\n");
	cancel_delayed_work(&chip->soc_monitor_work);
	schedule_delayed_work(&chip->soc_monitor_work, msecs_to_jiffies(1000));

	return 0;
}

static void neo_battery_update_psy_changed(struct neo_battery_data *chip)
{
	int i = 0;
	struct power_supply *psy =  chip->neo_psy;
	union power_supply_propval ret;

	/* Go through all properties for the psy */
	for (i = 0; i < psy->desc->num_properties; i++) {
		enum power_supply_property prop;
		prop = psy->desc->properties[i];

		if (power_supply_get_property(psy, prop, &ret))
			continue;
	};
}

static int neo_battery_get_raw_capacity(struct neo_battery_data *chip,
				  union power_supply_propval *val)
{
	int rc = -EINVAL;
#if defined(CONFIG_NEO_EXTERNAL_FG_SUPPORT)
	if (!chip->ex_batt_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->ex_batt_psy,
				POWER_SUPPLY_PROP_CAPACITY, val);
#else

       chip->bms_psy = power_supply_get_by_name("bms");
       if (!chip->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, val);
#endif

	chip->param.batt_raw_soc = val->intval;

	return rc;
}

static int neo_battery_get_batt_health(struct neo_battery_data *chip,
				  union power_supply_propval *val)
{
	int rc = -EINVAL;

	if (!chip->batt_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_HEALTH, val);

	chip->param.batt_health = val->intval;

	return rc;
}

static int neo_battery_get_batt_voltage(struct neo_battery_data *chip,
				  union power_supply_propval *val)
{
	int rc = -EINVAL;

	if (!chip->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, val);

	chip->param.batt_mv = val->intval/1000;

	return rc;
}

static int neo_battery_get_batt_current(struct neo_battery_data *chip,
				  union power_supply_propval *val)
{
	int rc = -EINVAL;

#if defined(CONFIG_NEO_EXTERNAL_FG_SUPPORT)
	if (!chip->ex_batt_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->ex_batt_psy,
				POWER_SUPPLY_PROP_CURRENT_NOW, val);
#else
	if (!chip->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CURRENT_NOW, val);
#endif

	chip->param.batt_ma = val->intval/1000;

	return rc;
}

int neo_battery_get_batt_temp(struct neo_battery_data *chip,
			      union power_supply_propval *val)
{
	int rc;

	if (!chip->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->bms_psy,
				       POWER_SUPPLY_PROP_TEMP, val);

	chip->param.batt_temp = val->intval;

	return rc;
}

int neo_battery_get_batt_status(struct neo_battery_data *chip,
			      union power_supply_propval *val)
{
	int rc;

	if (!chip->batt_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->batt_psy,
				       POWER_SUPPLY_PROP_STATUS, val);

	chip->param.batt_status = val->intval;

	return rc;
}

int neo_battery_get_charge_type(struct neo_battery_data *chip,
			      union power_supply_propval *val)
{
	int rc;

	if (!chip->batt_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->batt_psy,
				       POWER_SUPPLY_PROP_CHARGE_TYPE, val);

	return rc;
}

int neo_battery_get_usb_present(struct neo_battery_data *chip,
			      union power_supply_propval *val)
{
	int rc;

	if (!chip->usb_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->usb_psy,
				       POWER_SUPPLY_PROP_PRESENT, val);
	chip->param.usb_present = val->intval;

	return rc;
}

int neo_battery_get_usb_current_now(struct neo_battery_data *chip,
			      union power_supply_propval *val)
{
	int rc;

	if (!chip->usb_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->usb_psy,
				       POWER_SUPPLY_PROP_INPUT_CURRENT_NOW, val);
	chip->param.ibus_ma= val->intval/1000;

	return rc;
}

int neo_battery_get_usb_voltage_now(struct neo_battery_data *chip,
			      union power_supply_propval *val)
{
	int rc;

	if (!chip->usb_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->usb_psy,
				       POWER_SUPPLY_PROP_VOLTAGE_NOW, val);
	chip->param.vbus_mv= val->intval/1000;

	return rc;
}

int neo_battery_get_batt_charge_full(struct neo_battery_data *chip,
				     union power_supply_propval *val)
{
	int rc;

	if (!chip->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->bms_psy,
				       POWER_SUPPLY_PROP_CHARGE_FULL, val);
	chip->param.batt_full = val->intval;
	return rc;
}

int neo_battery_get_batt_charge_full_design(struct neo_battery_data *chip,
				     union power_supply_propval *val)
{
	int rc;

	if (!chip->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->bms_psy,
				       POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, val);
	chip->param.batt_full_design = val->intval;
	return rc;
}
int neo_battery_get_batt_charge_counter(struct neo_battery_data *chip,
				     union power_supply_propval *val)
{
	int rc;

	if (!chip->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->bms_psy,
				       POWER_SUPPLY_PROP_CHARGE_COUNTER, val);
	chip->param.charge_counter = val->intval;
	return rc;
}

int neo_battery_get_batt_cycle_count(struct neo_battery_data *chip,
				     union power_supply_propval *val)
{
	int rc;

	if (!chip->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chip->bms_psy,
				       POWER_SUPPLY_PROP_CYCLE_COUNT, val);
	chip->param.cycle_count = val->intval;
	return rc;
}

static int neo_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	int rc = 0;
	struct neo_battery_data *chip = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		rc = neo_battery_get_batt_status(chip, val);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		rc = neo_battery_get_usb_present(chip, val);
		val->intval = 1; //report battery present.
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		rc = neo_battery_get_charge_type(chip, val);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		rc = neo_battery_get_batt_health(chip, val);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		rc = neo_battery_get_raw_capacity(chip, val);
		val->intval = chip->param.batt_soc;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		rc = neo_battery_get_batt_current(chip, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = neo_battery_get_batt_voltage(chip, val);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		rc = neo_battery_get_batt_temp(chip, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		rc = neo_battery_get_batt_charge_full(chip, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		rc = neo_battery_get_batt_charge_full_design(chip, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		rc = neo_battery_get_batt_charge_counter(chip, val);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		rc = neo_battery_get_batt_cycle_count(chip, val);
		break;
	default:
		return -EINVAL;
	}

	if (rc < 0) {
		pr_debug("Couldn't get prop %d rc = %d\n", prop, rc);
		return -ENODATA;
	}

	return 0;
}

static void neo_battery_external_power_changed(struct power_supply *psy)
{
	int i = 0;
	union power_supply_propval ret;

	/* Go through all properties for the psy */
	for (i = 0; i < psy->desc->num_properties; i++) {
		enum power_supply_property prop;
		prop = psy->desc->properties[i];

		if (power_supply_get_property(psy, prop, &ret))
			continue;
	};
	power_supply_changed(psy);
}

static struct of_device_id neo_battery_match_table[] = {
	{
		.compatible     = "nubia,neo-battery",
	},
	{ },
};

static int neo_battery_parse_dt(struct neo_battery_data *chip)
{
	int rc = 0;
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(node,
				"neo,batt-capacity-uah", &chip->dt.batt_capacity_uah);
	if (rc < 0)
		chip->dt.batt_capacity_uah = -EINVAL;

	rc = of_property_read_u32(node,
				"neo,batt-cutoff-mv", &chip->dt.batt_cutoff_mv);
	if (rc < 0)
		chip->dt.batt_cutoff_mv = -EINVAL;

	rc = of_property_read_u32(node,
				"neo,batt-soc-increase-per-sec", &chip->dt.batt_soc_increase_per_sec);
	if (rc < 0)
		chip->dt.batt_soc_increase_per_sec = -EINVAL;

	rc = of_property_read_u32(node,
				"neo,batt-soc-decrease-per-sec", &chip->dt.batt_soc_decrease_per_sec);
	if (rc < 0)
		chip->dt.batt_soc_decrease_per_sec = -EINVAL;
#ifdef CONFIG_NUBIA_POWEROFFCHARGE_NODE
	chip->poweroffcharge = 0;
#endif
	return 0;
}

static int create_debugfs_entries(struct neo_battery_data *chip)
{
	int rc;

	chip->debug_root = debugfs_create_dir("neo-battery", NULL);
	if (!chip->debug_root) {
		dev_err(chip->dev, "Couldn't create debug dir\n");
		return -EINVAL;
	}

	if(chip->batt_psy) {
		rc = device_create_file(&chip->batt_psy->dev, &dev_attr_charge_full_design);
		if(rc)
			device_remove_file(&chip->batt_psy->dev, &dev_attr_charge_full_design);
		rc = device_create_file(&chip->batt_psy->dev, &dev_attr_update_now);
		if(rc)
			device_remove_file(&chip->batt_psy->dev, &dev_attr_update_now);
#ifdef CONFIG_NUBIA_POWEROFFCHARGE_NODE
		rc = device_create_file(&chip->batt_psy->dev, &dev_attr_poweroffcharge);
		if (rc)
			device_remove_file(&chip->batt_psy->dev, &dev_attr_poweroffcharge);
#endif
	}

	return 0;
}

static int calculate_delta_time(struct timespec *time_stamp, int *delta_time_s)
{
	struct timespec now_time;

	/* default to delta time = 0 if anything fails */
	*delta_time_s = 0;

	get_monotonic_boottime(&now_time);

	*delta_time_s = (now_time.tv_sec - time_stamp->tv_sec);

	/* remember this time */
	*time_stamp = now_time;
	return 0;
}

static void neo_battery_soc_smooth_tracking(struct neo_battery_data *chip)
{
	int delta_time = 0;
    int soc_changed;
	int last_batt_soc = chip->param.batt_soc;
    int time_since_last_change_sec;

    struct timespec last_change_time = chip->last_soc_change_time;

	if( chip->param.usb_present
			&& last_batt_soc == 100
			&& chip->param.batt_raw_soc >= 98
			&& chip->param.batt_ma >= 0 )
	{
		chip->param.batt_raw_soc = 100;
		NEO_DEB(PR_INFO, "wait for recharging, set soc to 100.\n");
	}

    /** Power-off Issue */
    if(chip->param.batt_mv > chip->dt.batt_cutoff_mv
		&& chip->param.batt_raw_soc == 0
		&& !chip->param.usb_present)
    {
        chip->param.batt_raw_soc = 1;
        NEO_DEB(PR_INFO, "battery voltage is above cutoff voltage, set soc to 1.\n");
    }

	calculate_delta_time(&last_change_time, &time_since_last_change_sec);

	if (chip->param.batt_ma < 0){
		delta_time = time_since_last_change_sec/ chip->dt.batt_soc_increase_per_sec;
	}
	else {
		delta_time = time_since_last_change_sec/ chip->dt.batt_soc_decrease_per_sec;
	}

	if(delta_time < 0) delta_time = 0;

    soc_changed = min(1, delta_time);

    if(last_batt_soc >= 0) {
        /** Battery in charging status */
        if( last_batt_soc < chip->param.batt_raw_soc
					&& chip->param.batt_ma < 0) {
            /** Update the soc when resuming device */
            last_batt_soc = chip->update_now ? chip->param.batt_raw_soc : last_batt_soc + soc_changed;
		}
        /** Battery in discharing status */
        else if( last_batt_soc > chip->param.batt_raw_soc
					&& chip->param.batt_ma > 0) {
            /** Update the soc when resuming device */
            last_batt_soc = chip->update_now ? chip->param.batt_raw_soc : last_batt_soc - soc_changed;
		}
        /** Unlikely status */
        else if( last_batt_soc != 100
					&& chip->param.batt_raw_soc >= 95
					&& chip->param.batt_status == POWER_SUPPLY_STATUS_FULL) {
            last_batt_soc = chip->update_now ? 100 : last_batt_soc + soc_changed;
        } else if ( last_batt_soc > chip->param.batt_raw_soc
					&& chip->param.batt_ma < 0) {
            /** Update the soc when resuming device */
            last_batt_soc = chip->update_now ? chip->param.batt_raw_soc : last_batt_soc - soc_changed;
		}
		chip->update_now = false;
    }
	else {
		last_batt_soc = chip->param.batt_raw_soc;
	}

	if(chip->param.batt_soc != last_batt_soc) {
		chip->param.batt_soc = last_batt_soc;
		chip->last_soc_change_time = last_change_time;
		power_supply_changed(chip->neo_psy);
	}

	NEO_DEB(PR_STATUS,"v:%d, soc:%d, last_soc:%d, raw_soc:%d, soc_changed:%d.\n", chip->param.batt_mv, chip->param.batt_soc, last_batt_soc, chip->param.batt_raw_soc, soc_changed);
}

#define MONITOR_SOC_WAIT_MS					1000
#define MONITOR_SOC_WAIT_PER_MS             10000
static void soc_monitor_work(struct work_struct *work)
{
    struct delayed_work *dwork = to_delayed_work(work);
    struct neo_battery_data *chip = container_of(dwork,
                                   struct neo_battery_data, soc_monitor_work);
    union power_supply_propval ret;
    /** Update battery informations */
	neo_battery_update_psy_changed(chip);

    //if(wake_lock_active(&chip->monitor_wake_lock) && chip->param.batt_mv > chip->dt.batt_cutoff_mv)
        //wake_unlock(&chip->monitor_wake_lock);

	spmi_dump_regs(chip);
	neo_battery_soc_smooth_tracking(chip);
	neo_battery_get_usb_voltage_now(chip,&ret);
	neo_battery_get_usb_current_now(chip,&ret);

    NEO_DEB(PR_INFO, "soc:%d, raw_soc:%d, v:%d,t:%d,u:%d,c:%d, vbus:%d,ibus:%d,s:%d,h:%d\n",
                    chip->param.batt_soc, chip->param.batt_raw_soc, chip->param.batt_mv, chip->param.batt_temp, 
                    chip->param.usb_present, chip->param.batt_ma, chip->param.vbus_mv,chip->param.ibus_ma,chip->param.batt_status,chip->param.batt_health);

	schedule_delayed_work(&chip->soc_monitor_work, msecs_to_jiffies(MONITOR_SOC_WAIT_PER_MS));
}

static const struct power_supply_desc neo_psy_desc = {
	.name = "neo-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = neo_battery_properties,
	.num_properties = ARRAY_SIZE(neo_battery_properties),
	.get_property = neo_battery_get_property,
	.external_power_changed = neo_battery_external_power_changed,
};

static char *neo_supplicants[] = {
	"battery"
};
static int neo_battery_init_power_supply(struct neo_battery_data *chip)
{
	struct power_supply_config neo_cfg = {};
	int rc = 0;

	chip->batt_psy = power_supply_get_by_name("battery");
	if (!chip->batt_psy){
		pr_err("Couldn't get battery power supply.\n");
		rc = -EPROBE_DEFER;
		return rc;
	}

	chip->bms_psy = power_supply_get_by_name("bms");
	if (!chip->bms_psy){
		pr_err("Couldn't get bms power supply.\n");
		rc = -EPROBE_DEFER;
		return rc;
	}

	chip->usb_psy = power_supply_get_by_name("usb");
	if (!chip->usb_psy){
		pr_err("Couldn't get usb power supply.\n");
		rc = -EPROBE_DEFER;
		return rc;
	}

#if defined(CONFIG_NEO_EXTERNAL_FG_SUPPORT)
#if defined(CONFIG_NEO_BQ27520_BATTERY)
	chip->ex_batt_psy = power_supply_get_by_name("bq-battery");
#else
	chip->ex_batt_psy = power_supply_get_by_name("battery");
#endif
	if (!chip->ex_batt_psy){
		pr_err("Couldn't get external battery power supply.\n");
		rc = -EPROBE_DEFER;
		return rc;
	}
#endif

	neo_cfg.supplied_to = neo_supplicants;
	neo_cfg.num_supplicants = ARRAY_SIZE(neo_supplicants);
	neo_cfg.drv_data = chip;
	neo_cfg.of_node = chip->dev->of_node;
	chip->neo_psy = devm_power_supply_register(chip->dev,
						   &neo_psy_desc,
						   &neo_cfg);
	if (IS_ERR(chip->neo_psy)) {
		pr_err("Couldn't register battery power supply\n");
		return PTR_ERR(chip->neo_psy);
	}

	return rc;
}

static int neo_battery_probe(struct platform_device *pdev)
{
	int rc;
	struct neo_battery_data *chip;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->pdev = pdev;
	chip->dev = &pdev->dev;
	chip->regmap = dev_get_regmap(chip->dev->parent, NULL);
	if (!chip->regmap) {
		pr_err("parent regmap is missing\n");
		return -EINVAL;
	}

	neo_battery_parse_dt(chip);

	dev_set_drvdata(&pdev->dev, chip);
	INIT_DELAYED_WORK(&chip->soc_monitor_work, soc_monitor_work);
	rc = neo_battery_init_power_supply(chip);
	if (rc < 0) {
		pr_err("Couldn't initialize batt psy rc=%d\n", rc);
		goto cleanup;
	}

	chip->debug_lock = wakeup_source_register("neo-battery");
	if (!chip->debug_lock)
		goto cleanup;

	chip->debug_awake_votable = create_votable("DEBUG_AWAKE", VOTE_SET_ANY,
					neo_battery_awake_vote_callback, chip);
	if (IS_ERR(chip->debug_awake_votable)) {
		rc = PTR_ERR(chip->debug_awake_votable);
		goto cleanup;
	}

	chip->soc_monitor_work_votable = create_votable("SOC_MONITOR", VOTE_SET_ANY,
					soc_monitor_work_vote_callback, chip);
	if (IS_ERR(chip->soc_monitor_work_votable)) {
		rc = PTR_ERR(chip->soc_monitor_work_votable);
		destroy_votable(chip->soc_monitor_work_votable);
		pr_err("NEO:can't create soc_monitor_work_votable\n");
	}

	create_debugfs_entries(chip);
	/** Init battery soc */
	chip->param.batt_soc = -EINVAL;

    //wake_lock_init(&chip->monitor_wake_lock, WAKE_LOCK_SUSPEND,	"soc_monitor_lock");
	schedule_delayed_work(&chip->soc_monitor_work, 0);

	dev_err(chip->dev,"Neo battery probe successful.\n");

	return 0;

cleanup:
	if (chip->neo_psy)
		power_supply_unregister(chip->neo_psy);
	platform_set_drvdata(pdev, NULL);
	wakeup_source_unregister(chip->debug_lock);
	return rc;
}

static int neo_battery_remove(struct platform_device *pdev)
{
	//struct neo_battery_data *chip = dev_get_drvdata(&pdev->dev);
	return 0;
}

static void neo_battery_shutdown(struct platform_device *pdev)
{
	//int rc;
	//struct neo_battery_data *chip = dev_get_drvdata(&pdev->dev);

}

static int neo_battery_suspend(struct device *dev)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct neo_battery_data *chip = platform_get_drvdata(pdev);

    cancel_delayed_work(&chip->soc_monitor_work);

    return 0;
}

static int neo_battery_resume(struct device *dev)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct neo_battery_data *chip = platform_get_drvdata(pdev);

	chip->update_now = true;

    schedule_delayed_work(&chip->soc_monitor_work, msecs_to_jiffies(MONITOR_SOC_WAIT_MS));

    return 0;
}

static const struct dev_pm_ops neo_battery_pm_ops = {
	.suspend 	= neo_battery_suspend,
	.resume 	= neo_battery_resume,
};

MODULE_DEVICE_TABLE(spmi, neo_battery_id);

static struct platform_driver neo_battery_driver = {
	.driver		= {
		.name		= "neo-battery",
		.owner		= THIS_MODULE,
		.of_match_table	= neo_battery_match_table,
		.pm		= &neo_battery_pm_ops,
	},
	.probe		= neo_battery_probe,
	.remove		= neo_battery_remove,
	.shutdown	= neo_battery_shutdown,
};

static int __init neo_battery_init(void)
{
	return platform_driver_register(&neo_battery_driver);
}

static void __exit neo_battery_exit(void)
{
	return platform_driver_unregister(&neo_battery_driver);
}

module_init(neo_battery_init);
module_exit(neo_battery_exit);

MODULE_DESCRIPTION("NEO Battery Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:neo battery");
