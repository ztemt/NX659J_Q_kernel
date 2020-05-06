/* Copyright (c) 2017-2019 The Linux Foundation. All rights reserved.
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


#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include "pca9468-step-chg.h"

#define MAX_STEP_CHG_ENTRIES	16

#define is_between(left, right, value) \
		(((left) >= (right) && (left) >= (value) \
			&& (value) >= (right)) \
		|| ((left) <= (right) && (left) <= (value) \
			&& (value) <= (right)))

struct range_data {
	u32 low_threshold;
	u32 high_threshold;
	u32 value;
};

struct step_chg_cfg {
	u32			psy_prop;
	char		*prop_name;
	int			counter;
	struct range_data	fcc_cfg[MAX_STEP_CHG_ENTRIES];
};

struct jeita_fcc_cfg {
	u32			psy_prop;
	char		*prop_name;
	int			hysteresis;
	struct range_data	fcc_cfg[MAX_STEP_CHG_ENTRIES];
};

struct jeita_fv_cfg {
	u32			psy_prop;
	char		*prop_name;
	int			hysteresis;
	struct range_data	fv_cfg[MAX_STEP_CHG_ENTRIES];
};


struct pca9468_step_chg_info {
	struct device		*dev;
	bool			config_is_read;
	bool			step_chg_cfg_valid;
	bool			sw_jeita_cfg_valid;
	bool 			lcd_on_jeita_limit;
	bool 			lcd_on_step_chg_limit;
	bool			lcd_on_enabled_step;
	bool			lcd_off_reset;
	int			jeita_fcc_index;
	int			jeita_fv_index;
	int			step_index;
	int			get_config_retry_count;
	
	int 		lcd_on_limit_enable;
	int			lcd_on_limit_temp;
	int 		lcd_on_limit_fcc;
	int 		lcd_on;	

	struct step_chg_cfg	*step_chg_config;
	struct jeita_fcc_cfg	*jeita_fcc_config;
	struct jeita_fv_cfg	*jeita_fv_config;

	struct wakeup_source	*step_chg_ws;
	struct power_supply	*batt_psy;
	struct power_supply *dcchg_psy;
	struct delayed_work	status_change_work;
	struct delayed_work	get_config_work;
	struct notifier_block	nb;
};

static struct pca9468_step_chg_info *the_chip;

#define STATUS_UPDATE_TIME		10000000	/* 10s */
#define JEITA_ERROR_RETRY_TIME	2000000		/* 2s */
#define STEP_ERROR_RETRY_TIME	5000000		/* 5s */
#define GET_CONFIG_DELAY_MS		2000		/* 2s */
#define GET_CONFIG_RETRY_COUNT	10
#define LCD_ON_CURRENT_CHARGE   1200000     /*2.4A*/

#define IIN_TOPOFF_DFT		750000 /* 750mA */
/* JEITA stage */
enum {
	JEITA_STAGE0,	/* 0~10.0degree - stop charging*/
	JEITA_STAGE1,	/* 10.0~15.0degree */
	JEITA_STAGE2,	/* 15.0~41.5degree - step charging */
	JEITA_STAGE3,	/* 41.5~42.5degree */
	JEITA_STAGE4,	/* 42.5~44.0degree */
	JEITA_STAGE5,	/* 44.0~53.0degree */
	JEITA_STAGE6,	/* 53.0~ 100degree  - stop charging */
	JEITA_STAGE_MAX,
};

/* Charging step */
enum {
	DC_STEP0,	/* 0% ~ 28% */
	DC_STEP1,	/* 29% ~ 50% */
	DC_STEP2,	/* 51% ~ 75% */
	DC_STEP3,	/* 76% ~ 79% */
	DC_STEP4,	/* 80% ~ 82% */
	DC_STEP5,	/* 83% ~ 85% */
	DC_STEP6,	/* 86% ~ 88% */
	DC_STEP7,	/* 89% ~ 91% */
	DC_STEP8,	/* 92% ~ 93% */
	DC_STEP9,	/* 94% ~ 95% */
	DC_STEP10,	/* 96% ~ 100% */
	DC_STEP_MAX,
};

static bool is_batt_available(struct pca9468_step_chg_info *chip)
{
	if (!chip->batt_psy)
		chip->batt_psy = power_supply_get_by_name("battery");

	if (!chip->batt_psy)
		return false;

	return true;
}

static bool is_dcchg_available(struct pca9468_step_chg_info *chip)
{
	if (!chip->dcchg_psy)
		chip->dcchg_psy = power_supply_get_by_name("pca9468-mains");

	if (!chip->dcchg_psy)
		return false;

	return true;
}

static int read_range_data_from_node(struct device_node *node,
		const char *prop_str, struct range_data *ranges)
{
	int rc = 0, i, length, per_tuple_length, tuples;
	
	pr_info("pca9468_%s: ========Start========\n", __func__);

	rc = of_property_count_elems_of_size(node, prop_str, sizeof(u32));
	if (rc < 0) {
		pr_err("Count %s failed, rc=%d\n", prop_str, rc);
		return rc;
	}

	length = rc;
	per_tuple_length = sizeof(struct range_data) / sizeof(u32);
	if (length % per_tuple_length) {
		pr_err("%s length (%d) should be multiple of %d\n",
				prop_str, length, per_tuple_length);
		return -EINVAL;
	}
	tuples = length / per_tuple_length;

	if (tuples > MAX_STEP_CHG_ENTRIES) {
		pr_err("too many entries(%d), only %d allowed\n",
				tuples, MAX_STEP_CHG_ENTRIES);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(node, prop_str,
			(u32 *)ranges, length);
	if (rc) {
		pr_err("Read %s failed, rc=%d", prop_str, rc);
		return rc;
	}

	for (i = 0; i < tuples; i++) {
		if (ranges[i].low_threshold >
				ranges[i].high_threshold) {
			pr_err("%s thresholds should be in ascendant ranges\n",
						prop_str);
			rc = -EINVAL;
			goto clean;
		}

		if (i != 0) {
			if (ranges[i - 1].high_threshold >
					ranges[i].low_threshold) {
				pr_err("%s thresholds should be in ascendant ranges\n",
							prop_str);
				rc = -EINVAL;
				goto clean;
			}
		}
	}

	return rc;
clean:
	memset(ranges, 0, tuples * sizeof(struct range_data));
	return rc;
}


static int get_step_chg_jeita_setting_from_profile(struct pca9468_step_chg_info *chip)
{
	struct device_node *profile_node = chip->dev->of_node;
	int rc;

	pr_info("pca9468_%s: ========Start========\n", __func__);

	chip->step_chg_cfg_valid = true;
	rc = read_range_data_from_node(profile_node,
			"pca9468,step-chg-ranges",
			chip->step_chg_config->fcc_cfg);
	if (rc < 0) {
		pr_err("Read pca9468,step-chg-ranges failed from battery profile, rc=%d\n",
					rc);
		chip->step_chg_cfg_valid = false;
	}

	chip->sw_jeita_cfg_valid = true;
	rc = read_range_data_from_node(profile_node,
			"pca9468,jeita-fcc-ranges",
			chip->jeita_fcc_config->fcc_cfg);
	if (rc < 0) {
		pr_err("Read pca9468,jeita-fcc-ranges failed from battery profile, rc=%d\n",
					rc);
		chip->sw_jeita_cfg_valid = false;
	}

	rc = read_range_data_from_node(profile_node,
			"pca9468,jeita-fv-ranges",
			chip->jeita_fv_config->fv_cfg);
	if (rc < 0) {
		pr_err("Read pca9468,jeita-fv-ranges failed from battery profile, rc=%d\n",
					rc);
		chip->sw_jeita_cfg_valid = false;
	}

	chip->lcd_on_limit_enable= of_property_read_bool(profile_node,
				"pca9468,lcd-on-limit-enable");
	
	chip->lcd_on_jeita_limit = true;
	chip->lcd_on_step_chg_limit = true;
	chip->lcd_off_reset = false;
	chip->lcd_on_enabled_step = true;		
	if(chip->lcd_on_limit_enable){		
		rc = of_property_read_u32(profile_node,
				"pca9468,lcd-on-limit-temp", &chip->lcd_on_limit_temp);
		rc = of_property_read_u32(profile_node,
				"pca9468,lcd-on-limit-fcc", &chip->lcd_on_limit_fcc);
		chip->lcd_on = 0;
	}

	return rc;
}

static void pca9468_get_config_work(struct work_struct *work)
{
	struct pca9468_step_chg_info *chip = container_of(work,
			struct pca9468_step_chg_info, get_config_work.work);
	int i, rc;

	pr_info("%s: ========Start========\n", __func__);

	chip->config_is_read = false;
	rc = get_step_chg_jeita_setting_from_profile(chip);

	if (rc < 0) {
		if (rc == -ENODEV || rc == -EBUSY) {
			if (chip->get_config_retry_count++
					< GET_CONFIG_RETRY_COUNT) {
				pr_info("pca9468 psy is not ready, retry: %d\n",
						chip->get_config_retry_count);
				goto reschedule;
			}
		}
	}

	chip->config_is_read = true;

	for (i = 0; i < MAX_STEP_CHG_ENTRIES; i++)
		pr_info("pca9468, step-chg-cfg: %d%% ~ %d%%, %duA\n",
			chip->step_chg_config->fcc_cfg[i].low_threshold,
			chip->step_chg_config->fcc_cfg[i].high_threshold,
			chip->step_chg_config->fcc_cfg[i].value);
	for (i = 0; i < MAX_STEP_CHG_ENTRIES; i++)
		pr_info("pca9468, jeita-fcc-cfg: %ddecidegree ~ %ddecidegre, %duA\n",
			chip->jeita_fcc_config->fcc_cfg[i].low_threshold,
			chip->jeita_fcc_config->fcc_cfg[i].high_threshold,
			chip->jeita_fcc_config->fcc_cfg[i].value);
	for (i = 0; i < MAX_STEP_CHG_ENTRIES; i++)
		pr_info("pca9468, jeita-fv-cfg: %ddecidegree ~ %ddecidegre, %duV\n",
			chip->jeita_fv_config->fv_cfg[i].low_threshold,
			chip->jeita_fv_config->fv_cfg[i].high_threshold,
			chip->jeita_fv_config->fv_cfg[i].value);

	return;

reschedule:
	schedule_delayed_work(&chip->get_config_work,
			msecs_to_jiffies(GET_CONFIG_DELAY_MS));

}

static int get_val(struct range_data *range, int hysteresis, int current_index,
		int threshold,
		int *new_index, int *val)
{
	int i;

	*new_index = -EINVAL;
	/* first find the matching index without hysteresis */
	for (i = 0; i < MAX_STEP_CHG_ENTRIES; i++) {
		if (is_between(range[i].low_threshold,
			range[i].high_threshold, threshold)) {
			*new_index = i;
			*val = range[i].value;
			break;
		}
	}

	/* if nothing was found, return -ENODATA */
	if (*new_index == -EINVAL)
		return -ENODATA;
	/*
	 * If we don't have a current_index return this
	 * newfound value. There is no hysterisis from out of range
	 * to in range transition
	 */
	if (current_index == -EINVAL)
		return 0;

	/*
	 * Check for hysteresis if it in the neighbourhood
	 * of our current index.
	 */
	if (*new_index == current_index + 1) {
		if (threshold < range[*new_index].low_threshold + hysteresis) {
			/*
			 * Stay in the current index, threshold is not higher
			 * by hysteresis amount
			 */
			*new_index = current_index;
			*val = range[current_index].value;
		}
	} else if (*new_index == current_index - 1) {
		if (threshold > range[*new_index].high_threshold - hysteresis) {
			/*
			 * stay in the current index, threshold is not lower
			 * by hysteresis amount
			 */
			*new_index = current_index;
			*val = range[current_index].value;
		}
	} else if (*new_index > current_index + 1) {
		/* 
		 * new index is already higher than hysteresis 
		 * increase one step
		 */
		*new_index = current_index + 1;
	} else if (*new_index < current_index - 1) {	
		/* 
		 * new index is already lower than hysteresis 
		 * decrease one step
		 */
		*new_index = current_index - 1;
	}
	return 0;
}

static int get_step_val(struct range_data *range, int *counter, int current_index,
		int threshold,
		int *new_index, int *val)
{
	int i;

	*new_index = -EINVAL;
	/* first find the matching index without hysteresis */
	for (i = 0; i < MAX_STEP_CHG_ENTRIES; i++) {
		if (is_between(range[i].low_threshold,
			range[i].high_threshold, threshold)) {
			*new_index = i;
			*val = range[i].value;
			break;
		}
	}

	/* if nothing was found, return -ENODATA */
	if (*new_index == -EINVAL)
		return -ENODATA;
	/*
	 * If we don't have a current_index return this
	 * newfound value. There is no counter in range
	 */
	if (current_index == -EINVAL)
		return 0;

	return 0;
}



static int pca9468_handle_jeita(struct pca9468_step_chg_info *chip)
{
	union power_supply_propval pval = {0, };	
	union power_supply_propval lcd_pval = {0, };
	int rc = 0, fcc_ua = 0, fv_uv = 0;
	int current_index;

	pr_info("%s: ========Start========\n", __func__);

	/* Get the temperature information */
	rc = power_supply_get_property(chip->batt_psy,
			chip->jeita_fcc_config->psy_prop, &pval);
	if (rc < 0) {
		pr_err("%s: Couldn't read %s property rc=%d\n",
				__func__, chip->jeita_fcc_config->prop_name, rc);
		return rc;
	}
	
	rc = power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_LCD_ON, &lcd_pval);
	if (rc < 0) {
		pr_err("%s: Couldn't read POWER_SUPPLY_PROP_LCD_ON property rc=%d\n",
				__func__, rc);
		return rc;
	}

	pr_info("%s: battery_temp=%d\n", __func__, pval.intval);
	if (pval.intval < 0) {
		/* minimum value is 0 */
		pval.intval = 0;
	} else if (pval.intval > 1000) {
		/* maximum value is 1000 */
		pval.intval = 1000;
	}
	
	current_index = chip->jeita_fcc_index;
	rc = get_val(chip->jeita_fcc_config->fcc_cfg,
			chip->jeita_fcc_config->hysteresis,
			chip->jeita_fcc_index,
			pval.intval,
			&chip->jeita_fcc_index,
			&fcc_ua);
	if (rc < 0) {
		pr_err("%s: There is no jeita step-based fcc, rc=%d\n",
				__func__, rc);
		return rc;
	}

	/* fv_index is same as fcc_index */
	chip->jeita_fv_index = chip->jeita_fcc_index;
	fv_uv = chip->jeita_fv_config->fv_cfg[chip->jeita_fv_index].value;

	pr_info("%s: jeita_index=%d, fcc_ua=%d, fv_uv=%d, fg_temp=%d\n", 
			__func__, chip->jeita_fcc_index, fcc_ua, fv_uv, pval.intval);

	/* Compare the current index with the previous index */
	if ((lcd_pval.intval !=1 )&&(current_index == chip->jeita_fcc_index)) {
		if(chip->lcd_off_reset == true){
			chip->lcd_on_jeita_limit = true;
			chip->lcd_off_reset =false;
			if ((chip->jeita_fcc_index != JEITA_STAGE0)&&
				(chip->jeita_fcc_index != JEITA_STAGE6)&&
				(chip->jeita_fcc_index != JEITA_STAGE2)) {
				if (!is_dcchg_available(chip)){
					pr_err("%s: Couldn't get pca9468-mains\n", __func__);
					return -ENODEV;
				}
				/* Set dc input current to fcc_ua */
				pval.intval = fcc_ua;
				rc = power_supply_set_property(chip->dcchg_psy,
						POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &pval);
				if (rc < 0) {
					pr_err("%s: Couldn't set input current, rc=%d\n", __func__, rc);
					return rc;
				}
				/* Set dc vfloat voltage to fv_uv */
				pval.intval = fv_uv;
				rc = power_supply_set_property(chip->dcchg_psy,
						POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE, &pval);
				if (rc < 0) {
					pr_err("%s: Couldn't set vfloat voltage, rc=%d\n", __func__, rc);
					return rc;
				}
				chip->step_index = -EINVAL;
			}			
		}
		/* We don't need to set charging parameters */
		/* It is same as previous stage */
		return rc;
	}
	if ((chip->jeita_fcc_index == JEITA_STAGE0) ||
		(chip->jeita_fcc_index == JEITA_STAGE6)) {
		/* Turn off direct charging */
		/* Set disable charging */
		pval.intval = 0;
		rc = power_supply_set_property(chip->dcchg_psy,
				POWER_SUPPLY_PROP_CHARGING_ENABLED, &pval);
		if (rc < 0) {
			pr_err("%s: Couldn't set disable charging, rc=%d\n", __func__, rc);
			return rc;
		}
		return rc;
	} else {
		if (chip->jeita_fcc_index != JEITA_STAGE2) {
			if((chip->jeita_fcc_index != JEITA_STAGE5)&&(lcd_pval.intval == 1)
				&&(pval.intval > chip->lcd_on_limit_temp)){
				if(chip->lcd_on_jeita_limit == true){
					pr_info("====>>>%s: enter lcd_on limit charge<<<<====\n", __func__);
					chip->lcd_on_jeita_limit = false;
					chip->lcd_off_reset = true;
					/* There is no step charging */
					if (!is_dcchg_available(chip)) {
						pr_err("%s: Couldn't get pca9468-mains\n", __func__);
						return -ENODEV;
					}
					/* Set dc input current to fcc_ua */
					pval.intval = LCD_ON_CURRENT_CHARGE;
					rc = power_supply_set_property(chip->dcchg_psy,
							POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &pval);
					if (rc < 0) {
						pr_err("%s: Couldn't set input current, rc=%d\n", __func__, rc);
						return rc;
					}
					/* Set dc vfloat voltage to fv_uv */
					pval.intval = fv_uv;
					rc = power_supply_set_property(chip->dcchg_psy,
							POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE, &pval);
					if (rc < 0) {
						pr_err("%s: Couldn't set vfloat voltage, rc=%d\n", __func__, rc);
						return rc;
					}

					/* Need to clear step_index */
					chip->step_index = -EINVAL;
				}				
			}else{
				pr_info("====>>>%s: remove lcd_on limit charge<<<<====\n", __func__);
				chip->lcd_on_jeita_limit = true;
				/* There is no step charging */
				if (!is_dcchg_available(chip)) {
					pr_err("%s: Couldn't get pca9468-mains\n", __func__);
					return -ENODEV;
				}
				/* Set dc input current to fcc_ua */
				pval.intval = fcc_ua;
				rc = power_supply_set_property(chip->dcchg_psy,
						POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &pval);
				if (rc < 0) {
					pr_err("%s: Couldn't set input current, rc=%d\n", __func__, rc);
					return rc;
				}
				/* Set dc vfloat voltage to fv_uv */
				pval.intval = fv_uv;
				rc = power_supply_set_property(chip->dcchg_psy,
						POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE, &pval);
				if (rc < 0) {
					pr_err("%s: Couldn't set vfloat voltage, rc=%d\n", __func__, rc);
					return rc;
				}

				/* Need to clear step_index */
				chip->step_index = -EINVAL;
			}			
		}
		
		/* Check the preivous stage */
		if ((current_index == JEITA_STAGE0) ||
			(current_index == JEITA_STAGE6)) {
			/* Turn on the direct charging */
			pval.intval = 1;
			rc = power_supply_set_property(chip->dcchg_psy,
					POWER_SUPPLY_PROP_CHARGING_ENABLED, &pval);
			if (rc < 0) {
				pr_err("%s: Couldn't set enable charging, rc=%d\n", __func__, rc);
				return rc;
			}
		}
	}

	return 0;
}

static int pca9468_handle_step_chg_config(struct pca9468_step_chg_info *chip)
{
	union power_supply_propval pval = {0, };
	union power_supply_propval temp_pval = {0, };
	union power_supply_propval lcd_pval = {0, };
	int rc = 0, fcc_ua = 0, fv_uv=0;
	int current_index, current_jeita_index;

	pr_info("%s: ========Start========\n", __func__);

	current_index = chip->step_index;
	current_jeita_index = chip->jeita_fcc_index;

	/* Check charging done first */
	rc = power_supply_get_property(chip->dcchg_psy,
				POWER_SUPPLY_PROP_CHARGE_DONE, &pval);
	if (rc < 0) {
		pr_err("Couldn't read %s property rc=%d\n",
			chip->dcchg_psy->desc->name, rc);
		return rc;
	}

	rc = power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_LCD_ON, &lcd_pval);
	if (rc < 0) {
		pr_err("%s: Couldn't read POWER_SUPPLY_PROP_LCD_ON property rc=%d\n",
				__func__, rc);
		return rc;
	}

	/* Get the temperature information */
	rc = power_supply_get_property(chip->batt_psy,
			chip->jeita_fcc_config->psy_prop, &temp_pval);
	if (rc < 0) {
		pr_err("%s: Couldn't read %s property rc=%d\n",
				__func__, chip->jeita_fcc_config->prop_name, rc);
		return rc;
	}	

	pr_info("%s: battery_temp=%d\n", __func__, temp_pval.intval);
	
	if (pval.intval == 1) {
		pr_info("%s: charging done happens from dc\n", __func__);
		/* Check the current step */
		if (current_index == DC_STEP10) {
			/* Step 4 is the final step */
			/* Set the charging done state */
			pr_info("%s: step4 - final charging done\n", __func__);
			rc = power_supply_set_property(chip->dcchg_psy,
					POWER_SUPPLY_PROP_CHARGE_DONE, &pval);
			if (rc < 0) {
				pr_err("Couldn't read %s property rc=%d\n",
					chip->dcchg_psy->desc->name, rc);
				return rc;
			}
		} else {
			pr_err("%s: abnormal charging_done - cannot enter here\n", __func__);
		}
	} else {
	
		/* Read battery soc */
		rc = power_supply_get_property(chip->batt_psy,
				chip->step_chg_config->psy_prop, &pval);
		if (rc < 0) {
			pr_err("Couldn't read %s property rc=%d\n",
				chip->step_chg_config->prop_name, rc);
			return rc;
		}
		/* Find the current step */
		rc = get_step_val(chip->step_chg_config->fcc_cfg,
							&chip->step_chg_config->counter,
							chip->step_index,
							pval.intval,
							&chip->step_index,
							&fcc_ua);
		if (rc < 0) {
			pr_err("%s: There is no step-based fcc, rc=%d\n",
					__func__, rc);
			return rc;
		}

		/* Set fv_uv */
		fv_uv = chip->jeita_fv_config->fv_cfg[current_jeita_index].value;

		pr_info("%s: step_index=%d, fcc_ua=%duA, fv_uv=%duV, fg_soc=%d%%\n", 
				__func__, chip->step_index, fcc_ua, fv_uv, pval.intval);
		
		if((lcd_pval.intval ==1)&&(temp_pval.intval > chip->lcd_on_limit_temp)){			
			if(chip->lcd_on_step_chg_limit== true){				
				chip->lcd_on_step_chg_limit = false;
				chip->lcd_on_enabled_step = false;
				pr_info("====>>>%s: enter lcd_on limit charge<<<<====\n", __func__);
				/* Need to set new input current limit and vfloat */
				pr_info("%s: current step=%d\n", __func__, chip->step_index);
				if (!is_dcchg_available(chip)) {
					pr_err("%s: Couldn't get pca9468-mains\n", __func__);
					return -ENODEV;
				}
				/* Set dc input current to fcc_ua */
				pval.intval = LCD_ON_CURRENT_CHARGE;
				rc = power_supply_set_property(chip->dcchg_psy,
						POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &pval);
				if (rc < 0) {
					pr_err("%s: Couldn't set input current, rc=%d\n", __func__, rc);
					return rc;
				}
				/* Set dc vfloat voltage to fv_uv */
				pval.intval = fv_uv;
				rc = power_supply_set_property(chip->dcchg_psy,
						POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE, &pval);
				if (rc < 0) {
					pr_err("%s: Couldn't set vfloat voltage, rc=%d\n", __func__, rc);
					return rc;
				}
			}			
		}else{				
			chip->lcd_on_step_chg_limit = true;
			pr_info("====>>>%s: remove lcd_on limit charge<<<<====\n", __func__);
			if ((chip->lcd_on_enabled_step == false)||(current_index != chip->step_index)) {
				chip->lcd_on_enabled_step = true;				
				/* step change happens */
				/* Need to set new input current limit and vfloat */
				pr_info("%s: current step=%d\n", __func__, chip->step_index);
				if (!is_dcchg_available(chip)) {
					pr_err("%s: Couldn't get pca9468-mains\n", __func__);
					return -ENODEV;
				}
				/* Set dc input current to fcc_ua */
				pval.intval = fcc_ua;
				rc = power_supply_set_property(chip->dcchg_psy,
						POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &pval);
				if (rc < 0) {
					pr_err("%s: Couldn't set input current, rc=%d\n", __func__, rc);
					return rc;
				}
				/* Set dc vfloat voltage to fv_uv */
				pval.intval = fv_uv;
				rc = power_supply_set_property(chip->dcchg_psy,
						POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE, &pval);
				if (rc < 0) {
					pr_err("%s: Couldn't set vfloat voltage, rc=%d\n", __func__, rc);
					return rc;
				}
			} else {
				/* Same step keep the current step */
				/* Keep the current step */
				pr_info("%s: keeep current step=%d\n", __func__, chip->step_index);
			}	
		}				
	}

	return rc;	
}

static void pca9468_status_change_work(struct work_struct *work)
{
	struct pca9468_step_chg_info *chip = container_of(work,
			struct pca9468_step_chg_info, status_change_work.work);
	int rc = 0;
	int reschedule_us;
	union power_supply_propval prop = {0, };

	pr_info("%s: ========Start========\n", __func__);

	if (!is_batt_available(chip)) {		
		/* cancel delayed work */
		if (delayed_work_pending(&chip->status_change_work))
			cancel_delayed_work(&chip->status_change_work);
		pr_info("%s: cancel status_change_work by no battery psy\n", __func__);		
		chip->step_index = -EINVAL;
		chip->jeita_fcc_index = -EINVAL;
		chip->jeita_fv_index = -EINVAL;
		chip->lcd_on_jeita_limit = true;
		chip->lcd_on_step_chg_limit= true;
		chip->lcd_off_reset = false;
		chip->lcd_on_enabled_step = true;
		__pm_relax(chip->step_chg_ws);
		return;
	}

	if (!is_dcchg_available(chip)) {
		/* direct charging power supply is not available */
		/* cancel delayed work */
		if (delayed_work_pending(&chip->status_change_work))
			cancel_delayed_work(&chip->status_change_work);
		pr_info("%s: cancel status_change_work by no pca9468-mains\n", __func__);
		/* Clear parameters */
		chip->step_index = -EINVAL;
		chip->jeita_fcc_index = -EINVAL;
		chip->jeita_fv_index = -EINVAL;
		chip->lcd_on_jeita_limit = true;
		chip->lcd_on_step_chg_limit= true;
		chip->lcd_off_reset = false;
		chip->lcd_on_enabled_step = true;
		__pm_relax(chip->step_chg_ws);
		return;
	}

	/* check jeita first */
	rc = pca9468_handle_jeita(chip);
	if (rc < 0) {
		/* error happened, set reschedule time - 2s */		
		pr_err("%s: Couldn't handle jeita rc = %d\n", __func__, rc);
		reschedule_us = JEITA_ERROR_RETRY_TIME;
		goto reschedule;
	}

	/* check step charging second */
	if (chip->jeita_fcc_index == JEITA_STAGE2) {
		/* Need step charging */
		rc = pca9468_handle_step_chg_config(chip);
		if (rc < 0) {
			pr_err("Couldn't handle step rc = %d\n", rc);
			reschedule_us = STEP_ERROR_RETRY_TIME;
			goto reschedule;
		}
	}

	/* Cancel delayed work if direct charing is stopped */
	power_supply_get_property(chip->dcchg_psy,
			POWER_SUPPLY_PROP_ONLINE, &prop);
	if (!prop.intval) {
		/* Check whether jeita stage is 0 or not */
		if ((chip->jeita_fcc_index == JEITA_STAGE0) ||
			(chip->jeita_fcc_index == JEITA_STAGE6)) {
			/* Set the periodic update */
			reschedule_us = STATUS_UPDATE_TIME;
			pr_info("%s: jeita_stage0 or 6, update status_change_work after %dus\n", __func__, reschedule_us);
		} else {
			/* does not work the direct charging */
			if (delayed_work_pending(&chip->status_change_work))
				cancel_delayed_work(&chip->status_change_work);
			pr_info("%s: cancel status_change_work by stop charging\n", __func__);

			/* Clear parameters */
			chip->step_index = -EINVAL;
			chip->jeita_fcc_index = -EINVAL;
			chip->jeita_fv_index = -EINVAL;
			chip->lcd_on_jeita_limit = true;
			chip->lcd_on_step_chg_limit= true;
			chip->lcd_off_reset = false;
			chip->lcd_on_enabled_step = true;
			__pm_relax(chip->step_chg_ws);
			return;
		}
	} else {
		/* Direct charging is working */
		/* Check whether jeita stage is 0 or not */
		if ((chip->jeita_fcc_index == JEITA_STAGE0) ||
			(chip->jeita_fcc_index == JEITA_STAGE6)) {
			/* Re-enable PCA9468 by other driver */
			/* Clear parameters  and re-initialization */
			chip->step_index = -EINVAL;
			chip->jeita_fcc_index = -EINVAL;
			chip->jeita_fv_index = -EINVAL;
			chip->lcd_on_jeita_limit = true;
			chip->lcd_on_step_chg_limit= true;
			chip->lcd_off_reset = false;
			chip->lcd_on_enabled_step = true;
			/* update step status at once */
			reschedule_us = 0;
		} else {
			/* Set the periodic update */
			reschedule_us = STATUS_UPDATE_TIME;
		}
		pr_info("%s: update status_change_work after %dus\n", __func__, reschedule_us);
	}

reschedule:
	schedule_delayed_work(&chip->status_change_work,
				usecs_to_jiffies(reschedule_us));
}

static int pca9468_step_chg_notifier_call(struct notifier_block *nb,
		unsigned long ev, void *v)
{
	struct power_supply *psy = v;
	struct pca9468_step_chg_info *chip = container_of(nb, struct pca9468_step_chg_info, nb);

	if (ev != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	if ((strcmp(psy->desc->name, "pca9468-mains") == 0)) {
		__pm_stay_awake(chip->step_chg_ws);
		/* reschedule work */
		if (delayed_work_pending(&chip->status_change_work))
			cancel_delayed_work(&chip->status_change_work);

		schedule_delayed_work(&chip->status_change_work, 0);
	}

	return NOTIFY_OK;
}

static int pca9468_step_chg_register_notifier(struct pca9468_step_chg_info *chip)
{
	int rc;

	chip->nb.notifier_call = pca9468_step_chg_notifier_call;
	rc = power_supply_reg_notifier(&chip->nb);
	if (rc < 0) {
		pr_err("Couldn't register psy notifier rc = %d\n", rc);
		return rc;
	}

	return 0;
}

int pca9468_step_chg_init(struct device *dev)
{
	int rc;
	struct pca9468_step_chg_info *chip;

	pr_info("%s: ========Start========\n", __func__);

	if (the_chip) {
		pr_err("Already initialized\n");
		return -EINVAL;
	}

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->step_chg_ws = wakeup_source_register("pca9468-step-chg");
	if (!chip->step_chg_ws)
		return -EINVAL;

	chip->dev = dev;
	chip->step_index = -EINVAL;
	chip->jeita_fcc_index = -EINVAL;
	chip->jeita_fv_index = -EINVAL;

	chip->step_chg_config = devm_kzalloc(dev,
			sizeof(struct step_chg_cfg), GFP_KERNEL);
	if (!chip->step_chg_config)
		return -ENOMEM;

	chip->step_chg_config->psy_prop = POWER_SUPPLY_PROP_CAPACITY;
	chip->step_chg_config->prop_name = "BATT_SOC";
	chip->step_chg_config->counter = 0;

	chip->jeita_fcc_config = devm_kzalloc(dev,
			sizeof(struct jeita_fcc_cfg), GFP_KERNEL);
	chip->jeita_fv_config = devm_kzalloc(dev,
			sizeof(struct jeita_fv_cfg), GFP_KERNEL);
	if (!chip->jeita_fcc_config || !chip->jeita_fv_config)
		return -ENOMEM;

	chip->jeita_fcc_config->psy_prop = POWER_SUPPLY_PROP_TEMP;
	chip->jeita_fcc_config->prop_name = "BATT_TEMP";
	chip->jeita_fcc_config->hysteresis = 10;
	chip->jeita_fv_config->psy_prop = POWER_SUPPLY_PROP_TEMP;
	chip->jeita_fv_config->prop_name = "BATT_TEMP";
	chip->jeita_fv_config->hysteresis = 10;

	INIT_DELAYED_WORK(&chip->status_change_work, pca9468_status_change_work);
	INIT_DELAYED_WORK(&chip->get_config_work, pca9468_get_config_work);

	rc = pca9468_step_chg_register_notifier(chip);
	if (rc < 0) {
		pr_err("%s:Couldn't register psy notifier rc = %d\n", __func__, rc);
		goto release_wakeup_source;
	}

	schedule_delayed_work(&chip->get_config_work,
			msecs_to_jiffies(GET_CONFIG_DELAY_MS));

	the_chip = chip;

	pr_info("%s: ========End========\n", __func__);

	return 0;

release_wakeup_source:
	wakeup_source_unregister(chip->step_chg_ws);
	return rc;
}


void pca9468_step_chg_deinit(void)
{
	struct pca9468_step_chg_info *chip = the_chip;

	if (!chip)
		return;

	cancel_delayed_work_sync(&chip->status_change_work);
	cancel_delayed_work_sync(&chip->get_config_work);
	power_supply_unreg_notifier(&chip->nb);
	wakeup_source_unregister(chip->step_chg_ws);
	the_chip = NULL;
}
