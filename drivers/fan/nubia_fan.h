#ifndef _NUBIA_FAN_H_
#define _NUBIA_FAN_H_

enum therm_product_id {
	FAN_LEVEL_0 = 0,   //close fan
	FAN_LEVEL_1,
	FAN_LEVEL_2,
	FAN_LEVEL_3,
	FAN_LEVEL_4,
	FAN_LEVEL_5,
	FAN_LEVEL_MAX = FAN_LEVEL_5,
};
struct fan_pinctrl {
    struct pinctrl *pinctrl;
    struct pinctrl_state *pin_active;
    struct pinctrl_state *pin_suspend;
};

enum reset_delay_utime {
	DELAY_200 = 200,
	DELAY_400 = 400,
	DELAY_600 = 600,
	DELAY_800 = 800,
	DELAY_900 = 900,   //close fan
	DELAY_1000 = 1000,
	DELAY_1100 = 1100,
	DELAY_1200 = 1200,
	DELAY_1300 = 1300,
	DELAY_1400 = 1400,
	DELAY_1500 = 1500,
	DELAY_1800 = 1800,
};


struct fan {
    struct i2c_client *i2c;
    struct device *dev;
	struct regulator *pwr_reg;
	int reset_gpio;
	struct fan_pinctrl pinctrl_info;
};
static void fan_i2c_write(struct fan *fan, unsigned char *data, unsigned int length);
static unsigned int fan_i2c_read(struct fan *fan, unsigned char *data, unsigned int length);
static bool get_fan_power_on_state(void);
static void set_fan_power_on_state(bool state);
static void fan_set_pwm_by_level(unsigned int level);

#endif //_NUBIA_FAN_H_