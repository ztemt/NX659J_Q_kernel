#ifndef _AW9106_H_
#define _AW9106_H_

#define NUBIA_LED0_AW9106

#define MAX_I2C_BUFFER_SIZE 65536

#define AW9106_ID 0x23

#ifdef NUBIA_LED0_AW9106
struct aw9106_pinctrl {
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_active;
	struct pinctrl_state *pin_suspend;
};

enum aw9106_led_mode {
	DIRECT_ON = 0,
	DIRECT_OFF,
	BLINK,
	FADE_IN,
	FADE_OUT,
	BREATH_ONCE,
};

enum aw9106_off_mode {
	IMMEDIATE = 0,
	GRADUAL,
	PAUSE,
	SHUTDOWN,
};

enum aw_outn_mode{
	AW_SW_RESET,	    // 0  soft_reset , all regs revert to default value.
	AW_CONST_ON,	    // 1 work on a constant lightness.
	AW_CONST_OFF,	    // 2 darkness is comming
	AW_AUTO_BREATH, 	// 3 self breathing, used in sences such as missing message.
	AW_STEP_FADE_IN,	// 4  fade in means that the lightness is getting stronger.
	AW_STEP_FADE_OUT,	// 5  fade out means that the lightness is getting weaker
	AW_BREATH_ONCE,     // 6 only breath once, touch the home menu for instance.
	AW_RESERVED,		// 7 reserverd.
};

#endif

struct aw9106 {
	struct i2c_client *i2c;
	struct device *dev;
	struct led_classdev cdev;
	struct work_struct brightness_work;

	int reset_gpio;

	unsigned char chipid;

	int imax;
	int rise_time;
	int on_time;
	int fall_time;
	int off_time;

#ifdef NUBIA_LED0_AW9106
	unsigned char led_sel;
	unsigned char led_mode;
	unsigned long led_level;
	unsigned char led_current;
	int delay_time;
	int rise_time_max;
	int on_time_max;
	int fall_time_max;
	int off_time_max;
	int delay_time_max;
	struct aw9106_pinctrl pinctrl_info;
	unsigned char min_grade;
	unsigned char max_grade;
	unsigned char blink_mode;
#endif
};

#endif

