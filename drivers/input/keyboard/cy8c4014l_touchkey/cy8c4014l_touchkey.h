#ifndef _CYPRESS_KEYS_H
#define _CYPRESS_KEYS_H

/* CYPRESS4000 Touchkey Reg Addr*/
#define CYPRESS4000_TOUCHKEY_KEYVAL         0x00  //key value reg
#define CYPRESS4000_TOUCHKEY_MODE           0x3
#define CYPRESS4000_TOUCHKEY_FW             0x4   //firmware ver reg
#define CYPRESS4000_KEY0_RAWDATA0           0x5
#define CYPRESS4000_KEY0_RAWDATA1           0x6
#define CYPRESS4000_KEY0_BASELINE0          0x7
#define CYPRESS4000_KEY0_BASELINE1          0x8
#define CYPRESS4000_KEY1_RAWDATA0           0x9
#define CYPRESS4000_KEY1_RAWDATA1           0xa
#define CYPRESS4000_KEY1_BASELINE0          0xb
#define CYPRESS4000_KEY1_BASELINE1          0xc

#define CYPRESS4000_KEY0_CP0                0xd
#define CYPRESS4000_KEY0_CP1                0xe
#define CYPRESS4000_KEY1_CP0                0xf
#define CYPRESS4000_KEY1_CP1                0x10

#define CYPRESS4000_TOUCHKEY_CMD            0x11

#define CYPRESS4000_KEY0_CP2                0x12
#define CYPRESS4000_KEY0_CP3                0x13
#define CYPRESS4000_KEY1_CP2                0x14
#define CYPRESS4000_KEY1_CP3                0x15

#define CYPRESS4000_TOUCHKEY_PROJECT        0x16
#define CYPRESS4000_TOUCHKEY_LSENSITIVITY   0x17
#define CYPRESS4000_TOUCHKEY_RSENSITIVITY   0x18

#define CYPRESS4000_WAKEUP_MODE             0x5a
#define CYPRESS4000_SLEEP_MODE              0xa5

/* Cy8c4044lqi-421 input key code */
#define CYPRESS_KEY_LEFT                    KEY_GAMEFIRE_LEFT
#define CYPRESS_KEY_RIGHT                   KEY_GAMEFIRE_RIGHT
#define CYPRESS_LEFT_BIT                    0x2
#define CYPRESS_RIGHT_BIT                   0x1

#define CYPRESS_BUFF_LENGTH                 1024

struct cypress_platform_data{
    struct input_dev *input_dev;
    int irq_gpio;
    int irq_flag;
    int power_gpio;
    int power_on_flag;
    struct regulator *avdd_ldo;
    int rst_gpio;
};
struct cypress_info {
    struct i2c_client *i2c;
    struct device *dev_t;
    struct cypress_platform_data *platform_data;
    struct delayed_work cypress_update_work;
    struct workqueue_struct *cypress_update_wq;
    int irq;
	int new_mode;
	bool bUpdateOver;

};

#endif
