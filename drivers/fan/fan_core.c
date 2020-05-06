#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#include "nubia_fan.h"
#define FAN_PINCTRL_STATE_ACTIVE "pull_up_default"
#define FAN_PINCTRL_STATE_SUSPEND "pull_down_default"
#define FAN_VREG_L6  "pm8150a_l6"
#define FAN_VREG_L6_VOLTAGE 3312000
static struct fan *nubia_fan;


#define STATUS_OK         	         (0)
#define STATUS_ERROR      	         (-1)
#define STATUS_I2C_OPEN_ERROR        (-2)
#define STATUS_I2C_CLOSE_ERROR       (-3)
#define STATUS_GPIO_OPEN_ERROR       (-4)
#define STATUS_GPIO_CLOSE_ERROR      (-5)
#define STATUS_READ_ERROR            (-6)
#define STATUS_WRITE_ERROR           (-7)
#define STATUS_SLADDR_ERROR          (-8)
#define STATUS_GPIO_HIGH_ERROR       (-9)
#define STATUS_GPIO_LOW_ERROR        (-10)
#define STATUS_INVALID_PROMPT_ERROR  (-11)
#define STATUS_INVALID_RESULT_ERROR  (-12)
#define STATUS_VERIFY_FAILED_ERROR   (-13)
#define STATUS_INVALID_ADDRESS_ERROR (-14)
#define VALUE_PROMPT            0x3E
#define DELAY_RESET_MS          (10 * 1000)  
#define DELAY_AFTER_RESET_US    (800)
#define DELAY_MASTERERASE_MS    (1000) 
#define DELAY_PROMPT_US			(200)

#define I2C_ADDRESS       (0x54 >> 1)
#define WRITECOMMAND      0xD0
#define READCOMMAND       0x20

#define MAX28200_LOADER_ERROR_NONE		0x00
#define MAX28200_LOADER_VERIFY_FAILED   0x05
static unsigned int fan_speed = 0;
static unsigned int fan_current = 0;
static unsigned int fan_temp = 0;
static unsigned int fan_level = 0;
static unsigned int g_fan_enable = 0;
static bool fan_power_on = 0;
static unsigned int fan_thermal_engine_level = 0;
static struct fan *nubia_fan;
static unsigned char firmware_magicvalue = 0x55;
static unsigned int firmware_version = 5;
static unsigned char firmware_version_reg = 0x09;
int gpio11_test;
struct delayed_work fan_delay_work;


static uint8_t fw_002[]= {
0x3a,0xda,0x00,0x6b,0x00,0x70,0x00,0x3b,0xf7,0x88,0x02,0x0b,0x32,0x0c,0xff,0xff,
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
0x8d,0x8c,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
0x70,0x0c,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
0x6a,0x0c,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
0x8d,0x8c,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
0x8d,0x8c,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
0x8d,0x8c,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
0x8d,0x8c,0x0a,0x8d,0x39,0x8d,0x80,0x3f,0x0f,0x8a,0x01,0x4a,0x0a,0x8f,0x00,0x40,
0x0d,0xb9,0x0d,0x8a,0x8d,0x8c,0x0a,0x8d,0x39,0x8d,0x00,0x2b,0xff,0x79,0x13,0x8a,
0xff,0x0b,0xf7,0x1a,0x0a,0x93,0x0d,0xb9,0x0d,0x8a,0x8d,0x8c,0x00,0x4b,0x00,0x03,
0x01,0x2b,0x00,0x13,0x01,0x2b,0x00,0x33,0x00,0x2b,0x0a,0xc3,0x00,0x2b,0x2a,0x43,
0x00,0x13,0x21,0x03,0x0d,0x8c,0x00,0x03,0x00,0x2b,0x00,0x43,0x0d,0x8c,0xf9,0x8a,
0x00,0x2b,0x00,0x79,0xff,0x78,0x02,0x3c,0x00,0x39,0x02,0x0c,0x02,0x0b,0x81,0x3d,
0x0d,0x8c,0x00,0x39,0xf4,0x3d,0x39,0x8a,0xf0,0x1a,0x24,0x5c,0x39,0x8a,0x2a,0x8a,
0xd2,0x4a,0x0a,0x8c,0x00,0x0b,0xf4,0x0c,0x00,0x0b,0xf5,0x0c,0x01,0x0b,0x07,0x0c,
0x01,0x0b,0x2a,0x0c,0x01,0x0b,0x36,0x0c,0x01,0x0b,0x91,0x0c,0x01,0x0b,0x99,0x0c,
0x01,0x0b,0xb8,0x0c,0x01,0x0b,0x5a,0x0c,0x01,0x0b,0xfe,0x0c,0x00,0x0b,0xf4,0x0c,
0x00,0x0b,0xf4,0x0c,0x00,0x0b,0xf4,0x0c,0x00,0x0b,0xf4,0x0c,0x00,0x0b,0xf4,0x0c,
0x00,0x0b,0xf4,0x0c,0x01,0x0b,0xd7,0x3d,0x0d,0x8c,0x02,0x0b,0x6f,0x3d,0x39,0x8a,
0x55,0x78,0x0a,0x3c,0x80,0x3f,0x00,0x0f,0x00,0x4b,0x37,0x80,0x37,0x80,0x00,0x40,
0x00,0x2b,0x08,0x20,0x08,0x20,0x02,0x0c,0x01,0x0b,0xd7,0x3d,0x0d,0x8c,0x80,0x3f,
0x0f,0x8a,0x0a,0xde,0x00,0x2b,0xc7,0xb3,0x0a,0xb3,0x9e,0xb3,0x6d,0x8d,0xff,0x0b,
0xfe,0x6d,0x03,0x4d,0x00,0x2b,0x55,0x29,0x03,0x0c,0x13,0xb7,0xfa,0x6c,0x37,0x93,
0x0d,0xed,0x39,0x8a,0x55,0x78,0x07,0x3c,0xa7,0x80,0x02,0x0b,0x6f,0x3d,0x39,0x8a,
0x55,0x78,0x01,0x3c,0x02,0x0c,0x01,0x0b,0xd7,0x3d,0x00,0x2b,0x47,0xb3,0x80,0x3f,
0x00,0x0f,0x0d,0x8c,0x02,0x0b,0x6f,0x3d,0x39,0x8a,0x55,0x78,0x02,0x3c,0x27,0x80,
0x02,0x0c,0x01,0x0b,0xd7,0x3d,0x00,0x20,0x00,0x40,0x0d,0x8c,0x02,0x0b,0x81,0x3d,
0x00,0x2b,0x39,0xa9,0x02,0x0b,0x6f,0x3d,0x00,0x2b,0x01,0x69,0x02,0x0b,0x2c,0x3d,
0x00,0x08,0xd9,0x8a,0x01,0x78,0x15,0x3c,0xa9,0x8a,0x00,0x78,0x0f,0x3c,0xa7,0x80,
0xa9,0xed,0x10,0x0b,0x00,0x21,0x18,0x0b,0x10,0x21,0xc8,0x01,0x00,0x0a,0x02,0x79,
0x79,0xca,0xfe,0x4d,0x0a,0x91,0x00,0x31,0xa7,0xa1,0x03,0x0c,0x27,0xa1,0x00,0x2b,
0x00,0x69,0x0d,0x8c,0x02,0x0b,0x81,0x3d,0x00,0x2b,0x39,0xa9,0x02,0x0b,0x6f,0x3d,
0x00,0x2b,0x01,0x69,0x02,0x0b,0x2c,0x3d,0x00,0x08,0xa9,0xed,0xd9,0x8a,0x01,0x78,
0x27,0x3c,0xa9,0x8a,0x00,0x78,0x21,0x3c,0xa7,0x80,0xa9,0xed,0x10,0x0b,0x00,0x21,
0x18,0x0b,0x10,0x21,0xf0,0x01,0x00,0x09,0x00,0x2b,0x00,0x29,0x02,0x79,0x00,0x2b,
0x04,0x09,0x00,0x08,0x79,0xca,0x0a,0x08,0x89,0xca,0x0a,0x78,0x03,0x2c,0x0a,0x5a,
0x00,0x08,0x01,0x4a,0xf6,0x4d,0x0a,0x08,0x06,0x78,0x02,0x2c,0x00,0x08,0x01,0x4a,
0x00,0x08,0x00,0x31,0x09,0x91,0xa7,0xa1,0x03,0x0c,0x27,0xa1,0x00,0x2b,0x00,0x69,
0x0d,0x8c,0x02,0x0b,0x6f,0x3d,0x02,0x0b,0x2c,0x3d,0x27,0xa1,0x00,0x2b,0x00,0x69,
0x0d,0x8c,0x00,0x0a,0x47,0x3d,0x0a,0xde,0x00,0x2b,0xc7,0xb3,0x0a,0xb3,0x9e,0xb3,
0x6d,0x8d,0xff,0x0b,0xfe,0x6d,0x03,0x4d,0x00,0x2b,0x55,0x29,0x03,0x0c,0x13,0xb7,
0xfa,0x6c,0x37,0x93,0x0d,0xed,0x39,0x8a,0x55,0x78,0x06,0x3c,0x02,0x0b,0x6f,0x3d,
0x39,0x8a,0x55,0x78,0x01,0x3c,0x01,0x0c,0x22,0x3d,0x00,0x2b,0x47,0xb3,0x0d,0x8c,
0x01,0x0a,0x28,0x3d,0x0a,0xde,0x00,0x2b,0xc7,0xb3,0x0a,0xb3,0x9e,0xb3,0x6d,0x8d,
0xff,0x0b,0xfe,0x6d,0x03,0x4d,0x00,0x2b,0x55,0x29,0x03,0x0c,0x13,0xb7,0xfa,0x6c,
0x37,0x93,0x0d,0xed,0x39,0x8a,0x55,0x78,0x06,0x3c,0x02,0x0b,0x6f,0x3d,0x39,0x8a,
0x55,0x78,0x01,0x3c,0x01,0x0c,0x03,0x3d,0x00,0x2b,0x47,0xb3,0x0d,0x8c,0x87,0x80,
0x00,0x2b,0xf7,0xb3,0x00,0x2b,0xf7,0x93,0x00,0x0b,0xbb,0x3d,0x00,0x0b,0xae,0x3d,
0x07,0x80,0x0d,0x8c,0x7d,0x8d,0x87,0x84,0x17,0x84,0x27,0x84,0x0a,0xea,0x02,0x2c,
0x37,0x84,0x01,0x0c,0xb7,0x84,0x47,0x84,0x67,0x84,0x00,0x1b,0x27,0x84,0x00,0x1b,
0xb7,0x84,0x00,0x1b,0x47,0x84,0x01,0x0b,0xf4,0x7d,0xff,0x5d,0x87,0x94,0x01,0x0b,
0xf4,0x7d,0xff,0x5d,0x24,0x8a,0x07,0x84,0x0d,0xfd,0x0d,0x8c,0x05,0x0a,0x0a,0xde,
0x00,0x2b,0xc7,0xb3,0x0a,0xb3,0x9e,0xb3,0x6d,0x8d,0xff,0x0b,0xfe,0x6d,0x03,0x4d,
0x00,0x2b,0x55,0x29,0x03,0x0c,0x13,0xb7,0xfa,0x6c,0x37,0x93,0x0d,0xed,0x39,0x8a,
0x55,0x78,0x06,0x3c,0x02,0x0b,0x6f,0x3d,0x39,0x8a,0x55,0x78,0x01,0x3c,0x01,0x0c,
0xbe,0x3d,0x00,0x2b,0x47,0xb3,0x0d,0x8c,0x00,0x2b,0xf7,0x93,0x00,0x1b,0xb7,0x93,
0x97,0xd3,0x00,0x6b,0xc7,0xf0,0x3a,0xda,0x0d,0x8c,0x00,0x2b,0xf7,0x93,0x00,0x1b,
0xb7,0x93,0x13,0xb7,0xfe,0x6c,0x0d,0x8c,0x00,0x21,0x00,0x31,0x00,0x11,0xff,0x0b,
0xff,0x01,0x0d,0x8c,0x00,0x6b,0x00,0x60,0x00,0x6b,0x00,0x70,0x87,0xd8,0x00,0x4b,
0x97,0x80,0x17,0x80,0x00,0x4b,0xa7,0x80,0x27,0x80,0x00,0x4b,0x87,0x80,0x07,0x80,
0x00,0x6b,0xe7,0x90,0x00,0x6b,0x77,0x90,0x00,0x6b,0x27,0x90,0x00,0x6b,0xb7,0x90,
0x00,0x2b,0x77,0x88,0x3a,0xda,0x3a,0xda,0x3a,0xda,0x00,0x0b,0xbb,0x3d,0x00,0x0b,
0xae,0x3d,0x00,0x2b,0x00,0x69,0x02,0x0c,0x00,0x0b,0xc9,0x3d,0xe9,0x8a,0x01,0x78,
0x05,0x3c,0xc2,0x3d,0xf9,0x8a,0xff,0x78,0xf9,0x7c,0xf6,0x0c,0xc6,0x3d,0xa7,0x80,
0xf3,0x0c,0x6d,0x8d,0x00,0x1b,0x37,0x93,0xff,0x0b,0xfe,0x6d,0x02,0x4d,0x55,0x39,
0x04,0x0c,0x13,0x8a,0x01,0x1a,0xfa,0x1c,0x07,0x93,0x0d,0xed,0x0d,0x8c,0x6d,0x8d,
0xff,0x0b,0xfe,0x6d,0x00,0x39,0x00,0x2b,0x00,0x59,0x04,0x4d,0x55,0x39,0x00,0x2b,
0x01,0x59,0x05,0x0c,0x13,0x8a,0x40,0x1a,0xf8,0x1c,0x07,0x93,0x67,0x93,0x0d,0xed,
0x0d,0x8c,0x6d,0x8d,0xff,0x0b,0xfe,0x6d,0x00,0x39,0x02,0x4d,0x55,0x39,0x12,0x0c,
0x23,0x8a,0x01,0x0b,0x00,0x1a,0xf9,0x5c,0x33,0xb9,0x39,0x8a,0x02,0x78,0x07,0x3c,
0x06,0x78,0x05,0x3c,0x07,0x78,0x03,0x3c,0x09,0x78,0x01,0x3c,0x03,0x0c,0x00,0x2b,
0xf7,0xb3,0xc7,0x3d,0x0d,0xed,0x0d,0x8c,0x0a,0xde,0x6d,0x8d,0xff,0x0b,0xfe,0x6d,
0x0a,0xb3,0x03,0x4d,0x01,0x0b,0xd7,0x3d,0x1a,0x0c,0x23,0x8a,0x04,0x0b,0x00,0x1a,
0xf8,0x1c,0xff,0x0b,0xfe,0x6d,0x9e,0xb3,0x03,0x4d,0x01,0x0b,0xd7,0x3d,0x0f,0x0c,
0x23,0x8a,0x04,0x0b,0x00,0x1a,0xf8,0x1c,0xff,0x0b,0xfe,0x6d,0x03,0x4d,0x01,0x0b,
0xd7,0x3d,0x05,0x0c,0x13,0x8a,0x40,0x1a,0xf9,0x1c,0x07,0x93,0x67,0x93,0x0d,0xed,
0x0d,0x8c};


static bool get_fan_power_on_state(void){
	printk(KERN_ERR "%s: fan_power_on=%d\n",__func__,fan_power_on);
	return fan_power_on;
}	
static void set_fan_power_on_state(bool state){
	fan_power_on = state;
	printk(KERN_ERR "%s: state=%d\n",__func__,state);
}
void lowlevel_delay(int microseconds) {
  //
  // perform platform specific delay here
    //udelay(microseconds);
    mdelay(microseconds);
}

int lowlevel_i2cWrite(uint8_t *val, int length){
	int ret;
	struct i2c_client *i2c = nubia_fan->i2c;
	struct i2c_msg msg[1];	
		msg[0].addr = i2c->addr;
		msg[0].flags = 0;
		msg[0].len = length;
		msg[0].buf = val;
	
	ret = i2c_transfer(i2c->adapter, msg, 1);
	if(ret){
		return STATUS_WRITE_ERROR;
	}
	return STATUS_OK;
}

int lowlevel_i2cRead(uint8_t *val, int length){
		int ret;
		struct i2c_client *i2c = nubia_fan->i2c;
		struct i2c_msg msg[1];
		msg[0].addr = i2c->addr;
		msg[0].flags = I2C_M_RD;
		msg[0].len = length;
		msg[0].buf = val;
	ret = i2c_transfer(i2c->adapter, msg, 1);
	if(ret){
		return STATUS_READ_ERROR;
	}
	return STATUS_OK;
}

static unsigned char fan_i2c_firmware_read(struct fan *fan, unsigned char *data, unsigned int length){
	struct i2c_client *i2c = fan->i2c;
	unsigned int ret;
	char read_data[1] = {0};
	struct i2c_msg msg[1];
		msg[0].addr = i2c->addr;
		msg[0].flags = I2C_M_RD;
		msg[0].len = sizeof(read_data);
		msg[0].buf = read_data;
	ret=i2c_transfer(i2c->adapter, msg, 1);
	//printk(KERN_ERR "George fan_i2c_firmware_read addr:%02x,ret=%d,read_data[0]=%x,\n", i2c ->addr,ret,read_data[0]);
    return read_data[0];
}

int MAX28200_program(uint8_t *image, int size) {
  int i;
  uint8_t val;
  int status;
  int index;
  int address;
  int payloadLength;
  // send the Magic Value
  val = firmware_magicvalue;
  fan_i2c_write(nubia_fan,&val,1);

  udelay(75);
  
  // S 55 [3E*] P
  // read the prompt
  status=fan_i2c_firmware_read(nubia_fan,&firmware_magicvalue,1);
  if (status != VALUE_PROMPT) {
	 printk(KERN_ERR "%s<-->%d STATUS_INVALID_PROMPT_ERROR status:%x\n",__FUNCTION__,__LINE__,status);
	 return -1;
  }
  // NOP
  // S 54 00 P
  val = 0x00;
  fan_i2c_write(nubia_fan,&val,1);
  // S 55 [3E*] P
  // read the prompt
  //DEBUG_PRINT(("read the prompt...\n"));
    //status=i2c_smbus_read_byte_data(nubia_fan ->i2c,MAGIC_VALUE);
  status=fan_i2c_firmware_read(nubia_fan,&firmware_magicvalue,1);
  if (status != VALUE_PROMPT) {
	printk(KERN_ERR "%s<-->%d STATUS_INVALID_PROMPT_ERROR status:%x\n",__FUNCTION__,__LINE__,status);
    return -1;
  }

  // Master Erase
  // S 54 02 P
  //DEBUG_PRINT(("master erase...\n"));
  //printk(KERN_ERR "%s<-->%d\n",__FUNCTION__,__LINE__);
  val = 0x02;
  fan_i2c_write(nubia_fan,&val,1);
  
  // delay after issuing master erase
  lowlevel_delay(DELAY_MASTERERASE_MS);
  
  // S 55 [3E*] P
  // read the prompt
  status=fan_i2c_firmware_read(nubia_fan,&firmware_magicvalue,1);
  if (status != VALUE_PROMPT) {
	printk(KERN_ERR "%s<-->%d STATUS_INVALID_PROMPT_ERROR status:%x\n",__FUNCTION__,__LINE__,status);
	return -1;
  }

  // Get Status 
  // S 54 04 P
  val = 0x04;
  fan_i2c_write(nubia_fan,&val,1);

  // S 55 [04*] P
  status=fan_i2c_firmware_read(nubia_fan,&firmware_magicvalue,1);
  if (status != 0x04) {
	  printk(KERN_ERR "%s<-->%d STATUS_INVALID_PROMPT_ERROR status:%x\n",__FUNCTION__,__LINE__,status);
	  return -1;
	}

  // S 55 [00*] P
  status=fan_i2c_firmware_read(nubia_fan,&firmware_magicvalue,1);
  if (status != 0x00) {
	  printk(KERN_ERR "%s<-->%d STATUS_INVALID_PROMPT_ERROR status:%x\n",__FUNCTION__,__LINE__,status);
	  return -1;
	}

  // S 55 [3E*] P
  // read the prompt
  status=fan_i2c_firmware_read(nubia_fan,&firmware_magicvalue,1);
  if (status != VALUE_PROMPT) {
	  printk(KERN_ERR "%s<-->%d STATUS_INVALID_PROMPT_ERROR status:%x\n",__FUNCTION__,__LINE__,status);
	  return -1;
	}

  // Bogus Command
  // S 54 55 P
  //DEBUG_PRINT(("bogus command...\n"));
  val = 0x55;
  fan_i2c_write(nubia_fan,&val,1);
  //if (status != STATUS_OK) return status;

  // S 54 00 P
  val = 0x00;
  fan_i2c_write(nubia_fan,&val,1);
  //if (status != STATUS_OK) return status;

  // S 54 00 P
  val = 0x00;
  fan_i2c_write(nubia_fan,&val,1);
  //if (status != STATUS_OK) return status;

  // S 54 00 P
  val = 0x00;
  fan_i2c_write(nubia_fan,&val,1);
  //if (status != STATUS_OK) return status;

  // S 55 [3E*] P
  // read the prompt
 // DEBUG_PRINT(("read prompt...\n"));
  status=fan_i2c_firmware_read(nubia_fan,&firmware_magicvalue,1);
  if (status != VALUE_PROMPT) {
	  printk(KERN_ERR "%s<-->%d STATUS_INVALID_PROMPT_ERROR status:%x\n",__FUNCTION__,__LINE__,status);
	  return -1;
	}

  // Get Status 
  // S 54 04 P
  //DEBUG_PRINT(("get status...\n"));
  //printk(KERN_ERR "%s<-->%d Get Status  S 54 04 P\n",__FUNCTION__,__LINE__);
  val = 0x04;
  fan_i2c_write(nubia_fan,&val,1);
  //if (status != STATUS_OK) return status;

  // S 55 [04*] P
  status=fan_i2c_firmware_read(nubia_fan,&firmware_magicvalue,1);
  printk(KERN_ERR "%s<-->%d S55 [04*] P val:%x\n",__FUNCTION__,__LINE__,status);
  if (status != 0x04) {
	  printk(KERN_ERR "%s<-->%d STATUS_INVALID_PROMPT_ERROR status:%x\n",__FUNCTION__,__LINE__,status);
	  return -1;
	}

  //S 55 [01*] P
  status=fan_i2c_firmware_read(nubia_fan,&firmware_magicvalue,1);
  if (status != 0x01) {
	  printk(KERN_ERR "%s<-->%d STATUS_INVALID_PROMPT_ERROR status:%x\n",__FUNCTION__,__LINE__,status);
	  return -1;
	}

  // S 55 [3E*] P
  // read the prompt
  //DEBUG_PRINT(("read the prompt...\n"));
  printk(KERN_ERR "%s<-->%d\n",__FUNCTION__,__LINE__);
  status=fan_i2c_firmware_read(nubia_fan,&firmware_magicvalue,1);
  if (status != VALUE_PROMPT) {
	printk(KERN_ERR "%s<-->%d STATUS_INVALID_PROMPT_ERROR status:%x\n",__FUNCTION__,__LINE__,status);
	return -1;
  }
  //if (status != STATUS_OK) return status;
  //if (val != VALUE_PROMPT) return STATUS_INVALID_PROMPT_ERROR;

  // Set Multiplier - This value needs to be set such that bytes written multiplied by 4 gives the actual desired length in the length byte for load command.
  // S 54 0B P
  //DEBUG_PRINT(("set multiplier...\n"));
  //printk(KERN_ERR "%s<-->%d\n",__FUNCTION__,__LINE__);
  val = 0x0B;
  fan_i2c_write(nubia_fan,&val,1);
  //if (status != STATUS_OK) return status;
  // S 54 00 P
  val = 0x00;
   fan_i2c_write(nubia_fan,&val,1);
  //if (status != STATUS_OK) return status;
  // S 54 04 P
  val = 0x04;
  fan_i2c_write(nubia_fan,&val,1);
  //if (status != STATUS_OK) return status;
  // S 54 00 P
  val = 0x00;
  fan_i2c_write(nubia_fan,&val,1);
  //if (status != STATUS_OK) return status;

  // S 55 [3E*] P
  // read the prompt
  status=fan_i2c_firmware_read(nubia_fan,&firmware_magicvalue,1);
  if (status != VALUE_PROMPT) {
	printk(KERN_ERR "%s<-->%d STATUS_INVALID_PROMPT_ERROR status:%x\n",__FUNCTION__,__LINE__,status);
	return -1;
  }
  //if (status != STATUS_OK) return status;
  //if (val != VALUE_PROMPT) return STATUS_INVALID_PROMPT_ERROR;

  //
  // load the image
  //
  index = 0;
  address = 0;
  payloadLength = 16;
  //DEBUG_PRINT(("load the image...\n"));
  while (index < size) { 
    // Write Command
    val = WRITECOMMAND;
    //status = lowlevel_i2cWrite(&val, 1);
	//if (status != STATUS_OK) return status;
    fan_i2c_write(nubia_fan,&val,1);
	
    // Byte Count
    val = (uint8_t)payloadLength;
    //DEBUG_PRINT(("byte count %02x ",val));
    //status = lowlevel_i2cWrite(&val, 1);
    //if (status != STATUS_OK) return status;
	fan_i2c_write(nubia_fan,&val,1);
	

    // Low Address
    val = (uint8_t)(address & 0xff);
    //DEBUG_PRINT(("lo %02x ",val));
    //status = lowlevel_i2cWrite(&val, 1);
    //if (status != STATUS_OK) return status;
	fan_i2c_write(nubia_fan,&val,1);

    // High Address
    val = (uint8_t)((address >> 8) & 0xff);
    //DEBUG_PRINT(("hi %02x ",val));
    //status = lowlevel_i2cWrite(&val, 1);
    //if (status != STATUS_OK) return status;
	fan_i2c_write(nubia_fan,&val,1);

	
    for (i = 0; i < payloadLength; i++) {
      val = image[index++];
      //DEBUG_PRINT(("%02x ",val));
      //status = lowlevel_i2cWrite(&val, 1);
      //if (status != STATUS_OK) return status;
	  fan_i2c_write(nubia_fan,&val,1);
    }

    // advance the address
    address += payloadLength;

    // S 55 [3E*] P
    // read the prompt
    //status = lowlevel_i2cRead(&val, 1);
    //if (status != STATUS_OK) return status;
    //if (val != VALUE_PROMPT) return STATUS_INVALID_PROMPT_ERROR;
	status=fan_i2c_firmware_read(nubia_fan,&firmware_magicvalue,1);
    if (status != VALUE_PROMPT) {
	printk(KERN_ERR "%s<-->%d STATUS_INVALID_PROMPT_ERROR status:%x\n",__FUNCTION__,__LINE__,status);
	return -1;
    }

	
    // Get Status 
    // S 54 04 P
    //DEBUG_PRINT(("get status...\n"));
    val = 0x04;
    //status = lowlevel_i2cWrite(&val, 1);
    //if (status != STATUS_OK) return status;
    fan_i2c_write(nubia_fan,&val,1);
	

    // S 55 [04*] P
    //status = lowlevel_i2cRead(&val, 1);
    //if (status != STATUS_OK) return status;
    //if (val != 0x04) return STATUS_VERIFY_FAILED_ERROR;
	status=fan_i2c_firmware_read(nubia_fan,&firmware_magicvalue,1);
	if (status != 0x04) {
		printk(KERN_ERR "%s<-->%d STATUS_INVALID_PROMPT_ERROR status:%x\n",__FUNCTION__,__LINE__,status);
		return -1;
	  }


    // S 55 [00*] P
    //status = lowlevel_i2cRead(&val, 1);
    //if (status != STATUS_OK) return status;
    //if (val != MAX28200_LOADER_ERROR_NONE) return STATUS_VERIFY_FAILED_ERROR;
	status=fan_i2c_firmware_read(nubia_fan,&firmware_magicvalue,1);
	if (status != MAX28200_LOADER_ERROR_NONE) {
		printk(KERN_ERR "%s<-->%d STATUS_INVALID_PROMPT_ERROR status:%x\n",__FUNCTION__,__LINE__,status);
		return -1;
	  }

    // S 55 [3E*] P
    // read the prompt
    //status = lowlevel_i2cRead(&val, 1);
    //if (status != STATUS_OK) return status;
    //if (val != VALUE_PROMPT) return STATUS_INVALID_PROMPT_ERROR; 
    status=fan_i2c_firmware_read(nubia_fan,&firmware_magicvalue,1);
	if (status != VALUE_PROMPT) {
		printk(KERN_ERR "%s<-->%d STATUS_INVALID_PROMPT_ERROR status:%x\n",__FUNCTION__,__LINE__,status);
		return -1;
	  }
  }
  return STATUS_OK;
}

int MAX28200_fw_updata(void) {
  int status;
  status = MAX28200_program(fw_002, sizeof(fw_002)/sizeof(fw_002[0]));
  if (status != STATUS_OK) {
    printk(KERN_ERR "Error: Programming returned with error: %d\n", status);
  }
  //printk(KERN_ERR "\ndone.\n");
  return status;
}


static void fan_i2c_write(struct fan *fan, unsigned char *data, unsigned int length){
	int ret;
	struct i2c_client *i2c = fan->i2c;
	struct i2c_msg msg[1];	
	//printk(KERN_ERR "George %s<-->%d,addr:%02x\n",__FUNCTION__,__LINE__,nubia_fan ->i2c ->addr);
		msg[0].addr = i2c->addr;
		msg[0].flags = 0;
		msg[0].len = length;
		msg[0].buf = data;
	
	ret = i2c_transfer(i2c->adapter, msg, 1);
	//printk(KERN_ERR "FAN fan_i2c_write ret=%d,msg[0].addr=%x, msg[0].buf:%x,len=%u\n", ret,msg[0].addr,*data,msg[0].len);
}

static unsigned int fan_i2c_read(struct fan *fan, unsigned char *data, unsigned int length){
	struct i2c_client *i2c = fan->i2c;
	unsigned int ret1,ret2;
	unsigned int ret;
	char read_data[2] = {0,0};
	struct i2c_msg msg[2];
		msg[0].addr = i2c->addr;
		msg[0].flags = 0;
		msg[0].len = length;
		msg[0].buf = data;

		msg[1].addr = i2c->addr;
		msg[1].flags = I2C_M_RD;
		msg[1].len = sizeof(read_data);
		msg[1].buf = read_data;
		
	ret=i2c_transfer(i2c->adapter, msg, 2);
	ret1 = read_data[0];
	ret2 = read_data[1];
	ret = ret1 + ret2*256;
	//printk(KERN_ERR "George fan_i2c_read addr:%02x,ret=%d,ret1=%d,ret2=%d,read_data[0]=%x,read_data[1]=%x\n", i2c ->addr,ret,ret1,ret2,read_data[0],read_data[1]);
	
	return ret;
}

static void start_speed_count(struct fan *fan){
	static unsigned char data = 0x01;
	fan_i2c_write(fan,&data,sizeof(data));
	printk(KERN_ERR "%s:fan_speed=%d\n",__func__,fan_speed);
}

static void stop_speed_count(struct fan *fan){
	static unsigned char data = 0x03;
	fan_i2c_write(fan,&data,sizeof(data));
	printk(KERN_ERR "%s:fan_speed=%d\n",__func__,fan_speed);
}

static unsigned int get_speed_count(struct fan *fan){
	static unsigned char data = 0x02;

	fan_speed = fan_i2c_read(fan,&data,sizeof(data));
	fan_speed =fan_speed *20; //the minute speed
	printk(KERN_ERR "%s:fan_speed=%d,fan_level=%d\n",__func__,fan_speed,fan_level);
	return fan_speed;
}

static unsigned int get_fan_current(struct fan *fan){
	static unsigned char data = 0x06;
	fan_current = fan_i2c_read(fan,&data,sizeof(data));
	return(fan_current);
}

static unsigned int get_fan_temp(struct fan *fan){
	static unsigned char data = 0x07;
	fan_temp = fan_i2c_read(fan,&data,sizeof(data));
	return(fan_temp);
}

static void start_pwm(struct fan *fan,unsigned short pwm_value){
	//static unsigned char data[2] ={0x04,0x00};//30KHZ
	static unsigned char data[2] ={0x08,0x00};// 25khz reg

	data[1] = pwm_value;
	//printk(KERN_ERR "George start_pwm data[0]=%x, data[1]=%x,sizeof=%d\n", data[0],data[1],sizeof(data));
	fan_i2c_write(fan,data,sizeof(data));
}

static void get_fan_read(void){
	unsigned int fan_count;
	unsigned int adc_current;
	unsigned int adc_temp;
	
	if((get_fan_power_on_state() == true) ){

		start_speed_count(nubia_fan);
		mdelay(1000);
		stop_speed_count(nubia_fan);
		
		fan_count = get_speed_count(nubia_fan)/3;
		
		adc_current = get_fan_current(nubia_fan);
		adc_temp = get_fan_temp(nubia_fan);

        /*if fan is running and read fan_count is 0,reset the fan*/
		if((0 == fan_speed) && get_fan_power_on_state()) {
		    printk(KERN_ERR "%s:begin reset the fan!!!,fan_level=%d\n",__func__,fan_level);
			set_fan_power_on_state(false);
			fan_set_pwm_by_level(fan_level);
		}
	
	}
	else{
		fan_speed = 0;
		fan_current = 0;
		fan_temp = 0;
	}

	printk(KERN_ERR "%s:fan_count=%d,adc_current=%d,adc_temp=%d\n",__func__,fan_count,adc_current,adc_temp);
}


static void fan_read_workqueue(struct work_struct *work){

	cancel_delayed_work(&fan_delay_work);
	get_fan_read();

}



static int fan_enable_reg(struct fan *fan,
		bool enable)
{
	int ret;

	printk(KERN_ERR "%s: enable=%d\n",__func__,enable);
	
	if (!enable) {
		ret = 0;
		goto disable_pwr_reg;
	}

	if ((fan->pwr_reg)&& (regulator_is_enabled(fan->pwr_reg) ==0)) {
		ret = regulator_enable(fan->pwr_reg);
		if (ret < 0) {
			dev_err(fan->dev->parent,"%s: Failed to enable power regulator\n",__func__);
		}
	}


     gpio_free(gpio11_test);
     ret = gpio_request(gpio11_test,"GPIO11");
     if (ret) {
	   pr_err("%s: fan reset gpio request failed\n",__func__);
          return ret;
     }
     gpio_direction_output(gpio11_test, 0);
     msleep(630);
     gpio_direction_output(gpio11_test,1);
     mdelay(100);
   return ret;

disable_pwr_reg:
      gpio_direction_output(gpio11_test,0);
      gpio_free(gpio11_test);


	if (fan->pwr_reg)
		regulator_disable(fan->pwr_reg);

	return ret;
}

static int fan_hw_reset(struct fan *fan,unsigned int delay)
{
    int ret;
	unsigned int reset_delay_time = 0;

    pr_info("%s enter %d\n", __func__,delay);
	reset_delay_time = delay;

    gpio_free(gpio11_test);
    ret = gpio_request(gpio11_test,"GPIO11");
    if (ret) {
	   pr_err("%s: fan reset gpio request failed\n",__func__);
          return ret;
    }
    gpio_direction_output(gpio11_test,1);
    msleep(10);
    gpio_direction_output(gpio11_test,0);
    msleep(100);
    gpio_direction_output(gpio11_test,1);

    switch(reset_delay_time){
			case DELAY_900:
				udelay(900);
				//pr_info("%s enter 900\n", __func__);
				break;
			case DELAY_1000:
				udelay(500);
				udelay(500);
				//pr_info("%s enter 1000\n", __func__);
				break;
			case DELAY_1100:
				udelay(500);
				udelay(600);
				//pr_info("%s enter 1100\n", __func__);
				break;
			case DELAY_1200:
				udelay(600);
				udelay(600);
				//pr_info("%s enter 1200\n", __func__);
				break;
			case DELAY_800:
				udelay(800);
				//pr_info("%s enter 800\n", __func__);
				break;
			case DELAY_600:	
				udelay(600);
				//pr_info("%s enter 600\n", __func__);
				break;
			case DELAY_400:	
				udelay(400);
				//pr_info("%s enter 400\n", __func__);
				break;	
			case DELAY_1300:
				udelay(600);
				udelay(700);
				//pr_info("%s enter 1300\n", __func__);
				break;	
			case DELAY_1400:
				udelay(700);
				udelay(700);
				//pr_info("%s enter 1400\n", __func__);
				break;	
			case DELAY_1500:
				udelay(800);
				udelay(700);
				//pr_info("%s enter 1500\n", __func__);
				break;		
			case DELAY_200:
				udelay(200);
				//pr_info("%s enter 200\n", __func__);
				break;	
			case DELAY_1800:
				udelay(900);
				udelay(900);
				//pr_info("%s enter 1800\n", __func__);
				break;					
			default:
				udelay(900);
				//pr_info("%s enter default\n", __func__);
				break;
		}
   	
       return 0;
}

static void fan_set_enable(bool enable) 
{
    printk(KERN_ERR "%s: enable=%d\n",__func__,enable);

	if(!enable)
	{
   	    start_pwm(nubia_fan,0);
		fan_speed = 0;
		fan_level = 0;
	}
	set_fan_power_on_state(enable);
	fan_enable_reg(nubia_fan, enable);
}

static void fan_set_pwm_by_level(unsigned int level)
{
	static unsigned int old_level = 0;

	printk(KERN_ERR "%s: level=%d,old_level=%d,fan_speed=%d\n",__func__,level,old_level,fan_speed);

	if(level < FAN_LEVEL_0 || level > FAN_LEVEL_MAX)
		return;

	fan_level = level;
	if(level == FAN_LEVEL_0){
       fan_set_enable(false);
	   old_level = level;
	}
	else {
		 	if(get_fan_power_on_state() == false){
                fan_set_enable(true);
                old_level = 0;
        	}
			
			if(old_level != level){
				switch(level){
					case FAN_LEVEL_1:
						start_pwm(nubia_fan,20);
						break;
					case FAN_LEVEL_2:
						start_pwm(nubia_fan,40);
						break;
					case FAN_LEVEL_3:
						//start_pwm(nubia_fan,60);
						start_pwm(nubia_fan,50);
						break;
					case FAN_LEVEL_4:
						start_pwm(nubia_fan,80);
						break;
					case FAN_LEVEL_5:
						start_pwm(nubia_fan,100);
						break;
					default:
						break;
				}
	        }
			old_level = level;
		schedule_delayed_work(&fan_delay_work,
			  round_jiffies_relative(msecs_to_jiffies
						(200)));
	}
}

static ssize_t fan_enable_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf){

	printk(KERN_ERR "%s: g_fan_enable=%d\n",__func__,g_fan_enable);	
	return sprintf(buf, "%d\n", g_fan_enable);
}

static ssize_t fan_enable_store(struct kobject *kobj,
	    struct kobj_attribute *attr, const char *buf, size_t count){
	    
 	sscanf(buf, "%d", &g_fan_enable);
    printk(KERN_ERR "%s: g_fan_enable=%d\n",__func__,g_fan_enable);

    return count;
}

static ssize_t fan_speed_level_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf){
		
	return sprintf(buf, "%d\n", fan_level);
}
	
static ssize_t fan_speed_level_store(struct kobject *kobj,
	    struct kobj_attribute *attr, const char *buf, size_t count){

    unsigned int level;
	static unsigned int old_level = 0;
 	sscanf(buf, "%d", &level);

	printk(KERN_ERR "%s: level=%d,old_level=%d\n",__func__,level,old_level);
	if(level == old_level && level == 0){
	   printk(KERN_ERR "%s: off before\n",__func__);
	}
	else{
		old_level = level;
        fan_set_pwm_by_level(level);
	}

    return count;
}

static ssize_t fan_speed_count_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf){
		
	if(get_fan_power_on_state() && fan_level != 0){

	   fan_set_pwm_by_level(fan_level);
	}

	printk(KERN_ERR "%s: fan_speed=%d\n",__func__,fan_speed);
	return sprintf(buf, "%d\n", fan_speed);
}

static ssize_t fan_current_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf){
		
	return sprintf(buf, "%d\n", fan_current);
}

static ssize_t fan_temp_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf){
		
	return sprintf(buf, "%d\n", fan_temp);
}
static ssize_t fan_thermal_engine_levell_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf){

	return sprintf(buf, "%d\n", fan_thermal_engine_level);
}

static ssize_t fan_thermal_engine_level_store(struct kobject *kobj,
	    struct kobj_attribute *attr, const char *buf, size_t count){
	    sscanf(buf, "%d", &fan_thermal_engine_level);
          //printk(KERN_ERR "%s: fan_thermal_engine_level=%d\n",__func__,fan_thermal_engine_level);
          return count;
}
static struct kobj_attribute fan_enable_attr=
    __ATTR(fan_enable, 0664, fan_enable_show, fan_enable_store);
static struct kobj_attribute fan_level_attr=
    __ATTR(fan_speed_level, 0664, fan_speed_level_show, fan_speed_level_store);
static struct kobj_attribute fan_speed_attr=
    __ATTR(fan_speed_count, 0664, fan_speed_count_show, NULL);
static struct kobj_attribute fan_current_attr=
    __ATTR(fan_current, 0664, fan_current_show, NULL);
static struct kobj_attribute fan_temp_attr=
    __ATTR(fan_temp, 0664, fan_temp_show, NULL);
static struct kobj_attribute fan_thermal_engine_level_attr=
    __ATTR(fan_thermal_engine_level, 0664, fan_thermal_engine_levell_show, fan_thermal_engine_level_store);



static struct attribute *fan_attrs[] = {
	&fan_enable_attr.attr,
    &fan_level_attr.attr,
	&fan_speed_attr.attr,
	&fan_current_attr.attr,
	&fan_temp_attr.attr,
	&fan_thermal_engine_level_attr.attr,
    NULL,
};

static struct attribute_group fan_attr_group = {
    .attrs = fan_attrs,
};
struct kobject *fan_kobj;

static int fan_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct fan *fan;
	struct device_node *np = i2c->dev.of_node;
	int ret;
    unsigned int delay_table[] = {DELAY_900,DELAY_1000,DELAY_1100,DELAY_1200,DELAY_800,
		                DELAY_600,DELAY_400,DELAY_1300,DELAY_1400,DELAY_1500,
		                DELAY_1800,DELAY_200};
	unsigned int try_count =0;
	unsigned int i =0;
	printk(KERN_ERR "fan_probe enter\n");
	
	fan = devm_kzalloc(&i2c->dev, sizeof(struct fan), GFP_KERNEL);
    if (fan == NULL)
        return -ENOMEM;

	fan->dev = &i2c->dev;
    fan->i2c = i2c;

	fan->pwr_reg = regulator_get(fan->dev->parent,
					FAN_VREG_L6);
		if (IS_ERR(fan->pwr_reg)) {
				dev_err(fan->dev->parent,
						"%s: Failed to get power regulator\n",
						__func__);
				ret = PTR_ERR(fan->pwr_reg);
				goto regulator_put;
		}
	
	
	ret = regulator_set_voltage(fan->pwr_reg, FAN_VREG_L6_VOLTAGE, FAN_VREG_L6_VOLTAGE);
		if (ret) {
			dev_err(fan->dev->parent,
						"Regulator vdd_fan set vtg failed rc=%d\n", ret);
			goto regulator_put;
		}
	  if (fan->pwr_reg) {
		ret = regulator_enable(fan->pwr_reg);
		if (ret < 0) {
			dev_err(fan->dev->parent,
					"%s: Failed to enable power regulator\n",
					__func__);
		}
	}

    nubia_fan = fan;
    gpio11_test =of_get_named_gpio(np,"fan,reset-gpio",0);

	fan_hw_reset(fan,900);
	mdelay(100); //delay 0.1 seconds
	ret= fan_i2c_read(fan,&firmware_version_reg,sizeof(firmware_version_reg));
	printk(KERN_ERR "%s: ret=%d\n",__func__,ret);
	//printk(KERN_ERR "George %s<-->%d read firmware version 0x09 value:%d\n", __func__,__LINE__,ret);
	
    if(ret < firmware_version){
		
		try_count = sizeof(delay_table)/sizeof(delay_table[0]);
		
		for(i = 0; i < try_count; i++){
			fan_hw_reset(fan,delay_table[i]);
			
			if(MAX28200_fw_updata()< 0){
			  printk(KERN_ERR "fan_fw_updata failed %d\n",delay_table[i]);
			  continue;
			}
			else {
				printk(KERN_ERR "fan_fw_updata done\n");
				
				break;
			}
		}

	    mdelay(10);
	    fan_hw_reset(fan,900);
	    mdelay(100); //delay 0.1 seconds
    }
	

	fan_kobj = kobject_create_and_add("fan", kernel_kobj);
	if (!fan_kobj)
	{
		printk(KERN_ERR "%s: fan kobj create error\n", __func__);
		return -ENOMEM;
	}
	ret = sysfs_create_group(fan_kobj,&fan_attr_group);
	if(ret)
	{
		printk(KERN_ERR "%s: failed to create fan group attributes\n", __func__);
	}

    //Begin [0016004715,fix the factory test result to panic,20181121]
    INIT_DELAYED_WORK(&fan_delay_work,  fan_read_workqueue);
    //End [0016004715,fix the factory test result to panic,20181121]
    fan_set_enable(false);
	//fan_enable_reg(nubia_fan, false);
	return 0;




	regulator_put:
        
    if (fan->pwr_reg) {
		regulator_put(fan->pwr_reg);
		fan->pwr_reg = NULL;
       }
	    devm_kfree(&i2c->dev, fan);
    fan = NULL;
	return ret;
}

static int fan_remove(struct i2c_client *i2c)
{
	struct fan *fan = i2c_get_clientdata(i2c);

	pr_info("%s remove\n", __func__);
   
	sysfs_remove_group(fan_kobj,&fan_attr_group);
	fan_kobj = NULL;
	
	if (gpio_is_valid(fan->reset_gpio))
		devm_gpio_free(&i2c->dev, fan->reset_gpio);

	fan_enable_reg(fan, false);
	
	if (fan->pwr_reg) {
		regulator_put(fan->pwr_reg);
		fan->pwr_reg = NULL;
	}

    devm_kfree(&i2c->dev, fan);
    fan = NULL;
	return 0;
}

static const struct i2c_device_id fan_i2c_id[] = {};


static const struct of_device_id of_match[] = {
        { .compatible = "nubia_fan_i2c" },
        { }
};

static struct i2c_driver fan_i2c_driver = {
	.driver = {
		.name = "nubia_fan",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(of_match),
	},
	.probe = fan_probe,
	.remove = fan_remove,
	.id_table = fan_i2c_id,
};

static int __init fan_init(void)
{
    int ret = 0;

    ret = i2c_add_driver(&fan_i2c_driver);
	
    if(ret){
        pr_err("fail to add fan device into i2c\n");
        return ret;
    }
	
	//printk(KERN_ERR "%d: failed to create fan group attributes\n", ret);
    return 0;

}

static void __exit fan_exit(void)
{
    i2c_del_driver(&fan_i2c_driver);
}

module_init(fan_init);
module_exit(fan_exit);

MODULE_AUTHOR("Fan, Inc.");
MODULE_DESCRIPTION("Fan Driver");
MODULE_LICENSE("GPL v2");

