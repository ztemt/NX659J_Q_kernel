#ifndef __CYPRESS8C40141_TOUCHKEY_FIRMWARE_UPDATE_H__
#define __CYPRESS8C40141_TOUCHKEY_FIRMWARE_UPDATE_H__

/* CYPRESS4000 Touchkey */
#define CYPRESS4000_TOUCHKEY_ENTER_BOOTLOADER_MODE		0x02   //enter bootloader mode reg
#define CYPRESS4000_TOUCHKEY_ENTER_BOOTLOADER_COMMON_VALUE 0xAA //common value
#define CYPRESS_BOOTLOADER_MODE_I2C_ADDR   (0x6C>>1)
#define CYPRESS_I2C_ADDR   0x10


#define NO_FLASH_ARRAY_DATA 0
#define MAX_FLASH_ARRAYS    4
#define CYPRESS4000_I2C_RETRY_TIMES 3

extern int cypress_firmware_update(struct i2c_client *client,const char *bootloadImagePtr[],unsigned int lineCount);
#endif
