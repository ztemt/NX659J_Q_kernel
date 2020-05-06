/*
 * Platform data for the NXP PCA9468 battery charger driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _PCA9468_CHARGER_H_
#define _PCA9468_CHARGER_H_

struct pca9468_platform_data {
	unsigned int	irq_gpio;	/* GPIO pin that's connected to INT# */
	unsigned int	iin_cfg;	/* Input Current Limit - uA unit */
	unsigned int 	ichg_cfg;	/* Charging Current - uA unit */
	unsigned int	v_float;	/* V_Float Voltage - uV unit */
	unsigned int 	iin_topoff;	/* Input Topoff current -uV unit */
	unsigned int 	snsres;		/* Current sense resister, 0 - 5mOhm, 1 - 10mOhm */
	unsigned int 	fsw_cfg; 	/* Switching frequency, refer to the datasheet, 0 - 833kHz, ... , 3 - 980kHz */
	unsigned int	ntc_th;		/* NTC voltage threshold : 0~2.4V - uV unit */
	unsigned int	sub_i2c_addr;	/* Sub charger I2C slave address - 7bit*/
	unsigned int	chg_mode;	/* Default charging mode, 0 - No direct charging, 1 - 2:1 charging mode, 2 - 4:1 charging mode */
	unsigned int	parallel;	/* Support parallel charging or not,  0 - Not support, 1 - Support */
	unsigned int	parallel_th;	/* parallel charging input current threshold - uA unit */
};

#endif
