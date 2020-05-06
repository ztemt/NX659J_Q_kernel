#ifndef _NUBIA_DISP_PREFERENCE_
#define _NUBIA_DISP_PREFERENCE_

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include "sde_kms.h"
#include "dsi_panel.h"
#include "dsi_display.h"

/* ------------------------- General Macro Definition ------------------------*/
#define NUBIA_DISP_COLORTMP_DEBUG        0

enum{
	CABC_OFF = 23,
	CABC_LEVEL1,
	CABC_LEVEL2,
	CABC_LEVEL3
};

enum{
	AOD_OFF =23 ,
	AOD_ON
};

enum{
	HBM_FP_OFF = 0,
	HBM_FP_ON,
	HBM_OFF = 23,
	HBM_ON
};

enum{
	DFPS_60 = 60,
	DFPS_62 = 62,
	DFPS_90 = 90,
	DFPS_120 = 120,
	DFPS_144 = 144
};
#define NUBIA_DISP_LOG_TAG "ZtemtDisp"
#define NUBIA_DISP_LOG_ON

#ifdef  NUBIA_DISP_LOG_ON
#define NUBIA_DISP_ERROR(fmt, args...) printk(KERN_ERR "[%s] [%s: %d] "  fmt, \
	NUBIA_DISP_LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define NUBIA_DISP_INFO(fmt, args...) printk(KERN_ERR "[%s] [%s: %d] "  fmt, \
	NUBIA_DISP_LOG_TAG, __FUNCTION__, __LINE__, ##args)

    #ifdef  NUBIA_DISP_DEBUG_ON
#define  NUBIA_DISP_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s: %d] "  fmt, \
	NUBIA_DISP_LOG_TAG, __FUNCTION__, __LINE__, ##args)
    #else
#define NUBIA_DISP_DEBUG(fmt, args...)
    #endif
#else
#define NUBIA_DISP_ERROR(fmt, args...)
#define NUBIA_DISP_INFO(fmt, args...)
#define NUBIA_DISP_DEBUG(fmt, args...)
#endif

/* ----------------------------- Structure ----------------------------------*/
struct nubia_disp_type{
  int en_cabc;
  unsigned int cabc;
#ifdef CONFIG_NUBIA_AOD_HBM_MODE
  int en_aod_mode;
  unsigned int aod_mode;
  int en_hbm_mode;
  unsigned int hbm_mode;
#endif
#ifdef CONFIG_NUBIA_DFPS_SWITCH
  int en_dfps;
  unsigned int dfps;
  int en_fps_change;
	u8 fps_change;

/**
*** after we currect the fps, we must write the back value to
*** register when the panel resume
***/
	int fps_60;
	int fps_90;
	int fps_120;
	int fps_144;

	/**
	*** NX659 has three panel, R66455 for NX629, the panel for 120Hz, the panel for 144Hz
	**/
	u8 panel_type;
#endif
};

/* ------------------------- Function Declaration ---------------------------*/
void nubia_disp_preference(void);
void nubia_set_dsi_ctrl(struct dsi_display *display);
#endif
