#ifndef _NUBIA_DP_PREFERENCE_
#define _NUBIA_DP_PREFERENCE_

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/component.h>
#include <linux/of_irq.h>
#include <linux/extcon.h>
#include <linux/soc/qcom/fsa4480-i2c.h>
#include <linux/usb/usbpd.h>

#include "sde_connector.h"

#include "msm_drv.h"
#include "dp_hpd.h"
#include "dp_parser.h"
#include "dp_power.h"
#include "dp_catalog.h"
#include "dp_aux.h"
#include "dp_link.h"
#include "dp_panel.h"
#include "dp_ctrl.h"
#include "dp_audio.h"
#include "dp_display.h"
#include "sde_hdcp.h"
#include "dp_debug.h"
#include "sde_dbg.h"

#define  CONFIG_NUBIA_HDMI_FEATURE


#ifdef CONFIG_NUBIA_HDMI_FEATURE
/**
* h :           the horizontal lenght
* v :           the vertical lenght
* fps:          frame per second
ratio:          the ratio for h / v
hdmi_connected: hdmi connect or disconnect
node_control:   if the edid_modes node to control the monitor fps and resulation
                we must release control access to usersapce
edid_mode_info: store the edid info for userspace to get used
**/
struct _select_sde_edid_info{
	int h;
	int v;
	int fps;
	int ratio;
	bool hdmi_connected;
	bool node_control;
	bool edid_mode_store;
	bool edid_hot_plug;
	char *edid_mode_info;
};
#endif

/* ------------------------- General Macro Definition ------------------------*/

/* ----------------------------- Structure ----------------------------------*/

/* ------------------------- Function Declaration ---------------------------*/
void nubia_dp_preference(void);
void nubia_set_usbdp_ctrl(struct dp_aux *display);
#endif
