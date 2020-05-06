/*
 * nubia_disp_preference.c - nubia lcd display color enhancement and temperature setting
 *	      Linux kernel modules for mdss
 *
 * Copyright (c) 2015 nubia <nubia@nubia.com.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 * Supports NUBIA lcd display color enhancement and color temperature setting
 */

/*------------------------------ header file --------------------------------*/
#include "nubia_disp_preference.h"
#include "nubia_dp_preference.h"
#include "dp_debug.h"
#include <linux/delay.h>

#define CONFIG_NUBIA_HDMI_NODE_FEATURE

static struct dsi_display *nubia_display=NULL;
/*------------------------------- variables ---------------------------------*/
static struct kobject *enhance_kobj = NULL;
struct nubia_disp_type nubia_disp_val = {
	.en_cabc = 1,
	.cabc = CABC_OFF,
#ifdef CONFIG_NUBIA_AOD_HBM_MODE
	.en_aod_mode = 1,
	.aod_mode = AOD_OFF,
	.en_hbm_mode = 1,
	.hbm_mode =HBM_OFF,
#endif
#ifdef CONFIG_NUBIA_DFPS_SWITCH
	.en_dfps = 1,
	.dfps = DFPS_90,
	.en_fps_change =1,
/**
*** after we currect the fps, we must write the back value to
*** register when the panel resume
***/
	.fps_60 = 0x5B,
	.fps_90 = 0x3C,
	.fps_120 = 0x3A,
	.fps_144 = 0x30,
	.panel_type = DFPS_120
#endif
};

int fps_store = 90;
int fps_temp = 90;
extern bool enable_flag;
#ifdef CONFIG_NUBIA_DFPS_SWITCH
extern bool is_66451_panel;
#endif 
#ifdef CONFIG_NUBIA_HDMI_NODE_FEATURE
extern struct _select_sde_edid_info select_sde_edid_info;
extern char edid_mode_best_info[32];
extern struct dp_debug_private *debug_node;
#endif

static ssize_t cabc_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
		if (nubia_disp_val.en_cabc)
				return snprintf(buf, PAGE_SIZE, "%d\n", nubia_disp_val.cabc);
		else
				return snprintf(buf, PAGE_SIZE, "NULL\n");
}

static ssize_t cabc_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t size)
{
	uint32_t val = 0;
	int ret = 0;
	if(!nubia_disp_val.en_cabc) {
		NUBIA_DISP_ERROR("no cabc\n");
		return size;
	}
	sscanf(buf, "%d", &val);
	if ((val != CABC_OFF) && (val != CABC_LEVEL1) &&
		(val != CABC_LEVEL2) && (val != CABC_LEVEL3)) {
			NUBIA_DISP_ERROR("invalid cabc val = %d\n", val);
			return size;
		}
	NUBIA_DISP_INFO("cabc value = %d\n", val);

	if(nubia_display == NULL)
		return size;

#ifdef CONFIG_NUBIA_DISP_PREFERENCE	
	ret = nubia_dsi_panel_cabc(nubia_display->panel, val);
	if (ret == 0) {
		nubia_disp_val.cabc = val;
		NUBIA_DISP_INFO("success to set cabc as = %d\n", val);
	}

#endif
	return size;
}

#ifdef CONFIG_NUBIA_AOD_HBM_MODE
static ssize_t aod_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
		if (nubia_disp_val.en_aod_mode)
				return snprintf(buf, PAGE_SIZE, "%d\n", nubia_disp_val.aod_mode);
		else
				return snprintf(buf, PAGE_SIZE, "NULL\n");
}

static ssize_t aod_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t size)
{
	uint32_t val = 0;
	int ret = 0;
	if(!nubia_disp_val.en_aod_mode) {
		NUBIA_DISP_ERROR("no aod_mode\n");
		return size;
	}
	sscanf(buf, "%d", &val);
	if ((val != AOD_OFF) && (val != AOD_ON)) {
			NUBIA_DISP_ERROR("invalid aod_mode val = %d\n", val);
			return size;
		}
	NUBIA_DISP_INFO("aod_mode value = %d\n", val);

	if(nubia_display == NULL)
		return size;
	
	ret = nubia_dsi_panel_aod(nubia_display->panel, val);
	if (ret == 0) {
		nubia_disp_val.aod_mode = val;
		NUBIA_DISP_INFO("success to set aod_mode as = %d\n", val);
	}

	return size;
}

static ssize_t hbm_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
		if (nubia_display == NULL){
				NUBIA_DISP_ERROR("no nubia_display node!\n");
				return -EINVAL;
		}

		return snprintf(buf, PAGE_SIZE, "%d\n", nubia_disp_val.hbm_mode);
}

static ssize_t hbm_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t size)
{
	uint32_t val = 0;
	int ret = 0;
	if(nubia_display == NULL) {
		NUBIA_DISP_ERROR("no hbm_mode\n");
		return size;
	}
	sscanf(buf, "%d", &val);
	if ((val != HBM_OFF) && (val != HBM_ON) && (val != HBM_FP_OFF) && (val != HBM_FP_ON)) {
			NUBIA_DISP_ERROR("invalid hbm_mode val = %d\n", val);
			return size;
	}
	if(!is_66451_panel){
		NUBIA_DISP_INFO("r66455 un-support hbm mode \n");
		return size;
	}
	NUBIA_DISP_INFO("hbm_mode value = %d\n", val);
	
	ret = nubia_dsi_panel_hbm(nubia_display->panel, val);
	if (ret == 0) {
		nubia_disp_val.hbm_mode = val;
		NUBIA_DISP_INFO("success to set hbm_mode as = %d\n", val);
	}

	return size;
}
#endif

#ifdef CONFIG_NUBIA_DFPS_SWITCH
static ssize_t dfps_show(struct kobject *kobj,
		 struct kobj_attribute *attr, char *buf)
{

	return snprintf(buf, PAGE_SIZE, "%d\n", nubia_disp_val.dfps);
}

static ssize_t dfps_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t size)
{
	return size;
/*we use qcom change fps, dsi_panel_post_switch()*/
#if 0
	uint32_t val = 0;
	int ret = 0;
	if(!nubia_disp_val.en_dfps) {
		NUBIA_DISP_ERROR("no dpfs\n");
		return size;
	}
	sscanf(buf, "%d", &val);
	if (fps_store == val)
		return size;

	if ((val != DFPS_60) && (val != DFPS_90) && (val != DFPS_120) && (val != DFPS_144)) {
		NUBIA_DISP_ERROR("invalid dfps val = %d\n", val);
		return size;
	}

	NUBIA_DISP_INFO("dfps value = %d, panel_type = %d\n", val, nubia_disp_val.panel_type);

	/* r66455 not support to switch fps*/
	if(nubia_display == NULL || !is_66451_panel)
		return size;

	/* before B2-2, the panel just support up to 120Hz*/
	if(nubia_disp_val.panel_type == DFPS_120){
		if(val == DFPS_144){
			NUBIA_DISP_INFO("this panel just support up to 120Hz");
			return size;
		}
	}

	ret = nubia_dsi_panel_dfps(nubia_display->panel, val);
	if (ret == 0) {
		nubia_disp_val.dfps = val;
		NUBIA_DISP_INFO("success to set dfps = %d\n", val);
	}

	fps_store = val;
	
	fps_temp = fps_store;

	return size;
#endif
}

/***
**** the panel osc has residual, so we must 
**** dynamic correct the fps
**** value: different value to register for different fps
****/
void nubia_dynamic_correct_fps(struct mipi_dsi_device *dsi, u8 value)
{
	u8 cmd1[1] = {0x00};
	u8 cmd2[20] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x3C, 0x00, 0x3C, 0x00, 0x3C, 0x00, 0x3C, 0x05, 0x00};

	cmd2[9] = value;
	cmd2[11] = value;
	cmd2[13] = value;
	cmd2[15] = value;
	cmd2[17] = value;
	dsi_panel_write_data(dsi, 0xB0, cmd1, sizeof(cmd1));
	dsi_panel_write_data(dsi, 0xD8, cmd2, sizeof(cmd2));
}

static ssize_t fps_change_show(struct kobject *kobj,
		 struct kobj_attribute *attr, char *buf)
{
	/* the dynamic fps currection just for 66451*/
	if(is_66451_panel == 0){
		pr_err("the fps currect function just for 66451 \n");
		return snprintf(buf, PAGE_SIZE, "%s\n","the fps currect function just for 66451");;
	}

	if(nubia_disp_val.dfps == 60)
		nubia_disp_val.fps_change = nubia_disp_val.fps_60;
	else if(nubia_disp_val.dfps == 90)
		nubia_disp_val.fps_change = nubia_disp_val.fps_90;
	else if(nubia_disp_val.dfps == 120)
		nubia_disp_val.fps_change = nubia_disp_val.fps_120;
	else
		nubia_disp_val.fps_change = nubia_disp_val.fps_144;

	return snprintf(buf, PAGE_SIZE, "%x\n", nubia_disp_val.fps_change);

}

static ssize_t fps_change_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t size)
{
	uint32_t val = 0;
	struct mipi_dsi_device *dsi = &nubia_display->panel->mipi_device;
	if(!nubia_disp_val.en_fps_change) {
		NUBIA_DISP_ERROR("no fps_change\n");
		return size;
	}

	if(nubia_display->panel->panel_initialized == false){
		pr_err("panel[%s] not ready\n", nubia_display->panel->name);
		return size;
	}

	sscanf(buf, "%x", &val);

	printk("%s: fps_store = %d, val = %d \n", __func__, fps_store, val);
	/*
	** for different fps, the value range is different
	*/
	switch(nubia_disp_val.dfps){
		case DFPS_60:
			if((val >= 0x59) && (val <= 0x5C))
				nubia_disp_val.fps_60 = val;
			break;
		case DFPS_90:
			if((val >= 0x3B) && (val <= 0x3D))
				nubia_disp_val.fps_90 = val;
			break;
		case DFPS_120:
			if((val >= 0x39) && (val <= 0x3B))
				nubia_disp_val.fps_120 = val;
			break;
		case DFPS_144:
			if((val >= 0x30) && (val <= 0x32))
				nubia_disp_val.fps_144 = val;
			break;
		default:
			pr_err("%s: the value is error, val = %d \n ", __func__, val);
			return size;
	}
	nubia_dynamic_correct_fps(dsi, val);
	return size;

}

static ssize_t panel_type_show(struct kobject *kobj,
		 struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", nubia_disp_val.panel_type);
}

static ssize_t panel_type_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t size)
{
	return size;
}

#endif

#ifdef CONFIG_NUBIA_DEBUG_LCD_REG
u8 rx_buf[1024] = {0};
u16 rx_len = 0;

u8 hex_to_char(u8 bChar)	
{	
	if((bChar >= 0x30) && (bChar <= 0x39))	
		bChar -= 0x30;	
	else if((bChar >= 0x41) && (bChar <= 0x46))
                bChar -= 0x37;
        else if((bChar >= 0x61) && (bChar <= 0x66))	
     	        bChar -= 0x57;	
        return bChar;	
}

unsigned char char_to_hex(unsigned char bChar)
{	
	if((bChar >=0) && (bChar<=9))
               bChar += 0x30;
        else if((bChar >=10) && (bChar<=15))
 	       bChar += 0x37;
        else bChar = 0xFF;
        return bChar;
}

static void kernel_str_int(const char *buf, unsigned char *tx_buf)
{
        int i = 0, j = 0, k = 0; 
        char *p;
        p = (char *)buf;
        for(i=0; *p != '\n'; i++){	
		if(*p == ' '){	
			j++;	
			k = 0;	
		}else{	
			tx_buf[j] = (tx_buf[j] << 4) | hex_to_char(*p);	
			k++;	
		}	
		p++;	
       }	
}

static ssize_t lcd_reg_show(struct kobject *kobj,	
		struct kobj_attribute *attr, char *buf)	
{	
	int i = 0, j = 0;	
	unsigned char val = 0;	
	for(i=0; i< strlen(rx_buf) / sizeof(char); i++){	
		val = (rx_buf[i] & 0xF0) >> 4;	
		buf[j++] = char_to_hex(val);
		val = rx_buf[i] & 0x0F;
		buf[j++] = char_to_hex(val);
		buf[j++] = ' ';	
	}	
	buf[j] = '\n';	
	//for(i=0; i<j-1; i++) 	
	//       printk("%c", buf[i]); 
	return j;
}

static ssize_t lcd_reg_store(struct kobject *kobj,	
	struct kobj_attribute *attr, const char *buf, size_t size)
{	
	int rc = 0 , i = 0;	
	u8 tx_buf[200] = {0};
	u8 *data;
	struct mipi_dsi_device *dsi = &nubia_display->panel->mipi_device;
	if(!dsi){
		pr_err("%s: lcd reg store error, dsi is NULL\n", __func__);
		return 0;
	}

	kernel_str_int(buf, tx_buf);
	//printk("read tx_buf[0] = %d, tx_buf[1] = %d, tx_buf[2] = %d", tx_buf[0] , tx_buf[1], tx_buf[2] ); 
	if(tx_buf[0] == 1){
		rx_len = tx_buf[2];
		data = (u8*)kzalloc(rx_len, GFP_KERNEL);
		printk("read tx_buf[0] = %d, tx_buf[1] = %d, tx_buf[2] = %d", tx_buf[0] , tx_buf[1], tx_buf[2] );
		rc = dsi_panel_read_data(dsi, tx_buf[1], data, tx_buf[2]);
		printk(">>>>>>> rc = %d \n", rc);
		for(i=0; i<rc; i++){
			printk("%d = %x \n",i, data[i]);
		}
		if (rc > 0)
			memcpy(rx_buf, data, rx_len);
		
		//printk("rx_buf[0] =%d, rx_buf[1] = %d \n", data[0], data[1]); 
		//printk("rx_buf[0] =%d, rx_buf[1] = %d \n", rx_buf[0], rx_buf[1]); 
		kfree(data);
		data = NULL;
		if(rc<0){
			pr_err("%s: read panel data error \n", __func__);
			return 0;
		}	
	}else{	
		//printk("read tx_buf[0] = %d, tx_buf[1] = %d, tx_buf[2] = %d, tx_buf[3] = %d", tx_buf[0] , tx_buf[1], tx_buf[2], tx_buf[3] ); 
		rc = dsi_panel_write_data(dsi, tx_buf[1], &tx_buf[3], tx_buf[2]);
	}
	if(rc<0){
		pr_err("%s: write panel data error \n", __func__);
		return 0;
	}
	return size;
}
#endif

#ifdef CONFIG_NUBIA_HDMI_NODE_FEATURE
static ssize_t dp_debug_hpd_show(struct kobject *kobj,
		 struct kobj_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t dp_debug_hpd_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t size)
{
	int const hpd_data_mask = 0x7;
	int hpd = 0;

	if (!debug_node)
		return -ENODEV;

    sscanf(buf, "%d", &hpd);
	printk("%s:  hpd = %d \n", __func__, hpd);
	
	hpd &= hpd_data_mask;
	debug_node->hotplug = !!(hpd & BIT(0));

	debug_node->dp_debug.psm_enabled = !!(hpd & BIT(1));

	/*
	 * print hotplug value as this code is executed
	 * only while running in debug mode which is manually
	 * triggered by a tester or a script.
	 */
	DP_INFO("%s\n", debug_node->hotplug ? "[CONNECT]" : "[DISCONNECT]");
	if(hpd == 0)
	{
		select_sde_edid_info.edid_hot_plug = true;
	}

	debug_node->hpd->simulate_connect(debug_node->hpd, debug_node->hotplug);

	return size;
}

static ssize_t edid_modes_show(struct kobject *kobj,
		 struct kobj_attribute *attr, char *buf)
{
		char *buf_edid;
		u32 len = 0, ret = 0, max_size = SZ_4K;
		int rc = 0;
	
		buf_edid = kzalloc(SZ_4K, GFP_KERNEL);
		if (ZERO_OR_NULL_PTR(buf_edid)) {
			rc = -ENOMEM;
			goto error;
		}
	
		ret = snprintf(buf_edid, max_size, "%s", edid_mode_best_info);
		len = snprintf(buf_edid + ret, max_size, "%s", select_sde_edid_info.edid_mode_info);
		
		NUBIA_DISP_INFO("--- len = %d, edid_mode_best_info = %s	 select_sde_edid_info.edid_mode_info = %s\n", 
			len, edid_mode_best_info, select_sde_edid_info.edid_mode_info);
				
		len = sprintf(buf, "%s", buf_edid);
 		kfree(buf_edid);
	
		return len;
	error:
		return rc;

}

static ssize_t edid_modes_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t size)
{
	int hdisplay = 0, vdisplay = 0, vrefresh = 0, aspect_ratio;

	if (!debug_node)
		return -ENODEV;

	if (sscanf(buf, "%d %d %d %d", &hdisplay, &vdisplay, &vrefresh,
				&aspect_ratio) != 4)
		goto clear;
	NUBIA_DISP_INFO("hdisplay = %d, vdisplay = %d, vrefresh = %d, aspect_ratio = %d\n", 
		hdisplay, vdisplay, vrefresh, aspect_ratio);
		
	if (!hdisplay || !vdisplay || !vrefresh)
		goto clear;
	select_sde_edid_info.node_control = true;
	debug_node->dp_debug.debug_en = true;
	debug_node->dp_debug.hdisplay = hdisplay;
	debug_node->dp_debug.vdisplay = vdisplay;
	debug_node->dp_debug.vrefresh = vrefresh;
	debug_node->dp_debug.aspect_ratio = aspect_ratio;
	/*store the select fps and resulation of edid_mode_info*/
	memset(edid_mode_best_info, 0x00, 32);
	snprintf(edid_mode_best_info, 32,"%dx%d %d %d\n",hdisplay, vdisplay,vrefresh, aspect_ratio);
	
	select_sde_edid_info.edid_mode_store = true;
	goto end;
clear:
	NUBIA_DISP_INFO("clearing debug modes\n");
	debug_node->dp_debug.debug_en = false;
end:
	return size;
}
#endif

static struct kobj_attribute lcd_disp_attrs[] = {
	__ATTR(cabc,        0664, cabc_show,       cabc_store),
#ifdef CONFIG_NUBIA_AOD_HBM_MODE
	__ATTR(aod_mode,        0664, aod_show,       aod_store),
	__ATTR(hbm_mode,        0664, hbm_show,       hbm_store),
#endif
#ifdef CONFIG_NUBIA_DEBUG_LCD_REG
	 __ATTR(lcd_reg,     0664, lcd_reg_show, lcd_reg_store),
#endif
#ifdef CONFIG_NUBIA_DFPS_SWITCH
	__ATTR(dfps,		0664,dfps_show,dfps_store),
	__ATTR(fps_change,		0664,fps_change_show,fps_change_store),
	__ATTR(panel_type,		0664,panel_type_show,panel_type_store),
#endif

#ifdef CONFIG_NUBIA_HDMI_NODE_FEATURE
    __ATTR(edid_modes,        0664, edid_modes_show,       edid_modes_store),
    __ATTR(hpd,        0664,        dp_debug_hpd_show,     dp_debug_hpd_store), 
#endif

};

void nubia_set_dsi_ctrl(struct dsi_display *display)
{
	NUBIA_DISP_INFO("start\n");

		nubia_display = display;
}

static int __init nubia_disp_preference_init(void)
{
	int retval = 0;
	int attr_count = 0;

	NUBIA_DISP_INFO("start\n");

	enhance_kobj = kobject_create_and_add("lcd_enhance", kernel_kobj);

	if (!enhance_kobj) {
		NUBIA_DISP_ERROR("failed to create and add kobject\n");
		return -ENOMEM;
	}

	/* Create attribute files associated with this kobject */
	for (attr_count = 0; attr_count < ARRAY_SIZE(lcd_disp_attrs); attr_count++) {
		retval = sysfs_create_file(enhance_kobj, &lcd_disp_attrs[attr_count].attr);
		if (retval < 0) {
			NUBIA_DISP_ERROR("failed to create sysfs attributes\n");
			goto err_sys_creat;
		}
	}
	NUBIA_DISP_INFO("success\n");

	return retval;

err_sys_creat:
//#ifdef CONFIG_NUBIA_SWITCH_LCD
	//cancel_delayed_work_sync(&nubia_disp_val.lcd_states_work);
//#endif
	for (--attr_count; attr_count >= 0; attr_count--)
		sysfs_remove_file(enhance_kobj, &lcd_disp_attrs[attr_count].attr);

	kobject_put(enhance_kobj);
	return retval;
}

static void __exit nubia_disp_preference_exit(void)
{
	int attr_count = 0;

	for (attr_count = 0; attr_count < ARRAY_SIZE(lcd_disp_attrs); attr_count++)
		sysfs_remove_file(enhance_kobj, &lcd_disp_attrs[attr_count].attr);

	kobject_put(enhance_kobj);

}

MODULE_AUTHOR("NUBIA LCD Driver Team Software");
MODULE_DESCRIPTION("NUBIA LCD DISPLAY Color Saturation and Temperature Setting");
MODULE_LICENSE("GPL");
module_init(nubia_disp_preference_init);
module_exit(nubia_disp_preference_exit);
