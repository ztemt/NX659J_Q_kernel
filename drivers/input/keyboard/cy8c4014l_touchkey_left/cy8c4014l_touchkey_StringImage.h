#ifndef __CYPRESS8C40141_TOUCHKEY_STRINGIMAGE_H__
#define __CYPRESS8C40141_TOUCHKEY_STRINGIMAGE_H__

#ifdef TARGET_NUBIA_NX629J_V1S
#define CURRENT_NEW_FIRMWARE_VER    0x1B

#define LINE_CNT_0 126

const char *stringImage_left_0[]={
#include "CY8C4014LQI-421_Bootloadable_NX659_right_1B.cyacd"
};

#else
#define CURRENT_NEW_FIRMWARE_VER    0x1B

#define LINE_CNT_0 126

const char *stringImage_left_0[]={
#include "CY8C4014LQI-421_Bootloadable_NX659_right_1B.cyacd"
};

#endif

#endif
