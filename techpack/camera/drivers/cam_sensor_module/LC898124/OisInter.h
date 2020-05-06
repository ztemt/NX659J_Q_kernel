/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef MSM_OIS_H
#define MSM_OIS_H

#define MSM_DEBUG_GYRO_OFFSET

/*ZTEMT: fengxun add for 627--------Start*/
#define		INT16	short
#define		INT32	int
#define		INT64	long
#define		UINT8	unsigned char
#define		UINT16	unsigned short
#define		UINT32	unsigned int
#define		UINT64	unsigned long

typedef struct {
	UINT8 Vendor;
	UINT8 User;
	UINT8 Model;
	UINT8 Version;
	UINT8 Reserve0;
	UINT8 Reserve1;
	UINT8 ActType;
	UINT8 GyroType;
} DSPVER;

typedef struct {
	UINT16	Cmd ;
	const UINT8* DataPM;
	UINT32 LengthPM;
	UINT32 Parity;
	const UINT8* DataDM;
	UINT32 LengthDM;
}DOWNLOAD_TBL ;


//****************************************************
//  DEFINES
//****************************************************
#define     ACT_HOZEL_OZ31A401      0x01
#define     GYRO_LSM6DSM            0x02

#define     BURST_LENGTH_PM ( 12*5 ) 	// 60 必ず5の倍数で設定すること。最大64Byteまで
#define     BURST_LENGTH_DM ( 10*6 ) 	// 60 必ず6の倍数で設定すること。最大64Byteまで
#define     BURST_LENGTH BURST_LENGTH_PM 	

#define		DmCheck_CheckSumDMA				0x53C
#define		DmCheck_CheckSumDMB				0x540

#define 	PmCheck_CheckSum				0x514
#define 	PmCheck_EndFlag					0x518

#define		CommandDecodeTable				0x8568
#define		CommandDecodeTable_08			(0x0020 + CommandDecodeTable)

#define		CMD_IO_ADR_ACCESS				0xC000				// IO Write Access
#define		CMD_IO_DAT_ACCESS				0xD000				// IO Read Access
#define		CMD_REMAP						0xF001				// Remap
#define		CMD_REBOOT						0xF003				// Reboot
#define		CMD_RETURN_TO_CENTER			0xF010				// Center Servo ON/OFF choose axis

#define 	SYSDSP_REMAP					0xD000AC

#define 	ROMINFO							0xE0500C

// HallFilterCoeff.h  DM_HFC_t
#define		HallFilterCoeffX				0x8090
#define		HallFilterCoeffX_HXIGAIN		(0x0000 + HallFilterCoeffX)
#define		HallFilterCoeffX_GYROXOUTGAIN	(0x0004 + HallFilterCoeffX_HXIGAIN)
#define		HallFilterCoeffX_HXOFFGAIN		(0x0004 + HallFilterCoeffX_GYROXOUTGAIN)

// HallFilterCoeff.h  DM_HFC_t
#define		HallFilterCoeffY				0x812c
#define		HallFilterCoeffY_HYIGAIN		(0x0000 + HallFilterCoeffY)
#define		HallFilterCoeffY_GYROYOUTGAIN	(0x0004 + HallFilterCoeffY_HYIGAIN)
#define		HallFilterCoeffY_HYOFFGAIN		(0x0004 + HallFilterCoeffY_GYROYOUTGAIN)

// GyroFilterCoeff.h  DM_GFC_t
#define		GyroFilterTableX				0x8270
#define		GyroFilterTableX_gx45x			(0x0000 + GyroFilterTableX)
#define		GyroFilterTableX_gx45y			(0x0004 + GyroFilterTableX_gx45x)

// GyroFilterCoeff.h  DM_GFC_t
#define		GyroFilterTableY				0x82D0
#define		GyroFilterTableY_gy45y			(0x0000 + GyroFilterTableY)
#define		GyroFilterTableY_gy45x			(0x0004 + GyroFilterTableY_gy45y)

// GyroFilterDelay.h GYRO_RAM_COMMON_t
#define		GYRO_RAM_COMMON					0x0258
#define		GYRO_RAM_GX_ADIDAT				(0x0000 + GYRO_RAM_COMMON)
#define		GYRO_RAM_GY_ADIDAT				(0x0004 + GYRO_RAM_GX_ADIDAT)
#define		GYRO_RAM_SINDX					(0x0004 + GYRO_RAM_GY_ADIDAT)
#define		GYRO_RAM_SINDY					(0x0004 + GYRO_RAM_SINDX)
#define		GYRO_RAM_GXLENSZ				(0x0004 + GYRO_RAM_SINDY)
#define		GYRO_RAM_GYLENSZ				(0x0004 + GYRO_RAM_GXLENSZ)
#define		GYRO_RAM_GXOX_OUT				(0x0004 + GYRO_RAM_GYLENSZ)
#define		GYRO_RAM_GYOX_OUT				(0x0004 + GYRO_RAM_GXOX_OUT)
#define		GYRO_RAM_GXOFFZ					(0x0004 + GYRO_RAM_GYOX_OUT)
#define		GYRO_RAM_GYOFFZ					(0x0004 + GYRO_RAM_GXOFFZ)
#define		GYRO_RAM_LIMITX					(0x0004 + GYRO_RAM_GYOFFZ)
#define		GYRO_RAM_LIMITY					(0x0004 + GYRO_RAM_LIMITX)
#define		GYRO_RAM_GZ_ADIDAT				(0x0004 + GYRO_RAM_LIMITY)
#define		GYRO_RAM_Reserve				(0x0004 + GYRO_RAM_GZ_ADIDAT)
#define		GYRO_RAM_GYRO_Switch			(0x0004 + GYRO_RAM_Reserve)			// 1Byte
#define		GYRO_RAM_GYRO_AF_Switch			(0x0001 + GYRO_RAM_GYRO_Switch)		// 1Byte


#ifdef MSM_DEBUG_GYRO_OFFSET
union	WRDVAL{
	INT16	SsWrdVal ;
	UINT16	UsWrdVal ;
	UINT8	UcWrkVal[ 2 ] ;
	signed char		ScWrkVal[ 2 ] ;
	struct {
		UINT8	UcLowVal ;
		UINT8	UcHigVal ;
	} StWrdVal ;
} ;

union	DWDVAL {
	UINT32	UlDwdVal ;
	UINT16	UsDwdVal[ 2 ] ;
	struct {
		UINT16	UsLowVal ;
		UINT16	UsHigVal ;
	} StDwdVal ;
	struct {
		UINT8	UcRamVa0 ;
		UINT8	UcRamVa1 ;
		UINT8	UcRamVa2 ;
		UINT8	UcRamVa3 ;
	} StCdwVal ;
} ;

union	ULLNVAL {
	UINT64	UllnValue ;
	UINT32	UlnValue[ 2 ] ;
	struct {
		UINT32	UlLowVal ;
		UINT32	UlHigVal ;
	} StUllnVal ;
} ;

// Float Data Union
union	FLTVAL {
	float			SfFltVal ;
	UINT32	UlLngVal ;
	UINT16	UsDwdVal[ 2 ] ;
	struct {
		UINT16	UsLowVal ;
		UINT16	UsHigVal ;
	} StFltVal ;
} ;

typedef union WRDVAL	UnWrdVal ;
typedef union DWDVAL	UnDwdVal;
typedef union ULLNVAL	UnllnVal;
typedef union FLTVAL	UnFltVal ;

typedef struct {
	INT32				SiSampleNum ;			// Measure Sample Number
	INT32				SiSampleMax ;			// Measure Sample Number Max

	struct {
		INT32			SiMax1 ;				// Max Measure Result
		INT32			SiMin1 ;				// Min Measure Result
		UINT32	UiAmp1 ;				// Amplitude Measure Result
		INT64		LLiIntegral1 ;			// Integration Measure Result
		INT64		LLiAbsInteg1 ;			// Absolute Integration Measure Result
		INT32			PiMeasureRam1 ;			// Measure Delay RAM Address
	} MeasureFilterA ;

	struct {
		INT32			SiMax2 ;				// Max Measure Result
		INT32			SiMin2 ;				// Min Measure Result
		UINT32	UiAmp2 ;				// Amplitude Measure Result
		INT64		LLiIntegral2 ;			// Integration Measure Result
		INT64		LLiAbsInteg2 ;			// Absolute Integration Measure Result
		INT32			PiMeasureRam2 ;			// Measure Delay RAM Address
	} MeasureFilterB ;
} MeasureFunction_Type ;

#define		MeasureFilterA_Delay			0x0308
				// MeasureFilter.h	MeasureFilter_Delay_Type
#define			MeasureFilterA_Delay_z11		(0x0000 + MeasureFilterA_Delay)
#define			MeasureFilterA_Delay_z12		(0x0004 + MeasureFilterA_Delay_z11)
#define			MeasureFilterA_Delay_z21		(0x0004 + MeasureFilterA_Delay_z12)
#define			MeasureFilterA_Delay_z22		(0x0004 + MeasureFilterA_Delay_z21)

#define		MeasureFilterB_Delay			0x0318
				// MeasureFilter.h	MeasureFilter_Delay_Type
#define			MeasureFilterB_Delay_z11		(0x0000 + MeasureFilterB_Delay)
#define			MeasureFilterB_Delay_z12		(0x0004 + MeasureFilterB_Delay_z11)
#define			MeasureFilterB_Delay_z21		(0x0004 + MeasureFilterB_Delay_z12)
#define			MeasureFilterB_Delay_z22		(0x0004 + MeasureFilterB_Delay_z21)

#define		WaitTimerData					0x035C
				// CommonLibrary.h  WaitTimer_Type
#define			WaitTimerData_UiWaitCounter		(0x0000 + WaitTimerData	)				// 0x06F4
#define			WaitTimerData_UiTargetCount		(0x0004 + WaitTimerData_UiWaitCounter)	// 0x06F8

#define 	ONE_MSEC_COUNT	20			// 20.223kHz * 20 ≒ 1ms


#define 	GYROF_NUM		2048			// 2048times

#define		MeasureFilterA_Coeff			0x8380
				// MeasureFilter.h  MeasureFilter_Type
#define			MeasureFilterA_Coeff_b1			(0x0000 + MeasureFilterA_Coeff)
#define			MeasureFilterA_Coeff_c1			(0x0004 + MeasureFilterA_Coeff_b1)
#define			MeasureFilterA_Coeff_a1			(0x0004 + MeasureFilterA_Coeff_c1)
#define			MeasureFilterA_Coeff_b2			(0x0004 + MeasureFilterA_Coeff_a1)
#define			MeasureFilterA_Coeff_c2			(0x0004 + MeasureFilterA_Coeff_b2)
#define			MeasureFilterA_Coeff_a2			(0x0004 + MeasureFilterA_Coeff_c2)

#define		MeasureFilterB_Coeff			0x8398
				// MeasureFilter.h  MeasureFilter_Type
#define			MeasureFilterB_Coeff_b1			(0x0000 + MeasureFilterB_Coeff)
#define			MeasureFilterB_Coeff_c1			(0x0004 + MeasureFilterB_Coeff_b1)
#define			MeasureFilterB_Coeff_a1			(0x0004 + MeasureFilterB_Coeff_c1)
#define			MeasureFilterB_Coeff_b2			(0x0004 + MeasureFilterB_Coeff_a1)
#define			MeasureFilterB_Coeff_c2			(0x0004 + MeasureFilterB_Coeff_b2)
#define			MeasureFilterB_Coeff_a2			(0x0004 + MeasureFilterB_Coeff_c2)

#define 	LOOPGAIN		1
#define 	THROUGH			2
#define 	NOISE			3
#define		OSCCHK			4

#define		StMeasureFunc					0x02B0
#define			StMeasFunc_SiSampleNum			(0x0000 + StMeasureFunc		)			// 0x0708
#define			StMeasFunc_SiSampleMax			(0x0004 + StMeasFunc_SiSampleNum)			// 0x070C

#define		StMeasureFunc_MFA				0x02B8
#define			StMeasFunc_MFA_SiMax1			(0x0000 + StMeasureFunc_MFA		)		// 0x0710
#define			StMeasFunc_MFA_SiMin1			(0x0004 + StMeasFunc_MFA_SiMax1	)		// 0x0714
#define			StMeasFunc_MFA_UiAmp1			(0x0004 + StMeasFunc_MFA_SiMin1	)		// 0x0718
#define			StMeasFunc_MFA_UiDUMMY1			(0x0004 + StMeasFunc_MFA_UiAmp1	)		// 0x071C
#define			StMeasFunc_MFA_LLiIntegral1		(0x0004 + StMeasFunc_MFA_UiDUMMY1)		// 0x0720
#define			StMeasFunc_MFA_LLiAbsInteg1		(0x0008 + StMeasFunc_MFA_LLiIntegral1)	// 0x0728	// 8Byte
#define			StMeasFunc_MFA_PiMeasureRam1	(0x0008 + StMeasFunc_MFA_LLiAbsInteg1)	// 0x0730	// 8Byte
#define			StMeasFunc_MFA_UiDUMMY2			(0x0004 + StMeasFunc_MFA_PiMeasureRam1)	// 0x0734

#define		StMeasureFunc_MFB				0x02E0
#define			StMeasFunc_MFB_SiMax2			(0x0000 + StMeasureFunc_MFB			)	// 0x0738
#define			StMeasFunc_MFB_SiMin2			(0x0004 + StMeasFunc_MFB_SiMax2		)	// 0x073C
#define			StMeasFunc_MFB_UiAmp2			(0x0004 + StMeasFunc_MFB_SiMin2		)	// 0x0740
#define			StMeasFunc_MFB_UiDUMMY1			(0x0004 + StMeasFunc_MFB_UiAmp2		)	// 0x0744
#define			StMeasFunc_MFB_LLiIntegral2		(0x0004 + StMeasFunc_MFB_UiDUMMY1	)	// 0x0748
#define			StMeasFunc_MFB_LLiAbsInteg2		(0x0008 + StMeasFunc_MFB_LLiIntegral2)	// 0x0750	// 8Byte
#define			StMeasFunc_MFB_PiMeasureRam2	(0x0008 + StMeasFunc_MFB_LLiAbsInteg2)	// 0x0758	// 8Byte

#endif


UINT8 msm_ois_lc898124_get_gyro_offset(void);
void msm_ois_lc898124_get_offset_enable(int enable);

UINT32 msm_ois_lc898124_read_gyro_offset(void);
void msm_ois_lc898124_write_gyro_offset(UINT32 data);
int32_t msm_nubia_eeprom_unlock_write(void);
int32_t msm_nubia_eeprom_lock_write(void);

/*ZTEMT: fengxun add for 627--------End*/


void CntWrt( void *	PcSetDat, unsigned short UsDatNum );
void WitTim( unsigned short	UsWitTim );
void RamRead32A( unsigned short addr, void * data);
void RamWrite32A(unsigned int addr, unsigned int data);

int32_t msm_ois_lc898124_init_AF(struct camera_io_master ois_master);
void msm_ois_lc898124_enable(int enable);
void msm_ois_lc898124_write_dac(unsigned int data);
void msm_ois_lc898124_deinit( void );

#endif

