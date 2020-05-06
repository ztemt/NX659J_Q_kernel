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

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/err.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include "cam_sensor_io.h"
#include "OisInter.h"

//**************************
//	Include Header File		
//**************************
//#include "LC898124EP2_Host_Code.h"	// INVEN 2030 & MTM Y3
#include "LC898124EP2_Code_1_2.h"       //ZTEMT:fengxun modify for 627 OIS


//#define MSM_OIS_DEBUG

#undef CDBG
#ifdef MSM_OIS_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#endif

#define		DeviceAddr		0x7C  	// Device address of driver IC

/*ZTEMT: fengxun add for 627--------Start*/
#define MSM_GYRO_OFFSET_ADDR 0x1DA0
#define MSM_ERRPOM_ADDR      0xA0
static char ois_get_gyro_offset_flag;
extern void ois_switch_update(int result);
/*ZTEMT: fengxun add for 627--------End*/

static char ois_status;
static char ois_disable;

static struct camera_io_master ois_master_info;

EXPORT_SYMBOL(msm_ois_lc898124_write_dac);
EXPORT_SYMBOL(msm_ois_lc898124_init_AF);
EXPORT_SYMBOL(msm_ois_lc898124_enable);
EXPORT_SYMBOL(CntWrt);
EXPORT_SYMBOL(WitTim);
EXPORT_SYMBOL(RamRead32A);
EXPORT_SYMBOL(RamWrite32A);

EXPORT_SYMBOL(msm_ois_lc898124_deinit);


/*ZTEMT: fengxun add for AF--------Start*/

/* for I2C communication */ 
void RamWrite32A(unsigned int addr, unsigned int data)
{
    uint8_t reqdata[4];
    int32_t rc=0;
    int i = 0;

    struct cam_sensor_i2c_reg_array reg_setting[4];
    struct cam_sensor_i2c_reg_setting write_setting;
    
    reqdata[0] = (data >> 24) & 0xFF;
    reqdata[1] = (data >> 16) & 0xFF;
    reqdata[2] = (data >> 8) & 0xFF;
    reqdata[3] = (data) & 0xFF;

    reg_setting[0].reg_addr = addr;
    for (i = 0; i < 4; i++) {
        reg_setting[i].reg_data = reqdata[i];
        reg_setting[i].delay = 0;
        reg_setting[i].data_mask = 0;
    }


    write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
    write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
    write_setting.delay = 0;
    write_setting.reg_setting = reg_setting;
    write_setting.size = 4;

    rc = cam_cci_i2c_write_continuous_table(&ois_master_info,&write_setting,0);

    if (rc < 0) {
        pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
    }
      
};
void RamRead32A( unsigned short addr, void * data)
{
    uint32_t *temp_read_data_32 = (uint32_t *)data;
    int32_t rc = 0;
    uint32_t readData = 0;

    rc = camera_io_dev_read(
        &ois_master_info,
        addr,
        &readData, CAMERA_SENSOR_I2C_TYPE_WORD,
        CAMERA_SENSOR_I2C_TYPE_DWORD);
    CDBG("RamRead32A %x \n", readData);
    
    if (rc < 0) 
    {
        pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
    }
    
    *temp_read_data_32=  readData;
    
}
/* for I2C Multi Translation : Burst Mode*/
void CntWrt( void *	PcSetDat, unsigned short UsDatNum )
{
    int rc = 0;
    int i = 0;
    unsigned char *SetDat = (unsigned char *)PcSetDat;
    uint16_t uNum = 0;

    struct cam_sensor_i2c_reg_array *reg_setting;
    struct cam_sensor_i2c_reg_setting write_setting;
    
    if((UsDatNum < 2)){
        pr_err("%s:%d UsDatNum is zero\n", __func__, __LINE__);
        return;
    }
    uNum = UsDatNum - 1;

    reg_setting = kzalloc(uNum *(sizeof(struct cam_sensor_i2c_reg_array)), GFP_KERNEL);
    if (!reg_setting) {
        pr_err("%s:%d no memory\n", __func__, __LINE__);
        return;
    }

    reg_setting[0].reg_addr = SetDat[0];
    for (i = 0; i < uNum; i++) {
        reg_setting[i].reg_data = SetDat[i+1];
        reg_setting[i].delay = 0;
        reg_setting[i].data_mask = 0;
    }


    write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
    write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
    write_setting.delay = 0;
    write_setting.reg_setting = reg_setting;
    write_setting.size = uNum;

    rc = cam_cci_i2c_write_continuous_table(&ois_master_info,&write_setting,0);

    if (rc < 0) {
        pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
    }

    return;
}
/* for Wait timer [Need to adjust for your system] */ 
void	WitTim( unsigned short	UsWitTim )
{
    msleep(UsWitTim);
}

/*ZTEMT: fengxun add for AF--------End*/

/*ZTEMT: fengxun add for 627--------Start*/
#ifdef MSM_DEBUG_GYRO_OFFSET
void SetTransDataAdr( UINT16 UsLowAddress , UINT32 UlLowAdrBeforeTrans )
{
	UnDwdVal	StTrsVal ;
	
	if( UlLowAdrBeforeTrans < 0x00009000 ){
		StTrsVal.UlDwdVal = UlLowAdrBeforeTrans ;
	}else{
		StTrsVal.StDwdVal.UsHigVal = (UINT16)(( UlLowAdrBeforeTrans & 0x0000F000 ) >> 8 ) ;
		StTrsVal.StDwdVal.UsLowVal = (UINT16)( UlLowAdrBeforeTrans & 0x00000FFF ) ;
	}
//TRACE(" TRANS  ADR = %04xh , DAT = %08xh \n",UsLowAddress , StTrsVal.UlDwdVal ) ;
	RamWrite32A( UsLowAddress	,	StTrsVal.UlDwdVal );
	
}

void SetWaitTime( UINT16 UsWaitTime )
{
	RamWrite32A( WaitTimerData_UiWaitCounter	, 0 ) ;
	RamWrite32A( WaitTimerData_UiTargetCount	, (UINT32)(ONE_MSEC_COUNT * UsWaitTime)) ;
}

void ClrMesFil( void )
{
	RamWrite32A ( MeasureFilterA_Delay_z11	, 0 ) ;
	RamWrite32A ( MeasureFilterA_Delay_z12	, 0 ) ;

	RamWrite32A ( MeasureFilterA_Delay_z21	, 0 ) ;
	RamWrite32A ( MeasureFilterA_Delay_z22	, 0 ) ;

	RamWrite32A ( MeasureFilterB_Delay_z11	, 0 ) ;
	RamWrite32A ( MeasureFilterB_Delay_z12	, 0 ) ;

	RamWrite32A ( MeasureFilterB_Delay_z21	, 0 ) ;
	RamWrite32A ( MeasureFilterB_Delay_z22	, 0 ) ;
}

void MemoryClear( UINT16 UsSourceAddress, UINT16 UsClearSize )
{
	UINT16	UsLoopIndex ;

	for ( UsLoopIndex = 0 ; UsLoopIndex < UsClearSize ; UsLoopIndex += 4 ) {
		RamWrite32A( UsSourceAddress + UsLoopIndex	, 	0x00000000 ) ;				// 4Byte
//TRACE("MEM CLR ADR = %04xh \n",UsSourceAddress + UsLoopIndex) ;
	}
}

void MeasureStart( INT32 SlMeasureParameterNum , UINT32 SlMeasureParameterA , UINT32 SlMeasureParameterB )
{
	MemoryClear( StMeasFunc_SiSampleNum , sizeof( MeasureFunction_Type ) ) ;
	RamWrite32A( StMeasFunc_MFA_SiMax1	 , 0x80000000 ) ;					// Set Min 
	RamWrite32A( StMeasFunc_MFB_SiMax2	 , 0x80000000 ) ;					// Set Min 
	RamWrite32A( StMeasFunc_MFA_SiMin1	 , 0x7FFFFFFF ) ;					// Set Max 
	RamWrite32A( StMeasFunc_MFB_SiMin2	 , 0x7FFFFFFF ) ;					// Set Max 
	
	SetTransDataAdr( StMeasFunc_MFA_PiMeasureRam1	 , SlMeasureParameterA ) ;		// Set Measure Filter A Ram Address
	SetTransDataAdr( StMeasFunc_MFB_PiMeasureRam2	 , SlMeasureParameterB ) ;		// Set Measure Filter B Ram Address
	RamWrite32A( StMeasFunc_SiSampleNum	 , 0 ) ;													// Clear Measure Counter 
	ClrMesFil() ;						// Clear Delay Ram
//	SetWaitTime(50) ;
	SetWaitTime(1) ;
	RamWrite32A( StMeasFunc_SiSampleMax	 , SlMeasureParameterNum ) ;						// Set Measure Max Number

}

void MesFil( UINT8	UcMesMod )		// 20.019kHz
{
	UINT32	UlMeasFilaA , UlMeasFilaB , UlMeasFilaC ;
	UINT32	UlMeasFilbA , UlMeasFilbB , UlMeasFilbC ;

	if( !UcMesMod ) {								// Hall Bias&Offset Adjust
		
		UlMeasFilaA	=	0x02F19B01 ;	// LPF 150Hz
		UlMeasFilaB	=	0x02F19B01 ;
		UlMeasFilaC	=	0x7A1CC9FF ;
		UlMeasFilbA	=	0x7FFFFFFF ;	// Through
		UlMeasFilbB	=	0x00000000 ;
		UlMeasFilbC	=	0x00000000 ;
		
	} else if( UcMesMod == LOOPGAIN ) {				// Loop Gain Adjust

		UlMeasFilaA	=	0x115CC757 ;	// LPF1000Hz
		UlMeasFilaB	=	0x115CC757 ;
		UlMeasFilaC	=	0x5D467153 ;
		UlMeasFilbA	=	0x7F667431 ;	// HPF30Hz
		UlMeasFilbB	=	0x80998BCF ;
		UlMeasFilbC	=	0x7ECCE863 ;
		
	} else if( UcMesMod == THROUGH ) {				// for Through

		UlMeasFilaA	=	0x7FFFFFFF ;	// Through
		UlMeasFilaB	=	0x00000000 ;
		UlMeasFilaC	=	0x00000000 ;
		UlMeasFilbA	=	0x7FFFFFFF ;	// Through
		UlMeasFilbB	=	0x00000000 ;
		UlMeasFilbC	=	0x00000000 ;

	} else if( UcMesMod == NOISE ) {				// SINE WAVE TEST for NOISE

		UlMeasFilaA	=	0x02F19B01 ;	// LPF150Hz
		UlMeasFilaB	=	0x02F19B01 ;
		UlMeasFilaC	=	0x7A1CC9FF ;
		UlMeasFilbA	=	0x02F19B01 ;	// LPF150Hz
		UlMeasFilbB	=	0x02F19B01 ;
		UlMeasFilbC	=	0x7A1CC9FF ;

	} else if(UcMesMod == OSCCHK) {
		UlMeasFilaA	=	0x05C141BB ;	// LPF300Hz
		UlMeasFilaB	=	0x05C141BB ;
		UlMeasFilaC	=	0x747D7C88 ;
		UlMeasFilbA	=	0x05C141BB ;	// LPF300Hz
		UlMeasFilbB	=	0x05C141BB ;
		UlMeasFilbC	=	0x747D7C88 ;
	}
	
	RamWrite32A ( MeasureFilterA_Coeff_a1	, UlMeasFilaA ) ;
	RamWrite32A ( MeasureFilterA_Coeff_b1	, UlMeasFilaB ) ;
	RamWrite32A ( MeasureFilterA_Coeff_c1	, UlMeasFilaC ) ;

	RamWrite32A ( MeasureFilterA_Coeff_a2	, UlMeasFilbA ) ;
	RamWrite32A ( MeasureFilterA_Coeff_b2	, UlMeasFilbB ) ;
	RamWrite32A ( MeasureFilterA_Coeff_c2	, UlMeasFilbC ) ;

	RamWrite32A ( MeasureFilterB_Coeff_a1	, UlMeasFilaA ) ;
	RamWrite32A ( MeasureFilterB_Coeff_b1	, UlMeasFilaB ) ;
	RamWrite32A ( MeasureFilterB_Coeff_c1	, UlMeasFilaC ) ;

	RamWrite32A ( MeasureFilterB_Coeff_a2	, UlMeasFilbA ) ;
	RamWrite32A ( MeasureFilterB_Coeff_b2	, UlMeasFilbB ) ;
	RamWrite32A ( MeasureFilterB_Coeff_c2	, UlMeasFilbC ) ;
}

void GyroOffsetMeasureStart( void )	
{
	MesFil( THROUGH ) ;								// Set Measure Filter
	MeasureStart( GYROF_NUM , GYRO_RAM_GX_ADIDAT , GYRO_RAM_GY_ADIDAT ) ;	// Start measure
}

UINT8 GetGyroOffset( UINT16* GyroOffsetX, UINT16* GyroOffsetY, INT16 GYROF_UPPER, INT16 GYROF_LOWER   )
{
	UnllnVal		StMeasValueA , StMeasValueB ;
	INT32			SlMeasureAveValueA , SlMeasureAveValueB ;
	INT32			SlMeasureMaxValue , SlMeasureMinValue ;
	UINT32 UlReadVal, UlCnt=0;
	UINT8 ans=0;

	// Wait complete of measurement
	do{
		if( UlCnt++ > 100 ){
			/* timeout error */
			*GyroOffsetX = 0;
			*GyroOffsetY = 0;
			return( 3 );
		}
		RamRead32A( StMeasFunc_SiSampleMax , &UlReadVal ) ;
	}while ( UlReadVal != 0 );
	
	RamRead32A( StMeasFunc_MFA_SiMax1 , ( UINT32 * )&SlMeasureMaxValue ) ;	// Max value
	RamRead32A( StMeasFunc_MFA_SiMin1 , ( UINT32 * )&SlMeasureMinValue ) ;	// Min value
	if (SlMeasureMaxValue == SlMeasureMinValue )
	{
		return( 3 ); 
	}
	
	RamRead32A( StMeasFunc_MFA_LLiIntegral1 		, &StMeasValueA.StUllnVal.UlLowVal ) ;	// X axis
	RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4		, &StMeasValueA.StUllnVal.UlHigVal ) ;
	RamRead32A( StMeasFunc_MFB_LLiIntegral2 		, &StMeasValueB.StUllnVal.UlLowVal ) ;	// Y axis
	RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4		, &StMeasValueB.StUllnVal.UlHigVal ) ;
	
	SlMeasureAveValueA = (INT32)( (INT64)StMeasValueA.UllnValue / GYROF_NUM ) ;
	SlMeasureAveValueB = (INT32)( (INT64)StMeasValueB.UllnValue / GYROF_NUM ) ;
	
	SlMeasureAveValueA = ( SlMeasureAveValueA >> 16 ) & 0x0000FFFF ;
	SlMeasureAveValueB = ( SlMeasureAveValueB >> 16 ) & 0x0000FFFF ;
	// EP1では反転処理しない。
//	SlMeasureAveValueA = 0x00010000 - SlMeasureAveValueA ;
//	SlMeasureAveValueB = 0x00010000 - SlMeasureAveValueB ;
	
	*GyroOffsetX = ( UINT16 )( SlMeasureAveValueA & 0x0000FFFF );		//Measure Result Store
	*GyroOffsetY = ( UINT16 )( SlMeasureAveValueB & 0x0000FFFF );		//Measure Result Store

	if(( (INT16)(*GyroOffsetX) > GYROF_UPPER ) || ( (INT16)(*GyroOffsetX) < GYROF_LOWER )){
		ans |= 1; 
	}
	if(( (INT16)(*GyroOffsetY) > GYROF_UPPER ) || ( (INT16)(*GyroOffsetY) < GYROF_LOWER )){
		ans |= 2; 
	}

	return( ans ); 
}

#endif

#define		CMD_GYRO_RD_ACCS				0xF01D				// Gyro Read Acess
#define		CMD_GYRO_WR_ACCS				0xF01E				// Gyro Write Acess

//********************************************************************************
// Function Name 	: msm_ois_lc898124_deinit
// Retun Value		: 
// Argment Value	: 
// Explanation		: 
// History			: First edition
//********************************************************************************
void msm_ois_lc898124_deinit( void )
{
	UINT32 UlReadVa;
	
	msm_ois_lc898124_write_dac(1024); //hufan add for imx586 crash voice
	ois_status = 0;

	/* SPI communication pending */
	RamRead32A ( (GYRO_RAM_GYRO_AF_Switch & 0xFFFC), &UlReadVa );
	RamWrite32A( (GYRO_RAM_GYRO_AF_Switch & 0xFFFC), (UlReadVa|0x00008000) );

	RamWrite32A( CMD_IO_ADR_ACCESS , ROMINFO );
	RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVa );
	if ( UlReadVa == 0x01 ){
		pr_err("FX %s: line %d Gyro SPI disable \n",__func__, __LINE__);
		/* Gyro SPI disable command set for ST by secondary SPI*/
		RamWrite32A( CMD_GYRO_WR_ACCS, 0x70000000 );
	}
}


//********************************************************************************
// Function Name 	: DMIOWrite32
// Retun Value		: None
// Argment Value	: IOadrs, IOdata
// Explanation		: Read From code version Command
// History			: First edition
//********************************************************************************
void DMIOWrite32( UINT32 IOadrs, UINT32 IOdata )
{
#if 1
	UINT8 data[10];
	data[0] = 0xC0;		// Pmem address set
	data[1] = 0x00;		// Command High
	data[2] = (UINT8)(IOadrs >>24);		// IOadres
	data[3] = (UINT8)(IOadrs >>16);		// Command High
	data[4] = (UINT8)(IOadrs >> 8);		// Command High
	data[5] = (UINT8)(IOadrs >> 0);		// Command High
	data[6] = (UINT8)(IOdata >>24);		// IOadres
	data[7] = (UINT8)(IOdata >>16);		// Command High
	data[8] = (UINT8)(IOdata >> 8);		// Command High
	data[9] = (UINT8)(IOdata >> 0);		// Command High
	CntWrt( data, 10 ); 	// I2C 1Byte address.
#else
	RamWrite32A( CMD_IO_ADR_ACCESS, IOadrs ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, IOdata ) ;
#endif
};


//********************************************************************************
// Function Name 	: SetAngleCorrection
// Retun Value		: True/Fail
// Argment Value	: 
// Explanation		: Angle Correction
// History			: First edition
//********************************************************************************
UINT8 SetAngleCorrection( float DegreeGap, UINT8 SelectAct, UINT8 Arrangement )
{
	//double OffsetAngle = 0.0f;
	INT32 Slgx45x, Slgx45y;
	INT32 Slgy45y, Slgy45x;

    CDBG("FX lc898 SetAngleCorrection \n");

	if( ( DegreeGap > 180.0f) || ( DegreeGap < -180.0f ) ) return ( 1 );
	if( Arrangement >= 2 ) return ( 1 );

#if 0
/************************************************************************/
/*      	Gyro angle correction										*/
/************************************************************************/
	switch(SelectAct) {
//			OffsetAngle = (double)( 45.0f + DegreeGap ) * 3.141592653589793238 / 180.0f ;
			break;

		default :
		case ACT_HOZEL_OZ31A401 :
			OffsetAngle = (double)( DegreeGap ) * 3.141592653589793238 / 180.0f ;
			
			if( Arrangement == 1 ){		// Gyro sensor => Back
				RamWrite32A( HallFilterCoeffX_GYROXOUTGAIN , 0x7fffffff );
				RamWrite32A( HallFilterCoeffY_GYROYOUTGAIN , 0x7fffffff );
				Slgx45x = (INT32)(cos( OffsetAngle )*2147483647.0);
				Slgx45y = (INT32)(sin( OffsetAngle )*2147483647.0);
				Slgy45y = (INT32)(-cos( OffsetAngle )*2147483647.0);
				Slgy45x = (INT32)(sin( OffsetAngle )*2147483647.0);
			}else{						// Gyro sensor => Front
				RamWrite32A( HallFilterCoeffX_GYROXOUTGAIN , 0x80000001 );
				RamWrite32A( HallFilterCoeffY_GYROYOUTGAIN , 0x7fffffff );
				Slgx45x = (INT32)(cos( OffsetAngle )*2147483647.0);
				Slgx45y = (INT32)(-sin( OffsetAngle )*2147483647.0);
				Slgy45y = (INT32)(-cos( OffsetAngle )*2147483647.0);
				Slgy45x = (INT32)(-sin( OffsetAngle )*2147483647.0);
			}

			break;
	}
#endif

    /*ZTEMT: fengxun add for 627--------Start*/
    if(((DegreeGap == 180)) && ( Arrangement == 1 )){     // Gyro sensor => Back
        CDBG("DegreeGap = 180 \n") ;
    
        RamWrite32A( HallFilterCoeffX_GYROXOUTGAIN , 0x7fffffff );
        RamWrite32A( HallFilterCoeffY_GYROYOUTGAIN , 0x7fffffff );

        Slgx45x = -2147483647;      //(INT32)(cos( OffsetAngle )*2147483647.0);
        Slgx45y = 0;                //(INT32)(sin( OffsetAngle )*2147483647.0);
        Slgy45y = 2147483647;       //(INT32)(-cos( OffsetAngle )*2147483647.0);
        Slgy45x = 0;                //(INT32)(sin( OffsetAngle )*2147483647.0);
    }else{
        CDBG("DegreeGap error \n") ;
        return (-1);
    }
    /*ZTEMT: fengxun add for 627--------End*/

	RamWrite32A( GyroFilterTableX_gx45x , (UINT32)Slgx45x );
	RamWrite32A( GyroFilterTableX_gx45y , (UINT32)Slgx45y );
	RamWrite32A( GyroFilterTableY_gy45y , (UINT32)Slgy45y );
	RamWrite32A( GyroFilterTableY_gy45x , (UINT32)Slgy45x );


	CDBG("Slgx45x = %lx \n", (unsigned int)Slgx45x ) ;
	CDBG("Slgx45y = %lx \n", (unsigned int)Slgx45y ) ;
	CDBG("Slgy45y = %lx \n", (unsigned int)Slgy45y ) ;
	CDBG("Slgy45x = %lx \n", (unsigned int)Slgy45x ) ;

	return (0);
}

//********************************************************************************
// Function Name 	: SetGyroOffset
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: set the gyro offset data. before do this before Remapmain.
// History			: First edition
//********************************************************************************
void SetGyroOffset( UINT16 GyroOffsetX, UINT16 GyroOffsetY )
{
    pr_err("FX lc898124 %s E  %x,%x\n",__func__, GyroOffsetX, GyroOffsetY);
	RamWrite32A(GYRO_RAM_GXOFFZ, (( GyroOffsetX << 16 ) & 0xFFFF0000 ));		// X axis Gyro offset
	RamWrite32A(GYRO_RAM_GYOFFZ, (( GyroOffsetY << 16 ) & 0xFFFF0000 ));		// Y axis Gyro offset
} 

//********************************************************************************
// Function Name 	: DownloadToEP1_Ver2
// Retun Value		: NON
// Argment Value	: PMlength: 5byte unit, DMlength : 1Byte unit 
// Explanation		: <Pmem Memory> Write Data
// History			: First edition
//********************************************************************************
unsigned char DownloadToEP1_Ver2( const UINT8* DataPM, UINT32 LengthPM, UINT32 Parity, const UINT8* DataDM, UINT32 LengthDM ) 
{
	UINT32 i, j;
	UINT8 data[64];		// work fifo buffer max size 64 byte
	UINT8 Remainder;	// 余り
	UINT32 UlReadVal, UlCnt;
	UINT32 ReadVerifyPM = 0;    // Checksum
	UINT32 ReadVerifyDM = 0;	//ZTEMT:fengxun modify

//--------------------------------------------------------------------------------
// 0. Start up to boot exection 
//--------------------------------------------------------------------------------
	WitTim(13) ;	//ZTEMT:fengxun modify
	RamWrite32A( CMD_IO_ADR_ACCESS , ROMINFO );
	RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
	switch ( (UINT8)UlReadVal ){
	case 0x0A:	/* Normal Rom program execution */
		break;
	
	case 0x01:	/* Normal Ram program execution */
		/* 再Donloadのためには RomRebootしなければならない。AutoDownloadさせるためにCORE_RSTで実行させる*/
		DMIOWrite32( SYSDSP_REMAP, 0x00001000 ); 	// CORE_RST
		WitTim( 6 ) ;								// Bootプログラムを回すのに6msec必要。
		break;

//	case 0x0B:
//	case 0x08:
	default:
		return( 1 );
	}
//--------------------------------------------------------------------------------
// 1. Download Program
//--------------------------------------------------------------------------------
	data[0] = 0x30;		// Pmem address set
	data[1] = 0x00;		// Command High
	data[2] = 0x10;		// Command High
	data[3] = 0x00;		// Command High
	data[4] = 0x00;		// Command High
	CntWrt( data, 5 ); 	// I2C 1Byte address.
	// program start
	data[0] = 0x40;		// Pmem address set
	Remainder = ( (LengthPM*5) / BURST_LENGTH_PM );
	for(i=0 ; i< Remainder ; i++)
	{
		UlCnt = 1;
		for(j=0 ; j < BURST_LENGTH_PM; j++)	data[UlCnt++] = *DataPM++;

		CntWrt( data, BURST_LENGTH_PM+1 );  // I2Caddresss 1Byte.
	}
	Remainder = ( (LengthPM*5) % BURST_LENGTH_PM); 
	if (Remainder != 0 )
	{
		UlCnt = 1;
		for(j=0 ; j < Remainder; j++)	data[UlCnt++] = *DataPM++;
        //CDBG("Remainder %d \n", (UINT8)Remainder );
		CntWrt( data, UlCnt );  // I2C 1Byte address.
	}
	// Chercksum start
	data[0] = 0xF0;											// Pmem address set
	data[1] = 0x0A;											// Command High
	data[2] = (unsigned char)(( LengthPM & 0xFF00) >> 8 );	// Size High
	data[3] = (unsigned char)(( LengthPM & 0x00FF) >> 0 );	// Size Low
	CntWrt( data, 4 ); 	// I2C 2Byte addresss.
#if 0
	UlCnt=0;
	do{
		if( UlCnt++ > 10 ) break;						// 400kHzで2回目。
		RamRead32A( PmCheck_EndFlag, &UlReadVal );
	}while ( UlReadVal == 0 );
	RamRead32A( PmCheck_CheckSum, &ReadVerifyPM );
#endif
//--------------------------------------------------------------------------------
// 2. Download Table Data
//--------------------------------------------------------------------------------
//CDBG("DM Start \n" );
//	RamWrite32A( DmCheck_CheckSumDMA, 0 );
	RamWrite32A( DmCheck_CheckSumDMB, 0 );		// DMB Parity Clear
	for( i=0; i < LengthDM; i+=6 )
	{
		if ((DataDM[0+i] == 0x80) && (DataDM[1+i] == 0x0C) )
		{
			RamWrite32A( CommandDecodeTable_08, (((UINT32)(DataDM[3+i])<<16) + ((UINT32)(DataDM[4+i])<<8) + DataDM[5+i] ) );
		}
	}
	// Data Send
	Remainder = ((LengthDM*6/4) / BURST_LENGTH_DM ); 
	for(i=0 ; i< Remainder ; i++)
	{
		CntWrt((UINT8*)DataDM, BURST_LENGTH_DM );  // I2Caddresss 1Byte.
		DataDM += BURST_LENGTH_DM;
	}
	Remainder = ((LengthDM*6/4) % BURST_LENGTH_DM ); 
	if (Remainder != 0 )
	{
		CntWrt((UINT8*)DataDM, (UINT8)Remainder );  // I2Caddresss 1Byte.
//		DataDM += BURST_LENGTH_DM;
	}

//--------------------------------------------------------------------------------
// 3. Verify
//--------------------------------------------------------------------------------
	RamRead32A( PmCheck_CheckSum, &ReadVerifyPM );
	RamRead32A( DmCheck_CheckSumDMB, &ReadVerifyDM );
	/* Verifyの値を読み出してから関数を基に戻す必要がある。下記CodeがReadより	*/
	/* 先に実行されるとParityの結果も変わってしまう。						    */
	RamWrite32A( CommandDecodeTable_08, 0x000418C4 );

    if( (ReadVerifyPM + ReadVerifyDM ) != Parity  ){
        CDBG("error! %lx %lx != %lx\n", ReadVerifyPM, ReadVerifyDM ,Parity); 
        return( 2 );
    }

	return(0);
}

//********************************************************************************
// Function Name 	: ReMapMain
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: <Pmem Memory> Write Data
// History			: First edition
//********************************************************************************
void RemapMain( void )
{
    CDBG("FX lc898 RemapMain \n");
	RamWrite32A( 0xF000, 0x00000000 );
}

//********************************************************************************
// Function Name 	: MonitorInfo
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: 
// History			: Second edition
//********************************************************************************
void MonitorInfo( DSPVER* Dspcode )
{
    CDBG("Vendor : %02x \n", Dspcode->Vendor );
    CDBG("User : %02x \n", Dspcode->User );
    CDBG("Model : %02x \n", Dspcode->Model );
    CDBG("Version : %02x \n", Dspcode->Version );

    if(Dspcode->ActType == ACT_HOZEL_OZ31A401 )
    CDBG("actuator type : HOZEL OZ31A401\n");

    if(Dspcode->GyroType == GYRO_LSM6DSM )
    CDBG("gyro type : ST LSM6DSM \n");
}

//********************************************************************************
// Function Name 	: GetInfomationBeforeDownlaod
// Retun Value		: True(0) / Fail(1)
// Argment Value	: NON
// Explanation		: <Pmem Memory> Write Data
// History			: First edition
//********************************************************************************
UINT8 GetInfomationBeforeDownlaod( DSPVER* Info, const UINT8* DataDM,  UINT32 LengthDM )
{
	UINT32 i;
	Info->ActType = 0;
	Info->GyroType = 0;

	for( i=0; i < LengthDM; i+=6 )
	{
		if ( (DataDM[0+i] == 0x80) && (DataDM[1+i] == 0x00) )
		{
			Info->Vendor = DataDM[2+i];
			Info->User = DataDM[3+i];
			Info->Model = DataDM[4+i];
			Info->Version = DataDM[5+i];
			if ( (DataDM[6+i] == 0x80) && (DataDM[7+i] == 0x04) )
			{
				Info->ActType = DataDM[10+i];
				Info->GyroType = DataDM[11+i];
			}
			MonitorInfo( Info );
			return (0);
		}
	}
	return(1);
}

const DOWNLOAD_TBL DTbl[] = {
    {0x0102, LC898124EP2_PM_1_2, LC898124EP2_PMSize_1_2, (UINT32)((UINT32)LC898124EP2_PMCheckSum_1_2 + (UINT32)LC898124EP2_DMB_CheckSum_1_2), LC898124EP2_DM_1_2, LC898124EP2_DMB_ByteSize_1_2 },
    {0xFFFF, (void*)0, 0, 0, (void*)0 ,0 }
};

unsigned char SelectDownload( UINT8 GyroSelect,  UINT8 ActSelect)
{
	DSPVER Dspcode;
	DOWNLOAD_TBL* ptr;

    CDBG("FX lc898 SelectDownload \n");

	/* どのCodeをDownloadするのかTableから検索 */
	ptr = ( DOWNLOAD_TBL * )DTbl;
	while (ptr->Cmd != 0xFFFF ){
		if( ptr->Cmd == ( ((UINT16)ActSelect<<8) + GyroSelect) ) break;
		ptr++ ;
	}
	if (ptr->Cmd == 0xFFFF)	return(0xF0);

	/* Downloadする前CodeのInformation情報確認 */
	if( GetInfomationBeforeDownlaod( &Dspcode, ptr->DataDM, ptr->LengthDM ) != 0 ){
		return(0xF1);
	}

	/* Downloadする前のCodeと、要求しているActuator/Gyro情報が一致しているか確認 */
	if( (ActSelect != Dspcode.ActType) || (GyroSelect != Dspcode.GyroType) ) return(0xF2);

	return(DownloadToEP1_Ver2( ptr->DataPM, ptr->LengthPM, ptr->Parity, ptr->DataDM, ptr->LengthDM ));
}
/*ZTEMT: fengxun add for 627--------End*/


#ifdef MSM_DEBUG_GYRO_OFFSET

int32_t msm_nubia_eeprom_unlock_write(void)
{
	int rc = 0;
	uint32_t readdata = 0;
	uint16_t sid_tmp = 0;

	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array unlock_reg[] =
		{
			{0xFFFF, 0x1D, 0x00, 0x00},
		};
	sid_tmp = ois_master_info.cci_client->sid;
	ois_master_info.cci_client->sid = 0xB0 >> 1;

	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.delay = 0;
	write_setting.reg_setting = unlock_reg;
	write_setting.size = 1;
	rc = camera_io_dev_write(&ois_master_info, &write_setting);
	if (rc < 0)
	{
		CAM_ERR(CAM_SENSOR, "camera_io_dev_write  [unlock] error");
		return -1;
	}
	usleep_range(10000, 11000);
	//for debug CSP
	rc = camera_io_dev_read(&ois_master_info, 0xFFFF,
				&readdata, CAMERA_SENSOR_I2C_TYPE_WORD,
				CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0)
	{
		CAM_ERR(CAM_EEPROM, "camera_io_dev_read error");
	}
	ois_master_info.cci_client->sid = sid_tmp;
	CAM_ERR(CAM_EEPROM, " read CSP =%x ", readdata);

    return rc;
}

int32_t msm_nubia_eeprom_lock_write(void)
{
	int rc = 0;
	uint32_t readdata = 0;
	uint16_t sid_tmp = 0;

	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array lock_reg[] =
		{
			{0xFFFF, 0x1F, 0x00, 0x00},
		};
	sid_tmp = ois_master_info.cci_client->sid;
	ois_master_info.cci_client->sid = 0xB0 >> 1;

	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.delay = 0;
	write_setting.reg_setting = lock_reg;
	write_setting.size = 1;
	rc = camera_io_dev_write(&ois_master_info, &write_setting);
	if (rc < 0)
	{
		CAM_ERR(CAM_SENSOR, "camera_io_dev_write  [lock] error");
		return -1;
	}
		usleep_range(10000, 11000);

	//for debug CSP
	rc = camera_io_dev_read(&ois_master_info, 0xFFFF,
				&readdata, CAMERA_SENSOR_I2C_TYPE_WORD,
				CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0)
	{
		CAM_ERR(CAM_EEPROM, "camera_io_dev_read error");
	}
	ois_master_info.cci_client->sid = sid_tmp;
	CAM_ERR(CAM_EEPROM, " read CSP =%x ", readdata);
	return rc;

}

void msm_ois_lc898124_write_gyro_offset(UINT32 data)
{
	uint16_t sid = 0;

    sid = ois_master_info.cci_client->sid;
	ois_master_info.cci_client->sid = MSM_ERRPOM_ADDR >> 1;
    pr_err("[FX] write gyro to eeprom change SID %x\n",ois_master_info.cci_client->sid);

	msm_nubia_eeprom_unlock_write();
	usleep_range(10000, 11000);

	RamWrite32A(MSM_GYRO_OFFSET_ADDR, data);
    pr_err("[FX]  write gyro to eeprom RamWrite32A %x\n",data);

	usleep_range(10000, 11000);
	msm_nubia_eeprom_lock_write();

    ois_master_info.cci_client->sid = sid;
    pr_err("[FX] write gyro to eeprom set SID %x\n",ois_master_info.cci_client->sid);

    return;
}

UINT32 msm_ois_lc898124_read_gyro_offset(void)
{
	uint16_t sid = 0;
    UINT32 data = 0;

    sid = ois_master_info.cci_client->sid;
	ois_master_info.cci_client->sid = MSM_ERRPOM_ADDR >> 1;
    CDBG("[FX] read gyro from eeprom change SID %x\n",ois_master_info.cci_client->sid);

	RamRead32A(MSM_GYRO_OFFSET_ADDR, &data);
    CDBG("[FX] readdata %x\n",data);

    ois_master_info.cci_client->sid = sid;
    CDBG("[FX] read gyro from eeprom set SID %x\n",ois_master_info.cci_client->sid);

    return data;
}

void msm_ois_lc898124_get_offset_enable(int enable)
{
    pr_err("%s enable = %d \n",__func__, enable);
    ois_get_gyro_offset_flag = enable;
    return;
}

UINT8 msm_ois_lc898124_get_gyro_offset(void)
{
    int rc = 0;
    UINT16 GyroOffsetX = 0;
    UINT16 GyroOffsetY = 0;

    UINT32 data = 0;

    GyroOffsetMeasureStart();
    msleep(200);

    rc = GetGyroOffset(&GyroOffsetX,&GyroOffsetY,0x06D6,0xF92A);
    if(rc != 0){
        pr_err("%s error return rc = %d\n",__func__,rc);
        return -1;
    }

    pr_err("%s GyroOffsetX=%d  GyroOffsetY=%d\n",__func__, GyroOffsetX,GyroOffsetY);

    data = (GyroOffsetX & 0xFFFF) | ((GyroOffsetY & 0xFFFF)<<16);
    msm_ois_lc898124_write_gyro_offset(data);

    return 0;
}
#endif

void msm_ois_lc898124_write_dac(unsigned int data)
{
    if(0 == ois_status){
        pr_err("msm_ois_lc898124_write_dac ois status = 0 return \n");
        return;
    }
    RamWrite32A(0xF01A, data | 0x00010000);
}

void msm_ois_lc898124_enable(int enable)
{

    if(ois_master_info.cci_client == NULL){
        pr_err("msm_ois_lc898124_enable ois_master_info is NULL ,error return\n");
        return;
    }

    if(0 == ois_status){
        if(0 == enable){
            ois_disable = 1;
            pr_err("msm_ois_lc898124_enable ois_disable \n");
        }
        pr_err("msm_ois_lc898124_enable ois status = 0 return \n");
        return;
    }

    pr_err("msm_ois_lc898124_enable %d\n",enable);

    if (enable == 1)
    {
        RamWrite32A(0xF012, 0x00000001);
    }
    else
    {
        RamWrite32A(0xF012, 0x00000000);
    }
}

 int msm_ois_lc898124_check_status()
{
    uint32_t readData = 0;
    int cnt = 0;

    CDBG("FX msm_ois_lc898124_check_status 0xF100 \n");

    do{
        RamRead32A(0xF100, &readData); 
        msleep(5);
    } while(((readData & 0x1000000) != 0) && cnt++ < 20);

    pr_err("msm_ois_lc898124_check_status read0xF100 = %x  cnt=%d\n",readData,cnt);

    if(cnt >= 20){
        ois_status = 0;
    }else{
        ois_status = 1;
    }
    CDBG("FX msm_ois_lc898124_check_status = %d \n",ois_status);

    return ois_status;
}

int32_t msm_ois_lc898124_init_AF(struct camera_io_master ois_master)
{
    int rc = 0;
    UINT32 offset = 0;

    ois_status = 0;
    ois_disable = 0;

    CDBG("FX msm_ois_lc898124_init_AF \n");

    ois_master_info = ois_master;

    if(1 == ois_get_gyro_offset_flag){
        ois_get_gyro_offset_flag = 0;
        rc = SelectDownload(GYRO_LSM6DSM, ACT_HOZEL_OZ31A401);
        if(rc != 0){
            pr_err("%s: EP2Download failed  line %d rc = %d\n", __func__, __LINE__, rc);
            return rc;
        }

        RemapMain();

        SetAngleCorrection(180.0, ACT_HOZEL_OZ31A401,1);

        msm_ois_lc898124_check_status();
        if(ois_status){
            rc = msm_ois_lc898124_get_gyro_offset();
            if(rc == 0){
                ois_switch_update(1);
            }
        }else{
            pr_err("%s: msm_ois_lc898124_check_status error  line %d \n", __func__, __LINE__);
        }

    }else{
        offset = msm_ois_lc898124_read_gyro_offset();

        rc = SelectDownload(GYRO_LSM6DSM, ACT_HOZEL_OZ31A401);
        if(rc != 0){
            pr_err("%s: EP2Download failed  line %d rc = %d\n", __func__, __LINE__, rc);
            return rc;
        }

        SetAngleCorrection(180.0, ACT_HOZEL_OZ31A401,1);
        if((offset == 0) || (offset == 0xFFFFFFFF)){
            SetGyroOffset(0,0);
        }else{
            SetGyroOffset(offset & 0xFFFF, (offset & 0xFFFF0000)>>16);
        }

        RemapMain();

        msm_ois_lc898124_check_status();
        if(ois_status){
            if(ois_disable){
                msm_ois_lc898124_enable(0);
            }else{
                msm_ois_lc898124_enable(1);
            }
        }else{
            pr_err("%s: msm_ois_lc898124_check_status error  line %d \n", __func__, __LINE__);
        }
    }

    return rc;
}
////


