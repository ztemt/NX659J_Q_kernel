/*
 * Touchkey driver for CYPRESS4000 controller
 *
 * Copyright (C) 2018 nubia
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/i2c.h>
//#include <linux/i2c/mcs.h>
#include <linux/string.h>
#include <linux/delay.h>
#include "cybtldr_touchkey_command.h"
#include "cy8c4014l_touchkey_firmware_update.h"

unsigned long g_validRows[MAX_FLASH_ARRAYS];
struct i2c_client *cypress_client;

unsigned char CyBtldr_FromHex(char value)
{
    if ('0' <= value && value <= '9')
        return (unsigned char)(value - '0');
    if ('a' <= value && value <= 'f')
        return (unsigned char)(10 + value - 'a');
    if ('A' <= value && value <= 'F')
        return (unsigned char)(10 + value - 'A');
    return 0;
}

int CyBtldr_FromAscii(unsigned int bufSize, unsigned char* buffer, unsigned short* rowSize, unsigned char* rowData)
{
    unsigned short i;
    int err = CYRET_SUCCESS;

    if (bufSize & 1) // Make sure even number of bytes
        err = CYRET_ERR_LENGTH;
    else
    {
        for (i = 0; i < bufSize / 2; i++)
        {
            rowData[i] = (CyBtldr_FromHex(buffer[i * 2]) << 4) | CyBtldr_FromHex(buffer[i * 2 + 1]);
        }
        *rowSize = i;
    }

    return err;
}


int CyBtldr_ParseHeader(unsigned int bufSize, unsigned char* buffer, unsigned long* siliconId, unsigned char* siliconRev, unsigned char* chksum)
{
    const unsigned int LENGTH_ID     = 5;            //4-silicon id, 1-silicon rev
    const unsigned int LENGTH_CHKSUM = LENGTH_ID + 1; //1-checksum type

    unsigned short rowSize;
    unsigned char rowData[MAX_BUFFER_SIZE];

    int err = CyBtldr_FromAscii(bufSize, buffer, &rowSize, rowData);

    if (CYRET_SUCCESS == err)
    {
        if (rowSize >= LENGTH_CHKSUM)
            *chksum = rowData[5];
        if (rowSize >= LENGTH_ID)
        {
            *siliconId = (rowData[0] << 24) | (rowData[1] << 16) | (rowData[2] << 8) | (rowData[3]);
            *siliconRev = rowData[4];
        }
        else
            err = CYRET_ERR_LENGTH;
    }

    return err;
}

void CyBtldr_SetCheckSumType(CyBtldr_ChecksumType chksumType)
{
    CyBtldr_Checksum = chksumType;
}

int CyBtldr_TransferData(unsigned char* inBuf, int inSize, unsigned char* outBuf, int outSize)
{
	unsigned char retry;
	int ret;

	for (retry = 0; retry < CYPRESS4000_I2C_RETRY_TIMES; retry++) {
		if ((ret = i2c_master_send(cypress_client, inBuf, inSize)) == inSize)
				break;
		dev_err(&cypress_client->dev,
						"%s: I2C retry %d\n",
						__func__, retry + 1);

	}

	if (retry == CYPRESS4000_I2C_RETRY_TIMES) {
		dev_err(&cypress_client->dev,
						"%s: I2C transfer over retry limit,ret is %d\n",
						__func__,ret);
		return CYRET_ERR_COMM_MASK;
	}

	    msleep(50);

	for (retry = 0; retry < CYPRESS4000_I2C_RETRY_TIMES; retry++) {
		if ((ret = i2c_master_recv(cypress_client, outBuf, outSize)) == outSize)
				break;
		dev_err(&cypress_client->dev,
						"%s: I2C retry recv %d\n",
						__func__, retry + 1);

	}

	if (retry == CYPRESS4000_I2C_RETRY_TIMES) {
		dev_err(&cypress_client->dev,
						"%s: I2C transfer over retry limit recv,ret=%d\n",
						__func__,ret);
		return CYRET_ERR_COMM_MASK;
	}

    return CYRET_SUCCESS;
}


int CyBtldr_StartBootloadOperation(unsigned long expSiId, unsigned char expSiRev, unsigned long* blVer)
{
    const unsigned long SUPPORTED_BOOTLOADER = 0x010000;
    const unsigned long BOOTLOADER_VERSION_MASK = 0xFF0000;
    unsigned long i;
    unsigned long inSize = 0;
    unsigned long outSize = 0;
    unsigned long siliconId = 0;
    unsigned char inBuf[MAX_COMMAND_SIZE];
    unsigned char outBuf[MAX_COMMAND_SIZE];
    unsigned char siliconRev = 0;
    unsigned char status = CYRET_SUCCESS;
    int err;

    for (i = 0; i < MAX_FLASH_ARRAYS; i++)
        g_validRows[i] = NO_FLASH_ARRAY_DATA;

    err = CyBtldr_CreateEnterBootLoaderCmd(inBuf, &inSize, &outSize);
    dev_err(&cypress_client->dev, " buf: %p, inSize:%d, outSize: %d, MAX_COMMAND_SIZE: %d\n", inBuf, inSize, outSize, MAX_COMMAND_SIZE);

    if (CYRET_SUCCESS == err)
       err = CyBtldr_TransferData(inBuf, inSize, outBuf, outSize);
	else
		dev_err(&cypress_client->dev, "CyBtldr_CreateEnterBootLoaderCmd ret is [%x]\n", err);

    if (CYRET_SUCCESS == err)
        err = CyBtldr_ParseEnterBootLoaderCmdResult(outBuf, outSize, &siliconId, &siliconRev, blVer, &status);
	else
		dev_err(&cypress_client->dev, "CyBtldr_TransferData ret is [%x]\n", err);

    if (CYRET_SUCCESS == err)
    {
        if (CYRET_SUCCESS != status)
            err = status | CYRET_ERR_BTLDR_MASK;
        if (expSiId != siliconId || expSiRev != siliconRev)
            err = CYRET_ERR_DEVICE;
        else if ((*blVer & BOOTLOADER_VERSION_MASK) != SUPPORTED_BOOTLOADER)
            err = CYRET_ERR_VERSION;
    }

    return err;
}

int CyBtldr_ParseRowData(unsigned int bufSize, unsigned char* buffer, unsigned char* arrayId, unsigned short* rowNum, unsigned char* rowData, unsigned short* size, unsigned char* checksum)
{
    const unsigned short MIN_SIZE = 6; //1-array, 2-addr, 2-size, 1-checksum
    const int DATA_OFFSET = 5;

    unsigned int i;
    unsigned short hexSize;
    unsigned char hexData[MAX_BUFFER_SIZE];
    int err = CYRET_SUCCESS;

    if (bufSize <= MIN_SIZE)
        err = CYRET_ERR_LENGTH;
    else if (buffer[0] == ':')
    {
        err = CyBtldr_FromAscii(bufSize - 1, &buffer[1], &hexSize, hexData);

        *arrayId = hexData[0];
        *rowNum = (hexData[1] << 8) | (hexData[2]);
        *size = (hexData[3] << 8) | (hexData[4]);
        *checksum = (hexData[hexSize - 1]);

        if ((*size + MIN_SIZE) == hexSize)
        {
            for (i = 0; i < *size; i++)
            {
                rowData[i] = (hexData[DATA_OFFSET + i]);
            }
        }
        else
            err = CYRET_ERR_DATA;
    }
    else
        err = CYRET_ERR_CMD;

    return err;
}

int CyBtldr_ValidateRow(unsigned char arrayId, unsigned short rowNum)
{
    unsigned long inSize;
    unsigned long outSize;
    unsigned short minRow = 0;
    unsigned short maxRow = 0;
    unsigned char inBuf[MAX_COMMAND_SIZE];
    unsigned char outBuf[MAX_COMMAND_SIZE];
    unsigned char status = CYRET_SUCCESS;
    int err = CYRET_SUCCESS;

    if (arrayId < MAX_FLASH_ARRAYS)
    {
        if (NO_FLASH_ARRAY_DATA == g_validRows[arrayId])
        {
            err = CyBtldr_CreateGetFlashSizeCmd(arrayId, inBuf, &inSize, &outSize);
            if (CYRET_SUCCESS == err)
                err = CyBtldr_TransferData(inBuf, inSize, outBuf, outSize);
            if (CYRET_SUCCESS == err)
                err = CyBtldr_ParseGetFlashSizeCmdResult(outBuf, outSize, &minRow, &maxRow, &status);
            if (CYRET_SUCCESS != status)
                err = status | CYRET_ERR_BTLDR_MASK;

            if (CYRET_SUCCESS == err)
            {
                if (CYRET_SUCCESS == status)
                    g_validRows[arrayId] = (minRow << 16) + maxRow;
                else
                    err = status | CYRET_ERR_BTLDR_MASK;
            }
        }
        if (CYRET_SUCCESS == err)
        {
            minRow = (unsigned short)(g_validRows[arrayId] >> 16);
            maxRow = (unsigned short)g_validRows[arrayId];
            if (rowNum < minRow || rowNum > maxRow)
                err = CYRET_ERR_ROW;
        }
    }
    else
        err = CYRET_ERR_ARRAY;

    return err;
}


int CyBtldr_ProgramRow(unsigned char arrayID, unsigned short rowNum, unsigned char* buf, unsigned short size)
{
    const int TRANSFER_HEADER_SIZE = 11;

    unsigned char inBuf[MAX_COMMAND_SIZE];
    unsigned char outBuf[MAX_COMMAND_SIZE];
    unsigned long inSize;
    unsigned long outSize;
    unsigned long offset = 0;
    unsigned short subBufSize;
    unsigned char status = CYRET_SUCCESS;
    unsigned int MaxTransferSize = 32u;


    int err = CyBtldr_ValidateRow(arrayID, rowNum);

    //Break row into pieces to ensure we don't send too much for the transfer protocol
    while ((CYRET_SUCCESS == err) && ((size - offset + TRANSFER_HEADER_SIZE) > MaxTransferSize))
    {
        subBufSize = (unsigned short)(MaxTransferSize - TRANSFER_HEADER_SIZE);

        err = CyBtldr_CreateSendDataCmd(&buf[offset], subBufSize, inBuf, &inSize, &outSize);
        if (CYRET_SUCCESS == err)
            err = CyBtldr_TransferData(inBuf, inSize, outBuf, outSize);
        if (CYRET_SUCCESS == err)
            err = CyBtldr_ParseSendDataCmdResult(outBuf, outSize, &status);
        if (CYRET_SUCCESS != status)
            err = status | CYRET_ERR_BTLDR_MASK;

        offset += subBufSize;
    }

    if (CYRET_SUCCESS == err)
    {
        subBufSize = (unsigned short)(size - offset);

        err = CyBtldr_CreateProgramRowCmd(arrayID, rowNum, &buf[offset], subBufSize, inBuf, &inSize, &outSize);
        if (CYRET_SUCCESS == err)
            err = CyBtldr_TransferData(inBuf, inSize, outBuf, outSize);
        if (CYRET_SUCCESS == err)
            err = CyBtldr_ParseProgramRowCmdResult(outBuf, outSize, &status);
        if (CYRET_SUCCESS != status)
            err = status | CYRET_ERR_BTLDR_MASK;
    }

    return err;
}

int CyBtldr_VerifyRow(unsigned char arrayID, unsigned short rowNum, unsigned char checksum)
{
    unsigned char inBuf[MAX_COMMAND_SIZE];
    unsigned char outBuf[MAX_COMMAND_SIZE];
    unsigned long inSize = 0;
    unsigned long outSize = 0;
    unsigned char rowChecksum = 0;
    unsigned char status = CYRET_SUCCESS;

    int err = CyBtldr_ValidateRow(arrayID, rowNum);
    if (CYRET_SUCCESS == err)
        err = CyBtldr_CreateVerifyRowCmd(arrayID, rowNum, inBuf, &inSize, &outSize);
    if (CYRET_SUCCESS == err)
        err = CyBtldr_TransferData(inBuf, inSize, outBuf, outSize);
    if (CYRET_SUCCESS == err)
        err = CyBtldr_ParseVerifyRowCmdResult(outBuf, outSize, &rowChecksum, &status);
    if (CYRET_SUCCESS != status)
        err = status | CYRET_ERR_BTLDR_MASK;
    if ((CYRET_SUCCESS == err) && (rowChecksum != checksum))
        err = CYRET_ERR_CHECKSUM;

    return err;
}

int CyBtldr_EndBootloadOperation(struct i2c_client *client)
{
    const unsigned char RESET = 0x00;
    unsigned long inSize;
    unsigned long outSize;
    unsigned char inBuf[MAX_COMMAND_SIZE];

    int err = CyBtldr_CreateExitBootLoaderCmd(RESET, inBuf, &inSize, &outSize);
    if (CYRET_SUCCESS == err)
    {
        err = i2c_master_send(client,inBuf, inSize);

        if ( err <= 0)
            return CYRET_ERR_COMM_MASK;
		else
			return CYRET_SUCCESS;
    }
	else
	{
		return CYRET_ERR_COMM_MASK;
	}
}

int cypress_firmware_update(struct i2c_client *client,const char *bootloadImagePtr[],unsigned int lineCount)
{
  int val;
  unsigned char rowData[128];
  unsigned int lineLen;
  unsigned long  siliconID;
  unsigned char siliconRev;
  unsigned char packetChkSumType;
  unsigned int lineCntr ;
  unsigned long blVer=0;
  unsigned int  err;
  unsigned char arrayId;
  unsigned short rowNum;
  unsigned short rowSize;
  unsigned char checksum ;
  unsigned char checksum2;

  cypress_client = client;

  val = i2c_smbus_write_byte_data(client, CYPRESS4000_TOUCHKEY_ENTER_BOOTLOADER_MODE, CYPRESS4000_TOUCHKEY_ENTER_BOOTLOADER_COMMON_VALUE);
  if (val < 0)
  {
	dev_err(&client->dev, "cypress write enter bootloader cmm error [%d],but it may be in bootloader mode already, so keep updating fw", val);
//    return val;
  }
  client->addr = CYPRESS_BOOTLOADER_MODE_I2C_ADDR;

  msleep(60);
  /* Initialize line counter */
  lineCntr = 0u;

  /* Get length of the first line in cyacd file*/
  lineLen = strlen(bootloadImagePtr[lineCntr]);

  /* Parse the first line(header) of cyacd file to extract siliconID, siliconRev and packetChkSumType */
  err = CyBtldr_ParseHeader(lineLen ,(unsigned char *)bootloadImagePtr[lineCntr] , &siliconID , &siliconRev ,&packetChkSumType);

  /* Set the packet checksum type for communicating with bootloader. The packet checksum type to be used 
  * is determined from the cyacd file header information */
  CyBtldr_SetCheckSumType((CyBtldr_ChecksumType)packetChkSumType);

  if(err == CYRET_SUCCESS)
  {
	  /* Start Bootloader operation */
	  err = CyBtldr_StartBootloadOperation(siliconID, siliconRev ,&blVer);
	  lineCntr++ ;
	  while((err == CYRET_SUCCESS)&& ( lineCntr <  lineCount ))
	  {
		  /* Get the string length for the line*/
		  lineLen =  strlen(bootloadImagePtr[lineCntr]);
		  /*Parse row data*/
		  err = CyBtldr_ParseRowData((unsigned int)lineLen,(unsigned char *)bootloadImagePtr[lineCntr], &arrayId, &rowNum, rowData, &rowSize, &checksum);

		  if (CYRET_SUCCESS == err)
		  {
			  /* Program Row */
			  err = CyBtldr_ProgramRow(arrayId, rowNum, rowData, rowSize);
			  if (CYRET_SUCCESS == err)
			  {
				  /* Verify Row . Check whether the checksum received from bootloader matches
				  * the expected row checksum stored in cyacd file*/
				  checksum2 = (unsigned char)(checksum + arrayId + rowNum + (rowNum >> 8) + rowSize + (rowSize >> 8));
				  err = CyBtldr_VerifyRow(arrayId, rowNum, checksum2);
			  }
			  else
			  {
                    dev_err(&client->dev, "CyBtldr_ProgramRow ret [%x]\n", err);

			  }
		  }
		  else
		  {
		  	dev_err(&client->dev, "CyBtldr_ParseRowData ret [%x]\n", err);
		  }
		  /* Increment the linCntr */
		  lineCntr ++;
	  }
	  /* End Bootloader Operation */
	  CyBtldr_EndBootloadOperation(client);
  }
  else
  {
    dev_err(&client->dev, "CyBtldr_ParseHeader err [%x]\n", err);
  }

  client->addr = CYPRESS_I2C_ADDR;
  return(err);

}

