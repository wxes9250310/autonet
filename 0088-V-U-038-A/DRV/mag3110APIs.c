/******************** (C) COPYRIGHT 2014 STMicroelectronics *******************
* File Name          : mag3110APIs.h
* Author             : Ethan Chung
* Version            : V1.0.0.0
* Date               : 04/11/2014
* Description        : This file provides a set of functions needed to manage the
*                      communication between I2C peripheral and TMP75 Driver.
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "mag3110REGs.h"
#include "mag3110APIs.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/*******************************************************************************
* Function Name  : Mag3110Init
* Description    : Initializes peripherals used by the Magnetic Sensor driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Mag3110Init(unsigned char DevID)
{
  unsigned char InitMode = 0x00;

	InitMode = 0x80;
	I2C_Write(DevID, CTRL_REG2, 1, &InitMode);
	InitMode = 0x01;
  I2C_Write(DevID, CTRL_REG1, 1, &InitMode);
}

/*******************************************************************************
* Function Name  : Mag3110ReadMag
* Description    : Read Temperature from the Magnetic Sensor driver.
* Input          : None
* Output         : - MagX,Y,Z: Magnetic from Senson.
* Return         : None
*******************************************************************************/
void Mag3110ReadMag(unsigned char DevID, short *pMagX, short *pMagY, short *pMagZ)
{
  unsigned char TempBuffer[2] = {0x00, 0x00};
  unsigned short *p = (unsigned short *)&TempBuffer[0];
  unsigned char swap = 0x00;

  I2C_Read(DevID, OUT_X_MSB, 2, &TempBuffer[0]);
  swap = TempBuffer[1];
  TempBuffer[1] = TempBuffer[0];
  TempBuffer[0] = swap;
  *pMagX = *p;

  I2C_Read(DevID, OUT_Y_MSB, 2, &TempBuffer[0]);
  swap = TempBuffer[1];
  TempBuffer[1] = TempBuffer[0];
  TempBuffer[0] = swap;
  *pMagY = *p;

  I2C_Read(DevID, OUT_Z_MSB, 2, &TempBuffer[0]);
  swap = TempBuffer[1];
  TempBuffer[1] = TempBuffer[0];
  TempBuffer[0] = swap;
  *pMagZ = *p;
}

/*******************************************************************************
* Function Name  : MAG3110 Read Temperature
* Description    : Read Temperature from the Temperature Sensor driver.
* Input          : None
* Output         : - Temperature: Temperature from Temperature Senson.
* Return         : None
*******************************************************************************/
void Mag3110ReadTemp(unsigned char DevID, char *pTemp)
{
  I2C_Read(DevID, DIE_TEMP, 1, (unsigned char *)pTemp);
	*pTemp = *pTemp + 25; // User Offset
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
