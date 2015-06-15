/******************** (C) COPYRIGHT 2014 STMicroelectronics *******************
* File Name          : tmp75APIs.h
* Author             : Ethan Chung
* Version            : V1.0.0.0
* Date               : 04/11/2014
* Description        : This file provides a set of functions needed to manage the
*                      communication between I2C peripheral and TMP75 Driver.
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "tmp75REGs.h"
#include "tmp75APIs.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/*******************************************************************************
* Function Name  : TMP75_Init
* Description    : Initializes peripherals used by the Temperature Sensor driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Tmp75Init(unsigned char DevID)
{
  unsigned char InitMode = 0x60; // Set Temperature Sensor resolution 12 Bit Mode.

  I2C_Write(DevID, CONFIGURATION_REGISTER, 1, &InitMode);
}

/*******************************************************************************
* Function Name  : TMP75_ReadTemperature
* Description    : Read Temperature from the Temperature Sensor driver.
* Input          : None
* Output         : - Temperature: Temperature from Temperature Senson.
* Return         : None
*******************************************************************************/
void Tmp75ReadTemperature(unsigned char DevID, float *pTemperature)
{
  unsigned char TempBuffer[2] = {0x00, 0x00};
  unsigned short *p = (unsigned short *)&TempBuffer[0];
  unsigned char swap = 0x00;

  I2C_Read(DevID, TEMPERATURE_REGISTER, 2, &TempBuffer[0]);
  swap = TempBuffer[1];
  TempBuffer[1] = TempBuffer[0];
  TempBuffer[0] = swap;
  *p = *p >> 4;
  *pTemperature = (float)*p * 0.0625;
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
