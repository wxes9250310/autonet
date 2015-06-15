/******************** (C) COPYRIGHT 2014 STMicroelectronics *******************
* File Name          : bh1750fviAPIs.h
* Author             : Ethan Chung
* Version            : V1.0.0.0
* Date               : 04/11/2014
* Description        : This file provides a set of functions needed to manage the
*                      communication between I2C peripheral and BH1750FVI Driver.
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "bh1750fviREGs.h"
#include "bh1750fviAPIs.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/*******************************************************************************
* Function Name  : BH1750FVI_Init
* Description    : Initializes peripherals used by the Light Sensor driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Bh1750fviInit(unsigned char DevID)
{
}

/*******************************************************************************
* Function Name  : BH1750FVI_ReadTemperature
* Description    : Read Lux from the Light Sensor driver.
* Input          : None
* Output         : - Lx: Lux from Light Sensor.
* Return         : None
*******************************************************************************/
void Bh1750fviReadLx(unsigned char DevID, unsigned short *pLx)
{
  unsigned char TempBuffer[2] = {0x00, 0x00};
  unsigned short *p = (unsigned short *)&TempBuffer[0];
  unsigned char swap = 0x00;

  I2C_Read(DevID, LUX_REGISTER_H, 2, &TempBuffer[0]);
  swap = TempBuffer[1];
  TempBuffer[1] = TempBuffer[0];
  TempBuffer[0] = swap;
  *pLx = (float)*p / 1.2;
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
