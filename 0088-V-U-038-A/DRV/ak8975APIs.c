/******************** (C) COPYRIGHT 2013 Uniband Electronic Corp. **************
* File Name          : ak8975APIs.c
* Author             : Ethan Chung
* Version            : v1.0.0.0
* Date               : 25-Jun-2013
* Description        : This file provides a set of functions needed to manage the
*                      communication between SPI peripheral and mpu6050 Driver.
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "ak8975APIs.h"
#include "ak8975REGs.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/*******************************************************************************
  * @brief  Initialize Ak8975
  * @param  None
  * @retval None
  */
void Ak8975Init(unsigned char DevID)
{
  unsigned char buf = 0;

	I2C_Write(0xD0, 0x6A, 1, &buf);
	buf = 0x82;
	I2C_Write(0xD0, 0x37, 1, &buf);

	I2C_Read(DevID, WIA, 1, &buf);

	buf = 0x01;
	I2C_Write(DevID, CNTL, 1, &buf);
}

/*******************************************************************************
  * @brief  Ak8975 Tx.
  * @param  DestAddr: To
  * @param  pData: The data Tx
  * @param  DataLen: The number of data need to Tx
  * @retval Ref to system.h - StatusTypeDef.
  */
void Ak8975ReadMag(unsigned char DevID, short *pX, short *pY, short *pZ)
{
  unsigned char buf[2] = {0, 0};

  buf[0] = 0x01;
	I2C_Write(DevID, CNTL, 1, &buf[0]);
	do {
		buf[0] = 0x00;
	  I2C_Read(DevID, ST1, 1, &buf[0]);
	} while (buf[0] != 0x01);

	I2C_Read(DevID, HXL, 1, &buf[0]);
	I2C_Read(DevID, HXH, 1, &buf[1] );
	*pX = (*(short *)buf);
	I2C_Read(DevID, HYL, 1, &buf[0]);
	I2C_Read(DevID, HYH, 1, &buf[1]);
	*pY = (*(short *)buf);
	I2C_Read(DevID, HZL, 1, &buf[0]);
	I2C_Read(DevID, HZH, 1, &buf[1]);
	*pZ = (*(short *)buf);
  I2C_Read(DevID, ST2, 1, &buf[0]);
}

/******** (C) COPYRIGHT 2013 Uniband Electronic Corp. ******END OF FILE********/
