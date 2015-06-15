/******************** (C) COPYRIGHT 2013 Uniband Electronic Corp. **************
* File Name          : us2400.c
* Author             : Ethan Chung
* Version            : v1.0.0.0
* Date               : 25-Jun-2013
* Description        : This file provides a set of functions needed to manage the
*                      communication between SPI peripheral and US2400 Driver.
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "st.h"
#include "spi.h"
#include "us2400APIs.h"
#include "us2400REGs.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
unsigned char Buffer[256] = {0};
unsigned short BufferLen = 0;
unsigned char Value[256] = {0};
unsigned char TxBuff[256] = {0};
unsigned char RxBuff[256] = {0};
unsigned char TxTempBuff[256] = {0};
unsigned char SerialNumber = 0x00;

/* Exported variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static unsigned char Us2400ReadLongRegBlock(unsigned short RegAddr, unsigned short RegNum, unsigned char *pRegVal);
static unsigned char Us2400WriteLongRegBlock(unsigned short RegAddr, unsigned short RegNum, unsigned char *pRegVal);
static void Us2400SetRecoveryTime(unsigned short wakecnt, unsigned short waketime);

/* Exported functions --------------------------------------------------------*/
/*******************************************************************************
  * @brief  Initialize US2400
  * @param  None
  * @retval None
  */
void Us2400Init(unsigned short Frequency, unsigned short PanID, unsigned short Addr, unsigned char Power)
{
  unsigned char *p = 0x00;

	Us2400WriteShortReg(SOFTRST, 0x07);
	Us2400WriteShortReg(GATECLK, 0x20);
  while(1) {
    if ((Us2400ReadShortReg(GATECLK) & 0x20) == 0x20) {
      break;
    }
  }

	Us2400WriteShortReg(PACON1, 0x08);
	Us2400WriteShortReg(FIFOEN, 0x94);
	Us2400WriteShortReg(TXPEMISP, 0x95);
	Us2400WriteShortReg(BBREG3, 0x50);
	Us2400WriteShortReg(BBREG5, 0x07);
	Us2400WriteShortReg(BBREG6, 0x40);

	if ((Frequency > 2400) && (Frequency < 2485)) {
  	Frequency = ((((Frequency - 2400) / 5) - 1) << 4) | 0x03;
	}
	else {
		Frequency = 0x03;
	}
	
	if(Power <= 0x1f) //0 - -8.3dBm
  {
	  Us2400WriteLongReg(RFCTRL3, (Power & 0x1f) << 3);
  }
	else {
    Us2400WriteLongReg(RFCTRL3, Us2400ReadLongReg(RFCTRL3) & 0x07);
	}
	
	Us2400WriteLongReg(RFCTRL0, Frequency);
	Us2400WriteLongReg(RFCTRL1, 0x01);
	Us2400WriteLongReg(RFCTRL2, 0x74);
	Us2400WriteLongReg(RFCTRL4, 0x06);
	Us2400WriteLongReg(RFCTRL6, 0x10);
	Us2400WriteLongReg(RFCTRL7, 0xEC);
	Us2400WriteLongReg(RFCTRL8, 0x8C);
	Us2400WriteLongReg(GPIODIR, 0x00);
	Us2400WriteLongReg(SECCTRL, 0x20);
	Us2400WriteLongReg(RFCTRL50, 0x07);
  Us2400WriteLongReg(RFCTRL51, 0xC0);
	Us2400WriteLongReg(RFCTRL52, 0x01);
	Us2400WriteLongReg(RFCTRL53, Us2400ReadLongReg(RFCTRL53) & 0xF0); // TX power
	Us2400WriteLongReg(RFCTRL59, 0x00);
	Us2400WriteLongReg(RFCTRL73, 0x80);
	Us2400WriteLongReg(RFCTRL74, 0xE5);
	Us2400WriteLongReg(RFCTRL75, 0x13);
	Us2400WriteLongReg(RFCTRL76, 0x07);

  p = (unsigned char *)&PanID;
	Us2400WriteShortReg(PANIDL, *p++);
  Us2400WriteShortReg(PANIDH, *p);
  p = (unsigned char *)&Addr;
	Us2400WriteShortReg(SADRL, *p++);
  Us2400WriteShortReg(SADRH, *p);

  Us2400WriteShortReg(INTMSK, 0x00);
	Us2400WriteShortReg(SOFTRST, 0x02);
	Us2400WriteShortReg(RFCTL, 0x04);
	Us2400WriteShortReg(RFCTL, 0x00);
	Us2400WriteShortReg(RFCTL, 0x02);
  Delay(1);
  Us2400WriteShortReg(RFCTL, 0x01);
  Delay(1);
  Us2400WriteShortReg(RFCTL, 0x00);
}

/*******************************************************************************
  * @brief  US2400 Tx.
  * @param  DestAddr: To
  * @param  pData: The data Tx
  * @param  DataLen: The number of data need to Tx
  * @retval Ref to system.h - StatusTypeDef.
  */
void Us2400Tx(unsigned short DestAddr, unsigned char *pData, unsigned short DataLen)
{	
	unsigned char *p = (unsigned char *)&DestAddr;
	
	memset(TxBuff, 0x00, sizeof(TxBuff));
	TxBuff[0] = 0x0B;
	TxBuff[1] = TxBuff[0] + DataLen;
	TxBuff[2] = 0x01;
	TxBuff[3] = 0x88;
    TxBuff[4] = SerialNumber++;
	TxBuff[5] = Us2400ReadShortReg(PANIDL);
	TxBuff[6] = Us2400ReadShortReg(PANIDH);
	p = (unsigned char *)&DestAddr;
	TxBuff[7] = *p++;
	TxBuff[8] = *p;
	TxBuff[9] = TxBuff[5];
	TxBuff[10] = TxBuff[6];
	TxBuff[11] = Us2400ReadShortReg(SADRL);
	TxBuff[12] = Us2400ReadShortReg(SADRH);
	memcpy(&TxBuff[TxBuff[0]+2], pData, DataLen);

  do {
    Us2400WriteLongRegBlock(TX_FIFO, TxBuff[1] + 0x02, TxBuff);
		memset(TxTempBuff, 0x00, sizeof(TxTempBuff));
		Us2400ReadLongRegBlock(TX_FIFO, TxBuff[1] + 0x02, TxTempBuff);
	} while(memcmp(TxBuff, TxTempBuff, TxBuff[1] + 0x02) != 0);

	Us2400WriteShortReg(TXNTRIG, 0x01);
}

/*******************************************************************************
  * @brief  US2400 Rx.
  * @param  SrcAddr: From
  * @param  pData: The data Rx
  * @param  pDataLen: Rx data length
  * @retval None
  */
void Us2400Rx(unsigned char *pData, unsigned char *pDataLen, unsigned char *pLqi, unsigned char *pRssi)
{
  memset(RxBuff, 0x00, sizeof(RxBuff));
	Us2400ReadLongRegBlock(RX_FIFO, Us2400ReadLongReg(RX_FIFO) + 9, RxBuff);
	
	/****************************************************************************
	 * Packet format(RxBuff): Frame Length(1) = 11 + N + 2
   *                        MAC header(11): frame control(2)
	 *                                        sequence number(1)
	 *                                        destination PANID(2)
	 *                                        destination address(2)
	 *                                        source PANID(2)
	 *                                        source address(2)
	 *                        Payload(N)
	 *                        MAC FCS(2)
	 *                        LQI(1)
	 *                        RSSI(1)
	 * Packet format(Data):   Payload Length(1) = N
   *                        MAC header(11): frame control(2)
	 *                                        sequence number(1)
	 *                                        destination PANID(2)
	 *                                        destination address(2)
	 *                                        source PANID(2)
	 *                                        source address(2)
	 *                        Payload(N)
	 ****************************************************************************/
	*pDataLen = RxBuff[0] + 0x01; // frame length - FCS + packet length
	memcpy(pData, RxBuff, *pDataLen);
	pData[0] = RxBuff[0] - 0x0B - 0x02;
	*pLqi = RxBuff[*pDataLen];
	*pRssi = RxBuff[*pDataLen+ 0x01];
}
/*******************************************************************************
  * @brief  US2400 Rx.
  * @param  SrcAddr: From
  * @param  pData: The data Rx
  * @param  pDataLen: Rx data length
  * @retval None
  */
void Us2400RxFIFODump(unsigned char *pData, unsigned char *pDataLen)
{
  memset(pData, 0x00, (unsigned int)*pDataLen);
	Us2400ReadLongRegBlock(RX_FIFO, Us2400ReadLongReg(RX_FIFO) + 9, pData);
	*pDataLen = pData[0] + 2;
}
/*******************************************************************************
  * @brief  US2400 Into Deep Sleep.
  * @param  None
  * @retval None
  */
void Us2400IntoDeepSleep(void)
{
  Us2400WriteShortReg(RXFLUSH, 0x40);
	Us2400SetRecoveryTime(0x12, 0x42);
  Us2400WriteLongReg(RFCTRL55, 0x00);
	Us2400WriteLongReg(RFCTRL77, 0x18);
	Us2400WriteLongReg(RFCTRL50, Us2400ReadLongReg(RFCTRL50) & ~0x10);
	Us2400WriteShortReg(RXFLUSH, Us2400ReadShortReg(RXFLUSH) & ~0x80);
	Us2400WriteShortReg(SLPACK, Us2400ReadShortReg(SLPACK) | 0x80);
}

/*******************************************************************************
  * @brief  US2400 Set Recovery Time.
  * @param  None
  * @retval None
  */
void Us2400SetRecoveryTime(unsigned short wakecnt, unsigned short waketime)
{
	Us2400WriteShortReg(RFCTL, (unsigned char)((wakecnt & 0x0180) >> 4));
	Us2400WriteShortReg(SLPACK, (unsigned char)(wakecnt & 0x007f));
	Us2400WriteLongReg(WAKETIMEH, (unsigned char)((waketime >> 8) & 0x0007));
	Us2400WriteLongReg(WAKETIMEL, (unsigned char)(waketime & 0x00ff));
}

/*******************************************************************************
  * @brief  US2400 Resume.
  * @param  None
  * @retval None
  */
void Us2400Resume(void)
{
	while ((Us2400ReadShortReg(ISRSTS) & 0x40) != 0x40);
	Us2400WriteShortReg(RXFLUSH, Us2400ReadShortReg(RXFLUSH) | 0x80);
  Us2400WriteLongReg(RFCTRL50, Us2400ReadLongReg(RFCTRL50) | 0x10);
}

/*******************************************************************************
  * @brief  Us2400 Set Tx power.
  * @param  Level:
  * @retval None
  */
void Us2400SetTxPower(unsigned char Level)
{
  if(Level <= 0x1f) //0 - -8.3dBm
  {
	  Us2400WriteLongReg(RFCTRL3, (Level & 0x1f) << 3);
  }
	else {
    Us2400WriteLongReg(RFCTRL3, Us2400ReadLongReg(RFCTRL3) & 0x07);
	}
  Us2400WriteLongReg(RFCTRL53, Us2400ReadLongReg(RFCTRL53) & 0xF0);
  Us2400WriteLongReg(RFCTRL74, 0xE5);
}

/*******************************************************************************
  * @brief  US2400 Single Tone.
  * @param  State: ON/OFF
  * @retval
  */
void Us2400SingleTone(unsigned char State)
{
	if (State == 0x01) {
		Us2400WriteLongReg(RFCTRL59, 0x01);
	  Us2400WriteLongReg(0x22A, 0x02);
	  Us2400WriteShortReg(RFCTL, 0x02);
	}
	else {
		Us2400WriteShortReg(RFCTL, 0x01);
	  Us2400WriteShortReg(RFCTL, 0x00);
	  Us2400WriteLongReg(0x22A, 0x00);
	  Us2400WriteLongReg(RFCTRL59, 0x00);
	}
}

/*******************************************************************************
  * @brief  US2400 Continue Wave.
  * @param  State: ON/OFF
  * @retval
  */
void Us2400ContinueWave(unsigned char State)
{
	if (State == 0x01) {
	  Us2400WriteLongReg(TX_FIFO + 1, 0x20);
	  Us2400WriteShortReg(BBREG0, Us2400ReadShortReg(BBREG0) | 0x04);
	  Us2400WriteShortReg(RFCTL, 0x02);
	  Us2400WriteShortReg(TXNTRIG, 0x01);
	}
	else {
		Us2400WriteShortReg(BBREG0, Us2400ReadShortReg(BBREG0) & ~0x04);
    Us2400WriteShortReg(SOFTRST, 0x02);
	  Us2400WriteShortReg(RFCTL, 0x04);
	  Us2400WriteShortReg(RFCTL, 0x00);
	  Us2400WriteShortReg(RFCTL, 0x02);
    Delay(1);
    Us2400WriteShortReg(RFCTL, 0x01);
    Delay(1);
    Us2400WriteShortReg(RFCTL, 0x00);
	}
}

/*******************************************************************************
  * @brief  US2400 read short register.
  * @param  RegAddr: 
  *         RegNum:
  *         pRegVal:
  * @retval 
  */
unsigned char Us2400ReadShortReg(unsigned char RegAddr)
{
  memset(Buffer, 0x00, sizeof(Buffer));
  BufferLen = 2;
  Buffer[0] = RegAddr << 1;
  SPI_Proc(Buffer, BufferLen);

  return Buffer[1];
}

/*******************************************************************************
  * @brief  US2400 write short register.
  * @param  RegAddr: 
  *         RegVal:
  * @retval None
  */
void Us2400WriteShortReg(unsigned char RegAddr, unsigned char RegVal)
{
  memset(Buffer, 0x00, sizeof(Buffer));
  BufferLen = 2;
  Buffer[0] = (RegAddr << 1) | 0x01;
	Buffer[1] = RegVal;
  SPI_Proc(Buffer, BufferLen);
}

/*******************************************************************************
  * @brief  US2400 read long register.
  * @param  RegAddr: 
  * @retval RegVal:
  */
unsigned char Us2400ReadLongReg(unsigned short RegAddr)
{
  unsigned char *p = (unsigned char *)&RegAddr;

  memset(Buffer, 0x00, sizeof(Buffer));
  BufferLen = 3;
  RegAddr = (((RegAddr << 1) | 0x800) << 4);
  Buffer[1] = *p++;
	Buffer[0] = *p;
  SPI_Proc(Buffer, BufferLen);

  return Buffer[2];
}

/*******************************************************************************
  * @brief  US2400 write long register.
  * @param  RegAddr: 
  *         RegVal:
  * @retval None
  */
void Us2400WriteLongReg(unsigned short RegAddr, unsigned char RegVal)
{
  unsigned char *p = (unsigned char *)&RegAddr;

  memset(Buffer, 0x00, sizeof(Buffer));
  BufferLen = 3;
  RegAddr = (((RegAddr << 1) | 0x801) << 4);
	Buffer[2] = RegVal;
  Buffer[1] = *p++;
	Buffer[0] = *p;
  SPI_Proc(Buffer, BufferLen);
}

/*******************************************************************************
  * @brief  US2400 read long register block.
  * @param  RegAddress: 
  *         RegNum:
  *         pRegVal:
  * @retval 
  */
static unsigned char Us2400ReadLongRegBlock(unsigned short RegAddr, unsigned short RegNum, unsigned char *pRegVal)
{
  unsigned char *p = (unsigned char *)&RegAddr;

  memset(Buffer, 0x00, sizeof(Buffer));
  BufferLen = RegNum + sizeof(RegAddr);
  RegAddr = (((RegAddr << 1) | 0x800) << 4);
  Buffer[1] = *p++;
	Buffer[0] = *p;
  SPI_Proc(Buffer, BufferLen);
  memcpy(pRegVal, &Buffer[sizeof(RegAddr)], RegNum);

  return 0;
}

/*******************************************************************************
  * @brief  US2400 write long register block.
  * @param  RegAddress: 
  *         RegNum:
  *         pRegVal:
  * @retval 
  */
static unsigned char Us2400WriteLongRegBlock(unsigned short RegAddr, unsigned short RegNum, unsigned char *pRegVal)
{
  unsigned char *p = (unsigned char *)&RegAddr;

  memset(Buffer, 0x00, sizeof(Buffer));
  BufferLen = RegNum + sizeof(RegAddr);
  RegAddr = (((RegAddr << 1) | 0x801) << 4);
  Buffer[1] = *p++;
	Buffer[0] = *p;
	memcpy(&Buffer[sizeof(RegAddr)], pRegVal, RegNum);
  SPI_Proc(Buffer, BufferLen);

  return 0;
}

/******** (C) COPYRIGHT 2013 Uniband Electronic Corp. ******END OF FILE********/
