/******************** (C) COPYRIGHT 2014 STMicroelectronics *******************
* File Name          : mcp2120APIs.h
* Author             : Ethan Chung
* Version            : V1.0.0.0
* Date               : 04/11/2014
* Description        : This file provides a set of functions needed to manage the
*                      communication between I2C peripheral and MCP2120 Driver.
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "com.h"
#include "mcp2120REGs.h"
#include "mcp2120APIs.h"
#include "st.h"

#define FRONT 1
#define REAR 2
extern char FrontID;
extern char RearID;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define COMMUNICATION_START 0x7F
#define COMMUNICATION_HEADER 0x25
#define COMMUNICATION_DATALEN_OFFSET 0x02
#define COMMUNICATION_DATA_OFFSET 0x03

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
unsigned char CommandTxBuffer[64] = {COMMUNICATION_START, COMMUNICATION_HEADER};
unsigned char CommandRxBuffer[64] = {0x0};
unsigned char CommandRxBufferLen = 0x00;
unsigned char CommandRxBuffer2[64] = {0x0};
unsigned char CommandRxBufferLen2 = 0x00;

uint8_t count =0;

/* Exported variables --------------------------------------------------------*/
extern TimObjTypeDef_s TimObj;
/* Private function prototypes -----------------------------------------------*/
static unsigned char Mcp2120ComplementCalc(unsigned char *p, unsigned char num);

/* Exported functions --------------------------------------------------------*/
/*******************************************************************************
* Function Name  : MCP2120_Init
* Description    : Initializes peripherals used by the Temperature Sensor driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

// unused initialization
/*
void Mcp2120Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIOB->BSRR = GPIO_Pin_15;  //MODE
  GPIOB->BSRR = GPIO_Pin_7;   //ENABLE
  GPIOB->BRR = GPIO_Pin_6;    //RESET

  Delay(50);

  GPIOB->BSRR = GPIO_Pin_6;
  GPIOB->BSRR = GPIO_Pin_7;
  GPIOB->BSRR = GPIO_Pin_15;
}
*/

/*******************************************************************************
* Function Name  : MCP2120_ReadTemperature
* Description    : Read Temperature from the Temperature Sensor driver.
* Input          : None
* Output         : - Temperature: Temperature from Temperature Senson.
* Return         : None
*******************************************************************************/
void Mcp2120Tx(unsigned char *p, unsigned short p_len, int COM)
{
	CommandTxBuffer[COMMUNICATION_DATALEN_OFFSET] = p_len;
	memcpy(&CommandTxBuffer[COMMUNICATION_DATA_OFFSET], p, p_len);
	CommandTxBuffer[p_len + 3] = Mcp2120ComplementCalc(CommandTxBuffer, p_len + 3);
	CommandTxBuffer[p_len + 4] = 0x0D;
  CommandTxBuffer[p_len + 5] = 0x0A;
	if(COM == 1)
	{
		while(COM1_Tx(CommandTxBuffer, p_len + 6) == ERROR);
	}
	if(COM == 2)
	{
		while(COM2_Tx(CommandTxBuffer, p_len + 6) == ERROR);
		count ++;
	}
}

/*******************************************************************************
* Function Name  : MCP2120_ReadTemperature
* Description    : Read Temperature from the Temperature Sensor driver.
* Input          : None
* Output         : - Temperature: Temperature from Temperature Senson.
* Return         : None
*******************************************************************************/
void Mcp2120Proc(unsigned char *p, unsigned short* Length, int COM)
{
  unsigned char Checksum = 0;

	if(COM == 1){
		if (CommandRxBufferLen != 0x00) {  // FRONT Rx
			if ((Checksum = Mcp2120ComplementCalc(CommandRxBuffer, CommandRxBufferLen - 1)) == CommandRxBuffer[CommandRxBufferLen - 1]) {
				memcpy(p, CommandRxBuffer, CommandRxBufferLen);
				*Length =  CommandRxBufferLen;
				//TimObj.Tim[TIM_IRTO_R].Ticks = 1000;
			}
			CommandRxBufferLen = 0x00;
		}
	}
	else if(COM == 2){
		if (CommandRxBufferLen2 != 0x00) { // REAR Rx
			if ((Checksum = Mcp2120ComplementCalc(CommandRxBuffer2, CommandRxBufferLen2 - 1)) == CommandRxBuffer2[CommandRxBufferLen2 - 1]) {
				memcpy(p, CommandRxBuffer2, CommandRxBufferLen2);
				*Length =  CommandRxBufferLen2;
				//TimObj.Tim[TIM_IRTO_F].Ticks = 1000;
			}
			CommandRxBufferLen2 = 0x00;
		}
	}
}

/*******************************************************************************
* Function Name  : SmokedetReadSmokeValue
* Description    : Read Temperature from the Temperature Sensor driver.
* Input          : None
* Output         : - Temperature: Temperature from Temperature Senson.
* Return         : None
*******************************************************************************/
static unsigned char Mcp2120ComplementCalc(unsigned char *p, unsigned char num)
{
  unsigned char v = 0;

	for (;num > 0; num--) {
    v += *p++;
	}

	return 0xFF - (v + 1);
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
