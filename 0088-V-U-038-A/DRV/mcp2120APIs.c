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

#define COMMUNICATION_START_BC 0xFF			// broadcast
#define COMMUNICATION_HEADER_BC 0xFF    // broadcast

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
unsigned char CommandTxBuffer[64] = {COMMUNICATION_START, COMMUNICATION_HEADER};
unsigned char CommandTxBuffer_BC[64] = {COMMUNICATION_START_BC, COMMUNICATION_HEADER_BC};
unsigned char CommandRxBuffer[64] = {0x0};
unsigned char CommandRxBufferLen = 0x00;
unsigned char CommandRxBuffer2[64] = {0x0};
unsigned char CommandRxBufferLen2 = 0x00;
unsigned char CommandRxBuffer_BC[64] = {0x0};
unsigned char CommandRxBufferLen_BC = 0x00;
unsigned char CommandRxBuffer2_BC[64] = {0x0};
unsigned char CommandRxBufferLen2_BC = 0x00;

uint8_t IRTxState = 0;
uint8_t IRRxState = 0;
uint8_t count =0;

/* Exported variables --------------------------------------------------------*/
extern TimObjTypeDef_s TimObj;
/* Private function prototypes -----------------------------------------------*/
static unsigned char Mcp2120ComplementCalc(unsigned char *p, unsigned char num);

/* Exported functions --------------------------------------------------------*/
/*******************************************************************************
* Function Name  : MCP2120_Init
* Description    : Initializes peripherals used by the IR Sensor driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Mcp2120Init(void)
{
}

/*******************************************************************************
* Function Name  : MCP2120_write Infrared 
* Description    :
* Input          : None
* Output         : 
* Return         : None
*******************************************************************************/
void Mcp2120Tx(unsigned char *p, unsigned short p_len, int COM)
{
	IRTxState = 1;
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
	}
	IRTxState = 0;
}

void IR_write(unsigned char *p, unsigned short p_len, int COM)
{
	IRTxState = 1; 
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
	}
	IRTxState = 0;
}

void IR_broadcast(uint16_t addr, uint8_t type, int COM)
{
	unsigned char p[2];
	unsigned short length = 2;
	p[0] = (unsigned char) type;
	p[1] = (unsigned char) addr;
	CommandTxBuffer_BC[COMMUNICATION_DATALEN_OFFSET] = length;
	memcpy(&CommandTxBuffer_BC[COMMUNICATION_DATA_OFFSET], p, length);
	CommandTxBuffer_BC[length + 3] = Mcp2120ComplementCalc(CommandTxBuffer_BC, length + 3);
	CommandTxBuffer_BC[length + 4] = 0x0D;
  CommandTxBuffer_BC[length + 5] = 0x0A;
	
	if(COM == 1)
	{
		while(COM1_Tx(CommandTxBuffer_BC, length + 6) == ERROR);
	}
	if(COM == 2)
	{
		while(COM2_Tx(CommandTxBuffer_BC, length + 6) == ERROR);
	}
}


/*******************************************************************************
* Function Name  : MCP2120_Read Infrared 
* Description    :
* Input          : None
* Output         : 
* Return         : None
*******************************************************************************/
void Mcp2120Proc(unsigned char *p, unsigned short* Length, int COM)
{
  unsigned char Checksum = 0;
  IRRxState = 1;
	
	if(COM == 1){
		if (CommandRxBufferLen != 0x00) {  
			if ((Checksum = Mcp2120ComplementCalc(CommandRxBuffer, CommandRxBufferLen - 1)) == CommandRxBuffer[CommandRxBufferLen - 1]) {
				memcpy(p, CommandRxBuffer, CommandRxBufferLen);
				*Length =  CommandRxBufferLen;
			}
			CommandRxBufferLen = 0x00;
		}
	}
	else if(COM == 2){
		if (CommandRxBufferLen2 != 0x00) { 
			if ((Checksum = Mcp2120ComplementCalc(CommandRxBuffer2, CommandRxBufferLen2 - 1)) == CommandRxBuffer2[CommandRxBufferLen2 - 1]) {
				memcpy(p, CommandRxBuffer2, CommandRxBufferLen2);
				*Length =  CommandRxBufferLen2;
			}
			CommandRxBufferLen2 = 0x00;
		}
	}
	IRRxState = 0;
}

uint8_t IR_read(unsigned char *p, unsigned short* Length, int COM)
{
  unsigned char Checksum = 0;
	uint8_t BeaconCheckFlag;
	uint8_t receivedFlag = 0;
  IRRxState = 1;
	
	if(COM == 1){
		if (CommandRxBufferLen != 0x00) {  
			if ((Checksum = Mcp2120ComplementCalc(CommandRxBuffer, CommandRxBufferLen - 1)) == CommandRxBuffer[CommandRxBufferLen - 1]) {
				memcpy(p, CommandRxBuffer, CommandRxBufferLen);
				*Length =  CommandRxBufferLen;
				
				BeaconCheckFlag = IRheaderCheck_AutoNet(p);
				if(!BeaconCheckFlag){
					receivedFlag = 1;
				}
			}
			CommandRxBufferLen = 0x00;
		}
	}
	else if(COM == 2){
		if (CommandRxBufferLen2 != 0x00) { 
			if ((Checksum = Mcp2120ComplementCalc(CommandRxBuffer2, CommandRxBufferLen2 - 1)) == CommandRxBuffer2[CommandRxBufferLen2 - 1]) {
				memcpy(p, CommandRxBuffer2, CommandRxBufferLen2);
				*Length =  CommandRxBufferLen2;
				
			  BeaconCheckFlag = IRheaderCheck_AutoNet(p);
				if(!BeaconCheckFlag){
					receivedFlag = 1;
				}
			}
			CommandRxBufferLen2 = 0x00;
		}
	}
	IRRxState = 0;
	return receivedFlag;
}

uint8_t IR_broadcast_read(int COM)
{
  unsigned char Checksum = 0;
	uint8_t BeaconCheckFlag;
	uint8_t receivedFlag = 0;
  IRRxState = 1;
	
	CommandRxBufferLen_BC  =0;
	CommandRxBufferLen2_BC =0;
	
	if(COM == 1){
		if (CommandRxBufferLen != 0x00) {  
			if ((Checksum = Mcp2120ComplementCalc(CommandRxBuffer, CommandRxBufferLen - 1)) == CommandRxBuffer[CommandRxBufferLen - 1]) {
				memcpy(CommandRxBuffer_BC, CommandRxBuffer, CommandRxBufferLen);
				CommandRxBufferLen_BC =  CommandRxBufferLen;
				
				BeaconCheckFlag = IRheaderCheck_AutoNet(CommandRxBuffer_BC);
				if(BeaconCheckFlag){
					receivedFlag = 1;
				}
			}
			CommandRxBufferLen = 0x00;
		}
	}
	else if(COM == 2){
		if (CommandRxBufferLen2 != 0x00) { 
			if ((Checksum = Mcp2120ComplementCalc(CommandRxBuffer2, CommandRxBufferLen2 - 1)) == CommandRxBuffer2[CommandRxBufferLen2 - 1]) {
				memcpy(CommandRxBuffer2_BC, CommandRxBuffer2, CommandRxBufferLen2);
				CommandRxBufferLen2_BC =  CommandRxBufferLen2;
				
			  BeaconCheckFlag = IRheaderCheck_AutoNet(CommandRxBuffer2_BC);
				if(BeaconCheckFlag){
					receivedFlag = 1;
				}
			}
			CommandRxBufferLen2 = 0x00;
		}
	}
	IRRxState = 0;
	return receivedFlag;
}

int IRheaderCheck_AutoNet(unsigned char* p){
	if(p[0] == COMMUNICATION_START_BC && p[1] == COMMUNICATION_HEADER_BC)
		return 1;
	else
		return 0;
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
