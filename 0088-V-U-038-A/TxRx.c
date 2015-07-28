/******************** (C) COPYRIGHT 2014 STMicroelectronics *******************
* File Name          : TxRx.c
* Author             : Ed Kung & Cheng-Han Wu
* Version            : V1.0.0.0
* Date               : 06/01/2014
* Description        : 
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "TxRx.h"
#include "string.h"
#include "lea6sAPIs.h"
#include "stdio.h"
#include "stdlib.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define TXBUFFERSIZE   0xFF
#define TX_PAUSE 			 1000
#define RX_OFFSET			 12

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern TimObjTypeDef_s TimObj;
extern uint8_t ID;									
extern uint8_t TYPE;
extern uint8_t CommanderID;
extern uint8_t type;
extern uint8_t pRxData[]; 


uint8_t Data[128] = {0}; 
uint8_t DataLen = 128;
uint8_t RFTxOccupied = 0;
uint8_t RFRxOccupied = 0;

uint8_t RFTxBuffer[TXBUFFERSIZE] = {0};
uint8_t RFLqi = 0;
uint8_t RFRssi = 0;
uint8_t rssi = 0;
uint8_t RSSI_BC = 0;
uint8_t RFTxState = 0;
uint8_t RFRxState = 0;
uint8_t RFSleepState = 0;
uint8_t TimeoutFlag = 1;
uint8_t lightState=0;
unsigned char DataLen_temp;


enum{
	false,
	true,
};

enum{
		Type_Controller = 0x00,			// delete? 
		Type_Light = 0x01,
		Type_Switch = 0x02,
};


//uint16_t Data_table[NUMOFDEVICE][NUMOFSENSOR];

/* Exported variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RF_Tx(uint16_t destAddr, uint8_t *data, uint16_t dataLen)
{
	RFTxOccupied = 1;
	TimObj.TimeoutFlag &= (~TIMOUT_FLAG_100MS);
	TimObj.Tim[TIM_100MS].Ticks = TimObj.Tim[TIM_100MS].Period;
	RFTxState = 1;
	
	Us2400Tx(destAddr, data, dataLen);
	
	while (RFTxState != 0) {
		if ((TimObj.TimeoutFlag & TIMOUT_FLAG_100MS) == TIMOUT_FLAG_100MS) {
			if (Us2400ReadShortReg(0x31) == 0x09) {
	      //Us2400Rx(Data, &DataLen, &RFLqi, &RFRssi);
				if (dataLen != 0){
					//Packet_Receive();
					dataLen = 0;
				}
				RFRxState = 0;
	      Us2400WriteShortReg(0x0D, Us2400ReadShortReg(0x0D) | 0x01);
			}
			TimObj.TimeoutFlag ^= TIMOUT_FLAG_100MS;
      RFTxState = 0;
		}
	}
	RFTxOccupied = 0;
}

uint8_t RF_Rx(uint8_t* RxData, uint8_t* Data_Length, uint8_t* RSSI){
	
	int _isReceived =0;
	int _checkFlag=0;
	
	RFRxOccupied = 1;
	// ============== Check RX FIFO Full/Overflow ================= //
	if (Us2400ReadShortReg(0x30) == 0x90){
		Us2400Rx(Data, &DataLen_temp, &RFLqi, &RFRssi);
		if (DataLen_temp != 0){
			DataLen = DataLen_temp;
			DataLen_temp = 0;
			
			memcpy(RxData, Data, DataLen);
			*Data_Length = DataLen;
			*RSSI = RFRssi;
			
			_checkFlag = headerCheck_AutoNet();			
			if(!_checkFlag) {							// Not For AutoNet
				_isReceived = 1;
				Us2400WriteShortReg(0x0D, Us2400ReadShortReg(0x0D) | 0x01);	
			}
		}
	}
	// ============== Packet Receive ================= //
	if (RFRxState == 1 || Us2400ReadShortReg(0x30) == 0x80){
		Us2400Rx(Data, &DataLen_temp, &RFLqi, &RFRssi);
		if (DataLen_temp != 0){
			DataLen = DataLen_temp;
			DataLen_temp = 0;			
			
			memcpy(RxData, Data, DataLen);
			*Data_Length = DataLen;
			*RSSI = RFRssi;
			
			_checkFlag = headerCheck_AutoNet();
			if(!_checkFlag) {    					// Not For AutoNet
				_isReceived = 1;
				RFRxState = 0;
			}
		}
	}
	RFRxOccupied = 0;
	return _isReceived;
}

uint8_t RF_RX_AUTONET(){
	
	int _isReceived =0;
	int _checkFlag=0;
	
	RFRxOccupied = 1;
	// ============== Check RX FIFO Full/Overflow ================= //
	if (Us2400ReadShortReg(0x30) == 0x90){
		Us2400Rx(Data, &DataLen_temp, &RFLqi, &RFRssi);
		if (DataLen_temp != 0){
			DataLen = DataLen_temp;
			DataLen_temp = 0;
			
			memcpy(pRxData, Data, DataLen);
			RSSI_BC = RFRssi;
			
			_checkFlag = headerCheck_AutoNet();
			if(_checkFlag) {					// For AutoNet
				_isReceived = 1;
				Us2400WriteShortReg(0x0D, Us2400ReadShortReg(0x0D) | 0x01);	
			}
		}
	}
	// ============== Packet Receive ================= //
	if (RFRxState == 1 || Us2400ReadShortReg(0x30) == 0x80){
		Us2400Rx(Data, &DataLen_temp, &RFLqi, &RFRssi);
		if (DataLen_temp != 0){
			DataLen = DataLen_temp;
			DataLen_temp = 0;			
			
			memcpy(pRxData, Data, DataLen);
			RSSI_BC = RFRssi;
			
			_checkFlag = headerCheck_AutoNet();	
			if(_checkFlag) {					// For AutoNet
				_isReceived = 1;
				RFRxState = 0;
			}
		}
	}
	RFRxOccupied = 0;
	return _isReceived;
}

int headerCheck_AutoNet(){
	
		if(Data[FRAME_BYTE_HEADER + RX_OFFSET] == 0xFF){
			return 1;
		}
		return 0;	
}


/******************* (C) COPYRIGHT 2015 NXG LAB *****END OF FILE****/
