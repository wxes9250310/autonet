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
#include "autonetAPIs.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define NUMOFBUFFER 		128
#define NUMOFSENSOR 		3
#define NUMOFDEVICE 		6
#define TX_PAUSE 				1000
#define RX_OFFSET  12

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint8_t ID;									
extern uint8_t TYPE;
extern uint8_t CommanderID;

extern uint8_t pTxData[];
extern uint8_t pRxData[];

extern uint8_t RFTxState;
extern uint8_t RFRxState;
extern uint8_t RFSleepState;
extern TimObjTypeDef_s TimObj;

extern uint8_t Data[];
extern uint8_t DataLen;
extern uint8_t RFRssi;
extern uint8_t RFLqi;
extern uint8_t rssi;
extern uint8_t lightState;

extern unsigned char cLatitude;
extern unsigned char cLongitude;
extern unsigned char Lat_deg; 
extern unsigned char Lat_min;
extern unsigned char Lat_sec;
extern unsigned char Lat_dir;
extern unsigned char Long_deg;
extern unsigned char Long_min;
extern unsigned char Long_sec;
extern unsigned char Long_dir;
extern unsigned char RealspeedH;     // speed first byte
extern unsigned char RealspeedL;		 // speed second byte
extern char FrontID, RearID;
extern int framelength;
extern int flat_headingH;
extern int flat_headingL;
extern uint8_t type;

Table table;
Device myAttribute;
unsigned char DataLen_temp;
int Group_count = 0;
int Group_flag = 0;
int num;
int i,j,k,gps_m;
int CheckTime;
int ConfirmGroupTime = 10;


enum{
	false,
	true,
};

enum{
	Message_Control,
	Message_Type,
	Message_BroadcastType,
	Message_Light,
};

enum{
		Type_Controller = 0x00,			// delete? 
		Type_Light = 0x01,
		Type_Switch = 0x02,
};

uint8_t  Group[NumOfDeviceInTable];
uint16_t Condition[ATTRIBUTE_NUM] = {0x10,0xFF,0xFF};
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
}

uint8_t RF_Rx(uint8_t* RxData, uint8_t* Data_Length, uint8_t* RSSI){
	
	  int _rx_check_flag =0;
	  int _header_check_flag=0;
	
		// ============== Check RX FIFO Full/Overflow ================= //
		if (Us2400ReadShortReg(0x30) == 0x90){
			Us2400Rx(Data, &DataLen_temp, &RFLqi, &RFRssi);
			if (DataLen_temp != 0){
				DataLen = DataLen_temp;
				DataLen_temp = 0;
				
				memcpy(pRxData, Data, DataLen);
				memcpy(RxData, pRxData, DataLen);
				*Data_Length = DataLen;
				*RSSI = RFRssi;
				
				_header_check_flag = autonet_header_check();			
				if(!_header_check_flag) {							// Not For AutoNet
					_rx_check_flag = 1;
					Us2400WriteShortReg(0x0D, Us2400ReadShortReg(0x0D) | 0x01);	
				}
			}
		}
		// ============== Packet Receive ================= //
		if (RFRxState == 1 || Us2400ReadShortReg(0x30) == 0x80){
			Us2400Rx(Data, &DataLen_temp, &RFLqi, &rssi);
			if (DataLen_temp != 0){
				DataLen = DataLen_temp;
				DataLen_temp = 0;			
				
				memcpy(pRxData, Data, DataLen);
				memcpy(RxData, pRxData, DataLen);
				*Data_Length = DataLen;
				*RSSI = RFRssi;
				
				_header_check_flag = autonet_header_check();
				if(!_header_check_flag) {    					// Not For AutoNet
					_rx_check_flag = 1;
					RFRxState = 0;
				}
			}
		}
		return _rx_check_flag;
}

int RF_RX_AUTONET(){
	
	  int _rx_check_flag =0;
	  int _header_check_flag=0;

		// ============== Check RX FIFO Full/Overflow ================= //
		if (Us2400ReadShortReg(0x30) == 0x90){
			Us2400Rx(Data, &DataLen_temp, &RFLqi, &RFRssi);
			if (DataLen_temp != 0){
				DataLen = DataLen_temp;
				DataLen_temp = 0;
				
				memcpy(pRxData, Data, DataLen);
				
				_header_check_flag = autonet_header_check();
				if(_header_check_flag) {					// For AutoNet
					_rx_check_flag = 1;
					Us2400WriteShortReg(0x0D, Us2400ReadShortReg(0x0D) | 0x01);	
				}
			}
		}
		// ============== Packet Receive ================= //
		if (RFRxState == 1 || Us2400ReadShortReg(0x30) == 0x80){
			Us2400Rx(Data, &DataLen_temp, &RFLqi, &rssi);
			if (DataLen_temp != 0){
				DataLen = DataLen_temp;
				DataLen_temp = 0;			
				
				memcpy(pRxData, Data, DataLen);
				
				_header_check_flag = autonet_header_check();	
				if(_header_check_flag) {					// For AutoNet
					_rx_check_flag = 1;
					RFRxState = 0;
				}
			}
		}
		return _rx_check_flag;
}


void broadcastSend(void)
{
	  uint8_t i;
		
		pTxData[FRAME_BYTE_HEADER] 			= 0xFF;
		pTxData[FRAME_BYTE_SRCADDR] 		= ID;
	  pTxData[FRAME_BYTE_TYPE]				= type;
		pTxData[FRAME_BYTE_NUMOFSENSOR] = NUMOFSENSOR;
	  for(i=0;i<ATTRIBUTE_NUM;i++){
		  pTxData[FRAME_BYTE_ATTRIBUTE + 2*i] = myAttribute.attribute[i] >> 8;
			pTxData[FRAME_BYTE_ATTRIBUTE + 2*i + 1] = myAttribute.attribute[i];
		}
		framelength = FRAME_BYTE_ATTRIBUTE + 2*ATTRIBUTE_NUM;
		RF_Tx(0xFFFF, pTxData, framelength);
}

void broadcast(void)
{
	  uint8_t i;
		
		pTxData[0] = Message_BroadcastType;
		pTxData[1] = TYPE;
	  pTxData[2] = ID;
	
	// TODO: to send out the status of the device

		for(i=3;i<128;i++){
			pTxData[i]=0x00;
		}	
		RF_Tx(0xFFFF, pTxData, 10);
}

void packet_receive(void)
{
	uint8_t index=0xFF;
	uint8_t newIndex=0xFF;
	uint8_t type;
	uint16_t addr;
	memcpy(pRxData, Data, DataLen);
	type = pRxData[FRAME_BYTE_TYPE+RX_OFFSET];
	addr = pRxData[FRAME_BYTE_SRCADDR+RX_OFFSET];
	index = ScanTableByAddress(addr);
	if(index == 0xFF){														// no such address in the table
		newIndex = ScanTableByAddress(0xFF);
		if(newIndex != 0xFF){
			setTable(newIndex,addr,type);
		}
	}
	else{
		setTable(index,addr,type);
	}
}

uint8_t ScanTableByAddress(uint8_t scan_value){
	for(i=0;i<NumOfDeviceInTable;i++){
		if(scan_value == table.device[i].address){
			return i;
		}
	}
	return 0xFF;
}

void setTable(uint8_t n,uint16_t device_addr,uint8_t device_type){
	table.device[n].type = device_type;
	table.device[n].address = device_addr;
  for(i=0;i<ATTRIBUTE_NUM;i++)
		table.device[n].attribute[i] = pRxData[2*i+5+RX_OFFSET] | (pRxData[2*i+4+RX_OFFSET]<<8);
}

int autonet_header_check(){
	
	/*
		if(pRxData[FRAME_BYTE_HEADER + RX_OFFSET] >= 0x00 || pRxData[FRAME_BYTE_HEADER + RX_OFFSET] <= 0x04) 
				return true;
		else
				return false;
		*/
	
	
		//memcpy(pRxData, Data, DataLen);
		if(pRxData[FRAME_BYTE_HEADER + RX_OFFSET] == 0xFF){
			return 1;
		}
		return 0;
	
}

void getSrcAddr(uint8_t* data_out, uint8_t* data_in){

		memset(data_out,0x00,sizeof(data_out));
  	data_out[0] = data_in[10];
	  data_out[1] = data_in[11];
}

void getDestAddr(uint8_t* data_out, uint8_t* data_in){

		memset(data_out,0x00,sizeof(data_out));
  	data_out[0] = data_in[6];
	  data_out[1] = data_in[7];
}

void getSrcPanID(uint8_t* data_out, uint8_t* data_in){

		memset(data_out,0x00,sizeof(data_out));
  	data_out[0] = data_in[8];
	  data_out[1] = data_in[9];
}

void getDestPanID(uint8_t* data_out, uint8_t* data_in){

		memset(data_out,0x00,sizeof(data_out));
  	data_out[0] = data_in[4];
	  data_out[1] = data_in[5];
}

void getSeqNum(uint8_t* data_out, uint8_t* data_in){

		memset(data_out,0x00,sizeof(data_out));
  	data_out[0] = data_in[3];
}

void getFrameControl(uint8_t* data_out, uint8_t* data_in){

		memset(data_out,0x00,sizeof(data_out));
  	data_out[0] = data_in[1];
	  data_out[1] = data_in[2];
}

void getPayload(uint8_t* data_out, uint8_t* data_in, uint8_t Data_Length){

	  int i;
		
	  memset(data_out,0x00,sizeof(data_out));
		for(i=0 ; i<Data_Length ; i++)
			data_out[i] = data_in[i+RX_OFFSET];
}

void getPayloadLength(uint8_t* data_out, uint8_t* data_in){

		memset(data_out,0x00,sizeof(data_out));
  	data_out[0] = data_in[0];
}

uint8_t Group_Diff(uint16_t* ID,uint8_t type, uint16_t center, uint16_t difference){
	  int NumofDevice = 0;
		for(i=0;i<10;i++)
			ID[i] = 0xFFFF;
	  for(i=0;i<NUMOFSENSOR;i++)
			Condition[i] = 0xFF;
		// ------------ set Condition --------------//
		Condition[type] = difference;
		// ----- AutoNet algorithm ----- //
		Group_Configuration();
	
		for(i=0;i<NUMOFDEVICE;i++){
			Group_flag = 1;
			if(Condition[type] != 0xFF && abs(Data_table[i][type]-center) > Condition[type])  		// Not complete yet
				Group_flag = 0;																							                        // buffers is needed
			//TODO: grouping results will be accumulated to be a threshold to change the original group
			if(Group_flag == 1)
				Group[i]++;
		}
		for(i=0;i<NUMOFDEVICE;i++)
			if(Group[i] > 0){
				ID[NumofDevice+1] = i;
				NumofDevice++;
			}
		ID[0] = NumofDevice;
		return NumofDevice;
}

void Group_Configuration(){
		for(i =0;i<NUMOFDEVICE;i++){
			Group[i] = 0x00;
		}
}

void lighting(uint8_t State){
    switch (State){
        case 0:
						GPIOB->BRR = GPIO_Pin_13;
            break;
        case 1:
						GPIOB->BSRR = GPIO_Pin_13;
						break;
        default:
						GPIOB->BRR = GPIO_Pin_13;
						break;
		}
}

/******************* (C) COPYRIGHT 2015 NXG LAB *****END OF FILE****/
