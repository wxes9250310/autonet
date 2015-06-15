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
#define NUMOFBUFFER 		128
#define NUMOFSENSOR 		3
#define NUMOFDEVICE 		6
#define COMMANDHEADER		0xFF
#define BROADCASTHEADER 0xFE
#define LIGHTSENDHEADER 0xFD
#define TX_PAUSE 				1000
#define RX_OFFSET  12

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint8_t ID;									// C represents cars
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

uint8_t  Group[NUMOFDEVICE];
uint16_t Condition[SIZE_OF_DATA_TABLE] = {0x10,0xFF,0xFF};
//uint16_t Data_table[NUMOFDEVICE][NUMOFSENSOR];
uint16_t Data_table[NUMOFDEVICE][SIZE_OF_DATA_TABLE];
uint16_t My_Data_table[SIZE_OF_DATA_TABLE];

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

void broadcastSend(void)
{
	  uint8_t i;
		
		pTxData[FRAME_BYTE_HEADER] 			= 0xFF;
		pTxData[FRAME_BYTE_TYPE] 	 			= BROADCASTHEADER;
		pTxData[FRAME_BYTE_SRC_ID] 			= ID;
		pTxData[FRAME_BYTE_NUMOFSENSOR] = NUMOFSENSOR;
	
		pTxData[FRAME_BYTE_STYPE1] 			= TYPE_HEADING; 		 // type1: eCompass
		pTxData[FRAME_BYTE_SVALUE11] 		= flat_headingH;  		
		pTxData[FRAME_BYTE_SVALUE12] 		= flat_headingL; 	
	
		pTxData[FRAME_BYTE_STYPE2] 			= TYPE_SPEED; 			 // type2: velocity
		pTxData[FRAME_BYTE_SVALUE21]		= RealspeedH; 
		pTxData[FRAME_BYTE_SVALUE22] 		= RealspeedL;	
	
		pTxData[FRAME_BYTE_STYPE3] 			= 0x03; 						 // type3: GPS degree
		pTxData[FRAME_BYTE_SVALUE31]		= Lat_deg;
		pTxData[FRAME_BYTE_SVALUE32] 		= Long_deg;	 
	
		pTxData[FRAME_BYTE_STYPE4] 			= 0x04; 						  // type4: GPS minute
		pTxData[FRAME_BYTE_SVALUE41]		= Lat_min;  
		pTxData[FRAME_BYTE_SVALUE42] 		= Long_min;	
	
		pTxData[FRAME_BYTE_STYPE5] 			= 0x05; 					 	 // type5: GPS second
		pTxData[FRAME_BYTE_SVALUE51]		= Lat_sec;
		pTxData[FRAME_BYTE_SVALUE52]		= Long_sec;    	

		pTxData[FRAME_BYTE_STYPE6] 			= 0x06; 						 // type6: GPS direction
		pTxData[FRAME_BYTE_SVALUE61]		= Lat_dir;
		pTxData[FRAME_BYTE_SVALUE62]		= Long_dir;
		
		pTxData[FRAME_BYTE_STYPE7] 			= 0x07; 						 // type6: GPS direction
		pTxData[FRAME_BYTE_SVALUE71]		= Lat_dir;
		pTxData[FRAME_BYTE_SVALUE72]		= Long_dir;

		for(i=framelength;i<128;i++){
			pTxData[i]=0x00;
		}
		
//		pTxData[framelength] = CheckSum(pTxData, framelength);
		
		RF_Tx(0xFFFF, pTxData, framelength);
}

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

uint8_t RF_Rx(uint8_t* RxData, uint8_t* Data_Length){
	
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


void packet_receive(void)
{
		//basicRfReceive(pRxData, MAX_RECV_BUF_LEN, &rssi);
	  memcpy(pRxData, Data, DataLen);
		//		if(pRxData[FRAME_BYTE_HEADER + RX_OFFSET] == 0xFF){
//			if(pRxData[FRAME_BYTE_TYPE + RX_OFFSET] == BROADCASTHEADER){
				for(i=0;i<pRxData[FRAME_BYTE_NUMOFSENSOR+RX_OFFSET];i++){
					if(i <= TYPE_SPEED){																		  // TYPE_SPEED = 2
						Data_table[(int)pRxData[FRAME_BYTE_SRC_ID+RX_OFFSET]][pRxData[3*i+4+RX_OFFSET]] = pRxData[3*i+6+RX_OFFSET] | (pRxData[3*i+5+RX_OFFSET]<<8);
					}
					else if(i==2)																							// STORE GPS values, special case
					{
						Data_table[(int)pRxData[FRAME_BYTE_SRC_ID+RX_OFFSET]][2] = pRxData[12];    // degree of Latitude 
						Data_table[(int)pRxData[FRAME_BYTE_SRC_ID+RX_OFFSET]][3] = pRxData[15];		 // minute of Latitude
						Data_table[(int)pRxData[FRAME_BYTE_SRC_ID+RX_OFFSET]][4] = pRxData[18];		 // second of Latitude
						Data_table[(int)pRxData[FRAME_BYTE_SRC_ID+RX_OFFSET]][5] = pRxData[21];    // direction of Latitude
						Data_table[(int)pRxData[FRAME_BYTE_SRC_ID+RX_OFFSET]][6] = pRxData[13];		 // degree of Longitude 
						Data_table[(int)pRxData[FRAME_BYTE_SRC_ID+RX_OFFSET]][7] = pRxData[16];		 // minute of Longitude
						Data_table[(int)pRxData[FRAME_BYTE_SRC_ID+RX_OFFSET]][8] = pRxData[19];	   // second of Longitude
						Data_table[(int)pRxData[FRAME_BYTE_SRC_ID+RX_OFFSET]][9] = pRxData[22];    // direction of Longitude
						break;
					}
				}
//			}
// 	 }
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
			if(pRxData[FRAME_BYTE_TYPE + RX_OFFSET] == BROADCASTHEADER){
				return 1;
			}
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

int headerCheck_AutoNet(uint8_t* data){
		
		// AutoNet reserves 0x00~0x04 as header
	  if(data[0+RX_OFFSET] >= 0x00 && data[0+RX_OFFSET] <= 0x04)
		    return true;
		else 
				return false;
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

void My_Table_Configuration(){
		for(i =0;i<NUMOFDEVICE;i++){
			My_Data_table[i] = 0x00;
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
