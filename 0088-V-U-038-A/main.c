#include <string.h>
#include "main.h"
#include "stm32f0xx_it.h"
#include "us2400APIs.h"
#include "st.h"
#include "TxRx.h" 

#include "autonetAPIs.h"
#include "mpu6050APIs.h"
#include "ak8975APIs.h"
#include "lea6sAPIs.h"
#include "tmp75APIs.h"
#include "mcp2120APIs.h"
#include "bh1750fviAPIs.h"
#include "mag3110APIs.h"


#include "app_local_grouping.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MAC_HEADER_LENGTH 12
#define Message_BroadcastType 0xFF
#define Message_Light 0x03

/* Private variable ----------------------------------------------------------*/


/* Private function ----------------------------------------------------------*/
void app_light_direction(void);
void app_control_light(void);

// application - Light direction
enum{
		Message_Control,
		Message_Type,
//		Message_BroadcastType = 0xFF,
//		Message_Light = 0x03,
};

// application - Control light
enum{
		Type_Controller = 0x00,			// delete?
		Type_Light = 0x01,
		Type_Switch = 0x02,
};

int main(void)
{
		//app_light_direction();
		app_control_light();
	  app_local_grouping();
}

void app_light_direction(){
	
	  uint8_t Type;
		uint16_t Addr;
		uint16_t radio_freq;
		uint16_t radio_panID;
		// local variables definition
		uint8_t TxData[256];  
		uint8_t RxData[256];
		uint8_t Rx_DataLen;
		uint8_t Rx_Payload[256];
		uint16_t addr_array[10];
		uint8_t i;
		uint8_t Lighting_Flag=0;
		uint8_t RSSI;
		int heading;
		uint8_t heading_diff=20;
		uint8_t Match_Devices_Num;
		
		// Initialization
		Addr = 0x0000;
		Type = Type_Controller;
		radio_freq = 2475;
		radio_panID = 0x00AA;
		Initial(Addr, Type, radio_freq, radio_panID);
		
		// setting the period of broadcasting information of AutoNet	
		Timer_Beacon(200);
		// every 500ms to pull up the time flag
		setTimer(1, 500, UNIT_MS);

		while(1){
			beacon();  // broadcast beacon information
			if(Type == Type_Controller){
				if(checkTimer(1) && get_direction(&heading)){
					Match_Devices_Num = Group_Diff(addr_array,Direction,heading,heading_diff);
					TxData[0] = Message_Control;
					TxData[1] = Match_Devices_Num;
					for( i=0 ; i<Match_Devices_Num ; i++ ){
						TxData[2*i+2] = addr_array[i+1];
						TxData[2*i+3] = addr_array[i+1] >> 8;
					}
					RF_Tx(0xFFFF,TxData,Match_Devices_Num*2+2);
				}
			}
			else if(Type == Type_Light){
				if(RF_Rx(RxData,&Rx_DataLen,&RSSI)){
					Lighting_Flag = 0;
					getPayload(Rx_Payload,RxData,Rx_DataLen);
					for( i=0 ; i<Rx_Payload[1]; i++ ){
						if((Rx_Payload[2*i+2] | Rx_Payload[2*i+3] << 8) == Addr){
							Lighting_Flag = 1;
							break;
						}
					}
					if(Lighting_Flag == 1)
						PIN_ON(1);
					else
						PIN_OFF(1);
				}
			}
		}
		
}


/*******************************************************************************
* Application Name  : Automatically turns on near lights
* Description    		: When a controller approaches lights, they will turn on  
*                   : their lights automatically
* Author            : Ed Kung
*******************************************************************************/
void app_control_light(){ 
		
	  uint8_t Type;
		uint16_t Addr;
		uint16_t radio_freq;
		uint16_t radio_panID;
	// Task:
	// Use IR to beacon
	
	  int times =0;
		uint8_t i,k;
	  uint8_t state;
	  uint8_t detect;  
	  uint8_t RSSI;
	  uint8_t TxData[256];  
		uint8_t RxData[256];
    uint8_t Tx_DataLen;
		uint8_t Rx_DataLen;
		uint8_t Rx_Payload[256];
	  uint8_t neighborNum = 0;
		uint8_t addFlag=0;							// record whether to renew the lighting table in the controller
	  uint8_t outFlag =0;
		uint16_t neighbor[10];
	  uint16_t RSSI_THRESHOLD = 200;	
		uint8_t r_DeviceAddr;						// the address of received devices 
		uint8_t msgLightFlag;
	
		// Initialization
		Addr = 0x0005;
		Type = Type_Light;
		//Type = Type_Controller;
		radio_freq = 2475;
		radio_panID = 0x00AA;
		Initial(Addr, Type, radio_freq, radio_panID);
		
		// set timers
		setTimer(1, 500, UNIT_MS);
		setTimer(2, 500, UNIT_MS);

		while(1){ 	
			if(Type == Type_Light){											// Light
					beacon();  															// broadcast beacon information	  
				  Delay(10);
				  // check whether the device should open the light or not
					if(checkTimer(1)){											// check every 1000 ms							
						times++;
						if(times==4){												  // every 2000 ms reset the light
							times=0;
							PIN_OFF(1);
						}
						if(RF_Rx(RxData,&Rx_DataLen,&RSSI)){	// returns 1 if it has received something
							if(RxData[MAC_HEADER_LENGTH + 0] == Message_Light){		// lighting message
								// check the address of the devices whether exists or not
								for(i=1; i<= (Rx_DataLen-MAC_HEADER_LENGTH); i++){											
									if(RxData[MAC_HEADER_LENGTH + i]==Addr){
										// bug
										PIN_ON(1);
									}
								}	
							}
						}
					}
			}
			
			else if(Type == Type_Controller){
				// check whether the device should open the light or not
				if(checkTimer(2)){										
					if(RF_Rx(RxData,&Rx_DataLen,&RSSI)){		// check whether a received frame exists
						if(RxData[MAC_HEADER_LENGTH + 0] == 0xFF 
												&& RxData[MAC_HEADER_LENGTH + 2] == Type_Light){
							r_DeviceAddr = RxData[MAC_HEADER_LENGTH + 1];
						
							if(RSSI >= RSSI_THRESHOLD){						// near enough
								addFlag = 0;												
								for(k=0; k<=neighborNum; k++){			// check neighbors list
									if(neighbor[k] != r_DeviceAddr){	// the devices does not exist in the list
										addFlag = 1;										// need to renew the list of neighbors									
									}
									else{															// the devices already exists in the list
										addFlag = 0;
										break;	
									}
								}
								
								if(addFlag ==1){										// add the device into the list
										neighbor[neighborNum++] = r_DeviceAddr;
								}
							}
							else{																	// not near enough 
								for(k=0; k<=neighborNum; k++){			// check neighbors list
									if(neighbor[k] == r_DeviceAddr){
										outFlag = 1;										// renew the list of neighbors
										neighborNum --;
									}
									if(outFlag ==1){									// delete it from the list and 
										neighbor[k] = neighbor[k+1];		// shift remaining addresses left by one byte
									}
								}
								outFlag=0;													// reset the flag
							}
						}
					}
					
					if(neighborNum != 0)										// need to send lighting message
						msgLightFlag = 1;
					else 
						msgLightFlag = 0;
					
					if(msgLightFlag){	 										 // need to send lighting messages

						for(i =0; i<= 10; i++){
							TxData[i] = 0;
						}
						// contruction of lighting message frame
						TxData[0]= Message_Light;		
						for(i =1; i<= neighborNum; i++){
							TxData[i] = neighbor[i-1];
						}
						// broadcast the packet
						RF_Tx(0xFFFF, TxData, 2);
					}
				}
			}
	}
}

/*
void ControlLight(){
	
		uint16_t radio_freq;
		uint16_t radio_panID;
		uint16_t srcAddr;
		uint8_t type;
	  uint8_t state;
	  uint8_t detect;
	  uint8_t* IR_Buffer_Length1;
	  uint8_t* IR_Buffer_Length2;
	 
	  unsigned char* IR_BufferTx[64] = {0x0};
		unsigned char* IR_BufferRx[64] = {0x0};
		unsigned char* IR_BufferRx2[64] = {0x0};
		unsigned short Lux =0;
	
		radio_freq = 2475;
		radio_panID = 0x00AA;
		Initial(srcAddr, type, radio_freq, radio_panID);
		
		//Timer_Beacon(100);	
		setTimer(1, 100, UNIT_MS);
		setTimer(2, 200, UNIT_MS);

		while(1){ 	
			
				if(checkTimer(1)){
					if(type == 0x01){		// observer
						//Mcp2120Proc((unsigned char *)IR_BufferRx, 1);
						Delay(10);
						Mcp2120Proc((unsigned char *)IR_BufferRx2, IR_Buffer_Length2, 2);
						Delay(10);
						state=1;
					}
					else if(type == 0x02){		// light
						
						*IR_BufferTx[0] = type;
						*IR_BufferTx[1] = srcAddr;
						
						Mcp2120Tx((unsigned char *)IR_BufferTx, 2 , 1);
						Delay(10);
						Mcp2120Tx((unsigned char *)IR_BufferTx, 2 , 2);
						Delay(10);
						state=2;
					}
					else{
						// do nothing
					}
				}
				
				if(*IR_BufferRx[0] == 0x02){
						detect = 1;
				}
				
				if(*IR_BufferRx2[0] == 0x02){
						detect = 2;
				}
				
				if(checkTimer(2)){
						Bh1750fviReadLx(0x46, &Lux);
						Delay(10);
				}
		}
}
*/
