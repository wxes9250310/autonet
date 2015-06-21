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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define RX_BUFFER_OFFSET 12

/* Private function ----------------------------------------------------------*/
void app_light_direction();
void app_control_light();

// application - Light direction
enum{
	Message_Control,
	Message_Type,
	Message_BroadcastType,
	Message_Light,
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
}

void app_light_direction(){
	
	// local variables definition
	uint16_t radio_freq;
  uint16_t radio_panID;
  uint16_t srcAddr;
	uint8_t TxData[256];  
	uint8_t RxData[256];
	uint8_t Rx_DataLen;
	uint8_t Rx_Payload[256];
	uint16_t addr_array[10];
	uint8_t Device_Type;
	uint8_t i;
	uint8_t Lighting_Flag=0;
	uint8_t RSSI;
	int heading;
	uint8_t heading_diff=20;
	uint8_t Match_Devices_Num;
	
	//
	Device_Type = Type_Controller;
	srcAddr = 0x0000;
	
	radio_freq = 2475;
  radio_panID = 0x00AA;
	// TODO: change to Initial();
  Initial(srcAddr, Device_Type, radio_freq, radio_panID);
	
  // setting the period of broadcasting information of AutoNet	
	Timer_Beacon(200);
  // every 500ms to pull up the time flag
	setTimer(1, 500, UNIT_MS);

	while(1){
		RF_beacon();  // broadcast beacon information
		if(Device_Type == Type_Controller){
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
		else if(Device_Type == Type_Light){
			if(RF_Rx(RxData,&Rx_DataLen,&RSSI)){
			  Lighting_Flag = 0;
				getPayload(Rx_Payload,RxData,Rx_DataLen);
				for( i=0 ; i<Rx_Payload[1]; i++ ){
					if((Rx_Payload[2*i+2] | Rx_Payload[2*i+3] << 8) == srcAddr){
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

void app_control_light(){
	
	  // definition of parameters
	  uint16_t srcAddr = 0x0005;
		uint8_t type = Type_Light;
		uint16_t radio_freq = 2475;
		uint16_t radio_panID = 0x00AA;
	
	  uint8_t openLightFlag;
	  uint8_t state;
	  uint8_t detect;  
	  uint8_t RSSI;
	
	  uint8_t TxData[256];  
		uint8_t RxData[256];
    uint8_t Tx_DataLen;
		uint8_t Rx_DataLen;
		uint8_t Rx_Payload[256];
		uint16_t neighbor[10];
	  uint8_t neighborNum = 0;
		uint8_t addFlag=0;
	  uint8_t outFlag =0;
	  uint16_t RSSI_THRESHOLD = 200;		// not quite sure the range
	
		uint8_t i,k;
		uint16_t DeviceAddr;
	
		// start
		Initial(srcAddr, type, radio_freq, radio_panID);
		
		setTimer(1, 1000, UNIT_MS);
		setTimer(2, 500, UNIT_MS);

		while(1){ 	
			
			// automatically broadcast for some defined devices
			if(type == Type_Light){
					RF_beacon();  // broadcast beacon information	  
				
					if(checkTimer(1)){										// check whether the device should open the light or not
						RF_Rx(RxData,&Rx_DataLen,&RSSI);
						
						if(RxData[RX_BUFFER_OFFSET + 0] == Message_Light){	
							for(i=1; i<= Rx_DataLen; i++){
								if(RxData[RX_BUFFER_OFFSET + i]==srcAddr){
									// bug
									blink();
								}
							}	
						}
					}
			}
			
			else if(type == Type_Controller){
				if(checkTimer(2)){										// check whether the device should open the light or not
					RF_Rx(RxData,&Rx_DataLen,&RSSI);
					
					if(RxData[RX_BUFFER_OFFSET + 0] == Message_BroadcastType && RxData[RX_BUFFER_OFFSET + 1] == Type_Light){
							DeviceAddr = RxData[RX_BUFFER_OFFSET + 2];
							if(RSSI >= RSSI_THRESHOLD){					// near enough
								addFlag = 0;
								for(k=0; k<=neighborNum; k++){			// check list
									if(neighbor[k] != DeviceAddr){
										addFlag = 1;									// renew the list of neighbors									
									}
									else{
										addFlag = 0;
										break;	
									}
								}
								if(addFlag ==1){								// shift left by one byte
										neighbor[neighborNum++] = DeviceAddr;
									}
							}
							else{																// not neighbors
								for(k=0; k<=neighborNum; k++){			// check list
									if(neighbor[k] == DeviceAddr){
										outFlag = 1;									// renew the list of neighbors
										neighborNum --;
									}
									if(outFlag ==1){								// shift left by one byte
										neighbor[k] = neighbor[k+1];
									}
								}
								outFlag=0;												// reset the flag
							}
						}
					if(neighborNum != 0){										// need to send lighting message
						openLightFlag = 1;
					}
					else {
						openLightFlag = 0;
					}
					
					if(openLightFlag){	 		// need to send lighting messages
						blink();
						TxData[0]= Message_Light;
						for(i =1; i<= neighborNum; i++){
							TxData[i] = neighbor[i-1];
						}
						RF_Tx(0xFFFF, TxData, neighborNum);
					}
					PIN_OFF(1);
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
