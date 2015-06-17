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
/* Private function ----------------------------------------------------------*/
void app_light_direction();
void app_control_light();

// application - Light direction
enum{
	Type_Controller,
	Type_Light,
};

enum{
	Message_Control,
	Message_Type,
};

// application - Control light
enum{
	Light = 0x0001,
	Switch = 0x0002,
};

enum{
	Message_Light,
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
  Initialization(radio_freq, radio_panID, srcAddr);
	
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
	
	  // still working on
	
		uint16_t radio_freq;
		uint16_t radio_panID;
		uint16_t srcAddr;
		uint8_t Device_Type;
	
	  uint8_t openLightFlag;
	  uint8_t state;
	  uint8_t detect;  
	  uint8_t RSSI;
	
	  uint8_t TxData[256];  
		uint8_t RxData[256];
		uint8_t Rx_DataLen;
		uint8_t Rx_Payload[256];
		uint16_t addr_array[10];
	
	  Device_Type = Light;
		srcAddr = 0x0000;
	  Autonet_Config(srcAddr, 0x0001);
	
		radio_freq = 2475;
		radio_panID = 0x00AA;
		Initialization(radio_freq, radio_panID, srcAddr);
		
		setTimer(1, 200, UNIT_MS);
		setTimer(2, 1000, UNIT_MS);

		while(1){ 	
			
			// automatically broadcast for some defined devices
			if(Device_Type == Type_Light){
					RF_beacon();  // broadcast beacon information
			}
			else{
			}
			
			if(checkTimer(1)){
			  RF_Rx(RxData,&Rx_DataLen,&RSSI);
				if(RxData[0] == Message_Light){		// command 
					if(RxData[1] == Type_Light){		// check whether the device should open the light or not
						openLightFlag = RxData[2];
						if(openLightFlag == 1){
							PIN_ON(1);
						}
					}
					else if(Device_Type == Type_Controller){
						
					}
			  }
			}
			
			if(checkTimer(2)){
				PIN_OFF(1);
			}
	}
}


void ControlLight(){
	
		uint16_t radio_freq;
		uint16_t radio_panID;
		uint16_t srcAddr;
		uint8_t Device_Type;
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
		Initialization(radio_freq, radio_panID, srcAddr);
		
		//Timer_Beacon(100);	
		setTimer(1, 100, UNIT_MS);
		setTimer(2, 200, UNIT_MS);

		while(1){ 	
			
				if(checkTimer(1)){
					if(Device_Type == 0x01){		// observer
						//Mcp2120Proc((unsigned char *)IR_BufferRx, 1);
						Delay(10);
						Mcp2120Proc((unsigned char *)IR_BufferRx2, IR_Buffer_Length2, 2);
						Delay(10);
						state=1;
					}
					else if(Device_Type == 0x02){		// light
						
						*IR_BufferTx[0] = Device_Type;
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
