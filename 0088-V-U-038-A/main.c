#include <string.h>
#include <stdlib.h>
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

#define MAC_HEADER_LENGTH 12
#define Message_BroadcastType 0xFF
#define Message_Light 0x03
//
int main(void)
{
	/*
	uint8_t Type;
	uint16_t Addr;
	uint16_t radio_freq;
	uint16_t radio_panID;
	uint8_t State=0;
	uint8_t MyWeight = 0xFF;
	uint8_t MaxWeight = 0;
	Addr = 0x0001;
	Type = Type_Light;
	radio_freq = 2475;
	radio_panID = 0x00AA;

	Initial(Addr, Type, radio_freq, radio_panID);	
	setTimer(1,500,UNIT_MS);
	
	while(1){
		beacon();
		ChangeLight(MyWeight);
		if(checkTimer(1))
			WeightBroadCast(MyWeight);
		switch(State){
			case 0:
				MyWeight = rand()%3;
			  State = 1;
			  setTimer(2,2000,UNIT_MS);
				break;
			case 1:
				StateOne(&MyWeight);
			  if(checkTimer(2)){
					State = 2;
					setTimer(2,0,UNIT_MS);
				}
				break;
			case 2:
				StateTwo(&MyWeight,&State);
				break;
			case 3:
				StateThree(&MyWeight,&State,&MaxWeight);
				if(checkTimer(3)){
					State = 0;
				}
				break;
			case 4:
				StateFour(&MyWeight,&State,&MaxWeight);
				break;
		}
	}
	*/
	//ControlLight();
	//app_light_direction();
	//app_control_light();
	//app_local_grouping();

	//IR_testing();
	//app_group_direction();
	//app_remote_control();
	testing();
}

void WeightBroadCast(uint8_t weight){
	uint8_t TxData[256];
	TxData[0] = weight;
	RF_Tx(0xFFFF,TxData,1);
}

void StateOne(uint8_t* weight){
	uint8_t RxData[256],RxLength,Rssi,RxPayload[256];
	if(RF_Rx(RxData,&RxLength,&Rssi)){
		getPayloadLength(&RxLength,RxData);
		getPayload(RxPayload,RxData,RxLength);
		if(*weight == RxPayload[0]){
			*weight = rand()%3;
			setTimer(2,2000,UNIT_MS);
		}
	}
}

void StateTwo(uint8_t* weight,uint8_t* State){
	uint8_t RxData[256],RxLength,Rssi,RxPayload[256],numOfGroup,temp[2];
	uint16_t GroupAddr[10],Addr;
	int direction,i;
	get_direction(&direction);
	numOfGroup = Group_Diff(GroupAddr,ATTRIBUTE_HEADING,direction,0x30);
	if(RF_Rx(RxData,&RxLength,&Rssi)){
		getPayloadLength(&RxLength,RxData);
		getPayload(RxPayload,RxData,RxLength);
		getSrcAddr(temp,RxData);
		Addr = temp[0] | temp[1] << 8;
		for(i=0;i<numOfGroup;i++){
			if(*weight < RxPayload[0] && Addr == GroupAddr[i]){
				*weight = RxPayload[0];
			}
			if(*weight == RxPayload[0] && Addr == GroupAddr[i]){
				setTimer(3,2000,UNIT_MS);
				*State = 3;
			}
		}
	}
}

int StateThree(uint8_t* weight,uint8_t* State,uint8_t* MaxWeight){
	uint8_t RxData[256],RxLength,Rssi,RxPayload[256],numOfGroup,temp[2];
	uint16_t GroupAddr[10],Addr;
	int direction,i;
	get_direction(&direction);
	numOfGroup = Group_Diff(GroupAddr,ATTRIBUTE_HEADING,direction,0x30);
	if(numOfGroup == 0){
		*State = 0;
		return 1;
	}
	if(RF_Rx(RxData,&RxLength,&Rssi)){
		getPayloadLength(&RxLength,RxData);
		getPayload(RxPayload,RxData,RxLength);
		getSrcAddr(temp,RxData);
		Addr = temp[0] | temp[1] << 8;
		for(i=0;i<numOfGroup;i++){
			if(*weight == RxPayload[0] && Addr == GroupAddr[i])
				setTimer(3,1500,UNIT_MS);
			if(*weight < RxPayload[0] && Addr == GroupAddr[i]){
				setTimer(3,0,UNIT_MS);
				setTimer(4,2000,UNIT_MS);
				*MaxWeight = RxPayload[0];
				*State = 4;
				return 1;
			}
		}
	}
	return 1;
}

int StateFour(uint8_t* weight,uint8_t* State,uint8_t* MaxWeight){
	uint8_t RxData[256],RxLength,Rssi,RxPayload[256],numOfGroup,temp[2],ListenStartFlag=0;
	uint16_t GroupAddr[10],Addr;
	int direction,i;
	get_direction(&direction);
	numOfGroup = Group_Diff(GroupAddr,ATTRIBUTE_HEADING,direction,0x30);
	if(RF_Rx(RxData,&RxLength,&Rssi) && ListenStartFlag ==1){
		getPayloadLength(&RxLength,RxData);
		getPayload(RxPayload,RxData,RxLength);
		getSrcAddr(temp,RxData);
		Addr = temp[0] | temp[1] << 8;
		for(i=0;i<numOfGroup;i++){
			if(*MaxWeight == RxPayload[0] && Addr == GroupAddr[i]){
				*weight = *MaxWeight;
				*State = 3;
				setTimer(5,0,UNIT_MS);
				return 1;
			}
		}
	}
	if(checkTimer(5)){
		*State = 3;
		return 1;
	}
	if(checkTimer(4)){
		setTimer(4,0,UNIT_MS);
		ListenStartFlag = 1;
		setTimer(5,1000,UNIT_MS);
	}
	return 1;
}

void ChangeLight(uint8_t MyWeight){
	switch(MyWeight){
		case 0:
			setGPIO(1 ,1);
			setGPIO(2 ,0);
			break;
		case 1:
			setGPIO(1 ,0);
			setGPIO(2 ,1);
			break;
		case 2:
			setGPIO(1 ,1);
			setGPIO(2 ,1);
			break;
	}
}
/*******************************************************************************
* Application Name  : Direction Local Grouping
* Description    		: Application
* Author            : Cheng Han
*******************************************************************************/

void app_group_direction(){
	
	uint8_t Type;
	uint16_t Addr;
	uint16_t radio_freq;
	uint16_t radio_panID;
	uint8_t State=0;
	uint8_t MyWeight = 0xFF;
	uint8_t MaxWeight = 0;
	Addr = 0x0001;
	Type = Type_Light;
	radio_freq = 2475;
	radio_panID = 0x00AA;

	Initial(Addr, Type, radio_freq, radio_panID);	
	setTimer(1,500,UNIT_MS);
	
	while(1){
		beacon();
		ChangeLight(MyWeight);
		if(checkTimer(1))
			WeightBroadCast(MyWeight);
		switch(State){
			case 0:
				MyWeight = rand()%3;
			  State = 1;
			  setTimer(2,2000,UNIT_MS);
				break;
			case 1:
				StateOne(&MyWeight);
			  if(checkTimer(2)){
					State = 2;
					setTimer(2,0,UNIT_MS);
				}
				break;
			case 2:
				StateTwo(&MyWeight,&State);
				break;
			case 3:
				StateThree(&MyWeight,&State,&MaxWeight);
				if(checkTimer(3)){
					State = 0;
				}
				break;
			case 4:
				StateFour(&MyWeight,&State,&MaxWeight);
				break;
		}
	}
}

/*
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
					GPIO_ON(1);
				else
					GPIO_OFF(1);
			}
		}
	}
		
}
*/

/*******************************************************************************
* Application Name  : Automatically turns on near lights in LOS
* Description    		: When a controller approaches lights, they will turn on 
*                   : their lights
* Author            : Ed Kung
* Date              : 2015.07.23
*******************************************************************************/
void app_remote_control(){
	uint8_t Type;
	uint16_t Addr;
	uint16_t radio_freq;
	uint16_t radio_panID;

	uint8_t i, k;
	uint16_t ID_LOS[10];
	uint16_t ID_RSSI[10];
	uint16_t ID_Both[10];
	uint8_t T_NUM_IR = 0;
	uint8_t T_NUM_RSSI = 0;
	uint8_t T_NUM_BOTH = 0;
	uint8_t num_LOS = 0;
	uint8_t num_RSSI = 0;
	
	uint8_t msg[256];
	uint8_t rcvd_msg[256];
	uint8_t rcvd_length;
	uint8_t rcvd_rssi;

	//Addr = 0x00EE;
	//Type = Type_Controller;
	
	Addr = 0x00BC;
	Type = Type_Light;

	radio_freq = 2475;
	radio_panID = 0x00CC;
	Initial(Addr, Type, radio_freq, radio_panID);	
	
	for(i=0;i<10;i++){
		ID_LOS[i] = 0xFF;
		ID_RSSI[i] = 0xFF;
		ID_Both[i] = 0xFF;
	}			
	
	if(Type == Type_Controller){
		setTimer(1,500,UNIT_MS);		// CONTROLLER
		setGPIO(1,1);
	}
	else if(Type == Type_Light) {
		setTimer(1,330,UNIT_MS);	// LIGHT
	}
	
	while(1){
			if(checkTimer(1)){
				if(Type == Type_Controller){
					num_LOS = get_LOS_device(ID_LOS);
					num_RSSI = getDeviceByRSSI(ID_RSSI, 180, 255);
					//num_RSSI = getDeviceByRSSI(ID_RSSI, 245, 255);
					
					// cross compare to find one light
					T_NUM_BOTH =0;
					if(num_LOS > 0 && num_RSSI > 0){
						for(k=0; k<NumOfDeviceInTable; k++){
							if(ID_RSSI[k]!=0xFFFF){
								for(i=0; i<NumOfDeviceInTable; i++){
									if(ID_LOS[i]!=0xFFFF){
										if(ID_RSSI[k] == ID_LOS[i] && ID_RSSI[k] != 0xFFFF){
											ID_Both[T_NUM_BOTH] = ID_RSSI[k];
											T_NUM_BOTH ++;
										}
									}
								}
							}
						}
					}
					if(T_NUM_BOTH>0){
						msg[0] = 0x01;
						msg[1] = T_NUM_BOTH;
						msg[2] = Type_Light;
						for(i=0; i<T_NUM_BOTH;i++){
							msg[3 + i] = ID_Both[i];
						}
						RF_Tx(0xFFFF, msg, 10);
					}		
				}
				
				if(Type == Type_Light){
					if(RF_Rx(rcvd_msg,&rcvd_length,&rcvd_rssi)){
						if(rcvd_msg[MAC_HEADER_LENGTH + 0] == 0x01 && 
							rcvd_msg[MAC_HEADER_LENGTH + 1] > 0x00 && 
							rcvd_msg[MAC_HEADER_LENGTH + 2] == Type_Light ){
							for(i=3; i<=(rcvd_length - MAC_HEADER_LENGTH); i++){	
								if(rcvd_msg[MAC_HEADER_LENGTH + i]==Addr){
									setGPIO(2,1);
									setTimer(2, 1750, UNIT_MS);
									break;
								}
							}
						}							
					}
				}		
		}	
		if(checkTimer(2)){
			if(Type == Type_Light){
				setTimer(2, 0, UNIT_MS);
				setGPIO(2,0);
			}
		}
	}
}


/*******************************************************************************
* Application Name  : Testing
* Description    		: Testing the abilities of the platform
* Author            : Ed Kung
* Date              : 2015.07.23
*******************************************************************************/
void testing(){
	uint8_t Type;
	uint16_t Addr;
	uint16_t radio_freq;
	uint16_t radio_panID;

	uint8_t i, k;
	
	uint8_t msg[256];
	uint8_t rcvd_msg[256];
	uint8_t rcvd_length;
	uint8_t rcvd_rssi;
	
	unsigned short* brighness;
	float* tmp;

	//Addr = 0x00EE;
	//Type = Type_Controller;
	
	Addr = 0x00BC;
	Type = Type_Light;

	radio_freq = 2475;
	radio_panID = 0x00CC;
	Initial(Addr, Type, radio_freq, radio_panID);	
	
	if(Type == Type_Controller){
		setTimer(1,500,UNIT_MS);		// CONTROLLER
		setGPIO(1,1);
	}
	else if(Type == Type_Light) {
		setTimer(1,500,UNIT_MS);	// LIGHT
	}
	
	while(1){
		if(checkTimer(1)){
			if(get_motion_status()){
					setGPIO(1,1);
					Delay(10);
					setGPIO(1,0);
				}
				else{
					setGPIO(2,1);
					Delay(10);
					setGPIO(2,0);				
				}
		}	
	}
}

