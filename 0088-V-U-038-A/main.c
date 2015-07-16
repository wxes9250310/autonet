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
	IR_testing2();
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

void IR_testing2(){
	uint8_t Type;
	uint16_t Addr;
	uint16_t radio_freq;
	uint16_t radio_panID;

	unsigned char IR_BufferRx1[64] = {0x0};
	unsigned char IR_BufferRx2[64] = {0x0};
	unsigned short IR_Buffer_Length1;
	unsigned short IR_Buffer_Length2;
	unsigned char rcvd_type1, rcvd_type2;
	unsigned char rcvd_addr1, rcvd_addr2;
	uint8_t TxBuffer[256],i=0;
	uint16_t rcvd_addr;
	uint16_t ID_IR[10];
	uint16_t ID_RSSI[10];
	uint16_t ID_Both[10];
	uint8_t T_NUM_IR = 0;
	uint8_t T_NUM_RSSI = 0;
	uint8_t T_NUM_BOTH = 0;
	uint16_t target;
	uint8_t num_IR = 0;
	uint8_t num_RSSI = 0;
	uint8_t k = 0;
	uint8_t existingFlag = 0;
	uint8_t renewFlag = 0;
	uint16_t Goal;
	
	typedef struct{
		uint16_t addr;
		uint8_t count;
	}DD;

	typedef struct{
  DD device[NumOfDeviceInTable];
  }TT;
	
	TT Table_IR;
	TT Table_RSSI;
	
	Addr = 0x000A;
	Type = Type_Light;
	
	//Addr = 0x0005;
	//Type = Type_Controller;
	
  //Addr = 0x00AA;
	//Type = Type_IR;

	radio_freq = 2475;
	radio_panID = 0x00AA;
	Initial(Addr, Type, radio_freq, radio_panID);	
	setTimer(1,1000,UNIT_MS);
	
	for(i=0;i<10;i++){
		ID_IR[i] = 0x0000;
		ID_RSSI[i] = 0x0000;
		ID_Both[i] = 0x0000;
		Table_IR.device[i].addr = 0xFFFF;
		Table_RSSI.device[i].addr = 0xFFFF;
	}			
	
	while(1){
			if(checkTimer(1)){
				// return list based on IR information
				/*
				num_IR = getDeviceByIR(ID_IR);
				if(num_IR != 0){
					existingFlag = 0;
					for(k=1; k<=num_IR; k++){
						target = ID_IR[k-1];
						for(i=0; i<NumOfDeviceInTable; i++){
							if(target == Table_IR.device[i].addr){		// exist in the table
								Table_IR.device[i].count =0;						// reset the count
								existingFlag = 1;
								break;
							}
						}
						// add a new member
						if(existingFlag == 0){
							T_NUM_IR ++;
							Table_IR.device[T_NUM_IR].addr = target;
							Table_IR.device[T_NUM_IR].count = 0;
						}
				  }
				}
				
				// delete a member
				for(i=0; i<NumOfDeviceInTable; i++){
					renewFlag  = 0;
					if(Table_IR.device[i].count == 5){		// renew the list of neighbors
						renewFlag = 1;
						T_NUM_IR --;
					}
					if(renewFlag == 1){		
						if(i+1 <= T_NUM_IR){
							Table_IR.device[i].addr = Table_IR.device[i+1].addr;
							Table_IR.device[i].count = Table_IR.device[i+1].count;
						}
						else{
							Table_IR.device[i].addr = 0xFFFF;
							Table_IR.device[i].count = 0x00;
						}
					}
				}
				
				// return list based on RSSI
				num_RSSI = getDeviceByRSSI(ID_RSSI, 150, 255);
				if(num_RSSI != 0){
					existingFlag = 0;
					for(k=1; k<=num_RSSI; k++){
						target = ID_RSSI[k-1];
						for(i=0; i<NumOfDeviceInTable; i++){
							if(target == Table_RSSI.device[i].addr){		// exist in the table
								Table_RSSI.device[i].count =0;						// reset the count
								existingFlag = 1;
								break;
							}
						}
						// add a new member
						if(existingFlag == 0){
							T_NUM_RSSI ++;
							Table_RSSI.device[T_NUM_RSSI].addr = target;
							Table_RSSI.device[T_NUM_RSSI].count = 0;
						}
				  }
				}
				
				// delete a member
				for(i=0; i<NumOfDeviceInTable; i++){
					renewFlag  = 0;
					if(Table_RSSI.device[i].count == 5){		// renew the list of neighbors
						renewFlag = 1;
						T_NUM_RSSI --;
					}
					if(renewFlag == 1){		
						if(i+1 <= T_NUM_RSSI){
							Table_RSSI.device[i].addr = Table_RSSI.device[i+1].addr;
							Table_RSSI.device[i].count = Table_RSSI.device[i+1].count;
						}
						else{
							Table_RSSI.device[i].addr = 0xFFFF;
							Table_RSSI.device[i].count = 0x00;
						}
					}
				}
				
				if(num_IR > 0){
					setGPIO(1,1);
					Delay(10);
					setGPIO(1,0);
				}
				else if(num_RSSI > 0){
					setGPIO(2,1);
					Delay(10);
					setGPIO(2,0);
				}
				*/
				num_IR = getDeviceByIR(ID_IR);
				num_RSSI = getDeviceByRSSI(ID_RSSI, 150, 255);
				
				// cross compare 
				// to find one light
				T_NUM_BOTH =0;
		
				if(num_IR > 0 && num_RSSI > 0){
					for(k=0; k<NumOfDeviceInTable; k++){
						if(ID_RSSI[k]!=0xFF){
							for(i=0; i<NumOfDeviceInTable; i++){
								if(ID_IR[i]!=0xFF){
									if(ID_RSSI[k] == ID_IR[i]){
										ID_Both[T_NUM_BOTH] = ID_RSSI[k];
										T_NUM_BOTH ++;
									}
								}
							}
						}
				  }
				}
				
				// TODO: compare two lists
				
			
				/*
				IR_read(IR_BufferRx1, &IR_Buffer_Length1, 1);
				//IR_read(IR_BufferRx2, &IR_Buffer_Length2, 2);
				
				if(IR_Buffer_Length1 !=0){
						rcvd_type1 =  IR_BufferRx1[3];
						rcvd_addr1 =  IR_BufferRx1[4];				
						if(rcvd_type1 == 0x01)
							blink(1);
					}
					
					if(IR_Buffer_Length2 !=0){
						rcvd_type2 =  IR_BufferRx2[3];
						rcvd_addr2 =  IR_BufferRx2[4];
						if(rcvd_type2 == 0x01)
							blink(1);
					}
					
					rcvd_type1 = rcvd_type2 = rcvd_addr1 = rcvd_addr2 = 0x00;
					IR_Buffer_Length1 = IR_Buffer_Length2 = 0;
				*/
				
		}
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
	//Addr = 0x0005;
	//Type = Type_Light;
	Addr = 0x00FF;
	Type = Type_Controller;
	radio_freq = 2450;
	radio_panID = 0x00AA;
	Initial(Addr, Type, radio_freq, radio_panID);

	// set timers
	setTimer(1, 500, UNIT_MS);
	setTimer(2, 500, UNIT_MS);

	while(1){ 	
		if(Type == Type_Light){											// Light
				Delay(10);
				// check whether the device should open the light or not
				if(checkTimer(1)){											// check every 1000 ms							
					times++;
					if(times==4){												  // every 2000 ms reset the light
						times=0;
						setGPIO(1, 0);
					}
					if(RF_Rx(RxData,&Rx_DataLen,&RSSI)){	// returns 1 if it has received something
						if(RxData[MAC_HEADER_LENGTH + 0] == Message_Light){		// lighting message
							// check the address of the devices whether exists or not
							for(i=1; i<= (Rx_DataLen-MAC_HEADER_LENGTH); i++){											
								if(RxData[MAC_HEADER_LENGTH + i]==Addr){
									setGPIO(1, 1);
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

/*******************************************************************************
* Application Name  : IR testing 
* Description    		: 1. transmitter and receiver
*                   : 2. IR beacon
* Author            : Ed Kung
*******************************************************************************/
void IR_testing(){

	uint16_t Addr;
	uint8_t Type;
	uint8_t detect;
	uint16_t radio_freq;
	uint16_t radio_panID;
	unsigned short IR_Buffer_Length1;
	unsigned short IR_Buffer_Length2;

	unsigned char rcvd_type1, rcvd_type2;
	unsigned char rcvd_addr1, rcvd_addr2;

	unsigned char IR_BufferTx[64] = {0x0};
	unsigned char IR_BufferRx1[64] = {0x0};
	unsigned char IR_BufferRx2[64] = {0x0};
	unsigned short Lux =0;

	Type = 0x01;
	Addr = 0x00AA;
	radio_freq = 2475;
	radio_panID = 0x00AA;
	Initial(Addr, Type, radio_freq, radio_panID);
	
	//Timer_Beacon(100);	
	setTimer(1, 200, UNIT_MS);
	setTimer(2, 1000, UNIT_MS);

	while(1){ 	
		if(checkTimer(1)){
			if(Type == 0x01){		// observer
				IR_read(IR_BufferRx1, &IR_Buffer_Length1, 1);
				Delay(10);
				IR_read(IR_BufferRx2, &IR_Buffer_Length2, 2);

				if(IR_Buffer_Length1 !=0){
					rcvd_type1 =  IR_BufferRx1[2];
					rcvd_addr1 =  IR_BufferRx1[3];				
					if(rcvd_type1 == 0x02)
						blink(1);
				}
				
				if(IR_Buffer_Length2 !=0){
					rcvd_type2 =  IR_BufferRx2[2];
					rcvd_addr2 =  IR_BufferRx2[3];
					if(rcvd_type2 == 0x02)
						blink(1);
				}
				
				rcvd_type1 = rcvd_type2 = rcvd_addr1 = rcvd_addr2 = 0x00;
				IR_Buffer_Length1 = IR_Buffer_Length2 = 0;
			}
		}
		if(checkTimer(2)){
			if(Type == 0x02){
				
  			IR_BufferTx[0] = (unsigned char) Addr;
				IR_BufferTx[1] = (unsigned char) Type;

				blink(1);
				IR_write(IR_BufferTx, 2 , 1);
				Delay(10);
				IR_write(IR_BufferTx, 2 , 2);
				Delay(10);
			}
		}
	}
}

