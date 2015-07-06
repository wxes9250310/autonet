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

#define NumofDeviceInTable 10
#define Length 256

int MyHeading;
uint16_t addr[NumofDeviceInTable],heading[NumofDeviceInTable],weight[NumofDeviceInTable];
uint16_t temp_addr,temp_heading,FollowingDevice,Addr,MyWeight,MyCommand,MyState;
uint8_t RxData[Length],TxData[Length],RxLength,TxLength,RxPayload[Length],command[NumofDeviceInTable],temp_weight,temp_command;
uint8_t RSSI;
uint8_t Type;
uint16_t radio_freq;
uint16_t radio_panID;
uint8_t commandList[5];

enum{
	Payload_Address_L,
	Payload_Address_H,
	Payload_Heading_L,
	Payload_Heading_H,
	Payload_Weight,
	Payload_Command
};


uint8_t ScanTableByAddress2(uint8_t scan_value){
	uint8_t i;
	for(i=0;i<NumofDeviceInTable;i++){
		if(scan_value == addr[i]){
			return i;
		}
	}
	return 0xFF;
}

void setInfoTable(uint8_t n,uint16_t device_addr,uint16_t device_heading,uint16_t device_weight,uint8_t device_command){
	addr[n] = device_addr;
	heading[n] = device_heading;
	weight[n] = device_weight;
	command[n] = device_command;
}

void ReGenerateWeight(){
	uint8_t i;
	while(1){
		MyWeight = rand();
		for(i=0;i<NumofDeviceInTable;i++){
			if(MyWeight==weight[i])
				continue;
		}
		break;
	}
}

void ReGenerateCommand(){
	uint8_t i;
	while(1){
		MyCommand = rand();
		for(i=0;i<NumofDeviceInTable;i++){
			if(MyCommand==command[i])
				continue;
		}
		break;
	}
}

uint16_t FindFollowingDevice(){
	
}

void ChangeLight(){
	uint8_t i;
	FollowingDevice = FindFollowingDevice();
	if(FollowingDevice == Addr){
		for(i=0;i<5;i++)
			if(commandList[i]!=1){
				MyCommand = i;
				MyState = i;
				break;
			}				
	}
	else{
		MyState = command[ScanTableByAddress2(Addr)];
	}
	switch(MyState){
		case 1:
			PIN_ON(1);
			PIN_OFF(2);
		break;
		case 2:
			PIN_ON(2);
			PIN_OFF(1);
		break;
		case 3:
			PIN_ON(1);
			PIN_ON(2);
		break;
		case 4:
		break;
		case 5:
		break;
		case 6:
		break;
	}
}

void app_local_grouping(){
	uint8_t idx;
	Initial(Addr, Type, radio_freq, radio_panID);
	setTimer(1,500,UNIT_MS);
	MyCommand = 0xFF;
	while(1){
		get_direction(&MyHeading);
		
		if(checkTimer(1)){
			TxData[Payload_Address_L] = Addr;
			TxData[Payload_Address_H] = Addr>>8;
			TxData[Payload_Heading_L] = MyHeading;
			TxData[Payload_Heading_H] = MyHeading>>8;
			TxData[Payload_Weight]  = MyWeight;
			TxData[Payload_Command] = MyCommand;
			RF_Tx(0xFFFF,TxData,6);
		}
		if(RF_Rx(RxData,&RxLength,&RSSI)){
			getPayload(RxPayload,RxData,RxLength);
			temp_addr = RxData[Payload_Heading_L] | RxData[Payload_Address_H] << 8;
			temp_heading = RxData[Payload_Heading_L] | RxData[Payload_Heading_H] << 8;
			temp_weight = RxData[Payload_Weight];
			temp_command = RxData[Payload_Command];
			if(temp_weight==MyWeight){
				ReGenerateWeight();
			}
			if(temp_command==MyCommand){
				ReGenerateCommand();
			}
			idx = ScanTableByAddress2(temp_addr);
			if(idx == 0xFF){
				idx = ScanTableByAddress2(0xFF);
				setInfoTable(idx,temp_addr,temp_heading,temp_weight,temp_command);
			}
			else{
				setInfoTable(idx,temp_addr,temp_heading,temp_weight,temp_command);
			}
			if(temp_command!=0xFF){
				commandList[temp_command] = 1;
			}
			ChangeLight();
		}
	}
}
