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

uint16_t addr[NumofDeviceInTable],heading[NumofDeviceInTable],weight[NumofDeviceInTable];
uint16_t temp_addr,temp_heading,FollowingDevice,Addr,MyHeading,MyWeight,MyCommand;
uint8_t RxData[Length],TxData[Length],RxLength,TxLength,RxPayload[Length],command[NumofDeviceInTable],temp_weight,temp_command;
uint8_t i;
uint8_t RSSI;
uint8_t Type;
uint16_t radio_freq;
uint16_t radio_panID;

enum{
	Payload_Address_L,
	Payload_Address_H,
	Payload_Heading_L,
	Payload_Heading_H,
	Payload_Weight,
	Payload_Command
};


uint8_t ScanTableByAddress(uint8_t scan_value){
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
	return 16;
}

void ChangeLight(){
	FollowingDevice = FindFollowingDevice();
	if(FollowingDevice == Addr){

	}
	else{

	}
}

int app_local_grouping(){
	Initial(Addr, Type, radio_freq, radio_panID);
	int index;
	setTimer(1,1000,UNIT_MS);
	while(1){
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
			index = ScanTableByAddress(temp_addr);
			if(index == 0xFF){
				index = ScanTableByAddress(0xFF);
				setInfoTable(index,temp_addr,temp_heading,temp_weight,temp_command);
			}
			else{
				setInfoTable(index,temp_addr,temp_heading,temp_weight,temp_command);
			}
			ChangeLight();
		}
	}
}
