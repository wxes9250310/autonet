/******************** (C) COPYRIGHT 2009 U-PEC Electronics Corp. ***************
* File Name          : TxRx.h
* Author             : Ed Kung
* Version            : V1.0.0.0
* Date               : 29/12/2014
* Description        : Header for TxRx
*******************************************************************************/
/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __TxRx_H
#define __TxRx_H

#include "us2400APIs.h"
#include "st.h"

#ifdef __cplusplus
 extern "C" {
#endif

enum FRAME_BYTE
{
		FRAME_BYTE_HEADER,
		FRAME_BYTE_SRCADDR,
		FRAME_BYTE_TYPE,
		FRAME_BYTE_NUMOFSENSOR,
	  FRAME_BYTE_ATTRIBUTE,
};

enum FRAME_BYTE_L
{
		FRAME_BYTE_L_HEADER         = 0,
		FRAME_BYTE_L_TYPE        		= 1,
		FRAME_BYTE_L_SRC_ID         = 2,
		FRAME_BYTE_L_NUMOFNEIGHBOR  = 3,
		FRAME_BYTE_L_ADDR1    			= 4,			
		FRAME_BYTE_L_light1      		= 5,
		FRAME_BYTE_L_ADDR2   				= 6,
		FRAME_BYTE_L_light2    			= 7,
		FRAME_BYTE_L_ADDR3   				= 8,
		FRAME_BYTE_L_light3   			= 9,
		FRAME_BYTE_L_ADDR4    			= 10,
		FRAME_BYTE_L_light4   			= 11,
		FRAME_BYTE_L_ADDR5    			= 12,
		FRAME_BYTE_L_light5   			= 13
};

void RF_Tx(uint16_t destAddr, uint8_t *data, uint16_t dataLen);
uint8_t RF_Rx(uint8_t* RxData, uint8_t* Data_Length, uint8_t* RSSI);

void packet_receive(void);
void broadcastSend(void);
void broadcast(void);
void lightingSend(void);
void commandSend(void);
void Group_Configuration(void);
uint8_t Group_Diff(uint16_t* addr_array, uint8_t attribute, uint16_t center, uint16_t difference);
void lighting(uint8_t State);
int RF_RX_AUTONET(void);

int autonet_header_check(void);
uint8_t ScanTableByAddress(uint8_t);
void setTable(uint8_t,uint16_t,uint8_t);
void getSrcAddr(uint8_t* data_out, uint8_t* data_in);
void getDestAddr(uint8_t* data_out, uint8_t* data_in);
void getSrcPanID(uint8_t* data_out, uint8_t* data_in);
void getDestPanID(uint8_t* data_out, uint8_t* data_in);
void getSeqNum(uint8_t* data_out, uint8_t* data_in);
void getFrameControl(uint8_t* data_out, uint8_t* data_in);
void getPayload(uint8_t* data_out, uint8_t* data_in, uint8_t Data_Length);
void getPayloadLength(uint8_t* data_out, uint8_t* data_in);

#ifdef __cplusplus
}
#endif

#endif /* __TxRx_H */

