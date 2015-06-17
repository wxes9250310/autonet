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
		FRAME_BYTE_HEADER        = 0,
		FRAME_BYTE_TYPE        		= 1,
		FRAME_BYTE_SRC_ID         = 2,
		FRAME_BYTE_NUMOFSENSOR    = 3,
		FRAME_BYTE_STYPE1    			= 4,			// sensor data type
		FRAME_BYTE_SVALUE11      	= 5,
		FRAME_BYTE_SVALUE12   		= 6,
		FRAME_BYTE_STYPE2    			= 7,
		FRAME_BYTE_SVALUE21   		= 8,
		FRAME_BYTE_SVALUE22   		= 9,
		FRAME_BYTE_STYPE3    			= 10,
		FRAME_BYTE_SVALUE31   		= 11,
		FRAME_BYTE_SVALUE32   		= 12,
		FRAME_BYTE_STYPE4    			= 13,
		FRAME_BYTE_SVALUE41   		= 14,
		FRAME_BYTE_SVALUE42   		= 15,
		FRAME_BYTE_STYPE5    			= 16,
		FRAME_BYTE_SVALUE51   		= 17,
		FRAME_BYTE_SVALUE52   		= 18,
		FRAME_BYTE_STYPE6    			= 19,
		FRAME_BYTE_SVALUE61   		= 20,
		FRAME_BYTE_SVALUE62   		= 21,
		FRAME_BYTE_STYPE7    			= 22,
		FRAME_BYTE_SVALUE71   		= 23,
		FRAME_BYTE_SVALUE72   		= 24
		
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

enum RECEVIE_DATA_TABLE
{
		TYPE_HEADING      = 0,
		TYPE_SPEED        = 1,
		TYPE_GPS_LAT_DEG  = 2,
		TYPE_GPS_LAT_MIN  = 3,
		TYPE_GPS_LAT_SEC  = 4,	
		TYPE_GPS_LAT_DIR  = 5,
		TYPE_GPS_LONG_DEG = 6,
		TYPE_GPS_LONG_MIN = 7,
		TYPE_GPS_LONG_SEC = 8,	
		TYPE_GPS_LONG_DIR = 9,
		TYPE_LOS_FRONT    = 10,
		TYPE_LOS_REAR     = 11,
	  SIZE_OF_DATA_TABLE,
};

void RF_Tx(uint16_t destAddr, uint8_t *data, uint16_t dataLen);
void packet_receive(void);
void broadcastSend(void);
void lightingSend(void);
void commandSend(void);
void Group_Configuration(void);
uint8_t Group_Diff(uint16_t* addr_array, uint8_t attribute, uint16_t center, uint16_t difference);
void lighting(uint8_t State);
int RF_RX_AUTONET(void);
int RF_RX(void);
uint8_t RF_Rx(uint8_t* RxData, uint8_t* Data_Length, uint8_t*);
int autonet_header_check(void);
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

