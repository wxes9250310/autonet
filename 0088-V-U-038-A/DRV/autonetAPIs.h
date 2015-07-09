/******************** (C) COPYRIGHT 2015 NXG Lab ******************************
* File Name          : autonetAPI.h
* Author             : AutoNet team
* Version            : V1.0.0.0
* Date               : 08/04/2015
* Description        : Header for autonetAPIs
*******************************************************************************/
/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __AUTONET_API_H
#define __AUTONET_API_H

#ifdef __cplusplus
 extern "C" {
#endif

	 
/* Includes ------------------------------------------------------------------*/
#include "us2400APIs.h"
#include "st.h"
	 
/* Private define ------------------------------------------------------------*/
#define Direction 	1 
#define Velocity 		2
#define LOS 				3
#define NumOfDeviceInTable 10

/* Structure definition ------------------------------------------------------*/
enum RECEVIE_DATA_TABLE
{
		ATTRIBUTE_HEADING,
		ATTRIBUTE_SPEED,
		ATTRIBUTE_GPS_LAT_DEG,
		ATTRIBUTE_GPS_LAT_MIN,
		ATTRIBUTE_GPS_LAT_SEC,	
		ATTRIBUTE_GPS_LAT_DIR,
		ATTRIBUTE_GPS_LONG_DEG,
		ATTRIBUTE_GPS_LONG_MIN,
		ATTRIBUTE_GPS_LONG_SEC,	
		ATTRIBUTE_GPS_LONG_DIR,
		ATTRIBUTE_LOS_FRONT,
		ATTRIBUTE_LOS_REAR,
    ATTRIBUTE_NUM
};

typedef struct{
	uint16_t type;
	uint16_t address;
	uint16_t attribute[ATTRIBUTE_NUM];
	uint8_t RSSI;
}Device;

typedef struct{
  Device device[NumOfDeviceInTable];
}Table;
	 
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
extern void lighting(uint8_t State);
void Autonet_Config(uint16_t SrcAddr, uint16_t type);
void Initialization(uint16_t, uint8_t, uint16_t, uint16_t);
void Initial(uint16_t srcAddr, uint8_t type, uint16_t radio_freq, uint16_t radio_panID);
void InitialCheck(void);
void TimerBeaconSetting(void);
void Autonet_search_type(char *a);
void beacon(void);

// Lights Control
void setGPIO(uint8_t pin_idx, uint8_t state);
void GPIO_ON(uint8_t n);
void GPIO_OFF(uint8_t n);
void blink(uint8_t n);

/* 9-axis */
void Mag_Error_Handle (short *pX,short *pY,short *pZ, short *max_x, short *min_x, short *max_y, short *min_y, short *max_z, short *min_z);
int Mag_flatsurface(short *pX,short *pY);
int getcompasscourse(short *ax,short *ay,short *az,short *cx,short *cy,short *cz);
void data_fetch(uint8_t* data_out, uint8_t* data_in, uint8_t d_offset, uint8_t d_length);

/* utility APIs */
uint8_t get_direction(int *heading_deg);
uint8_t get_brightness (unsigned short* brightness);
uint8_t get_gps(uint8_t*, uint8_t*, uint8_t*, uint8_t*, uint8_t*, uint8_t*, uint8_t*, uint8_t*);
uint8_t get_temperature(float* temp);
uint8_t get_velocity(int* speed);
uint8_t  IR_read(uint8_t, uint8_t, uint8_t);
uint8_t  IR_write(uint8_t, uint8_t, uint8_t);

//void get_gps(void);
void get_LOS_address(char *f_id, char *r_id);
void update_sensor_table(void);

void packet_receive(void);
void broadcastSend(void);
void broadcast(void);
void lightingSend(void);
void commandSend(void);
void Group_Configuration(void);
uint8_t Group_Diff(uint16_t* addr_array, uint8_t attribute, uint16_t center, uint16_t difference);
void lighting(uint8_t State);
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

#endif /* __AUTONET_API_H */

