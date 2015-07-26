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
#define NumOfDeviceInTable 10

/* Structure definition ------------------------------------------------------*/
enum RECEVIE_DATA_TABLE
{
		ATTRIBUTE_HEADING,
		ATTRIBUTE_SPEED,
		ATTRIBUTE_TMP,
		ATTRIBUTE_BRIGHTNESS,
		ATTRIBUTE_GPS_LAT_DEG,
		ATTRIBUTE_GPS_LAT_MIN,
		ATTRIBUTE_GPS_LAT_SEC,	
		ATTRIBUTE_GPS_LAT_DIR,
		ATTRIBUTE_GPS_LONG_DEG,
		ATTRIBUTE_GPS_LONG_MIN,
		ATTRIBUTE_GPS_LONG_SEC,	
		ATTRIBUTE_GPS_LONG_DIR,
    ATTRIBUTE_NUM
};

typedef struct{
	uint16_t type;
	uint16_t address;
	uint16_t Rssi;
	uint16_t attribute[ATTRIBUTE_NUM];
}Device;

typedef struct{
  Device device[NumOfDeviceInTable];
}Table;

typedef struct{
	uint16_t type;
	uint16_t address;
	// TODO: to modify as timer to 
	uint8_t count;
}IRDevice;

typedef struct{
  IRDevice IRdevice_1[NumOfDeviceInTable];
	IRDevice IRdevice_2[NumOfDeviceInTable];
}IRTable;
	 
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/* Configuration */
void Initial(uint16_t, uint8_t, uint16_t, uint16_t);
void SENSOR_CONFIGURATION(void);
void VARIABLE_Configuration(void);
void TimerBeaconSetting(void);
void Autonet_search_type(char *a);
uint8_t GroupByType(uint16_t* ID, uint8_t type);
void update_group_info(void);

/* Lights Control */
void setGPIO(uint8_t pin_idx, uint8_t state);
void GPIO_ON(uint8_t n);
void GPIO_OFF(uint8_t n);
void blink(uint8_t n);

/* utility APIs */
int getcompasscourse(short *ax,short *ay,short *az,short *cx,short *cy,short *cz);
uint8_t get_direction(int *heading_deg);
uint8_t get_brightness (unsigned short* brightness);
uint8_t get_gps_value(uint8_t*, uint8_t*, uint8_t*, uint8_t*, uint8_t*, uint8_t*, uint8_t*, uint8_t*);
uint8_t get_temperature(float* temp);
uint8_t get_velocity(int* speed);
uint8_t get_LOS_device(uint16_t* ID, int COM);
uint8_t get_motion_status(void);
uint8_t get_distance(uint16_t* ID, float distance);
uint8_t getDeviceByRSSI(uint16_t* ID,uint8_t min, uint8_t max);

/* Sensors' Accessing APIs */
uint8_t Pir_StatusCheck(void);
void Mag_Error_Handle (short *pX,short *pY,short *pZ, short *max_x, short *min_x, short *max_y, short *min_y, short *max_z, short *min_z);
int Mag_flatsurface(short *pX,short *pY);
void data_fetch(uint8_t* data_out, uint8_t* data_in, uint8_t d_offset, uint8_t d_length);

/* */
void packet_receive(void);
void IR_receive(int);
void broadcastSend(void);
void broadcast(void);
void lightingSend(void);
void commandSend(void);
void Group_Configuration(void);
uint8_t Group_Diff(uint16_t*, uint16_t*, uint8_t, uint16_t, uint16_t);

/* Inner Table Processing */
uint16_t ScanTableByAddress(uint16_t);
uint16_t ScanIRTableByAddress(uint16_t,int);
void update_sensor_table(void);
void setTable(uint8_t,uint16_t,uint8_t,uint8_t);
void setIRTable(uint8_t,uint16_t,uint8_t,int);
void ResetCountIRTable(uint8_t n,int);
void UpdateIRTable(void);
void IRupdate(void);

/* get messages from received frames */
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

