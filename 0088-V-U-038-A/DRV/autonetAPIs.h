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

#define Direction 	1 
#define Velocity 		2
#define LOS 				3
 
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Autonet_Config(uint16_t SrcAddr, uint16_t type);
void Initialization(uint16_t, uint8_t, uint16_t, uint16_t);
void Initial(uint16_t srcAddr, uint8_t type, uint16_t radio_freq, uint16_t radio_panID);
void InitialCheck(void);
void TimerBeaconSetting(uint16_t SrcAddr, uint8_t type);
void Autonet_search_type(char *a);
void RF_beacon(void);

// Lights Control
void PIN_ON(uint8_t n);
void PIN_OFF(uint8_t n);
void blink(void);

void Mag_Error_Handle (short *pX,short *pY,short *pZ, short *max_x, short *min_x, short *max_y, short *min_y, short *max_z, short *min_z);
int Mag_flatsurface(short *pX,short *pY);
int getcompasscourse(short *ax,short *ay,short *az,short *cx,short *cy,short *cz);
void data_fetch(uint8_t* data_out, uint8_t* data_in, uint8_t d_offset, uint8_t d_length);

/* utility APIs */
uint8_t get_direction(int *heading_deg);
void get_gps(void);
void get_LOS_address(char *f_id, char *r_id);
void update_sensor_table(void);

extern void lighting(uint8_t State);

#ifdef __cplusplus
}
#endif

#endif /* __AUTONET_API_H */

