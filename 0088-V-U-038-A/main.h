/******************** (C) COPYRIGHT 2015 NXG Lab ******************************
* File Name          : main.h
* Author             : AutoNet team
* Version            : V1.0.0.0
* Date               : 08/04/2015
* Description        : Header for main
*******************************************************************************/
/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "us2400APIs.h"
#include "st.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
enum{
		Message_Control,
		Message_Type,
};

enum{
		Type_Controller = 0x00,	
		Type_Light = 0x01,
		Type_Switch = 0x02,
	  Type_IR = 0x03,
};	 
	 
void app_light_direction(void);
void app_control_light(void);
void light_testing(void);
void app_group_direction(void);
void IR_testing(void);
void StateOne(uint8_t*);
void StateTwo(uint8_t*,uint8_t*);	 
int StateThree(uint8_t*,uint8_t*,uint8_t*);
int StateFour(uint8_t*,uint8_t*,uint8_t*);
void ChangeLight(uint8_t);
void WeightBroadCast(uint8_t);
void app_remote_control(void);
void testing(void);
	 
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

