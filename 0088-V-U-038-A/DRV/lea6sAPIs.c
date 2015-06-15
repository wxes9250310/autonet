/******************** (C) COPYRIGHT 2014 STMicroelectronics *******************
* File Name          : lea6sAPIs.h
* Author             : Ethan Chung
* Version            : V1.0.0.0
* Date               : 04/11/2014
* Description        : This file provides a set of functions needed to manage the
*                      communication between I2C peripheral and LEA6S Driver.
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "i2c.h"
#include "lea6sREGs.h"
#include "lea6sAPIs.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
unsigned char Temp;
unsigned char TempBuffer[64];
unsigned char TempBufferCount;
unsigned char GPSDataReady = 0;
extern unsigned char Lat;
extern unsigned char Long;
extern unsigned int Lat_deg; 
extern unsigned int Lat_min;
extern unsigned int Lat_sec;
extern unsigned int Lat_dir;
extern unsigned int Long_deg;
extern unsigned int Long_min;
extern unsigned int Long_sec;
extern unsigned int Long_dir;
extern unsigned char pos1;
extern unsigned char pos2;

char *p = 0;
/* Exported variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/*******************************************************************************
* Function Name  : LEA6S_Init
* Description    : Initializes peripherals used by the Temperature Sensor driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Lea6SInit(unsigned char DevID)
{
  // Reset
}

/*******************************************************************************
* Function Name  : LEA6S_Read
* Description    : Read Temperature from the Temperature Sensor driver.
* Input          : None
* Output         : - Temperature: Temperature from Temperature Senson.
* Return         : None
*******************************************************************************/
//void Lea6SRead(unsigned char DevID, int *pLatitude, int *pLongitude, int *pHour, int *pMinute)
//void Lea6SRead(unsigned char DevID, unsigned char *cLatitude, unsigned char *cLongitude)
void Lea6SRead(unsigned char DevID, unsigned char *Lat_deg, unsigned char *Lat_min, unsigned char *Lat_sec, unsigned char *Long_deg, unsigned char *Long_min, unsigned char *Long_sec, unsigned char *Lat_dir, unsigned char *Long_dir)
{
  while(1) {
    I2C_Read(0x84, 0xFF, 0x1, &Temp);
	  if (Temp != 0xFF) {
			if (Temp == '$') {
				TempBufferCount = 0;
			}
			TempBuffer[TempBufferCount] = Temp;
			if ((TempBuffer[TempBufferCount - 1] == 0x0D) && (TempBuffer[TempBufferCount] == 0x0A)) {
        GPSDataReady = 0x01;
			  TempBufferCount = 0;
			  break;
      }
			else {
				TempBufferCount++;
			}
		}
	}
  if (GPSDataReady == 1) {
    GPSDataReady = 0;
    p = strstr((char *)TempBuffer, "$GPGLL");
    if (p != 0) {
      if (p[7] != ',') {
//				sprintf((char *)cLatitude,"%c%c%c%c%c%c%c%c",p[7],p[8],p[9],p[10],p[12],p[13],p[14],p[18]);       // Degree/minute/second + direction
//				sprintf((char *)cLongitude,"%c%c%c%c%c%c%c%c",p[20],p[21],p[22],p[23],p[24],p[26],p[27],p[32]);
				Lat_deg = (unsigned char *)((p[7] - 48) * 10 + (p[8] - 48));
				Lat_min = (unsigned char *)((p[9] - 48) * 10 + (p[10] - 48));									
				Lat_sec = (unsigned char *)((p[12] - 48) * 100 + (p[13] - 48) * 10 + (p[14] - 48) * 1);
				Long_deg = (unsigned char *)((p[20] - 48) * 100 + (p[21] - 48) * 10 + (p[22] - 48));
			  Long_min = (unsigned char *)((p[23] - 48) * 10 + (p[24] - 48));														// let the highest bit of Lat_min represent the direction W:1
			  Long_sec = (unsigned char *)((p[26] - 48) * 100 + (p[27] - 48) * 10 + (p[28] - 48) * 1);
		
				if (p[18] == 'N') {
					Lat_dir = (unsigned char *)('N');
				}
				else if(p[18] == 'S'){
					// TODO: reverse all the values	
					Lat_dir = (unsigned char *)('S');
				}
				else{
					Lat_deg = (unsigned char *)(255);
					Lat_min = (unsigned char *)(255);
					Lat_sec = (unsigned char *)(255);
					Lat_dir = 0x00;
				}
				
				if (p[32] == 'E') {
					Long_dir = (unsigned char *)('E');
				}
				else if(p[32] == 'W'){
					// TODO: reverse all the values
					Long_dir = (unsigned char *)('W');
				}
				else{
					Long_deg = (unsigned char *)(255);
					Long_min = (unsigned char *)(255);
					Long_sec = (unsigned char *)(255);
					Long_dir = 0x00;
				}
			}
		}
	}
}

/*
//void GPStoInt(unsigned char *Lat, unsigned char *Long, unsigned char *pos1, unsigned char *pos2, unsigned int *Lat_deg, unsigned int *Lat_min, unsigned int *Lat_sec, unsigned int *Long_deg, unsigned int *Long_min, unsigned int *Long_sec)
void GPStoInt(unsigned char *Lat, unsigned char *Long)
{
		if(Lat_min > 128)  pos1 = 'N';
		else	pos1 = 'S';
	
		if(Long_min > 128)  pos1 = 'E';
		else	pos2 = 'W';	
	
		// TODO: change the values of GPS if we were in the South or in the West 
	
}
*/
/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
