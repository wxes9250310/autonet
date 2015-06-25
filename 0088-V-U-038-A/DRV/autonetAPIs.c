/**
  ******************************************************************************
  * @file    autonetAPIs.c
  * @author  AutoNet Team
  * @version V0
  * @date    02-March-2015
  * @brief   AutoNet's APIs
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "autonetAPIs.h"
#include "st.h"
#include "i2c.h"
#include "spi.h"
#include "com.h"
#include "mpu6050APIs.h"
#include "ak8975APIs.h"
#include "lea6sAPIs.h"
#include "tmp75APIs.h"
#include "mcp2120APIs.h"
#include "bh1750fviAPIs.h"
#include "mag3110APIs.h"
#include "us2400APIs.h"
#include "math.h"
#include "TxRx.h"
#include "stm32f0xx_it.h"

/** @addtogroup STM32F0xx_StdPeriph_Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define NUMOFBUFFER 		128
#define NUMOFSENSOR 		4
#define RX_OFFSET  12
#define PAN_ID  0x00AA  
#define FRONT 1
#define REAR 2

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t pTxData[128];
uint8_t pRxData[128];
uint8_t CommanderID = 0xFF;

Table table;
Device myAttribute;
uint8_t  Group[NumOfDeviceInTable];
uint16_t Condition[ATTRIBUTE_NUM] = {0x10,0xFF,0xFF};

int BeaconEnabled = 0;
int Group_count = 0;
int Group_flag = 0;
int num;
int i,j,k,gps_m;
int CheckTime;
int ConfirmGroupTime = 10;

enum{
	Message_Control,
	Message_Type,
	Message_BroadcastType,
	Message_Light,
};

enum{
	Type_Controller = 0x00,			// delete? 
	Type_Light = 0x01,
	Type_Switch = 0x02,
};

uint16_t _Addr;
uint8_t _Type;

uint8_t framelength = 0;
extern uint8_t Data[]; 
extern uint8_t DataLen;

extern TimObjTypeDef_s TimObj;
extern int BeaconTimerFlag;

/* Private function prototypes -----------------------------------------------*/
static void GPIO_Configuration(void);
static void EXTI_Configuration(void);

/* Private functions ---------------------------------------------------------*/

/**
	* @title Initialization
	* @brief 
	* @param 
	*/

void Initial(uint16_t srcAddr, uint8_t type, uint16_t radio_freq, uint16_t radio_panID){
		/* GPIO configuration */
		GPIO_Configuration();

		/* EXTI configuration */
		EXTI_Configuration();

		/* Software Timer configuration */
		ST_Configuration();

		/* COM configuration */
		COM1_Configuration();						//for IR sensors
		COM2_Configuration();						//for IR sensors
		//COM_Configuration();

		/* SPI configuration */
		SPI_Configuration();

		/* I2C configuration */
		I2C_Configuration();

		/* Us2400 Initialization*/
		//Us2400Init(Freq, PanID, SrcAddr, TPower);
		Us2400Init(radio_freq, radio_panID, srcAddr, 0);  
		
		Mpu6050Init(0xD0);
		Ak8975Init(0x18);
		Mcp2120Init();       	/* MCP2120 Initialize */
		Bh1750fviInit(0x46);
		Tmp75Init(0x90);
		Mag3110Init(0x1C);
		
		_Addr = srcAddr;
		_Type = type;
		
		blink();
		TimerBeaconSetting();
}

void TimerBeaconSetting(){
	 
		// TODO
		if(_Type == Type_Light){
				Timer_Beacon(500);
				BeaconEnabled = 1;
		}
		else if(_Type == Type_Switch){
				Timer_Beacon(200);
				BeaconEnabled = 1;
		}
		else{ 												// not defined type
				//Timer_Beacon(0);				// no beacon? 
				BeaconEnabled = 0;
		}
}

void RF_beacon(void){
	
		if(BeaconEnabled == 1){
			if(RF_RX_AUTONET()){						// check AutoNet header
					packet_receive();						// receive sensors' data from others
			}
			if(BeaconTimerFlag == 1){
				if(_Type == Type_Light){	
					broadcast();
					//blink();
					BeaconTimerFlag = 0;
				}
				else if(_Type == Type_Switch){	
					broadcast();
					//blink();
					BeaconTimerFlag = 0;
				}
				else{
					update_sensor_table();
					broadcastSend();
					BeaconTimerFlag = 0;
				}				
			}
		}
		else;
}

void broadcastSend(void)
{
	  uint8_t i;
		
		pTxData[FRAME_BYTE_HEADER] 			= 0xFF;
		pTxData[FRAME_BYTE_SRCADDR] 		= _Addr;
	  pTxData[FRAME_BYTE_TYPE]				= _Type;
		pTxData[FRAME_BYTE_NUMOFSENSOR] = NUMOFSENSOR;
	
	  for(i=0;i<ATTRIBUTE_NUM;i++){
		  pTxData[FRAME_BYTE_ATTRIBUTE + 2*i] = myAttribute.attribute[i] >> 8;
			pTxData[FRAME_BYTE_ATTRIBUTE + 2*i + 1] = myAttribute.attribute[i];
		}
		framelength = FRAME_BYTE_ATTRIBUTE + 2*ATTRIBUTE_NUM;
		
		RF_Tx(0xFFFF, pTxData, framelength);
}

void broadcast(void)
{
	  uint8_t i;
		
		pTxData[0] = Message_BroadcastType;
		pTxData[1] = _Type;
	  pTxData[2] = _Addr;
	
	// TODO: to send out the status of the device

		for(i=3;i<128;i++){
			pTxData[i]=0x00;
		}	
		RF_Tx(0xFFFF, pTxData, 10);
}

void packet_receive(void)
{
	uint8_t index=0xFF;
	uint8_t newIndex=0xFF;
	uint8_t type;
	uint16_t addr;
	
	memcpy(pRxData, Data, DataLen);
	
	type = pRxData[FRAME_BYTE_TYPE+RX_OFFSET];
	addr = pRxData[FRAME_BYTE_SRCADDR+RX_OFFSET];
	index = ScanTableByAddress(addr);
	
	if(index == 0xFF){														// no such address in the table
		newIndex = ScanTableByAddress(0xFF);
		if(newIndex != 0xFF){
			setTable(newIndex,addr,type);
		}
	}
	else{
		setTable(index,addr,type);
	}
}

uint8_t ScanTableByAddress(uint8_t scan_value){
	for(i=0;i<NumOfDeviceInTable;i++){
		if(scan_value == table.device[i].address){
			return i;
		}
	}
	return 0xFF;
}

void setTable(uint8_t n,uint16_t device_addr,uint8_t device_type){
	table.device[n].type = device_type;
	table.device[n].address = device_addr;
  for(i=0;i<ATTRIBUTE_NUM;i++)
		table.device[n].attribute[i] = pRxData[2*i+5+RX_OFFSET] | (pRxData[2*i+4+RX_OFFSET]<<8);
}

void blink(){
		PIN_ON(1);
		Delay(100);
		PIN_OFF(1);
}

void PIN_ON(uint8_t n){
	// TODO: to distinguish GPIOA and GPIOB
	// TODO: to find available GPIO
	switch(n){
		case 1:
			GPIOB->BSRR = GPIO_Pin_13;
		break;
	}
}

void PIN_OFF(uint8_t n){
	// TODO: to distinguish GPIOA and GPIOB
	switch(n){
		case 1:
			GPIOB->BRR = GPIO_Pin_13;
		break;
	}
}


void getSrcAddr(uint8_t* data_out, uint8_t* data_in){

		memset(data_out,0x00,sizeof(data_out));
  	data_out[0] = data_in[10];
	  data_out[1] = data_in[11];
}

void getDestAddr(uint8_t* data_out, uint8_t* data_in){

		memset(data_out,0x00,sizeof(data_out));
  	data_out[0] = data_in[6];
	  data_out[1] = data_in[7];
}

void getSrcPanID(uint8_t* data_out, uint8_t* data_in){

		memset(data_out,0x00,sizeof(data_out));
  	data_out[0] = data_in[8];
	  data_out[1] = data_in[9];
}

void getDestPanID(uint8_t* data_out, uint8_t* data_in){

		memset(data_out,0x00,sizeof(data_out));
  	data_out[0] = data_in[4];
	  data_out[1] = data_in[5];
}

void getSeqNum(uint8_t* data_out, uint8_t* data_in){

		memset(data_out,0x00,sizeof(data_out));
  	data_out[0] = data_in[3];
}

void getFrameControl(uint8_t* data_out, uint8_t* data_in){

		memset(data_out,0x00,sizeof(data_out));
  	data_out[0] = data_in[1];
	  data_out[1] = data_in[2];
}

void getPayload(uint8_t* data_out, uint8_t* data_in, uint8_t Data_Length){

	  int i;
		
	  memset(data_out,0x00,sizeof(data_out));
		for(i=0 ; i<Data_Length ; i++)
			data_out[i] = data_in[i+RX_OFFSET];
}

void getPayloadLength(uint8_t* data_out, uint8_t* data_in){

		memset(data_out,0x00,sizeof(data_out));
  	data_out[0] = data_in[0];
}

uint8_t Group_Diff(uint16_t* ID,uint8_t type, uint16_t center, uint16_t difference){
	  int NumofDevice = 0;
	/*	for(i=0;i<10;i++)
			ID[i] = 0xFFFF;
	  for(i=0;i<NUMOFSENSOR;i++)
			Condition[i] = 0xFF;
		// ------------ set Condition --------------//
		Condition[type] = difference;
		// ----- AutoNet algorithm ----- //
		Group_Configuration();
	
		for(i=0;i<NUMOFDEVICE;i++){
			Group_flag = 1;
			if(Condition[type] != 0xFF && abs(Data_table[i][type]-center) > Condition[type])  		// Not complete yet
				Group_flag = 0;																							                        // buffers is needed
			//TODO: grouping results will be accumulated to be a threshold to change the original group
			if(Group_flag == 1)
				Group[i]++;
		}
		for(i=0;i<NUMOFDEVICE;i++)
			if(Group[i] > 0){
				ID[NumofDevice+1] = i;
				NumofDevice++;
			}
		ID[0] = NumofDevice;
		*/
		return NumofDevice;
}

void Group_Configuration(){
		for(i =0; i < NumOfDeviceInTable; i++){
			Group[i] = 0x00;
		}
}

void lighting(uint8_t State){
    switch (State){
        case 0:
						GPIOB->BRR = GPIO_Pin_13;
            break;
        case 1:
						GPIOB->BSRR = GPIO_Pin_13;
						break;
        default:
						GPIOB->BRR = GPIO_Pin_13;
						break;
		}
}

void Autonet_search_type(char *a){

}

void data_fetch(uint8_t* data_out, uint8_t* data_in, uint8_t d_offset, uint8_t d_length) {
	  
		int i;	
	
	  for(i=0; i< strlen((char*)data_out); i++)
			data_out[i] = 0;
		
		for(i=0; i< d_length; i++)
			data_out[i] = data_in[d_offset+i];
		
}

/**
	* @title Sensors' APIs
	* @brief 
	* @param 
	*/

/* sensors variables */
short AK8975MagX = 0;
short AK8975MagY = 0;
short AK8975MagZ = 0;
short MPU6050AccX = 0;
short MPU6050AccY = 0;
short MPU6050AccZ = 0;
short MPU6050GyroX = 0;
short MPU6050GyroY = 0;
short MPU6050GyroZ = 0;
unsigned char RealspeedH;     // speed first byte
unsigned char RealspeedL;			// speed second byte
float Temperature = 0;
unsigned short Lux = 0;
short Mag3110MagX = 0;
short Mag3110MagY = 0;
short Mag3110MagZ = 0;
int time;
int flat_heading=999;
int tilt_heading=999;
char flat_headingH=0;
char flat_headingL=0;
short max_x=12, min_x=-20, max_y=21, min_y=-14, max_z=5, min_z=-30;
//short max_x=129, min_x=-205, max_y=210, min_y=-141, max_z=59, min_z=-300;

float Speed_x = 0;
float Speed_y = 0;
float Speed_z = 0;
uint16_t Speed=0;
int drive = 0;

double PI = 3.14159265359;

/* for IR */
char s1[2];
char s2[2];
char FrontID = 0xFF;
char RearID = 0xFF;

/* for GPS */
unsigned char Lat;
unsigned char Long;
unsigned char Lat_deg; 
unsigned char Lat_min;
unsigned char Lat_sec;
unsigned char Lat_dir;
unsigned char Long_deg;
unsigned char Long_min;
unsigned char Long_sec;
unsigned char Long_dir;
unsigned char pos1;
unsigned char pos2;
//int Hour = 0;
//int Minute = 0;

uint8_t get_direction(int *heading_deg){
	
    Mpu6050ReadGyro(0xD0, &MPU6050GyroX, &MPU6050GyroY, &MPU6050GyroZ);
		Ak8975ReadMag(0x18, &AK8975MagX, &AK8975MagY, &AK8975MagZ);
		
		if(AK8975MagX>max_x) max_x = AK8975MagX;
		if(AK8975MagX<min_x) min_x = AK8975MagX;
		if(AK8975MagY>max_y) max_y = AK8975MagY;
		if(AK8975MagY<min_y) min_y = AK8975MagY;
		if(AK8975MagZ>max_z) max_z = AK8975MagZ;
		if(AK8975MagZ<min_z) min_z = AK8975MagZ;

		Mag_Error_Handle(&AK8975MagX, &AK8975MagY, &AK8975MagZ, &max_x, &min_x, &max_y, &min_y, &max_z, &min_z);
		flat_heading = Mag_flatsurface(&AK8975MagX, &AK8975MagY);  // flat surface degree (0~360)
		flat_headingL = flat_heading; 			// Lower 8 bit of flat_heading
		flat_headingH = flat_heading >> 8; 	// Higher 8 bit of flat_heading
//		tilt_heading = getcompasscourse(&AK8975MagX, &AK8975MagY, &AK8975MagZ,&MPU6050AccX, &MPU6050AccY, &MPU6050AccZ);
	
	  *heading_deg = flat_heading;
		return 1;
}

void get_gps(){
		Lea6SRead(0x84, &Lat_deg, &Lat_min, &Lat_sec, &Long_deg, &Long_min, &Long_sec, &Lat_dir, &Long_dir);   // read data from GPS
}

// --------------  Matnetometer functions by chih-wei -----------------
void Mag_Error_Handle (short *pX, short *pY,short *pZ, short *max_x, short *min_x , short *max_y, short *min_y, short *max_z, short *min_z)
{
	short vmax_x,vmax_y,vmax_z;
	short vmin_x,vmin_y,vmin_z;
	short avgs_x,avgs_y,avgs_z;
	float avg_rad;
	float x_scale,y_scale,z_scale;

	vmax_x = *max_x - ((*min_x + *max_x)/2.0);
	vmax_y = *max_y - ((*min_y + *max_y)/2.0);
	vmax_z = *max_z - ((*min_z + *max_z)/2.0);

	vmin_x = *min_x - ((*min_x + *max_x)/2.0);
	vmin_y = *min_y - ((*min_y + *max_y)/2.0);
	vmin_z = *min_z - ((*min_z + *max_z)/2.0);

	avgs_x = vmax_x + (vmin_x*-1); //multiply by -1 to make negative values positive
	avgs_x = avgs_x / 2.0;
	avgs_y = vmax_y + (vmin_y*-1); //multiply by -1 to make negative values positive
	avgs_y = avgs_y / 2.0;
	avgs_z = vmax_z + (vmin_z*-1); //multiply by -1 to make negative values positive
	avgs_z = avgs_z / 2.0;

	avg_rad = avgs_x + avgs_y + avgs_z;
	avg_rad = avg_rad/3.0;

	x_scale = (avg_rad/avgs_x);
	y_scale = (avg_rad/avgs_y);
	z_scale = (avg_rad/avgs_z);

	*pX -= (*min_x + *max_x)/2.0;
	*pY -= (*min_y + *max_y)/2.0;
	*pZ -= (*min_z + *max_z)/2.0;

	*pX *= x_scale;
	*pX *= y_scale;
	*pX *= z_scale;
}

int Mag_flatsurface(short *pX,short *pY)
{
	int heading;
	/*heading = atan((double)(*pY) / (double)(*pX))* (180 / PI) -90;
	if (heading>0){heading=heading-360;}
	heading=360+heading;*/
	
	heading = atan2((double)(*pY), (double)*pX) * 180.0/3.14159265 + 180 -270;
	while (heading < 0) heading += 360;
	while (heading > 360) heading -= 360;
 
	return (heading);
	 
}

int getcompasscourse(short *ax,short *ay,short *az,short *cx,short *cy,short *cz)
{
	float xh,yh,ayf,axf;
	int var_compass;
	ayf=*ay/57.295;//Convert to rad
	axf=*ax/57.295;//Convert to rad
	xh=*cx*cos(ayf)+*cy*sin(ayf)*sin(axf)-*cz*cos(axf)*sin(ayf);
	yh=*cy*cos(axf)+*cz*sin(axf);
	var_compass=atan2((double)yh,(double)xh) * (180 / PI) -90-90; // angle in degrees
	if (var_compass>0){var_compass=var_compass-360;}
	var_compass=360+var_compass;
	//var_compass = atan2((*cz * sin(*ax) - *cy * cos(*ax)), *cx * cos(*ay) + *cy * sin(*ax) * sin(*ay) + *cz * sin(*ay) * cos(*ax))* 180.0/3.14159265 + 180;
	// while (var_compass < 0) var_compass += 360;
	//while (var_compass > 360) var_compass -= 360;
	return (var_compass);
}

/*
void get_LOS_address(char *f_id, char *r_id){
		
	  // TODO: there might be more than one car in front of or in rear of, we need buffers
	  // TODO: integrate following codes
		// TODO: return 0xFF is there is no car in front or rear of it
	  Mcp2120Tx((unsigned char *)s1, 2 , FRONT);
		Delay(10);
	  Mcp2120Proc();
		Mcp2120Tx((unsigned char *)s2, 2 , REAR);
		Delay(10);
	
	  *f_id = FRONT;
		*r_id = REAR;
}
*/


void update_sensor_table(){
		get_direction(&flat_heading);
		//ToDo: More sensor data
	
		myAttribute.attribute[ATTRIBUTE_SPEED] = drive;
		myAttribute.attribute[ATTRIBUTE_GPS_LAT_DEG] = Lat_deg;
		myAttribute.attribute[ATTRIBUTE_GPS_LAT_MIN] = Lat_min;
		myAttribute.attribute[ATTRIBUTE_GPS_LAT_SEC] = Lat_sec;
		myAttribute.attribute[ATTRIBUTE_GPS_LAT_DIR] = Lat_dir;
		myAttribute.attribute[ATTRIBUTE_GPS_LONG_DEG] = Long_deg;
		myAttribute.attribute[ATTRIBUTE_GPS_LONG_MIN] = Long_min;
		myAttribute.attribute[ATTRIBUTE_GPS_LONG_SEC] = Long_sec;
		myAttribute.attribute[ATTRIBUTE_GPS_LONG_DIR] = Long_dir;
		myAttribute.attribute[ATTRIBUTE_HEADING] = flat_heading;
		myAttribute.attribute[ATTRIBUTE_LOS_FRONT] = FrontID;
		myAttribute.attribute[ATTRIBUTE_LOS_REAR] = RearID;
}

/**
* @title Autonet_spatial_dynamic (API for AutoNet demo)
* @brief developers decide the target ID of devices and add those
* 			 attributes of interest to find out the devices which matches
*        the attributes
* @param ID: the ID of target device
* @param Direction: 123
* @param Velocity: 
* @param LOS: 
*/

void EXTI_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the EXTI Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource8);

  /* Configure Button EXTI line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line8;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;    
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set Button EXTI Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
}

/**
  * @brief  GPIO.
  * @param  COM: Specifies the COM port to be configured.
  *          This parameter can be one of following parameters:    
  *            @arg COM1
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  */
static void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIOB->BSRR = GPIO_Pin_14;
	// To represent different states by controlling LEDs
	GPIOB->BSRR = GPIO_Pin_13;
}

/**
  * @brief  Interrupt.
  * @param  COM: Specifies the COM port to be configured.
  *          This parameter can be one of following parameters:    
  *            @arg COM1
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  */
void RF_WakeUp(void)
{
	GPIOB->BSRR = GPIO_Pin_1;
	Delay(1);
	GPIOB->BRR = GPIO_Pin_1;
  Us2400Resume();
}
