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

/** @addtogroup STM32F0xx_StdPeriph_Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PAN_ID  0x00AA  
#define FRONT 1
#define REAR 2
#define TXBUFFERSIZE   0xFF
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t ID = 0x0000;
uint16_t TYPE = 0x0000;
uint8_t CommanderID = 0xFF;

uint8_t RFTxBuffer[TXBUFFERSIZE] = {0};
uint8_t Data[128] = {0}; 
uint8_t DataLen = 128;
uint8_t RFLqi = 0;
uint8_t RFRssi = 0;
uint8_t rssi = 0;
uint8_t pTxData[128];
uint8_t pRxData[128];
uint8_t framelength = 22;
uint8_t RFTxState = 0;
uint8_t RFRxState = 0;
uint8_t RFSleepState = 0;
uint8_t TimeoutFlag = 1;
uint8_t lightState=0;
							
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
//short max_x=129, min_x=-205, max_y=210, min_y=-141, max_z=59, min_z=-300;
short max_x=12, min_x=-20, max_y=21, min_y=-14, max_z=5, min_z=-30;

float Speed_x = 0;
float Speed_y = 0;
float Speed_z = 0;
uint16_t Speed=0;
int drive = 0;

double PI = 3.14159265359;
//char *s = "Hello World!";
//char s_len = sizeof("Hello World!");

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

enum{
	Light = 0x0001,
	Switch = 0x0002,
};

extern TimObjTypeDef_s TimObj;
extern uint16_t My_Data_table[];
extern int timer_flag_Beacon;

/* Private function prototypes -----------------------------------------------*/
static void GPIO_Configuration(void);
static void EXTI_Configuration(void);

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Main program.
  * @param  None
  * @retval None
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
	Us2400Init(radio_freq, radio_panID, srcAddr, 0);  //Us2400Init(Freq, PanID, SrcAddr, TPower);
	
  // TODO: to adjust the input of PIN_ON/PIN_OFF function (choose PINs)
  ID = srcAddr;
	PIN_ON(1);
	Delay(500);
	PIN_OFF(1);
	
	Mpu6050Init(0xD0);
  Ak8975Init(0x18);
  Mcp2120Init();       	/* MCP2120 Initialize */
	Bh1750fviInit(0x46);
	Tmp75Init(0x90);
	Mag3110Init(0x1C);
	
	//InitialCheck();
	TimerBeaconSetting(srcAddr, type);
}

void InitialCheck(){
	
	GPIOB->BSRR = GPIO_Pin_13;
	Delay(500);
	GPIOB->BRR = GPIO_Pin_13;
	Delay(500);
	GPIOB->BSRR = GPIO_Pin_13;
}

void TimerBeaconSetting(uint16_t SrcAddr, uint16_t type){
	 
	 if(type == Light){
		 ID = SrcAddr;
		 TYPE = type;
		 Timer_Beacon(1000);
	 }
	 else if(type == Switch){
		 ID = SrcAddr;
		 TYPE = type;
		 Timer_Beacon(2000);
	 }
	 else {				// not defined type
		 ID = SrcAddr;
     TYPE = 0x0000;
		 // TODO: to set non-beacon devices
     // Timer_Beacon(0);		 
	 }
	  
}

void Autonet_search_type(char *a){

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
/*
void Autonet_spatial_dynamic(uint16_t* ID,uint16_t type1, uint16_t value1, uint16_t type2, uint16_t value2, uint16_t type3, uint16_t value3){


	// -------------------------- sensors -------------------------- //
    Mpu6050ReadGyro(0xD0, &MPU6050GyroX, &MPU6050GyroY, &MPU6050GyroZ);
		Ak8975ReadMag(0x18, &AK8975MagX, &AK8975MagY, &AK8975MagZ);
		//Tmp75ReadTemperature(0x90, &Temperature);
		//Bh1750fviReadLx(0x46, &Lux);
		//Mag3110ReadMag(0x1C, &Mag3110MagX, &Mag3110MagY, &Mag3110MagZ);

		// ------------ Accerlometer by Cheng han -----------------
		time = TimObj.Tim[TIM_2].Period - TimObj.Tim[TIM_2].Ticks;
		TimObj.Tim[TIM_2].Ticks = TimObj.Tim[TIM_2].Period;
		Mpu6050ReadAccel(0xD0, &MPU6050AccX,&MPU6050AccY,&MPU6050AccZ);
		Estimate_State();
		Calculate_Speed(&Speed_x,&Speed_y,&Speed_z,&Speed,time);
		RealspeedL = drive;
		RealspeedH = drive >> 8;

		// --------------  GPS by Ed -----------------------------
		//Lea6SRead(0x84, &Lat_deg, &Lat_min, &Lat_sec, &Long_deg, &Long_min, &Long_sec, &Lat_dir, &Long_dir);   // read data from GPS

		// --------------  Magnetometer by chih-wei -----------------
		if(AK8975MagX>max_x) max_x = AK8975MagX;
		if(AK8975MagX<min_x) min_x = AK8975MagX;
		if(AK8975MagY>max_y) max_y = AK8975MagY;
		if(AK8975MagY<min_y) min_y = AK8975MagY;
		if(AK8975MagZ>max_z) max_z = AK8975MagZ;
		if(AK8975MagZ<min_z) min_z = AK8975MagZ;

		Mag_Error_Handle(&AK8975MagX, &AK8975MagY, &AK8975MagZ, &max_x, &min_x, &max_y, &min_y, &max_z, &min_z);
		flat_heading = Mag_flatsurface(&AK8975MagX, &AK8975MagY);  // flat surface degree (0~360)
		flat_headingL = flat_heading; // Lower 8 bit of flat_heading
		flat_headingH = flat_heading >> 8; // Higher 8 bit of flat_heading
//		tilt_heading = getcompasscourse(&AK8975MagX, &AK8975MagY, &AK8975MagZ,&MPU6050AccX, &MPU6050AccY, &MPU6050AccZ);
		
		// --------------  IR by chih-wei -----------------
		// TODO: there might be more than one car in front of or in rear of, we need buffers
		Mcp2120Tx((unsigned char *)s1, 2 , FRONT);
		Delay(10);
	  	Mcp2120Proc();
		Mcp2120Tx((unsigned char *)s2, 2 , REAR);
		Delay(10);

		// ============== Sensors' data table ================= //
		My_Data_table[TYPE_SPEED] = drive;
		My_Data_table[TYPE_GPS_LAT_DEG] = Lat_deg;
		My_Data_table[TYPE_GPS_LAT_MIN] = Lat_min;
		My_Data_table[TYPE_GPS_LAT_SEC] = Lat_sec;
		My_Data_table[TYPE_GPS_LAT_DIR] = Lat_dir;
		My_Data_table[TYPE_GPS_LONG_DEG] = Long_deg;
		My_Data_table[TYPE_GPS_LONG_MIN] = Long_min;
		My_Data_table[TYPE_GPS_LONG_SEC] = Long_sec;
		My_Data_table[TYPE_GPS_LONG_DIR] = Long_dir;
		My_Data_table[TYPE_HEADING] = flat_heading;
		My_Data_table[TYPE_LOS_FRONT] = FrontID;
		My_Data_table[TYPE_LOS_REAR] = RearID;

		// ============== Rx ================= //
		// ============== Check RX FIFO Full/Overflow ================= //
		if (Us2400ReadShortReg(0x30) == 0x90){
			Us2400Rx(Data, &DataLen, &RFLqi, &RFRssi);
			if (DataLen != 0){
				packet_receive();
				DataLen = 0;
			}
			Us2400WriteShortReg(0x0D, Us2400ReadShortReg(0x0D) | 0x01);
		}
		// ============== Packet Receive ================= //
		if (RFRxState == 1 || Us2400ReadShortReg(0x30) == 0x80){
			Us2400Rx(Data, &DataLen, &RFLqi, &rssi);
			if (DataLen != 0){
				// TODO: add judge function 
				packet_receive();
				DataLen = 0;
			}
			RFRxState = 0;
		}

		// ============== Event Handle (System Tick) ================= //
		if ((TimObj.TimeoutFlag & TIMOUT_FLAG_200MS) == TIMOUT_FLAG_200MS){
				broadcastSend();
				TimObj.TimeoutFlag ^= TIMOUT_FLAG_200MS;
		}
		if ((TimObj.TimeoutFlag & TIMOUT_FLAG_IRTO_FRONT) == TIMOUT_FLAG_IRTO_FRONT){
				FrontID = 0xFF;
				TimObj.TimeoutFlag ^= TIMOUT_FLAG_IRTO_FRONT;
		}
		if ((TimObj.TimeoutFlag & TIMOUT_FLAG_IRTO_REAR) == TIMOUT_FLAG_IRTO_REAR){
				RearID = 0xFF;
				TimObj.TimeoutFlag ^= TIMOUT_FLAG_IRTO_REAR;
		}
		if ((TimObj.TimeoutFlag & TIMOUT_FLAG_STOP) == TIMOUT_FLAG_STOP){
				drive = 0;
				TimObj.TimeoutFlag ^= TIMOUT_FLAG_STOP;
		}
		//Group_Process(ID,type1,value1,type2,value2,type3,value3);
}*/

void data_fetch(uint8_t* data_out, uint8_t* data_in, uint8_t d_offset, uint8_t d_length) {
	  
		int i;	
	
	  for(i=0; i< strlen((char*)data_out); i++)
			data_out[i] = 0;
		
		for(i=0; i< d_length; i++)
			data_out[i] = data_in[d_offset+i];
		
}


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

void get_gps(){
	
		Lea6SRead(0x84, &Lat_deg, &Lat_min, &Lat_sec, &Long_deg, &Long_min, &Long_sec, &Lat_dir, &Long_dir);   // read data from GPS
}

void update_sensor_table(){
		
		My_Data_table[TYPE_SPEED] = drive;
		My_Data_table[TYPE_GPS_LAT_DEG] = Lat_deg;
		My_Data_table[TYPE_GPS_LAT_MIN] = Lat_min;
		My_Data_table[TYPE_GPS_LAT_SEC] = Lat_sec;
		My_Data_table[TYPE_GPS_LAT_DIR] = Lat_dir;
		My_Data_table[TYPE_GPS_LONG_DEG] = Long_deg;
		My_Data_table[TYPE_GPS_LONG_MIN] = Long_min;
		My_Data_table[TYPE_GPS_LONG_SEC] = Long_sec;
		My_Data_table[TYPE_GPS_LONG_DIR] = Long_dir;
		My_Data_table[TYPE_HEADING] = flat_heading;
		My_Data_table[TYPE_LOS_FRONT] = FrontID;
		My_Data_table[TYPE_LOS_REAR] = RearID;
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

// --------------  Matnetometer functions by chih-wei -----------------

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

void PIN_ON(uint8_t n){
	// TODO: to distinguish GPIOA and GPIOB
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

void RF_beacon(void){
	if(RF_RX_AUTONET()){					// check AutoNet header
		packet_receive();						// receive sensors' data from others
	}
	if(timer_flag_Beacon == 1){
		if(TYPE == Light){
		
			My_Data_table[TYPE_GPS_LAT_DEG] = Lat_deg;
			My_Data_table[TYPE_GPS_LAT_MIN] = Lat_min;
			My_Data_table[TYPE_GPS_LAT_SEC] = Lat_sec;
			My_Data_table[TYPE_GPS_LAT_DIR] = Lat_dir;
			My_Data_table[TYPE_GPS_LONG_DEG] = Long_deg;
			My_Data_table[TYPE_GPS_LONG_MIN] = Long_min;
			My_Data_table[TYPE_GPS_LONG_SEC] = Long_sec;
			My_Data_table[TYPE_GPS_LONG_DIR] = Long_dir;
			My_Data_table[TYPE_HEADING] = flat_heading;
			My_Data_table[TYPE_LOS_FRONT] = FrontID;
			My_Data_table[TYPE_LOS_REAR] = RearID;
			
			broadcastSend();
			timer_flag_Beacon = 0;
		}
		else{
			get_direction(&flat_heading);
			//ToDo: More sensor data
		
			My_Data_table[TYPE_GPS_LAT_DEG] = Lat_deg;
			My_Data_table[TYPE_GPS_LAT_MIN] = Lat_min;
			My_Data_table[TYPE_GPS_LAT_SEC] = Lat_sec;
			My_Data_table[TYPE_GPS_LAT_DIR] = Lat_dir;
			My_Data_table[TYPE_GPS_LONG_DEG] = Long_deg;
			My_Data_table[TYPE_GPS_LONG_MIN] = Long_min;
			My_Data_table[TYPE_GPS_LONG_SEC] = Long_sec;
			My_Data_table[TYPE_GPS_LONG_DIR] = Long_dir;
			My_Data_table[TYPE_HEADING] = flat_heading;
			My_Data_table[TYPE_LOS_FRONT] = FrontID;
			My_Data_table[TYPE_LOS_REAR] = RearID;
			
			broadcastSend();
			timer_flag_Beacon = 0;
		}				
	}
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
