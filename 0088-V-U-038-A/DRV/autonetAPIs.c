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
#include <stdlib.h>
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
#define Message_BroadcastType 0xFF
#define Message_Light 0x03

#define NUMOFBUFFER 		128
#define NUMOFSENSOR 		4
#define MAC_HEADER_LENGTH  12
#define PAN_ID  0x00AA  
#define FRONT 1
#define REAR 2

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t pTxData[128];
uint8_t pRxData[128];
uint8_t CommanderID = 0xFF;

Table table;
IRTable IR_table;
Device myAttribute;
uint8_t Group[NumOfDeviceInTable];
uint8_t numOfIR=0;

int BeaconEnabled = 0;
int num;
int i,j,k,gps_m;
int CheckTime;
int ConfirmGroupTime = 10;
uint8_t I2COccupied = 0;

uint8_t GPS_ResetFlag = 1;
uint8_t GPS_ResetCount = 0;
uint8_t GPS_ResetMax = 10;

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

extern unsigned char CommandRxBuffer_BC[];
extern unsigned char CommandRxBufferLen_BC;
extern unsigned char CommandRxBuffer2_BC[];
extern unsigned char CommandRxBufferLen2_BC;
extern uint8_t alive_flag_MPU6050;

enum{
	Type_Controller = 0x00,
	Type_Light = 0x01,
	Type_Switch = 0x02,
};

uint16_t _Addr;
uint8_t _Type;

extern uint8_t Data[]; 
extern uint8_t DataLen;
extern uint8_t RSSI_BC;
extern int BeaconTimerFlag;
extern int IR_BeaconTimerFlag;
extern TimObjTypeDef_s TimObj;

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
	COM1_Configuration();						//for IR sensor 1
	COM2_Configuration();						//for IR sensor 2
	//COM_Configuration();

	/* SPI configuration */
	SPI_Configuration();

	/* I2C configuration */
	I2C_Configuration();
	
	/* Us2400 Initialization*/
	Us2400Init(radio_freq, radio_panID, srcAddr, 0);  
	
	/* Sensors' configuration */
	SENSOR_CONFIGURATION();
	
	_Addr = srcAddr;
	_Type = type;
	
	VARIABLE_Configuration();
	
	TimerBeaconSetting();
	
	blink(1);
}

void SENSOR_CONFIGURATION(){
	Mcp2120Init();
	Bh1750fviInit(0x46);
	Tmp75Init(0x90);
	Lea6SInit(0x84);
	Mpu6050Init(0xD0);
	Ak8975Init(0x18);
	Mag3110Init(0x1C);
}

void VARIABLE_Configuration(){
  for(i = 0;i<NumOfDeviceInTable;i++){
		table.device[i].address = 0xFFFF;
		IR_table.IRdevice_1[i].address = 0xFFFF;
		IR_table.IRdevice_2[i].address = 0xFFFF;
	}
}

void TimerBeaconSetting(){
	 
	if(_Type == Type_Light){
			//Timer_Beacon(150);			// backup
			//Timer_IR_Beacon(120);		// backup
			Timer_Beacon(150);
			Timer_IR_Beacon(120);
			BeaconEnabled = 1;
	}
	else if(_Type == Type_Switch){	
			Timer_Beacon(1000);		
			Timer_IR_Beacon(800);
			BeaconEnabled = 1;
	}
	else{ 											
			//Timer_Beacon(121);		// backup
			//Timer_IR_Beacon(80);	// backup
			Timer_Beacon(121);		
			Timer_IR_Beacon(80);	
			BeaconEnabled = 1;
	}
}

void update_group_info(void){
	//void beacon(void){
	if(BeaconEnabled == 1){	
		/* Receive RF */
		if(RF_RX_AUTONET()){				// check AutoNet header
			packet_receive();					// receive sensors' data from others
		}
		/* Receive IR */
		if(IR_broadcast_read(1)){
			IR_receive(1);
			Delay(10);
			IR_receive(2);
		}
		/* Beacon */
		if(BeaconTimerFlag == 1){
			update_sensor_table();
			broadcastSend();
			BeaconTimerFlag = 0;
		}
		/* IRBeacon */
		if(IR_BeaconTimerFlag == 1){	
			IR_broadcast(_Addr, _Type, 1);
			IRupdate();
			IR_BeaconTimerFlag = 0;
		}
	}
	else;
}

void broadcastSend(void)
{
	uint8_t i,framelength;
	
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

void packet_receive(void)
{
	uint8_t index=0xFF;
	uint8_t newIndex=0xFF;
	uint8_t type;
	uint8_t rssi;
	uint16_t addr;
	
	type = pRxData[FRAME_BYTE_TYPE + MAC_HEADER_LENGTH];
	addr = pRxData[FRAME_BYTE_SRCADDR + MAC_HEADER_LENGTH];
	rssi = RSSI_BC;
	index = ScanTableByAddress(addr);
	
	// for now, new members will fill in the position values 0xFF
	// considering sort out the table
	if(index == 0xFF){														// no such address in the table
		newIndex = ScanTableByAddress(0xFFFF);
		if(newIndex != 0xFF){
			setTable(newIndex,addr,type,rssi);
		}
	}
	else{
		setTable(index,addr,type,rssi);
	}
}

uint8_t getDeviceByRSSI(uint16_t* ID,uint8_t min, uint8_t max){
	
	int NumofDevice = 0;
	for(i=0;i<NumOfDeviceInTable;i++)
		ID[i] = 0xFFFF;
	
	for(i=0;i<NumOfDeviceInTable;i++){
		if(table.device[i].Rssi >= min && table.device[i].Rssi <= max){
			ID[NumofDevice] = table.device[i].address;
			NumofDevice++;
		}
	}
	return NumofDevice;
}

uint8_t getDeviceByIR(uint16_t* ID){
	
	int NumofDevice = 0;
	for(i=0;i<NumOfDeviceInTable;i++)
		ID[i] = 0xFFFF;
	
	for(i=0;i<NumOfDeviceInTable;i++){
		if(IR_table.IRdevice_1[i].address != 0xFFFF){
			ID[NumofDevice] = IR_table.IRdevice_1[i].address;
			NumofDevice++;
		}
	}
	return NumofDevice;
}

void IRupdate(){
	if ((TimObj.TimeoutFlag & TIMOUT_FLAG_IR) == TIMOUT_FLAG_IR){
		TimObj.TimeoutFlag ^= TIMOUT_FLAG_IR;
		CommandRxBufferLen_BC = 0;
		CommandRxBufferLen2_BC = 0;
		UpdateIRTable();		
	}
}	

void IR_receive(int COM)
{
	uint8_t index = 0xFF;
	uint8_t newIndex = 0xFF;
	uint8_t type = 0xFF;
	uint8_t rssi = 0x00;
	uint16_t addr = 0x00FF;
	
	if(COM == 1){
		if(CommandRxBuffer_BC[2] ==  2 && CommandRxBufferLen_BC != 0){
			type = CommandRxBuffer_BC[3];
			addr = CommandRxBuffer_BC[4];
		}
	}
	else if(COM == 2){
		if(CommandRxBuffer2_BC[2] ==  2 && CommandRxBufferLen2_BC != 0){
			type = CommandRxBuffer2_BC[3];
			addr = CommandRxBuffer2_BC[4];
		}
	}	
	if(addr != 0xFF && type != 0xFF){
		index = ScanIRTableByAddress(addr, COM);
		if(index == 0xFF){														// no such address in the table
			newIndex = ScanIRTableByAddress(0xFFFF, COM);
			if(newIndex != 0xFF){
				setIRTable(newIndex,addr,type, COM);
				numOfIR ++;
			}
		}
		else{
			ResetCountIRTable(index, COM);
		}
  }
}

uint16_t ScanTableByAddress(uint16_t scan_value){
	for(i=0;i<NumOfDeviceInTable;i++){
		if(scan_value == table.device[i].address){
			return i;
		}
	}
	return 0xFF;
}

uint16_t ScanIRTableByAddress(uint16_t scan_Addr, int COM){
	if(COM == 1)
		for(i=0;i<NumOfDeviceInTable;i++){
			if(scan_Addr == IR_table.IRdevice_1[i].address){
				return i;
			}
		}
	else if(COM == 2)
		for(i=0;i<NumOfDeviceInTable;i++){
			if(scan_Addr == IR_table.IRdevice_2[i].address){
				return i;
			}
		}
	return 0xFF;		// find nothing
}

void setTable(uint8_t n,uint16_t device_addr,uint8_t device_type, uint8_t rssi){
	table.device[n].type = device_type;
	table.device[n].address = device_addr;
	table.device[n].Rssi = rssi;
  for(i=0; i<ATTRIBUTE_NUM; i++)
		table.device[n].attribute[i] = pRxData[2*i+5+MAC_HEADER_LENGTH] | (pRxData[2*i+4+MAC_HEADER_LENGTH]<<8);
}

void setIRTable(uint8_t n,uint16_t device_addr,uint8_t device_type,int COM){
	if(COM == 1){
		IR_table.IRdevice_1[n].type = device_type;
		IR_table.IRdevice_1[n].address = device_addr;
		IR_table.IRdevice_1[n].count = 0;
	}
	else if(COM == 2){
		IR_table.IRdevice_2[n].type = device_type;
		IR_table.IRdevice_2[n].address = device_addr;
		IR_table.IRdevice_2[n].count = 0;
	}
}

void UpdateIRTable(){
	uint8_t renewFlag;
	uint8_t resetThreshold = 0x0F;
	
	for(i=0; i<NumOfDeviceInTable; i++){
		renewFlag  = 0;
		IR_table.IRdevice_1[i].count ++;
		
		if(IR_table.IRdevice_1[i].address != 0xFFFF 
				&& IR_table.IRdevice_1[i].address != 0x00 
				&&IR_table.IRdevice_1[i].count == resetThreshold){
			renewFlag = 1;
			numOfIR --;
				}
		if(IR_table.IRdevice_1[i].count == resetThreshold){
			IR_table.IRdevice_1[i].count = 0;
		}
		
		if(renewFlag == 1){		
			if(i+1 <= NumOfDeviceInTable){
				IR_table.IRdevice_1[i].type = IR_table.IRdevice_1[i+1].type;
				IR_table.IRdevice_1[i].address = IR_table.IRdevice_1[i+1].address;
				IR_table.IRdevice_1[i].count = IR_table.IRdevice_1[i+1].count;
			}
			else{
				IR_table.IRdevice_1[i].type = 0x00;
				IR_table.IRdevice_1[i].address = 0xFFFF;
				IR_table.IRdevice_1[i].count = 0x00;
			}
		}
	}
	for(i=0; i<NumOfDeviceInTable; i++){
		renewFlag  = 0;
		IR_table.IRdevice_2[i].count ++;
		
		if(IR_table.IRdevice_2[i].address != 0xFFFF 
				&& IR_table.IRdevice_2[i].address != 0x00 
				&&IR_table.IRdevice_2[i].count == resetThreshold){
			renewFlag = 1;
			numOfIR --;
				}
		if(IR_table.IRdevice_2[i].count == resetThreshold){
			IR_table.IRdevice_2[i].count = 0;
		}
		
		if(renewFlag == 1){		
			if(i+1 <= NumOfDeviceInTable){
				IR_table.IRdevice_2[i].type = IR_table.IRdevice_2[i+1].type;
				IR_table.IRdevice_2[i].address = IR_table.IRdevice_2[i+1].address;
				IR_table.IRdevice_2[i].count = IR_table.IRdevice_2[i+1].count;
			}
			else{
				IR_table.IRdevice_2[i].type = 0x00;
				IR_table.IRdevice_2[i].address = 0xFFFF;
				IR_table.IRdevice_2[i].count = 0x00;
			}
		}
	}
}

void ResetCountIRTable(uint8_t n, int COM){
	if(COM == 1){
		IR_table.IRdevice_1[n].count = 0;
	}
	else if(COM == 2){
		IR_table.IRdevice_2[n].count = 0;
	}
}

void blink(uint8_t n){
	if(n >= 1 && n <= 5){
		GPIO_ON(n);
		Delay(100);
		GPIO_OFF(n);
	}
}

void setGPIO(uint8_t pin_idx, uint8_t state){
	if(pin_idx >= 1 && pin_idx <= 4){
			if(state == 1)	
					GPIO_ON(pin_idx);
			else if(state == 0)
					GPIO_OFF(pin_idx);
	}
}

void GPIO_ON(uint8_t n){
	switch(n){
		case 1:
			GPIOB->BSRR = GPIO_Pin_13;
			break;
	  case 2:
			GPIOB->BSRR = GPIO_Pin_14;
			break;
		case 3:
			GPIOB->BSRR = GPIO_Pin_15;
			break;
		case 4:
			GPIOB->BSRR = GPIO_Pin_6;
			break;
		default:
			break;
	}
}

void GPIO_OFF(uint8_t n){
	switch(n){
		case 1:
			GPIOB->BRR = GPIO_Pin_13;
			break;
	  case 2:
			GPIOB->BRR = GPIO_Pin_14;
			break;
		case 3:
			GPIOB->BRR = GPIO_Pin_15;
			break;
		case 4:
			GPIOB->BRR = GPIO_Pin_6;
			break;
		default:
			break;
	}
}

uint8_t Group_Diff(uint16_t* ID, uint16_t* Value, uint8_t type, uint16_t center, uint16_t difference){
	int NumofDevice = 0;
	for(i=0;i<NumOfDeviceInTable;i++){
		ID[i] = 0xFFFF;
		Value[i] = 0x0000;
	}
	
	for(i=0;i<NumOfDeviceInTable;i++){
		if(difference != 0xFF && abs(table.device[i].attribute[type]-center) < difference){
			ID[NumofDevice] = table.device[i].address;
			Value[NumofDevice] = table.device[i].attribute[type];
			NumofDevice++;
		}
	}
	return NumofDevice;
}

//uint8_t Autonet_search_type(uint16_t* ID){
uint8_t GroupByType(uint16_t* ID, uint8_t type){
	int NumofDevice = 0;
	for(i=0;i<NumOfDeviceInTable;i++){
		ID[i] = 0xFFFF;
	}
	for(i=0;i<NumOfDeviceInTable;i++){
		if(table.device[i].type == _Type){
			ID[NumofDevice] = table.device[i].address;
			NumofDevice++;
		}
	}
	return NumofDevice;
}

/**
	* @title Sensors' APIs
	* @brief 
	* @param 
	*/

/**
	* @title get brightness
	* @brief 
	* @param 
	*/
uint8_t get_brightness(unsigned short* brightness){
	*brightness = 0;
	Bh1750fviReadLx(0x46, brightness);
	if(*brightness !=0) 
		return 1;
	else 
		return 0;
}

/**
	* @title get temperature
	* @brief 
	* @param 
	*/
uint8_t get_temperature(float* temp){
	*temp = 0;
	Tmp75ReadTemperature(0x90, temp);
	if(*temp !=0) 
		return 1;
	else 
		return 0;
}

/**
	* @title get GPS
	* @brief 
	* @param 
	*/

uint8_t get_gps_value(uint8_t* Lat_deg, uint8_t* Lat_min, uint8_t* Lat_sec, uint8_t* Long_deg, 
									uint8_t* Long_min, uint8_t* Long_sec, uint8_t* Lat_dir, uint8_t* Long_dir){
	return Lea6SRead(0x84, Lat_deg, Lat_min, Lat_sec, Long_deg, Long_min, Long_sec, Lat_dir, Long_dir);
}

/**
  * @brief  get line-of-sight device' address
  * @param  None
  * @retval None
  */
uint8_t get_LOS_device(uint16_t* ID, int COM){
	int NumofDevice = 0;
	for(i=0;i<NumOfDeviceInTable;i++)
		ID[i] = 0xFFFF;
	
	if(COM == 1)
		for(i=0;i<NumOfDeviceInTable;i++){
			if(IR_table.IRdevice_1[i].address != 0xFFFF){
				ID[NumofDevice] = IR_table.IRdevice_1[i].address;
				NumofDevice++;
			}
		}
	else if(COM == 2)
		for(i=0;i<NumOfDeviceInTable;i++){
			if(IR_table.IRdevice_2[i].address != 0xFFFF){
				ID[NumofDevice] = IR_table.IRdevice_2[i].address;
				NumofDevice++;
			}
		}
		
	return NumofDevice;
}

/**
	* @title get Motion Status
	* @brief 
	* @param 
	*/

uint8_t get_motion_status(){
	return Pir_StatusCheck();
}
uint8_t Pir_StatusCheck()
{
  if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == Bit_SET) {
		//GPIO_SetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_9);
		return 1;
  }
	else {
		//GPIO_ResetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_9);
		return 0;
	}
}

/**
	* @title 9-axis sensor
	* @brief get_direction()
	*  			 future work: get_velocity()
	* @param 
	* @author Chih-Wei, Chen-Han
	*/
int flat_heading = 999;
int tilt_heading = 999;
char flat_headingH = 0;
char flat_headingL = 0;
unsigned char RealspeedH;     // speed first byte
unsigned char RealspeedL;			// speed second byte
float Speed_x = 0;
float Speed_y = 0;
float Speed_z = 0;
uint16_t Speed=0;
int time;

short AK8975MagX = 0;
short AK8975MagY = 0;
short AK8975MagZ = 0;
short MPU6050AccX = 0;
short MPU6050AccY = 0;
short MPU6050AccZ = 0;
short MPU6050GyroX = 0;
short MPU6050GyroY = 0;
short MPU6050GyroZ = 0;
short Mag3110MagX = 0;
short Mag3110MagY = 0;
short Mag3110MagZ = 0;
short max_x=12, min_x=-20, max_y=21, min_y=-14, max_z=5, min_z=-30;
//short max_x=129, min_x=-205, max_y=210, min_y=-141, max_z=59, min_z=-300;
double PI = 3.14159265359;

// TODO: to check whether alive check works or not
uint8_t get_direction(int *heading_deg){
	if(alive_flag_MPU6050){
		I2COccupied = 1;
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
		// tilt_heading = getcompasscourse(&AK8975MagX, &AK8975MagY, &AK8975MagZ,&MPU6050AccX, &MPU6050AccY, &MPU6050AccZ);

		*heading_deg = flat_heading;
		I2COccupied = 0;
		return 1;
	} 
	else
		return 0;
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

uint8_t get_velocity(int* speed){
		
	return 1;
}

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

/**
	* @title update_GPS 
	* @brief maintain the values of GPS for a period of time
	*        since GPS cannot get positions in a very short period of time
	*/
void updateGPS(){
	if(GPS_ResetFlag){
		Lat_deg = Lat_min = Lat_sec = Lat_dir = Long_deg = Long_min = Long_sec = Long_dir = 0;
		if(get_gps_value(&Lat_deg, &Lat_min, &Lat_sec, &Long_deg, &Long_min, &Long_sec, &Lat_dir, &Long_dir)){
			GPS_ResetFlag = 0;
			setGPIO(1,1);
		}
	} else {
		++GPS_ResetCount;
		if(GPS_ResetCount == GPS_ResetMax){
			GPS_ResetFlag = 1;
			GPS_ResetCount = 0;
		}
	}
}

/**
	* @title get distance
	* @brief use RSSI to make reference distance tables 
	* @param 
	*/
void get_distance(){
	uint8_t rssi_boundary_0 = 255;
	uint8_t rssi_boundary_1 = 245;
	uint8_t rssi_boundary_2 = 220;
	uint8_t rssi_boundary_3 = 180;
	uint8_t rssi_boundary_4 = 150;
	uint8_t rssi_boundary_5 = 120;
	uint8_t rssi_boundary_6 = 80;
	
	
}

/**
	* @title update_sensor_table (AutoNet API)
	* @brief access sensors and update the table 
	*/
void update_sensor_table(){
	
	int heading = 0;
	int drive = 0;
	float tmp = 0;
	unsigned short brighness = 0;
	//unsigned char pos1 = 0;
	//unsigned char pos2 = 0;
	//int Hour = 0;
	//int Minute = 0;
	
	get_direction(&heading);
	get_brightness(&brighness);
	get_temperature(&tmp);
	updateGPS();
	
	myAttribute.attribute[ATTRIBUTE_HEADING] = heading;
	myAttribute.attribute[ATTRIBUTE_SPEED] = drive;
	myAttribute.attribute[ATTRIBUTE_TMP] = tmp;
	myAttribute.attribute[ATTRIBUTE_BRIGHTNESS] = brighness;
	myAttribute.attribute[ATTRIBUTE_GPS_LAT_DEG] = Lat_deg;
	myAttribute.attribute[ATTRIBUTE_GPS_LAT_MIN] = Lat_min;
	myAttribute.attribute[ATTRIBUTE_GPS_LAT_SEC] = Lat_sec;
	myAttribute.attribute[ATTRIBUTE_GPS_LAT_DIR] = Lat_dir;
	myAttribute.attribute[ATTRIBUTE_GPS_LONG_DEG] = Long_deg;
	myAttribute.attribute[ATTRIBUTE_GPS_LONG_MIN] = Long_min;
	myAttribute.attribute[ATTRIBUTE_GPS_LONG_SEC] = Long_sec;
	myAttribute.attribute[ATTRIBUTE_GPS_LONG_DIR] = Long_dir;
}

void data_fetch(uint8_t* data_out, uint8_t* data_in, uint8_t d_offset, uint8_t d_length) {
	  
		int i;	
	
	  for(i=0; i< strlen((char*)data_out); i++)
			data_out[i] = 0;
		
		for(i=0; i< d_length; i++)
			data_out[i] = data_in[d_offset+i];
		
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
			data_out[i] = data_in[i+MAC_HEADER_LENGTH];
}

void getPayloadLength(uint8_t* data_out, uint8_t* data_in){
  	*data_out = data_in[0];
}

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

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_6 | GPIO_Pin_15;
  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_6 | GPIO_Pin_15 | GPIO_Pin_2 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;								// PB0 for PIR sensor
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
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
