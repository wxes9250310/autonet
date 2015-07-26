/******************** (C) COPYRIGHT 2013 Uniband Electronic Corp. **************
* File Name          : mpu6050.c
* Author             : Ethan Chung
* Version            : v1.0.0.0
* Date               : 25-Jun-2013
* Description        : This file provides a set of functions needed to manage the
*                      communication between SPI peripheral and mpu6050 Driver.
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "mpu6050REGs.h"
#include "mpu6050APIs.h"
#include "math.h"
#include "st.h"
#include "stdlib.h"



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define Drive_Threshold 1000
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/*******************************************************************************
  * @brief  Initialize mpu6050
  * @param  None
  * @retval None
  */
uint8_t alive_flag_MPU6050;

float true_x,true_y,true_z;
short ax_offset = 0;
short ay_offset = 0;
short az_offset = 0;
short mean_ax;
short mean_ay;
short mean_az;

int initial = 1;
int state = 0;
int buffersize=100;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=10;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)

extern TimObjTypeDef_s TimObj;
extern short MPU6050AccX, MPU6050AccY, MPU6050AccZ;
extern float Speed_x,Speed_y,Speed_z;
extern int drive;
//extern float ax[],ay[],az[];


void Mpu6050Init(unsigned char DevID)
{
  unsigned char buf = 0;

  I2C_Read(DevID, WHO_AM_I, 1, &buf);
	
	// check alive
	alive_flag_MPU6050 =0;
	if(buf == 0x68){
		alive_flag_MPU6050 = 1;
	}
	
	buf = 0x00;
	I2C_Write(DevID, PWR_MGMT_1, 1, &buf);
	buf = 0x07;
	I2C_Write(DevID, SMPLRT_DIV, 1, &buf);
	buf = 0x06;
	I2C_Write(DevID, CONFIG, 1, &buf);
	buf = 0x18;
	I2C_Write(DevID, GYRO_CONFIG, 1, &buf);
	buf = 0x00;
	I2C_Write(DevID, ACCEL_CONFIG, 1, (uint8_t *)&buf);
	loop();
}

/*******************************************************************************
  * @brief  mpu6050 Tx.
  * @param  DestAddr: To
  * @param  pData: The data Tx
  * @param  DataLen: The number of data need to Tx
  * @retval Ref to system.h - StatusTypeDef.
  */
void Mpu6050ReadAccel(unsigned char DevID, short *pX, short *pY, short *pZ)
{
  unsigned char buf[2] = {0, 0};

	I2C_Read(DevID, ACCEL_XOUT_L, 1, &buf[0]);
	I2C_Read(DevID, ACCEL_XOUT_H, 1, &buf[1] );
	*pX = (*(short *)buf)+ax_offset;
	I2C_Read(DevID, ACCEL_YOUT_L, 1, &buf[0]);
	I2C_Read(DevID, ACCEL_YOUT_H, 1, &buf[1]);
	*pY = (*(short *)buf)+ay_offset;
	I2C_Read(DevID, ACCEL_ZOUT_L, 1, &buf[0]);
	I2C_Read(DevID, ACCEL_ZOUT_H, 1, &buf[1]);
	*pZ = (*(short *)buf)+az_offset;
  /*ax[0] = (float)(*pX)/16384*9.8;
	//kalmanFilter(ax);
	ay[0] = (float)(*pY)/16384*9.8;
	//kalmanFilter(ay);
	az[0] = (float)(*pZ)/16384*9.8;
	//kalmanFilter(az);
	true_x = ax[0]-ax_offset;
	true_y = ay[0]-ay_offset;
	true_z = az[0]-az_offset;
	*/
}

void loop() {
  if (state==0){
    meansensors();
    state++;
    Delay(1000);
  }
  if (state==1) {
    calibration();
    state++;
		initial = 0;
    Delay(1000);
  }
}

void meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0;

  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    Mpu6050ReadAccel(0xD0, &MPU6050AccX, &MPU6050AccY, &MPU6050AccZ);
    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+MPU6050AccX;
      buff_ay=buff_ay+MPU6050AccY;
      buff_az=buff_az+MPU6050AccZ;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
    }
    i++;
    Delay(2); //Needed so we don't get repeated measures
  }
}

void calibration(){
  ax_offset=-mean_ax;
  ay_offset=-mean_ay;
  az_offset=-mean_az;
	
  while (1){
    int ready=0;
		
    meansensors();

    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay;

    if (abs(mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset-mean_az;

    if (ready==3) break;
  }
}

void Calculate_Speed(float *Speed_x,float *Speed_y,float *Speed_z, uint16_t *Speed, int t){
	uint16_t temp1,temp2,temp3,temp;
	true_x = (float)(MPU6050AccX)/16384*9.8;
	true_y = (float)(MPU6050AccY)/16384*9.8;
	true_z = (float)(MPU6050AccZ)/16384*9.8; 
	if( abs(MPU6050AccX) >= Acc_Threshold)
		(*Speed_x) += true_x*t/1000;
	if( abs(MPU6050AccY) >= Acc_Threshold)
		(*Speed_y) += true_y*t/1000;
	if( abs(MPU6050AccZ) >= Acc_Threshold)
		(*Speed_z) += true_z*t/1000;
	
	temp3 = pow(*Speed_z,2);
	temp1 = pow(*Speed_x,2);
	temp2 = pow(*Speed_y,2);
	
	temp = temp1+temp2+temp3;
	(*Speed) = (uint16_t)sqrt(temp);
}

void kalmanFilter(float *inData)
{                        
    //static float prevData=0;//inData[1]=prevData
    //static float p=10, q=0.0001, r=0.05, kGain=0;//inData[2]=p,inData[3]=q,inData[4]=r,inData[5]=kGain

    //Kalman filter function start*******************************
    inData[2] = inData[3] + inData[2];//p = p+q;
    inData[5] = inData[2]/(inData[2]+inData[4]);//kGain = p/(p+r);
    
    inData[0] = inData[1] + (inData[5]*(inData[0]-inData[1]));//inData = prevData+(kGain*(inData-prevData));
    
    inData[2] = (1-inData[5])*inData[2];//p = (1-kGain)*p;

		inData[1] = inData[0];//prevData = inData;
    //Kalman filter function stop********************************
    
    //return inData;
}

/*******************************************************************************
  * @brief  mpu6050 Tx.
  * @param  DestAddr: To
  * @param  pData: The data Tx
  * @param  DataLen: The number of data need to Tx
  * @retval Ref to system.h - StatusTypeDef.
  */
void Mpu6050ReadGyro(unsigned char DevID, short *pX, short *pY, short *pZ)
{
  unsigned char buf[2] = {0, 0};

	I2C_Read(DevID, GYRO_XOUT_L, 1, &buf[0]);
	I2C_Read(DevID, GYRO_XOUT_H, 1, &buf[1]);
	*pX = (*(short *)buf);
	I2C_Read(DevID, GYRO_YOUT_L, 1, &buf[0]);
	I2C_Read(DevID, GYRO_YOUT_H, 1, &buf[1]);
	*pY = (*(short *)buf);
	I2C_Read(DevID, GYRO_ZOUT_L, 1, &buf[0]);
	I2C_Read(DevID, GYRO_ZOUT_H, 1, &buf[1]);
	*pZ = (*(short *)buf);
}


int Estimate_State(){
		if(abs(MPU6050AccX)>Drive_Threshold || abs(MPU6050AccY)>Drive_Threshold || abs(MPU6050AccZ)>Drive_Threshold) {
			//drive = 1;
			
			TimObj.Tim[TIM_STOP].Ticks = 4000;
			return 1;
		}
		else 
			return 0;
}

/*******************************************************************************
  * @brief  mpu6050 Tx.
  * @param  DestAddr: To
  * @param  pData: The data Tx
  * @param  DataLen: The number of data need to Tx
  * @retval Ref to system.h - StatusTypeDef.
  */
void Mpu6050ReadTemp(unsigned char DevID, char *pTemp)
{
  unsigned char buf[2] = {0, 0};

	I2C_Read(DevID, TEMP_OUT_L, 1, &buf[0]);
	I2C_Read(DevID, TEMP_OUT_H, 1, &buf[1]);
	*pTemp = (*(short *)buf) / 340 + 36.53;
}

/******** (C) COPYRIGHT 2013 Uniband Electronic Corp. ******END OF FILE********/
