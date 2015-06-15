/******************** (C) COPYRIGHT 2013 Uniband Electronic Corp. **************
* File Name          : mpu6050.c
* Author             : Ethan Chung
* Version            : v1.0.0.0
* Date               : 25-Jun-2013
* Description        : This file provides a set of functions needed to manage the
*                      communication between SPI peripheral and mpu6050 Driver.
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "mpuDmpAPIs.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
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
void MpuDMPInit(const signed char *mtx)
{
  mpu_init();
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_sample_rate(100);
  dmp_load_motion_driver_firmware();
  dmp_set_orientation(MpuDMPOrientationMatrixToScalar(mtx));
  dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
    DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
    DMP_FEATURE_GYRO_CAL);
  dmp_set_fifo_rate(100);
	dmp_enable_gyro_cal(1);
	MpuDMPRunSelfTest();
  mpu_set_dmp_state(1);
}

/*******************************************************************************
  * @brief  Initialize mpu6050
  * @param  None
  * @retval None
  */
void MpuDMPRead(short *gyro, short *accel, long *quat)
{
  short sensors;
  unsigned char more;
  unsigned long sensor_timestamp;

	dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
}

/*******************************************************************************
  * @brief  Initialize mpu6050
  * @param  None
  * @retval None
  */
void MpuDMPRunSelfTest(void)
{
  int result;
  long gyro[3], accel[3];
  float sens;
  unsigned short accel_sens;

  result = mpu_run_self_test(gyro, accel);
  if (result == 0x7) {
    /* Test passed. We can trust the gyro data here, so let's push it down
     * to the DMP.
     */
    mpu_get_gyro_sens(&sens);
    gyro[0] = (long)(gyro[0] * sens);
    gyro[1] = (long)(gyro[1] * sens);
    gyro[2] = (long)(gyro[2] * sens);
    dmp_set_gyro_bias(gyro);
    mpu_get_accel_sens(&accel_sens);
    accel[0] *= accel_sens;
    accel[1] *= accel_sens;
    accel[2] *= accel_sens;
    dmp_set_accel_bias(accel);
  }
}

/*******************************************************************************
  * @brief  Initialize mpu6050
  * @param  None
  * @retval None
  */
unsigned short MpuDMPOrientationMatrixToScalar(const signed char *mtx)
{
  unsigned short scalar;

  /*
    XYZ  010_001_000 Identity Matrix
    XZY  001_010_000
    YXZ  010_000_001
    YZX  000_010_001
    ZXY  001_000_010
    ZYX  000_001_010
  */

  scalar = MpuDMPRowToScale(mtx);
  scalar |= MpuDMPRowToScale(mtx + 3) << 3;
  scalar |= MpuDMPRowToScale(mtx + 6) << 6;

  return scalar;
}

/*******************************************************************************
  * @brief  Initialize mpu6050
  * @param  None
  * @retval None
  */
unsigned short MpuDMPRowToScale(const signed char *row)
{
  unsigned short b;

  if (row[0] > 0) {
    b = 0;
	}
  else if (row[0] < 0) {
    b = 4;
	}
  else if (row[1] > 0) {
    b = 1;
	}
  else if (row[1] < 0) {
    b = 5;
	}
  else if (row[2] > 0) {
    b = 2;
	}
  else if (row[2] < 0) {
    b = 6;
	}
  else {
    b = 7;      // error
	}

  return b;
}

/******** (C) COPYRIGHT 2013 Uniband Electronic Corp. ******END OF FILE********/
