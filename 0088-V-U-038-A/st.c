/******************** (C) COPYRIGHT 2013 Uniband Electronic Corp. **************
* File Name          : st.c
* Author             : Ethan Chung
* Version            : v1.0.0.0
* Date               : 25-Jun-2013
* Description        : This file provides a set of functions needed to manage the
*                      communication between SPI peripheral and US2400 Driver.
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "st.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TimObjTypeDef_s TimObj =
{
	{
    {0, 0}, // Tim STOP
    {0, 0}, // Delay
		{100, 100}, //TIM_100MS
		{1000,1000},//IR
	},
  0
};

/* Exported variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
  * @brief  Initializes the SPI and put it into StandBy State (Ready for 
  *         data transfer).
  * @param  None
  * @retval None
  */
void ST_Configuration(void)
{
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
}

/**
  * @brief  SPI Processing Tx/Rx
  * @param  p: Specifies the data to be process.
  * @param  len: Specifies the data len to be process.
  * @retval None
  */
void ST_Proc(void)
{
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 1 ms.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{
	TimObj.Tim[TIM_DELAY].Ticks = nTime;
	while(TimObj.Tim[TIM_DELAY].Ticks != 0);
}

/******** (C) COPYRIGHT 2013 Uniband Electronic Corp. ******END OF FILE********/
