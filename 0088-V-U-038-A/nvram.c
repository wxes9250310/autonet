/******************** (C) COPYRIGHT 2013 Uniband Electronic Corp. **************
* File Name          : nvram.c
* Author             : Ethan Chung
* Version            : v1.0.0.0
* Date               : 25-Jun-2013
* Description        : This file provides a set of functions needed to manage the
*                      communication between SPI peripheral and US2400 Driver.
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "nvram.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define FLASH_PAGE_SIZE         ((uint32_t)0x00000400)   /* FLASH Page Size */
#define FLASH_USER_START_ADDR   ((uint32_t)0x08005000)   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ((uint32_t)0x08005400)   /* End @ of user Flash area */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
  * @brief  Flash Read.
  * @param  None
  * @retval None
  */
void Flash_Read(void)
{/*
  Address = FLASH_USER_START_ADDR;

  Mode = *(__IO uint16_t *)Address;
  Address += 2;
  Freq = *(__IO uint16_t *)Address;
  Address += 2;
  PanID = *(__IO uint16_t *)Address;
  Address += 2;
  SrcAddr = *(__IO uint16_t *)Address;
  Address += 2;
  DestAddr = *(__IO uint16_t *)Address;
	Address += 2;
	TPower = *(__IO uint16_t *)Address;*/
}

/**
  * @brief  Flash Write.
  * @param  None
  * @retval None
  */
void Flash_Write(void)
{
  FLASH_Unlock();

  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
  FLASH_ErasePage(FLASH_USER_START_ADDR);
/*  Address = FLASH_USER_START_ADDR;
  FLASH_ProgramHalfWord(Address, Mode);
  Address += 2;
  FLASH_ProgramHalfWord(Address, Freq);
  Address += 2;
  FLASH_ProgramHalfWord(Address, PanID);
  Address += 2;
  FLASH_ProgramHalfWord(Address, SrcAddr);
  Address += 2;
  FLASH_ProgramHalfWord(Address, DestAddr);
	Address += 2;
	FLASH_ProgramHalfWord(Address, TPower);
*/
  FLASH_Lock(); 
}

/******** (C) COPYRIGHT 2013 Uniband Electronic Corp. ******END OF FILE********/
