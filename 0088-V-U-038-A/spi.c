/******************** (C) COPYRIGHT 2013 Uniband Electronic Corp. **************
* File Name          : spi.c
* Author             : Ethan Chung
* Version            : v1.0.0.0
* Date               : 25-Jun-2013
* Description        : This file provides a set of functions needed to manage the
*                      communication between SPI peripheral and US2400 Driver.
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
  * @brief  Initializes the SPI and put it into StandBy State (Ready for 
  *         data transfer).
  * @param  None
  * @retval None
  */
void SPI_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef SPI_InitStructure;

  /*!< SPI_CS_GPIO, SPI_MOSI_GPIO, SPI_MISO_GPIO, SPI_INT_GPIO 
       and SPI_SCK_GPIO Periph clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);

  /*!< SPI Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); 

  /* Enable the DMA peripheral */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /*!< Configure SPI pins: SCK */
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;	// FCM2401
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;	// ST-Link
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  //GPIO_Init(GPIOB, &GPIO_InitStructure);	// FCM2401
	GPIO_Init(GPIOA, &GPIO_InitStructure);	// ST-Link

  /*!< Configure SPI_CS_PIN pin */
  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;		// FCM2401
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;		// ST-Link
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //GPIOA->BSRR = GPIO_Pin_15;
	GPIOA->BSRR = GPIO_Pin_4;

  /* Connect PXx to SPI_SCK */
  //GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_0);	//FCM2401
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_0);

  /* Connect PXx to SPI_MISO */
  //GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_0);	//FCM2401
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_0); 

  /* Connect PXx to SPI_MOSI */
  //GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_0);	//FCM2401
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_0);  
  
  /*!< SPI Config */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
  //SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;	// FCM2401

  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_Init(SPI1, &SPI_InitStructure);
  
  SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);

  SPI_Cmd(SPI1, ENABLE); /*!< SPI enable */
}

/**
  * @brief  SPI Processing Tx/Rx
  * @param  p: Specifies the data to be process.
  * @param  len: Specifies the data len to be process.
  * @retval None
  */
ErrorStatus SPI_Proc(uint8_t *p, uint16_t len)
{
  DMA_InitTypeDef  DMA_InitStructure;
	
	//if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_15) == Bit_RESET){ 
	if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_4) == Bit_RESET) {
		return ERROR;
	}
	
	//GPIOA->BRR = GPIO_Pin_15;
  GPIOA->BRR = GPIO_Pin_4;

  DMA_DeInit(DMA1_Channel3); 
  DMA_DeInit(DMA1_Channel2); 
  // DMA channel Rx of SPI Configuration
  DMA_InitStructure.DMA_BufferSize = (uint16_t) len;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) p;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize =  DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel2, &DMA_InitStructure);
  // DMA channel Tx of SPI Configuration
  DMA_InitStructure.DMA_BufferSize = (uint16_t) len;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) p;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
  DMA_Init(DMA1_Channel3, &DMA_InitStructure);

  // Enable the SPI Rx and Tx DMA requests
  DMA_Cmd(DMA1_Channel2, ENABLE);
  DMA_Cmd(DMA1_Channel3, ENABLE);
  // Wait the SPI DMA transfers complete
  while (!DMA_GetFlagStatus(DMA1_FLAG_TC3));
  while (!DMA_GetFlagStatus(DMA1_FLAG_TC2));
  DMA_Cmd(DMA1_Channel3, DISABLE);
  DMA_Cmd(DMA1_Channel2, DISABLE);
  DMA_ClearFlag(DMA1_FLAG_TC3);
  DMA_ClearFlag(DMA1_FLAG_TC2);

  //GPIOA->BRR = GPIO_Pin_15; 
	GPIOA->BSRR = GPIO_Pin_4;

  return SUCCESS;
}

/******** (C) COPYRIGHT 2013 Uniband Electronic Corp. ******END OF FILE********/
