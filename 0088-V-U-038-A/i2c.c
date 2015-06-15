/******************** (C) COPYRIGHT 2013 Uniband Electronic Corp. **************
* File Name          : i2c.c
* Author             : Ethan Chung
* Version            : v1.0.0.0
* Date               : 25-Jun-2013
* Description        : This file provides a set of functions needed to manage the
*                      communication between SPI peripheral and US2400 Driver.
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
  * @brief  Initializes the I2C and put it into StandBy State (Ready for 
  *         data transfer).
  * @param  None
  * @retval None
  */
void I2C_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  I2C_InitTypeDef I2C_InitStructure;

  /* Reconfigure and enable I2C2 error interrupt to have the higher priority */
  NVIC_InitStructure.NVIC_IRQChannel = I2C2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /*!< SPI_CS_GPIO, SPI_MOSI_GPIO, SPI_MISO_GPIO, SPI_INT_GPIO 
       and SPI_SCK_GPIO Periph clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  /*!< SPI Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE); 

  /* Configure the I2C clock source. The clock is derived from the HSI */
 // RCC_I2CCLKConfig(RCC_I2C1CLK_HSI);

  /* Enable the DMA peripheral */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /*!< Configure SPI pins: SCK */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Connect PXx to I2C SCL */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_1); 

  /* Connect PXx to I2C SDA */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_1);  
  
  /*!< I2C Config */
  /* I2C2 configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
  I2C_InitStructure.I2C_DigitalFilter = 0x00;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_Timing = 0x10805E89;// 100khz 0x10805E89 ; 400khz 0x00901850;
  
  /* Apply I2C2 configuration after enabling it */
  I2C_Init(I2C2, &I2C_InitStructure);

  /* I2C2 Peripheral Enable */
  I2C_Cmd(I2C2, ENABLE);
}

/**
  * @brief  I2C Processing Read
  * @param  p: Specifies the data to be process.
  * @param  len: Specifies the data len to be process.
  * @retval None
  */
ErrorStatus I2C_Read(uint8_t DevID, uint8_t RegName, uint16_t NumByteToRead, uint8_t *p)
{
//  DMA_InitTypeDef  DMA_InitStructure;
  uint32_t DataNum = 0;
	uint32_t Timeout = 0;
/*
  if (I2C_GetFlagStatus(I2C2, I2C_ISR_BUSY) != RESET) {
    return ERROR;
  }
*/
  /* Test on BUSY Flag */
  Timeout = 0x10000;
  while(I2C_GetFlagStatus(I2C2, I2C_ISR_BUSY) != RESET)
  {
    if((Timeout--) == 0) return ERROR;
  }
  
  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2C2, DevID, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
  
  /* Wait until TXIS flag is set */
  Timeout = 0x10000;
  while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET)
  {
    if((Timeout--) == 0) return ERROR;
  }
  
  /* Send Register address */
  I2C_SendData(I2C2, (uint8_t)RegName);
  
  /* Wait until TC flag is set */
  Timeout = 0x10000;
  while(I2C_GetFlagStatus(I2C2, I2C_ISR_TC) == RESET)
  {
    if((Timeout--) == 0) return ERROR;
  }  
  
  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2C2, DevID, NumByteToRead, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
  
  /* Reset local variable */
  DataNum = 0;
  
  /* Wait until all data are received */
  while (DataNum != NumByteToRead)
  {
    /* Wait until RXNE flag is set */
    Timeout = 0x10000;
    while(I2C_GetFlagStatus(I2C2, I2C_ISR_RXNE) == RESET)    
    {
      if((Timeout--) == 0) return ERROR;
    }
    
    /* Read data from RXDR */
    *p++ = I2C_ReceiveData(I2C2);
    
    /* Update number of received data */
    DataNum++;
  }    
  
  /* Wait until STOPF flag is set */
  Timeout = 0x10000;
  while(I2C_GetFlagStatus(I2C2, I2C_ISR_STOPF) == RESET)   
  {
    if((Timeout--) == 0) return ERROR;
  }
  
  /* Clear STOPF flag */
  I2C_ClearFlag(I2C2, I2C_ICR_STOPCF);

	return SUCCESS;
}

/**
  * @brief  I2C Processing Write
  * @param  p: Specifies the data to be process.
  * @param  len: Specifies the data len to be process.
  * @retval None
  */
ErrorStatus I2C_Write(uint8_t DevID, uint8_t RegName, uint16_t NumByteToWrite, uint8_t *p)
{
//  DMA_InitTypeDef  DMA_InitStructure;
  uint32_t DataNum = 0;
	uint32_t Timeout = 0;
/*
  if (I2C_GetFlagStatus(I2C2, I2C_ISR_BUSY) != RESET) {
    return ERROR;
  }
*/
  /* Test on BUSY Flag */
  Timeout = 0x10000;
  while(I2C_GetFlagStatus(I2C2, I2C_ISR_BUSY) != RESET)
  {
    if((Timeout--) == 0) return ERROR;
  }
  
  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2C2, DevID, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);
  
  /* Wait until TXIS flag is set */
  Timeout = 0x10000;  
  while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET)   
  {
    if((Timeout--) == 0) return ERROR;
  }
  
  /* Send Register address */
  I2C_SendData(I2C2, (uint8_t)RegName);
  
  /* Wait until TCR flag is set */
  Timeout = 0x10000;
  while(I2C_GetFlagStatus(I2C2, I2C_ISR_TCR) == RESET)
  {
    if((Timeout--) == 0) return ERROR;
  }
  
  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2C2, DevID, NumByteToWrite, I2C_AutoEnd_Mode, I2C_No_StartStop);
  
  while (DataNum != NumByteToWrite)
  {      
    /* Wait until TXIS flag is set */
    Timeout = 0x10000;
    while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET)
    {
      if((Timeout--) == 0) return ERROR;
    }  
    
    /* Write data to TXDR */
    I2C_SendData(I2C2, (uint8_t)*p++);
    
    /* Update number of transmitted data */
    DataNum++;
  }  
  
  /* Wait until STOPF flag is set */
  Timeout = 0x10000;
  while(I2C_GetFlagStatus(I2C2, I2C_ISR_STOPF) == RESET)
  {
    if((Timeout--) == 0) return ERROR;
  }   
  
  /* Clear STOPF flag */
  I2C_ClearFlag(I2C2, I2C_ICR_STOPCF);

	return SUCCESS;
}

/******** (C) COPYRIGHT 2013 Uniband Electronic Corp. ******END OF FILE********/
