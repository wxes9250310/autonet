/******************** (C) COPYRIGHT 2013 Uniband Electronic Corp. **************
* File Name          : UART.c
* Author             : Ethan Chung
* Version            : v1.0.0.0
* Date               : 25-Jun-2013
* Description        : This file provides a set of functions needed to manage the
*                      UARTmunication between SPI peripheral and US2400 Driver.
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "com.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TXBUFFERSIZE   0xFF
#define RXBUFFERSIZE   0xFF

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
uint8_t UartTxBuffer[TXBUFFERSIZE] = {0};
uint8_t UartRxBuffer[RXBUFFERSIZE] = {0};
uint8_t NbrOfDataToTransfer = TXBUFFERSIZE;
uint8_t NbrOfDataToRead = RXBUFFERSIZE;
__IO uint8_t UartTxCount = 0;
__IO uint16_t UartRxCount = 0;
uint8_t UartTxState = 0;
uint8_t UartRxState = 0;

/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
  * @brief  Configures UART port.
  * @param  UART: Specifies the UART port to be configured.
  *          This parameter can be one of following parameters:    
  *            @arg UART1
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  */
void COM1_Configuration()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the USART Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

  /* USARTx configuration ------------------------------------------------------*/
  /* USARTx configured as follow:
  - BaudRate = 115200 baud
  - Word Length = 8 Bits
  - One Stop Bit
  - No parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */

  USART_DeInit(USART1);
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  /* Enable USART clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 

  /* Connect PXx to USARTx_Tx */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);

  /* Connect PXx to USARTx_Rx */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);
  
  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(USART1, &USART_InitStructure);
    
  /* Enable USART */
  USART_Cmd(USART1, ENABLE);

  /* Enable the EVAL_UART Receive interrupt: this interrupt is generated when the
  EVAL_UART receive data register is not empty */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

/**
  * @brief  UART port process.
  * @param  UART: Specifies the UART port to be configured.
  *          This parameter can be one of following parameters:
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  */
ErrorStatus COM1_Tx(uint8_t *p, uint16_t len)
{
  if (UartTxState != 0) {
    return ERROR;
  }

	UartTxState = 1;
	memcpy(UartTxBuffer, p, len);
	UartTxCount = 0;
	NbrOfDataToTransfer = len;
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	while(UartTxState == 1);

	return SUCCESS;
}

/**
  * @brief  Configures UART port.
  * @param  UART: Specifies the UART port to be configured.
  *          This parameter can be one of following parameters:    
  *            @arg UART1
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  */
void COM2_Configuration()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the USART Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x03;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

  /* USARTx configuration ------------------------------------------------------*/
  /* USARTx configured as follow:
  - BaudRate = 115200 baud
  - Word Length = 8 Bits
  - One Stop Bit
  - No parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */

  USART_DeInit(USART2);
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  /* Enable USART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 

  /* Connect PXx to USARTx_Tx */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);

  /* Connect PXx to USARTx_Rx */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
  
  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(USART2, &USART_InitStructure);
    
  /* Enable USART */
  USART_Cmd(USART2, ENABLE);

  /* Enable the EVAL_UART Receive interrupt: this interrupt is generated when the
  EVAL_UART receive data register is not empty */
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

/**
  * @brief  UART port process.
  * @param  UART: Specifies the UART port to be configured.
  *          This parameter can be one of following parameters:
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  */
ErrorStatus COM2_Tx(uint8_t *p, uint16_t len)
{
  if (UartTxState != 0) {
    return ERROR;
  }

	UartTxState = 1;
	memcpy(UartTxBuffer, p, len);
	UartTxCount = 0;
	NbrOfDataToTransfer = len;
  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
	while(UartTxState == 1);

	return SUCCESS;
}

void COM_Configuration()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the USART Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x03;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

  /* USARTx configuration ------------------------------------------------------*/
  /* USARTx configured as follow:
  - BaudRate = 115200 baud
  - Word Length = 8 Bits
  - One Stop Bit
  - No parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */

  USART_DeInit(USART2);
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  /* Enable USART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 

  /* Connect PXx to USARTx_Tx */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);

  /* Connect PXx to USARTx_Rx */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
  
  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(USART2, &USART_InitStructure);
    
  /* Enable USART */
  USART_Cmd(USART2, ENABLE);

  /* Enable the EVAL_UART Receive interrupt: this interrupt is generated when the
  EVAL_UART receive data register is not empty */
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

/**
  * @brief  UART port process.
  * @param  UART: Specifies the UART port to be configured.
  *          This parameter can be one of following parameters:
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  */
ErrorStatus COM_Tx(uint8_t *p, uint16_t len)
{
  if (UartTxState != 0) {
    return ERROR;
  }

	UartTxState = 1;
	memcpy(UartTxBuffer, p, len);
	UartTxCount = 0;
	NbrOfDataToTransfer = len;
  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
	while(UartTxState == 1);

	return SUCCESS;
}



/******** (C) COPYRIGHT 2013 Uniband Electronic Corp. ******END OF FILE********/
