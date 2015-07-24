 /**
  ******************************************************************************
  * @file    Project/STM32F0xx_StdPeriph_Templates/stm32f0xx_it.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    31-July-2013
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f0xx_it.h"
#include "st.h"
#include "com.h"
#include "us2400REGs.h"
#include "us2400APIs.h"
#include "autonetAPIs.h"
#include "mcp2120APIs.h"
#include "TxRx.h"
/** @addtogroup STM32F0xx_StdPeriph_Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define NUM_TIME_FLAG 8
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern TimObjTypeDef_s TimObj;

extern uint16_t _Addr;
extern uint8_t _Type;

extern uint8_t NbrOfDataToTransfer;
extern uint8_t NbrOfDataToRead;
extern __IO uint8_t UartTxCount;
extern __IO uint16_t UartRxCount;
extern uint8_t UartTxBuffer[];
extern uint8_t UartRxBuffer[];
extern uint8_t UartTxState;
extern uint8_t UartRxState;

extern unsigned char CommandRxBuffer[16];
extern unsigned char CommandRxBufferLen;
extern unsigned char CommandRxBuffer2[16];
extern unsigned char CommandRxBufferLen2;

extern uint8_t RFTxState;
extern uint8_t RFRxState;
extern uint8_t BeaconEnabled;
extern uint8_t RFSleepState;
void (*function)(void);

/* Timer for beacon */
int Timer_Connect_Flag_Beacon;
unsigned int timer_ticks_Beacon;
unsigned int timer_period_Beacon;
int BeaconTimerFlag;
/* Timer for IR beacon */
int Timer_Connect_Flag_IR_Beacon;
unsigned int timer_ticks_IR_Beacon;
unsigned int timer_period_IR_Beacon;
int IR_BeaconTimerFlag;

int Timer_Connect_Flag[NUM_TIME_FLAG];
unsigned int timer_ticks[NUM_TIME_FLAG];
unsigned int timer_period[NUM_TIME_FLAG];
uint8_t timer_flag[NUM_TIME_FLAG];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */

void Timer_Beacon(unsigned int time){
	timer_period_Beacon = time;
	timer_ticks_Beacon = time;
	Timer_Connect_Flag_Beacon = 1;
	BeaconTimerFlag = 0;
}


void Timer_IR_Beacon(unsigned int time){
	timer_period_IR_Beacon = time;
	timer_ticks_IR_Beacon = time;
	Timer_Connect_Flag_IR_Beacon = 1;
	IR_BeaconTimerFlag = 0;
}

void setTimer(uint8_t index, unsigned int period, uint8_t unit){
	if(period == 0){
		timer_period[index] = period;
		timer_ticks[index] = period;
		Timer_Connect_Flag[index] = 0;
		timer_flag[index] = 0;
	}
	else{
		timer_period[index] = period;
		timer_ticks[index] = period;
		Timer_Connect_Flag[index] = 1;
		timer_flag[index] = 0;
	}
}

uint8_t checkTimer (uint8_t index){
	
	if(timer_flag[index] == 1){
		timer_flag[index] = 0;
		return 1;
	}
	else {
		return 0;
	}
}

void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	unsigned char i = 0;
	uint8_t state = 0;
	//uint8_t AllowToDo =0;
	uint8_t CanRun = 1;

  for (i = 0; i < TIM_n; i++) {
    if ((TimObj.Tim[i].Ticks != 0) && (--TimObj.Tim[i].Ticks == 0)) {
			TimObj.Tim[i].Ticks = TimObj.Tim[i].Period;
		  TimObj.TimeoutFlag |= 1 << i;
    }
  }
	for (i = 0; i < 8 ; i++){
		if(Timer_Connect_Flag[i] == 1){
			if(--timer_ticks[i] == 0){
				timer_ticks[i] = timer_period[i];
				timer_flag[i] = 1;
			}
		}
  }
	
	//if(RFTxState == 0 && BeaconEnabled && I2COccupied == 0 && IRTxState == 0)
	//	AllowToDo=1;
	
	/* Beacon Read */
	/*
	if(Timer_Connect_Flag_Beacon_read==1){
		if(timer_ticks_Beacon_read != 0)
			--timer_ticks_Beacon_read;
		if(timer_ticks_Beacon_read == 0 && AllowToDo == 1){
			timer_ticks_Beacon_read = timer_period_Beacon_read;
			if(RFRxOccupied == 0){	
				if(RF_RX_AUTONET()){
					packet_receive();
				}
			}
		}
	}
	*/
	/* IR Beacon Read */
	/*
	if(Timer_Connect_Flag_IR_Beacon_read == 1){
		if(timer_ticks_IR_Beacon_read != 0)
			--timer_ticks_IR_Beacon_read;

		if(timer_ticks_IR_Beacon_read == 0 && AllowToDo ==1){
			timer_ticks_IR_Beacon_read = timer_period_IR_Beacon_read;
			if(IRRxState == 0){
				if(IR_broadcast_read(1)){
					IR_receive(1);
				}
			}
		}
	}
	*/

	
	/* Beacon */
	if(Timer_Connect_Flag_Beacon==1){
		if(timer_ticks_Beacon != 0)
			--timer_ticks_Beacon;
		if(timer_ticks_Beacon == 0){
			timer_ticks_Beacon = timer_period_Beacon;
			BeaconTimerFlag = 1;
			//broadcastSend();
		}
	}
	
	/* IR Beacon */
	if(Timer_Connect_Flag_IR_Beacon==1){
		if(timer_ticks_IR_Beacon != 0)
			--timer_ticks_IR_Beacon;
		if(timer_ticks_IR_Beacon == 0){
			timer_ticks_IR_Beacon = timer_period_IR_Beacon;
			IR_BeaconTimerFlag = 1;
			//IR_broadcast(_Addr, _Type, 1);
			//IRupdate();
		}
	}
}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @brief  This function handles USART1 global interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
		UartRxBuffer[UartRxCount] = USART_ReceiveData(USART1);
		if ((UartRxBuffer[UartRxCount - 1] == 0x7F) && (UartRxBuffer[UartRxCount] == 0x25)) {
			UartRxBuffer[0] = 0x7F;
			UartRxBuffer[1] = 0x25;
			UartRxCount = 1;
		}
		if ((UartRxBuffer[UartRxCount - 1] == 0x0D) && (UartRxBuffer[UartRxCount] == 0x0A)) {
			memcpy(CommandRxBuffer, UartRxBuffer, UartRxCount - 1);
			CommandRxBufferLen = UartRxCount - 1;
			UartRxCount = 0;
		}
		else {
			if (UartRxCount < 0x40) {
			  UartRxCount++;
		  }
			else {
				UartRxCount = 0;
			}
		}
  }

  if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET) {
    /* Write one byte to the transmit data register */
    USART_SendData(USART1, UartTxBuffer[UartTxCount++]);

    if(UartTxCount == NbrOfDataToTransfer) {
      /* Disable the USART1 Transmit interrupt */
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
			UartTxState = 0;
    }
  }
}

/**
  * @brief  This function handles USART2 global interrupt request.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
		UartRxBuffer[UartRxCount] = USART_ReceiveData(USART2);
		if ((UartRxBuffer[UartRxCount - 1] == 0x7F) && (UartRxBuffer[UartRxCount] == 0x25)) {
			UartRxBuffer[0] = 0x7F;
			UartRxBuffer[1] = 0x25;
			UartRxCount = 1;
		}
		if ((UartRxBuffer[UartRxCount - 1] == 0x0D) && (UartRxBuffer[UartRxCount] == 0x0A)) {
			memcpy(CommandRxBuffer2, UartRxBuffer, UartRxCount - 1);
			CommandRxBufferLen2 = UartRxCount - 1;
			UartRxCount = 0;
		}
		else {
			if (UartRxCount < 0x40) {
			  UartRxCount++;
		  }
			else {
				UartRxCount = 0;
			}
		}
  }

  if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET) {
    /* Write one byte to the transmit data register */
    USART_SendData(USART2, UartTxBuffer[UartTxCount++]);

    if(UartTxCount == NbrOfDataToTransfer) {
      /* Disable the USART2 Transmit interrupt */
      USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
			UartTxState = 0;
    }
  }
}

/**
  * @brief  This function handles External lines 4 to 15 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI4_15_IRQHandler(void)
{
  uint8_t IntStatus = 0;

  if(EXTI_GetITStatus(EXTI_Line8) != RESET) {
    IntStatus = Us2400ReadShortReg(ISRSTS);
    IntStatus |= Us2400ReadShortReg(ISRSTS);
    if ((IntStatus & 0x01) == 0x01) {
      RFTxState = 0x00;
    }
    if ((IntStatus & 0x08) == 0x08) {
      RFRxState = 0x01;
    }
    if ((IntStatus & 0x40) == 0x40) {
      RFSleepState = 0x00;
    }

    EXTI_ClearITPendingBit(EXTI_Line8);
  }
}

/**
  * @brief  This function handles I2C2 Error interrupt request.
  * @param  None
  * @retval None
  */
void I2C2_IRQHandler(void)
{
  /* Check on I2C1 SMBALERT flag and clear it */
  if (I2C_GetITStatus(I2C1, I2C_IT_ALERT))
  {
    I2C_ClearITPendingBit(I2C1, I2C_IT_ALERT);
  }
  /* Check on I2C1 Time out flag and clear it */
  if (I2C_GetITStatus(I2C1, I2C_IT_TIMEOUT))
  {
    I2C_ClearITPendingBit(I2C1, I2C_IT_TIMEOUT);
  }
  /* Check on I2C1 Arbitration Lost flag and clear it */
  if (I2C_GetITStatus(I2C1, I2C_IT_ARLO))
  {
    I2C_ClearITPendingBit(I2C1, I2C_IT_ARLO);
  }   
  /* Check on I2C1 PEC error flag and clear it */
  if (I2C_GetITStatus(I2C1, I2C_IT_PECERR))
  {
    I2C_ClearITPendingBit(I2C1, I2C_IT_PECERR);
  } 
  /* Check on I2C1 Overrun/Underrun error flag and clear it */
  if (I2C_GetITStatus(I2C1, I2C_IT_OVR))
  {
    I2C_ClearITPendingBit(I2C1, I2C_IT_OVR);
  } 
  /* Check on I2C1 Acknowledge failure error flag and clear it */
  if (I2C_GetITStatus(I2C1, I2C_IT_NACKF))
  {
    I2C_ClearITPendingBit(I2C1, I2C_IT_NACKF);
  }
  /* Check on I2C1 Bus error flag and clear it */
  if (I2C_GetITStatus(I2C1, I2C_IT_BERR))
  {
    I2C_ClearITPendingBit(I2C1, I2C_IT_BERR);
  }   
}

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
