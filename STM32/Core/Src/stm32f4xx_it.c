/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void clearUartISr(USART_TypeDef *uart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	clearUartISr(USART1);
  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */
    if (LL_DMA_IsEnabledIT_HT(DMA2, LL_DMA_STREAM_2) && LL_DMA_IsActiveFlag_HT2(DMA2))
    {
        LL_DMA_ClearFlag_HT2(DMA2);
    }

	  if(LL_DMA_IsActiveFlag_TC2(DMA2))
	  {
	    LL_DMA_ClearFlag_TC2(DMA2);
	    /* Call function Transmission complete Callback */
	    DMA2_RxComplete_Callback();
	  }
	  else if(LL_DMA_IsActiveFlag_TE2(DMA2))
	  {
		  LL_DMA_ClearFlag_TE2(DMA2);
	    /* Call Error function */
		  DMA2_RxError_Callback();
	  }
  /* USER CODE END DMA2_Stream2_IRQn 0 */

  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */
    if (LL_DMA_IsEnabledIT_HT(DMA2, LL_DMA_STREAM_7) && LL_DMA_IsActiveFlag_HT7(DMA2))
    {
        LL_DMA_ClearFlag_HT7(DMA2);
    }

	  if(LL_DMA_IsActiveFlag_TC7(DMA2))
	  {
	    LL_DMA_ClearFlag_TC7(DMA2);
	    /* Call function Transmission complete Callback */
	    DMA2_TxComplete_Callback();
	  }
	  else if(LL_DMA_IsActiveFlag_TE7(DMA2))
	  {
		  LL_DMA_ClearFlag_TE7(DMA2);
	    /* Call Error function */
		  DMA2_TxError_Callback();
	  }
  /* USER CODE END DMA2_Stream7_IRQn 0 */

  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/* USER CODE BEGIN 1 */
static void clearUartISr(USART_TypeDef *uart)
{
    if ((LL_USART_IsEnabledIT_IDLE(uart)) && (LL_USART_IsActiveFlag_IDLE(uart)))
    {
        LL_USART_ClearFlag_IDLE(uart);
        if (uart == USART1)
        {
        	UART1_FrameIdle_Callback();
        }
    }

    /* USART Parity Error Detected */
    if ((LL_USART_IsEnabledIT_PE(uart)) && (LL_USART_IsActiveFlag_PE(uart)))
    {
        LL_USART_ClearFlag_PE(uart);
    }

    /* Noise error detected  */
    if (LL_USART_IsActiveFlag_NE(uart))
    {
        LL_USART_ClearFlag_NE(uart);
    }

    /* USART Frame Error Detected */
    if (LL_USART_IsActiveFlag_FE(uart))
    {
        LL_USART_ClearFlag_FE(uart);
    }

    /* USART Transmit Data Register Empty */
    if ((LL_USART_IsEnabledIT_RXNE(uart)) && (LL_USART_IsActiveFlag_RXNE(uart)))
    {
        LL_USART_ClearFlag_RXNE(uart);
    }


    /* USART LIN Break Detection*/
    if ((LL_USART_IsEnabledIT_LBD(uart)) && (LL_USART_IsActiveFlag_LBD(uart)))
    {
    	LL_USART_ClearFlag_LBD(uart);
    }

    /* USART Clear To Send Detection*/
    if ((LL_USART_IsEnabledIT_CTS(uart)) && (LL_USART_IsActiveFlag_nCTS(uart)))
    {
        LL_USART_ClearFlag_nCTS(uart);
    }
}

/* USER CODE END 1 */
