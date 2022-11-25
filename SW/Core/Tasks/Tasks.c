/*
 * Tasks.c
 *
 *  Created on: Nov 25, 2022
 *      Author: Robby
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "main.h"
#include "cmsis_os.h"

#define DMA_BUF_SIZE	32

static volatile bool rxCompl = false;
static volatile bool rxErr = false;
static volatile bool txCompl = false;
static volatile bool txErr = false;

static volatile uint8_t txBufCpy[DMA_BUF_SIZE] = {0};
static volatile uint8_t txBuffer[DMA_BUF_SIZE] = {0};
static volatile uint8_t rxBuffer[DMA_BUF_SIZE] = {0};

static void resetDmaTx(void);
static void resetDmaRx(void);
static void StartTransfer(void);


void Task_Init(void)
{
	memset((void *)&rxBuffer, 0, sizeof(rxBuffer));
	memset((void *)&txBuffer, 0, sizeof(txBuffer));
	memset((void *)&txBufCpy, 0, sizeof(txBufCpy));

	  LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_2, LL_USART_DMA_GetRegAddr(USART1), (uint32_t)rxBuffer, LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_2));
	  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, sizeof(rxBuffer));
	  LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_7, (uint32_t)txBuffer, LL_USART_DMA_GetRegAddr(USART1), LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_7));
	  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, sizeof(txBuffer));
}

void Task_RunRx(void *arg)
{
	for (;;) {
		if (rxCompl == true) {
			rxCompl = false;
			resetDmaRx();
			if ((txBufCpy[0] == 0x20) && (txBufCpy[1] == 0x40)) {
				/* Placeholder. Parsing of frame will go here */
				LL_GPIO_SetOutputPin(GPIOB, DEBUG_2_Pin);
				LL_GPIO_ResetOutputPin(GPIOB, DEBUG_2_Pin);
			}
			StartTransfer();
		}

		if (txCompl == true) {
			txCompl = false;
			/* Disable DMA1 Tx Channel */
			LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_7);

			resetDmaTx();
		}
	}
}

void Task_RunDefault(void *arg)
{
	  for(;;)
	  {
	    osDelay(1);
	  }
}

void Task_RunEncoder(void *arg)
{
	  for(;;)
	  {
	    osDelay(1);
	  }
}

void Task_RunHeartbeat(void *arg)
{
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
	/* Infinite loop */
	for (;;) {
		LL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		osDelay(500);
	}
}

void Task_RunMotorControl(void *arg)
{
	  for(;;)
	  {
	    osDelay(1);
	  }
}









static void resetDmaRx(void)
{
	if (LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_2))
	{
		/* Reset DMA */
		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_2);
	}
	memset((void *)&rxBuffer, 0, sizeof(rxBuffer));
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_2, LL_USART_DMA_GetRegAddr(USART1), (uint32_t)rxBuffer, LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_2));
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, sizeof(rxBuffer));
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
}





void DMA2_RxError_Callback(void)
{
	rxErr = true;
}

void DMA2_TxError_Callback(void)
{
	txErr = true;
}

void DMA2_TxComplete_Callback(void)
{
	txCompl = true;
}

void DMA2_RxComplete_Callback(void)
{
	memset((void *)&txBufCpy, 0, sizeof(txBufCpy));
	memcpy((void *)&txBufCpy[0], (void *)&rxBuffer[0], sizeof(rxBuffer));
	memcpy((void *)&txBuffer[0], (void *)&rxBuffer[0], sizeof(rxBuffer));
}

static void resetDmaTx(void)
{
	if (LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_7))
	{
		/* Reset DMA */
		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_7);
	}
	memset((void *)&txBuffer, 0, sizeof(txBuffer));
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_7, (uint32_t)txBuffer, LL_USART_DMA_GetRegAddr(USART1), LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_7));
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, sizeof(txBuffer));
	//LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7);
}



void UART1_FrameIdle_Callback(void)
{
	LL_GPIO_SetOutputPin(GPIOB, DEBUG_1_Pin);
	LL_GPIO_ResetOutputPin(GPIOB, DEBUG_1_Pin);
	rxCompl = true;
}

static void StartTransfer(void)
{
	txCompl = false;
	txErr = false;

	/* Enable DMA TX Interrupt */
	LL_USART_EnableDMAReq_TX(USART1);

	/* Enable DMA Channel Rx */
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7);
}
