/**
  ******************************************************************************
  * File Name          : USART.h
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */
#define DMA_TX_BUFFER_SIZE 256
#define DMA_RX_BUFFER_SIZE 64
#define SDIN_BUFFER_SIZE 256

typedef enum State{
	BUFFER_idle = 0,
	BUFFER_busy
}BufferStateTypeDef;

typedef struct C_BUFFER_HANDLER{
	uint8_t * BufferStart_p;
	size_t BufferLength;
	size_t BufferStartOffset; 		/* El encargado de cambiar el offset es la interrupción sobre el final de la transmisión*/
	size_t BufferEndOffset;		    /* Es la posición donde se va a escribir el próximo dato */
	size_t ToSend;
	size_t BufferNextStartOffset;
	size_t BufferDataLength;				/* Usado para indicarle al modulo DMA cuantos datos se van a enviar */
	size_t DataSize;
	BufferStateTypeDef BufferState;
}cBufferHandler_t;

typedef enum BUFFER_TYPE{
	dataBufferType,
	sdoutBufferType,
	errorBufferType,
	sdinBufferType
}bufferType_t;

/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void TransmitionStart(cBufferHandler_t*);
HAL_StatusTypeDef sendChar(char ch);
//int __io_putchar(int ch);
//int __io_getchar(void);

void ReceptionStart();
void uartSetReady();
void uartOnReceive();
HAL_UART_StateTypeDef uartReceive();
uint8_t* uartGetData();
uint8_t uartIsReady();
BufferStateTypeDef sdinIsBusy();

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
