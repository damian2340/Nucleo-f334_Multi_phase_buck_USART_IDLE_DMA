/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "hrtim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum PARSE_STATE {
	StateParseSTOPPED = 0x00,
	StateParseIDLE = 0x01,
	StateParseBUSY0 = 0x02,
	StateParseERROR = 0x04,
} parseState_t;

typedef struct PARSE_CONTEXT {
	parseState_t ParseState;

} parseContext_t;

typedef enum COMM_STATE {
	StateCommSTOPPED = 0x00,
	StateCommIDLE = 0x01,
	StateCommBUSY0 = 0x02,
	StateSetDataBuffer = 0x04,
	StateSetSdouBuffer = 0x08,
	StateResetDataBuffer = 0x0F,
	StateResetSdoBuffer = 0x10,
	StateCommERROR = 0x40,
} commState_t;

typedef struct COMM_CONTEXT {
	commState_t CommState;
	cBufferHandler_t * CurrentBuffer;
} commContext_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t posicionTargetX = 0; // valor de la pwm a usar como input (ancho), se usa el valor de numeroRecivido
uint16_t posicionTargetY = 0; // valor de la pwm a usar como input (ancho), se usa el valor de numeroRecivido
uint16_t posicionTargetZ = 0; // valor de la pwm a usar como input (ancho), se usa el valor de numeroRecivido
uint16_t phase = 0;
float frecuency = 1500000.0;

extern uint8_t sdin_buffer[SDIN_BUFFER_SIZE];
extern volatile BufferStateTypeDef SdInBufferState;
extern cBufferHandler_t TxbufferHandler;

extern bufferType_t CurrentBufferType;

extern HRTIM_HandleTypeDef hhrtim1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void parse_task();
void parseInitialize(parseContext_t * parseContext);

void comm_task();
void commInitialize(commContext_t * commContext);

void setpidTargetX(float posicioTarget);
void setpidTargetY(float posicioTarget);
void setpidTargetZ(float posicioTarget);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_HRTIM1_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();

	/* USER CODE BEGIN 2 */
	/* USER CODE END 2 */
	// primero se abre la comunicación antes de entrar al while, allí una vez que se recivió y se seteó la pwm,
	//se vuelve a abrir el receive para que constantemente se pueda mandar datos.
	ReceptionStart();
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {
		comm_task();		// tarea de comunicacion con la pc
		parse_task();		// tarea de control del PWM

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_HRTIM1;
	PeriphClkInit.Hrtim1ClockSelection = RCC_HRTIM1CLK_PLLCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

void parse_task() {
	static parseContext_t parseContext = { .ParseState = StateParseSTOPPED, };

	switch (parseContext.ParseState) {
	case StateParseSTOPPED:
		parseInitialize(&parseContext);

		break;
	case StateParseIDLE:
		if (sdinIsBusy() == BUFFER_busy) {
			parseContext.ParseState = StateParseBUSY0;
		}
		break;
	case StateParseBUSY0:
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

		switch (sdin_buffer[0]) {
		case 'S':
		case 's':
			if (sdin_buffer[2] == 'A' || sdin_buffer[2] == 'a') {
				HAL_HRTIM_SimpleBaseStart(&hhrtim1, HRTIM_TIMERINDEX_MASTER);
				HAL_HRTIM_SimplePWMStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
				HRTIM_OUTPUT_TA1);
				HAL_HRTIM_SimplePWMStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
				HRTIM_OUTPUT_TA2);
				HAL_HRTIM_SimplePWMStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
				HRTIM_OUTPUT_TB1);
				HAL_HRTIM_SimplePWMStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
				HRTIM_OUTPUT_TB2);
				HAL_HRTIM_SimplePWMStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C,
				HRTIM_OUTPUT_TC1);
				HAL_HRTIM_SimplePWMStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C,
				HRTIM_OUTPUT_TC2);
				HAL_HRTIM_SimplePWMStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D,
				HRTIM_OUTPUT_TD1);
				HAL_HRTIM_SimplePWMStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D,
				HRTIM_OUTPUT_TD2);
				HAL_HRTIM_SimplePWMStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E,
				HRTIM_OUTPUT_TE1);
				HAL_HRTIM_SimplePWMStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E,
				HRTIM_OUTPUT_TE2);
			} else if (sdin_buffer[2] == 'O' || sdin_buffer[2] == 'o') {
				HAL_HRTIM_SimpleBaseStop(&hhrtim1, HRTIM_TIMERINDEX_MASTER);
				HAL_HRTIM_SimplePWMStop(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
				HRTIM_OUTPUT_TA1);
				HAL_HRTIM_SimplePWMStop(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
				HRTIM_OUTPUT_TA2);
				HAL_HRTIM_SimplePWMStop(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
				HRTIM_OUTPUT_TB1);
				HAL_HRTIM_SimplePWMStop(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
				HRTIM_OUTPUT_TB2);
				HAL_HRTIM_SimplePWMStop(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C,
				HRTIM_OUTPUT_TC1);
				HAL_HRTIM_SimplePWMStop(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C,
				HRTIM_OUTPUT_TC2);
				HAL_HRTIM_SimplePWMStop(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D,
				HRTIM_OUTPUT_TD1);
				HAL_HRTIM_SimplePWMStop(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D,
				HRTIM_OUTPUT_TD2);
				HAL_HRTIM_SimplePWMStop(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E,
				HRTIM_OUTPUT_TE1);
				HAL_HRTIM_SimplePWMStop(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E,
				HRTIM_OUTPUT_TE2);
			}
			break;
		case 'X':
		case 'x':
			posicionTargetX = atof((char const *) (sdin_buffer + 1));
			__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
					HRTIM_COMPAREUNIT_1, posicionTargetX);
			__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
					HRTIM_COMPAREUNIT_1, posicionTargetX);
			break;
		case 'Y':
		case 'y':
			posicionTargetY = atof((char const *) (sdin_buffer + 1));
			__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C,
					HRTIM_COMPAREUNIT_1, posicionTargetY);
			__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D,
					HRTIM_COMPAREUNIT_1, posicionTargetY);
			break;
		case 'Z':
		case 'z':
			posicionTargetZ = atof((char const *) (sdin_buffer + 1));

			__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E,
					HRTIM_COMPAREUNIT_1, posicionTargetZ);
			break;
		case 'P':
		case 'p':
			phase = atoi((char const *) (sdin_buffer + 1));
			break;
		case 'F':
		case 'f':
			frecuency = atof((char const *) (sdin_buffer + 1));
//			__HAL_HRTIM_SETPERIOD
			break;
		default:
			printf("Comando Desconocido\n\r");
			;
		}

		printf("Receive: %s \n\rPhase %d \n\rX %d \n\r", sdin_buffer, phase, posicionTargetX);
		SdInBufferState = BUFFER_idle;
		parseContext.ParseState = StateParseIDLE;
		break;
	case StateParseERROR:
		printf("\n\n\rParse Error: %s \n\r", sdin_buffer);
		break;

	}

}

void setpidTargetX(float posicioTargetX) {
	/*
	 * Crear codigo para el PID que controla el eje X
	 *
	 * */
}

void setpidTargetY(float posicioTargetY) {
	/*
	 * Crear codigo para el PID que controla el eje Y
	 *
	 * */
}

void setpidTargetZ(float posicioTargetZ) {
	/*
	 * Crear codigo para el PID que controla el eje Z
	 *
	 * */
}

void parseInitialize(parseContext_t * parseContext) {
	parseContext->ParseState = StateParseIDLE;
}

void parseBusy0() {
	if (sdinIsBusy() == BUFFER_busy) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		printf("\n\n\rReceive complete: %s \n\n\r", sdin_buffer);
		SdInBufferState = BUFFER_idle;
	}
}

void comm_task() {
	static commState_t requestState = StateCommIDLE;
	static commContext_t commContext = { .CommState = StateCommSTOPPED, };
	switch (commContext.CommState) {
	case StateCommSTOPPED:
		commInitialize(&commContext);
		break;
	case StateCommIDLE:
		if (commContext.CurrentBuffer->BufferState == BUFFER_idle) {
			commContext.CommState = requestState;
			if (commContext.CommState == StateCommIDLE) {
				if (commContext.CurrentBuffer->BufferDataLength > 0) {
					TransmitionStart(commContext.CurrentBuffer);
				}
			} else {
				requestState = StateCommIDLE;
			}
		}
		break;
	case StateCommBUSY0:
		break;
	case StateSetDataBuffer:
		if (commContext.CurrentBuffer->BufferState == BUFFER_idle) {
			commContext.CommState = StateCommIDLE;
		}
		break;
	case StateSetSdouBuffer:
		if (commContext.CurrentBuffer->BufferState == BUFFER_idle) {
			commContext.CurrentBuffer = &TxbufferHandler;
			CurrentBufferType = sdoutBufferType;
			commContext.CommState = StateCommIDLE;
		}
		break;
	case StateResetDataBuffer:
		if (commContext.CurrentBuffer->BufferState == BUFFER_idle) {
			commContext.CommState = StateCommIDLE;
		}
		break;
	case StateResetSdoBuffer:
		if (commContext.CurrentBuffer->BufferState == BUFFER_idle) {
			commContext.CommState = StateCommIDLE;
		}
		break;
	case StateCommERROR:
		break;
	}

}

void commInitialize(commContext_t * commContext) {
	commContext->CommState = StateCommIDLE;
	commContext->CurrentBuffer = &TxbufferHandler;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(char *file, uint32_t line) {
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
