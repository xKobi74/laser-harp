/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CD74HC4067.h"
#include "ssd1306.h"
#include "audio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DISPLAY_OCTAVE_MODE 0
#define DISPLAY_VOLUME_MODE 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern Multiplexer* multiplexer;
extern ADC_HandleTypeDef hadc1;
extern unsigned char states[7];


uint32_t clock_TIM4 = 0;

uint8_t display_mode = DISPLAY_VOLUME_MODE;

GPIO_PinState up_button;
GPIO_PinState down_button;
extern uint32_t edge[7];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void update_state();

void update_display();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
/* USER CODE BEGIN EV */
extern uint32_t clock;

//from "audio.c'
extern unsigned char octave_number;
extern unsigned char volume;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
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
  * @brief This function handles Prefetch fault, memory access fault.
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
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
	update_state();
	
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
	++clock;
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
	
	up_button = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
	down_button = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
	
	if (up_button == GPIO_PIN_RESET && down_button == GPIO_PIN_RESET) {
		++clock_TIM4;
		if (clock_TIM4 % 10 == 0) {
			display_mode = !display_mode;
			clock_TIM4 = 0;
			update_display();
		}
	}
	
	else if (up_button == GPIO_PIN_RESET) {
		++clock_TIM4;
		if (clock_TIM4 % 10 == 0) {
			if (display_mode == DISPLAY_VOLUME_MODE) {
				change_volume(CHANGE_PARAMETER_UP);
			}
			else {
				change_octave(CHANGE_PARAMETER_UP);
			}
			update_display();
		}
	}
	
	else if (down_button == GPIO_PIN_RESET) {
		++clock_TIM4;
		if (clock_TIM4 % 10 == 0) {
			if (display_mode == DISPLAY_VOLUME_MODE) {
				change_volume(CHANGE_PARAMETER_DOWN);
			}
			else {
				change_octave(CHANGE_PARAMETER_DOWN);
			}
			update_display();
		}
	}
	
	else {
		clock_TIM4 = 0;
	}
  /* USER CODE END TIM4_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void update_state() {

	uint32_t channel, i, value, edgei;
	for (channel = 9, i = 0; channel < 16; ++channel, ++i) {
		setMultiplexerChannel(multiplexer, channel);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 10);
		value = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		edgei = edge[i];
		if (value > edgei) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			++(states[i]);
		}
	}
}

void update_display() {
	/*
	ssd1306_Fill(Black);
	ssd1306_SetCursor(5, 10);
	ssd1306_WriteString("AZA LOX", Font_11x18, White);
	ssd1306_UpdateScreen();
	*/
	ssd1306_Fill(Black);
	ssd1306_SetCursor(5, 10);
	
	if (display_mode == DISPLAY_VOLUME_MODE) {
		switch (volume) {
			case 0: {
				ssd1306_WriteString("Volume:0%", Font_11x18, White);
				break;
			}
			case 10: {
				ssd1306_WriteString("Volume:10%", Font_11x18, White);
				break;
			}
			case 20: {
				ssd1306_WriteString("Volume:20%", Font_11x18, White);
				break;
			}
			case 30: {
				ssd1306_WriteString("Volume:30%", Font_11x18, White);
				break;
			}
			case 40: {
				ssd1306_WriteString("Volume:40%", Font_11x18, White);
				break;
			}
			case 50: {
				ssd1306_WriteString("Volume:50%", Font_11x18, White);
				break;
			}
			case 60: {
				ssd1306_WriteString("Volume:60%", Font_11x18, White);
				break;
			}
			case 70: {
				ssd1306_WriteString("Volume:70%", Font_11x18, White);
				break;
			}
			case 80: {
				ssd1306_WriteString("Volume:80%", Font_11x18, White);
				break;
			}
			case 90: {
				ssd1306_WriteString("Volume:90%", Font_11x18, White);
				break;
			}
			case 100: {
				ssd1306_WriteString("Volume:100%", Font_11x18, White);
				break;
			}
		}
	}
	
	else if (display_mode == DISPLAY_OCTAVE_MODE) {
		switch (octave_number) {
			case 1: {
				ssd1306_WriteString("Octave: 1st", Font_11x18, White);
				break;
			}
			case 2: {
				ssd1306_WriteString("Octave: 2nd", Font_11x18, White);
				break;
			}
			case 3: {
				ssd1306_WriteString("Octave: 3rd", Font_11x18, White);
				break;
			}
			case 4: {
				ssd1306_WriteString("Octave: 4th", Font_11x18, White);
				break;
			}
			case 5: {
				ssd1306_WriteString("Octave: 5th", Font_11x18, White);
				break;
			}
		}
	}
	
	ssd1306_UpdateScreen();

}
/* USER CODE END 1 */
