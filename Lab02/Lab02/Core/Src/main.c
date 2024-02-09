/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

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
void SystemClock_Config(void);
void EXTI0_1_IRQHandler(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init(); // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config(); //Configure the system clock

	__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
	// Set up a configuration struct to pass to the initialization function
	GPIO_InitTypeDef initStr = {GPIO_PIN_6 | GPIO_PIN_7| GPIO_PIN_8 | GPIO_PIN_9,
	GPIO_MODE_OUTPUT_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC6,7,8 & PC9
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); // Start PC9 high (green)
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7); //toggle blue on
	
	//*****Configuring the EXTI*****************//
	// Setting PA0 to interrupt -- low speed & internal pulldown enabled
	GPIO_InitTypeDef initStrPA0 = {GPIO_PIN_0,
	GPIO_SPEED_FREQ_LOW,
	GPIO_PULLDOWN};
	//PA0 connected to EXTI0 --  SYSCFG_EXTICR1
	SYSCFG->EXTICR[0] &= ~(1<<0); //sets 0th bit = 0
	SYSCFG->EXTICR[0] &= ~(1<<1); //sets 1th bit = 0
	SYSCFG->EXTICR[0] &= ~(1<<2); //sets 2nd bit = 0
	SYSCFG->EXTICR[0] &= ~(1<<3); //sets 3rd bit = 0
	// unmask interrupt
	EXTI->IMR |= 0b1;
	// enable rising-edge trigger
	EXTI->RTSR |= 0b1;
	
	//*****Setting the SYSCFG Pin Multiplexer*****************//
	//Use the RCC to enable the peripheral clock to the SYSCFG peripheral
	RCC->APB2ENR |= 0b1; 
	//The SYSCFG multiplexer that can route PA0 to the EXTI peripheral: SYSCFG_EXTICR1
	
	//*****Enable and Set Priority of the EXTI Interrupt*****************//
	//Enable the selected EXTI interrupt that references line 0
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	//Set the priority for the interrupt to 1 (or 3)
	NVIC_SetPriority(EXTI0_1_IRQn, 1);
	// Set priority for Systick to 2
	NVIC_SetPriority(SysTick_IRQn, 2);
	
	while (1) {
	HAL_Delay(400); // Delay 400ms
	// Toggle red 
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
	}
}

/**
* EXTI Interrupt Handler
*/
void EXTI0_1_IRQHandler(){
	//toggle green and orange
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9 | GPIO_PIN_8); 
	// Delay loop of roughly 1-2 seconds
	for (volatile int i = 0; i < 1500000; i++){}
	//toggle green and orange again
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9 | GPIO_PIN_8); 
	// clear pending register
	EXTI->PR |= 0b1;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
