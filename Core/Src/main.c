/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RELAY_HIGH GPIO_PIN_15
#define RELAY_LOW GPIO_PIN_14
#define RELAY_PRECHARGE GPIO_PIN_12
#define RELAY_CHARGE_EN GPIO_PIN_13
#define LED_PIN GPIO_PIN_13
#define RELAY_BUS GPIOB
#define LED_BUS GPIOC


#define PRECHARGE_DURATION 5000
#define RELAY_DELAY 1000
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t OUTPUT_BOOL = 0;
RELAY_SET_FLAG = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
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
  /* USER CODE BEGIN 2 */
	HAL_Delay(500);
	HAL_GPIO_WritePin(LED_BUS, LED_PIN, SET); //turn on LED

	HAL_GPIO_WritePin(RELAY_BUS, RELAY_LOW, SET); //turn on lower relay
	HAL_Delay(RELAY_DELAY); //wait


	HAL_GPIO_WritePin(RELAY_BUS, RELAY_PRECHARGE, SET); //turn on precharge
	HAL_Delay(PRECHARGE_DURATION); //wait for precharge duration

	HAL_GPIO_WritePin(RELAY_BUS, RELAY_HIGH, SET); //turn on upper relay
	HAL_Delay(500); //wait a bit
	HAL_GPIO_WritePin(RELAY_BUS, RELAY_PRECHARGE, RESET); //turn off precharge
	HAL_Delay(1000);
	HAL_GPIO_WritePin(RELAY_BUS, RELAY_CHARGE_EN, SET); //turn on charger

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(1000);
	  HAL_GPIO_TogglePin(LED_BUS, LED_PIN);
//	  if(OUTPUT_BOOL == 0 && RELAY_SET_FLAG == 0){//if output is disabled
//		  HAL_GPIO_WritePin(LED_BUS, LED_PIN, RESET); //turn off LED
//		  HAL_GPIO_WritePin(RELAY_BUS, RELAY_HIGH, RESET); //turn off high relay
//		  HAL_GPIO_WritePin(RELAY_BUS, RELAY_LOW, RESET); //turn off low relay
//		  HAL_GPIO_WritePin(RELAY_BUS, RELAY_CHARGE_EN, RESET); //turn off charge en
//		  HAL_GPIO_WritePin(RELAY_BUS, RELAY_PRECHARGE, RESET); //turn off precharge
//		  RELAY_SET_FLAG = 1;
//	  }
//	  else if(OUTPUT_BOOL == 1 && RELAY_SET_FLAG == 0){ //if output is enabled
//		  if(OUTPUT_BOOL == 1){
//			  HAL_GPIO_WritePin(LED_BUS, LED_PIN, SET); //turn on LED
//		  }
//		  if(OUTPUT_BOOL == 1){
//			  HAL_Delay(RELAY_DELAY);
//		  }
//		  if(OUTPUT_BOOL == 1){
//			  HAL_GPIO_WritePin(RELAY_BUS, RELAY_LOW, SET); //turn on lower relay
//		  }
//		  if(OUTPUT_BOOL == 1){
//			  HAL_Delay(RELAY_DELAY); //wait
//		  }
//		  if(OUTPUT_BOOL == 1){
//			  HAL_GPIO_WritePin(RELAY_BUS, RELAY_PRECHARGE, SET); //turn on precharge
//		  }
//		  if(OUTPUT_BOOL == 1){
//			  HAL_Delay(PRECHARGE_DURATION); //wait for precharge duration
//		  }
//		  if(OUTPUT_BOOL == 1){
//			  HAL_GPIO_WritePin(RELAY_BUS, RELAY_HIGH, SET); //turn on upper relay
//		  }
//		  if(OUTPUT_BOOL == 1){
//			  HAL_GPIO_WritePin(RELAY_BUS, RELAY_PRECHARGE, RESET); //turn off precharge
//		  }
//		  RELAY_SET_FLAG = 1;
//	  }
//	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == RESET && OUTPUT_BOOL == 0){ //if PIN A0 is low and output is off
//	      HAL_Delay(100);
//		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == RESET && OUTPUT_BOOL == 0){ //debounce
//			  OUTPUT_BOOL = 1; //set output enabled
//			  RELAY_SET_FLAG = 0;
//	      }
//
//	  }
//	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == SET && OUTPUT_BOOL == 0){ //if PIN A0 is high
//		  HAL_Delay(20);
//		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == SET && OUTPUT_BOOL == 0){ //debounce
//			  HAL_GPIO_WritePin(RELAY_BUS, RELAY_HIGH, RESET); //turn off high relay
//			  HAL_GPIO_WritePin(RELAY_BUS, RELAY_LOW, RESET); //turn off low relay
//			  HAL_GPIO_WritePin(RELAY_BUS, RELAY_CHARGE_EN, RESET); //turn off charge en
//			  HAL_GPIO_WritePin(RELAY_BUS, RELAY_PRECHARGE, RESET); //turn off precharge
//			  OUTPUT_BOOL = 0;
//		  }
//	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_0) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); //turn off led
    HAL_GPIO_WritePin(RELAY_BUS, RELAY_HIGH, RESET); //turn off high relay
    HAL_GPIO_WritePin(RELAY_BUS, RELAY_LOW, RESET); //turn off low relay
   	HAL_GPIO_WritePin(RELAY_BUS, RELAY_CHARGE_EN, RESET); //turn off charge en
    HAL_GPIO_WritePin(RELAY_BUS, RELAY_PRECHARGE, RESET); //turn off precharge
    OUTPUT_BOOL = 0; //disable relays
    RELAY_SET_FLAG = 0;
  } else {
      __NOP();
  }
}
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
