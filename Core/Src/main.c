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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "bme280.h"
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IDLE_STATE				0		//startup not pre-charged
#define DRIVE_STATE				1		//post charged DSC and CHG EN
#define CHARGE_STATE			2		//DSC disabled, CHARGE_DETECTED true
#define FAULT_STATE				3		//DSC CH disabled

#define DRIVE_GOOD				1
#define DRIVE_FAULT				254
#define CHG_GOOD				2
#define CHG_COMPLETE			3
#define CHG_FAULT				255

#define RELAY_HIGH 				GPIO_PIN_15
#define RELAY_LOW 				GPIO_PIN_14
#define RELAY_PRECHARGE 		GPIO_PIN_12
#define RELAY_CHARGE 			GPIO_PIN_11
#define LED_PIN 				GPIO_PIN_13
#define RELAY_BUS 				GPIOB
#define LED_BUS 				GPIOC

//input pins from BMS and Antisafe
#define ANTISAFE_PIN			GPIO_PIN_3 //antisafe button PB3 pulled up
#define CH_EN 					GPIO_PIN_5 //charge enable pin PB5
#define DSC_EN 					GPIO_PIN_6 //discharge enable pin PB6
#define SP_EN 					GPIO_PIN_7 //spare enable pin PB7
#define EN_INPUT_BUS 			GPIOB

#define HEATER_EN				GPIO_PIN_1
#define MOTOR_FAN_EN			GPIO_PIN_2
#define SPARE_HV_EN				GPIO_PIN_3
#define MOTOR_FAN_EN_BUS		GPIOA

#define PRECHARGE_DURATION 		5000
#define RELAY_DELAY 			1000

#define BME280_SPI_CS_PIN   	GPIO_PIN_8 // PA8
#define BME280_SPI_CS_PORT  	GPIOA      //

#define ADE7912_SPI_CS_PIN  	GPIO_PIN_4 // PA4
#define ADE7912_SPI_CS_PORT   	GPIOA      //

#define RS485DR_PIN				GPIO_PIN_4 //pb4
#define RS485DR_PORT	 		GPIOB
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

//uint8_t OUTPUT_BOOL = 0;
typedef struct {
    bool chargeEnabled;
    bool dischargeEnabled;
    bool chargerDetected;
    bool antisafeEnabled;

    bool relayHigh;
    bool relayLow;
    bool relayPC;
    bool relayCHG;
} InputData;

uint8_t currentState = IDLE_STATE;
uint8_t status = 0;
uint8_t prechargeFlag = 0;
uint8_t relay_sequence = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

// Read PMB input states and create boolean struct plus debounce
InputData readInputs(void) {
    InputData data;
    // Read for chargeEnabled
    if(HAL_GPIO_ReadPin(EN_INPUT_BUS, CH_EN) == false) { //if PB5 CH_EN low
			data.chargeEnabled = true;
	}
	else{
		data.chargeEnabled = false;
	}

    // Read for dischargeEnabled
    if(HAL_GPIO_ReadPin(EN_INPUT_BUS, DSC_EN) == false) { //if PB6 DSC_EN low
			data.dischargeEnabled = true;
	}
	else{
		data.dischargeEnabled = false;
	}

    // Read for chargerDetected
    if(HAL_GPIO_ReadPin(EN_INPUT_BUS, SP_EN) == false) { //if SP_EN PB7 is low  (charger detected)
            data.chargerDetected = true;
    }
    else{
    	data.chargerDetected = false;
    }

    // Read for antisafeEnabled
    if(HAL_GPIO_ReadPin(EN_INPUT_BUS, ANTISAFE_PIN) == false){ //if PB3 antisafe is low
        data.antisafeEnabled = true;
	}
	else{
		data.antisafeEnabled = false;
	}

    // Read Relays
    data.relayHigh = HAL_GPIO_ReadPin(RELAY_BUS, RELAY_HIGH); //output state relay high
    data.relayLow = HAL_GPIO_ReadPin(RELAY_BUS, RELAY_LOW); //output state relay low
    data.relayPC = HAL_GPIO_ReadPin(RELAY_BUS, RELAY_PRECHARGE); //output state relay pre-charge
    data.relayCHG = HAL_GPIO_ReadPin(RELAY_BUS, RELAY_CHARGE); //output state relay charge

    return data;
}

void allRelayOff(void){
	HAL_GPIO_WritePin(RELAY_BUS, RELAY_LOW, RESET); //turn off negative relay
	HAL_GPIO_WritePin(RELAY_BUS, RELAY_HIGH, RESET); //turn off positive relay
	HAL_GPIO_WritePin(RELAY_BUS, RELAY_CHARGE, RESET); //turn off charge relay
	HAL_GPIO_WritePin(RELAY_BUS, RELAY_PRECHARGE, RESET); //turn off precharge relay
//	HAL_GPIO_WritePin(RELAY_BUS, GPIO_PIN_0, RESET); //
//	HAL_GPIO_WritePin(RELAY_BUS, GPIO_PIN_1, RESET); //
//	HAL_GPIO_WritePin(RELAY_BUS, GPIO_PIN_10, RESET); //
//	HAL_GPIO_WritePin(RELAY_BUS, GPIO_PIN_11, RESET); //
}

void allRelayOn(void){
	HAL_GPIO_WritePin(RELAY_BUS, RELAY_LOW, SET); //turn off negative relay
	HAL_GPIO_WritePin(RELAY_BUS, RELAY_HIGH, SET); //turn off positive relay
	HAL_GPIO_WritePin(RELAY_BUS, RELAY_CHARGE, SET); //turn off charge relay
	HAL_GPIO_WritePin(RELAY_BUS, RELAY_PRECHARGE, SET); //turn off precharge relay
//	HAL_GPIO_WritePin(RELAY_BUS, GPIO_PIN_0, SET); //
//	HAL_GPIO_WritePin(RELAY_BUS, GPIO_PIN_1, SET); //
//	HAL_GPIO_WritePin(RELAY_BUS, GPIO_PIN_10, SET); //
//	HAL_GPIO_WritePin(RELAY_BUS, GPIO_PIN_11, SET); //
}



void sendUSB_BMS_state(void){
	InputData data = readInputs();
	uint8_t buffer[128] = {0}; // Initialize to zeros
	int length = snprintf((char *)buffer, sizeof(buffer),
						  "CH_EN: %s, "
						  "DSC_EN: %s, "
						  "CH_DET: %s, "
						  "ASAFE: %s, "
						  "Mode: %d, "
						  "Status: %d\r\n",
						  data.chargeEnabled ? "true" : "false",
						  data.dischargeEnabled ? "true" : "false",
						  data.chargerDetected ? "true" : "false",
						  data.antisafeEnabled ? "true" : "false",
						  currentState, status);  // Adding the status variable here

	CDC_Transmit_FS(buffer, length);  // Use length here
}

void sendUSB_Relay_State(void){
	InputData data = readInputs();
	uint8_t buffer[128] = {0}; // Initialize to zeros
	int length = snprintf((char *)buffer, sizeof(buffer),
						  "RELAY OUTPUT| POS: %s, "
						  "NEG: %s, "
						  "PreCharge: %s, "
						  "CHARGE: %s\r\n",
						  data.relayHigh ? "EN" : "DIS",
						  data.relayLow ? "EN" : "DIS",
						  data.relayPC ? "EN" : "DIS",
						  data.relayCHG ? "EN" : "DIS"
						  );

	CDC_Transmit_FS(buffer, length);  // Use length here
}

void serialPrintln(const char *str) {
    uint8_t buffer[128];  // Make sure this buffer is large enough for your messages
    int length = snprintf((char *)buffer, sizeof(buffer), "%s\n", str);

    if (length > 0) {
        CDC_Transmit_FS(buffer, length);
    }
}
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //0.1s loop 10Hz
	  static int i = 0;
	  i = 0;

	  while(currentState == IDLE_STATE && i < 10){
		  HAL_Delay(100);
		  InputData data = readInputs();
		  if(data.chargeEnabled && data.dischargeEnabled && !data.chargerDetected){
			  currentState = DRIVE_STATE;
			  serialPrintln("Drive STATE");
			  relay_sequence = 1; //begin startup sequence
			  i = 0;
			  break;
		  }
		  if(data.chargerDetected && data.chargeEnabled){
			  currentState = CHARGE_STATE;
			  serialPrintln("Charge STATE");
			  relay_sequence = 1; //begin startup sequence
			  i = 0;
			  break;
		  }
		  else{
			  currentState = IDLE_STATE;
			  serialPrintln("IDLE STATE");
			  i = 0;
		  }
		  i++;

	  }
	  if(currentState == DRIVE_STATE && relay_sequence == 1){
		  InputData data = readInputs();
		  if(data.chargeEnabled && data.dischargeEnabled){
			  HAL_GPIO_WritePin(RELAY_BUS, RELAY_LOW, SET); //turn on lower relay
			  serialPrintln("NEG Contact Engaged");
			  sendUSB_Relay_State();
			  HAL_Delay(100);
		  }
		  else{
			  allRelayOff();
			  status = DRIVE_FAULT;
			  //break;
		  }
		  data = readInputs();
		  if(data.chargeEnabled && data.dischargeEnabled){
			  HAL_GPIO_WritePin(RELAY_BUS, RELAY_PRECHARGE, SET); //turn on Precharge
			  serialPrintln("Precharging");
			  sendUSB_Relay_State();
			  HAL_Delay(100);
		  }
		  else{
			  allRelayOff();
			  status = DRIVE_FAULT;
			  break;
		  }
		  data = readInputs();
		  if(data.chargeEnabled && data.dischargeEnabled){
			  HAL_Delay(PRECHARGE_DURATION); //wait for precharge
			  serialPrintln("Wait 5000ms");
			  sendUSB_Relay_State();
		  }
		  else{
			  allRelayOff();
			  status = DRIVE_FAULT;
			  break;
		  }
		  data = readInputs();
		  if(data.chargeEnabled && data.dischargeEnabled){
			  HAL_GPIO_WritePin(RELAY_BUS, RELAY_HIGH, SET); //turn on POS contactor
			  HAL_Delay(100);
			  HAL_GPIO_WritePin(RELAY_BUS, RELAY_PRECHARGE, RESET); //turn off Precharge
			  serialPrintln("POS Contact Engaged");
			  sendUSB_Relay_State();
			  HAL_Delay(100);
		  }
		  else{
			  allRelayOff();
			  status = DRIVE_FAULT;
			  break;
		  }
		  data = readInputs();
		  if(data.chargeEnabled && data.dischargeEnabled){
			  serialPrintln("Startup Sequence Complete");
			  sendUSB_Relay_State();
			  HAL_Delay(100);
		  }
		  else{
			  allRelayOff();
			  status = DRIVE_FAULT;
			  break;
		  }
		  relay_sequence = 0;
	  }

	  if(currentState == CHARGE_STATE && relay_sequence == 1){
	  		  InputData data = readInputs();
	  		  if(data.chargeEnabled && data.chargerDetected){
	  			  HAL_GPIO_WritePin(RELAY_BUS, RELAY_LOW, SET); //turn on lower relay
	  			  serialPrintln("NEG Contact Engaged");
	  			  sendUSB_Relay_State();
	  			  HAL_Delay(100);
	  		  }
	  		  else{
	  			  allRelayOff();
	  			  status = CHG_FAULT;
	  			  break;
	  		  }

	  		  data = readInputs();
	  		  if(data.chargeEnabled && data.chargerDetected){
	  			  HAL_GPIO_WritePin(RELAY_BUS, RELAY_CHARGE, SET); //turn on charge relay
	  			  serialPrintln("CHG Relay Engaged");
	  			  sendUSB_Relay_State();
	  			  HAL_Delay(100);
	  		  }
	  		  else{
	  			  allRelayOff();
	  			  status = CHG_FAULT;
	  			  break;
	  		  }

	  		  data = readInputs();
			  if(data.chargeEnabled && data.chargerDetected){
				  serialPrintln("Charging Sequence Complete");
				  sendUSB_Relay_State();
				  HAL_Delay(100);
			  }
			  else{
				  allRelayOff();
				  status = CHG_FAULT;
				  break;
			  }

	  		  relay_sequence = 0;
	  }
	  InputData data = readInputs();

	  //currentState = updateState();
	  if(currentState == DRIVE_STATE){
		  if(!(data.chargeEnabled && data.dischargeEnabled)){
			  allRelayOff();
			  status = DRIVE_FAULT;
			  serialPrintln("Drive Fault");
		  }
		  else{
			  status = DRIVE_GOOD;
		  }
	  }

	  if(currentState == CHARGE_STATE){
		  if(!data.chargeEnabled){ //charge disabled by BMS
			  allRelayOff(); //shutdown
			  status = CHG_FAULT;
			  serialPrintln("CHG Fault");
		  }
		  if(!data.chargerDetected && data.chargeEnabled){ //done charging or unplugged but not fault
			  allRelayOff(); //turn off all relays
			  status = CHG_COMPLETE;
		  }
		  else{
			  status = CHG_GOOD;
		  }
	  }

	  if(currentState == CHG_COMPLETE){
		  currentState = IDLE_STATE; //back to startup
	  }

	  if(status == CHG_FAULT || status == DRIVE_FAULT){ //if in fault state
		  allRelayOff();
		  HAL_Delay(5000); //wait 5000ms
		  InputData data = readInputs(); //read the BMS states
		  if(data.chargeEnabled && data.dischargeEnabled){ //if everything is good
			  currentState = IDLE_STATE; //back to startup
			  status = DRIVE_GOOD;
		  }

	  }

	  if(i%10 == 0){
		  sendUSB_BMS_state();
		  sendUSB_Relay_State();
	  }
	  else{

	  }

	  i++;

	  HAL_Delay(100);
	  HAL_GPIO_TogglePin(LED_BUS, LED_PIN);


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : PA1 PA2 PA3 PA4
                           PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11
                           PB12 PB13 PB14 PB15
                           PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
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
