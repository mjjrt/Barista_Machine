/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include <string.h>

#include "pin_config.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum state_t{
	IDLE, // 0
	ON,
	ONE_CUP,
	TWO_CUPS,
	HEATING,
	FEHLER,
}STATE; // global state literal

typedef enum ADC_Pin{
	TEMP = 14,
	TANK = 15,
}ADC_Pin; // ADC Pin literals

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TRANSMIT_TIMEOUT 0xFF
#define DEBOUNCE_DELAY 40
#define BUFFER_SIZE 64			// message buffer size

#define TEMP_HOT 3000
#define WATER_LEVEL_LOW 1000

#define CUP_1_TIME 5
#define CUP_2_TIME 10


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

static int global_state = IDLE;			// initialize global state to IDLE (0)

static uint8_t buffer[BUFFER_SIZE];		// global message buffer

static uint16_t timer_val; 				// global timer variable
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */

void set_global_state(STATE s);
void ADC_Select_CH14();
void ADC_Select_CH15();
uint16_t get_analog_state(ADC_Pin adc);
uint16_t get_temp();
uint16_t get_water_level();
void transmit_sensors(uint16_t* vals, uint8_t* msg);
void transmit_state(STATE s, uint8_t* msg);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == ON_OFF_KEY)
	{
		if(global_state == IDLE)
		{
			set_global_state(ON);
		}else{
			set_global_state(IDLE);
			HAL_GPIO_WritePin(GPIOB, LED_CONTROL, 0);
			HAL_GPIO_WritePin(GPIOB, HEATER_CONTROL, 0);
			HAL_GPIO_WritePin(GPIOB, PUMP_CONTROL, 0);
		}

	}
}
*/

void set_global_state(STATE s)
{
	/* Only touch global_state with this function */
	global_state = s;
	transmit_state(s, buffer);
	return;
}

void ADC_Select_CH14()
{
	/* Select ADC Channel for readout */
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_14;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
}

void ADC_Select_CH15()
{
	/* Select ADC Channel for readout */
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
}


uint16_t get_analog_state(ADC_Pin adc)
{
	/* get the analog value (Temperature or Fill Level) from ADCs */
	uint16_t value = 0;

	switch(adc){
		case TEMP:
			ADC_Select_CH14();
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 1000);

			value = HAL_ADC_GetValue(&hadc1);

			HAL_ADC_Stop(&hadc1);
			break;

		case TANK:
			ADC_Select_CH15();
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 1000);

			value = HAL_ADC_GetValue(&hadc1);

			HAL_ADC_Stop(&hadc1);
			break;

		default:
			break;
	}

	return value;
}


uint16_t get_temp()
{
	/* get temperature */
	return get_analog_state(TEMP);
}

uint16_t get_water_level()
{
	/* get fill level */
	return get_analog_state(TANK);
}

void transmit_sensors(uint16_t* vals, uint8_t* msg)
{
	/* Transmit the sensor values */
	 vals[0] = get_temp();
	 vals[1] = get_water_level();

	 sprintf((char*)msg, "%d,%d\r\n", vals[0], vals[1]);

	 HAL_UART_Transmit(&huart2, msg, 64, 0xFF);

	 return;
}

void transmit_state(STATE s, uint8_t* msg)
{
	/* Transmit the global state */
	switch(global_state){
		case IDLE:
			sprintf((char*)msg, "State: IDLE\r\n");
			break;
		case ON:
			sprintf((char*)msg, "State: ON\r\n");
			break;
		case ONE_CUP:
			sprintf((char*)msg, "State: ONE_CUP\r\n");
			break;
		case TWO_CUPS:
			sprintf((char*)msg, "State: TWO_CUPS\r\n");
			break;
		case HEATING:
			sprintf((char*)msg, "State: HEATING\r\n");
			break;
		case FEHLER:
			sprintf((char*)msg, "State: FEHLER\r\n");
			break;
		default:
			break;
	}

	HAL_UART_Transmit(&huart2, msg, 64, 0xFF);

	for(int i = 0; i != BUFFER_SIZE; ++i)
	{
		// reset the message buffer
		buffer[i] = '\r';
	}

	return;
}

void on_startup()
{
	/* Message transmitted on startup */
	char* startup_message = "****************************\r\n" \
							"\tBARISTA PROJECT\t\r\n" \
							"\t J. Hoersch\r\n\t W. Klaffke\r\n" \
							"\t PRM WS 2022\r\n****************************\r\n";

	HAL_UART_Transmit(&huart2, (uint8_t*)startup_message, strlen(startup_message), TRANSMIT_TIMEOUT);

	return;
}


void get_buttons(void)
{
	/* Read States of Power, CUP_1 and CUP_2 Keys */
	if(HAL_GPIO_ReadPin(GPIOA, ON_OFF_KEY) != 0)
	{

		if(global_state == IDLE)
		{
			// Turn ON if Power Button is pressed
			set_global_state(ON);
			HAL_GPIO_WritePin(GPIOB, LED_CONTROL, 1);

			on_startup();

		}else if(global_state == ON){
			// Turn OFF if Power Button is pressed in state ON
			set_global_state(IDLE);
			HAL_GPIO_WritePin(GPIOB, LED_CONTROL, 0);
		}

		while(HAL_GPIO_ReadPin(GPIOA, ON_OFF_KEY) != 0) // Debouncing
		{
			HAL_Delay(DEBOUNCE_DELAY);
		}
	}

	if(HAL_GPIO_ReadPin(GPIOA, CUP_1_KEY) != 0 && global_state == ON)
	{
		/* Can only be activated if global_state is ON */
		set_global_state(ONE_CUP);

		while(HAL_GPIO_ReadPin(GPIOA, CUP_1_KEY) != 0) // Debouncing
		{
			HAL_Delay(DEBOUNCE_DELAY);
		}
	}

	if(HAL_GPIO_ReadPin(GPIOA, CUP_2_KEY) != 0 && global_state == ON)
	{
		/* Can only be activated if global_state is ON */
		set_global_state(TWO_CUPS);

		while(HAL_GPIO_ReadPin(GPIOA, CUP_2_KEY) != 0) // Debouncing
		{
			HAL_Delay(DEBOUNCE_DELAY);
		}

	}

	return;
}

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim10);

  uint16_t vals[2];

  int secs = 0; // Tracking the seconds the control LED has been blinking

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  transmit_sensors(vals, buffer);
	  get_buttons();


	  switch(global_state){
	  	  case IDLE:
	  		  break; // IDLE

	  	  case ON:
	  		  HAL_GPIO_WritePin(GPIOB, LED_CONTROL, 1);

	  		  if(get_water_level() < WATER_LEVEL_LOW)
	  		  {
	  			  set_global_state(FEHLER);
	  		  }

	  		  if(get_temp() < TEMP_HOT)
	  		  {
	  			  set_global_state(HEATING);
	  		  }

	  		  break; // ON

	  	  case ONE_CUP:
	  		  HAL_GPIO_WritePin(GPIOB, HEATER_CONTROL, 1);
	  		  HAL_GPIO_WritePin(GPIOB, PUMP_CONTROL, 1);
//	  		  transmit_sensors(vals, buffer);

	  		  if(__HAL_TIM_GET_COUNTER(&htim10) - timer_val >= 5000)
	  		  {
	  			  HAL_GPIO_TogglePin(GPIOB, LED_CONTROL);
	  			  timer_val = __HAL_TIM_GET_COUNTER(&htim10);
	  			  secs++;
	  		  }
	  		  if(secs > CUP_1_TIME*2)
	  		  {
	  			  secs = 0;
	  			  set_global_state(ON);
		  		  HAL_GPIO_WritePin(GPIOB, HEATER_CONTROL, 0);
		  		  HAL_GPIO_WritePin(GPIOB, PUMP_CONTROL, 0);
	  		  }

	  		  break; // ONE_CUP

	  	  case TWO_CUPS:
	  		  HAL_GPIO_WritePin(GPIOB, HEATER_CONTROL, 1);
	  		  HAL_GPIO_WritePin(GPIOB, PUMP_CONTROL, 1);
//	  		  transmit_sensors(vals, buffer);

	  		  if(__HAL_TIM_GET_COUNTER(&htim10) - timer_val >= 1000)
	  		  {
	  			  HAL_GPIO_TogglePin(GPIOB, LED_CONTROL);
	  			  timer_val = __HAL_TIM_GET_COUNTER(&htim10);
	  			  secs++;
	  		  }

	  		  if(secs > CUP_2_TIME*5)
	  		  {
	  			  secs = 0;
	  			  set_global_state(ON);
		  		  HAL_GPIO_WritePin(GPIOB, HEATER_CONTROL, 0);
		  		  HAL_GPIO_WritePin(GPIOB, PUMP_CONTROL, 0);

	  		  }
	  		  break; // TWO_CUPS

	  	  case HEATING:
	  		  HAL_GPIO_WritePin(GPIOB, HEATER_CONTROL, 1);
	  		  if(get_temp() >= TEMP_HOT)
	  		  {
		  		  HAL_GPIO_WritePin(GPIOB, HEATER_CONTROL, 0);
	  			  set_global_state(ON);
	  		  }
	  		  break; // HEATING

	  	  case FEHLER:
	  		  if(__HAL_TIM_GET_COUNTER(&htim10) - timer_val >= 1000)
	  		  {
	  			  HAL_GPIO_TogglePin(GPIOB, LED_CONTROL);
	  			  timer_val = __HAL_TIM_GET_COUNTER(&htim10);
	  		  }

	  		  if(get_water_level() >= WATER_LEVEL_LOW)
	  		  {
	  			  set_global_state(ON);
	  		  }
	  		  break; // FEHLER

	  	  default:
	  		  break; // default
	  } // switch

	 }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 8000-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB12 PB15 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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

  char msg[32];
  sprintf(msg, "Error_Handler: Reset board\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

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
