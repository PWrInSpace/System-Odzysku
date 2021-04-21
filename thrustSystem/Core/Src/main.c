/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "valve_control.h"
#include "motor_control.h"
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

uint8_t limit_switch_open_state;
uint8_t limit_switch_close_state;
int16_t enc = 0;

struct Valve v;
struct Motor m;
struct Valve * valve = &v;
struct Motor * motor = &m;

enum Direction {CLOSE = 0, OPEN = 1};
enum Direction direction = CLOSE;

enum State {VALVE, ZIGBEE};
enum State current_state = ZIGBEE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void stateValve();
void stateZIGBEE();

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
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  int8_t max_state = 12;
  int8_t encoder_tick = 2;

  // Alternatywna inicjalizacja silnika
  //uint16_t  motor_speed = 500; 500 - 50% wypelnienia
  //Motor_Init(motor_speed);

  Motor_Init_Default(motor,
		  Motor_IN1_GPIO_Port,
		  Motor_IN1_Pin,
		  Motor_IN2_GPIO_Port,
		  Motor_IN2_Pin);
  //Motor_Init(motor, (uint16_t)910);
  Valve_Init(valve, motor, max_state, encoder_tick);

  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //maszyna stanow
	  switch (current_state) {
	  	  case VALVE: {
	  		  stateValve();
	  	  }
	  	  break;
	  	  case ZIGBEE: {
	  		  stateZIGBEE();
	  	  }
	  	  break;
	  }
	  //Valve_Force_Close(valve);
	  /*
	  enc = TIM4->CNT;
	  if (enc > 0) {
		  limit_switch_close_state = HAL_GPIO_ReadPin(Limit_Switch_Close_GPIO_Port, Limit_Switch_Close_Pin);
		  limit_switch_open_state  = HAL_GPIO_ReadPin(Limit_Switch_Open_GPIO_Port, Limit_Switch_Open_Pin);

		  if (limit_switch_close_state == 0) {
			  direction = OPEN;
		  }

		  if (limit_switch_open_state == 0) {
			  direction = CLOSE;
			  //Motor_Stop();
		  }

		  if (direction == CLOSE) {
			  Valve_Close(valve);
		  }

		  if (direction == OPEN) {
			  Valve_Open(valve);
		  }
	  }
	  else {
		  Motor_Stop();
	  }
	  */
	  /*
	  if (limit_switch_close_state == 0) {
		  HAL_GPIO_WritePin(led_on_board_GPIO_Port, led_on_board_Pin, GPIO_PIN_RESET);
	  }
	  else {
		  HAL_GPIO_WritePin(led_on_board_GPIO_Port, led_on_board_Pin, GPIO_PIN_SET);
	  }

	  if (limit_switch_open_state == 0) {
		  HAL_GPIO_WritePin(led_on_board_GPIO_Port, led_on_board_Pin, GPIO_PIN_RESET);
	  }
	  else {
		  HAL_GPIO_WritePin(led_on_board_GPIO_Port, led_on_board_Pin, GPIO_PIN_SET);
	  }
	  */
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

/* USER CODE BEGIN 4 */

void stateValve() {
	while (current_state == VALVE) {
		enc = TIM4->CNT;
		if (enc > 0) {
			limit_switch_close_state = HAL_GPIO_ReadPin(Limit_Switch_Close_GPIO_Port, Limit_Switch_Close_Pin);
			limit_switch_open_state  = HAL_GPIO_ReadPin(Limit_Switch_Open_GPIO_Port, Limit_Switch_Open_Pin);

			if (limit_switch_close_state == 0) {
				direction = OPEN;
			}

			if (limit_switch_open_state == 0) {
				direction = CLOSE;
			}

			if (direction == CLOSE) {
				Valve_Close(valve);
			}

			if (direction == OPEN) {
				Valve_Open(valve);
			}
		  }
		else {
			Motor_Stop(motor);
		}
	/*
	 	 if (cos co powie, ze trzeba zmienic stan) {
	 	 	 current_state = ZIGBEE;
	 	 }
	*/
	}
}

void stateZIGBEE() {
	while (current_state == ZIGBEE) {
		current_state = VALVE;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
