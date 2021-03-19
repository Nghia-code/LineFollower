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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID.h"
#include "stdbool.h"

float error = 0;
int sensor[5];
int R_motor_speed = 0, L_motor_speed = 0;
int init_speed = 200;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
PID_t PID_LEFT = {.Kp = 50, .Ki = 0.0001, .Kd = 30, .dt = 30,  .I_saturation = 180, .error = 0, .last_err = 0};
PID_t PID_RIGHT = {.Kp = 50, .Ki = 0.0001, .Kd = 30, .dt = 30,  .I_saturation = 180, .error = 0, .last_err = 0};

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
/* USER CODE BEGIN PFP */


void read_sensor(void)
{
	  sensor[0] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
	  sensor[1] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
	  sensor[2] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14);
	  sensor[3] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
	  sensor[4] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
	  //if only middle sensor detects black line
	  if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 1 && sensor[4] == 1)
		  error = 0;
	  //going forward with full speed [1 1 0 1 1]- 1


	  //if only left sensor detects black line
	  if (sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1)
		  error = -1;
	  //going right with full speed [1 0 1 1 1] - 2


	  //if only left most sensor detects black line
	  if (sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1)
		  error = -4;
	  //going right with full speed [0 1 1 1 1]- 3


	  //if only right sensor detects black line
	  if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 0 && sensor[4] ==1)
		  error = 1;
	  //going left with full speed [1 1 1 0 1] - 4


	  //if only right most sensor detects black line
	  if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 0)
	   	  error = 4;
	  //going left with full speed [1 1 1 1 0] - 5


	  //if middle and right sensor detects black line
	  if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 1)
	 	  error = 2;
	  //going left with full speed [1 1 0 0 1] - 6


	  //if middle and left sensor detects black line
	  if(sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 1 && sensor[4] == 1)
		  error = -2;
	  //going right with full speed [1 0 0 1 1] - 7


	  //if middle, left and left most sensor detects black line
	  if(sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 1 && sensor[4] == 1)
		 error = -3;
	  //going right with full speed [0 0 0 1 1] - 8


	  //if middle, right and right most sensor detects black line
	  if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 0)
		  error = 3;
	  //going left with full speed [1 1 0 0 0] - 9

	  //if all sensors are on a black line - 10
	  if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 0)
		  error = 6;


}

void go_straight_ahead(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
}

void go_back(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

void stop(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  PID_init(&PID_LEFT, (float)30, (float)200, (float)180);
  PID_init(&PID_RIGHT, (float)30, (float)200, (float)180);
  PID_set_params(&PID_LEFT, (float)50, (float)0.001, (float)30);
  PID_set_params(&PID_RIGHT, (float)50, (float)0.001, (float)30);

  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
 // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  //HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,L_motor_speed);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,600);
//			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,0);
//			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,500);
//			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,0);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == htim4.Instance ){
		read_sensor();
		PID_update(&PID_LEFT, 0 , error);
		PID_update(&PID_RIGHT, 0 , error);

		R_motor_speed = init_speed + PID_ReadValue(&PID_LEFT)+ 50;
		L_motor_speed = init_speed + PID_ReadValue(&PID_RIGHT)+ 50;

		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,L_motor_speed);
		go_back();
		//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,200);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,R_motor_speed);
		//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,0);


		if(error == 6){
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,0);
			return;
		}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
