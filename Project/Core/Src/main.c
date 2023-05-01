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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "ds18b20.h"
#include "hcsr04.h"
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
	uint8_t x = 0;
	uint8_t y = 0;
	double temperatureValue = 0;
	uint8_t tempratureValueInteger;
	uint8_t tempratureValueDecimal;
	double distanceValue = 0;
	uint8_t distanceValueInteger;
	uint8_t distanceValueDecimal;
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();
	OLED_Clear();
	OLED_ShowChinese(x + 32 + 16 * 0, y + 2 * 0, 0);
	OLED_ShowChinese(x + 32 + 16 * 1, y + 2 * 0, 1);
	OLED_ShowChinese(x + 32 + 16 * 2, y + 2 * 0, 2);
	OLED_ShowChinese(x + 32 + 16 * 3, y + 2 * 0, 3);
	OLED_ShowChinese(x + 24 + 16 * 0, y + 2 * 1, 4);
	OLED_ShowChinese(x + 24 + 16 * 1, y + 2 * 1, 5);
	OLED_ShowChinese(x + 24 + 16 * 2, y + 2 * 1, 6);
	OLED_ShowChinese(x + 24 + 16 * 3, y + 2 * 1, 7);
	OLED_ShowChinese(x + 24 + 16 * 4, y + 2 * 1, 8);
	OLED_ShowChinese(x + 16 * 0, y + 2 * 2, 9);
	OLED_ShowChinese(x + 16 * 1, y + 2 * 2, 10);
	OLED_ShowChinese(x + 16 * 2, y + 2 * 2, 11);
	OLED_ShowChinese(x + 16 * 3, y + 2 * 2, 12);
	OLED_ShowChar(x + 16 * 4 + 8 * 0, y + 2 * 2, ':', 16);
	OLED_ShowChinese(x + 16 * 0, y + 2 * 3, 9);
	OLED_ShowChinese(x + 16 * 1, y + 2 * 3, 10);
	OLED_ShowChinese(x + 16 * 2, y + 2 * 3, 13);
	OLED_ShowChinese(x + 16 * 3, y + 2 * 3, 14);
	OLED_ShowChar(x + 16 * 4 + 8 * 0, y + 2 * 3, ':', 16);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (HAL_GPIO_ReadPin(HCSR505_IO_GPIO_Port, HCSR505_IO_Pin) == GPIO_PIN_SET)
		{
			temperatureValue = DS18B20_GetTemperture();
			tempratureValueInteger = (int)temperatureValue;
			tempratureValueDecimal = 10 * (temperatureValue - (int)temperatureValue);
			OLED_ShowNum(x + 16 * 4 + 8 * 2, y + 2 * 3, tempratureValueInteger, 2, 16);
			OLED_ShowChar(x + 16 * 4 + 8 * 4, y + 2 * 3, '.', 16);
			OLED_ShowNum(x + 16 * 4 + 8 * 5, y + 2 * 3, tempratureValueDecimal, 1, 16);
			OLED_ShowChar(x + 16 * 4 + 8 * 6, y + 2 * 3, 'C', 16);
		}
		
		distanceValue = HCSR04_GetDistance_Single();
		distanceValueInteger = (int)distanceValue;
		distanceValueDecimal = 10 * (distanceValue - (int)distanceValue);
		OLED_ShowNum(x + 16 * 4 + 8 * 2, y + 2 * 2, distanceValueInteger, 2, 16);
		OLED_ShowChar(x + 16 * 4 + 8 * 4, y + 2 * 2, '.', 16);
		OLED_ShowNum(x + 16 * 4 + 8 * 5, y + 2 * 2, distanceValueDecimal, 1, 16);
		OLED_ShowChar(x + 16 * 4 + 8 * 6, y + 2 * 2, 'c', 16);
		OLED_ShowChar(x + 16 * 4 + 8 * 7, y + 2 * 2, 'm', 16);
		
		if (HAL_GPIO_ReadPin(OUT3_GPIO_Port, OUT3_Pin) == GPIO_PIN_SET)
		{
			HAL_GPIO_WritePin(MotorDrive1_IN1_GPIO_Port, MotorDrive1_IN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MotorDrive1_IN2_GPIO_Port, MotorDrive1_IN2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MotorDrive1_IN3_GPIO_Port, MotorDrive1_IN3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MotorDrive1_IN4_GPIO_Port, MotorDrive1_IN4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MotorDrive2_IN1_GPIO_Port, MotorDrive2_IN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MotorDrive2_IN2_GPIO_Port, MotorDrive2_IN2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MotorDrive2_IN3_GPIO_Port, MotorDrive2_IN3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MotorDrive2_IN4_GPIO_Port, MotorDrive2_IN4_Pin, GPIO_PIN_RESET);
		}
		else if (HAL_GPIO_ReadPin(OUT1_GPIO_Port, OUT1_Pin) == GPIO_PIN_SET || HAL_GPIO_ReadPin(OUT2_GPIO_Port, OUT2_Pin) == GPIO_PIN_SET)
		{
			HAL_GPIO_WritePin(MotorDrive1_IN1_GPIO_Port, MotorDrive1_IN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MotorDrive1_IN2_GPIO_Port, MotorDrive1_IN2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MotorDrive1_IN3_GPIO_Port, MotorDrive1_IN3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MotorDrive1_IN4_GPIO_Port, MotorDrive1_IN4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MotorDrive2_IN1_GPIO_Port, MotorDrive2_IN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MotorDrive2_IN2_GPIO_Port, MotorDrive2_IN2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MotorDrive2_IN3_GPIO_Port, MotorDrive2_IN3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MotorDrive2_IN4_GPIO_Port, MotorDrive2_IN4_Pin, GPIO_PIN_RESET);
		}
		else if (HAL_GPIO_ReadPin(OUT4_GPIO_Port, OUT4_Pin) == GPIO_PIN_SET || HAL_GPIO_ReadPin(OUT5_GPIO_Port, OUT5_Pin) == GPIO_PIN_SET)
		{
			HAL_GPIO_WritePin(MotorDrive1_IN1_GPIO_Port, MotorDrive1_IN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MotorDrive1_IN2_GPIO_Port, MotorDrive1_IN2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MotorDrive1_IN3_GPIO_Port, MotorDrive1_IN3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MotorDrive1_IN4_GPIO_Port, MotorDrive1_IN4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MotorDrive2_IN1_GPIO_Port, MotorDrive2_IN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MotorDrive2_IN2_GPIO_Port, MotorDrive2_IN2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MotorDrive2_IN3_GPIO_Port, MotorDrive2_IN3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MotorDrive2_IN4_GPIO_Port, MotorDrive2_IN4_Pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(MotorDrive1_IN1_GPIO_Port, MotorDrive1_IN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MotorDrive1_IN2_GPIO_Port, MotorDrive1_IN2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MotorDrive1_IN3_GPIO_Port, MotorDrive1_IN3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MotorDrive1_IN4_GPIO_Port, MotorDrive1_IN4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MotorDrive2_IN1_GPIO_Port, MotorDrive2_IN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MotorDrive2_IN2_GPIO_Port, MotorDrive2_IN2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MotorDrive2_IN3_GPIO_Port, MotorDrive2_IN3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MotorDrive2_IN4_GPIO_Port, MotorDrive2_IN4_Pin, GPIO_PIN_RESET);
		}
		
		HAL_Delay(1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
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
