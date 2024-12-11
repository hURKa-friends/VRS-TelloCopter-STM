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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LSM6DS0.h"
#include "LIS3MDL.h"
#include "SensorDataProcessing.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_DATA_BUFFER		8
uint8_t dataCounter = 0;
float gyroX[MAX_DATA_BUFFER];
float gyroY[MAX_DATA_BUFFER];
float gyroZ[MAX_DATA_BUFFER];
float acclX[MAX_DATA_BUFFER];
float acclY[MAX_DATA_BUFFER];
float acclZ[MAX_DATA_BUFFER];
float gyroMeanValues[3];
float acclMeanValues[3];
float radAngleValues[3];
float degAngleValues[3];
float outputData[3];
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
/**
  * @brief   Main interrupt handler for TIM2. Handles gyroscope data acquisition.
  * @details This function should be used for I2C handling.
  * @note    This interrupt handler is called with period T_TIM2 = 8.333 ms (120 Hz).
  * @retval  None
  */
void TIM2_IRQ_main(void)
{
	int16_t rawGyroX, rawGyroY, rawGyroZ;
	int16_t rawAcclX, rawAcclY, rawAcclZ;

	LSM6DS0_get_gyro(&rawGyroX, &rawGyroY, &rawGyroZ);
	LSM6DS0_get_accl(&rawAcclX, &rawAcclY, &rawAcclZ);

	gyroX[dataCounter] = LSM6DS0_parse_gyro_data(rawGyroX);
	gyroY[dataCounter] = LSM6DS0_parse_gyro_data(rawGyroY);
	gyroZ[dataCounter] = LSM6DS0_parse_gyro_data(rawGyroZ);

	acclX[dataCounter] = LSM6DS0_parse_accl_data(rawAcclX);
	acclY[dataCounter] = LSM6DS0_parse_accl_data(rawAcclY);
	acclZ[dataCounter] = LSM6DS0_parse_accl_data(rawAcclZ);

	gyroMeanValues[0] = movingAvgFilter((float *)gyroX, MAX_DATA_BUFFER);
	gyroMeanValues[1] = movingAvgFilter((float *)gyroY, MAX_DATA_BUFFER);
	gyroMeanValues[2] = movingAvgFilter((float *)gyroZ, MAX_DATA_BUFFER);

	acclMeanValues[0] = movingAvgFilter((float *)acclX, MAX_DATA_BUFFER);
	acclMeanValues[1] = movingAvgFilter((float *)acclY, MAX_DATA_BUFFER);
	acclMeanValues[2] = movingAvgFilter((float *)acclZ, MAX_DATA_BUFFER);

	dataCounter++;
	if(dataCounter >= MAX_DATA_BUFFER)
		dataCounter = 0;
}


/**
  * @brief   Main interrupt handler for TIM3.
  * @details This function should be used for data processing and USART handling.
  * @note    This interrupt handler is called with period T_TIM3 = 50 ms (20 Hz).
  * @retval  None
  */
void TIM3_IRQ_main(void)
{
	calculate_angles(radAngleValues, acclMeanValues);

	for(int i = 0; i < 3; i++)
		degAngleValues[i] = rad2deg(radAngleValues[i]);

	// X - Pitch
	outputData[0] = linInterpolation(degAngleValues[0], 5.0, 45.0, 0, 100);
	// Y - Roll
	outputData[1] = linInterpolation(degAngleValues[1], 5.0, 45.0, 0, 100);
	// Z - Yaw
	outputData[2] = linInterpolation(degAngleValues[2], 5.0, 45.0, 0, 100);

	// Send Data
	USART2_send_data("NONE", (int8_t)outputData[1], (int8_t)outputData[0], (int8_t)outputData[2]);
}

/**
  * @brief   Main cycle handler for the system.
  * @details Should be used for data processing.
  * @retval  None
  */
void SystemMainCycleRoutine(void)
{
	LL_GPIO_SetOutputPin(GPIOA, TESTPIN_4_Pin);
	asm("#NOP");
	// TODO: Possible Data Processing ??
	LL_GPIO_ResetOutputPin(GPIOA, TESTPIN_4_Pin);
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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  LL_SYSTICK_EnableIT();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  for (int i = 0; i < MAX_DATA_BUFFER; i++)
  {
	gyroX[dataCounter] = 0;
	gyroY[dataCounter] = 0;
	gyroZ[dataCounter] = 0;
  }

  LSM6DS0_init(I2C1_MultiByteRead, I2C1_MultiByteWrite);

  /* TODO: Change sensor initialization
	  LIS3MDL_registerReadCallback(I2C1_MultiByteRead);
	  LIS3MDL_registerWriteCallback(I2C1_MultiByteWrite);
	  LIS3MDL_init();
  */

  TIM2_RegisterCallback(TIM2_IRQ_main);
  TIM3_RegisterCallback(TIM3_IRQ_main);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		SystemMainCycleRoutine();
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_16);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(64000000);
  LL_SetSystemCoreClock(64000000);
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
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
