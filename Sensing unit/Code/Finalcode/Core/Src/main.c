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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static const uint8_t HIH7000_ADDR = 0x27 << 1; 							// Use 7-bit address
static const uint8_t MiCS_VZ89_ADDR = 0x70 << 1; 							// Use 7-bit address

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
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
  HAL_StatusTypeDef ret;

  uint8_t string_buf[50];
  uint8_t string_buf_transmit[50];
  uint8_t data_buf[6];			// HIH7000 requires 4 bytes, MICSVZ89 requires 6 bytes

  uint8_t status = 0;
  int16_t rawTemp = 0;
  int16_t rawHumid = 0;

  float humid = 0;
  float temp = 0;

  unsigned int decHumid = 0;
  unsigned int fracHumid = 0;
  unsigned int decTemp = 0;
  unsigned int fracTemp = 0;

  int tVOC = 200;
  int CO2 = 150;

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // ******************************************
	  // RELTATIVE HUMIDITY AND TEMPERATURE SENSING
	  // ******************************************

	  // Manually set what we want to write to the sensor (address)
	  data_buf[0] = 0x00;

	  // Tell HIH7000 that we want to read from the register
	  ret = HAL_I2C_Master_Transmit(&hi2c1, HIH7000_ADDR, data_buf, 1, HAL_MAX_DELAY);
	  HAL_Delay(250);
	  if ( ret != HAL_OK ) {
		  strcpy((char*)string_buf, "Error Tx\r\n");}
	  else {
		  // Read 4 bytes from the HIH7000 register
		  ret = HAL_I2C_Master_Receive(&hi2c1, HIH7000_ADDR, data_buf, 4, HAL_MAX_DELAY);
		  HAL_Delay(250);
		  if ( ret != HAL_OK ) {
			  strcpy((char*)string_buf, "Error Rx\r\n");}
		  else {

			  // Process the data

			  // The two leftmost MSB's of buf[0] are the status bits. We need to shift 6 times to the right to get those.
			  status = data_buf[0] >> 6;

			  // The next 14 bits are the raw humidity data bits
			  rawHumid = ((uint16_t)(data_buf[0] & 0x3f) << 8) | data_buf[1];

			  // The next 14 bits are the raw temperature data bits
			  rawTemp = (((uint16_t)data_buf[2] << 6) | (data_buf[3] >> 2));

			  // Process the humidity and temperature data
			  humid = ((float)rawHumid / 16382.0) * 100.0;
			  temp = ((float)rawTemp / 16382.0) * 165.0 - 40.0;

			  // Get the value before and after the decimal point
			  decHumid = (unsigned int) humid;
			  fracHumid = (humid - (float)decHumid)*100;
			  decTemp = (unsigned int) temp;
			  fracTemp = (temp - (float)decTemp)*100;


	  	  }
	  }



	  // **********************
      // VOC AND CO2 SENSING
      // **********************

      // Manually set what we want to write to the sensor (address)
      data_buf[0] = 0x0C;
      data_buf[1] = 0x00;
      data_buf[2] = 0x00;
      data_buf[3] = 0x00;
      data_buf[4] = 0x00;
      data_buf[5] = 0x00;

      // Tell MiCS_VZ89 that we want to read from the register
      ret = HAL_I2C_Master_Transmit(&hi2c1, MiCS_VZ89_ADDR, data_buf, 6, HAL_MAX_DELAY);
      HAL_Delay(250);
      if ( ret != HAL_OK ) {
          strcpy((char)string_buf, "Error Tx\r\n");}
      else {
           // Read 4 bytes from the HIH7000 register
           ret = HAL_I2C_Master_Receive(&hi2c1, MiCS_VZ89_ADDR, data_buf, 6, HAL_MAX_DELAY);
           HAL_Delay(250);
           if ( ret != HAL_OK ) {
               strcpy((char)string_buf, "Error Rx\r\n");}
           else {

               // Process the data

               //data_buf[0] = D1, calculating value tVOC

               tVOC     = ((uint16_t)(data_buf[0] - 13)*(1000.0/229));

               //data_buf[1] = D2, calculating value CO2

               CO2   	= ((uint16_t)(data_buf[1] - 13)*(1600.0/229) + 400);



            }
     }

    // Create output string
    sprintf((char*)string_buf, "Status: %u - Humid: %u.%02u % - Temp: %u.%02u - tVOC: %u - CO2: %u  \r\n",
         	(unsigned int) status,decHumid, fracHumid, decTemp, fracTemp, tVOC, CO2);

    sprintf((char*)string_buf_transmit, "T:%u.%02u,RH:%u.%02u,CO2:%u,VOC:%u\r\n",
                  		   	   	   	   	   	   	   	   	   	   	   decTemp, fracTemp, decHumid, fracHumid, CO2, tVOC);

    // Print the output
    	  HAL_UART_Transmit(&huart2, string_buf, strlen((char*)string_buf), HAL_MAX_DELAY);
    	  HAL_UART_Transmit(&huart1, string_buf_transmit, strlen((char*)string_buf_transmit), HAL_MAX_DELAY);
    	  HAL_Delay(500);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
