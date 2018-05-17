/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "fatfs.h"
#include "light_ws2812_cortex.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// GPS/GSM variables
int Rx_indx;
unsigned char Rx_data[2];
unsigned char Rx_Buffer[100];
unsigned char last_Rx[100];
int Full_indx;
int Transfer_cplt;
int Rx_size;
char Tx_Buffer[100];
int len;

// SD variables
FATFS userDiskFatFs;
FIL file;
char path[4];
uint32_t wbytes; // file write counts

// WiFi Array of BSSID values
int wifi_arr[10] = {-50, -60, -55, -35, -87, -67, -70, -43, -90, -30};
// g, y, y, g, r, y, y, g, r, g
int ii = 0;

// Testing constants
int i = 0;

// Other flags
int tim_flag = 0;

// LED constants
const char green[18] = {0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255,
                       0, 0, 255, 0, 0, 255};
const char red[18] = {0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0,
                     0, 255, 0, 0, 255, 0};
const char blue[18] = {255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0,
                      255, 0, 0, 255, 0, 0};
const char orange[18] = {0, 255, 165, 0, 255, 165, 0, 255, 165, 0, 255, 165,
                        0, 255, 165, 0, 255, 165};
const char white[18] = {255, 255, 255, 255, 255, 255, 255, 255, 255,
                        255, 255, 255, 255, 255, 255, 255, 255, 255};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM15_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  /*
  while (1) {
      // test LED functionality with wifi_arr
      for(ii = 0; ii < 10; ii++){
          if ((wifi_arr[ii] <= -30) && (wifi_arr[ii] >= -50)) {
              ws2812_sendarray(green, 18);
          }
          else if ((wifi_arr[ii] < -50) && (wifi_arr[ii] >= -70)){
              ws2812_sendarray(orange, 18);
          }
          else if ((wifi_arr[ii] < -70) && (wifi_arr[ii] >= -90)){
              ws2812_sendarray(red, 18);
          }
          HAL_Delay(1000);
          if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_SET) {
              ws2812_sendarray(white, 18);
              HAL_Delay(1000);
          }
      }
  }
  */

  // turn LED strip white
  ws2812_sendarray((uint8_t *) white, 18);

  // wait 5 seconds for power on
  HAL_Delay(5000);

  // turn on UART1 interrupts
  HAL_UART_Receive_IT(&huart1, Rx_data, 1);

  // set auto baud
  len = sprintf(Tx_Buffer, "AT\r");
  while (sendGPS(Tx_Buffer, len, 50) == 0);

  // turn the led blue
  ws2812_sendarray((uint8_t *) blue, 18);

  //Establish and acquire GPS location
  len = sprintf(Tx_Buffer, "AT+CGPSPWR=1\r");
  sendGPS(Tx_Buffer, len, 60000);

  len = sprintf(Tx_Buffer, "AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r");
  sendGPS(Tx_Buffer, len, 50);

  len = sprintf(Tx_Buffer, "AT+SAPBR=3,1,\"APN\",\"CMNET\"\r");
  sendGPS(Tx_Buffer, len, 50);

  len = sprintf(Tx_Buffer, "AT+SAPBR=1,1\r");
  sendGPS(Tx_Buffer, len, 85000);

  len = sprintf(Tx_Buffer, "AT+SAPBR=2,1\r");
  sendGPS(Tx_Buffer, len, 50);

  len = sprintf(Tx_Buffer, "AT+CREG=2\r");
  sendGPS(Tx_Buffer, len, 50);

  // Create the SD file
  uint8_t wtext[20];
  len = sprintf((char *)wtext, "%s\n", "Longitude,Latitude");
  if (FATFS_LinkDriver(&USER_Driver, path) == 0) {
      if (f_mount(&userDiskFatFs, (TCHAR const*)path, 0) == FR_OK) {
          if ((f_open(&file, "OUTPUT.CSV", FA_CREATE_ALWAYS | FA_WRITE)) == FR_OK) {
              if (f_write(&file, wtext, len, (void *)&wbytes) == FR_OK) {
                  f_close(&file);
              }
          }
      }
      FATFS_UnLinkDriver(path);
  }

  // start the timer interrupts
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
      if (tim_flag) {
          /*
          if ((i % 2) == 0) {
              uint8_t new_led_arr[] = {255, 0, 0}; // blue
              ws2812_sendarray(new_led_arr, 3);
          }
          else {
              uint8_t new_led_arr[] = {0, 255, 0}; // red
              ws2812_sendarray(new_led_arr, 3);
          }
          i++;
          */
          if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_SET) {
              ws2812_sendarray((uint8_t *) white, 18);
              HAL_Delay(500);
          }

          if ((wifi_arr[ii] <= -30) && (wifi_arr[ii] >= -50)) {
              ws2812_sendarray((uint8_t *) green, 18);
          }
          else if ((wifi_arr[ii] < -50) && (wifi_arr[ii] >= -70)){
              ws2812_sendarray((uint8_t *) orange, 18);
          }
          else if ((wifi_arr[ii] < -70) && (wifi_arr[ii] >= -90)){
              ws2812_sendarray((uint8_t *) red, 18);
          }
          ii = (ii+1) % 10;


          char code[4];
          char lon[12];
          char lat[12];
          code[3] = '\0';
          lon[11] = '\0';
          lon[10] = '\0';
          lon[9] = '\0';
          lat[11] = '\0';
          lat[10] = '\0';
          lat[9] = '\0';

          // check the signal via GPS
          //len = sprintf(Tx_Buffer, "AT+CGPSINF=0\r");
          //sendGPS(Tx_Buffer, len, 50);


          // check the signal via GSM
          len = sprintf(Tx_Buffer, "AT+CIPGSMLOC=1,1\r");
          sendGPS(Tx_Buffer, len, 4000);

          // parse the Rx_Buffer
          int this_idx = 0;
          int buff_idx = 2;
          while (!(last_Rx[buff_idx] == ' ' && last_Rx[buff_idx-1] == ':' && last_Rx[buff_idx-2] == 'C') && buff_idx < 100) {
              buff_idx++;
          }
          buff_idx++;
          int commas = 0;
          if (buff_idx > 100) {
              code[0] = 'e';
              code[1] = 'r';
              code[2] = 'r';
              commas = 3;
          }
          char c;
          while (commas < 3) {
              c = last_Rx[buff_idx++];
              if (c == ',') {
                  this_idx = 0;
                  commas++;
              }
              else {
                  if (commas == 0) {
                      code[this_idx++] = c;
                      if (this_idx == 3) {
                          commas = 3;
                      }
                  }
                  else if (commas == 1) {
                      lon[this_idx++] = c;
                  }
                  else {
                      lat[this_idx++] = c;
                  }
              }
          }

          // Output to SD card
          if (code[0] == '0') {
              uint8_t new_wtext[20];
              len = sprintf((char *) new_wtext, "%s,%s\n", lon, lat);
              if (FATFS_LinkDriver(&USER_Driver, path) == 0) {
                  if (f_mount(&userDiskFatFs, (TCHAR const*)path, 0) == FR_OK) {
                      if (f_open(&file, "OUTPUT.CSV", FA_OPEN_ALWAYS | FA_WRITE) == FR_OK) {
                          if (f_lseek(&file, f_size(&file)) == FR_OK) {
                              if (f_write(&file, new_wtext, len, (void *)&wbytes) == FR_OK) {
                                  f_close(&file);
                              }
                          }
                      }
                  }
                  FATFS_UnLinkDriver(path);
              }
          }
          else {
              uint8_t new_wtext[11];
              len = sprintf((char *) new_wtext, "ERROR,%s\n", code);
              if (FATFS_LinkDriver(&USER_Driver, path) == 0) {
                  if (f_mount(&userDiskFatFs, (TCHAR const*)path, 0) == FR_OK) {
                      if (f_open(&file, "OUTPUT.CSV", FA_OPEN_ALWAYS | FA_WRITE) == FR_OK) {
                          if (f_lseek(&file, f_size(&file)) == FR_OK) {
                              if (f_write(&file, new_wtext, len, (void *)&wbytes) == FR_OK) {
                                  f_close(&file);
                              }
                          }
                      }
                  }
                  FATFS_UnLinkDriver(path);
              }
          }

          tim_flag = 0;
      }

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM15 init function */
static void MX_TIM15_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 48000;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 0;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin: PC5 (switch) */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int sendGPS(char * buff, int len, int waitTime) {
    HAL_UART_Transmit(&huart1, (unsigned char*)buff, len, 100);
    HAL_Delay(waitTime);
    if (Transfer_cplt) {
        Transfer_cplt = 0;
        return 1;
    }
    // reset Rx_indx
    Rx_indx = 0;
    return 0;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
