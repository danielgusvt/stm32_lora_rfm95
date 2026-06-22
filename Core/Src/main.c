/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdint.h>
#include <stdio.h>

#include "rfm95.h"
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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}

uint32_t get_precision_tick(void) {
  uint32_t ms;
  uint32_t st_val;
  uint32_t load = SysTick->LOAD;
  do {
    ms = HAL_GetTick();
    st_val = SysTick->VAL;
  } while (ms != HAL_GetTick());
  
  // SysTick counts down. Convert to microseconds.
  return (ms * 1000) + ((load - st_val) * 1000) / load;
}

void precision_sleep_until(uint32_t target) {
  while ((int32_t)(target - get_precision_tick()) > 0) {
    // Wait
  }
}

rfm95_handle_t rfm95_handle = {
  .spi_handle = &hspi1,
  .nss_port = RFM95_NSS_GPIO_Port,
  .nss_pin = RFM95_NSS_Pin,
  .nrst_port = RFM95_NRST_GPIO_Port,
  .nrst_pin = RFM95_NRST_Pin,
  .precision_tick_frequency = 1000000,
  .precision_tick_drift_ns_per_s = 20000, // Roughly 20ppm drift
  .get_precision_tick = get_precision_tick,
  .precision_sleep_until = precision_sleep_until
};

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == RFM95_DIO0_Pin) {
    rfm95_on_interrupt(&rfm95_handle, RFM95_INTERRUPT_DIO0);
  }
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * LoRa image-transfer framing (must match the BL808 sender).
 *   HEADER : [0xA1][len(4,BE)][chunks(2,BE)]
 *   DATA   : [0xA2][idx(2,BE)][ up to 240 JPEG bytes ]
 *   TAIL   : [0xA3][len(4,BE)]
 *
 * Each received image is dumped to the serial port as:
 *   JPEG_BEGIN len=<N> chunks=<C> snr=<S>
 *   <hex line per DATA chunk>
 *   ...
 *   JPEG_END received=<r>/<C> gaps=<g>
 * Copy the hex lines between the markers and run:  xxd -r -p > img.jpg
 */
#define PKT_TYPE_HEADER 0xA1
#define PKT_TYPE_DATA   0xA2
#define PKT_TYPE_TAIL   0xA3

static void uart_write_hex(const uint8_t *data, size_t len)
{
  static const char hexchars[] = "0123456789abcdef";
  char line[2 * 240 + 2];
  size_t pos = 0;
  for (size_t i = 0; i < len; i++) {
    line[pos++] = hexchars[data[i] >> 4];
    line[pos++] = hexchars[data[i] & 0x0F];
  }
  line[pos++] = '\r';
  line[pos++] = '\n';
  HAL_UART_Transmit(&huart2, (uint8_t *)line, pos, HAL_MAX_DELAY);
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // Initialise RFM95 module.
  if (!rfm95_init(&rfm95_handle)) {
    printf("RFM95 init failed\r\n");
  }
  printf("RFM95 init successfully!\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t rx_buffer[252];
  size_t  rx_len;
  int8_t  received_snr;

  uint32_t img_len      = 0;
  uint16_t total_chunks = 0;
  uint16_t recv_chunks  = 0;
  uint16_t expected_idx = 0;
  uint16_t gaps         = 0;
  uint8_t  in_image     = 0;
  uint32_t t_begin      = 0;

  /* Read the modem configuration registers once (they don't change at runtime). */
  uint8_t cfg1 = 0, cfg2 = 0, cfg3 = 0;
  rfm95_read_register(&rfm95_handle, RFM95_REGISTER_MODEM_CONFIG_1, &cfg1);
  rfm95_read_register(&rfm95_handle, RFM95_REGISTER_MODEM_CONFIG_2, &cfg2);
  rfm95_read_register(&rfm95_handle, RFM95_REGISTER_MODEM_CONFIG_3, &cfg3);

  printf("LoRa image receiver ready — waiting for transmissions...\r\n");

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (!rfm95_receive_package(&rfm95_handle, rx_buffer, &rx_len, &received_snr)) {
      continue;  /* timeout, keep listening */
    }
    if (rx_len < 1) {
      continue;
    }

    switch (rx_buffer[0]) {
      case PKT_TYPE_HEADER:
        if (rx_len < 7) break;
        {
          uint32_t new_len = ((uint32_t)rx_buffer[1] << 24) | ((uint32_t)rx_buffer[2] << 16) |
                             ((uint32_t)rx_buffer[3] << 8)  |  (uint32_t)rx_buffer[4];
          uint16_t new_chunks = ((uint16_t)rx_buffer[5] << 8) | rx_buffer[6];
          /* The sender transmits HEADER several times for resilience. Ignore
             repeats of the image we're already receiving, but still accept a
             genuinely new image (e.g. if the previous TAIL was lost). */
          if (in_image && new_len == img_len && new_chunks == total_chunks) {
            break;
          }
          img_len = new_len;
          total_chunks = new_chunks;
        }
        recv_chunks = 0;
        expected_idx = 0;
        gaps = 0;
        in_image = 1;
        t_begin = HAL_GetTick();
        printf("\r\nJPEG_BEGIN len=%lu chunks=%u snr=%d CONFIG1=0x%02X CONFIG2=0x%02X CONFIG3=0x%02X\r\n",
               (unsigned long)img_len, total_chunks, (int)received_snr,
               cfg1, cfg2, cfg3);
        break;

      case PKT_TYPE_DATA:
        if (rx_len < 3 || !in_image) break;
        {
          uint16_t idx = ((uint16_t)rx_buffer[1] << 8) | rx_buffer[2];
          if (idx != expected_idx) {
            gaps++;  /* one or more missing/out-of-order chunks */
          }
          expected_idx = (uint16_t)(idx + 1);
          recv_chunks++;
          uart_write_hex(&rx_buffer[3], rx_len - 3);
        }
        break;

      case PKT_TYPE_TAIL:
        if (!in_image) break;
        printf("JPEG_END received=%u/%u gaps=%u time=%lums\r\n",
               recv_chunks, total_chunks, gaps,
               (unsigned long)(HAL_GetTick() - t_begin));
        in_image = 0;
        break;

      default:
        break;
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RFM95_NSS_Pin|RFM95_NRST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RFM95_DIO0_Pin */
  GPIO_InitStruct.Pin = RFM95_DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RFM95_DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RFM95_NSS_Pin LD3_Pin RFM95_NRST_Pin */
  GPIO_InitStruct.Pin = RFM95_NSS_Pin|LD3_Pin|RFM95_NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
