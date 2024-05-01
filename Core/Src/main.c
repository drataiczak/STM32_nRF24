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
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "nrf24l01p.h"
#include <stdio.h>

#ifdef __GNUC__
  #define _PUTCHAR int __io_putchar(int ch)
#else
  #define _PUTCHAR int fputc(int ch, FILE *f)
#endif

_PUTCHAR {
  HAL_UART_Transmit(&huart2, (uint8_t *) &ch, 1, HAL_MAX_DELAY);
  return 1;
}

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
void dumpRegs(nrf24_t *nrf);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void dumpRegs(nrf24_t *nrf) {
  uint8_t buf[5] = {0};
  uint8_t reg;
  
  reg = nRF24_GetConfig(nrf);
  printf("Config reg value: %02x\r\n", reg);
  reg = 0;

  reg = nRF24_GetAutoAck(nrf);
  printf("AutoAck reg value: %02x\r\n", reg);
  reg = 0;

  reg = nRF24_GetEnabledRxAddrs(nrf);
  printf("Enabled RX addrs reg value: %02x\r\n", reg);
  reg = 0;

  reg = nRF24_GetAddrWidth(nrf);
  printf("Addr width reg value: %02x\r\n", reg);
  reg = 0;

  reg = nRF24_GetAutoRetransmit(nrf);
  printf("Auto retransmit reg value: %02x\r\n", reg);
  reg = 0;

  reg = nRF24_GetRFChannel(nrf);
  printf("RF channel reg value: %02x\r\n", reg);
  reg = 0;

  reg = nRF24_GetRFConfig(nrf);
  printf("RF config reg value: %02x\r\n", reg);
  reg = 0;

  reg = nRF24_GetStatus(nrf);
  printf("Status reg value: %02x\r\n", reg);
  reg = 0;

  reg = nRF24_GetTxObserve(nrf);
  printf("TX observation reg value: %02x\r\n", reg);
  reg = 0;

  reg = nRF24_GetRPD(nrf);
  printf("RPD reg value: %02x\r\n", reg);
  reg = 0;

  nRF24_GetRXPipeAddr(nrf, p0, buf, sizeof(buf));
  for(uint8_t i = 0; i < sizeof(buf); i++) {
    printf("Rx pipe 0 reg value byte %d: %02x\r\n", i, buf[i]);
    buf[i] = 0;
  }

  nRF24_GetRXPipeAddr(nrf, p1, buf, sizeof(buf));
  for(uint8_t i = 0; i < sizeof(buf); i++) {
    printf("Rx pipe 1 reg value byte %d: %02x\r\n", i, buf[i]);
    buf[i] = 0;
  }

  nRF24_GetRXPipeAddr(nrf, p2, &reg, 1);
  printf("Rx pipe 2 value: %02x\r\n", reg);

  nRF24_GetRXPipeAddr(nrf, p3, &reg, 1);
  printf("Rx pipe 2 value: %02x\r\n", reg);

  nRF24_GetRXPipeAddr(nrf, p4, &reg, 1);
  printf("Rx pipe 2 value: %02x\r\n", reg);

  nRF24_GetRXPipeAddr(nrf, p5, &reg, 1);
  printf("Rx pipe 2 value: %02x\r\n", reg);

  nRF24_GetTXAddr(nrf, buf, sizeof(buf));
  for(uint8_t i = 0; i < sizeof(buf); i++) {
    printf("Tx addr byte %d: %02x\r\n", i, buf[i]);
    buf[i] = 0;
  }

  reg = nRF24_GetRXPipeWidth(nrf, p0);
  printf("Rx pipe 0 width reg value: %02x\r\n", reg);
  reg = 0;

  reg = nRF24_GetRXPipeWidth(nrf, p1);
  printf("Rx pipe 1 width reg value: %02x\r\n", reg);
  reg = 0;

  reg = nRF24_GetRXPipeWidth(nrf, p2);
  printf("Rx pipe 2 width reg value: %02x\r\n", reg);
  reg = 0;

  reg = nRF24_GetRXPipeWidth(nrf, p3);
  printf("Rx pipe 3 width reg value: %02x\r\n", reg);
  reg = 0;

  reg = nRF24_GetRXPipeWidth(nrf, p4);
  printf("Rx pipe 4 width reg value: %02x\r\n", reg);
  reg = 0;

  reg = nRF24_GetRXPipeWidth(nrf, p5);
  printf("Rx pipe 5 width reg value: %02x\r\n", reg);
  reg = 0;

  nRF24_ClearFifoStatus(nrf);
  reg = nRF24_GetFifoStatus(nrf);
  printf("Fifo status reg value: %02x\r\n", reg);
  reg = 0;

  reg = nRF24_GetDynamicPayloadConfig(nrf);
  printf("Dynamic payload config reg value: %02x\r\n", reg);
  reg = 0;

  reg = nRF24_GetFeatureConfig(nrf);
  printf("Feature config reg value: %02x\r\n", reg);
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

  nrf24_t nrf;
  uint8_t status = 0;
  status = nRF24_Init(&nrf, &hspi1, CSN_GPIO_Port, CSN_Pin, CE_GPIO_Port, CE_Pin);

  #ifdef DUMPREGS
    dumpRegs(&nrf);
  #endif
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

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
