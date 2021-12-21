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
#include "dac.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "retarget.h"
#include "kalman.h"
#include "stdbool.h"
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
uint16_t Adc_Values[4];
float Adc_Kalman_calc[4];
SV_KalmanFilter AdcKalman1;
SV_KalmanFilter AdcKalman2;
SV_KalmanFilter AdcKalman3;
SV_KalmanFilter AdcKalman4;
float ADC_Mean;

uint16_t Tab_For_ESP[32]; //500 Hz - meaning we can get 8 sec of sounds

volatile _Bool generate_sound_flag = true;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void SetChannel(uint32_t Channel);
void ReadMultiADC(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
UART_HandleTypeDef huart1;
RTC_TimeTypeDef RtcTime;
RTC_DateTypeDef RtcDate;
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
  MX_ADC_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_RTC_Init();
  MX_DAC_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  RetargetInit(&huart1);

  SV_PopulateKalmanFilter(&AdcKalman1, 2, 2, 0.1);
  SV_PopulateKalmanFilter(&AdcKalman2, 2, 2, 0.1);
  SV_PopulateKalmanFilter(&AdcKalman3, 2, 2, 0.1);
  SV_PopulateKalmanFilter(&AdcKalman4, 2, 2, 0.1);

  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, SET);
  HAL_TIM_Base_Start_IT(&htim2);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* USER CODE BEGIN 4 */

//set ADC channel for next sampling
void SetChannel(uint32_t Channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
  sConfig.Channel = Channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_9CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

// reads four ADC channels in polling method
void ReadMultiADC()
{

  // SWITCH TO CH0
  SetChannel(ADC_CHANNEL_5);
  // CH0
  HAL_ADC_Start(&hadc);

  if (HAL_ADC_PollForConversion(&hadc, 10) == HAL_OK)
  {
    Adc_Values[0] = HAL_ADC_GetValue(&hadc);
  }

  // SWITCH TO CH1
  SetChannel(ADC_CHANNEL_8);
  //CH1
  HAL_ADC_Start(&hadc);

  if (HAL_ADC_PollForConversion(&hadc, 10) == HAL_OK)
  {
    Adc_Values[1] = HAL_ADC_GetValue(&hadc);
  }

  // SWITCH TO CH1
  SetChannel(ADC_CHANNEL_7);
  //CH1
  HAL_ADC_Start(&hadc);

  if (HAL_ADC_PollForConversion(&hadc, 10) == HAL_OK)
  {
    Adc_Values[2] = HAL_ADC_GetValue(&hadc);
  }

  // SWITCH TO CH1
  SetChannel(ADC_CHANNEL_6);
  //CH1
  HAL_ADC_Start(&hadc);

  if (HAL_ADC_PollForConversion(&hadc, 10) == HAL_OK)
  {
    Adc_Values[3] = HAL_ADC_GetValue(&hadc);
  }
}

//
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint32_t tempDAC;

  if (htim->Instance == TIM2)
  {
    ReadMultiADC();
    Adc_Kalman_calc[0] = SV_UpdateEstimate(&AdcKalman1, Adc_Values[0]);
    Adc_Kalman_calc[1] = SV_UpdateEstimate(&AdcKalman2, Adc_Values[1]);
    Adc_Kalman_calc[2] = SV_UpdateEstimate(&AdcKalman3, Adc_Values[2]);
    Adc_Kalman_calc[3] = SV_UpdateEstimate(&AdcKalman4, Adc_Values[3]);

    ADC_Mean = (Adc_Kalman_calc[0] + Adc_Kalman_calc[1] + Adc_Kalman_calc[2] + Adc_Kalman_calc[4]) / 4;

    if (generate_sound_flag == true)
    {
      tempDAC = ADC_Mean * (0xFFF + 1) / 3.3;
      HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, tempDAC);
    }

    printf("%d \r\n", (uint16_t)ADC_Mean); //add CRC?
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
