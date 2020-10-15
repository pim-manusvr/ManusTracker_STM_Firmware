/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
uint16_t f_updateLed = 0;
static uint8_t read_r = 0;
static uint8_t read_g = 0;
static uint8_t read_b = 0;

uint32_t SineLookup[512] = {
512,518,524,530,537,543,549,555,562,568,574,580,587,593,599,605,
611,617,624,630,636,642,648,654,660,666,672,678,684,690,696,701,
707,713,719,725,730,736,741,747,753,758,764,769,774,780,785,790,
796,801,806,811,816,821,826,831,836,841,846,850,855,860,864,869,
873,878,882,886,890,895,899,903,907,911,915,919,922,926,930,933,
937,940,944,947,950,953,957,960,963,966,968,971,974,977,979,982,
984,986,989,991,993,995,997,999,1001,1003,1004,1006,1008,1009,1011,1012,
1013,1014,1015,1017,1017,1018,1019,1020,1021,1021,1022,1022,1022,1023,1023,1023,
1023,1023,1023,1023,1022,1022,1022,1021,1021,1020,1019,1018,1017,1017,1015,1014,
1013,1012,1011,1009,1008,1006,1004,1003,1001,999,997,995,993,991,989,986,
984,982,979,977,974,971,968,966,963,960,957,953,950,947,944,940,
937,933,930,926,922,919,915,911,907,903,899,895,890,886,882,878,
873,869,864,860,855,850,846,841,836,831,826,821,816,811,806,801,
796,790,785,780,774,769,764,758,753,747,741,736,730,725,719,713,
707,701,696,690,684,678,672,666,660,654,648,642,636,630,624,617,
611,605,599,593,587,580,574,568,562,555,549,543,537,530,524,518,
512,505,499,493,486,480,474,468,461,455,449,443,436,430,424,418,
412,406,399,393,387,381,375,369,363,357,351,345,339,333,327,322,
316,310,304,298,293,287,282,276,270,265,259,254,249,243,238,233,
227,222,217,212,207,202,197,192,187,182,177,173,168,163,159,154,
150,145,141,137,133,128,124,120,116,112,108,104,101,97,93,90,
86,83,79,76,73,70,66,63,60,57,55,52,49,46,44,41,
39,37,34,32,30,28,26,24,22,20,19,17,15,14,12,11,
10,9,8,6,6,5,4,3,2,2,1,1,1,0,0,0,
0,0,0,0,1,1,1,2,2,3,4,5,6,6,8,9,
10,11,12,14,15,17,19,20,22,24,26,28,30,32,34,37,
39,41,44,46,49,52,55,57,60,63,66,70,73,76,79,83,
86,90,93,97,101,104,108,112,116,120,124,128,133,137,141,145,
150,154,159,163,168,173,177,182,187,192,197,202,207,212,217,222,
227,233,238,243,249,254,259,265,270,276,282,287,293,298,304,310,
316,322,327,333,339,345,351,357,363,369,375,381,387,393,399,406,
412,418,424,430,436,443,449,455,461,468,474,480,486,493,499,505
};

uint32_t dma_Blue_Destination = (uint32_t) &(TIM1->CCR1);
uint32_t dma_Green_Destination = (uint32_t) &(TIM1->CCR2);
uint32_t dma_Red_Destination = (uint32_t) &(TIM1->CCR3);
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_OC_Start(&htim3,TIM_CHANNEL_1);
  //HAL_DMA_Start_IT(&hdma_tim3_ch1_trig,(uint32_t)SineLookup,dma_Blue_Destination,512);
  HAL_DMA_Start_IT(&hdma_tim3_ch3,(uint32_t)SineLookup,dma_Green_Destination,512);
  HAL_DMA_Start_IT(&hdma_tim3_ch4_up,(uint32_t)SineLookup,dma_Red_Destination,512);
  //__HAL_TIM_ENABLE_DMA(&htim3,TIM_DMA_CC1);
  //__HAL_TIM_ENABLE_DMA(&htim3,TIM_DMA_CC3);
  //__HAL_TIM_ENABLE_DMA(&htim3,TIM_DMA_CC4);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  setRGB(1023,1023,1023);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //   if (f_updateLed){
    //   setRGB(1023,1023,1023);
    //   if(!read_b){
    //     setBlue(255);
    //   }
    //   if(!read_r){
    //     setRed(255);
    //   }
    //   if(!read_g){
    //     setGreen(255);
    //   }
    //   f_updateLed = 0;
    // }
    __WFI();
    // for(int i = 0; i < 1023; i ++){
    //   //HAL_Delay(10);
    //   setBlue(i);
    // }
    // setRGB(1023,1023,1023);
    // for(int i = 0; i < 1023; i ++){
    //   //HAL_Delay(10);
    //   setRed(i);
    // }
    // setRGB(1023,1023,1023);
    // for(int i = 0; i < 1023; i ++){
    //   //HAL_Delay(10);
    //   setGreen(i);
    // }
    // setRGB(1023,1023,1023);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t Pin)
{
  f_updateLed = 1;
  read_r = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7);
  read_g = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6);
  read_b = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5);
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
