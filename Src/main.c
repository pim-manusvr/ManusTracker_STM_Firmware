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
#include "states.h"

//#define LEDPASSTHROUGH

uint16_t f_updateLed = 0;
uint8_t read_r = 0;
uint8_t read_g = 0;
uint8_t read_b = 0;
static uint8_t g_trackerState = TRACKER_ON;
uint8_t g_oldTrackerState = TRACKER_ON;
uint32_t lastIT = 0;
uint32_t currentIT = 0;
uint16_t timeThreshold = 500;

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
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_OC_Start(&htim3,TIM_CHANNEL_1);
  HAL_DMA_Start_IT(&hdma_tim3_ch1_trig,(uint32_t)BreathingLookup,dma_Blue_Destination,512);
  HAL_DMA_Start_IT(&hdma_tim3_ch3,(uint32_t)BreathingLookup,dma_Green_Destination,512);
  HAL_DMA_Start_IT(&hdma_tim3_ch4_up,(uint32_t)BreathingLookup,dma_Red_Destination,512);
  //__HAL_TIM_ENABLE_DMA(&htim3,TIM_DMA_CC1);
  //__HAL_TIM_ENABLE_DMA(&htim3,TIM_DMA_CC3);
  //__HAL_TIM_ENABLE_DMA(&htim3,TIM_DMA_CC4);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  setRGB(1024,1024,1024);
  __HAL_TIM_ENABLE_DMA(&htim3,TIM_DMA_CC1);
  __HAL_TIM_ENABLE_DMA(&htim3,TIM_DMA_CC3);
  HAL_Delay(2000);

  timeThreshold = 0;
  HAL_GPIO_EXTI_Callback(GPIO_PIN_1);
  timeThreshold = 250;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    #ifdef LEDPASSTHROUGH
      if (f_updateLed){
      setRGB(1023,1023,1023);
      if(!read_b){
        setBlue(255);
      }
      if(!read_r){
        setRed(255);
      }
      if(!read_g){
        setGreen(255);
      }
      f_updateLed = 0;
    }
    #else
      if(f_updateLed){
        //setRGB(1023,1023,1023);
        switch(g_trackerState){
          case TRACKER_PAIRING:
            setRGB(1024,1024,1024);
            resetDMAIT((uint32_t)BlinkLookup);
            __HAL_TIM_ENABLE_DMA(&htim3,TIM_DMA_CC1);
            __HAL_TIM_ENABLE_DMA(&htim3,TIM_DMA_CC3);
            __HAL_TIM_ENABLE_DMA(&htim3,TIM_DMA_CC4);
          break;
          case TRACKER_CONNECTED:
            __HAL_TIM_DISABLE_DMA(&htim3,TIM_DMA_CC1);
            __HAL_TIM_DISABLE_DMA(&htim3,TIM_DMA_CC3);
            __HAL_TIM_DISABLE_DMA(&htim3,TIM_DMA_CC4);
            setRGB(1024,100,100);
          break;
          case TRACKER_ON:
            setRGB(1024,1024,1024);
            resetDMAIT((uint32_t)BreathingLookup);
            __HAL_TIM_DISABLE_DMA(&htim3,TIM_DMA_CC1);
            __HAL_TIM_DISABLE_DMA(&htim3,TIM_DMA_CC3);
            __HAL_TIM_DISABLE_DMA(&htim3,TIM_DMA_CC4);
            __HAL_TIM_ENABLE_DMA(&htim3,TIM_DMA_CC1);
            __HAL_TIM_ENABLE_DMA(&htim3,TIM_DMA_CC3);
          break;
          case TRACKER_NOT_CONNECTED:
            __HAL_TIM_DISABLE_DMA(&htim3,TIM_DMA_CC1);
            __HAL_TIM_DISABLE_DMA(&htim3,TIM_DMA_CC3);
            __HAL_TIM_DISABLE_DMA(&htim3,TIM_DMA_CC4);
            setRGB(100,100,100);
          break;
        }
      f_updateLed = 0;
      }
    #endif
    __WFI();
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t Pin)
{
  g_oldTrackerState = g_trackerState;
  currentIT = HAL_GetTick();
  uint32_t deltaTimeIT = currentIT - lastIT;
  read_r = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7);
  read_g = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6);
  read_b = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5);

  if(deltaTimeIT <= timeThreshold)
  {
    HAL_TIM_Base_Stop_IT(&htim14);
    HAL_TIM_Base_Start_IT(&htim14);
    if (read_b == 0 && read_r && read_g){
    g_trackerState = TRACKER_PAIRING;
    }
    else if (read_b && read_g && read_r){
      g_trackerState = TRACKER_PAIRING;
    }
  }
  else
  {
    if(read_g == 0 && read_r && read_b)
    {
      g_trackerState = TRACKER_CONNECTED;
    }
    else if(read_b == 0 && read_r && read_g )
    {
      g_trackerState = TRACKER_NOT_CONNECTED;
    }
    else if (read_b && read_g && read_r ){
      g_trackerState = TRACKER_ON;
    }
  }
  
  lastIT = currentIT;
  if(g_oldTrackerState != g_trackerState)
  {
      f_updateLed = 1;
  }
}

void resetDMAIT(uint32_t lookupTable){
  HAL_DMA_Abort_IT(&hdma_tim3_ch1_trig);
  HAL_DMA_Abort_IT(&hdma_tim3_ch3);
  HAL_DMA_Abort_IT(&hdma_tim3_ch4_up);
  HAL_DMA_Start_IT(&hdma_tim3_ch1_trig,(uint32_t)lookupTable,dma_Blue_Destination,512);
  HAL_DMA_Start_IT(&hdma_tim3_ch3,(uint32_t)lookupTable,dma_Green_Destination,512);
  HAL_DMA_Start_IT(&hdma_tim3_ch4_up,(uint32_t)lookupTable,dma_Red_Destination,512);
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
