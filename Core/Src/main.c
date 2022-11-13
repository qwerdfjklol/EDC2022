/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "API.h"
#include "system.h"
#include "pid.h"
#include "jy62.h"
#include "move.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_Switch_Threshold 0.1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern float Setrpm[4];
extern pidParms MypidParms;
extern pidVars wheelpid[4];
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
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  //jy62_Init(&huart3);     //uart3作为和加速度计�?�信的串�????

  rpmpid_Init();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //u1_printf("sb\r\n");
    MoveBasic(0,-100,0);

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
int pwm_temp;
int count_temp;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim->Instance == TIM6)
  {
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2))
    {
      count_temp = __HAL_TIM_GET_COUNTER(&htim2) - 65535;
    }
    else{
      count_temp = __HAL_TIM_GET_COUNTER(&htim2);
    }
    pwm_temp = Getrpmpid(&MypidParms, &wheelpid[0], count_temp, Setrpm[0]);
    //u1_printf("%f %f\r\n",Setrpm[0],pwm_temp);
    if (pwm_temp > PWM_Switch_Threshold)
    {
      PBout(15) = 1;
      PBout(14) = 0;
    }
    else if (pwm_temp < -PWM_Switch_Threshold)
    {
      PBout(15) = 0;
      PBout(14) = 1;
      pwm_temp = -pwm_temp;
    }
    else{
      PBout(15) = 0;
      PBout(14) = 0;
      pwm_temp = 0;
    }
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm_temp);
    u2_printf("%d %d \r\n",__HAL_TIM_GET_COUNTER(&htim2),__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2));
    __HAL_TIM_SET_COUNTER(&htim2, 0);
  
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4))
    {
      count_temp = __HAL_TIM_GET_COUNTER(&htim4) - 65535;
    }
    else{
      count_temp = __HAL_TIM_GET_COUNTER(&htim4);
    }
    pwm_temp = Getrpmpid(&MypidParms, &wheelpid[1], count_temp, Setrpm[1]);
    if (pwm_temp > PWM_Switch_Threshold)
    {
      PBout(13) = 1;
      PBout(12) = 0;
    }
    else if (pwm_temp < -PWM_Switch_Threshold)
    {
      PBout(13) = 0;
      PBout(12) = 1;
      pwm_temp = -pwm_temp;
    }
    else{
      PBout(13) = 0;
      PBout(12) = 0;
      pwm_temp = 0;
    }
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm_temp);
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5))
    {
      count_temp = __HAL_TIM_GET_COUNTER(&htim5) - 65535;
    }
    else{
      count_temp = __HAL_TIM_GET_COUNTER(&htim5);
    }
    pwm_temp = Getrpmpid(&MypidParms, &wheelpid[2], count_temp, Setrpm[2]);
    if (pwm_temp > PWM_Switch_Threshold)
    {
      PCout(0) = 1;
      PCout(1) = 0;
    }
    else if (pwm_temp < -PWM_Switch_Threshold)
    {
      PCout(0) = 0;
      PCout(1) = 1;
      pwm_temp = -pwm_temp;
    }
    else{
      PCout(0) = 0;
      PCout(1) = 0;
      pwm_temp = 0;
    }
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm_temp);
    __HAL_TIM_SET_COUNTER(&htim5, 0);

    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim8))
    {
      count_temp = __HAL_TIM_GET_COUNTER(&htim8) - 65535;
    }
    else{
      count_temp = __HAL_TIM_GET_COUNTER(&htim8);
    }
    pwm_temp = Getrpmpid(&MypidParms, &wheelpid[3], count_temp, Setrpm[3]);
    if (pwm_temp > PWM_Switch_Threshold)
    {
      PCout(2) = 1;
      PCout(3) = 0;
    }
    else if (pwm_temp < -PWM_Switch_Threshold)
    {
      PCout(2) = 0;
      PCout(3) = 1;
      pwm_temp = -pwm_temp;
    }
    else{
      PCout(2) = 0;
      PCout(3) = 0;
      pwm_temp = 0;
    }
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwm_temp);
    __HAL_TIM_SET_COUNTER(&htim8, 0);
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
