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
#include "jy62.h"
#include "move.h"
#include "qmc5883.h"
#include "delay.h"
#include "zigbee_edc24.h"
#include "dijkstra.h"
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
uint8_t rxData[JY_BUF_SIZE << 1];                     // Rx buffer for JY62
extern int WheelCounts[4];
extern WheelPidVars WheelPid[4];
extern PidParms WheelPidParms;
extern PidParms YawPidParms;
extern StateInfo AxisData;
struct JY62_Mes JY62;
struct JY62_Data jy62data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void JY_handler(uint8_t *);
void Accdecode(uint8_t *rx);
void Velodecode(uint8_t *rx);
void Angledecode(uint8_t *rx);
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
  MX_UART5_Init();
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
  delay_init();
  HAL_UART_Receive_DMA(&huart3, rxData, JY_BUF_SIZE << 1);

  delay_ms(2000);

  jy62_Init(&huart3);     //uart3作为和加速度计�?�信的串�???????????
  InitPid();
  WheelPidParms.kp = 1.2;
  WheelPidParms.kd = 3.8;
  WheelPidParms.ki = 1;
  YawPidParms.kp = 1.2;
  YawPidParms.kd = 3.8;
  YawPidParms.ki = 1;
  HAL_UART_Receive_IT(&huart2,Message,16);
  SetBaud(115200);
  SetHorizontal();
  InitAngle();
  Calibrate();
  //SleepOrAwake();
  QMC5883_InitConfig();
  //zigbee_Init(&huart3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  delay_ms(2000);
  //MoveByAbs(50, 90);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    InitNode();
    BuildGraph();
    Dijkstra();
    shortest_path1();
    MoveByAbs(50,angles[0]);
    //jy62_Init(&huart3); 
    //delay_ms(10); 
    //Barrier_edc24 b=getOneBarrier(0); 
    //u2_printf("sb");
    //u2_printf("yaw:%6f    roll:%6f    pitch:%6f\r\n",jy62data.yaw,jy62data.roll,jy62data.pitch);
    //Barrier_edc24 pos=getOneBarrier(0);
    //u2_printf("%d",pos.pos_1.x);
    delay_ms(1000);
    //float asdfg[3];
    //dijkstra();
    //QMC5883_GetData(asdfg);
    u2_printf("%f %f %f %f\r\n", JY62.angleyaw, JY62.velox, JY62.veloy, JY62.accz);
    //QMC5883_InitConfig();
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim->Instance == TIM6)
  {
    
    WheelCounts[0] = __HAL_TIM_GET_COUNTER(&htim2);
    WheelCounts[1] = __HAL_TIM_GET_COUNTER(&htim4);
    WheelCounts[2] = __HAL_TIM_GET_COUNTER(&htim5);
    WheelCounts[3] = __HAL_TIM_GET_COUNTER(&htim8);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    __HAL_TIM_SET_COUNTER(&htim8, 0);

    SetStateInfo(jy62data.accx, jy62data.accy, jy62data.accz, jy62data.velox, jy62data.veloy, jy62data.veloz, jy62data.pitch, jy62data.yaw, jy62data.roll);
    YawPidCalculate();
    WheelPidCalculate();

    if (WheelPid[0].pwm >= 0)
    {
      PBout(15) = 1;
      PCout(14) = 0;
    }
    else
    {
      PBout(15) = 0;
      PCout(14) = 1;
      WheelPid[0].pwm = -WheelPid[0].pwm;
    }
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, WheelPid[0].pwm);
    if (WheelPid[1].pwm >= 0)
    {
      PBout(13) = 1;
      PBout(12) = 0;
    }
    else
    {
      PBout(13) = 0;
      PBout(12) = 1;
      WheelPid[1].pwm = -WheelPid[1].pwm;
    }
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, WheelPid[1].pwm);
    if (WheelPid[2].pwm >= 0)
    {
      PCout(0) = 1;
      PCout(1) = 0;
    }
    else
    {
      PCout(0) = 0;
      PCout(1) = 1;
      WheelPid[2].pwm = -WheelPid[2].pwm;
    }
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, WheelPid[2].pwm);
    if (WheelPid[3].pwm >= 0)
    {
      PCout(2) = 1;
      PCout(3) = 0;
    }
    else
    {
      PCout(2) = 0;
      PCout(3) = 1;
      WheelPid[3].pwm = -WheelPid[3].pwm;
    }
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, WheelPid[3].pwm);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == SigIn_Pin)
  {
    PAout(5) = 1;
  }
}

void JY_handler(uint8_t *rx) {
  Angledecode(rx);
  Velodecode(rx);
  Accdecode(rx);
}

void Accdecode(uint8_t *rx){   //加�?�度解码
  static uint8_t sum = SUMACC, p = 0;
  uint8_t rxi;
  for (uint8_t i = 0; i < JY_BUF_SIZE; ++i) {
    rxi = rx[i];
    switch (p) {
      case 0:
        p = rxi == 0x55u ? 1 : 0;
        break;
      case 1:
        p = rxi == 0x51u ? 2 : 0;
        break;
      case 6:
      case 7:
        JY62.accz[p - 6] = rxi;
        sum += rxi;
        ++p;
        break;
      case 2:
      case 3:
        JY62.accx[p-2]=rxi;
        sum += rxi;
        ++p;
        break;
      case 4:
      case 5:
        JY62.accy[p-4]=rxi;
        sum += rxi;
        ++p;
        break;
      case 8:
      case 9:
        JY62.temperature[p-8]=rxi;
        sum += rxi;
        ++p;
        break;
      case 10:
        if (rxi == sum) {
          jy62data.accxRaw=calAcc(JY62.accx[0],JY62.accx[1]);
          jy62data.accyRaw=calAcc(JY62.accy[0],JY62.accy[1]);
          jy62data.acczRaw=calAcc(JY62.accz[0],JY62.accz[1]);
          //处理数据
          p=0;
          sum=SUMACC;
          return;
        } 
        else {
          i = i >= 9 ? i - 9 : 0;
        }
        // no break
      default:
        p = 0;
        sum = SUMACC;
    }
  }
}

void Velodecode(uint8_t *rx){    //角�?�度解码
  static uint8_t sum = SUMVELO, p = 0;
  uint8_t rxi;
  for (uint8_t i = 0; i < JY_BUF_SIZE; ++i) {
    rxi = rx[i];
    switch (p) {
      case 0:
        p = rxi == 0x55u ? 1 : 0;
        break;
      case 1:
        p = rxi == 0x52u ? 2 : 0;
        break;
      case 6:
      case 7:
        JY62.veloz[p - 6] = rxi;
        sum += rxi;
        ++p;
        break;
      case 2:
      case 3:
        JY62.velox[p - 2] = rxi;
        sum += rxi;
        ++p;
        break;
      case 4:
      case 5:
        JY62.veloy[p - 4] = rxi;
        sum += rxi;
        ++p;
        break;
      case 8:
      case 9:
        JY62.temperature[p - 8]=rxi;
        sum += rxi;
        ++p;
        break;
      case 10:
        if (rxi == sum) {
          jy62data.veloxRaw=calVelox(JY62.velox[0],JY62.velox[1]);
          jy62data.veloyRaw=calVelox(JY62.veloy[0],JY62.veloy[1]);
          jy62data.velozRaw=calVelox(JY62.veloz[0],JY62.veloz[1]);
          //处理数据
          p=0;
          sum=SUMVELO;
          return;
        } 
        else {
          i = i >= 9 ? i - 9 : 0;
        }
        // no break
      default:
        p = 0;
        sum = SUMVELO;
    }
  }
}

void Angledecode(uint8_t *rx){
  static uint8_t sum = SUMANGLE, p = 0;
  uint8_t rxi;
  for (uint8_t i = 0; i < JY_BUF_SIZE; ++i) {
    rxi = rx[i];
    switch (p) {
      case 0:
        p = rxi == 0x55u ? 1 : 0;
        break;
      case 1:
        p = rxi == 0x53u ? 2 : 0;
        break;
      case 6:
      case 7:
        JY62.angleyaw[p - 6] = rxi;
        sum += rxi;
        ++p;
        break;
      case 2:
      case 3:
        JY62.angleroll[p-2] = rxi;
        sum += rxi;
        ++p;
        break;
      case 4:
      case 5:
        JY62.anglepitch[p-4]=rxi;
        sum += rxi;
        ++p;
        break;
      case 8:
      case 9:
        JY62.temperature[p-8]=rxi;
        sum += rxi;
        ++p;
        break;
      case 10:
        if (rxi == sum) {
          float z = calcAngle(JY62.angleyaw[0], JY62.angleyaw[1]);
          float x = calcAngle(JY62.angleroll[0],JY62.angleroll[1]);
          float y = calcAngle(JY62.anglepitch[0],JY62.anglepitch[1]);

          float nextYawBias = jy62data.yawBias;
          float nextRollBias = jy62data.rollBias;
          float nextPitchBias = jy62data.pitchBias;
          if (z > jy62data.yawRaw + 180) {
            nextYawBias -= 360;
          } 
          else if (z < jy62data.yawRaw - 180) {
            nextYawBias += 360;
          }
          if (y > jy62data.pitchRaw + 180) {
            nextPitchBias -= 360;
          } 
          else if (y < jy62data.pitchRaw - 180) {
            nextPitchBias += 360;
          }
          if (x > jy62data.rollRaw + 180) {
            nextRollBias -= 360;
          } 
          else if (x < jy62data.rollRaw - 180) {
            nextRollBias += 360;
          }
          jy62data.yawRaw = z;
          jy62data.pitchRaw=y;
          jy62data.rollRaw=x;
          float nextYaw = jy62data.yawRaw + nextYawBias - jy62data.yawZero;
          float nextRoll = jy62data.rollRaw + nextRollBias - jy62data.rollZero;
          float nextPitch = jy62data.pitchRaw + nextPitchBias - jy62data.pitchZero;
          if (abs(jy62data.yaw - nextYaw) < 180) {
            jy62data.yaw = nextYaw;
            jy62data.yawBias = nextYawBias;
          }
          if (abs(jy62data.pitch - nextPitch) < 180) {
            jy62data.pitch = nextPitch;
            jy62data.pitchBias = nextPitchBias;
          }
          if (abs(jy62data.roll - nextRoll) < 180) {
            jy62data.roll = nextRoll;
            jy62data.rollBias = nextRollBias;
          }
          p = 0;
          sum = SUMANGLE;
          return;
        } 
        else {
          i = i >= 9 ? i - 9 : 0;
        }
        // no break
      default:
        p = 0;
        sum = SUMANGLE;
    }
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
