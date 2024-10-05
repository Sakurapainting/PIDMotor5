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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "pid.h"
#include "Serial.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 5  // 定义接收缓冲区大小
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


uint8_t RxBuffer[RX_BUFFER_SIZE];  // 接收缓冲区

float Speed = 0;

PID speedpid;
float Encoder_Speed = 0;
float Target_val = 200;  

PID positionpid;
float Position = 0;

int16_t RxNUM = 0;

int16_t m, u1;
float PI = 3.1415926, u;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Motor_SetSpeed(int16_t Speed){
	if(Speed >= 0){
		// GPIO_SetBits(GPIOA, GPIO_Pin_4);
		// GPIO_ResetBits(GPIOA, GPIO_Pin_5);
		// PWM_SetCompare3(Speed);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, Speed);
	}
	else{
		// GPIO_ResetBits(GPIOA, GPIO_Pin_4);
		// GPIO_SetBits(GPIOA, GPIO_Pin_5);
		// PWM_SetCompare3(-Speed);		
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, -Speed);
	}
}

//获得电机的脉�?????
int16_t Encoder_Get(void){
	int16_t Temp;
	Temp = __HAL_TIM_GET_COUNTER(&htim3); //获取编码器当前�??
	__HAL_TIM_SET_COUNTER(&htim3, 0);     //将编码器计数器清0
	return Temp;
}

void MotorControl(void){
	Encoder_Speed = Encoder_Get();  //1.获取电机10ms的脉冲数(encoder t1 + t2, 未除2)。即10ms获取的脉冲数。即速度�?????
  Position += Encoder_Speed ;  //累计实际脉冲数

	Speed = PID_Incremental_Calc(&speedpid,Target_val ,Encoder_Speed);

  // Speed = PID_Position_Calc(&positionpid, Target_val, Position);   

	Motor_SetSpeed(Speed);  //3.PWM输出给电�?????
}

//TIM1中断函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim == &htim1){
    MotorControl();
  }
}

//usart1中断函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  // if(huart == &huart1){
    // Serial_RxFlag = 1;
  //   HAL_UART_Receive_IT(&huart1, &ByteRecv, 1);
  // }

  if(huart == &huart1){
    if(ByteRecv == 'a'){
      Target_val += 10;
      Serial_printf("Target_val:%f\n", Target_val);
      HAL_UART_Receive_IT(&huart1, &ByteRecv, 1);
    }
    else if(ByteRecv == 's'){
      Target_val -= 10;
      Serial_printf("Target_val:%f\n", Target_val);
      HAL_UART_Receive_IT(&huart1, &ByteRecv, 1);
    }
    else if(ByteRecv == 'd'){
      RxNUM = 0;
      ByteRecv = 'f';
      HAL_UART_Receive_IT(&huart1, RxBuffer, 5);
    }
    else if(ByteRecv == 'f'){
      uint8_t i;
      for(i = 0; i < 5; i++){
          RxNUM += ((int16_t)(RxBuffer[i] - '0')) * Serial_Pow(10, 4 - i);
          // Serial_printf("RxNUM:%d\n", RxNUM);
      }
      
      Serial_RxFlag = 1;
      HAL_UART_Receive_IT(&huart1, &ByteRecv, 1);
    }
    
  }

  // if(huart == &huart1){
  //   Serial_RxFlag = 1;  // 设置接收完成标志

  //   HAL_UART_Receive_IT(&huart1, RxBuffer, 1);  // 继续接收下一个字节
  // }
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
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();

  PID_Init(&speedpid, 60, 30, 1, 10000);
  PID_Init(&positionpid, 20, 0, 2, 10000);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  Motor_SetSpeed(2000);

  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1 | TIM_CHANNEL_2);

  HAL_TIM_Base_Start_IT(&htim1);

  OLED_ShowString(1, 1, "Speed:");
  // OLED_ShowString(1, 13, "r/s");
  
  HAL_UART_Receive_IT(&huart1, &ByteRecv, 1);   //start itrpt receive a byte
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //Speed 单位(13xian, 减su比1:20, itrpt 10ms, rising & falling edge record twice): round/s
    // OLED_ShowSignedNum(1, 7, (Encoder_Speed*100/13/20/2), 5);
    // OLED_ShowSignedNum(2, 7, (Speed*100/13/20/2), 5);
    OLED_ShowSignedNum(1, 7, (Encoder_Speed), 5);
    OLED_ShowSignedNum(2, 7, (Speed), 5);

    Serial_printf("Speed, Enc_spd, RxNUM:%f,%f,%d\n", Speed, Encoder_Speed, RxNUM); //Speed is duty, Encoder_Speed is pulse
    //position 单位 target_val / 20 /2 *27.7 = angle
    //RxNUM is angle
    if(Serial_GetRxFlag() == 1){
      Target_val = RxNUM /27.7 * 2 * 20;
      // Serial_printf("RxNUM:%d\n", RxNUM);
      Serial_printf("Target_val:%f\n", Target_val);
    }

    // if(Serial_GetRxFlag() == 1){
    //   if(ByteRecv == 'a'){
    //     Target_val += 10;
    //   }
    //   else if(ByteRecv == 's'){
    //     Target_val -= 10;
    //   }
    //   Serial_printf("Target_val:%f\n", Target_val);
    // }

    //sin
    for(m = 0; m <= 628; m++){  //2*PI = 628
      u = sin(m/100.0);
      u1 = (int16_t)(u*10000);

      OLED_ShowSignedNum(1, 7, (Encoder_Speed), 5);
      OLED_ShowSignedNum(2, 7, (Speed), 5);
      Serial_printf("Speed, Enc_spd, RxNUM:%f,%f,%d\n", Speed, Encoder_Speed, RxNUM);

      HAL_Delay(10);
      // OLED_ShowSignedNum(3, 7, u1, 5);
      Serial_printf("u1:%d\n", u1);
      Target_val = u1;
    }
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
