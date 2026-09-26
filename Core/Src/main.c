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
#include "adc.h"
#include "can.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led.h"
#include "uart1.h"
#include "encoder.h"
#include "motor.h"
#include "jy61p.h"
#include "control.h"
#include "kinematics.h"
#include <stdio.h>
#include "SCServo.h"
#include "receive.h"
#include "OLED.h"
#include "oled_ui.h"
#include "PID.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t  g_uart3_receivedate = 0;  /* USART3(HWT905) 单字节接收缓冲 */
uint8_t  g_uart1_receivedate = 0;  /* USART1(蓝牙)   单字节接收缓冲 */
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
  MX_I2C3_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  
  UART1_Send_Str((uint8_t *)"Car System Ready!  cmd: #N v  |  #a v1 v2 v3 v4\r\n");
  Encoder_Init();                                 /* 启动四路编码器计数 */
  Motor_Init();                                   /* 启动四路电机 PWM 输出 */
  Kinematics_Init();
	
  HAL_TIM_Base_Start_IT(&htim6); //【必须手动加，开启定时器+中断】
  /* HWT905 九轴陀螺仪（USART3 PD8/PD9，模块波特率 115200）。
   * 平放时 |A| 应≈1.0g；对不上先查波特率，再查 jy61p.c 的换算系数。
   * 后续逐字节接收由 HAL_UART_RxCpltCallback 重武装（见 USER CODE BEGIN 4） */
  HAL_UART_Receive_IT(&huart3, &g_uart3_receivedate, 1);
  /* 视觉串口接收初始化 */
  Vision_UART_Init();

  /* XY 舵机 PID 控制初始化 */
  Servo_PID_Init();

  /* OLED 初始化 */
  OLED_Init();
  OLED_ShowString(0, 0, "Waiting...", OLED_6X8);
  OLED_Update();

  /* 非阻塞调度时间戳(毫秒) */
  uint32_t prev_led   = 0;
  uint32_t prev_servo = 0;
  uint32_t prev_oled  = 0;

  /* 蓝牙 PID 调试口（USART1 PA9/PA10，115200）：和上面一样先武装起来，
   * 每字节进 HAL_UART_RxCpltCallback -> Debug_RxByte()。
   * 指令与回传格式见 app/control.h */
  HAL_UART_Receive_IT(&huart1, &g_uart1_receivedate, 1);
  
  /* 电机自检：架空车轮后取消下面一行的注释即可运行（约 11 秒后四轮自动停转） */
  /* Motor_TestRun(); */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  
   LED_On(1);
   LED_On(2);
   LED_On(3);
   LED_On(4);
  HAL_Delay(3000);  /* 等待蓝牙连接，避免开机就发一堆垃圾 */
   LED_Off(1);
   LED_Off(2);
   LED_Off(3);
   LED_Off(4);
  while (1)
  {
    uint8_t txbuf[128];
    int     len;
	
//	  Motor_Load(0,0,100,0);
//	  Motor_Load(100,-100,+100,-100);
//   len = sprintf((char *)txbuf,
//                 "E1:%d  E2:%d "
//                 "E3:%d  E4:%d\r\n",
//                 (int)Encoder_GetDelta(ENC_WHEEL1),
//                 (int)Encoder_GetDelta(ENC_WHEEL2),
//                 (int)Encoder_GetDelta(ENC_WHEEL3),
//                 (int)Encoder_GetDelta(ENC_WHEEL4));
//   UART1_Send_Buf(txbuf, (uint16_t)len);

	  // //轮1
	  // len = sprintf((char *)txbuf,
		// 			"%d,%d,%d\r\n",
		// 			(int)Debug_Target[0], (int)Encoder_GetDelta(ENC_WHEEL1), (int)Debug_Pwm[0]);
	  // UART1_Send_Buf(txbuf, (uint16_t)len);

	  // //轮2
    // len = sprintf((char *)txbuf,
    //       "%d,%d,%d\r\n",
    //       (int)Debug_Target[1], (int)Encoder_GetDelta(ENC_WHEEL2), (int)Debug_Pwm[1]);
    // UART1_Send_Buf(txbuf, (uint16_t)len);
    // //轮3
    // len = sprintf((char *)txbuf,
    //       "%d,%d,%d\r\n",
    //       (int)Debug_Target[2], (int)Encoder_GetDelta(ENC_WHEEL3), (int)Debug_Pwm[2]);
    // UART1_Send_Buf(txbuf, (uint16_t)len);
//    //轮4
//    len = sprintf((char *)txbuf,
//          "%d,%d,%d\r\n",
//          (int)Debug_Target[3], (int)Encoder_GetDelta(ENC_WHEEL4), (int)Debug_Pwm[3]);
//    UART1_Send_Buf(txbuf, (uint16_t)len);

//	   len = sprintf((char *)txbuf,
//                  "T1:%d E1:%d P1:%d T2:%d E2:%d P2:%d "
//                  "T3:%d E3:%d P3:%d T4:%d E4:%d P4:%d\r\n",
//                  (int)Debug_Target[0], (int)Encoder_GetDelta(ENC_WHEEL1), (int)Debug_Pwm[0],
//                  (int)Debug_Target[1], (int)Encoder_GetDelta(ENC_WHEEL2), (int)Debug_Pwm[1],
//                  (int)Debug_Target[2], (int)Encoder_GetDelta(ENC_WHEEL3), (int)Debug_Pwm[2],
//                  (int)Debug_Target[3], (int)Encoder_GetDelta(ENC_WHEEL4), (int)Debug_Pwm[3]);
//    UART1_Send_Buf(txbuf, (uint16_t)len);
	  
//      Debug_Poll();   /* 20ms 回传 T/E/P，内部自带计时 */

      /* HWT905 示例（要看时取消注释）：数据由 USART3 中断直接填进
       * Ax/Ay/Az/Gx/Gy/Gz/Roll/Pitch/Yaw。取消注释后请把上面那行 Debug_Poll()
       * 也一并关掉，否则两者会抢同一条蓝牙口 */
      /* sprintf((char *)txbuf, "A:%.3f %.3f %.3f G:%.2f %.2f %.2f RPY:%.2f %.2f %.2f\r\n",
                 Ax, Ay, Az, Gx, Gy, Gz, Roll, Pitch, Yaw);
      UART1_Send_Str(txbuf); */
    /* XY 舵机: 每来一帧抓取数据算一次 PID(须在 OLED 块之前) */
    Servo_PID_Update();

    uint32_t now = HAL_GetTick();

    /* OLED 每 200ms 刷新一次(显示最新视觉数据) */
    if (now - prev_oled >= 200)
    {
        prev_oled = now;
        OLED_ShowVision();
        vision_data.qr_flag = 0;
        vision_data.track_flag = 0;
        vision_data.turn_flag = 0;
    }

    /* LED1 闪烁: 每 500ms 翻转(亮500ms灭500ms) */
    if (now - prev_led >= 500)
    {
        prev_led = now;
        LED_Toggle(1);
        WritePosEx(2, 1600, 5, 0);
    }

    /* 舵机指令 + LED4: 约每 1s 一次 */
    if (now - prev_servo >= 1000)
    {
        prev_servo = now;
        // WritePosEx(1, 2680, 5, 0);
        // WritePosEx(2, 3560, 5, 0);
        // WritePosEx(3, 700, 5, 0);
        LED_Toggle(4);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// 放在USER CODE BEGIN 4这个代码段！！CubeMX重新生成不会删掉这里！
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if( huart == &huart3 )
    {
        jy61p_ReceiveData(g_uart3_receivedate);
        HAL_UART_Receive_IT(&huart3,&g_uart3_receivedate,1);
    }
    else if( huart == &huart1 )
    {
        Debug_RxByte(g_uart1_receivedate);
        HAL_UART_Receive_IT(&huart1,&g_uart1_receivedate,1);
    }
		 else if (huart == &huart4)
    {
        Vision_UART_RxCpltCallback();
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
