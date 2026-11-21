/* USER CODE BEGIN Header */
/*
 * 整车程序入口。上电先准备时钟、GPIO、定时器和串口，再初始化外设接收与机构，最后启动 FreeRTOS。
 * 读代码时先看 main()，启动调度器后继续看 freertos.c 的 StartDefaultTask() 和 StartTask02()。
 */
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
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_config.h"
#include "bsp_usart.h"
#include "hwt101.h"
#include "Emm_V5.h"
#include "FSUS.h"
#include "actionInit.h"
#include "maixcam.h"
#include "QR_code.h"
#include "chassic.h"
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
void MX_FREERTOS_Init(void);
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
  /* 复位后先准备 HAL 和毫秒时基，外设初始化过程里的延时、超时判断都可以使用这个时基。 */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* 先配置普通引脚和 DMA 时钟，再启动定时器、串口。串口初始化会给对应引脚设置复用功能。 */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  /* UART5 的硬件初始化在这里执行。是否发出雷达握手和扫描指令由主任务中的 APP_RADAR_ENABLED 决定。 */
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  /*
   * 下面的初始化会实际发出停止指令和机构归位指令。选区按钮由主任务读取，到这里时还没有开始按键等待。
   * uart_init_User() 只负责摄像头和扫码接收；UART4 舵机、USART6 航向传感器在下面分别开启接收。
   */
  /* HWT101 提供原始航向。上电发设备清零命令，之后主任务用选区配置对齐地图航向。 */
	
	//基本配置初始化
	/* 挂起 USART1/2 的首次接收。接收请求会立刻返回，真正收到一个字节时由中断通知解析函数。 */
	uart_init_User();
	User_Uart_Init(&huart4);           //FUSH舵机
  /* USART6 用 DMA 把一段串口字节搬到 RXbuf6。收到空闲事件后，HAL 给解析函数传入本次长度。 */
  HWT101_ResReceiveToIdle_DMA();     //IMU串口
	//基本的运动初始化 
  Hwt101_Yaw_Res();
	/* 地址 0 用于发送停止指令。随后 action_Init() 将机械臂、舵机和 PWM 输出设到程序规定的初始状态。 */
	Emm_V5_Stop_Now(0,false);
	action_Init();

  /* USER CODE END 2 */

  /* 准备好硬件后创建工程的两个任务。函数实现在 freertos.c，整车业务也从那里开始读。 */
  MX_FREERTOS_Init();

  /* Start scheduler */
  /*
   * osKernelStart() 让 FreeRTOS 接管任务调度。
   * 读整车顺序时下一站是 freertos.c 的 StartDefaultTask()。
   * 看轮子如何更新则读同一个文件的 StartTask02()。下面的空循环用于调度器异常返回的情况。
   */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  /* 以工程配置的 8 MHz HSE 为例，先除 4，再乘 168，最后除 2，得到 168 MHz 系统时钟。 */
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
/* 硬件定时器周期回调见下方。TIM6 用来更新 HAL 的毫秒计数，HAL_GetTick() 和 HAL_Delay() 都依赖这个计数。 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    /* TIM6 更新中断使 HAL 毫秒计数加一。相机和 IMU 新鲜度、串口超时等都可以读取这个计数。 */
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
