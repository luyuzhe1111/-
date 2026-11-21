/* USER CODE BEGIN Header */
/*
 * 整车主逻辑阅读入口
 *
 * 1. 先找本文件的 StartDefaultTask()。它决定整车的先后顺序。
 *    上电的硬件初始化在 main.c。main() 创建下面两个任务，随后启动调度器。
 *
 *      main() -> MX_FREERTOS_Init() -> osKernelStart()
 *                                           |
 *                 +-------------------------+------------------------+
 *                 |                                                  |
 *          StartDefaultTask()                                  StartTask02()
 *       选区、算路、到站、做工位动作                      周期性读取底盘反馈并发轮速
 *
 * 2. StartDefaultTask() 的上半段只执行一遍。
 *    MainTask_PrepareStart() 等按键、底盘反馈和起点位姿确认。
 *    RadarStartup_Process() 推进雷达建链、扫描和规划。
 *    PathFollower_Start() 接收八段路线，准备在主循环中执行。
 *
 * 3. 同一个函数的 for (;;) 持续推进路线。
 *    走路时调用 PathFollower_Process()，到站后按 completed_stage 执行工位动作。
 *    MainTask_RunStageAction() 返回 1 后，PathFollower_Continue() 放行离站或收尾。
 *
 * 4. 看懂上面再按需要沿调用往下读。
 *    启动选区    -> 本文件 MainTask_PrepareStart()
 *    雷达和算路  -> bsp/scr/radar_startup.c -> bsp/scr/path_plan.c
 *    走路到站    -> bsp/scr/path_follower.c -> bsp/scr/chassis_runtime.c
 *    扫码和抓放  -> 本文件 MainTask_RunStageAction() -> QR_code.c 或 action.c
 *    轮速怎么算  -> bsp/scr/chassis_runtime.c -> bsp/scr/chassis_nav.c
 *
 * FreeRTOS 负责调度这些任务函数。单核 STM32 在就绪任务之间切换。
 * 本工程底盘任务优先级较高。它等待下一周期时，主任务可以继续运行。
 * osThreadDef 填写任务定义，osThreadCreate 创建任务，osKernelStart 开始调度。
 *
 * 调试从 g_app_start.status 开始。它描述整车进行到哪一步。
 * g_path_follower.status 描述该路段正在转向、前进还是等工位动作。
 * g_chassis_runtime.status 描述底盘当前命令是否完成。三个状态属于不同层。
 */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_config.h"
#include "chassis_runtime.h"
#include "path_follower.h"
#include "radar_startup.h"
#include "robot_task.h"
#include "action.h"
#include "QR_code.h"
#include <math.h>
#include "tim.h"
#include "actionInit.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 雷达只在主流程启动阶段扫描一次，区域由两个启动键同步选择。 */
#define RADAR_STARTUP_NODE_ID       1U
#define RADAR_STARTUP_TIMEOUT_MS    20000U
#define RADAR_STARTUP_POLL_MS       5U
#define START_KEY_DEBOUNCE_MS       20U
#define START_FEEDBACK_TIMEOUT_MS  5000U
#define START_POSE_TIMEOUT_MS      1000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
AppStart_State_t g_app_start;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static uint8_t MainTask_RunStageAction(uint8_t completed_stage);
static uint8_t MainTask_PrepareStart(void);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /*
   * 队列是可以保存多条消息的缓冲。主任务把位置校正、转向或位移命令放进去，
   * StartTask02() 在自己的周期中取命令执行。方向计算和串口轮询由底盘任务负责。
   */
  (void)ChassisRuntime_Create();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  /*
   * 参数依次是任务名、入口函数、优先级、实例字段和栈空间配置。
   * osThreadDef 只给出定义，下一行用这份定义创建任务。返回的 handle 用来标识任务。
   * 本工程传入 NULL，所以下面的 StartDefaultTask() 没有启动参数要读。
   */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  /*
   * myTask02 是 CubeMX 中的任务名，StartTask02 是真正执行的函数。
   * 两个任务都就绪时，底盘任务优先运行。它等待下一周期后，主任务继续执行。
   */
  osThreadDef(myTask02, StartTask02, osPriorityHigh, 0, 512);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}
extern void PutMaterialFromSlot(int number, uint8_t slot);
extern void PutMaterial2FromSlot(int number, uint8_t slot);
/* USER CODE BEGIN Header_StartDefaultTask */
/* 主任务按顺序等待选区、建立地图位姿、规划路线并驱动车辆。到站动作返回成功后，才允许继续下一段。 */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
__weak void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;

  /* 上半段只执行一次。选区成功后才执行雷达扫描和路径规划。 */
//雷达的启动扫描和规划***********************************************************************
  /* 先选择实际放车区域并建立地图位姿，不能先用1区路线规划再选2区。 */
  /* 主任务启动路线的第一道条件是选区和位姿准备成功。失败原因可在 g_app_start.fault 查看。 */
  if (MainTask_PrepareStart() != 0U)
  {
    g_app_start.status = APP_START_PLANNING;
  /*
   * 启动阶段只扫描一次全场障碍并完成A*规划。
   * 结果保存在g_radar_startup.path，成功后再进入正式比赛主流程。
   */
#if APP_RADAR_ENABLED
  /* app_config.h 当前选择这个编译分支。启动请求之后，要反复调用 Process 才能收发后续消息。 */
  (void)RadarStartup_Start(RADAR_STARTUP_NODE_ID,
                          g_app_start.selected_zone,
                          RADAR_STARTUP_TIMEOUT_MS);
  /* 雷达选区和扫描由多次串口收发组成。每次调用 Process 推进一步，中间用 osDelay 等下一轮。 */
  while (!RadarStartup_IsFinished())
  {
    RadarStartup_Process();
    osDelay(RADAR_STARTUP_POLL_MS);
  }
#else
  /* APP_RADAR_ENABLED 为 0 时走这个编译分支。规划用固定地图，新增障碍集合为空。 */
  (void)RadarStartup_PlanWithoutRadar();
#endif

  /* 先确认规划结果有效，再把路线交给执行器。两项都通过才进入 RUNNING。 */
  if (RadarStartup_IsSuccessful() && PathFollower_Start(RadarStartup_GetPath()))
  {
    /* 两种模式都使用同一个路径执行器，工位动作和离站流程不变。 */
    g_app_start.status = APP_START_RUNNING;
  }
  else
  {
    g_app_start.fault = RadarStartup_IsSuccessful() ?
                        APP_START_FAULT_FOLLOWER : APP_START_FAULT_PLAN;
    g_app_start.status = APP_START_ERROR;
    (void)ChassisRuntime_Stop();
  }
  }
		 /* 主任务在进入路径循环前还执行这一段 TIM3 通道 1 的定时脉宽序列。它位于上方启动成功判断之外。 */
		 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,1000);  
	 osDelay(700);
	 	 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,1500);  
	 osDelay(50);
  
//雷达的启动扫描和规划************************************************************************	
  /* 下面连续以双斜线开头的是历史台架示例，第一次读直接跳到下方的 for (;;) 循环。 */
//测试代码************************************************************************************


 
  
  
  
//	 QRcode_print_1();
//	/* 先启动MaixCam物料模式，并等待第一批有效坐标帧。 */
//	osDelay(100U);
//		Maixcam_setmode(5U);
//		(void)VisualPosition_Run(155U, 138, 90.0f, 1U, 8000U); //加工区，地图航向
//  	(void)VisualPosition_Run(78U, 93, 0.0f, 1U, 8000U);   //码垛
	//	(void)VisualPosition_Run(60U, 100, -90.0f, 1U, 8000U);   //原料区，地图航向
//	    ser_set(-69, 800);
//    osDelay(810U);
//	/* 视觉微调直接控制四轮，期间关闭任务2，避免USART3指令互相覆盖。 */
//	ChassisRuntime_SetEncoderQueryEnabled(false);
//	MotorSetPulse(5000, 1, 200, 200);
//	osDelay(1000);
//		put_material2(3);
//		put_material2(3);
//	put_material2(3);
////	osDelay(20U);
//	grab_material(3);
//	grab_material(2);
//	grab_material(3);
//	put_material(1);
//	put_material(2);
//	put_material(3);
//	grab_middle(1);
//	ChassisRuntime_SetEncoderQueryEnabled(true);
//VisualPosition_RunRawMaterial(RAW_GRAB_MIDDLE_X_PX, RAW_GRAB_MIDDLE_Y_PX,
//            0, RAW_GRAB_POSITION_TOLERANCE_PX,
//            RAW_GRAB_POSITION_TIMEOUT_MS);
//VisualPosition_Run(145U, 127U, 0.0f, 1U, 8000U);  //加工区定位点
//VisualPosition_Run(72U, 83U, 0.0f, 1U, 8000U);  //码垛定位点
//ChassisRuntime_SetEncoderQueryEnabled(false);
//    PutMaterial2FromSlot(1, 1U);
//    PutMaterial2FromSlot(2, 2U);
//    PutMaterial2FromSlot(3, 3U);

//    GrabMaterialToSlot(1, 1U);
//   GrabMaterialToSlot(2, 2U);
//    GrabMaterialToSlot(3, 3U);

//   RawGrab_Run(1,2,3);
//ChassisRuntime_SetEncoderQueryEnabled(true);
//  while(1)
//	{
//		osDelay(10);
//	}
//测试代码************************************************************************************
  /*
   * 下半段是整车主循环。for (;;) 表示一直重复下面的代码。
   * 每轮先推进走路状态，再判断是否到站或发生故障。
   * PathFollower_Process() 做一次检查并返回，所以后续判断能在同一轮继续执行。
   * 某些工位动作需要等待扫码或机械动作执行完，这一轮会在工位函数里等到它返回。
   */
  for(;;)
  {
    if (g_app_start.status == APP_START_RUNNING)
    {
      PathFollower_Process();
      /*
       * 一段路走完并完成需要的对接转向后，路径状态变成 WAIT_ACTION。
       * completed_stage 是刚走完的段号。例如值为 1 时，下面执行第一圈原料动作。
       */
      if (g_path_follower.status == PATH_FOLLOWER_WAIT_ACTION)
      {
        /*
         * 这里的 if 检查工位动作的返回值。返回 1 才调用 Continue()。
         * 原料、加工、暂存通常会进入离站状态，完成后再切下一段。
         * 返回 0 时保留 WAIT_ACTION。下一轮还会检查这个工位，是否重做由工位内部状态决定。
         */
        if (MainTask_RunStageAction(g_path_follower.completed_stage) != 0U)
        {
          (void)PathFollower_Continue();
        }
      }
      if (g_path_follower.status == PATH_FOLLOWER_FAULT)
      {
        g_app_start.fault = APP_START_FAULT_FOLLOWER;
        g_app_start.status = APP_START_ERROR;
      }
      else if (g_path_follower.status == PATH_FOLLOWER_DONE)
      {
        g_app_start.status = APP_START_DONE;
      }
    }
    else
    {
      /* 准备失败或全程结束时，不再推进路径。本任务继续在循环中等待。 */
    }
    /* 这一轮处理完后，主任务等待约 10 ms。工位函数的耗时会额外占用本轮时间。 */
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/* 底盘任务独立于主任务运行。即使主任务在等扫码或执行机械臂动作，这里的固定周期调度仍会继续。 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
__weak void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  uint32_t last_wake = osKernelSysTick();
  (void)argument;

  /*
   * 以下是另一个由调度器运行的循环。主任务在等待扫码或到站动作时，它仍按自己的节拍被调度。
   * 每轮先读四轮和 IMU，再据需要发命令。机构和底盘共用 USART3，机构占用时底盘周期仍计数，
   * ChassisRuntime_TaskStep() 会从暂停查询分支提前返回，把串口让给机构。
   */
  for(;;)
  {
    ChassisRuntime_TaskStep();
    /*
     * last_wake 保存上一计划唤醒时刻。osDelayUntil() 每次把计划往后移 20 ms。
     * 例如从 0 ms 开始，这一轮花 8 ms，就等待到约 20 ms；下一轮仍以 40 ms 为计划时刻。
     * &last_wake 把变量的地址交给函数，所以函数能更新它。步进函数耗时可看 bus_time_ms。
     */
    osDelayUntil(&last_wake,20U);
  }
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* 应用层入口按调用顺序阅读。MainTask_PrepareStart() 只建立软件位姿。MainTask_RunStageAction() 根据已完成的路径段执行对应工位动作。 */

/* 两键同时按下不选择，避免放在2区却误选1区；只读取一次并锁定。 */
static uint8_t MainTask_ReadStartKey(void)
{
    GPIO_PinState key1 = HAL_GPIO_ReadPin(START_KEY_GPIO_Port, START_KEY_Pin);
    GPIO_PinState key2 = HAL_GPIO_ReadPin(START_KEY2_GPIO_Port, START_KEY2_Pin);
    /* 上拉输入平时为高，按下接地后变低。两个键都按下或都松开时返回 0，主任务继续等。 */
    if (key1 == GPIO_PIN_RESET && key2 == GPIO_PIN_SET) return PP_START_ZONE_1;
    if (key2 == GPIO_PIN_RESET && key1 == GPIO_PIN_SET) return PP_START_ZONE_2;
    return 0U;
}

/*
 * 主任务启动前的三个关口，按代码从上往下读。
 *
 * WAIT_KEY      等 PD12 或 PD13 单键稳定按下，可以一直等。
 * WAIT_FEEDBACK 等底盘任务报告四轮位置和有效 IMU，最多等 5 秒。
 * WAIT_POSE     主任务先把校正命令放进队列，再等底盘确认，最多等 1 秒。
 *
 * 校正位姿是给已摆好的车标记地图 X、Y 和车头朝向。当前初始航向由 PP_START_YAW_DEG 指定。
 * 队列接受命令后，底盘任务还需取出并执行。pose_sequence 是这次请求的序号，
 * 等 completed_sequence 变成同一个值并核对坐标，才能确认准备完成。
 *
 * 准备成功返回 1，StartDefaultTask() 接着规划。失败先登记原因并请求停车，再返回 0。
 */
static uint8_t MainTask_PrepareStart(void)
{
    const PathPlan_StartProfile_t *profile;
    ChassisRuntime_State_t chassis;
    uint32_t wait_start;
    uint8_t zone;
    float yaw_error;

    g_app_start.status = APP_START_WAIT_KEY;
    g_app_start.fault = APP_START_FAULT_NONE;
    g_app_start.selected_zone = 0U;
    g_app_start.pose_sequence = 0U;
    for (;;) {
        zone = MainTask_ReadStartKey();
        if (zone != 0U) {
            /* 等待触点抖动结束后再读一次。确认两次读数相同才锁定该区。 */
            osDelay(START_KEY_DEBOUNCE_MS);
            if (MainTask_ReadStartKey() == zone) break;
        }
        osDelay(5U);
    }
    if (path_plan_select_start_zone(zone) == 0) {
        g_app_start.fault = APP_START_FAULT_BAD_ZONE;
        goto start_failed;
    }
    g_app_start.selected_zone = zone;
    /* profile 指向选区的数据，包括起点坐标、初始航向、离站点和回库点。 */
    profile = path_plan_get_start_profile();
    /* 这里等待底盘任务先建立里程计。四轮掩码需为 0x0F，IMU 要在线，状态还需处在可接受位姿修正的阶段。 */
    g_app_start.status = APP_START_WAIT_FEEDBACK;
    wait_start = HAL_GetTick();
    for (;;) {
        ChassisRuntime_GetState(&chassis);
        if (chassis.status == CHASSIS_RUNTIME_FAULT) {
            g_app_start.fault = APP_START_FAULT_CHASSIS;
            goto start_failed;
        }
        /* IMU有效后把当前原始读数对齐PP_START_YAW_DEG配置的地图航向。
         * 不要求IMU开机读数为0；先按配置角保持航向离站，再允许路径转向。
         */
        if (chassis.nav_initialized != 0U && chassis.encoder_query_enabled != 0U &&
            chassis.encoder_valid_mask == CHASSIS_NAV_ENCODER_ALL_VALID &&
            chassis.imu_valid != 0U && isfinite(chassis.imu_yaw_deg) &&
            (chassis.status == CHASSIS_RUNTIME_IDLE ||
             chassis.status == CHASSIS_RUNTIME_DONE)) break;
        if (HAL_GetTick() - wait_start >= START_FEEDBACK_TIMEOUT_MS) {
            g_app_start.fault = APP_START_FAULT_FEEDBACK_TIMEOUT;
            goto start_failed;
        }
        osDelay(5U);
    }
    /* 把车手工摆好的起点坐标和航向登记到地图。此调用只把校正请求入队，同时返回可跟踪的序号。 */
    if (!ChassisRuntime_CorrectPoseWithSequence(profile->initial_x_mm,
                                                profile->initial_y_mm,
                                                profile->initial_yaw_deg,
                                                &g_app_start.pose_sequence)) {
        g_app_start.fault = APP_START_FAULT_POSE_QUEUE;
        goto start_failed;
    }
    g_app_start.status = APP_START_WAIT_POSE;
    wait_start = HAL_GetTick();
    for (;;) {
        ChassisRuntime_GetState(&chassis);
        if (chassis.status == CHASSIS_RUNTIME_FAULT) {
            g_app_start.fault = APP_START_FAULT_CHASSIS;
            goto start_failed;
        }
        /* 航向差统一折回一圈的正负 180 度内。例如 +179 度和 -179 度只相差 2 度。 */
        yaw_error = chassis.nav.pose.yaw_deg - profile->initial_yaw_deg;
        if (isfinite(yaw_error)) {
            while (yaw_error > 180.0f) yaw_error -= 360.0f;
            while (yaw_error < -180.0f) yaw_error += 360.0f;
        }
        /* 等到底盘确认这条校正命令完成，还要核对读回位姿与目标相近，才放行扫描与规划。 */
        if (g_app_start.pose_sequence != 0U &&
            chassis.completed_sequence == g_app_start.pose_sequence &&
            (chassis.status == CHASSIS_RUNTIME_IDLE ||
             chassis.status == CHASSIS_RUNTIME_DONE) &&
            chassis.encoder_valid_mask == CHASSIS_NAV_ENCODER_ALL_VALID &&
            isfinite(chassis.nav.pose.x_mm) && isfinite(chassis.nav.pose.y_mm) &&
            isfinite(yaw_error) && fabsf(yaw_error) <= 1.0f &&
            hypotf(chassis.nav.pose.x_mm - profile->initial_x_mm,
                   chassis.nav.pose.y_mm - profile->initial_y_mm) <= 2.0f) return 1U;
        if (HAL_GetTick() - wait_start >= START_POSE_TIMEOUT_MS) {
            g_app_start.fault = APP_START_FAULT_POSE_TIMEOUT;
            goto start_failed;
        }
        osDelay(5U);
    }

/* 准备阶段失败统一走这里。写入 ERROR，再请求底盘停止，具体的故障值由前面的分支设置。 */
start_failed:
    g_app_start.status = APP_START_ERROR;
    (void)ChassisRuntime_Stop();
    return 0U;
}

/*
 * 工位右侧对接姿态的临时观察时间。
 * 正式运行设为0；需要观察对接方向时可临时改为1000毫秒。
 * 动作本身的完成状态决定何时继续路径。
 */
/* 正式动作已接入，取消旧的工位对接后额外1秒观察等待。 */
#define STAGE_DOCK_TEST_HOLD_MS 0U

/*
 * 六次工位动作各自的执行顺序，下标与completed_stage相同。
 * 每行依次是a、b、c；修改某行只影响那一次到站，不影响另一圈。
 * 原料行是颜色编号1~6：第一圈来自A1/B1/C1，第二圈来自A3/B3/C3。
 * 第一圈加工/暂存用A2/B2/C2；第二圈加工仍用A4/B4/C4。
 * 第二圈暂存按第1/2/3组扫码计算同色目标，不使用第4组加工顺序。
 * 原料按抓取先后存入槽1/2/3；加工、暂存参数a/b/c分别指定这三个槽
 * 应去的圆环位置。例：原料6/2/4、圆环3/1/2，则槽1颜色6放圆环3，
 * 槽2颜色2放圆环1，槽3颜色4放圆环2；加工取回时仍回到原槽。
 * 表在实际扫码或手动任务加载后生成，不再混用A1/A2/A3，也不把颜色号当槽号。
 */
static uint8_t s_stage_action_order[PP_MAX_STAGES][3] = {
    {0U, 0U, 0U}, /* 0：扫码，无机构顺序。 */
    {0U, 0U, 0U}, /* 1：第一圈原料，扫码第1组，颜色1~6。 */
    {0U, 0U, 0U}, /* 2：第一圈加工，扫码第2组，圆环1~3。 */
    {0U, 0U, 0U}, /* 3：第一圈暂存，扫码第2组，圆环1~3。 */
    {0U, 0U, 0U}, /* 4：第二圈原料，扫码第3组，颜色1~6。 */
    {0U, 0U, 0U}, /* 5：第二圈加工，扫码第4组，圆环1~3。 */
    {0U, 0U, 0U}, /* 6：第二圈暂存，同色匹配的目标圆环，第二层叠放。 */
    {0U, 0U, 0U}  /* 7：最终停车，无机构顺序。 */
};

/*
 * 工位执行入口。只在主任务看到 PATH_FOLLOWER_WAIT_ACTION 时调用。
 *
 * completed_stage 记录车刚走完哪一段，编号从 0 开始。
 * 0 扫码 -> 1 第一圈取料 -> 2 第一圈加工 -> 3 放第一层
 *        -> 4 第二圈取料 -> 5 第二圈加工 -> 6 放第二层 -> 7 回库结束
 *
 * switch 根据段号直接跳到对应的 case，case 在文件中的排放位置不影响跑车顺序。
 * 定位目标、抓取和放置的细节可沿对应 case 跳到 bsp/scr/action.c。
 *
 * 返回 1 表示向路径执行器报告本工位可以放行。返回 0 表示保留当前工位，
 * 原因可能是条件还未满足、任务编号无效或动作已经失败。查看对应分支的状态变量。
 * case 中的函数可能在本次调用中等待扫码或等待动作完成，这期间底盘任务仍可被调度。
 */
static uint8_t MainTask_RunStageAction(uint8_t completed_stage)
{
  uint8_t *order;
  uint8_t max_id;
  /* 只记录两次原料到站：下标0=第一圈、1=第二圈，失败不自动重抓。
   * 1=动作函数返回成功，非传感器确认抓牢；调试排障后复位重跑。
   */
  /* static 局部变量在函数返回后仍保留。这样下一轮回到同一原料工位时，能知道是否已经试抓过。 */
  static uint8_t raw_attempted[2];
  static uint8_t raw_done[2];

  if (completed_stage >= PP_MAX_STAGES) return 0U;
  /* order 指向本工位三个编号。原料段读取颜色，其他有机构动作的段读取圆环号。 */
  order = s_stage_action_order[completed_stage];
  max_id = (completed_stage == 1U || completed_stage == 4U) ? 6U : 3U;

  /* 避免无效编号导致原料忙等待永不退出，或抓放函数不执行却放行路径。 */
  if (completed_stage >= 1U && completed_stage <= 6U &&
      (order[0] < 1U || order[0] > max_id ||
       order[1] < 1U || order[1] > max_id ||
       order[2] < 1U || order[2] > max_id ||
       order[0] == order[1] || order[0] == order[2] || order[1] == order[2]))
  {
    return 0U; /* 顺序配置错误：保持WAIT_ACTION，不执行机构或继续路径。 */
  }

#if STAGE_DOCK_TEST_HOLD_MS > 0U
  static uint8_t hold_stage = 0xFFU;
  static uint32_t hold_start_ms = 0U;

  /* 阶段1~6都是原料/加工/暂存工位，先非阻塞保持对接姿态供观察。 */
  if (completed_stage >= 1U && completed_stage <= 6U &&
      STAGE_DOCK_TEST_HOLD_MS != 0U)
  {
    if (hold_stage != completed_stage)
    {
      hold_stage = completed_stage;
      hold_start_ms = HAL_GetTick();
      return 0U;
    }

    if ((uint32_t)(HAL_GetTick() - hold_start_ms) <
        STAGE_DOCK_TEST_HOLD_MS)
    {
      return 0U;
    }

    hold_stage = 0xFFU;
  }
#endif

  switch (completed_stage)
  {
    /* 第 0 段终点是扫码位。从四组三位数生成两圈动作顺序，第二层存放目标在这里按颜色提前计算。 */
    case 0U:
      /* 扫码区域：车身左侧朝向二维码板，底盘已经停车。 */
      if (g_storage_stack.fault != STORAGE_STACK_FAULT_NONE) return 0U;
      raw_attempted[0] = raw_attempted[1] = 0U;
      raw_done[0] = raw_done[1] = 0U;
#if APP_MANUAL_TASK_ENABLED
      /* 手动测试：不调用扫码等待；同样解码成A1~C4，复用后续流程。
       * 不改扫码接收标志E和中断接收缓冲，避免与实际接收互相覆盖。
       */
      openmv_shun_xu1 = APP_MANUAL_TASK_GROUP1;
      openmv_shun_xu2 = APP_MANUAL_TASK_GROUP2;
      openmv_shun_xu3 = APP_MANUAL_TASK_GROUP3;
      openmv_shun_xu4 = APP_MANUAL_TASK_GROUP4;
      if (!ShunXu_Decode(openmv_shun_xu1, 6U, &A1, &B1, &C1) ||
          !ShunXu_Decode(openmv_shun_xu2, 3U, &A2, &B2, &C2) ||
          !ShunXu_Decode(openmv_shun_xu3, 6U, &A3, &B3, &C3) ||
          !ShunXu_Decode(openmv_shun_xu4, 3U, &A4, &B4, &C4)) {
        g_storage_stack.fault = STORAGE_STACK_FAULT_BAD_SCAN;
        return 0U; /* 手动编号非法时保持停车，不能盲目执行机构。 */
      }
#else
      /* 实际扫码由 USART2 中断接收数据，这里等待结果并将四组三位数拆成 A1/B1/C1 等业务字段。 */
      QRcode_print_1();
#endif
      /*
       * 二维数组的第一维是阶段号，第二维是顺序里的第 1、2、3 项。
       * 例如第一组为 642 时，[1][0] 是颜色 6，第一件放入车载槽 1。
       * 第二组为 312 时，[2][0] 是圆环 3，所以加工时让槽 1 的物料去圆环 3。
       */
      /* 第一圈：第1组颜色取料，第2组圆环加工/暂存。 */
      s_stage_action_order[1][0] = A1;
      s_stage_action_order[1][1] = B1;
      s_stage_action_order[1][2] = C1;
      s_stage_action_order[2][0] = A2;
      s_stage_action_order[2][1] = B2;
      s_stage_action_order[2][2] = C2;
      s_stage_action_order[3][0] = A2;
      s_stage_action_order[3][1] = B2;
      s_stage_action_order[3][2] = C2;

      /* 第二圈：第3组颜色取料，第4组只用于加工区。 */
      s_stage_action_order[4][0] = A3;
      s_stage_action_order[4][1] = B3;
      s_stage_action_order[4][2] = C3;
      s_stage_action_order[5][0] = A4;
      s_stage_action_order[5][1] = B4;
      s_stage_action_order[5][2] = C4;
      /* 根据扫码快照保存第一层圆环颜色，并生成第二圈同色码垛目标。
       * 两圈颜色不同/编号非法时保持停车，Watch查看g_storage_stack.fault。
       */
      if (!StorageStack_Configure(s_stage_action_order[1],
                                   s_stage_action_order[3],
                                   s_stage_action_order[4])) return 0U;
      s_stage_action_order[6][0] = g_storage_stack.target_ring[0];
      s_stage_action_order[6][1] = g_storage_stack.target_ring[1];
      s_stage_action_order[6][2] = g_storage_stack.target_ring[2];
      return 1U;
    /* 第 1 段是第一圈原料。raw_attempted 记录是否已经执行，失败后留在当前工位等待排查。 */
    case 1U:
      /* 第一圈原料：第1组颜色，action.c完成排列识别、定位、左中右抓取、
       * 查询开关恢复和收臂。返回0保持WAIT_ACTION，不放行路径或重复抓取。
       */
      if (raw_attempted[0] != 0U) return raw_done[0];
      raw_attempted[0] = 1U;
      raw_done[0] = RawGrab_Run(order[0], order[1], order[2]);
      return raw_done[0];

    /* 第 4 段是第二圈原料。流程和第一圈相同，使用独立的完成标志与第 3 组颜色顺序。 */
    case 4U:
      /* 第二圈原料：第3组颜色；直接调用同一流程，完成标志仍与第一圈独立。 */
      if (raw_attempted[1] != 0U) return raw_done[1];
      raw_attempted[1] = 1U;
      raw_done[1] = RawGrab_Run(order[0], order[1], order[2]);
      return raw_done[1];

    /* 加工工位从这里调用圆环视觉对位。机械臂和旧底盘共用 USART3，执行机构动作时临时停用轮询，结束后恢复。 */
    case 2U:
      /* 第一圈加工：保持原视觉模式、定位坐标及+90度工位航向。 */
		  Maixcam_setmode(2U);
			ser_set(-70,1000);
	    osDelay(700);
		  /*
		   * 五个参数依次是目标 X/Y 像素、地图航向、像素容差和总等待毫秒数。
		   * 当前加工分支直接调用该函数，返回后就去执行 Processing_First()。
		   * 暂存分支的成功判定在 StorageStack_RunFirst() 内。
		   */
		  VisualPosition_Run(145U, 127U, 90.0f, 1U, 8000U);  //加工区定位点
		ChassisRuntime_SetEncoderQueryEnabled(false);
      /* 共享串口已让出，按本阶段顺序放置并取回。 */
      Processing_First(order[0], order[1], order[2]);
		ChassisRuntime_SetEncoderQueryEnabled(true);
			ser_set(-145,1000);
	    osDelay(700);
      return 1U;

    case 5U:
      /* 第二圈加工：定位与第一圈相同，动作顺序和入口独立。 */
      Maixcam_setmode(2U);
      ser_set(-70, 1000);
      osDelay(700U);
      /* 与第一圈加工相同，当前分支在视觉函数返回后直接继续抓放。 */
     VisualPosition_Run(145U, 127U, 90.0f, 1U, 8000U);  //加工区定位点
      ChassisRuntime_SetEncoderQueryEnabled(false);
      Processing_Second(order[0], order[1], order[2]);
      ChassisRuntime_SetEncoderQueryEnabled(true);
      /* 必须收回机构后返回1，之后路径状态机才允许离站。 */
      ser_set(-145, 1000);
      osDelay(700U);
      return 1U;

    /* 第 3 段用地面圆环做视觉定位，通过后放第一层并记录完成标志。 */
    case 3U:
      /* 第一圈暂存：原圆环模式和定位坐标不变，按第2组放第一层。
       * 定位失败不抓放、不放行；成功后记录first_layer_done。
       */
      return StorageStack_RunFirst() ? 1U : 0U;
    /* 第 6 段按圆环 2 第一层物料的颜色切换定位模式，定位成功后把三块物料叠在各自同色圆环。 */
    case 6U:
      /* 第二圈暂存：识别ring_color[2]对应物料，只定位成功才同色叠放。
       * 当前坐标在 action.c 的 StorageStack_RunLayer()。定位失败时会登记故障并保留本工位。
       */
      return StorageStack_RunSecond() ? 1U : 0U;
    /* 最后一段对应回库。这里确认结束后，路径模块进入 DONE，主任务写入 APP_START_DONE。 */
    case 7U:
      /* 所选启停区的返回动作已由路径模块完成。这里返回 1，交给 Continue() 结束全程。 */
      return 1U;

    default:
      /* 非法阶段不放行，车辆保持停车，便于调试发现问题。 */
      return 0U;
  }
}

/* USER CODE END Application */
