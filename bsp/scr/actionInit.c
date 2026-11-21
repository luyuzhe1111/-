/* 封装机构的基础动作。这里的编号和位置值对应实车执行器的标定，工位层通过这些接口组合一系列动作。 */

#include "actionInit.h"
#include "usart.h"
#include "tim.h"
#include "FSUS.h"
#include "Emm_V5.h"
#include "cmsis_os.h"
#include "maixcam.h"

/* 上电执行的机构初始姿态。此函数会驱动舵机与步进轴，并向 MaixCam 发送物料模式指令。 */
void action_Init(void)
 {	 
  /* 比较值决定每个 PWM 周期的高电平持续时间。这里先给夹爪和存料盘设置上电初始值。 */
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,1300);  //夹紧950   完全打开1300    
  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,700);//tim3 ch2 1550初始化  

	
  ser_set(-145,1000);   
  ZDT_set(6,1300);
	HAL_Delay(500);
	ZDT_set(1,0);
	 /* 归位步骤中发送 FF 01 FE 让摄像头进入物料识别模式，发送通过 USART1。 */
	 Maixcam_setmode(1U);
	HAL_Delay(300);
	 

 }

/* 将机构编号和目标脉冲值交给 Emm 位置驱动。现函数只处理地址 1 和 6，各地址使用自己的运动方向及参数。 */
void ZDT_set(uint8_t id , uint16_t num)
{
	if(id==1)
	{
		Emm_V5_Pos_Control(1,1, 230,240,num, true, false); 
	}
 	if(id==6)
	{
	 Emm_V5_Pos_Control(6,0, 230,240,num, true, false); 
	}
}
 
 
/* Faster motor profile used only by coarse-processing retrieval. */
/* 加工取料使用的另一套步进运动参数，和 ZDT_set() 共享地址但速度参数独立。 */
void ZDT_set2(uint8_t id, uint16_t num)
{
    if (id == 1U)
    {
        Emm_V5_Pos_Control(1,1,550,240,num,true,false);
    }
    if (id == 6U)
    {
        Emm_V5_Pos_Control(6,0,550,240,num,true,false);
    }
}

/* 按逻辑编号选择夹爪的 PWM 脉宽，具体输出是 TIM4 通道 2。 */
void servo_set(uint16_t num)  
{
		if(num==1)
		{
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,950);  
			osDelay(500);
		}
		if(num==2)
		{
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,1300);
			osDelay(500);
		}
}
 
/* 按槽号旋转车上的存料盘，具体输出是 TIM3 通道 2。参数 1、2、3 是槽位编号。 */
void Plate(uint16_t num)
{
		if(num==1)
		{
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,1800);  //500转盘1 1100转盘2 1600转盘3

		}
		if(num==2)
		{
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,1300);

		}
		if(num==3)
		{
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,700);
		}

}
