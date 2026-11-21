/*
 * 左、中、右三个抓取位置的实际机械动作。num 是车上存料槽号，动作由升降、伸缩、转角、夹爪和等待组成。
 * material_last 标记最后一抓，用于跳过部分归中动作。每个坐标对应机械标定，读到 ZDT_set() 或 servo_set() 时可跳到 actionInit.c。
 */

#include "robot_task.h"
#include "chassic.h"
#include "tim.h"
#include "action.h"
#include "actionInit.h"
#include "cmsis_os.h"

uint8_t material_last = 0U; /* 0: return to middle; 1: last raw grab. */

/* 中位抓取动作使用中间转盘位置的标定。函数参数 num 仍是车上存料槽号，放料前通过 Plate(num) 将该槽转到抓手前。 */
void grab_middle(int num)
{
	ZDT_set(6,1300);
osDelay(400);
    ZDT_set(1,4400);//下降
    osDelay(500);
    Plate(num);
	servo_set(1);//抓紧
    ZDT_set(1,1500);//上升速度更快
    osDelay(200);
    ZDT_set(6,400);
    osDelay(100);
    ser_set(-145,750);
    osDelay(400);
    ZDT_set(1,2600);
    osDelay(400);
    servo_set(2); 
    ZDT_set(1,0);
    osDelay(200);
    if (material_last == 0U)
    {
        ZDT_set(6,5500);
        osDelay(300);
        ser_set(-69,800);
        osDelay(810);
    }
}
/* 左位抓取的舵机角度、步进脉冲数和动作等待与中位不同。上层 RawGrab_GrabPosition() 根据左、中、右决定调用哪个函数。 */
void grab_left(int num)
{	
ser_set(-81,400);
osDelay(200);
ZDT_set(6,9800);
osDelay(300);
    ZDT_set(1,4400);//下降
    osDelay(500);
Plate(num);
servo_set(1);
    ZDT_set(1,1500);//上升速度更快
	osDelay(300);
    ZDT_set(6,400);
    osDelay(400);
    ser_set(-145,700);
	osDelay(300);
	ZDT_set(1,2600);
    osDelay(400);
    servo_set(2); 
    ZDT_set(1,0);
    osDelay(100);
    if (material_last == 0U)
    {
        ZDT_set(6,5500);
        osDelay(300);
        ser_set(-69,800);
        osDelay(810);
    }
}
/* 右位动作把抓取、放到车载槽和准备下一次抓取连在一起。最后一抓通过 material_last 省掉部分恢复动作。 */
void grab_right(int num)
{
ser_set(-54,400);
osDelay(200);
ZDT_set(6,9800);
osDelay(300);
    ZDT_set(1,4400);//下降
    osDelay(500);
Plate(num);
servo_set(1);
    ZDT_set(1,0);
	osDelay(600);
    ZDT_set(6,400);
    osDelay(300);
    ser_set(-145,900);
	osDelay(400);
	ZDT_set(1,2600);
    osDelay(500);
    servo_set(2); 
    ZDT_set(1,0);
    osDelay(100);
    if (material_last == 0U)
    {
        ZDT_set(6,5500);
        osDelay(300);
        ser_set(-69,800);
        osDelay(810);
    }
}
