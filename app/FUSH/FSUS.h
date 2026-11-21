/* 本车串口舵机动作接口。angle_set 为角度，time 为动作时间，内部使用统一的 FSUS_Usart。 */

#ifndef __FSUS_H__
#define __FSUS_H__

#include "stm32f4xx_hal.h"

//#include "fashion_star_uart_servo_examples.h"
#include "fashion_star_uart_servo.h"

void ser_set(float angle_set,uint16_t time);

#define WULIAO1         -142 //放回角度
#define WULIAO2          102   
#define ZHONG            50                      
#define LFte_1           10  
#define RIght_1          34

#define LFte_2           -15  
#define RIght_2          56   

#define LFte_3          -15
#define RIght_3          55  
#define Turn_RIght       -160
#endif
