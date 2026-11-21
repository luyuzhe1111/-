/* 串口屏文本发送接口，语法类似 printf。当前实现使用 USART1。 */

#ifndef __AIHMI_H__
#define __AIHMI_H__

#include "stm32f4xx_hal.h"

#include "usart.h"
#include "bsp_usart.h"

void AIHMI_printf(const char *format, ...);

#endif

