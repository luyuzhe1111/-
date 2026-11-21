/* FSUS 串口适配层的公共声明。收、发缓冲区与实际 HAL 串口句柄由同一个结构体管理。 */

#ifndef __USER_UART_H__
#define __USER_UART_H__

#include "ring_buffer.h"
#include "usart.h"

#define USART_RECV_BUF_SIZE 500
#define USART_SEND_BUF_SIZE 500


typedef struct
{
    UART_HandleTypeDef *huartX;
    RingBufferTypeDef *sendBuf;
    RingBufferTypeDef *recvBuf;
} Usart_DataTypeDef;

extern Usart_DataTypeDef FSUS_Usart;

void User_Uart_Init(UART_HandleTypeDef *huartx);
void Usart_SendAll(Usart_DataTypeDef *usart);
#endif
