/* 串口收发的板级分发层。usart.c 初始化硬件，本文件启动接收并在 HAL 回调里根据串口选择设备协议。 */

#include "bsp_usart.h"
#include "stm32f4xx_hal.h"
#include "user_uart.h"    //串口舵机
#include "QR_code.h"    //扫码
#include "chassic.h"   //底盘-张大头
#include "maixcam.h"    //摄像头
#include "radar_v6.h"   //雷达


extern UART_HandleTypeDef huart1;
//extern UART_HandleTypeDef huart5;
//extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;

extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern uint8_t rc1;//接收中断缓存  串口舵机  环缓

uint8_t RXbuf1,RXbuf5,RXbuf2,RXbuf3[EMM_RX_BUFFER_SIZE],RXbuf4,RXbuf6[HWT101_RX_BUFFER_SIZE];   //各个串口的数据缓冲区

/**
  * @brief UART重定向
  * @param 
  * @retval 无
  */

//int fputc(int ch, FILE *fp)
//{
//  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
//  return ch;
//}
// 
//int fgetc(FILE *fp)
//{
//  uint8_t ch = 0;
//  HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
//  return ch;
//}

/**
  * @brief UART 初始化 
  * @param huart: UART handle
  * @retval 无
  */

void EMM_ResReceiveToIdle_DMA(void)
{
	  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, RXbuf3, sizeof(RXbuf3)); //接收完毕后重启
	  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);		                  //手动关闭DMA_IT_HT中断
}

void HWT101_ResReceiveToIdle_DMA(void)
{
		/* DMA 负责搬运串口字节。空闲事件可以把已经写入的一段交给上层，关闭半满通知是为了跳过只填到一半就回调的情况。 */
		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, RXbuf6, sizeof(RXbuf6)); //接收完毕后重启
	  __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);		                  //手动关闭DMA_IT_HT中断
}

/* 摄像头和扫码各挂一个单字节接收请求。HAL 报告该字节收完后进入下面的回调，再次挂接收才能继续收下一字节。 */
void uart_init_User()
{
	
	/* 三个参数依次是串口、装数据的内存地址、本次字节数。这里两个串口各挂一个字节的接收。 */
	HAL_UART_Receive_IT(&huart1,&RXbuf1,1);           //摄像头 		--普通中端
	HAL_UART_Receive_IT(&huart2,&RXbuf2,1);           //扫码模块  --普通中端
	//	HAL_UART_Receive_IT(&huart3,&RXbuf3,1);      //ZDT电机    --轮询模式-无需开启中断
//	HAL_UART_Receive_IT(&huart4,&RXbuf4,1);          //串口舵机 --dma中端
//	HAL_UART_Receive_IT(&huart5,&RXbuf5,1);	         //激光雷达 --内部代码开启-通用中断无需开启
}

/**
  * @brief UART的单个或多个字节发送函数
  * @param huart: UART handle  TXBuf   size
  * @retval 无
  */

void UART_SendbyteOne(UART_HandleTypeDef *huart,uint8_t *txbuf,uint16_t size)
{
	/* 按 size 发送二进制数据，所以 00 也能作为有效字节发送。0xff 是给 HAL 的超时参数。 */
	HAL_UART_Transmit(huart,txbuf,size,0xff);
}

/**
  * @brief UART的字符串发送函数
  * @param huart: UART handle  str
  * @retval 无
  */
void Usart_SendString(UART_HandleTypeDef *huart,uint8_t *str)
{
	unsigned int k=0;
  do 
  {
      HAL_UART_Transmit(huart,(uint8_t *)(str + k) ,1,1000);
      k++;
  } while(*(str + k)!='\0');
}

/************************   串口中断     ***************************/
int a=0,b=0,c=0,d=0,e=0;

//接收中断
/* UART4 只把字节放进舵机缓冲区。USART1 交给 MaixCam 拼帧，USART2 交给二维码解析。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//usart1
	uint8_t ucTemp;
	if(huart==&huart4)          //串口舵机
	{
		a++;
		ucTemp = rc1;
		/* 舵机回包先写进环形缓冲，协议库随后从这里读完整帧。这样中断可以很快结束。 */
		RingBuffer_Push(FSUS_Usart.recvBuf, ucTemp);// 接收到数据放入缓冲区，不在中断具体处理数据
		HAL_UART_Receive_IT(FSUS_Usart.huartX, (uint8_t *)&rc1, 1);
		
	}
	else if(huart->Instance == UART5)
  {
		b++;
  }
	else if(huart==&huart2)   
	{
		c++;
		/* 扫码器的一个字节进入 QR_code.c，那里累计满 16 字节后给主任务写入四组任务号。 */
		USART2_IRQHandler_uaer();
	}
	else if(huart==&huart1)
	{
		e++;
		/* 摄像头的一个字节进入 maixcam.c，那里判断这一字节属于帧头、坐标、颜色还是帧尾。 */
		USART1_IRQHandler_uaer();
	}	
	/* 单字节接收完成后重新挂接收，HAL 才能继续填下一个字节。USART1 和 USART2 的挂接收在这里一起尝试执行。 */
	HAL_UART_Receive_IT(&huart2,&RXbuf2,1);
	HAL_UART_Receive_IT(&huart1,&RXbuf1,1);
	
}

//事件中断
/* 空闲或接收事件可携带一批字节。USART6 当前将这批数据交给 HWT101 识别角度帧。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
	if(huart->Instance == USART6)     //HWT101陀螺仪
	{
   /* Size 是 HAL 报告的本次接收位置或长度。HWT101 驱动据此在 DMA 缓冲内寻找完整的角度帧。 */
   USART6_IRQHandler_uaer(Size);
	}
}

//错误中断
/* 按出错端口恢复各自的接收方式。雷达错误交由 v6 驱动记录，IMU 路径重新挂 DMA 接收。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==UART4)
	{
		HAL_UART_Receive_IT(FSUS_Usart.huartX, (uint8_t *)&rc1, 1);
	}
	else if(huart->Instance==USART2)
	{
		HAL_UART_Receive_IT(&huart2,&RXbuf2,1);
	}	
  else  if(huart->Instance==USART3)
	{
		
	}
	else if(huart->Instance==USART1)
	{
		HAL_UART_Receive_IT(&huart1,&RXbuf1,1);
	}
   else if (huart->Instance == UART5)
   {
     /* 雷达中断只通知协议客户端有错误。真正的 DMA 恢复和重新建链由任务调用的 RadarClient_Poll() 完成。 */
     RadarClient_OnUartErrorInterrupt(huart->ErrorCode);
   }
  else if(huart->Instance==USART6)
	{
		HWT101_ResReceiveToIdle_DMA();
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
        RadarClient_OnUartTxCompleteInterrupt();
}
