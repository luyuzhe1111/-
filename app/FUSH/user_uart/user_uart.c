/* 将 FSUS 协议库接到 STM32 的 HAL 串口。User_Uart_Init() 绑定串口句柄与收发缓冲并开启接收。 */

#include "user_uart.h"

uint8_t UsartSendBuf[USART_SEND_BUF_SIZE + 1];
uint8_t UsartRecvBuf[USART_RECV_BUF_SIZE + 1];
RingBufferTypeDef UsartSendRingBuf;//发送缓冲区
RingBufferTypeDef UsartRecvRingBuf;//接收缓冲区
Usart_DataTypeDef FSUS_Usart;
uint8_t rc1;//接收中断缓存

void User_Uart_Init(UART_HandleTypeDef *huartx)  //串口初始化
{
	// 创建缓冲组
	RingBuffer_Init(&UsartSendRingBuf, USART_SEND_BUF_SIZE, UsartSendBuf);
	RingBuffer_Init(&UsartRecvRingBuf, USART_RECV_BUF_SIZE, UsartRecvBuf);
	// 初始化自定义用户串口结构体
	FSUS_Usart.recvBuf = &UsartRecvRingBuf;
	FSUS_Usart.sendBuf = &UsartSendRingBuf;
	/* main() 调用时传的是 huart4。绑定以后，FSUS 协议函数通过这一结构就能找到 UART4 和对应的收发队列。 */
	FSUS_Usart.huartX = huartx;
	// 开启接收中断
	/* 接收第一个字节到 rc1。其后由 bsp_usart.c 的 UART4 分支把字节压入队列并继续接收。 */
	HAL_UART_Receive_IT(FSUS_Usart.huartX, (uint8_t *)&rc1, 1);
}
// 发送数据
/* 把发送环形缓冲区中积累的字节发到串口。帧字段和校验由 FSUS 协议层生成。 */
void Usart_SendAll(Usart_DataTypeDef *usart)
{
	uint8_t value;
	/* 发送缓冲还有字节就继续取出并发走。业务传入的是舵机的地址、角度等，协议库会先把它们编成字节。 */
	while (RingBuffer_GetByteUsed(usart->sendBuf))
	{
		value = RingBuffer_Pop(usart->sendBuf);
		HAL_UART_Transmit(usart->huartX, &value, 1, 1);
	}
}
//中断接收
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	uint8_t ucTemp;
//	if (huart->Instance == FSUS_Usart.huartX->Instance)
//	{
//		ucTemp = rc1;
//		RingBuffer_Push(FSUS_Usart.recvBuf, ucTemp);// 接收到数据放入缓冲区，不在中断具体处理数据
//		HAL_UART_Receive_IT(FSUS_Usart.huartX, (uint8_t *)&rc1, 1);
//	}
//}


