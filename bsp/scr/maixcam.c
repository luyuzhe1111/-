/*
 * MaixCam 视觉串口协议。STM32 的 USART1 通过 PA9/PA10 和相机通信。
 * 发送模式用 Maixcam_setmode()。接收从 bsp_usart.c 的 HAL 回调进入本文件，识别结果写到 YuanData、WuLiaoData 等变量。
 */

#include "maixcam.h"

/*
 * 这是中断与任务共享的数据。YuanData[0] 是图像 X，YuanData[1] 是图像 Y，单位都是像素。
 * 相机每交付一帧就更新帧号和毫秒时刻。动作代码用帧号分辨新旧，用时间差分辨是否过期。
 */
volatile uint16_t YuanData[2];
volatile uint32_t maixcam_frame_count = 0U;
volatile uint32_t maixcam_last_update_ms = 0U;
uint8_t WuLiaoData=0,move_maix=3;

/*
 * 下行字节格式是 FF + 模式号 + FE。例如模式 2 发 FF 02 FE，每组都是 3 个十六进制字节。
 * 模式 3、4、5、6、7、8 依次对应红、绿、蓝、黑、黄、浅蓝。任务颜色编号到模式号的对照在 action.c 的 ColorToCameraMode()。
 */
uint8_t mode1[3]={0xFF,0x01,0xFE},mode2[3]={0xFF,0x02,0xFE},mode3[3]={0xFF,0x03,0xFE},mode4[3]={0xFF,0x04,0xFE},\
mode5[3]={0xFF,0x05,0xFE},mode6[3]={0xFF,0x06,0xFE},mode7[3]={0xFF,0x07,0xFE},mode8[3]={0xFF,0x08,0xFE};

uint16_t yuanhuanx_last=0,yuanhuany_last=0;
/* 向相机发送 3 个原始字节 FF 模式 FE。模式 1 是物料，模式 2 是圆环，模式 3 到 8 对应不同颜色识别。 */
void Maixcam_setmode(uint8_t mode)
{
	/* mode 是上层选的识别模式。一条 if/else 分支负责发出对应的三个字节并等待 3 ms，这里也会被机构初始化调用。 */
	if(mode==1)              //物料模式
	{
		UART_SendbyteOne(&huart1,mode1,3);
		HAL_Delay(3);		
	}
	else if(mode==2)         //地面圆环定位
	{
		UART_SendbyteOne(&huart1,mode2,3);
		HAL_Delay(3);	
	}
	else if(mode==3)         //只识别红色物料
	{
		UART_SendbyteOne(&huart1,mode3,3);
		HAL_Delay(3);	
	}
	else if(mode==4)        //只识别绿色物料
	{
		UART_SendbyteOne(&huart1,mode4,3);
		HAL_Delay(3);		
	}
	else if(mode==5)        
	{
		UART_SendbyteOne(&huart1,mode5,3);
		HAL_Delay(3);		
	}
	else if(mode==6)        
	{
		UART_SendbyteOne(&huart1,mode6,3);
		HAL_Delay(3);		
	}
	else if(mode==7)        
	{
		UART_SendbyteOne(&huart1,mode7,3);
		HAL_Delay(3);		
	}
	else if(mode==8)        
	{
		UART_SendbyteOne(&huart1,mode8,3); /* FF 08 FE，禁止越界读第4个字节。 */
		HAL_Delay(3);		
	}
}
/******************************************
函数功能：对Maixcam接收的协议包进行解析
入口参数：原始协议包
返回  值：0：解析错误，协议包不正确
					1：解析正确，是要接收的数据
*******************************************/
static int maixcam_data_parse(uint8_t *pack)
{
    /* 这个函数只检查固定帧头 B3 B3 和末尾 5B。接收函数确保长度满 10 字节后才调用。 */
    if (pack[0] != 0xB3)                     return 0;
    if (pack[1] != 0xB3)                     return 0;
    if (pack[maixcam_DATA_SIZE - 1] != 0x5B) return 0;
    return 1;
}

/*
 * 每次收到一个字节就调用一次。state 记录收到了哪一部分，先找 B3 B3，再收满 10 字节检查帧尾 5B。
 * 调试时看 maixcam_frame_count 是否增加。YuanData[0/1] 为像素 X/Y，WuLiaoData 为颜色编号。
 */
void USART1_IRQHandler_uaer(void)
{
    /* i 是下一个待写的下标。static 让 i、state 和数组在中断返回之后继续保留，下一次中断可以接着拼上一帧。 */
    static uint8_t i = 0;
    static uint8_t state = 0;   /* 0=等第一个B3  1=等第二个B3  2=收数据 */
    static uint8_t maixcam_data_buff[maixcam_DATA_SIZE] = {0};
    /* RXbuf1 由 HAL 写入，每次保存刚到的一个字节。此函数先复制到局部变量，再根据拼帧状态处理。 */
    uint8_t rx = (uint8_t)RXbuf1;

    switch (state)
    {
    /* 此时还没找到帧头。逐字节等待第一个 B3，其他字节直接略过。 */
    case 0:                                     /* 等第一个帧头 0xB3 */
        if (rx == 0xB3) state = 1;
        break;

    /* 紧接的字节也要是 B3。匹配后先填数组下标 0、1，从下标 2 开始收剩下的 8 字节。 */
    case 1:                                     /* 等第二个帧头 0xB3 */
        if (rx == 0xB3)
        {
            maixcam_data_buff[0] = 0xB3;
            maixcam_data_buff[1] = 0xB3;
            i = 2;
            state = 2;
        }
        else
        {
            state = 0;                          /* 第二个不是 B3，重新找帧头 */
        }
        break;

    /* 数组长度到 10 后重新进入找帧头状态。校验过的帧才会更新坐标，坏帧保留上一次数据和帧号。 */
    case 2:                                     /* 收剩余 8 字节 */
        maixcam_data_buff[i++] = rx;
        if (i >= maixcam_DATA_SIZE)             /* 10 字节收满 */
        {
            state = 0;
            if (maixcam_data_parse(maixcam_data_buff))
            {
                /*
                 * 完整回包的十个字节依次是 B3 B3、类型、X 高低位、Y 高低位、附加字节、颜色和 5B。
                 * 例如 B3 B3 B1 00 45 00 65 00 01 5B 表示物料帧、X=69、Y=101、颜色编号 1。
                 */
                uint8_t  cmd   = maixcam_data_buff[2];
                /* X 和 Y 各占两个字节。以 01 2C 为例，高位左移 8 位再合上低位，得到 0x012C，也就是十进制 300。 */
                uint16_t x     = ((uint16_t)maixcam_data_buff[3] << 8) | maixcam_data_buff[4];
                uint16_t y     = ((uint16_t)maixcam_data_buff[5] << 8) | maixcam_data_buff[6];
                WuLiaoData = maixcam_data_buff[8];   /* 物料=颜色码，圆环=0 */

                YuanData[0] = x;
                YuanData[1] = y;
                move_maix   = cmd;                       /* 0xB1 物料 / 0xB2 圆环 */
                /* 字段先写入，随后更新接收时刻并增加帧号。Keil Watch 先看帧号是否增加，再根据类型查看颜色和坐标。 */
                maixcam_last_update_ms = HAL_GetTick();
                maixcam_frame_count++;

                /* pack[7]的半径/圆环编号目前未参与业务，保留原协议布局。
                 * 去掉未使用的局部副本，不影响坐标、颜色或帧计数更新。
                 */

                yuanhuanx_last = YuanData[0];
                yuanhuany_last = YuanData[1];
            }
        }
        break;
    }
}
