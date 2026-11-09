#include "s21_board.h"
/***********************************************
公司：东莞市微宏智能科技有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：5.7
修改时间：2021-04-29

Company: WeiHong Co.Ltd
Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version:5.7
Update：2021-04-29

All rights reserved
***********************************************/
#include "usart3.h"
#include <string.h>

extern u16 distance_1,distance_2,distance_3,distance_4;
/**************************************************************************
Function: Usart3 initialization
Input   : bound:Baud rate
Output  : none
函数功能：串口3初始化
入口参数：bound:波特率
返回  值：无
**************************************************************************/

LidarPointTypedef Pack_Data[12];/* 雷达接收的数据储存在这个变量之中 */
LidarPointTypedef Pack_sum;     /* 输出结果储存 */
extern u16 receive_cnt2,receive_cnt3,receive_cnt4,receive_cnt5;//计算成功接收数据帧次数;
extern u8 confidence;
extern u16 distance,noise,reftof;
extern u32 peak,intg;

void uart3_init(u32 bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//使能UGPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//使能USART3时钟
	//USART3_TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//USART3_RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//Usart3 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	//USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART3, &USART_InitStructure);     //初始化串口3
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
	USART_Cmd(USART3, ENABLE);                    //使能串口3

}

/**************************************************************************
Function: Receive interrupt function
Input   : none
Output  : none
函数功能：串口3接收中断
入口参数：无
返回  值：无
**************************************************************************/
void USART3_IRQHandler(void) {
 const uint32_t status=USART3->SR;
 if(status & (USART_SR_RXNE|USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE)) {
  const uint8_t byte=(uint8_t)USART3->DR;
  s21_receive(1u,status,byte);
 }
}



/* Processing moved to src/stp23l.c. */

void data_trastion(void) {
    const uint16_t mm[4]={distance_1,distance_2,distance_3,distance_4};
    stp_can_payload(mm,CAN_Send_Data);
    stp_usb_payload(CAN_Send_Data,Usart_Send_Data);
}

/**************************************************************************
函数功能：计算发送的数据校验位
入口参数：
返回  值：检验位
**************************************************************************/
u8 Check_Sum(void)
{
	unsigned char check_sum=0,k;
	for(k=0;k<9;k++)
	{
	  check_sum=check_sum^Usart_Send_Data[k];
	}
	return check_sum;
}


//void data_process(void)//数据处理函数，完成一帧之后可进行数据处理
//{
//	//计算距离
//	static u8 cnt2 = 0;
//	static u8 cnt3 = 0;
//	static u8 cnt4 = 0;
//	static u8 cnt5 = 0;
//	u8 i;
//	static u16 count = 0;
//	static u32 sum = 0;
//	for(i=0;i<12;i++)//12个点取平均
//	{
//		if(Pack_Data.point[i].distance != 0)//去除0的点
//		{
//			count++;
//			sum += Pack_Data.point[i].distance;
//		}
//	}

//	if(flag == 2)
//	{

//		if(++cnt2 == 10)//100个数据帧计算一次距离
//		{
//			distance1 = sum/count;
//			sum = 0;
//			count = 0;
//			cnt2 = 0;
//			USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
//			USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
//		}
//	}
//	else if(flag == 3)
//	{

//		if(++cnt3 == 10)//100个数据帧计算一次距离
//		{
//			distance2 = sum/count;
//			sum = 0;
//			count = 0;
//			cnt3 = 0;
//			USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启串口接受中断
//			USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
//		}
//	}
//	else if(flag == 4)
//	{

//		if(++cnt4 == 10)//100个数据帧计算一次距离
//		{
//			distance3 = sum/count;
//			sum = 0;
//			count = 0;
//			cnt4 = 0;
//			USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//开启串口接受中断
//			USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);
//		}
//	}
//	else if(flag == 5)
//	{

//		if(++cnt5 == 10)//100个数据帧计算一次距离
//		{
//			distance4 = sum/count;
//			sum = 0;
//			count = 0;
//			cnt5 = 0;
//			USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
//			USART_ITConfig(UART5, USART_IT_RXNE, DISABLE);
//		}
//	}
//}
