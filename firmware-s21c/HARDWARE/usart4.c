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
#include "usart4.h"
#include <string.h>

LidarPointTypedef4 Pack_Data4[12];/* 雷达接收的数据储存在这个变量之中 */
LidarPointTypedef4 Pack_sum4;     /* 输出结果储存 */
extern u16 distance1,distance_2,distance_3,distance_4;
extern u8 flag ;
extern u16 receive_cnt2,receive_cnt3,receive_cnt4,receive_cnt5;//计算成功接收数据帧次数;
/**************************************************************************
Function: Usart3 initialization
Input   : bound:Baud rate
Output  : none
函数功能：串口2初始化
入口参数：bound:波特率
返回  值：无
**************************************************************************/
void uart4_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE );
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;          //USART2 TX；
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    //复用推挽输出；
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);             //端口A；

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;               //USART2 RX；
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //浮空输入；
	GPIO_Init(GPIOC, &GPIO_InitStructure);                  //端口A；

	//Usart3 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
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
	USART_Init(UART4, &USART_InitStructure);     //初始化串口2
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启串口接受中断
	USART_Cmd(UART4, ENABLE);                    //使能串口2
}

/**************************************************************************
Function: Receive interrupt function
Input   : none
Output  : none
函数功能：串口2接收中断
入口参数：无
返回  值：无
**************************************************************************/
void UART4_IRQHandler(void) {
 const uint32_t status=UART4->SR;
 if(status & (USART_SR_RXNE|USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE)) {
  const uint8_t byte=(uint8_t)UART4->DR;
  s21_receive(2u,status,byte);
 }
}


/* Processing moved to src/stp23l.c. */
