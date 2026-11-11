#ifndef __FUN_H
#define __FUN_H

#include "headfile.h"

/* 下标 0~3 对应 A~D；计数单位为脉冲/10ms，速度单位为 RPM。 */
extern volatile int32_t Encoder_Count[4];
extern volatile int32_t Speed[4];

/* 固定方向的旧接口，当前区域肘击方向在 vision_motor.c 中单独控制。 */
void Motor_Statu(uint8_t mode);
/* 指示灯翻转，time 单位为毫秒。 */
void LED_Blink(uint32_t time);
/* 外设配置完成后启动 PWM、编码器和视觉接收，初始电机输出为 0。 */
void Init(void);
/* GPIO 初始化后立即启动 OLED，显示等待信息。 */
void OLED_PowerOn(void);
/* 主循环显示入口，内部按 100 ms 节奏刷新车辆、机构与四路激光数据。 */
void OLED_Display(void);
/* 独立串口上报任务，在主循环中持续调用，内部按 500ms 限频。 */
void UART_SendStatus(void);
/* 主循环处理视觉数据，继续执行作业流程与电机动作计时。 */
void Vision_UART_Task(void);
extern volatile uint32_t Vision_UART_ErrorCount, Vision_UART_LastError;
extern volatile uint32_t Vision_UART_ArmErrorCount;


#endif



