#ifndef ELBOW_WORK_CAN_H
#define ELBOW_WORK_CAN_H

#include "work_can_protocol.h"

/* CAN 业务接口，实际收发实现在 can_test.c，名字沿用原工程。
 * 肘击端发送 0x610，底盘回复 0x611，每 100 ms 重发，500 ms 无应答判离线。
 * 8 字节依次为版本、状态、任务编号的 4 字节小端值、区域/运动标志、结果码。
 * 结果码对应 OLED 的 E，具体数字定义见 work_can_protocol.h 的 WorkResult。
 */
/* 更新待发送状态；任务、区域或结果改变后，下一次收发任务立即安排发送。 */
void Work_CAN_SetRequest(uint8_t state, uint32_t task, uint8_t region, uint8_t result);
/* 获取当前任务的应答。离线/任务不匹配返回 0，底盘故障可跨任务返回。 */
uint8_t Work_CAN_GetReply(uint32_t task);
/* 最近 500 ms 内收到合法且任务匹配的底盘应答才在线，故障报文也可证明在线。 */
uint8_t Work_CAN_Online(void);
/* 最近一份底盘应答的结果码，调用者需先确认该应答有效。 */
uint8_t Work_CAN_Result(void);
/* 在线时读取底盘应答中区域字节的最高位，1 表示底盘报告车辆正在运动。 */
uint8_t Work_CAN_Moving(void);
/* Version-2 command snapshot. The caller checks task, station and phase. */
uint8_t Work_CAN_StationCommand(WorkCanPacket *command);

#endif
