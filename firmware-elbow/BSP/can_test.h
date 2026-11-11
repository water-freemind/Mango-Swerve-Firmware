#ifndef CAN_TEST_H
#define CAN_TEST_H
#include "main.h"
/* OLED 的 C 诊断码。0 等待，1 收到有效底盘应答，2 初始化失败，
 * 3 发送失败，4 接收失败，5 底盘应答超时。激光帧不刷新底盘在线状态。
 */
extern volatile uint32_t CAN_Test_TxCount, CAN_Test_RxCount, CAN_Test_ErrorCount;
extern volatile uint32_t CAN_Test_LastHalError;
extern volatile uint8_t CAN_Test_State;
/* 初始化 CAN2 接收过滤器，然后在主循环中反复执行收发任务。 */
void CAN_Test_Init(void);
void CAN_Test_Task(void);
/* 激光端口 1～4 对应左、右、后、前，数组下标/读取参数使用 0～3。
 * 同总线的标准数据帧 0x601，共 8 字节，每 2 字节大端表示一路原始毫米值。
 */
#define CAN_LASER_ID 0x601u
#define CAN_LASER_TIMEOUT_MS 300u
#define CAN_LASER_MAX_MM 15000u
/* 读取一路激光。返回 0 等待/参数无效，1 有效毫米值，2 超出 15000 mm，3 超时。
 * 只有返回 1 或 2 时才会写入 distance_mm，屏幕应按返回状态决定显示内容。
 */
uint8_t CAN_Laser_Read(uint8_t channel, uint16_t *distance_mm);
#endif
