#ifndef ELBOW_WORK_CONTROL_H
#define ELBOW_WORK_CONTROL_H
#include <stdint.h>
/* 上电时调用，清除作业状态、任务编号与本棵树各区域的执行次数。 */
void Work_Control_Init(void);
/* 主循环调用。now 为当前毫秒数，frame_count 为已解析的视觉帧总数。
 * 使用帧总数判断有没有新图像，根据 vision_motor.h 的模式开关执行
 * 独立视觉测试，或推进“接近、停车、复检、动作、离树”的协同流程。
 */
void Work_Control_Task(uint32_t now, uint32_t frame_count);
#endif
