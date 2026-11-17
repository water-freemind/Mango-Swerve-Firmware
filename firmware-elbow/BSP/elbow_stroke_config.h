#ifndef ELBOW_STROKE_CONFIG_H
#define ELBOW_STROKE_CONFIG_H

/* 已停用：2026-11-16 起肘击改为纯毫秒定时（vision_motor.h 的 VISION_CW_MS /
 * VISION_CCW_MS），不再使用编码器行程闭环、软限位和回位核验。
 * 本文件不再被任何源文件包含，只作为 2026-11-14 手推量程的测量记录保留。
 * 行程为硬件四倍频累计计数，正负号表示从回收端向伸出端的编码器方向。
 * 区域1=B、区域2=D、区域3=C、区域4=A。
 */
#define ELBOW_STROKE_A_COUNTS 3822
#define ELBOW_STROKE_B_COUNTS 3866
#define ELBOW_STROKE_C_COUNTS (-3866)
#define ELBOW_STROKE_D_COUNTS (-3858)

#endif