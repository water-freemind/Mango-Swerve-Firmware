#ifndef VISION_MOTOR_H
#define VISION_MOTOR_H

#include "main.h"

/* ==================== 运行模式：先改这一处 ====================
 * 1 为视觉肘击独立测试，0 为底盘 CAN 协同作业。
 * 测试模式只要求视觉在线、同一区域稳定，不要求 Tree 或底盘停稳应答。
 * 同一区域持续存在时只动作一次；无目标持续 500 ms/至少 3 帧后可再次触发。
 * 稳定识别到另一区域也可触发该区域。动作期间的新区域不会排队执行。
 * 测试时请将机构放在适合动作的工位。修改后需要重新编译和烧录。
 */
#ifndef ELBOW_TEST_MODE
#define ELBOW_TEST_MODE 2
#endif
#if ELBOW_TEST_MODE != 0 && ELBOW_TEST_MODE != 1 && ELBOW_TEST_MODE != 2
#error "ELBOW_TEST_MODE must be 0, 1 or 2"
#endif
/* 2 为底盘定点作业，使用版本2协议，由底盘任务号和树位号触发。 */

/* ==================== 肘击触发条件的参数入口 ====================
 * 先看这里的区间和等待参数，再读 work_control.c 中的作业流程。
 * dx 是相机输出的树水平偏移，单位为像素。时间宏的单位均为毫秒。
 * 例：WORK_STOP_DX 为 30，中心为 0 时，停车区间就是 -30～+30。
 * 本节区间参数用于底盘协同模式。独立测试只使用目标稳定、无目标确认参数。
 * 修改参数后需要重新编译和烧录。当前激光读数只参与显示。
 */
#define WORK_DX_CENTER 0                  // 机构的作业中心偏置，按相机安装位置标定。
#define WORK_SLOW_DX 100                  // 进入中心 ±100 像素时请求底盘减速。
#define WORK_STOP_DX 30                   // 目标稳定且进入中心 ±30 像素时请求停车。
#define WORK_LEAVE_DX 120                 // 树越过离树位置后，才允许触发下一棵树。
#define WORK_LEAVE_DIRECTION (-1)         // -1 表示树离开时 dx 变小，+1 表示变大，按实车标定。
#define WORK_TARGET_STABLE_MS 200U        // 同一区域目标至少持续 200 ms，代码另要求至少 3 帧。
#define WORK_SETTLE_MS 300U               // 返回计时结束后至少等 300 ms，再用新图像复检。
#define WORK_FRESH_VISION_MS 200U         // 首次收到停稳确认后至少等 200 ms，再取新帧。
#define WORK_NO_TARGET_MS 500U            // 树仍在停车区间内，连续无目标至少 500 ms 才正常放行。
#define WORK_NO_TARGET_FRAMES 3U          // 无目标确认还需要至少 3 帧新图像。
#define WORK_MAX_ATTEMPTS_PER_REGION 2U   // 同一棵树每个区域最多执行 2 次。
#define WORK_TREE_TIMEOUT_MS 8000U        // 从首次请求停车起，8 秒后结束作业，当前动作先完成返回。
#define WORK_STOP_TIMEOUT_MS 2000U        // 请求停车后 2 秒未收到 STOPPED，进入故障。
#define WORK_VISION_WAIT_MS 2000U         // 停稳复检时连续 2 秒无有效树信息，进入视觉故障。
#define WORK_LEAVE_MISSING_MS 600U        // 已观察到车辆运动且视觉在线，连续无树 600 ms 后允许下一棵。
#define WORK_LEAVE_MIN_MOVING_MS 300U     // 离树判定前，先要求底盘运动标志连续存在至少 300 ms。

typedef enum {
    /* 数值从 0 开始依次递增，与 OLED 的 W 阶段文字对应。 */
    VISION_WORK_BOOT, VISION_WORK_IDLE, VISION_WORK_APPROACH,
    VISION_WORK_WAIT_STOP, VISION_WORK_CHECK, VISION_WORK_ACTING,
    VISION_WORK_RELEASE, VISION_WORK_LEAVING, VISION_WORK_FAULT,
    VISION_WORK_TEST /* 独立测试等待状态，OLED 显示 TEST_WAIT。 */
} VisionWorkState;

extern volatile uint8_t Vision_WorkState, Vision_WorkResult; // 当前阶段、结果码，供屏幕和串口显示。
extern volatile uint32_t Vision_WorkTask;                   // 当前树任务编号，用于配对底盘应答。
uint8_t Vision_Motor_Start(uint8_t region);                 // 空闲时锁定区域并开始动作，成功返回 1。
uint32_t Vision_Motor_ReturnedAt(void);                     // 读取返回计时结束时刻，用于图像稳定等待。

/* ==================== 常用参数：主要修改下面五行 ====================
 * 修改方法：只改右侧数字，保留数字后面的 U。U 是 C 语言的类型标记，不是单位。
 * 时间单位：毫秒，1000 毫秒 = 1 秒；500 = 0.5 秒；5000 = 5 秒。
 * 四路电机共用下面的占空比和时间，修改后需要重新编译并下载到 STM32。
 *
 * 检测到目标后的动作顺序：
 * 打击 → 换向停顿 → 返回 → 打击后等待 → 接收下一条指令。
 * 实际机械方向取决于接线，方向调整在各区域的独立控制函数中完成。
 * 这里是按时间控制，逆转相同时间不保证机构精确回到原位。
 */
#define VISION_MOTOR_PWM 100U           // 电机占空比，当前 100 表示 100%，可填 0～100；0 为不输出。
                                     // 当前 TIM1 的 ARR=99，因此这里的数值对应百分比。
                                     // 占空比调大通常会提高驱动力/转速，不代表固定转速。
#define VISION_CW_MS 600U            // 打击阶段运行时间，当前 600 毫秒，即 0.6 秒；改 1000U 就是 1 秒。
#define VISION_CCW_MS 600U           // 返回阶段运行时间，当前 600 毫秒，即 0.6 秒；改 2000U 就是 2 秒。
#define VISION_REVERSE_PAUSE_MS 50U  // 换向前先停止输出，当前停 50 毫秒，即 0.05 秒，再开始返回。
#define VISION_RETRIGGER_DELAY_MS 50U // 一轮结束后等待，当前 50 毫秒，即 0.05 秒，再接收新区域指令。

/* ==================== 修改电机方向的位置 ====================
 * 打开同目录 vision_motor.c，找到对应区域的独立控制函数。
 * 第1区 B电机，Region1_Motor_Control，方向 PC2/PC3，PWM PE11。
 * 第2区 D电机，Region2_Motor_Control，方向 PD3/PD5，PWM PE14。
 * 第3区 C电机，Region3_Motor_Control，方向 PC4/PC5，PWM PE13。
 * 第4区 A电机，Region4_Motor_Control，方向 PC0/PC1，PWM PE9。
 * 每个函数都直接写出打击和返回时的高低电平，需要反向时交换两组电平。
 */

/* ==================== 视觉通信：通常不需要修改 ==================== */
#define VISION_TIMEOUT_MS 500U       // 超过 0.5 秒无合法帧，显示视觉离线；这不是打击时间。
                                    // 已经开始的正反转动作仍会执行完，不因视觉离线中断。
#define VISION_TREE_DX_LIMIT 2048     // 接受的树水平偏移范围为 -2048～2048 像素；另外允许 9999 表示无树。

/* 以下是程序运行时的数据和函数声明，不是在这里填写参数。 */
extern volatile uint8_t Vision_Region;          // 当前视觉区域：0 无目标，1～4 为对应区域。
extern volatile int32_t Vision_TreeDX;          // 树的水平偏移：左负右正；9999 表示无树或视觉离线。
extern volatile uint8_t Vision_Online;          // 视觉通信状态：1 正常，0 离线。
extern volatile uint32_t Vision_FrameCount;     // 累计解析成功的帧数，状态机用它区分新帧和旧帧。
extern volatile uint32_t Vision_ByteCount;      // 累计收到并交给解析器的字节数，排查串口时查看。
extern volatile uint32_t Vision_BadFrameCount;  // 累计丢弃的格式错误帧/超长数据次数。

extern volatile uint8_t Vision_MotorRegion;  // 本次动作锁定的电机：0 无，1=A、2=B、3=C、4=D。
extern volatile uint8_t Vision_MotorPhase;   // 动作阶段：0 等待指令，1 打击，2 换向停顿，3 返回，4 打击后等待。

void Vision_Motor_Init(void);                // 上电初始化，清除旧状态并停止全部电机输出。
void Vision_Motor_Task(void);                // 主循环反复调用，负责触发动作、计时和换向。
uint32_t Vision_Motor_RemainingMs(void);     // 读取当前阶段还剩多少毫秒，用于 OLED 倒计时。
uint8_t Vision_Motor_IsArmed(void);          // 是否允许开始新动作；完成后按设定间隔等待，再接收新指令。
void Vision_Motor_Receive(const uint8_t *data, uint16_t size); // 串口回调送入数据，支持一帧拆成多次接收。
void Vision_Motor_RxError(void);             // 串口出错时丢弃未接收完整的数据。

#endif
