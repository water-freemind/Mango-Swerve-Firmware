/* 本文件负责“视觉串口解析”和“肘击电机的分阶段定时运行”。
 * 是否允许开始肘击，由 work_control.c 统一判断。
 * 动作只用毫秒计时推进：打击 → 换向停顿 → 返回 → 打击后等待 → 空闲，
 * 与 RED_Demo 的分阶段计时一致，不使用编码器位置闭环。
 * 编码器只保留在 fun.c 中做测速和手动换行程读数，不参与停车判断。
 * 调整打击/返回时间和占空比看 vision_motor.h，调整方向看下面四个区域函数。
 */
#include "vision_motor.h"
#include "tim.h"
#include "work_control.h"
#include <string.h>

volatile uint8_t Vision_Region;
volatile int32_t Vision_TreeDX = 9999;
volatile uint8_t Vision_Online;
volatile uint32_t Vision_FrameCount;
volatile uint32_t Vision_StateFrameCount;
volatile uint32_t Vision_ByteCount;
volatile uint32_t Vision_BadFrameCount;

static char line[32];
static uint8_t used;
/* 串口中断先把最新完整帧存到 pending_*，主循环再一次性取走这些数据。
 * received 表示接收过合法帧，last_frame 用于判断视觉超时。
 */
static volatile uint8_t received, pending_region;
static volatile int32_t pending_dx;
static volatile uint32_t last_frame;

/* 收到区域指令后锁定本次电机，完成打击和返回后再处理下一轮。 */
volatile uint8_t Vision_MotorRegion;
volatile uint8_t Vision_MotorPhase;
volatile uint8_t Vision_MotorResult;
volatile uint8_t Vision_HomeConfirmed;
volatile uint32_t Vision_StrokeLastMs;
volatile int32_t Vision_StrokeLastCounts;
static uint32_t motor_pwm = VISION_MOTOR_PWM;
/* 数组下标是视觉区域，数组值是电机编号。0无目标，1区=B，2区=D，3区=C，4区=A。 */
static const uint8_t region_to_motor[5] = {0, 2, 4, 3, 1};
static uint32_t phase_started;
/* 行程结束检测的状态：起点计数、上一次计数变化时刻、是否真的动过。 */
static int32_t stroke_base, stroke_last;
static uint32_t stroke_last_move_ms;
static uint8_t stroke_moved;
/* 运行期读取的阈值；0 表示关闭行程结束检测（纯定时，与 elbow-controller 一致）。 */
static uint32_t stroke_end_ms = ELBOW_STROKE_END_MS;
/* 停止全部四路 PWM 输出。上电、换向停顿、动作结束时都会调用。 */
static void Motor_Stop(void)
{
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->CCR4 = 0;
}

/* 记录本段动作的编码器起点。打击和返回各调用一次。 */
static void stroke_begin(uint32_t now)
{
    stroke_base = Encoder_Position[Vision_MotorRegion - 1U];
    stroke_last = stroke_base;
    stroke_last_move_ms = now;
    stroke_moved = 0;
}

/* 判断本段行程是否已经走到头。
 * 计数还在变化就不断刷新时间戳；动过之后连续 ELBOW_STROKE_END_MS 没有变化，
 * 认为顶到机械限位或已经卡住，返回 1 让调用方提前收尾。
 * 只做停止判断，不把计数换算成位置。
 */
static uint8_t stroke_finished(uint32_t now)
{
    int32_t position = Encoder_Position[Vision_MotorRegion - 1U];
    int32_t step = position - stroke_last;
    int32_t travel = position - stroke_base;
    if (step >= 2 || step <= -2)
    {
        stroke_last = position;
        stroke_last_move_ms = now;
    }
    if (travel >= ELBOW_STROKE_MIN_COUNTS || travel <= -ELBOW_STROKE_MIN_COUNTS)
        stroke_moved = 1;
    if (!stroke_end_ms || !stroke_moved) return 0;
    if ((uint32_t)(now - phase_started) < ELBOW_STROKE_MIN_MS) return 0;
    return (uint32_t)(now - stroke_last_move_ms) >= stroke_end_ms;
}

/* 记录本段实际用时和编码器变化量，主循环串口用它标定真实行程时间。 */
static void stroke_record(uint32_t now)
{
    Vision_StrokeLastMs = (uint32_t)(now - phase_started);
    Vision_StrokeLastCounts = Encoder_Position[Vision_MotorRegion - 1U] - stroke_base;
}

/* 故障或中止时把正在打击的一路提前转入换向停顿，随后照常定时返回。
 * 已处于停顿、返回或空闲阶段时不做额外动作。 */
void Vision_Motor_RequestReturn(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    if (Vision_MotorPhase == 1)
    {
        Motor_Stop();
        phase_started = HAL_GetTick();
        Vision_MotorPhase = 2;
    }
    __set_PRIMASK(key);
}

/* ==================== 每个区域单独控制方向 ====================
 * striking 填 1 表示打击，填 0 表示返回。
 * GPIO_PIN_SET 表示输出高电平，GPIO_PIN_RESET 表示输出低电平。
 * 想把某一区域反过来，就交换该函数中打击和返回两组 GPIO 电平。
 * 每组的两个方向引脚必须一高一低。PWM 寄存器决定该电机的占空比。
 */

/* 第1区 → BOUT / B电机
 * 方向引脚 PC2、PC3，PWM 引脚 PE11，对应 TIM1_CH2。
 * 打击时 PC2=高、PC3=低，返回时 PC2=低、PC3=高。
 */
static void Region1_Motor_Control(uint8_t striking)
{
    if (striking)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);   // 打击，PC2 输出高。
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); // 打击，PC3 输出低。
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); // 返回，PC2 输出低。
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);   // 返回，PC3 输出高。
    }
    TIM1->CCR2 = motor_pwm;
}

/* 第2区 → DOUT / D电机
 * 方向引脚 PD3、PD5，PWM 引脚 PE14，对应 TIM1_CH4。
 * 打击时 PD3=低、PD5=高，返回时 PD3=高、PD5=低。
 */
static void Region2_Motor_Control(uint8_t striking)
{
    if (striking)
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET); // 打击，PD3 输出低。
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);   // 打击，PD5 输出高。
    }
    else
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);   // 返回，PD3 输出高。
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET); // 返回，PD5 输出低。
    }
    TIM1->CCR4 = motor_pwm;
}

/* 第3区 → COUT / C电机
 * 方向引脚 PC4、PC5，PWM 引脚 PE13，对应 TIM1_CH3。
 * 打击时 PC4=低、PC5=高，返回时 PC4=高、PC5=低。
 */
static void Region3_Motor_Control(uint8_t striking)
{
    if (striking)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET); // 打击，PC4 输出低。
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);   // 打击，PC5 输出高。
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);   // 返回，PC4 输出高。
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // 返回，PC5 输出低。
    }
    TIM1->CCR3 = motor_pwm;
}

/* 第4区 → AOUT / A电机
 * 方向引脚 PC0、PC1，PWM 引脚 PE9，对应 TIM1_CH1。
 * 打击时 PC0=高、PC1=低，返回时 PC0=低、PC1=高。
 */
static void Region4_Motor_Control(uint8_t striking)
{
    if (striking)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);   // 打击，PC0 输出高。
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); // 打击，PC1 输出低。
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); // 返回，PC0 输出低。
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);   // 返回，PC1 输出高。
    }
    TIM1->CCR1 = motor_pwm;
}

/* motor_id 是电机编号，1=A、2=B、3=C、4=D。
 * 先停止所有 PWM，再调用对应区域函数，保证每次只有一路运行。
 */
static void Motor_Run(uint8_t motor_id, uint8_t striking)
{
    Motor_Stop();
    switch (motor_id)
    {
    case 1: Region4_Motor_Control(striking); break; // A 电机属于第4区。
    case 2: Region1_Motor_Control(striking); break; // B 电机属于第1区。
    case 3: Region3_Motor_Control(striking); break; // C 电机属于第3区。
    case 4: Region2_Motor_Control(striking); break; // D 电机属于第2区。
    default: break;
    }
}

void Vision_Motor_Init(void)
{
    motor_pwm = VISION_MOTOR_PWM;
    Vision_MotorResult = 0;
    /* 定时动作没有编码器零位要求，上电即可应答底盘的 HOME 确认。 */
    Vision_HomeConfirmed = 1;
    used = received = pending_region = 0;
    pending_dx = Vision_TreeDX = 9999;
    Vision_Region = Vision_Online = 0;
    last_frame = Vision_FrameCount = 0;
    Vision_StateFrameCount = 0;
    Vision_ByteCount = Vision_BadFrameCount = 0;
    Vision_MotorRegion = Vision_MotorPhase = 0;
    phase_started = 0;
    stroke_base = stroke_last = 0;
    stroke_last_move_ms = 0;
    stroke_moved = 0;
    Vision_StrokeLastMs = 0;
    Vision_StrokeLastCounts = 0;
    Work_Control_Init();
    Motor_Stop();
}

/* 校验完整串口帧，成功返回 1，格式错误返回 0。接收中断仅更新数据。
 * 格式示例为 "$VIS,3,-12\r\n"，表示区域 3、树偏移 -12 像素。
 * 区域 0 表示当前没有待处理目标，dx=9999 表示当前没有有效树。
 * 普通 dx 必须在 ±2048 范围内，格式错误的帧不会刷新视觉在线时间。
 */
static uint8_t Parse_Frame(void)
{
    uint8_t pos = 7, digits = 0, negative = 0;
    int32_t value = 0;
    if (used < 10 || memcmp(line, "$VIS,", 5) != 0 ||
        line[5] < '0' || line[5] > '4' || line[6] != ',') return 0;
    if (line[pos] == '-') { negative = 1; pos++; }
    while (pos < used && line[pos] >= '0' && line[pos] <= '9')
    {
        if (++digits > 4) return 0;
        value = value * 10 + (line[pos++] - '0');
    }
    if (digits == 0 || pos + 2 != used ||
        line[pos] != '\r' || line[pos + 1] != '\n') return 0;
    if (negative) value = -value;
    if (value != 9999 && (value < -VISION_TREE_DX_LIMIT ||
                          value > VISION_TREE_DX_LIMIT)) return 0;
    pending_region = (uint8_t)(line[5] - '0');
    pending_dx = value;
    last_frame = HAL_GetTick();
    received = 1;
    Vision_FrameCount++;
    return 1;
}

void Vision_Motor_Receive(const uint8_t *data, uint16_t size)
{
    /* 一帧可能被串口拆成几次接收，也可能一次收到多帧，逐字节拼接。
     * 遇到 $ 重新找帧头，遇到换行再校验，超长帧丢弃后继续找下一帧。
     */
    uint16_t i;
    Vision_ByteCount += size;
    for (i = 0; i < size; i++)
    {
        if (data[i] == '$') used = 0;
        else if (used == 0) continue;
        if (used >= sizeof(line)) { used = 0; Vision_BadFrameCount++; continue; }
        line[used++] = (char)data[i];
        if (data[i] == '\n')
        {
            if (!Parse_Frame()) Vision_BadFrameCount++;
            used = 0;
        }
    }
}

void Vision_Motor_RxError(void)
{
    /* 丢弃出错的半帧，上一条完整合法帧按原时间戳超时。
     * 单次串口错误不撤销在线状态，也不延长合法帧的有效期。 */
    used = 0;
}

void Vision_Motor_Task(void)
{
    uint8_t valid, region;
    int32_t dx;
    uint32_t timestamp, now, primask, frame_count;
    /* 短暂保护共享数据，保证区域、dx、时间戳与帧编号来自同一次读取。
     * 取完恢复原中断状态，后续状态判断和电机计时均在主循环执行。
     */
    primask = __get_PRIMASK();
    __disable_irq();
    valid = received;
    region = pending_region;
    dx = pending_dx;
    timestamp = last_frame;
    frame_count = Vision_FrameCount;
    __set_PRIMASK(primask);
    now = HAL_GetTick();
    if (!valid || (uint32_t)(now - timestamp) >= VISION_TIMEOUT_MS)
    {
        /* 连续无合法帧达到超时阈值，对外显示离线并清除当前目标/树位置。 */
        region = 0;
        dx = 9999;
        valid = 0;
    }

    Vision_Region = region;
    Vision_TreeDX = dx;
    Vision_Online = valid;
    Vision_StateFrameCount = frame_count;

    if (ELBOW_TEST_MODE == 3)
    {
        Motor_Stop();
        Vision_MotorRegion = Vision_MotorPhase = 0;
    }
    else
        Work_Control_Task(now, frame_count); /* 先检查是否允许开始新的肘击。 */
    now = HAL_GetTick();

    /* 使用时间差推进动作，主循环无需停下来等待整段行程。
     * 阶段 1 打击 → 2 换向停顿 → 3 返回 → 4 轮次等待 → 0 空闲。
     * 阶段 1/3 既看时间上限，也看编码器是否已经停走，顶到限位就提前收尾。
     * 时间差使用无符号减法，系统毫秒计数回绕时仍可计算经过时间。
     */
    switch (Vision_MotorPhase)
    {
    case 0: /* 空闲阶段，是否开始下一次动作由上面的 Work_Control_Task 决定。 */
        break;
    case 1:
        /* 打到行程结束或到时间上限，先关 PWM，再进入换向停顿。 */
        if (stroke_finished(now) || (uint32_t)(now - phase_started) >= VISION_CW_MS)
        {
            stroke_record(now);
            Motor_Stop();
            phase_started = now;
            Vision_MotorPhase = 2;
        }
        break;
    case 2: /* 换向前停止 PWM，等待设定的间隔，再开始返回。 */
        if ((uint32_t)(now - phase_started) >= VISION_REVERSE_PAUSE_MS)
        {
            motor_pwm = VISION_MOTOR_PWM;
            stroke_begin(now);
            Motor_Run(Vision_MotorRegion, 0);
            phase_started = now;
            Vision_MotorPhase = 3;
        }
        break;
    case 3:
        /* 回到行程结束或到时间上限，关 PWM 并记录返回结束时刻。
         * 这里依据编码器停走/时间推定返回结束，机构没有回位开关确认。
         */
        if (stroke_finished(now) || (uint32_t)(now - phase_started) >= VISION_CCW_MS)
        {
            stroke_record(now);
            Motor_Stop();
            Vision_MotorRegion = 0;
            phase_started = now;
            Vision_MotorPhase = 4;
        }
        break;
    case 4: /* 一轮结束后的等待阶段，结束后接收新指令。 */
        if ((uint32_t)(now - phase_started) >= VISION_RETRIGGER_DELAY_MS)
        {
            Vision_MotorPhase = 0;
        }
        break;
    default:
        Motor_Stop();
        Vision_MotorRegion = Vision_MotorPhase = 0;
        break;
    }
}

/* 由作业控制器调用。仅接受区域 1～4，并且上一轮必须已完全结束。
 * 成功后按映射锁定电机，后续视觉区域变化不会切换本轮动作。
 * 定时方案不检查编码器零位，上电后直接可以动作。
 */
uint8_t Vision_Motor_Start(uint8_t region)
{
    if (ELBOW_TEST_MODE == 3 || Vision_MotorPhase || region < 1 || region > 4) return 0;
    Vision_MotorRegion = region_to_motor[region];
    Vision_MotorResult = 0;
    motor_pwm = VISION_MOTOR_PWM;
    phase_started = HAL_GetTick();
    stroke_begin(phase_started);
    Motor_Run(Vision_MotorRegion, 1);
    Vision_MotorPhase = 1;
    return 1;
}

/* 底盘发来 HOME 确认时调用。定时方案不需要登记编码器零位，
 * 只要机构不在动作中就记录确认，保持原握手流程不变。 */
uint8_t Vision_Motor_ConfirmHome(void)
{
    if (Vision_MotorPhase) return 0;
    Vision_HomeConfirmed = 1;
    Vision_MotorResult = 0;
    return 1;
}

/* 作业流程在放行前调用。定时方案没有编码器回位核验，
 * 机构空闲且没有结果码即视为已经收回。 */
uint8_t Vision_Motor_VerifyReturned(void)
{
    return Vision_MotorPhase == 0 && Vision_MotorResult == 0;
}

/* 作业控制器在空闲复检前读取，此时 phase_started 仍保存返回结束时刻。 */
uint32_t Vision_Motor_ReturnedAt(void) { return phase_started; }

/* 供 OLED 在主循环中读取当前阶段的剩余毫秒数。 */
uint32_t Vision_Motor_RemainingMs(void)
{
    uint32_t duration, elapsed;
    switch (Vision_MotorPhase)
    {
    case 1: duration = VISION_CW_MS; break;
    case 2: duration = VISION_REVERSE_PAUSE_MS; break;
    case 3: duration = VISION_CCW_MS; break;
    case 4: duration = VISION_RETRIGGER_DELAY_MS; break;
    default: return 0;
    }
    elapsed = (uint32_t)(HAL_GetTick() - phase_started);
    return elapsed >= duration ? 0 : duration - elapsed;
}

uint8_t Vision_Motor_IsArmed(void)
{
    /* 协同复检或独立测试等待阶段且机构空闲时，具备启动动作的阶段条件。
     * 目标稳定等详细条件还会在对应作业流程中继续检查。
     */
    return Vision_MotorPhase == 0 &&
        (Vision_WorkState == VISION_WORK_CHECK || Vision_WorkState == VISION_WORK_TEST);
}