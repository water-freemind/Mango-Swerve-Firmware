#include "vision_motor.h"
#include "tim.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

FakeTimer timer;
volatile int32_t Encoder_Position[4];
GPIO_TypeDef portc, portd;
UART_HandleTypeDef huart3;
char uart_rx[32];
volatile uint8_t uart_flag;
volatile uint16_t uart_rx_size;
volatile uint32_t Vision_UART_ErrorCount, Vision_UART_LastError, Vision_UART_ArmErrorCount;
volatile uint8_t Vision_WorkState, Vision_WorkResult;
volatile uint32_t Vision_WorkTask;
static uint32_t tick;
static unsigned pins_c, pins_d, arm_count;
static int arm_result;

void Work_Control_Init(void) { Vision_WorkState = VISION_WORK_BOOT; }
void Work_Control_Task(uint32_t now, uint32_t count) { (void)now; (void)count; }
uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return 0; }
void __disable_irq(void) {}
void __set_PRIMASK(uint32_t value) { (void)value; }
void HAL_GPIO_WritePin(GPIO_TypeDef* port, uint16_t pin, int state) {
    unsigned* pins = port == GPIOC ? &pins_c : &pins_d;
    if(state) *pins |= pin; else *pins &= ~pin;
}
int HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef* h, uint8_t* data, uint16_t size) {
    int result = arm_result;
    assert(data == (uint8_t*)uart_rx && size == sizeof(uart_rx));
    arm_count++; arm_result = 0;
    if(result) return result;
    h->RxState = BUSY_RX;
    return HAL_OK;
}
int HAL_UART_AbortReceive(UART_HandleTypeDef* h) { h->RxState = HAL_UART_STATE_READY; return HAL_OK; }
uint32_t HAL_UART_GetError(UART_HandleTypeDef* h) { return h->ErrorCode; }

static void feed(const char* frame) {
    Vision_Motor_Receive((const uint8_t*)frame, (uint16_t)strlen(frame));
    Vision_Motor_Task();
}
static void check_duty(unsigned motor, unsigned duty) {
    assert(timer.CCR1 == (motor == 1 ? duty : 0));
    assert(timer.CCR2 == (motor == 2 ? duty : 0));
    assert(timer.CCR3 == (motor == 3 ? duty : 0));
    assert(timer.CCR4 == (motor == 4 ? duty : 0));
}
static void check_pwm(unsigned motor) {
    check_duty(motor, VISION_MOTOR_PWM);
}
static void check_direction(unsigned region, int striking) {
    const unsigned high[] = {0, GPIO_PIN_2, GPIO_PIN_5, GPIO_PIN_5, GPIO_PIN_0};
    const unsigned low[] = {0, GPIO_PIN_3, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_1};
    unsigned pins = region == 2 ? pins_d : pins_c;
    assert(!!(pins & high[region]) == !!striking);
    assert(!!(pins & low[region]) == !striking);
}
int main(void) {
    unsigned region;
    const unsigned motors[] = {0, 2, 4, 3, 1};
    uint32_t frames;
    /* 四段均为满占空比，只有完整作业之间停 50 ms。 */
    assert(VISION_MOTOR_PWM == 100U);
    assert(VISION_CW_MS == 600U && VISION_CCW_MS == 600U);
    assert(VISION_PARTIAL_RETURN_MS == 300U && VISION_SECOND_EXTEND_MS == 320U);
    if (ELBOW_TEST_MODE == 3)
    {
        Vision_Motor_Init();
        Encoder_Position[0] = 123;
        Encoder_Position[1] = -456;
        Encoder_Position[2] = 789;
        Encoder_Position[3] = -12;
        for (region = 1; region <= 4; region++)
        {
            assert(!Vision_Motor_Start((uint8_t)region));
            timer.CCR1 = timer.CCR2 = timer.CCR3 = timer.CCR4 = 99;
            feed("$VIS,1,-35\r\n");
            tick += 1000U;
            Vision_Motor_Task();
            check_pwm(0);
            assert(!Vision_MotorPhase && !Vision_MotorRegion);
        }
        assert(Encoder_Position[0] == 123 && Encoder_Position[1] == -456);
        assert(Encoder_Position[2] == 789 && Encoder_Position[3] == -12);
        puts("MANUAL_ENCODER_MODE_OK all_outputs_zero_visual_and_api_cannot_start_counts_preserved");
        return 0;
    }
    const struct { unsigned phase, ms; int direction; } sequence[] = {
        {VISION_MOTOR_EXTEND_FIRST, 600, 1},
        {VISION_MOTOR_RETURN_PARTIAL, 300, -1},
        {VISION_MOTOR_EXTEND_SECOND, 320, 1},
        {VISION_MOTOR_RETURN_FINAL, 600, -1},
    };
    for (region = 1; region <= 4; region++) {
        Vision_Motor_Init();
        assert(Vision_HomeConfirmed && Vision_Motor_VerifyReturned());
        tick = UINT32_MAX - 100;
        const uint32_t start = tick;
        assert(Vision_Motor_Start((uint8_t)region));
        feed("$VIS,0,9999\r\n");
        for (unsigned i = 0; i < sizeof(sequence)/sizeof(sequence[0]); ++i) {
            assert(Vision_MotorPhase == sequence[i].phase);
            assert(Vision_MotorRegion == motors[region] && !Vision_Motor_VerifyReturned());
            assert(!Vision_Motor_Start(1));
            assert(Vision_Motor_RemainingMs() == sequence[i].ms);
            const unsigned powered = sequence[i].direction ? motors[region] : 0;
            check_pwm(powered);
            if (powered) check_direction(region, sequence[i].direction > 0);
            tick += sequence[i].ms - 1;
            Vision_Motor_Task();
            check_pwm(powered);
            assert(Vision_MotorPhase == sequence[i].phase && Vision_Motor_RemainingMs() == 1);
            ++tick;
            Vision_Motor_Task();
        }
        assert((uint32_t)(tick-start) == 1820U && Vision_Motor_ReturnedAt() == tick);
        assert(Vision_MotorPhase == VISION_MOTOR_COOLDOWN && !Vision_MotorRegion);
        assert(!Vision_Motor_VerifyReturned());
        check_pwm(0);
        tick += VISION_RETRIGGER_DELAY_MS - 1;
        Vision_Motor_Task();
        assert(!Vision_Motor_Start((uint8_t)region));
        check_pwm(0);
        ++tick;
        Vision_Motor_Task();
        assert(Vision_MotorPhase == VISION_MOTOR_IDLE && Vision_Motor_VerifyReturned());
        assert((uint32_t)(tick-start) == 1870U && (uint32_t)(Vision_Motor_ReturnedAt()-start) == 1820U);
    }
    puts("TIMED_STRIKE_OK four_regions_100pct_600_300_320_600ms_50ms_between_cycles");
    puts("MOTOR_MUTEX_OK single_channel_pwm_and_pause_outputs_zero");
#if ELBOW_STROKE_END_MS > 0
    /* 编码器停走＝顶到机械限位：不等满时间上限就结束本段，避免长时间堵转。 */
    Vision_Motor_Init(); tick = UINT32_MAX - 3000;  /* 同时覆盖毫秒回绕 */
    memset((void *)Encoder_Position, 0, sizeof(Encoder_Position));
    assert(Vision_Motor_Start(2));                   /* 2 区 → D 电机 → 下标 3 */
    tick += 100; Encoder_Position[3] = -120; Vision_Motor_Task();
    assert(Vision_MotorPhase == 1);
    tick += ELBOW_STROKE_END_MS - 1; Vision_Motor_Task();
    assert(Vision_MotorPhase == 1);
    tick += 1; Vision_Motor_Task();
    assert(Vision_MotorPhase == VISION_MOTOR_RETURN_PARTIAL);
    check_pwm(motors[2]); check_direction(2, 0);
    assert(Vision_StrokeLastMs == 100 + ELBOW_STROKE_END_MS);
    assert(Vision_StrokeLastCounts == -120);
    puts("STROKE_END_DETECT_OK encoder_stall_ends_phase_before_time_cap");
#endif
    /* 默认纯定时：编码器不动也按时间走满，行为与 elbow-controller 一致。 */
    Vision_Motor_Init(); tick = 1000;
    memset((void *)Encoder_Position, 0, sizeof(Encoder_Position));
    assert(Vision_Motor_Start(3));
    tick += VISION_CW_MS - 1; Vision_Motor_Task();
    assert(Vision_MotorPhase == 1 && Vision_Motor_RemainingMs() == 1);
    check_pwm(motors[3]);
    tick += 1; Vision_Motor_Task();
    assert(Vision_MotorPhase == VISION_MOTOR_RETURN_PARTIAL);
    check_pwm(motors[3]); check_direction(3, 0);
    assert(Vision_StrokeLastMs == VISION_CW_MS && Vision_StrokeLastCounts == 0);
    puts("TIMED_FULL_STROKE_OK dead_or_idle_encoder_still_runs_full_time");
    if (ELBOW_TEST_MODE == 2) {
        /* 底盘 HOME 握手照常应答；动作中复核返回状态为假，空闲为真。 */
        Vision_Motor_Init(); tick = 5000;
        assert(Vision_Motor_ConfirmHome());
        assert(Vision_Motor_Start(1));
        assert(!Vision_Motor_VerifyReturned());
        assert(!Vision_Motor_ConfirmHome()); /* 动作中不能再次登记。 */
        /* 任何一段收到取消后都只能停顿或向后回收。 */
        for (unsigned cancel_at = 0; cancel_at < sizeof(sequence)/sizeof(sequence[0]); ++cancel_at) {
            Vision_Motor_Init();
            tick = 5000;
            assert(Vision_Motor_Start(1));
            for (unsigned i = 0; i < cancel_at; ++i) {
                tick += sequence[i].ms;
                Vision_Motor_Task();
            }
            Vision_Motor_RequestReturn();
            const uint32_t cancel = tick;
            unsigned returned_drive_ms = 0;
            while (!Vision_Motor_VerifyReturned()) {
                assert((uint32_t)(tick - cancel) < 670U);
                if (timer.CCR2) {
                    check_direction(1, 0);
                    ++returned_drive_ms;
                }
                Vision_Motor_RequestReturn();
                ++tick;
                Vision_Motor_Task();
            }
            assert(returned_drive_ms == VISION_CCW_MS);
            check_pwm(0);
        }
        puts("TIMED_HOME_AND_FAULT_RETURN_OK every_phase_skips_remaining_extension");
        /* 空闲时请求返回不产生动作。 */
        Vision_Motor_RequestReturn();
        assert(Vision_MotorPhase == 0); check_pwm(0);
    }
    Vision_Motor_Init(); check_pwm(0);
    feed("noise$VIS,1,"); feed("-80\r\n");
    assert(Vision_Region == 1 && Vision_TreeDX == -80 && Vision_FrameCount == 1);
    check_pwm(0); /* Parsing a target alone cannot start an action. */
    frames = Vision_FrameCount;
    feed("$VIS,5,0\r\n$VIS,1,99999\r\n$VIS,1,1junk\r\n");
    feed("$VIS,1,\r\n$VIS,1,3000\r\n$VIS,1,0\n");
    assert(Vision_FrameCount == frames && Vision_BadFrameCount == 6);
    Vision_Motor_Init();
    huart3.Instance = USART3; huart3.RxState = HAL_UART_STATE_READY;
    Vision_UART_Task(); assert(huart3.RxState == BUSY_RX && arm_count == 1);
    strcpy(uart_rx, "$VIS,0,9999\r\n"); huart3.RxState = HAL_UART_STATE_READY;
    HAL_UARTEx_RxEventCallback(&huart3, (uint16_t)strlen(uart_rx));
    Vision_UART_Task();
    assert(Vision_Online && Vision_FrameCount == 1 && Vision_ByteCount == 13);
    strcpy(uart_rx, "$VIS,1,"); huart3.RxState = HAL_UART_STATE_READY;
    HAL_UARTEx_RxEventCallback(&huart3, 7);
    strcpy(uart_rx, "0\r\n"); huart3.RxState = HAL_UART_STATE_READY;
    arm_result = HAL_ERROR;
    HAL_UARTEx_RxEventCallback(&huart3, 3);
    Vision_UART_Task();
    assert(Vision_UART_ArmErrorCount == 1 && huart3.RxState == BUSY_RX);
    assert(Vision_Region == 1); check_pwm(0);
    huart3.ErrorCode = 8; HAL_UART_ErrorCallback(&huart3); Vision_UART_Task();
    assert(Vision_UART_ErrorCount == 1 && Vision_UART_LastError == 8 && Vision_Online);
    assert(Vision_Region == 1 && Vision_FrameCount == 2);
    check_pwm(0);

    tick = UINT32_MAX - VISION_TIMEOUT_MS / 2U;
    Vision_Motor_Init();
    feed("$VIS,3,20\r\n");
    tick += VISION_TIMEOUT_MS - 1U;
    feed("$VIS,2,");
    Vision_Motor_RxError();
    feed("0\r\n");
    assert(Vision_Online && Vision_Region == 3 && Vision_FrameCount == 1);
    tick++;
    Vision_Motor_Task();
    assert(!Vision_Online && Vision_Region == 0 && Vision_TreeDX == 9999);
    feed("$VIS,5,20\r\n");
    Vision_Motor_RxError();
    Vision_Motor_Task();
    assert(!Vision_Online && Vision_FrameCount == 1);
    feed("$VIS,4,-20\r\n");
    assert(Vision_Online && Vision_Region == 4 && Vision_FrameCount == 2);
    check_pwm(0);
    puts("VISION_OFF_FILTER_OK short_gap_uart_error_no_false_off_exact_timeout_wrap_fresh_recovery");
    {
        extern volatile uint16_t Car_ButtonSeq;
        extern volatile uint8_t Car_ButtonOp;
        uint32_t frames=Vision_FrameCount, bad=Vision_BadFrameCount;
        feed("$CAR,123,"); feed("2\r\n");
        assert(Car_ButtonSeq==123 && Car_ButtonOp==2);
        assert(Vision_FrameCount==frames && Vision_BadFrameCount==bad);
        feed("$CAR,65535,6\r\n");
        assert(Car_ButtonSeq==65535 && Car_ButtonOp==6);
        feed("$CAR,7,"); feed("8\r\n");
        assert(Car_ButtonSeq==7 && Car_ButtonOp==8);
        feed("$CAR,8,9\r\n");
        assert(Car_ButtonSeq==8 && Car_ButtonOp==9);
        feed("$CAR,65536,5\r\n"); feed("$CAR,0,1\r\n"); feed("$CAR,2,0\r\n");
        assert(Car_ButtonSeq==8 && Car_ButtonOp==9 && Vision_BadFrameCount==bad+3);
        assert(Vision_FrameCount==frames);
        puts("CAR_UART_FRAGMENT_BOUNDS_AND_VISION_ISOLATION_OK");
    }
    puts("VISION_UART_PARSER_DIRECTION_TIMING_WRAP_TEST_OK");
    return 0;
}
