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

/* The interlock is exercised with both production endpoints in test_work_can.c. */
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
    assert(VISION_MOTOR_PWM == 10U);
    if (ELBOW_TEST_MODE == 2)
    {
        const int signs[] = {1, 1, -1, -1};
        for (region = 1; region <= 4; region++)
        {
            unsigned index = motors[region]-1U;
            memset((void *)Encoder_Position, 0, sizeof(Encoder_Position));
            Vision_Motor_Init();
            assert(!Vision_Motor_Start((uint8_t)region));
            assert(Vision_Motor_ConfirmHome());
            assert(Vision_Motor_Start((uint8_t)region));
            check_duty(motors[region], ELBOW_POSITION_PWM);
            assert(Vision_ExtendTarget > Vision_RetractTarget && Vision_RetractTarget > 0);
            Encoder_Position[index] = signs[index] * (Vision_ExtendTarget - 100);
            tick += 20U; Vision_Motor_Task();
            assert(Vision_MotorPhase == 1 && !Vision_MotorResult);
            check_duty(motors[region], ELBOW_POSITION_SLOW_PWM);
            Encoder_Position[index] = signs[index] * Vision_ExtendTarget;
            tick += 20U;
            Vision_Motor_EncoderTick();
            assert(Vision_MotorPhase == 1);
            check_pwm(0);
            Vision_Motor_EncoderTick();
            check_pwm(0);
            Vision_Motor_Task();
            assert(Vision_MotorPhase == 2);
            check_pwm(0);
            tick += VISION_REVERSE_PAUSE_MS; Vision_Motor_Task();
            assert(Vision_MotorPhase == 3);
            check_duty(motors[region], ELBOW_POSITION_RETURN_PWM);
            Encoder_Position[index] = signs[index] * (Vision_RetractTarget + 100);
            tick += 20U; Vision_Motor_Task();
            check_duty(motors[region], ELBOW_POSITION_SLOW_PWM);
            Encoder_Position[index] = signs[index] * Vision_RetractTarget;
            tick += 20U; Vision_Motor_Task();
            assert(Vision_MotorPhase == 4 && !Vision_MotorResult);
            check_pwm(0);
        }
        Vision_Motor_Init();
        assert(Vision_Motor_ConfirmHome() && Vision_Motor_Start(1));
        tick += ELBOW_POSITION_STALL_MS; Vision_Motor_Task();
        assert(Vision_MotorResult == STATION_RESULT_ENCODER_STALL && !Vision_HomeConfirmed && !Vision_MotorPhase);
        assert(Vision_FaultMotor == 2 && Vision_FaultPhase == 1);
        assert(Vision_FaultProgress == 0 && Vision_FaultTarget == Vision_ExtendTarget);
        check_pwm(0);
        Vision_Motor_Init();
        assert(Vision_Motor_ConfirmHome() && Vision_Motor_Start(1));
        Encoder_Position[1] += ELBOW_STROKE_B_COUNTS * 96 / 100;
        tick += 20U; Vision_Motor_Task();
        assert(Vision_MotorResult == STATION_RESULT_ENCODER_LIMIT && !Vision_HomeConfirmed && !Vision_MotorPhase);
        assert(Vision_FaultMotor == 2 && Vision_FaultProgress > Vision_ExtendTarget);
        check_pwm(0);
        puts("ENCODER_DUTY_PROFILE_OK strike_80_return_40_slow_35_soft_margin_10_overrun_stops");
        Vision_Motor_Init();
        assert(Vision_Motor_ConfirmHome() && Vision_Motor_Start(1));
        Encoder_Position[1] -= 20;
        tick += 20U; Vision_Motor_Task();
        assert(Vision_MotorResult == STATION_RESULT_ENCODER_DIRECTION && !Vision_MotorPhase);
        assert(Vision_FaultMotor == 2 && Vision_FaultProgress == -20);
        check_pwm(0);
        Vision_Motor_Init();
        assert(Vision_Motor_ConfirmHome() && Vision_Motor_Start(1));
        Encoder_Position[1] += 2;
        tick += ELBOW_POSITION_TIMEOUT_MS;
        Vision_Motor_EncoderTick();
        assert(Vision_MotorResult == STATION_RESULT_ENCODER_TIMEOUT && !Vision_HomeConfirmed);
        assert(Vision_FaultMotor == 2 && Vision_FaultPhase == 1 && Vision_FaultProgress == 2);
        check_pwm(0);
        puts("ENCODER_POSITION_LIMITS_OK four_regions_home_required_two_end_margins_stall_wrong_direction_stop");
        return 0;
    }
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
    Vision_Motor_Init(); check_pwm(0);
    feed("noise$VIS,1,"); feed("-80\r\n");
    assert(Vision_Region == 1 && Vision_TreeDX == -80 && Vision_FrameCount == 1);
    check_pwm(0); /* Parsing a target alone cannot start an action. */
    frames = Vision_FrameCount;
    feed("$VIS,5,0\r\n$VIS,1,99999\r\n$VIS,1,1junk\r\n");
    feed("$VIS,1,\r\n$VIS,1,3000\r\n$VIS,1,0\n");
    assert(Vision_FrameCount == frames && Vision_BadFrameCount == 6);
    for(region = 1; region <= 4; region++) {
        Vision_Motor_Init();
        tick = UINT32_MAX - 100;
        assert(Vision_Motor_Start((uint8_t)region));
        assert(!Vision_Motor_Start(1));
        check_pwm(motors[region]); check_direction(region, 1);
        assert(Vision_Motor_RemainingMs() == VISION_CW_MS);
        feed("$VIS,0,9999\r\n"); check_pwm(motors[region]);
        tick += VISION_CW_MS; Vision_Motor_Task(); check_pwm(0);
        assert(Vision_MotorPhase == 2);
        tick += VISION_REVERSE_PAUSE_MS; Vision_Motor_Task();
        check_pwm(motors[region]); check_direction(region, 0);
        tick += VISION_CCW_MS; Vision_Motor_Task(); check_pwm(0);
        assert(Vision_MotorPhase == 4 && Vision_Motor_ReturnedAt() == tick);
        tick += VISION_RETRIGGER_DELAY_MS; Vision_Motor_Task();
        assert(Vision_MotorPhase == 0);
    }
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
    puts("VISION_UART_PARSER_DIRECTION_TIMING_WRAP_TEST_OK");
    return 0;
}
