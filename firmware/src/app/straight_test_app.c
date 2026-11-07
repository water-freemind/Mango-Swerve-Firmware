#include "app_config.h"
#if APP_RUN_MODE == APP_MODE_STRAIGHT_TEST
#include "straight_test_app.h"
#include "straight_test.h"
#include "assemble/assemble.h"
#include "chassis.h"
#include "chassis_yaw_hold.h"
#include "steering_calibration.h"
#include "chassis_can_map.h"
#include "stm32_hal_can.h"
#include "stm32_hal_uart.h"
#include "stm32_hal_dwt.h"
#include "imu/imu.h"
#include "remote.h"
#include "work_link.h"
#include "laser.h"
#include "log.h"
#include <math.h>
#include <stddef.h>
#include <stdlib.h>

static StraightTest test;
static bool initialized;
static uint32_t last_step, last_log, last_imu;
static volatile uint32_t motor_ms[8];
static volatile uint8_t seen, motor_state[4];
static uint8_t byte;
static volatile uint8_t rx_ring[128], rx_head, rx_tail;
static volatile bool rx_fault;
static char line[48];
static unsigned line_len;
static const float zeros[4] = STEERING_FORWARD_ZERO_RAD;
static bool tx_fault;

static void stop_outputs(void) {
    for(uint16_t id = 1u; id <= 4u; ++id) {
        BusMotorFeedback f;
        if(drive_motor.stop(id) != MOTOR_STATUS_OK) tx_fault = true;
        /* No position command until this boot has actually received the motor angle. */
        if(steer_motor.update_feedback(id, &f) == MOTOR_STATUS_OK && isfinite(f.position)) {
            if(steer_motor.brake(id) != MOTOR_STATUS_OK) tx_fault = true;
        } else if(steer_motor.disable(id) != MOTOR_STATUS_OK) tx_fault = true;
    }
}

static void received(void) {
    const uint8_t next = (uint8_t)((rx_head + 1u) & 127u);
    if(next == rx_tail) rx_fault = true;
    else { rx_ring[rx_head] = byte; rx_head = next; }
    if(!uart_receive_it(&huart1, &byte, 1u)) rx_fault = true;
}
static void uart_error(void) { rx_fault = true; }
static void motor_received(FDCAN_HandleTypeDef* bus, const FDCAN_RxHeaderTypeDef* h,
                           const uint8_t data[8], void* user) {
    unsigned index;
    (void)user;
    if(h->IdType != FDCAN_STANDARD_ID || h->RxFrameType != FDCAN_DATA_FRAME ||
       h->FDFormat != FDCAN_CLASSIC_CAN || h->DataLength != FDCAN_DLC_BYTES_8) return;
    if(bus == CHASSIS_STEER_CAN && h->Identifier >= 0x11u && h->Identifier <= 0x14u) {
        index = h->Identifier - 0x11u;
        if((data[0] & 15u) != index + 1u) return;
        motor_state[index] = data[0] >> 4;
    } else if(bus == CHASSIS_DRIVE_CAN && h->Identifier >= 0x201u && h->Identifier <= 0x204u) {
        index = 4u + h->Identifier - 0x201u;
    } else return;
    motor_ms[index] = HAL_GetTick();
    seen |= (uint8_t)(1u << index);
}
static bool remote_permit(void) {
    RemoteState rc;
    remote_process();
    if(!remote_get_state(&rc) || !remote_is_online(100u) || !rc.manual_request ||
       rc.manual_source != REMOTE_MANUAL_SOURCE_CHASSIS ||
       rc.rc_data.channel[REMOTE_CH_SWC] == REMOTE_SW_HIGH ||
       rc.rc_data.channel[REMOTE_CH_VRA] <= REMOTE_VR_LOW_THRESHOLD ||
       rc.rc_data.channel[REMOTE_CH_VRB] > REMOTE_VR_LOW_THRESHOLD) return false;
    const unsigned channels[] = {REMOTE_CH_RIGHT_X, REMOTE_CH_RIGHT_Y, REMOTE_CH_LEFT_X};
    for(unsigned i = 0u; i < 3u; ++i)
        if(abs((int)rc.rc_data.channel[channels[i]] - (int)REMOTE_CENTER) > REMOTE_DEADBAND) return false;
    return true;
}
static StraightInput input(uint32_t now) {
    StraightInput in = {.healthy = true, .stationary = true, .vx_mps = 0.0f};
    ImuSample sample;
    const uint32_t key = __get_PRIMASK();
    __disable_irq();
    in.healthy = seen == 255u;
    for(unsigned i = 0u; i < 8u; ++i) in.healthy = in.healthy && now - motor_ms[i] <= 100u;
    for(unsigned i = 0u; i < 4u; ++i) in.healthy = in.healthy && motor_state[i] == 1u;
    __set_PRIMASK(key);
    in.healthy = in.healthy && remote_permit() && !rx_fault && !tx_fault &&
        imu_get_sample(&sample) == IMU_STATUS_OK;
    if(in.healthy) in.healthy = (uint32_t)(dwt_get_us() - sample.gyro_timestamp_us) <= 100000u;
    for(uint16_t id = 1u; id <= 4u; ++id) {
        const float sign = (id == 2u || id == 3u) ? -1.0f : 1.0f;
        const float speed = drive_motor.get_spd(id);
        const float angle = steer_motor.get_pos(id) - zeros[id - 1u];
        in.healthy = in.healthy && isfinite(speed) && isfinite(angle);
        in.stationary = in.stationary && fabsf(speed) < 0.05f;
        in.vx_mps += sign * speed * STRAIGHT_TEST_WHEEL_RADIUS_M * cosf(angle) * 0.25f;
    }
    const ImuAngle attitude = imu.get_angle();
    const ImuGyro gyro = imu_get_gyro_corrected();
    in.healthy = in.healthy && isfinite(attitude.yaw) && isfinite(gyro.z);
    return in;
}
static void commands(uint32_t now, StraightInput in) {
    if(rx_fault) {
        straight_fault(&test, "UART_ERROR_OR_OVERFLOW");
        rx_tail = rx_head;
        line_len = 0u;
        (void)uart_abort_receive_it(&huart1);
        rx_fault = false;
        if(!uart_receive_it(&huart1, &byte, 1u)) rx_fault = true;
        return;
    }
    /* Bound work per loop; sustained flooding cannot starve the control watchdog. */
    for(unsigned n = 0u; n < 128u && rx_tail != rx_head; ++n) {
        const uint8_t ch = rx_ring[rx_tail];
        rx_tail = (uint8_t)((rx_tail + 1u) & 127u);
        if(ch == '\r' || ch == '\n') {
            if(line_len == 0u) continue;
            line[line_len] = '\0';
            const StraightPhase before = test.phase;
            const bool ok = straight_command(&test, line, now, in);
            if(before == ST_ARMED && test.phase == ST_RUNNING) {
                const ImuAngle a = imu.get_angle();
                chassis_yaw_hold_set_target(a.yaw);
            }
            if(line[0] != 'H') log_info("TEST_ACK ok=%u phase=%u token=%lu reason=%s",
                (unsigned)ok, (unsigned)test.phase, (unsigned long)test.token, test.result);
            line_len = 0u;
        } else if(ch < 32u || ch > 126u || line_len >= sizeof(line) - 1u) {
            rx_fault = true;
            break;
        } else line[line_len++] = (char)ch;
    }
}
static bool can_healthy(void) {
    STM32HalCanDiagnostics a, b;
    return can_get_diagnostics(CHASSIS_STEER_CAN, &a) && can_get_diagnostics(CHASSIS_DRIVE_CAN, &b) &&
        !a.bus_off && !b.bus_off && !a.error_passive && !b.error_passive &&
        a.tx_failed_count == 0u && b.tx_failed_count == 0u;
}
void straight_test_app_init(void) {
    initialized = false;
    (void)assemble_delay();
    if(assemble_log() != SYSTEM_STATUS_OK) return;
    straight_init(&test, HAL_GetTick());
    if(assemble_imu() != SYSTEM_STATUS_OK || assemble_laser() != SYSTEM_STATUS_OK ||
       !work_link_init() || assemble_can3() != SYSTEM_STATUS_OK ||
       can_register_rx_callback(CHASSIS_STEER_CAN, motor_received, NULL) != STM32_HAL_CAN_OK ||
       can_register_rx_callback(CHASSIS_DRIVE_CAN, motor_received, NULL) != STM32_HAL_CAN_OK ||
       assemble_chassis() != SYSTEM_STATUS_OK || assemble_remote() != SYSTEM_STATUS_OK) {
        straight_fault(&test, "INIT_FAILED");
        if(chassis.get_chassis()->initialized) stop_outputs();
        log_error("TEST_INIT_FAILED");
        return;
    }
    remote_init();
    const ChassisYawHoldConfig heading = {.enabled = true, .kp = APP_YAW_HOLD_KP,
        .kd = APP_YAW_HOLD_KD, .v_deadband = 0.01f, .wz_deadband = 0.0f, .wz_limit = APP_YAW_HOLD_MAX_WZ};
    chassis_yaw_hold_init(&heading);
    uart_register_rx_complete_callback(&huart1, received);
    uart_register_error_callback(&huart1, uart_error);
    rx_fault = !uart_receive_it(&huart1, &byte, 1u);
    (void)chassis.set_steer_then_drive_enabled(true);
    stop_outputs();
    initialized = true;
    last_step = last_imu = HAL_GetTick();
    log_info("TEST_READY mode=6 target=2.0m speed<=0.10m/s radius=0.051 NOMINAL; PING ARM GO <token> HB <token> STOP; no automatic restart");
}
void straight_test_app_loop(void) {
    const uint32_t now = HAL_GetTick();
    if(!initialized) return;
    can_poll_rx(CHASSIS_STEER_CAN);
    can_poll_rx(CHASSIS_DRIVE_CAN);
    can_poll_rx(&hfdcan3);
    if(now - last_imu >= 4u) {
        last_imu = now;
        (void)imu.update(); /* Sample timestamps below enforce the 100 ms age limit. */
    }
    StraightInput in = input(now);
    in.healthy = in.healthy && can_healthy();
    commands(now, in);
    if(rx_fault) straight_fault(&test, "UART_ERROR_OR_OVERFLOW");
    if(test.phase == ST_RUNNING) {
        const ImuAngle a = imu.get_angle();
        const float error = remainderf(chassis_yaw_hold_get_yaw_ref() - a.yaw, 6.283185307f);
        if(!isfinite(error) || fabsf(error) > 0.15f) straight_fault(&test, "HEADING_DEVIATION");
    }
    if(now - last_step >= 10u) {
        last_step = now;
        straight_step(&test, now, in);
        if(test.phase == ST_RUNNING) {
            const ImuAngle a = imu.get_angle();
            const ImuGyro g = imu_get_gyro_corrected();
            const float wz = chassis_yaw_hold_apply(test.speed_mps, 0.0f, 0.0f, a.yaw, g.z, 0.01f);
            if(chassis.set_velocity(test.speed_mps, 0.0f, wz) != chassis.OK || chassis.process() != chassis.OK)
                straight_fault(&test, "CHASSIS_CONTROL");
        } else chassis_yaw_hold_disable();
        if(test.phase != ST_RUNNING) {
            stop_outputs();
        }
        if(tx_fault || !can_healthy()) straight_fault(&test, "CAN_TX_OR_BUS");
    }
    if(now - last_log >= 100u) {
        LaserSnapshot laser = {0};
        (void)laser_get_snapshot(&laser);
        last_log = now;
        const ImuAngle a = imu.get_angle();
        log_info("TEST_STATUS phase=%u token=%lu healthy=%u stopped=%u s=%f vx=%f cmd=%f yaw=%f ref=%f correction=%f right_mm=%u laser_fresh=%u result=%s",
            (unsigned)test.phase, (unsigned long)test.token, (unsigned)in.healthy, (unsigned)in.stationary,
            (double)test.distance_m, (double)in.vx_mps, (double)test.speed_mps, (double)a.yaw,
            (double)chassis_yaw_hold_get_yaw_ref(), (double)chassis_yaw_hold_get_output_wz(),
            (unsigned)laser.distance_mm[1], (unsigned)laser.frame_fresh, test.result);
        /* Dedicated numeric frame; human-readable status is not a FireWater channel. */
        const LogVofaValue wave[4] = {
            log_vofa_value_f64((double)a.yaw * 57.295779513),
            log_vofa_value_f64((double)chassis_yaw_hold_get_yaw_ref() * 57.295779513),
            log_vofa_value_f64((double)chassis_yaw_hold_get_yaw_error() * 57.295779513),
            log_vofa_value_f64((double)chassis_yaw_hold_get_output_wz()),
        };
        (void)log_vofa_write("FW", 4u, wave);
    }
}
#endif
