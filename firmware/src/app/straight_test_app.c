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
#include "imu/bmi088.h"
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

/* Capture each sensor frame once. BMI088 get_sample() consumes its NEW flags. */
static ImuSample imu_sample;
static bool imu_seen;
static ImuStatus imu_update_status = IMU_STATUS_NOT_READY;
static uint32_t imu_dt_us, imu_dt_max_us, imu_samples, imu_errors;
static uint32_t control_dt_ms, control_dt_max_ms, detail_slot;
static uint32_t motor_age[8], blockers;
static uint8_t feedback_seen, enabled_mask;
static RemoteState rc_state;
static bool rc_online, rc_switches, rc_centered;
static Bmi088AttitudeDebug imu_debug;
static bool imu_debug_valid;
static uint32_t imu_age_us;
static bool drift_tracking, drift_valid;
static uint32_t drift_start_us, drift_last_us, drift_window_ms;
static float drift_last_yaw, drift_yaw_sum, drift_gyro_sum, drift_yaw_rate, drift_gyro_rate;
enum {
    BLOCK_STEER_RX = 1u << 0, BLOCK_DRIVE_RX = 1u << 1, BLOCK_STEER_STATE = 1u << 2,
    BLOCK_RC_OFFLINE = 1u << 3, BLOCK_RC_SWITCH = 1u << 4, BLOCK_RC_STICK = 1u << 5,
    BLOCK_IMU_SAMPLE = 1u << 6, BLOCK_IMU_STALE = 1u << 7, BLOCK_IMU_CAL = 1u << 8,
    BLOCK_NONFINITE = 1u << 9, BLOCK_UART = 1u << 10, BLOCK_TX = 1u << 11,
    BLOCK_CAN = 1u << 12
};
static const char* first_blocker(void) {
    static const char* const names[] = {"STEER_FEEDBACK", "DRIVE_FEEDBACK", "STEER_STATE",
        "RC_OFFLINE", "RC_SWITCH", "RC_STICK", "IMU_SAMPLE", "IMU_STALE", "IMU_CALIB",
        "NONFINITE", "UART_RX", "CAN_TX", "CAN_BUS"};
    for(unsigned i = 0u; i < sizeof(names) / sizeof(names[0]); ++i)
        if(blockers & (1u << i)) return names[i];
    return "NONE";
}
static void update_imu_sample(void) {
    ImuSample fresh;
    imu_update_status = imu.update();
    if(imu_update_status != IMU_STATUS_OK) { ++imu_errors; return; }
    if(imu_get_sample(&fresh) != IMU_STATUS_OK || !(fresh.flags & IMU_SAMPLE_GYRO_VALID)) return;
    if(imu_seen) {
        imu_dt_us = fresh.gyro_timestamp_us - imu_sample.gyro_timestamp_us;
        if(imu_dt_us > imu_dt_max_us) imu_dt_max_us = imu_dt_us;
    }
    imu_sample = fresh;
    imu_seen = true;
    ++imu_samples;
}

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
    remote_process();
    rc_online = remote_get_state(&rc_state) && remote_is_online(100u);
    rc_switches = rc_state.manual_request && rc_state.manual_source == REMOTE_MANUAL_SOURCE_CHASSIS &&
        rc_state.rc_data.channel[REMOTE_CH_SWC] != REMOTE_SW_HIGH &&
        rc_state.rc_data.channel[REMOTE_CH_VRA] > REMOTE_VR_LOW_THRESHOLD;
    /* Mode 6 uses the latching SWD switch as permit/stop. The FS-i6S VRB
     * wheel springs back to centre; normal manual mode keeps its own VRB gate. */
    rc_centered = true;
    const unsigned channels[] = {REMOTE_CH_RIGHT_X, REMOTE_CH_RIGHT_Y, REMOTE_CH_LEFT_X};
    for(unsigned i = 0u; i < 3u; ++i)
        if((unsigned)abs((int)rc_state.rc_data.channel[channels[i]] - (int)REMOTE_CENTER) > REMOTE_DEADBAND)
            rc_centered = false;
    if(!rc_online) blockers |= BLOCK_RC_OFFLINE;
    else {
        if(!rc_switches) blockers |= BLOCK_RC_SWITCH;
        if(!rc_centered) blockers |= BLOCK_RC_STICK;
    }
    return rc_online && rc_switches && rc_centered;
}
static StraightInput input(uint32_t now) {
    StraightInput in = {.healthy = true, .stationary = true, .vx_mps = 0.0f};
    uint32_t stamp[8];
    blockers = 0u;
    const uint32_t key = __get_PRIMASK();
    __disable_irq();
    feedback_seen = seen;
    enabled_mask = 0u;
    for(unsigned i = 0u; i < 8u; ++i) stamp[i] = motor_ms[i];
    for(unsigned i = 0u; i < 4u; ++i)
        if(motor_state[i] == 1u) enabled_mask |= (uint8_t)(1u << i);
    __set_PRIMASK(key);
    /* Read time after copying stamps: an RX interrupt during SPI work may be
     * newer than the loop's original 'now', which must not underflow its age. */
    now = HAL_GetTick();
    for(unsigned i = 0u; i < 8u; ++i) motor_age[i] = now - stamp[i];
    for(unsigned i = 0u; i < 8u; ++i)
        if(!(feedback_seen & (1u << i)) || motor_age[i] > 100u)
            blockers |= i < 4u ? BLOCK_STEER_RX : BLOCK_DRIVE_RX;
    if(enabled_mask != 15u) blockers |= BLOCK_STEER_STATE;
    (void)remote_permit(); /* Evaluate all conditions, even when another one fails. */
    if(rx_fault) blockers |= BLOCK_UART;
    if(tx_fault) blockers |= BLOCK_TX;
    imu_age_us = imu_seen ? (uint32_t)(dwt_get_us() - imu_sample.gyro_timestamp_us) : UINT32_MAX;
    if(!imu_seen || imu_update_status != IMU_STATUS_OK) blockers |= BLOCK_IMU_SAMPLE;
    if(imu_age_us > 100000u) blockers |= BLOCK_IMU_STALE;
    imu_debug_valid = bmi088_get_attitude_debug(&imu_debug) == IMU_STATUS_OK;
    if(!imu_debug_valid || !imu_debug.calibrated || !imu_debug.has_angle) blockers |= BLOCK_IMU_CAL;
    for(uint16_t id = 1u; id <= 4u; ++id) {
        const float sign = (id == 2u || id == 3u) ? -1.0f : 1.0f;
        const float speed = drive_motor.get_spd(id);
        const float angle = steer_motor.get_pos(id) - zeros[id - 1u];
        if(!isfinite(speed) || !isfinite(angle)) blockers |= BLOCK_NONFINITE;
        in.stationary = in.stationary && fabsf(speed) < 0.05f;
        in.vx_mps += sign * speed * STRAIGHT_TEST_WHEEL_RADIUS_M * cosf(angle) * 0.25f;
    }
    const ImuAngle attitude = imu.get_angle();
    const ImuGyro gyro = imu_get_gyro_corrected();
    if(!isfinite(attitude.yaw) || !isfinite(gyro.z)) blockers |= BLOCK_NONFINITE;
    in.healthy = blockers == 0u;
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
static void update_drift(StraightInput in) {
    const uint32_t bad_data = BLOCK_STEER_RX | BLOCK_DRIVE_RX | BLOCK_IMU_SAMPLE |
        BLOCK_IMU_STALE | BLOCK_IMU_CAL | BLOCK_NONFINITE;
    /* Wheel feedback cannot detect someone rotating/lifting the entire robot.
     * This is an observation window, never an automatic bias adjustment. */
    if((blockers & bad_data) || !in.stationary ||
       (test.phase != ST_IDLE && test.phase != ST_DONE && test.phase != ST_FAULT)) {
        drift_tracking = drift_valid = false;
        return;
    }
    const uint32_t stamp = imu_sample.gyro_timestamp_us;
    const float yaw = imu.get_angle().yaw;
    if(!drift_tracking) {
        drift_tracking = true;
        drift_start_us = drift_last_us = stamp;
        drift_last_yaw = yaw;
        drift_yaw_sum = drift_gyro_sum = 0.0f;
        return;
    }
    if(stamp == drift_last_us) return;
    const uint32_t dt = stamp - drift_last_us;
    drift_yaw_sum += remainderf(yaw - drift_last_yaw, 6.283185307f);
    drift_gyro_sum += imu_get_gyro_corrected().z * (float)dt * 1.0e-6f;
    drift_last_us = stamp;
    drift_last_yaw = yaw;
    const uint32_t elapsed = stamp - drift_start_us;
    if(elapsed >= 5000000u) {
        const float factor = 57.295779513f * 1.0e6f / (float)elapsed;
        drift_yaw_rate = drift_yaw_sum * factor;
        drift_gyro_rate = drift_gyro_sum * factor;
        drift_window_ms = elapsed / 1000u;
        drift_valid = true;
        drift_start_us = stamp;
        drift_yaw_sum = drift_gyro_sum = 0.0f;
    }
}
static void log_detail(uint32_t now, StraightInput in) {
    /* One bounded line per 100 ms, rather than a burst that blocks the controller.
     * All formats fit the existing 256-byte logger at documented sensor ranges. */
    const unsigned slot = detail_slot++ % 15u;
    if(slot == 0u) {
        log_info("TEST_HEALTH t_ms=%lu blocked=0x%04lx first=%s seen=0x%02x steer_enabled=0x%x rc_ok=%u imu_ok=%u uart_fault=%u tx_fault=%u",
            (unsigned long)now, (unsigned long)blockers, first_blocker(), (unsigned)feedback_seen,
            (unsigned)enabled_mask, (unsigned)(rc_online && rc_switches && rc_centered),
            (unsigned)((blockers & (BLOCK_IMU_SAMPLE | BLOCK_IMU_STALE | BLOCK_IMU_CAL)) == 0u),
            (unsigned)rx_fault, (unsigned)tx_fault);
    } else if(slot == 1u) {
        const uint16_t* ch = rc_state.rc_data.channel;
        log_info("TEST_REMOTE t_ms=%lu online=%u switches_ok=%u centered=%u source=%u SWA=%u SWB=%u SWC=%u SWD=%u VRA=%u VRB=%u RX=%u RY=%u LX=%u vrb_required=0",
            (unsigned long)now, (unsigned)rc_online, (unsigned)rc_switches, (unsigned)rc_centered,
            (unsigned)rc_state.manual_source, ch[REMOTE_CH_SWA], ch[REMOTE_CH_SWB], ch[REMOTE_CH_SWC],
            ch[REMOTE_CH_SWD], ch[REMOTE_CH_VRA], ch[REMOTE_CH_VRB], ch[REMOTE_CH_RIGHT_X],
            ch[REMOTE_CH_RIGHT_Y], ch[REMOTE_CH_LEFT_X]);
    } else if(slot == 2u) {
        const ImuAngle a = imu.get_angle();
        log_info("TEST_IMU t_ms=%lu ready=%u cal=%u/%u age_us=%lu temp_C=%.2f ref_C=%.2f yaw_deg=%.3f pitch_deg=%.3f roll_deg=%.3f gz_rad_s=%.6f",
            (unsigned long)now, (unsigned)(imu_debug_valid && imu_debug.calibrated && imu_debug.has_angle),
            (unsigned)imu_debug.calib_count, (unsigned)imu_debug.calib_target, (unsigned long)imu_age_us,
            (double)imu_debug.temperature, (double)imu_debug.gyro_temp_ref,
            (double)a.yaw * 57.295779513, (double)a.pitch * 57.295779513, (double)a.roll * 57.295779513,
            (double)imu_debug.gyro_corrected.z);
    } else if(slot == 3u) {
        log_info("TEST_IMU_RAW t_ms=%lu unit=rad/s raw=%.6f,%.6f,%.6f startup_bias=%.6f,%.6f,%.6f corrected=%.6f,%.6f,%.6f",
            (unsigned long)now, (double)imu_sample.gyro.x, (double)imu_sample.gyro.y, (double)imu_sample.gyro.z,
            (double)imu_debug.gyro_bias.x, (double)imu_debug.gyro_bias.y, (double)imu_debug.gyro_bias.z,
            (double)imu_debug.gyro_corrected.x, (double)imu_debug.gyro_corrected.y, (double)imu_debug.gyro_corrected.z);
    } else if(slot == 4u) {
        log_info("TEST_IMU_BIAS t_ms=%lu applied_z_rad_s=%.6f temp_z_rad_s=%.6f slope_rad_s_C=%.7f intercept_rad_s=%.6f zru=%u active=%u policy=BOOT_MEAN",
            (unsigned long)now, (double)imu_debug.gyro_z_bias_effective, (double)imu_debug.gyro_temp_comp.z,
            (double)imu_debug.gyro_temp_coeff.z, (double)imu_debug.gyro_z_temp_intercept,
            (unsigned)imu_debug.zru_enabled, (unsigned)imu_debug.zru_active);
    } else if(slot <= 8u) {
        const uint16_t id = (uint16_t)(slot - 4u);
        static const char* const wheel[] = {"FL", "FR", "RR", "RL"};
        const unsigned i = id - 1u;
        log_info("TEST_MOTOR t_ms=%lu id=%u wheel=%s steer_seen=%u drive_seen=%u steer_age_ms=%lu drive_age_ms=%lu state=%u raw_rad=%.4f wheel_rad=%.4f drive_rad_s=%.4f current_raw=%.0f",
            (unsigned long)now, (unsigned)id, wheel[i], (unsigned)((feedback_seen >> i) & 1u),
            (unsigned)((feedback_seen >> (i + 4u)) & 1u), (unsigned long)motor_age[i],
            (unsigned long)motor_age[i + 4u], (unsigned)motor_state[i], (double)steer_motor.get_pos(id),
            (double)remainderf(steer_motor.get_pos(id) - zeros[i], 6.283185307f),
            (double)drive_motor.get_spd(id), (double)drive_motor.get_tor(id));
    } else if(slot <= 11u) {
        STM32HalCanDiagnostics d = {0};
        const unsigned bus = slot - 8u;
        FDCAN_HandleTypeDef* const handles[] = {&hfdcan1, &hfdcan2, &hfdcan3};
        const bool ok = can_get_diagnostics(handles[bus - 1u], &d);
        log_info("TEST_CAN t_ms=%lu bus=%u ok=%u rx=%lu tx=%lu done=%lu tx_fail=%lu rx_fail=%lu lost=%lu off=%u passive=%u proto_err=%lu hal=0x%lx",
            (unsigned long)now, bus, (unsigned)ok, (unsigned long)d.rx_count, (unsigned long)d.tx_submit_count,
            (unsigned long)d.tx_complete_count, (unsigned long)d.tx_failed_count, (unsigned long)d.rx_failed_count,
            (unsigned long)d.rx_fifo0_lost_count, (unsigned)d.bus_off, (unsigned)d.error_passive,
            (unsigned long)d.protocol_error_count, (unsigned long)d.hal_error_code);
    } else if(slot == 12u) {
        LaserSnapshot laser = {0};
        (void)laser_get_snapshot(&laser);
        log_info("TEST_LASER t_ms=%lu fresh=%u frames=%lu age_ms=%lu left_mm=%u right_mm=%u rear_mm=%u front_mm=%u control=OBSERVE_ONLY",
            (unsigned long)now, (unsigned)laser.frame_fresh, (unsigned long)laser.frame_count,
            (unsigned long)(now - laser.last_rx_ms), laser.distance_mm[0], laser.distance_mm[1],
            laser.distance_mm[2], laser.distance_mm[3]);
    } else if(slot == 13u) {
        log_info("TEST_TIMING t_ms=%lu control_dt_ms=%lu max_ms=%lu imu_dt_us=%lu imu_max_us=%lu imu_frames=%lu imu_errors=%lu update=%s HB_age_ms=%lu lease_ms=750",
            (unsigned long)now, (unsigned long)control_dt_ms, (unsigned long)control_dt_max_ms,
            (unsigned long)imu_dt_us, (unsigned long)imu_dt_max_us, (unsigned long)imu_samples,
            (unsigned long)imu_errors, imu.status_str(imu_update_status),
            (unsigned long)(test.phase == ST_RUNNING ? now - test.heartbeat_ms : 0u));
    } else {
        log_info("TEST_DRIFT t_ms=%lu valid=%u wheel_still=%u window_ms=%lu yaw_deg_s=%.6f gyro_deg_s=%.6f yaw_deg_min=%.3f body_static=USER_CONFIRM auto_zero=0",
            (unsigned long)now, (unsigned)drift_valid, (unsigned)in.stationary, (unsigned long)drift_window_ms,
            (double)drift_yaw_rate, (double)drift_gyro_rate, (double)drift_yaw_rate * 60.0);
    }
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
    log_info("TEST_READY mode=6 target=%.2fm speed<=0.10m/s radius=0.051 NOMINAL laser_limit=0; PING ARM GO <token> HB <token> STOP; no automatic restart",
        (double)STRAIGHT_TEST_TARGET_M);
    log_info("TEST_READY imu_bias=BOOT_MEAN keep_body_static_until_ready=1 calibration_samples=2000 detail_period_ms=1500 FW_period_ms=100");
    log_info("TEST_READY remote_policy=SWD_LATCH SWD_permit=2000 SWD_stop=1000 vrb_required=0");
}
void straight_test_app_loop(void) {
    const uint32_t now = HAL_GetTick();
    if(!initialized) return;
    can_poll_rx(CHASSIS_STEER_CAN);
    can_poll_rx(CHASSIS_DRIVE_CAN);
    can_poll_rx(&hfdcan3);
    if(now - last_imu >= 4u) {
        last_imu = now;
        update_imu_sample();
    }
    StraightInput in = input(now);
    if(!can_healthy()) blockers |= BLOCK_CAN;
    in.healthy = blockers == 0u;
    commands(now, in);
    update_drift(in);
    if(rx_fault) straight_fault(&test, "UART_ERROR_OR_OVERFLOW");
    if(test.phase == ST_RUNNING) {
        const ImuAngle a = imu.get_angle();
        const float error = remainderf(chassis_yaw_hold_get_yaw_ref() - a.yaw, 6.283185307f);
        if(!isfinite(error) || fabsf(error) > 0.15f) straight_fault(&test, "HEADING_DEVIATION");
    }
    if(now - last_step >= 10u) {
        control_dt_ms = now - last_step;
        if(control_dt_ms > control_dt_max_ms) control_dt_max_ms = control_dt_ms;
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
        last_log = now;
        const ImuAngle a = imu.get_angle();
        log_info("TEST_STATUS phase=%u token=%lu healthy=%u stopped=%u target_m=%.2f s=%.3f vx=%.3f cmd=%.3f yaw_deg=%.2f ref_deg=%.2f wz=%.4f blocked=%s result=%s",
            (unsigned)test.phase, (unsigned long)test.token, (unsigned)in.healthy, (unsigned)in.stationary,
            (double)STRAIGHT_TEST_TARGET_M,
            (double)test.distance_m, (double)in.vx_mps, (double)test.speed_mps, (double)a.yaw * 57.295779513,
            (double)chassis_yaw_hold_get_yaw_ref() * 57.295779513, (double)chassis_yaw_hold_get_output_wz(),
            first_blocker(), test.result);
        /* Dedicated numeric frame; human-readable status is not a FireWater channel. */
        const LogVofaValue wave[4] = {
            log_vofa_value_f64((double)a.yaw * 57.295779513),
            log_vofa_value_f64((double)chassis_yaw_hold_get_yaw_ref() * 57.295779513),
            log_vofa_value_f64((double)chassis_yaw_hold_get_yaw_error() * 57.295779513),
            log_vofa_value_f64((double)chassis_yaw_hold_get_output_wz()),
        };
        (void)log_vofa_write("FW", 4u, wave);
        log_detail(now, in);
    }
}
#endif
