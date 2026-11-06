#ifndef MANGO_SWERVE_APP_ENTRY_H
#define MANGO_SWERVE_APP_ENTRY_H

#include "app_config.h"
#include "assemble/assemble.h"

#if APP_RUN_MODE == APP_MODE_CAN_DIAGNOSTICS || APP_RUN_MODE == APP_MODE_STEERING_DIAGNOSTICS || APP_RUN_MODE == APP_MODE_CAN2_QUERY || APP_RUN_MODE == APP_MODE_STEERING_ZERO_TEST

#include "commissioning.h"
#if APP_RUN_MODE == APP_MODE_STEERING_ZERO_TEST
#include "steering_zero_test.h"
#endif

static inline void entry_init(void) {
    (void)assemble_delay();
    (void)assemble_log();
    commissioning_init();
#if APP_RUN_MODE == APP_MODE_STEERING_DIAGNOSTICS
    (void)assemble_steering_diagnostics();
#elif APP_RUN_MODE == APP_MODE_STEERING_ZERO_TEST
    steering_zero_test_init();
#endif
}

static inline void entry_loop(void) {
    commissioning_process();
#if APP_RUN_MODE == APP_MODE_STEERING_ZERO_TEST
    steering_zero_test_process();
#endif
}

#else

#include "chassis.h"
#include "chassis_yaw_hold.h"
#include "delay.h"
#include "odom.h"
#include "remote.h"
#include "stm32_hal_can.h"
#include "log.h"
#include "work_link.h"
#include "work_config.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

static bool app_initialized = false;
static uint8_t app_odom_tick = 0u;
static uint8_t app_remote_tick = 0u;
static bool app_remote_armed = false;

static float app_channel_normalized(uint16_t value) {
    int32_t delta = (int32_t)value - (int32_t)REMOTE_CENTER;
    if(delta >= -(int32_t)REMOTE_DEADBAND && delta <= (int32_t)REMOTE_DEADBAND) {
        return 0.0f;
    }
    if(delta > 500) delta = 500;
    if(delta < -500) delta = -500;
    return (float)delta / REMOTE_SPAN;
}

static void app_apply_remote(void) {
    RemoteState state;
    float max_linear;
    float max_angular;
    float vx;
    float vy;
    float speed;
    float wz;

    remote_process();
    if(!remote_get_state(&state) || !remote_is_online(100u) ||
       !state.manual_request || state.manual_source != REMOTE_MANUAL_SOURCE_CHASSIS ||
       !chassis.is_ready()) {
        app_remote_armed = false;
        chassis_yaw_hold_disable();
        (void)chassis.brake();
        return;
    }

    if(state.rc_data.channel[REMOTE_CH_SWC] == REMOTE_SW_HIGH ||
       state.rc_data.channel[REMOTE_CH_VRA] <= REMOTE_VR_LOW_THRESHOLD) {
        app_remote_armed = false;
        chassis_yaw_hold_disable();
        (void)chassis.brake();
        return;
    }

    (void)chassis.set_steer_then_drive_enabled(
        state.rc_data.channel[REMOTE_CH_SWC] != REMOTE_SW_LOW);

    if(state.rc_data.channel[REMOTE_CH_VRB] > REMOTE_VR_LOW_THRESHOLD) {
        app_remote_armed = true;
        chassis_yaw_hold_disable();
        (void)chassis.set_velocity(0.0f, 0.0f, 0.0f);
        return;
    }
    if(!app_remote_armed) {
        chassis_yaw_hold_disable();
        (void)chassis.stop();
        return;
    }

    if(state.rc_data.channel[REMOTE_CH_SWB] >= REMOTE_SW_LOW - REMOTE_SWITCH_TOLERANCE) {
        max_linear = 1.5f;
        max_angular = 6.0f;
    } else if(state.rc_data.channel[REMOTE_CH_SWB] <= REMOTE_SW_HIGH + REMOTE_SWITCH_TOLERANCE) {
        max_linear = 0.4f;
        max_angular = 1.5f;
    } else {
        max_linear = 0.8f;
        max_angular = 3.0f;
    }

    if(max_linear > APP_MANUAL_MAX_LINEAR_MPS) {
        max_linear = APP_MANUAL_MAX_LINEAR_MPS;
    }
    if(max_angular > APP_MANUAL_MAX_ANGULAR_RAD_S) {
        max_angular = APP_MANUAL_MAX_ANGULAR_RAD_S;
    }

    vx = app_channel_normalized(state.rc_data.channel[REMOTE_CH_RIGHT_Y]) * max_linear;
    vy = -app_channel_normalized(state.rc_data.channel[REMOTE_CH_RIGHT_X]) * max_linear;
    speed = sqrtf(vx * vx + vy * vy);
    if(speed > max_linear) {
        vx *= max_linear / speed;
        vy *= max_linear / speed;
    }
    wz = -app_channel_normalized(state.rc_data.channel[REMOTE_CH_LEFT_X]) * max_angular;
    const Odom* attitude = odom.get_state();
    if(wz != 0.0f || (fabsf(vx) <= 0.01f && fabsf(vy) <= 0.01f) ||
       !odom.is_ready() || !isfinite(attitude->angle.z) ||
       !isfinite(attitude->gyro_corrected.z) || work_link_hold()) {
        chassis_yaw_hold_disable();
    } else {
        if(!chassis_yaw_hold_is_active()) {
            chassis_yaw_hold_set_target(attitude->angle.z);
        }
        wz = chassis_yaw_hold_apply(vx, vy, 0.0f, attitude->angle.z,
                                    attitude->gyro_corrected.z, 0.01f);
    }
    (void)chassis.set_velocity(vx, vy, wz);
}

static inline void entry_init(void) {
    if(assemble_delay() != SYSTEM_STATUS_OK || assemble_log() != SYSTEM_STATUS_OK) {
        return;
    }
    log_info("CHASSIS_BOOT log ready");
    log_info("CHASSIS_BOOT mode=%u work_enabled=%u limits linear=%f angular=%f",
             (unsigned)APP_RUN_MODE, (unsigned)WORK_LINK_ENABLED,
             (double)APP_MANUAL_MAX_LINEAR_MPS, (double)APP_MANUAL_MAX_ANGULAR_RAD_S);
    if(assemble_rgb() != SYSTEM_STATUS_OK) {
        log_error("CHASSIS_BOOT rgb failed");
        return;
    }
    if(assemble_imu() != SYSTEM_STATUS_OK) {
        log_error("CHASSIS_BOOT imu failed");
        return;
    }
    log_info("CHASSIS_BOOT imu ready");
    if(assemble_odom() != SYSTEM_STATUS_OK) {
        log_error("CHASSIS_BOOT odom failed");
        return;
    }
    const ChassisYawHoldConfig yaw_hold_config = {
        .enabled = true,
        .kp = APP_YAW_HOLD_KP,
        .kd = APP_YAW_HOLD_KD,
        .v_deadband = 0.01f,
        .wz_deadband = 0.0f,
        .wz_limit = APP_YAW_HOLD_MAX_WZ,
    };
    chassis_yaw_hold_init(&yaw_hold_config);
    if(assemble_laser() != SYSTEM_STATUS_OK || !work_link_init() ||
       assemble_can3() != SYSTEM_STATUS_OK) {
        log_error("CHASSIS_BOOT range sensor bus failed");
        return;
    }
    if(assemble_chassis() != SYSTEM_STATUS_OK) {
        log_error("CHASSIS_BOOT motor assembly failed");
        return;
    }
    if(assemble_remote() != SYSTEM_STATUS_OK) {
        log_error("CHASSIS_BOOT remote receiver failed");
        return;
    }
    remote_init();
    (void)chassis.brake();
    if(assemble_tim6_500hz() != SYSTEM_STATUS_OK) {
        log_error("CHASSIS_BOOT 500 Hz timer failed");
        return;
    }
    app_initialized = true;
    log_info("CHASSIS_BOOT initialized; waiting for motor feedback and manual remote request");
}

static inline void entry_loop(void) {
    static uint32_t last_status_ms;
    const uint32_t now_ms = HAL_GetTick();
    if((uint32_t)(now_ms - last_status_ms) >= 1000u) {
        const Chassis* state = chassis.get_chassis();
        last_status_ms = now_ms;
        log_info("CHASSIS_CHECK boot=%u steer=%u drive=%u remote=%u odom=%u armed=%u cmd=%f,%f,%f",
                 (unsigned)app_initialized, (unsigned)state->steer_motor_ready,
                 (unsigned)state->drive_motor_ready, (unsigned)remote_is_online(100u),
                 (unsigned)odom.is_ready(), (unsigned)app_remote_armed, (double)state->kine.control.vx,
                 (double)state->kine.control.vy, (double)state->kine.control.wz);
        log_info("WHEEL_CHECK angle=%f,%f,%f,%f omega=%f,%f,%f,%f",
                 (double)state->kine.state.cur_wheels[0].steer_angle,
                 (double)state->kine.state.cur_wheels[1].steer_angle,
                 (double)state->kine.state.cur_wheels[2].steer_angle,
                 (double)state->kine.state.cur_wheels[3].steer_angle,
                 (double)state->kine.state.cur_wheels[0].wheel_omega,
                 (double)state->kine.state.cur_wheels[1].wheel_omega,
                 (double)state->kine.state.cur_wheels[2].wheel_omega,
                 (double)state->kine.state.cur_wheels[3].wheel_omega);
        if(app_initialized) {
            const Odom* attitude = odom.get_state();
            log_info("YAW_CHECK ready=%u yaw=%f gyro_z=%f wheel_wz=%f bias_enabled=%u hold=%u ref=%f error=%f correction=%f",
                     (unsigned)odom.is_ready(), (double)attitude->angle.z,
                     (double)attitude->gyro_corrected.z, (double)state->kine.state.cur_wz,
                     (unsigned)state->config.yaw_bias.enabled,
                     (unsigned)chassis_yaw_hold_is_active(), (double)chassis_yaw_hold_get_yaw_ref(),
                     (double)chassis_yaw_hold_get_yaw_error(), (double)chassis_yaw_hold_get_output_wz());
            for(uint16_t id = 1u; id <= 4u; ++id) {
                BusMotorFeedback steer_feedback = {0};
                BusMotorFeedback drive_feedback = {0};
                const BusMotorStatus steer_status = steer_motor.update_feedback(id, &steer_feedback);
                const BusMotorStatus drive_status = drive_motor.update_feedback(id, &drive_feedback);
                log_info("MOTOR_CHECK id=%u steer_status=%u state=%u pos=%f speed=%f drive_status=%u speed=%f current=%f",
                         (unsigned)id, (unsigned)steer_status, (unsigned)steer_feedback.error_code,
                         (double)steer_feedback.position, (double)steer_feedback.speed,
                         (unsigned)drive_status, (double)drive_feedback.speed, (double)drive_feedback.torque);
            }
        }
        log_info("WORK_CHECK enabled=%u paired=%u active=%u hold=%u state=%u result=%u rx=%lu tx=%lu rejected=%lu",
                 (unsigned)WORK_LINK_ENABLED, (unsigned)g_work_link.paired, (unsigned)g_work_link.active,
                 (unsigned)work_link_hold(), (unsigned)g_work_link.reply.state,
                 (unsigned)g_work_link.reply.result, (unsigned long)g_work_link.rx_count,
                 (unsigned long)g_work_link.tx_count, (unsigned long)g_work_link.rejected_count);
        const FsIa10bData* rc = remote_get_raw_data();
        if(rc != NULL) {
            log_info("REMOTE_CHECK SWA=%u SWB=%u SWC=%u SWD=%u VRA=%u VRB=%u X=%u Y=%u W=%u",
                     (unsigned)rc->channel[REMOTE_CH_SWA], (unsigned)rc->channel[REMOTE_CH_SWB],
                     (unsigned)rc->channel[REMOTE_CH_SWC], (unsigned)rc->channel[REMOTE_CH_SWD],
                     (unsigned)rc->channel[REMOTE_CH_VRA], (unsigned)rc->channel[REMOTE_CH_VRB],
                     (unsigned)rc->channel[REMOTE_CH_RIGHT_X], (unsigned)rc->channel[REMOTE_CH_RIGHT_Y],
                     (unsigned)rc->channel[REMOTE_CH_LEFT_X]);
        }
    }
    if(app_initialized) {
        can_poll_rx(&hfdcan3);
    }
    if(!app_initialized || !tim6_500hz_flag) {
        return;
    }
    tim6_500hz_flag = false;

    work_link_process(now_ms, chassis.is_ready());

    if(++app_remote_tick >= 5u) {
        app_remote_tick = 0u;
        app_apply_remote();
    }
    (void)chassis.process();
    if(++app_odom_tick >= 2u) {
        app_odom_tick = 0u;
        (void)odom.process();
    }
}

#endif /* APP_RUN_MODE */

#endif
