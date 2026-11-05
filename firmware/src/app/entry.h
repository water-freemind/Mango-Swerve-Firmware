#ifndef MANGO_SWERVE_APP_ENTRY_H
#define MANGO_SWERVE_APP_ENTRY_H

#include "app_config.h"
#include "assemble/assemble.h"

#if APP_RUN_MODE == APP_MODE_CAN_DIAGNOSTICS || APP_RUN_MODE == APP_MODE_STEERING_DIAGNOSTICS || APP_RUN_MODE == APP_MODE_CAN2_QUERY

#include "commissioning.h"

static inline void entry_init(void) {
    (void)assemble_delay();
    (void)assemble_log();
    commissioning_init();
#if APP_RUN_MODE == APP_MODE_STEERING_DIAGNOSTICS
    (void)assemble_steering_diagnostics();
#endif
}

static inline void entry_loop(void) {
    commissioning_process();
}

#else

#include "chassis.h"
#include "delay.h"
#include "odom.h"
#include "remote.h"
#include "stm32_hal_can.h"
#include "log.h"

#include <stdbool.h>
#include <stdint.h>

static bool app_initialized = false;
static uint8_t app_odom_tick = 0u;
static uint8_t app_remote_tick = 0u;

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

    remote_process();
    if(!remote_get_state(&state) || !remote_is_online(100u) ||
       !state.manual_request || state.manual_source != REMOTE_MANUAL_SOURCE_CHASSIS ||
       !chassis.is_ready()) {
        (void)chassis.brake();
        return;
    }

    if(state.rc_data.channel[REMOTE_CH_SWC] == REMOTE_SW_HIGH ||
       state.rc_data.channel[REMOTE_CH_VRA] <= REMOTE_VR_LOW_THRESHOLD) {
        (void)chassis.brake();
        return;
    }

    (void)chassis.set_steer_then_drive_enabled(
        state.rc_data.channel[REMOTE_CH_SWC] != REMOTE_SW_LOW);

    if(state.rc_data.channel[REMOTE_CH_VRB] > REMOTE_VR_LOW_THRESHOLD) {
        (void)chassis.set_velocity(0.0f, 0.0f, 0.0f);
        return;
    }

    if(state.rc_data.channel[REMOTE_CH_SWB] == REMOTE_SW_LOW) {
        max_linear = 2.0f;
        max_angular = 8.0f;
    } else if(state.rc_data.channel[REMOTE_CH_SWB] == REMOTE_SW_HIGH) {
        max_linear = 0.5f;
        max_angular = 2.0f;
    } else {
        max_linear = 1.0f;
        max_angular = 4.0f;
    }

    (void)chassis.set_velocity(
        app_channel_normalized(state.rc_data.channel[REMOTE_CH_RIGHT_Y]) * max_linear,
        -app_channel_normalized(state.rc_data.channel[REMOTE_CH_RIGHT_X]) * max_linear,
        -app_channel_normalized(state.rc_data.channel[REMOTE_CH_LEFT_X]) * max_angular);
}

static inline void entry_init(void) {
    if(assemble_delay() != SYSTEM_STATUS_OK || assemble_log() != SYSTEM_STATUS_OK) {
        return;
    }
    log_info("CHASSIS_BOOT log ready");
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
    if(assemble_laser() != SYSTEM_STATUS_OK || assemble_can3() != SYSTEM_STATUS_OK) {
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
        log_info("CHASSIS_CHECK boot=%u steer=%u drive=%u remote=%u odom=%u",
                 (unsigned)app_initialized, (unsigned)state->steer_motor_ready,
                 (unsigned)state->drive_motor_ready, (unsigned)remote_is_online(100u),
                 (unsigned)odom.is_ready());
    }
    if(app_initialized) {
        can_poll_rx(&hfdcan3);
    }
    if(!app_initialized || !tim6_500hz_flag) {
        return;
    }
    tim6_500hz_flag = false;

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
