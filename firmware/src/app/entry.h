#ifndef MANGO_SWERVE_APP_ENTRY_H
#define MANGO_SWERVE_APP_ENTRY_H

#include "app_config.h"
#include "assemble/assemble.h"

#if APP_RUN_MODE == APP_MODE_CAN_DIAGNOSTICS

#include "commissioning.h"

static inline void entry_init(void) {
    (void)assemble_delay();
    (void)assemble_log();
    commissioning_init();
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
#include "thinning.h"

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
    ThinningSnapshot thinning_state;
    float max_linear;
    float max_angular;

    if(thinning_get_snapshot(&thinning_state) && thinning_state.chassis_brake_required) {
        (void)chassis.brake();
        return;
    }

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
    if(assemble_delay() != SYSTEM_STATUS_OK ||
       assemble_log() != SYSTEM_STATUS_OK ||
       assemble_rgb() != SYSTEM_STATUS_OK ||
       assemble_imu() != SYSTEM_STATUS_OK ||
       assemble_odom() != SYSTEM_STATUS_OK ||
       assemble_laser() != SYSTEM_STATUS_OK ||
       assemble_can3() != SYSTEM_STATUS_OK ||
       assemble_chassis() != SYSTEM_STATUS_OK ||
       assemble_remote() != SYSTEM_STATUS_OK) {
        return;
    }
    remote_init();
    thinning_init();
    (void)chassis.brake();
    if(assemble_tim6_500hz() != SYSTEM_STATUS_OK) {
        return;
    }
    app_initialized = true;
}

static inline void entry_loop(void) {
    if(app_initialized) {
        can_poll_rx(&hfdcan3);
    }
    if(!app_initialized || !tim6_500hz_flag) {
        return;
    }
    tim6_500hz_flag = false;
    thinning_process(HAL_GetTick());

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
