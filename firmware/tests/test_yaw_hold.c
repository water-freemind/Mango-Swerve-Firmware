#include "chassis_yaw_hold.h"
#include "app_config.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void) {
    const ChassisYawHoldConfig config = {
        .enabled = true, .kp = APP_YAW_HOLD_KP, .kd = APP_YAW_HOLD_KD,
        .v_deadband = 0.01f, .wz_deadband = 0.0f, .wz_limit = APP_YAW_HOLD_MAX_WZ,
    };
    float output;
    chassis_yaw_hold_init(&config);
    assert(!chassis_yaw_hold_is_active());
    chassis_yaw_hold_set_target(0.0f);
    output = chassis_yaw_hold_apply(0.1f, 0.0f, 0.0f, 0.01f, 0.0f, 0.01f);
    assert(output < 0.0f);
    assert(fabsf(output + APP_YAW_HOLD_KP * 0.01f) < 0.00001f);
    output = chassis_yaw_hold_apply(0.1f, 0.0f, 0.0f, -0.01f, 0.0f, 0.01f);
    assert(output > 0.0f);
    output = chassis_yaw_hold_apply(0.1f, 0.0f, 0.0f, 0.5f, 0.2f, 0.01f);
    assert(fabsf(output + APP_YAW_HOLD_MAX_WZ) < 0.00001f);
    assert(fabsf(chassis_yaw_hold_apply(0.1f, 0.0f, 0.001f, 0.5f, 0.2f, 0.01f) - 0.001f) < 0.000001f);
    assert(chassis_yaw_hold_apply(0.0f, 0.0f, 0.0f, 0.5f, 0.2f, 0.01f) == 0.0f);
    assert(chassis_yaw_hold_apply(0.1f, 0.0f, 0.0f, NAN, 0.0f, 0.01f) == 0.0f);
    chassis_yaw_hold_disable();
    assert(!chassis_yaw_hold_is_active());
    assert(chassis_yaw_hold_apply(0.1f, 0.0f, 0.0f, 0.5f, 0.2f, 0.01f) == 0.0f);
    chassis_yaw_hold_set_target(3.13f);
    output = chassis_yaw_hold_apply(0.1f, 0.0f, 0.0f, -3.13f, 0.0f, 0.01f);
    assert(output < 0.0f && fabsf(output) < APP_YAW_HOLD_MAX_WZ);
    puts("YAW_HOLD_TEST_OK");
    return 0;
}
