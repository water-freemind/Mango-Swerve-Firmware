#include "chassis_profile.h"
#include "app_config.h"
#include "straight_test.h"
#include <stddef.h>

/* A nominal 102mm wheel is confirmed by the user, but its effective loaded
 * radius is not calibrated. Never infer axis spacing from the 270mm envelope. */
static const ChassisProfile legacy = {
    .name = "LEGACY_GEOMETRY",
    .model = {.length = 0.16f, .width = 0.16f,
        .wheel_radius = 0.0501f, .max_wheel_linear_speed = 2.0f},
    .wheel_drive_ratio = 1.0f,
    .span_source = CHASSIS_DIMENSION_LEGACY,
    .radius_source = CHASSIS_DIMENSION_LEGACY,
    .axis_coordinates_confirmed = false,
    .loaded_radius_calibrated = false
};
static const ChassisProfile combined = {
    .name = "NOMINAL102_LEGACY_SPAN",
    .model = {.length = 0.16f, .width = 0.16sf,
        .wheel_radius = STRAIGHT_TEST_WHEEL_RADIUS_M,
        .max_wheel_linear_speed = STRAIGHT_TEST_SPEED_CEILING_MPS},
    .wheel_drive_ratio = 1.0f,
    .span_source = CHASSIS_DIMENSION_LEGACY,
    .radius_source = CHASSIS_DIMENSION_USER_NOMINAL,
    .axis_coordinates_confirmed = false,
    .loaded_radius_calibrated = false
};
const ChassisProfile* chassis_profile_for_mode(unsigned run_mode) {
    switch(run_mode) {
        case APP_MODE_CAN_DIAGNOSTICS:
        case APP_MODE_MANUAL_DRIVE:
        case APP_MODE_STEERING_DIAGNOSTICS:
        case APP_MODE_CAN2_QUERY:
        case APP_MODE_STEERING_ZERO_TEST:
        case APP_MODE_MANUAL_STANDALONE: return &legacy;
        case APP_MODE_STRAIGHT_TEST: return &combined;
        default: return NULL;
    }
}
