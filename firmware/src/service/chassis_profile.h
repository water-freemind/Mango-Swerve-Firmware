#ifndef CHASSIS_PROFILE_H
#define CHASSIS_PROFILE_H
#include "steer_wheel_kine.h"
#include <stdbool.h>

/* Profiles identify the values actually used today, not a completed calibration.
 * Both profiles retain historical axis spacing until four steering-axis positions
 * are supplied. Motor reduction is applied by the DJI driver, not a second time
 * here. Steering zero offsets remain STEERING_FORWARD_ZERO_RAD. */
typedef enum {
    CHASSIS_DIMENSION_LEGACY = 0,
    CHASSIS_DIMENSION_USER_NOMINAL = 1
} ChassisDimensionSource;
typedef struct {
    const char* name;
    SteerWheelModel model;
    float wheel_drive_ratio;
    ChassisDimensionSource span_source;
    ChassisDimensionSource radius_source;
    bool axis_coordinates_confirmed;
    bool loaded_radius_calibrated;
} ChassisProfile;

/* Explicit mapping of application modes 0..6. NULL for an unsupported mode.
 * The name and provenance should be recorded alongside field measurements. */
const ChassisProfile* chassis_profile_for_mode(unsigned run_mode);
#endif
