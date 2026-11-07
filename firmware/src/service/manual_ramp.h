#ifndef MANGO_MANUAL_RAMP_H
#define MANGO_MANUAL_RAMP_H
#include "drive_source.h"

/* Unmeasured commissioning candidates, not a guarantee against tipping. */
#define MANUAL_RAMP_LINEAR_ACCEL 0.20f
#define MANUAL_RAMP_LINEAR_DECEL 0.30f
#define MANUAL_RAMP_ANGULAR_ACCEL 0.40f
#define MANUAL_RAMP_ANGULAR_DECEL 0.60f
#define MANUAL_RAMP_LINEAR_JERK 0.80f /* m/s^3 */
#define MANUAL_RAMP_ANGULAR_JERK 1.60f /* rad/s^3 */
#define MANUAL_RAMP_LATERAL_DEMAND 0.30f /* |v| * |wz|, m/s^2: target budget only */
#define MANUAL_RAMP_LINEAR_RETARGET_BAND 0.006f /* two fast-gear stick counts, m/s */
#define MANUAL_RAMP_ANGULAR_RETARGET_BAND 0.024f /* two fast-gear stick counts, rad/s */
#define MANUAL_RAMP_MAX_DT_S 0.050f
#define MANUAL_RAMP_PREPARE_DT_S 0.010f

/* Private trajectory state exposed for static allocation, not caller mutation.
 * Velocity follows a straight vector segment; acceleration ramps up, holds,
 * and ramps down. A changed target first cancels existing acceleration. */
typedef struct {
    float x, y, ax, ay, tx, ty, dx, dy;
    float remaining_s, ramp_s, hold_s, peak_accel;
    float origin_x, origin_y, origin_ax, origin_ay, duration_s;
    unsigned phase;
} ManualRampPath;
typedef struct {
    DriveVelocity output, last_motion;
    bool reversing;
    DriveVelocity acceleration, limited_target;
    ManualRampPath linear, angular;
} ManualRamp;

void manual_ramp_reset(ManualRamp* ramp);
/* Reversal brakes the complete twist to zero, then waits for fresh stationary
 * wheel feedback. Emergency stops bypass this function and reset its state.
 * Angular target is coupled to max(current, requested) translation; linear
 * target is also limited by max(current, limited-requested) yaw. A sudden
 * retarget cannot remove existing velocity instantly, so transient |v*wz|
 * may exceed the target budget. No hardware stability claim is implied. */
bool manual_ramp_step(ManualRamp* ramp, DriveVelocity target, bool stationary,
                      bool drive_allowed, float dt_s);
#endif
