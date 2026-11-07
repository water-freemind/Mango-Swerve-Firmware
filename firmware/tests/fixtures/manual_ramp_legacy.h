#ifndef MANGO_MANUAL_RAMP_H
#define MANGO_MANUAL_RAMP_H
#include "drive_source.h"

/* Unmeasured commissioning values, not a guarantee against tipping. */
#define MANUAL_RAMP_LINEAR_ACCEL 0.20f
#define MANUAL_RAMP_LINEAR_DECEL 0.30f
#define MANUAL_RAMP_ANGULAR_ACCEL 0.40f
#define MANUAL_RAMP_ANGULAR_DECEL 0.60f
#define MANUAL_RAMP_MAX_DT_S 0.050f
#define MANUAL_RAMP_PREPARE_DT_S 0.010f

typedef struct {
    DriveVelocity output, last_motion;
    bool reversing;
} ManualRamp;

void manual_ramp_reset(ManualRamp* ramp);
/* Reversal brakes the complete twist to zero, then waits for fresh stationary
 * wheel feedback. Emergency stops bypass this function and reset its state. */
bool manual_ramp_step(ManualRamp* ramp, DriveVelocity target, bool stationary,
                      bool drive_allowed, float dt_s);
#endif
