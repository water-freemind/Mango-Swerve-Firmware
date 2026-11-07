/* Frozen comparison fixture from 1a4345b, not linked into firmware. */
#include "app_config.h"
#if APP_RUN_MODE == APP_MODE_STRAIGHT_TEST
#include "manual_ramp_legacy.h"
#include <math.h>

void manual_ramp_reset(ManualRamp* r) { *r = (ManualRamp){0}; }

static bool finite_velocity(DriveVelocity v) {
    return isfinite(v.vx) && isfinite(v.vy) && isfinite(v.wz);
}

bool manual_ramp_step(ManualRamp* r, DriveVelocity target, bool stationary, bool drive_allowed, float dt_s) {
    if(!finite_velocity(target) || !finite_velocity(r->output) || !finite_velocity(r->last_motion) ||
       !isfinite(dt_s) || dt_s < 0.0f || dt_s > MANUAL_RAMP_MAX_DT_S + 0.000001f) {
        manual_ramp_reset(r);
        return false;
    }
    if(dt_s == 0.0f) return true;
    /* Retain the previous direction after a zero command until feedback says
     * stopped, so release-then-reverse cannot skip the reversal interlock. */
    if(r->output.vx != 0.0f || r->output.vy != 0.0f) {
        r->last_motion.vx = r->output.vx;
        r->last_motion.vy = r->output.vy;
    } else if(stationary) r->last_motion.vx = r->last_motion.vy = 0.0f;
    if(r->output.wz != 0.0f) r->last_motion.wz = r->output.wz;
    else if(stationary) r->last_motion.wz = 0.0f;
    if(target.vx * r->last_motion.vx + target.vy * r->last_motion.vy < 0.0f ||
       target.wz * r->last_motion.wz < 0.0f) r->reversing = true;
    if(r->reversing) target = (DriveVelocity){0};
    if(!drive_allowed) {
        /* Closed steering gate: never accumulate a larger command behind it.
         * A <=10 ms preparation step gives IK a direction, capped at 0.002 m/s
         * and 0.004 rad/s. Keep reversal history until measured wheel stop. */
        r->output = (DriveVelocity){0};
        dt_s = fminf(dt_s, MANUAL_RAMP_PREPARE_DT_S);
    }

    const float current_speed = hypotf(r->output.vx, r->output.vy);
    const float target_speed = hypotf(target.vx, target.vy);
    const float dx = target.vx - r->output.vx, dy = target.vy - r->output.vy;
    const float delta = hypotf(dx, dy);
    if(!isfinite(current_speed) || !isfinite(target_speed) || !isfinite(delta)) {
        manual_ramp_reset(r);
        return false;
    }
    const float limit = (target_speed < current_speed ? MANUAL_RAMP_LINEAR_DECEL : MANUAL_RAMP_LINEAR_ACCEL) * dt_s;
    if(delta <= limit) {
        r->output.vx = target.vx;
        r->output.vy = target.vy;
    } else {
        r->output.vx += dx * (limit / delta);
        r->output.vy += dy * (limit / delta);
    }
    const float angular_limit = (fabsf(target.wz) < fabsf(r->output.wz) ?
        MANUAL_RAMP_ANGULAR_DECEL : MANUAL_RAMP_ANGULAR_ACCEL) * dt_s;
    const float dw = target.wz - r->output.wz;
    r->output.wz += fmaxf(-angular_limit, fminf(angular_limit, dw));
    if(fabsf(dw) <= angular_limit) r->output.wz = target.wz;
    if(r->reversing && stationary && r->output.vx == 0.0f && r->output.vy == 0.0f && r->output.wz == 0.0f)
        manual_ramp_reset(r); /* Leave a zero-output cycle before a new direction. */
    return true;
}
#endif
