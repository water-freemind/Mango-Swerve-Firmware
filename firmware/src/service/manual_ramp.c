#include "app_config.h"
#if APP_RUN_MODE == APP_MODE_STRAIGHT_TEST
#include "manual_ramp.h"
#include <math.h>
#include <stddef.h>

enum { PATH_IDLE, PATH_CANCEL_ACCEL, PATH_RAMP_UP, PATH_HOLD, PATH_RAMP_DOWN };
static bool finite_velocity(DriveVelocity v) {
    return isfinite(v.vx) && isfinite(v.vy) && isfinite(v.wz);
}
void manual_ramp_reset(ManualRamp* r) { if(r) *r = (ManualRamp){0}; }

static bool valid_path(const ManualRampPath* p) {
    return isfinite(p->x) && isfinite(p->y) && isfinite(p->ax) && isfinite(p->ay) &&
        isfinite(p->tx) && isfinite(p->ty) && isfinite(p->dx) && isfinite(p->dy) &&
        isfinite(p->remaining_s) && isfinite(p->ramp_s) && isfinite(p->hold_s) &&
        isfinite(p->peak_accel) && isfinite(p->origin_x) && isfinite(p->origin_y) &&
        isfinite(p->origin_ax) && isfinite(p->origin_ay) && isfinite(p->duration_s) && p->remaining_s >= 0 && p->phase <= PATH_RAMP_DOWN;
}
static void phase(ManualRampPath* p, unsigned next, float duration) {
    p->phase = next; p->remaining_s = p->duration_s = duration;
    p->origin_x = p->x; p->origin_y = p->y; p->origin_ax = p->ax; p->origin_ay = p->ay;
}
static void plan(ManualRampPath* p, float accel, float decel, float jerk) {
    const float dx = p->tx - p->x, dy = p->ty - p->y;
    const float distance = hypotf(dx, dy);
    if(distance == 0) { p->phase = PATH_IDLE; p->ax = p->ay = 0; return; }
    const float limit = hypotf(p->tx, p->ty) < hypotf(p->x, p->y) ? decel : accel;
    p->dx = dx / distance; p->dy = dy / distance;
    p->ramp_s = fminf(limit / jerk, sqrtf(distance / jerk));
    p->peak_accel = jerk * p->ramp_s;
    p->hold_s = fmaxf(0, distance / p->peak_accel - p->ramp_s);
    phase(p, PATH_RAMP_UP, p->ramp_s);
}
static void path_step(ManualRampPath* p, float tx, float ty, float accel, float decel, float jerk, float band, float dt) {
    /* Ignore only small nonzero endpoint noise; zero, initial motion and
     * reversal remain immediate requests. The stored endpoint is unchanged
     * inside the band, preventing perpetual cancel/replan on stick jitter. */
    bool retarget = hypotf(tx - p->tx, ty - p->ty) > band + 0.000001f ||
        ((tx == 0 && ty == 0) != (p->tx == 0 && p->ty == 0));
    if(retarget) {
        p->tx = tx; p->ty = ty;
        if(p->phase != PATH_CANCEL_ACCEL) {
            const float a = hypotf(p->ax, p->ay);
            if(a > 0) {
                p->dx = p->ax / a; p->dy = p->ay / a;
                phase(p, PATH_CANCEL_ACCEL, a / jerk);
            } else plan(p, accel, decel, jerk);
        }
    } else if(p->phase == PATH_IDLE && (p->x != p->tx || p->y != p->ty)) plan(p, accel, decel, jerk);
    /* At most cancel + up + hold + down; each transition consumes a phase,
     * including a zero-duration triangular-profile hold. */
    for(unsigned transitions = 0; dt > 0 && p->phase != PATH_IDLE && transitions < 6; ++transitions) {
        const float t = fminf(dt, p->remaining_s);
        const float j = p->phase == PATH_RAMP_UP ? jerk : p->phase == PATH_HOLD ? 0 : -jerk;
        dt -= t; p->remaining_s -= t;
        const float elapsed = p->duration_s - p->remaining_s;
        /* Evaluate from the phase origin rather than accumulating velocity
         * roundoff over thousands of samples before an endpoint snap. */
        p->x = p->origin_x + p->origin_ax * elapsed + 0.5f * j * p->dx * elapsed * elapsed;
        p->y = p->origin_y + p->origin_ay * elapsed + 0.5f * j * p->dy * elapsed * elapsed;
        p->ax = p->origin_ax + j * p->dx * elapsed; p->ay = p->origin_ay + j * p->dy * elapsed;
        if(p->remaining_s > 0) break;
        if(p->phase == PATH_CANCEL_ACCEL) {
            p->ax = p->ay = 0;
            plan(p, accel, decel, jerk);
        } else if(p->phase == PATH_RAMP_UP) {
            p->ax = p->peak_accel * p->dx; p->ay = p->peak_accel * p->dy;
            phase(p, PATH_HOLD, p->hold_s);
        } else if(p->phase == PATH_HOLD) {
            phase(p, PATH_RAMP_DOWN, p->ramp_s);
        } else {
            p->x = p->tx; p->y = p->ty; p->ax = p->ay = 0; p->phase = PATH_IDLE;
        }
    }
}

bool manual_ramp_step(ManualRamp* r, DriveVelocity target, bool stationary, bool drive_allowed, float dt_s) {
    if(!r) return false;
    if(!finite_velocity(target) || !finite_velocity(r->output) || !finite_velocity(r->last_motion) ||
       !finite_velocity(r->acceleration) || !valid_path(&r->linear) || !valid_path(&r->angular) ||
       !isfinite(dt_s) || dt_s < 0 || dt_s > MANUAL_RAMP_MAX_DT_S + 0.000001f ||
       hypotf(target.vx, target.vy) > APP_MANUAL_MAX_LINEAR_MPS + 0.00001f ||
       fabsf(target.wz) > APP_MANUAL_MAX_ANGULAR_RAD_S + 0.00001f) {
        manual_ramp_reset(r); return false;
    }
    if(dt_s == 0) return true;
    if(r->output.vx != 0 || r->output.vy != 0) {
        r->last_motion.vx = r->output.vx; r->last_motion.vy = r->output.vy;
    } else if(stationary) r->last_motion.vx = r->last_motion.vy = 0;
    if(r->output.wz != 0) r->last_motion.wz = r->output.wz;
    else if(stationary) r->last_motion.wz = 0;
    if(target.vx * r->last_motion.vx + target.vy * r->last_motion.vy < 0 ||
       target.wz * r->last_motion.wz < 0) r->reversing = true;
    if(r->reversing) target = (DriveVelocity){0};
    const float speed = fmaxf(hypotf(target.vx, target.vy), hypotf(r->output.vx, r->output.vy));
    if(speed > 0) {
        const float cap = fminf(APP_MANUAL_MAX_ANGULAR_RAD_S, MANUAL_RAMP_LATERAL_DEMAND / speed);
        target.wz = fmaxf(-cap, fminf(cap, target.wz));
    }
    /* When entering translation from an existing fast spin, also limit the
     * translation target using the current yaw rate. No output hard clipping:
     * both trajectories retain their acceleration/jerk continuity. */
    const float rotation = fmaxf(fabsf(target.wz), fabsf(r->output.wz));
    const float requested_speed = hypotf(target.vx, target.vy);
    if(rotation > 0 && requested_speed > 0) {
        const float scale = fminf(1, MANUAL_RAMP_LATERAL_DEMAND / (rotation * requested_speed));
        target.vx *= scale; target.vy *= scale;
    }
    r->limited_target = target;
    if(!drive_allowed) {
        /* Only an IK direction preview: chassis gate keeps actual drive zero.
         * Never integrate a trajectory behind the closed steering gate. */
        r->linear = r->angular = (ManualRampPath){0};
        r->acceleration = (DriveVelocity){0};
        const float preview_dt = fminf(dt_s, MANUAL_RAMP_PREPARE_DT_S);
        const float v = hypotf(target.vx, target.vy);
        const float scale = v > 0 ? fminf(1, MANUAL_RAMP_LINEAR_ACCEL * preview_dt / v) : 0;
        r->output = (DriveVelocity){target.vx * scale, target.vy * scale,
            fmaxf(-MANUAL_RAMP_ANGULAR_ACCEL * preview_dt, fminf(MANUAL_RAMP_ANGULAR_ACCEL * preview_dt, target.wz))};
        r->linear.x = r->output.vx; r->linear.y = r->output.vy; r->angular.x = r->output.wz;
    } else {
        path_step(&r->linear, target.vx, target.vy, MANUAL_RAMP_LINEAR_ACCEL, MANUAL_RAMP_LINEAR_DECEL, MANUAL_RAMP_LINEAR_JERK, MANUAL_RAMP_LINEAR_RETARGET_BAND, dt_s);
        path_step(&r->angular, target.wz, 0, MANUAL_RAMP_ANGULAR_ACCEL, MANUAL_RAMP_ANGULAR_DECEL, MANUAL_RAMP_ANGULAR_JERK, MANUAL_RAMP_ANGULAR_RETARGET_BAND, dt_s);
        r->output = (DriveVelocity){r->linear.x, r->linear.y, r->angular.x};
        r->acceleration = (DriveVelocity){r->linear.ax, r->linear.ay, r->angular.ax};
    }
    if(!finite_velocity(r->output) || !finite_velocity(r->acceleration)) { manual_ramp_reset(r); return false; }
    if(r->reversing && stationary && r->output.vx == 0 && r->output.vy == 0 && r->output.wz == 0)
        manual_ramp_reset(r);
    return true;
}
#endif
