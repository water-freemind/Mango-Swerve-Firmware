#include "straight_test.h"
#include "laser.h"
#include "laser_prediction.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

const char* straight_plan_name(StraightPlan plan) {
    static const char* const names[] = {"FWD", "ROUND", "WALL", "APPROACH"};
    return (unsigned)plan < 4u ? names[plan] : "INVALID";
}
const char* straight_leg_name(StraightLeg leg) {
    static const char* const names[] = {"OUT", "WAIT", "BACK"};
    return (unsigned)leg < 3u ? names[leg] : "INVALID";
}
bool straight_laser_ready(StraightPlan plan, StraightInput in) {
    /* These are test operating windows, not sensor range specifications. */
    if(plan == ST_PLAN_FWD) return true;
    if(in.laser_predicted_mask) return false; /* Never ARM/GO on predicted data. */
    if(!in.laser_fresh || in.laser_stale_mask || in.left_mm < 70u || in.left_mm > LASER_ACCEPT_MAX_MM ||
       in.right_mm < 70u || in.right_mm > LASER_ACCEPT_MAX_MM || in.rear_mm < 70u ||
       in.rear_mm > LASER_ACCEPT_MAX_MM || in.front_mm < 70u || in.front_mm > LASER_ACCEPT_MAX_MM) return false;
    if(plan == ST_PLAN_WALL) return in.right_mm >= 150u;
    if(plan == ST_PLAN_APPROACH) return in.front_mm >= 70u;
    return true;
}
bool straight_motion_laser_ready(const StraightTest* t, StraightInput in) {
    if((t->phase != ST_RUNNING && t->phase != ST_STOPPING) || t->plan != ST_PLAN_WALL)
        return straight_laser_ready(t->plan, in);
    /* Keep near/invalid returns protective in every direction. Far returns
     * behind the travel direction are unnecessary for current motion. Rear
     * range must be valid throughout WAIT before reverse can start. */
    if(!in.laser_fresh || in.left_mm < 70u || in.right_mm < 150u || in.right_mm > LASER_ACCEPT_MAX_MM ||
       in.rear_mm < 70u || in.front_mm < 70u || in.left_mm == UINT16_MAX ||
       in.rear_mm == UINT16_MAX || in.front_mm == UINT16_MAX) return false;
    const bool front_required = t->leg == ST_LEG_OUT || in.vx_mps > 0.01f || t->speed_mps > 0.0f;
    const bool rear_required = t->leg != ST_LEG_OUT || in.vx_mps < -0.01f || t->speed_mps < 0.0f;
    const bool left_required = in.vy_mps > 0.01f || t->lateral_mps > 0.0f;
    const uint8_t required = (uint8_t)(2u | (front_required ? 8u : 0u) |
        (rear_required ? 4u : 0u) | (left_required ? 1u : 0u));
    return !(in.laser_stale_mask & required) && (!front_required || in.front_mm <= LASER_ACCEPT_MAX_MM) &&
        (!rear_required || in.rear_mm <= LASER_ACCEPT_MAX_MM) && (!left_required || in.left_mm <= LASER_ACCEPT_MAX_MM);
}
static bool round_trip(const StraightTest* t) {
    return t->plan == ST_PLAN_ROUND || t->plan == ST_PLAN_WALL;
}
uint16_t straight_front_target_mm(const StraightTest* t) {
    return t->body_edge_mm ? (uint16_t)(t->body_edge_mm + STRAIGHT_TEST_FRONT_GAP_MM) : 0u;
}
void straight_guard_limits(const StraightTest* t, StraightInput in, uint16_t limits[4]) {
    const float closing[] = {
        fmaxf(0.0f, fmaxf(in.vy_mps, t->lateral_mps)),
        fmaxf(0.0f, fmaxf(-in.vy_mps, -t->lateral_mps)),
        fmaxf(0.0f, fmaxf(-in.vx_mps, -t->speed_mps)),
        fmaxf(0.0f, fmaxf(in.vx_mps, t->speed_mps))};
    for(unsigned i = 0; i < 4u; ++i) {
        const float v = closing[i];
        const float extra = (v * STRAIGHT_TEST_SENSOR_DELAY_S + v * v / (2.0f * STRAIGHT_TEST_BRAKE_DECEL_MPS2)) * 1000.0f;
        limits[i] = (uint16_t)ceilf(fminf(65535.0f, t->body_edge_mm + STRAIGHT_TEST_COLLISION_MARGIN_MM + extra));
    }
}
static uint8_t collision_mask(const StraightTest* t, StraightInput in) {
    uint16_t limits[4];
    const uint16_t ranges[] = {in.left_mm, in.right_mm, in.rear_mm, in.front_mm};
    uint8_t mask = 0u;
    if(t->plan == ST_PLAN_FWD) return 0u;
    straight_guard_limits(t, in, limits);
    for(unsigned i = 0; i < 4u; ++i)
        if(ranges[i] <= limits[i]) mask |= (uint8_t)(1u << i);
    return mask;
}
static bool arm_selection(const char* line, StraightPlan* plan, float* target) {
    *target = STRAIGHT_TEST_TARGET_M;
    if(!strcmp(line, "ARM")) { *plan = ST_PLAN_FWD; return true; }
    if(!strcmp(line, "ARM APPROACH")) { *plan = ST_PLAN_APPROACH; return true; }
    const char* suffix;
    if(!strncmp(line, "ARM ROUND", 9u)) { *plan = ST_PLAN_ROUND; suffix = line + 9u; }
    else if(!strncmp(line, "ARM WALL", 8u)) { *plan = ST_PLAN_WALL; suffix = line + 8u; }
    else return false;
    if(!strcmp(suffix, "") || !strcmp(suffix, " 1")) return true;
    if(!strcmp(suffix, " 2")) { *target = 2.0f; return true; }
    if(!strcmp(suffix, " 4")) { *target = 4.0f; return true; }
    return false;
}
static void stopping(StraightTest* t, uint32_t now, const char* reason) {
    t->laser_paused = false;
    t->phase = ST_STOPPING;
    t->speed_mps = t->lateral_mps = 0.0f;
    t->stopped_timing = false;
    t->guard_mask = 0u;
    t->guard_clear = false;
    t->turn_ms = now;
    t->result = reason;
}
void straight_init(StraightTest* t, uint32_t now) {
    memset(t, 0, sizeof(*t));
    t->target_m = STRAIGHT_TEST_TARGET_M;
    t->max_speed_mps = STRAIGHT_TEST_DEFAULT_SPEED_MPS;
    t->accel_mps2 = STRAIGHT_TEST_ACCEL_MPS2;
    t->body_edge_mm = STRAIGHT_TEST_BODY_EDGE_MM;
    t->last_ms = now;
    t->result = "IDLE";
}
void straight_fault(StraightTest* t, const char* reason) {
    t->laser_paused = false;
    t->phase = ST_FAULT;
    t->speed_mps = t->lateral_mps = 0.0f;
    t->result = reason;
    t->guard_mask = 0u;
    t->guard_clear = false;
}
static bool token_command(const char* line, const char* prefix, uint32_t token) {
    char* end;
    const size_t n = strlen(prefix);
    unsigned long value;
    if(strncmp(line, prefix, n) || line[n] < '0' || line[n] > '9') return false;
    value = strtoul(line + n, &end, 10);
    return *end == '\0' && value == token;
}
bool straight_command(StraightTest* t, const char* line, uint32_t now, StraightInput in) {
    if(!strcmp(line, "PING")) return true;
    if(!strncmp(line, "ACCEL ", 6u)) {
        char* end;
        const float accel = strtof(line + 6u, &end);
        if(t->phase == ST_ARMED || t->phase == ST_RUNNING || t->phase == ST_STOPPING || !in.stationary ||
           end == line + 6u || *end != '\0' || !isfinite(accel) || accel < 0.05f || accel > 0.50f) return false;
        t->accel_mps2 = accel;
        t->result = "ACCEL_SET";
        return true;
    }
    if(!strcmp(line, "SPEED 0.05") || !strcmp(line, "SPEED 0.08") || !strcmp(line, "SPEED 0.10") ||
       !strcmp(line, "SPEED 0.20") || !strcmp(line, "SPEED 0.30") || !strcmp(line, "SPEED 0.50") ||
       !strcmp(line, "SPEED 0.40") || !strcmp(line, "SPEED 0.80") || !strcmp(line, "SPEED 1.00") ||
       !strcmp(line, "SPEED 1.20") || !strcmp(line, "SPEED 1.50") ||
       !strcmp(line, "SPEED 1.80") || !strcmp(line, "SPEED 2.00")) {
        if(t->phase == ST_ARMED || t->phase == ST_RUNNING || t->phase == ST_STOPPING || !in.stationary) return false;
        t->max_speed_mps = strtof(line + 6u, NULL);
        t->result = "SPEED_SET"; return true;
    }
    if(!strncmp(line, "EDGE ALL ", 9u)) {
        char* end;
        if(t->phase == ST_ARMED || t->phase == ST_RUNNING || t->phase == ST_STOPPING || !in.stationary ||
           line[9] < '0' || line[9] > '9') return false;
        const unsigned long mm = strtoul(line + 9u, &end, 10);
        if(*end != '\0' || mm < 70u || mm > 1000u) return false;
        t->body_edge_mm = (uint16_t)mm; t->result = "EDGE_RECORDED";
        return true;
    }
    if(!strcmp(line, "STOP")) {
        if(t->phase == ST_RUNNING || t->phase == ST_ARMED) stopping(t, now, "ABORT");
        return true;
    }
    StraightPlan plan;
    float target;
    if(arm_selection(line, &plan, &target)) {
        if(t->phase == ST_RUNNING || t->phase == ST_STOPPING || !in.healthy || !in.stationary) return false;
        if(plan != ST_PLAN_FWD && t->body_edge_mm == 0u) { t->result = "BODY_EDGE_UNSET"; return false; }
        if(!straight_laser_ready(plan, in)) { t->result = "LASER_NOT_READY"; return false; }
        const unsigned guard = t->body_edge_mm + STRAIGHT_TEST_COLLISION_MARGIN_MM;
        if(plan != ST_PLAN_FWD && (in.left_mm <= guard || in.right_mm <= guard || in.rear_mm <= guard || in.front_mm <= guard)) {
            t->result = "COLLISION_GUARD"; return false;
        }
        if(plan == ST_PLAN_APPROACH && in.front_mm <= straight_front_target_mm(t) + 30u) {
            t->result = "FRONT_ALREADY_CLOSE"; return false;
        }
        if(++t->token == 0u) ++t->token;
        t->phase = ST_ARMED; t->plan = plan; t->target_m = target; t->leg = ST_LEG_OUT;
        t->arm_ms = now;
        t->distance_m = t->speed_mps = t->lateral_mps = t->outbound_m = 0.0f;
        t->right_ref_mm = t->right_filtered_mm = 0.0f;
        t->stopped_timing = false; t->result = "ARMED";
        t->guard_mask = 0u;
        t->guard_clear = false;
        t->laser_paused = false;
        t->wrong_direction_timing = false;
        return true;
    }
    if(t->phase == ST_ARMED && (uint32_t)(now - t->arm_ms) <= 5000u &&
       in.healthy && in.stationary && straight_laser_ready(t->plan, in) &&
       !collision_mask(t, in) &&
       (t->plan != ST_PLAN_APPROACH || in.front_mm > straight_front_target_mm(t) + STRAIGHT_TEST_FRONT_TOL_MM) &&
       token_command(line, "GO ", t->token)) {
        t->phase = ST_RUNNING;
        t->start_ms = t->leg_start_ms = t->last_ms = t->heartbeat_ms = now;
        t->right_ref_mm = t->right_filtered_mm = (float)in.right_mm;
        t->laser_frame = in.laser_frame; t->result = "RUNNING";
        return true;
    }
    /* Legacy HB packets are harmless, but operation no longer leases motion
     * from the host. Distance, sensors and the independent RC govern stopping. */
    if(t->phase == ST_RUNNING &&
       token_command(line, "HB ", t->token)) { t->heartbeat_ms = now; return true; }
    return false;
}
static void wall_correction(StraightTest* t, StraightInput in, float dt) {
    if(in.laser_frame != t->laser_frame) {
        t->right_filtered_mm += 0.25f * ((float)in.right_mm - t->right_filtered_mm);
        t->laser_frame = in.laser_frame;
    }
    /* Public chassis coordinates: +vy left. Too close -> move left.
     * Reverse uses the same sign because the body does not turn around. */
    const float error = t->right_ref_mm - t->right_filtered_mm;
    float target = fabsf(error) <= 5.0f ? 0.0f : STRAIGHT_TEST_WALL_KP * error * 0.001f;
    target = fmaxf(-STRAIGHT_TEST_WALL_MAX_VY_MPS, fminf(STRAIGHT_TEST_WALL_MAX_VY_MPS, target));
    const float delta = STRAIGHT_TEST_WALL_ACCEL_MPS2 * dt;
    t->lateral_mps += fmaxf(-delta, fminf(delta, target - t->lateral_mps));
}
void straight_step(StraightTest* t, uint32_t now, StraightInput in) {
    const uint32_t elapsed = now - t->last_ms;
    const float dt = (float)elapsed * 0.001f;
    t->last_ms = now;
    /* While stationary and merely prepared, laser readiness may recover before
     * GO. GO still checks the current frame and never starts on invalid data. */
    if(t->phase == ST_ARMED && (!in.healthy || now - t->arm_ms > 5000u))
        straight_fault(t, "ARM_EXPIRED_OR_NOT_READY");
    if(t->phase == ST_RUNNING) {
        float target;
        if(!in.healthy || !isfinite(in.vx_mps) || !isfinite(in.vy_mps)) { straight_fault(t, "FEEDBACK_OR_PERMIT"); return; }
        const bool guard_sample_timeout = !t->laser_paused && t->guard_mask && !t->guard_clear &&
            now - t->guard_ms >= LASER_FRAME_TIMEOUT_MS;
        const uint8_t needed_mask = t->guard_mask | (t->laser_paused ? t->laser_pause_mask : 0u);
        const bool laser_ready = straight_motion_laser_ready(t, in) && !(needed_mask & in.laser_stale_mask);
        if(elapsed > 50u) { straight_fault(t, "CONTROL_TIMING"); return; }
        const uint32_t timeout = t->plan == ST_PLAN_APPROACH ? STRAIGHT_TEST_APPROACH_TIMEOUT_MS :
            (uint32_t)(t->target_m / t->max_speed_mps * 1000.0f) + 15000u;
        if(!t->laser_paused && laser_ready && ((t->leg != ST_LEG_WAIT && now - t->leg_start_ms > timeout) ||
           now - t->start_ms > 2u * timeout + 5000u)) {
            straight_fault(t, "RUN_TIMEOUT"); return;
        }
        const bool opposite = !t->guard_mask && !t->laser_paused && laser_ready &&
            ((t->leg == ST_LEG_OUT && in.vx_mps < -0.03f) || (t->leg == ST_LEG_BACK && in.vx_mps > 0.03f));
        if(opposite && !t->wrong_direction_timing) { t->wrong_direction_timing = true; t->wrong_direction_ms = now; }
        if(!opposite) t->wrong_direction_timing = false;
        if(fabsf(in.vx_mps) > fmaxf(0.30f, t->max_speed_mps + 0.15f) ||
           (opposite && (fabsf(in.vx_mps) > 0.30f || now - t->wrong_direction_ms >= 100u))) {
            straight_fault(t, "VELOCITY_RANGE"); return;
        }
        t->distance_m += in.vx_mps * dt;
        if(t->distance_m > t->target_m + 0.10f || (round_trip(t) && t->distance_m < -0.10f)) {
            straight_fault(t, "OVERSHOOT"); return;
        }
        const uint8_t near = laser_ready ? collision_mask(t, in) : 0u;
        if((!laser_ready && t->leg != ST_LEG_WAIT) || guard_sample_timeout) {
            if(in.laser_fresh && !in.laser_stale_mask && !guard_sample_timeout) {
                straight_fault(t, "LASER_INVALID_OR_STALE"); return;
            }
            if(!t->laser_paused) {
                t->laser_paused = true; t->laser_pause_ms = now;
                t->laser_pause_mask = in.laser_fresh ? in.laser_stale_mask : 15u;
                t->laser_pause_mask |= t->guard_mask;
            }
            t->laser_pause_mask |= in.laser_stale_mask;
            t->guard_mask = 0u;
            t->laser_recovery_frame = in.laser_frame;
            t->laser_recovery_frames = 0u;
            t->stopped_timing = false;
        }
        if(t->laser_paused) {
            t->speed_mps = t->lateral_mps = 0.0f;
            t->result = "LASER_WAIT";
            if(now - t->laser_pause_ms >= STRAIGHT_TEST_LASER_RECOVERY_MS) {
                straight_fault(t, "LASER_RECOVERY_TIMEOUT"); return;
            }
            if(laser_ready && in.laser_frame != t->laser_recovery_frame) {
                t->laser_recovery_frame = in.laser_frame;
                if(near) {
                    const uint8_t confirmed_near = near & in.laser_updated_mask;
                    if(confirmed_near && in.laser_frame != t->guard_frame && (confirmed_near & t->guard_mask)) {
                        straight_fault(t, "COLLISION_GUARD"); return;
                    }
                    if(confirmed_near) {
                        t->guard_mask = confirmed_near;
                        t->guard_frame = in.laser_frame;
                    }
                    t->laser_recovery_frames = 0u;
                    t->stopped_timing = false;
                    return;
                }
                if((in.laser_updated_mask & t->guard_mask) != t->guard_mask) {
                    t->laser_recovery_frames = 0u;
                    t->stopped_timing = false;
                    return;
                }
                t->guard_mask = 0u;
                if((in.laser_updated_mask & t->laser_pause_mask) == t->laser_pause_mask) {
                    if(t->laser_recovery_frames < 2u) ++t->laser_recovery_frames;
                } else t->laser_recovery_frames = 0u;
            }
            if(!laser_ready || near || !in.stationary || t->laser_recovery_frames < 2u) {
                t->stopped_timing = false; return;
            }
            if(!t->stopped_timing) { t->stopped_timing = true; t->stopped_ms = now; }
            if(now - t->stopped_ms < 200u) return;
            const uint32_t pause_ms = now - t->laser_pause_ms;
            t->start_ms += pause_ms; t->leg_start_ms += pause_ms;
            t->laser_paused = false; t->guard_mask = 0u; t->guard_clear = false;
            t->stopped_timing = false;
            t->result = t->leg == ST_LEG_BACK ? "RETURNING" : "RUNNING";
        }
        if(t->guard_mask) {
            t->speed_mps = t->lateral_mps = 0.0f;
            if(!t->guard_clear && now - t->guard_ms >= LASER_FRAME_TIMEOUT_MS) {
                straight_fault(t, "GUARD_SAMPLE_TIMEOUT"); return;
            }
            if(t->guard_clear && now - t->guard_ms >= 3000u) {
                straight_fault(t, "GUARD_STOP_TIMEOUT"); return;
            }
            if(in.laser_frame != t->guard_frame) {
                if(near & in.laser_updated_mask) {
                    straight_fault(t, "COLLISION_GUARD"); return;
                }
                /* Cached values never count as a fresh confirmation. */
                if(laser_ready && (in.laser_updated_mask & t->guard_mask) == t->guard_mask) {
                    t->guard_clear = true;
                    t->guard_frame = in.laser_frame;
                }
            }
            if(!t->guard_clear || !in.stationary) { t->stopped_timing = false; return; }
            if(!t->stopped_timing) { t->stopped_timing = true; t->stopped_ms = now; }
            if(now - t->stopped_ms < 200u) return;
            t->guard_mask = 0u;
            t->guard_clear = false;
            t->stopped_timing = false;
            t->result = t->leg == ST_LEG_BACK ? "RETURNING" : t->leg == ST_LEG_WAIT ? "TURNAROUND" : "RUNNING";
        } else if(near) {
            t->guard_mask = near; t->guard_frame = in.laser_frame; t->guard_ms = now;
            t->guard_clear = false;
            t->stopped_timing = false;
            t->speed_mps = t->lateral_mps = 0.0f;
            t->result = "GUARD_WAIT";
            return;
        }
        if(laser_ready && t->plan == ST_PLAN_WALL && fabsf((float)in.right_mm - t->right_ref_mm) > 250.0f) {
            straight_fault(t, "WALL_DISTANCE_JUMP"); return;
        }
        if(t->leg == ST_LEG_WAIT) {
            t->speed_mps = t->lateral_mps = 0.0f;
            if(in.stationary) {
                if(!t->stopped_timing) { t->stopped_ms = now; t->stopped_timing = true; }
                if(laser_ready && now - t->stopped_ms >= 200u + STRAIGHT_TEST_TURN_DWELL_MS) {
                    t->outbound_m = t->distance_m; t->leg = ST_LEG_BACK; t->leg_start_ms = now;
                    t->stopped_timing = false; t->result = "RETURNING";
                }
            } else t->stopped_timing = false;
            if(now - t->turn_ms > 5000u)
                straight_fault(t, laser_ready ? "TURN_STOP_TIMEOUT" : "TURN_LASER_TIMEOUT");
            return;
        }
        if(t->plan == ST_PLAN_APPROACH) {
            if(in.front_mm <= straight_front_target_mm(t) + STRAIGHT_TEST_FRONT_TOL_MM) {
                stopping(t, now, "FRONT_DISTANCE_REACHED"); return;
            }
            if(t->distance_m >= t->target_m) { straight_fault(t, "APPROACH_TRAVEL_LIMIT"); return; }
            target = fminf(0.05f, 0.5f * ((float)in.front_mm - straight_front_target_mm(t)) * 0.001f);
        } else {
            const float remaining = t->leg == ST_LEG_BACK ? t->distance_m : t->target_m - t->distance_m;
            if(remaining <= 0.01f) {
                if(round_trip(t) && t->leg == ST_LEG_OUT) {
                    t->leg = ST_LEG_WAIT; t->turn_ms = now; t->stopped_timing = false;
                    t->speed_mps = t->lateral_mps = 0.0f; t->result = "TURNAROUND";
                } else stopping(t, now, round_trip(t) ? "ROUND_COMPLETE" : "TARGET_REACHED");
                return;
            }
            target = fminf(t->max_speed_mps, sqrtf(2.0f * STRAIGHT_TEST_PLAN_DECEL_MPS2 * remaining));
            if(round_trip(t)) {
                const uint16_t range = t->leg == ST_LEG_BACK ? in.rear_mm : in.front_mm;
                const float closing = fmaxf(fabsf(in.vx_mps), fabsf(t->speed_mps));
                const float lookahead = closing * (STRAIGHT_TEST_SPEED_LOOKAHEAD_S + (float)in.laser_age_ms * 0.001f);
                const float clearance = fmaxf(0.0f, ((float)range - t->body_edge_mm -
                    STRAIGHT_TEST_COLLISION_MARGIN_MM - STRAIGHT_TEST_SPEED_RESERVE_MM) * 0.001f - lookahead);
                const float delay_velocity = STRAIGHT_TEST_BRAKE_DECEL_MPS2 * STRAIGHT_TEST_SENSOR_DELAY_S;
                /* Invert the existing braking budget with extra room so smooth
                 * approach slows down before the hard collision guard fires. */
                const float safe_speed = sqrtf(delay_velocity * delay_velocity +
                    2.0f * STRAIGHT_TEST_BRAKE_DECEL_MPS2 * clearance) - delay_velocity;
                target = fminf(target, fmaxf(0.0f, safe_speed - STRAIGHT_TEST_SPEED_TRACKING_MARGIN_MPS));
            }
        }
        if(in.laser_predicted_mask) {
            const float cap = fmaxf(LASER_PREDICT_SPEED_CAP_MPS,
                fabsf(t->speed_mps) - t->accel_mps2 * dt);
            target = fminf(target, cap);
        }
        const float speed = fminf(target, fabsf(t->speed_mps) + t->accel_mps2 * dt);
        t->speed_mps = t->leg == ST_LEG_BACK ? -speed : speed;
        if(t->plan == ST_PLAN_WALL) wall_correction(t, in, dt);
    } else if(t->phase == ST_STOPPING) {
        if(!in.healthy || !isfinite(in.vx_mps) || !straight_motion_laser_ready(t, in)) {
            straight_fault(t, "STOP_FEEDBACK"); return;
        }
        t->distance_m += in.vx_mps * dt;
        if(t->distance_m > t->target_m + 0.10f || (round_trip(t) && t->distance_m < -0.10f)) {
            straight_fault(t, "STOP_OVERSHOOT"); return;
        }
        if(in.stationary) {
            if(!t->stopped_timing) { t->stopped_ms = now; t->stopped_timing = true; }
            if(now - t->stopped_ms >= 200u) t->phase = ST_DONE;
        } else t->stopped_timing = false;
        if(t->phase == ST_STOPPING && now - t->turn_ms > 3000u) straight_fault(t, "STOP_TIMEOUT");
    }
}
