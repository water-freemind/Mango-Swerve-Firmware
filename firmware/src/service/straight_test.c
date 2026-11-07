#include "straight_test.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static void stopping(StraightTest* t, uint32_t now, const char* reason) {
    t->phase = ST_STOPPING;
    t->speed_mps = 0.0f;
    t->stopped_timing = false;
    t->start_ms = now;
    t->result = reason;
}
void straight_init(StraightTest* t, uint32_t now) {
    memset(t, 0, sizeof(*t));
    t->last_ms = now;
    t->result = "IDLE";
}
void straight_fault(StraightTest* t, const char* reason) {
    t->phase = ST_FAULT;
    t->speed_mps = 0.0f;
    t->result = reason;
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
    if(!strcmp(line, "STOP")) {
        if(t->phase == ST_RUNNING || t->phase == ST_ARMED) stopping(t, now, "ABORT");
        return true;
    }
    if(!strcmp(line, "ARM")) {
        if(t->phase == ST_RUNNING || t->phase == ST_STOPPING || !in.healthy || !in.stationary) return false;
        if(++t->token == 0u) ++t->token;
        t->phase = ST_ARMED;
        t->arm_ms = now;
        t->distance_m = t->speed_mps = 0.0f;
        t->result = "ARMED";
        return true;
    }
    if(t->phase == ST_ARMED && (uint32_t)(now - t->arm_ms) <= 5000u &&
       in.healthy && in.stationary && token_command(line, "GO ", t->token)) {
        t->phase = ST_RUNNING;
        t->start_ms = t->last_ms = t->heartbeat_ms = now;
        t->result = "RUNNING";
        return true;
    }
    if(t->phase == ST_RUNNING && (uint32_t)(now - t->heartbeat_ms) <= 750u &&
       token_command(line, "HB ", t->token)) {
        t->heartbeat_ms = now;
        return true;
    }
    return false;
}
void straight_step(StraightTest* t, uint32_t now, StraightInput in) {
    const uint32_t elapsed = now - t->last_ms;
    t->last_ms = now;
    if(t->phase == ST_ARMED && (!in.healthy || now - t->arm_ms > 5000u)) {
        straight_fault(t, "ARM_EXPIRED_OR_NOT_READY");
    }
    if(t->phase == ST_RUNNING) {
        float target;
        if(!in.healthy || !isfinite(in.vx_mps)) { straight_fault(t, "FEEDBACK_OR_PERMIT"); return; }
        if(elapsed > 50u) { straight_fault(t, "CONTROL_TIMING"); return; }
        if(now - t->heartbeat_ms > 750u) { straight_fault(t, "LINK_TIMEOUT"); return; }
        if(now - t->start_ms > STRAIGHT_TEST_RUN_TIMEOUT_MS) { straight_fault(t, "RUN_TIMEOUT"); return; }
        if(in.vx_mps < -0.03f || fabsf(in.vx_mps) > 0.30f) { straight_fault(t, "VELOCITY_RANGE"); return; }
        t->distance_m += in.vx_mps * (float)elapsed * 0.001f;
        if(t->distance_m > STRAIGHT_TEST_OVERSHOOT_M) { straight_fault(t, "OVERSHOOT"); return; }
        if(t->distance_m >= STRAIGHT_TEST_TARGET_M - 0.01f) { stopping(t, now, "TARGET_REACHED"); return; }
        target = fminf(0.10f, sqrtf(0.20f * (STRAIGHT_TEST_TARGET_M - t->distance_m)));
        t->speed_mps = fminf(target, t->speed_mps + 0.05f * (float)elapsed * 0.001f);
    } else if(t->phase == ST_STOPPING) {
        if(!in.healthy || !isfinite(in.vx_mps)) { straight_fault(t, "STOP_FEEDBACK"); return; }
        t->distance_m += in.vx_mps * (float)elapsed * 0.001f;
        if(t->distance_m > STRAIGHT_TEST_OVERSHOOT_M) { straight_fault(t, "STOP_OVERSHOOT"); return; }
        if(in.stationary) {
            if(!t->stopped_timing) { t->stopped_ms = now; t->stopped_timing = true; }
            if(now - t->stopped_ms >= 200u) t->phase = ST_DONE;
        } else t->stopped_timing = false;
        if(t->phase == ST_STOPPING && now - t->start_ms > 3000u) straight_fault(t, "STOP_TIMEOUT");
    }
}
