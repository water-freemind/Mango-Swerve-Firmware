#include "straight_test.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static StraightInput good = {.healthy = true, .stationary = true};
static void start(StraightTest* t, uint32_t now) {
    char cmd[32];
    assert(straight_command(t, "ARM", now, good));
    snprintf(cmd, sizeof(cmd), "GO %lu", (unsigned long)t->token);
    assert(straight_command(t, cmd, now, good));
}
int main(void) {
    StraightTest t;
    straight_init(&t, 0u);
    assert(t.max_speed_mps == 0.50f && t.accel_mps2 == 0.20f);
    assert(straight_command(&t, "SPEED 0.10", 0u, good));
    assert(straight_command(&t, "ACCEL 0.08", 0u, good));
    assert(!straight_command(&t, "GO 0", 0u, good));
    assert(!straight_command(&t, "GO -1", 0u, good));
    assert(straight_command(&t, "PING", 0u, good) && t.phase == ST_IDLE);
    straight_step(&t, 100u, good);
    assert(t.speed_mps == 0.0f);
    StraightInput bad = good;
    bad.healthy = false;
    assert(!straight_command(&t, "ARM", 100u, bad));
    bad = good; bad.stationary = false;
    assert(!straight_command(&t, "ARM", 100u, bad));
    start(&t, 100u);
    straight_step(&t, 100u, good); /* GO and scheduler may share a tick. */
    assert(t.phase == ST_RUNNING && t.speed_mps == 0.0f);
    assert(!straight_command(&t, "GO 1", 101u, good));
    assert(!straight_command(&t, "HB 1 extra", 101u, good));
    assert(!straight_command(&t, "HB -1", 101u, good));
    uint32_t now = 100u;
    StraightInput moving = good;
    for(unsigned i = 0u; i < 4000u && t.phase == ST_RUNNING; ++i) {
        now += 10u;
        /* No host traffic after GO; the MCU must complete the full distance. */
        moving.vx_mps = t.speed_mps;
        moving.stationary = moving.vx_mps < 0.002f;
        straight_step(&t, now, moving);
        assert(t.speed_mps <= 0.100001f && t.speed_mps >= 0.0f);
    }
    assert(t.phase == ST_STOPPING && t.distance_m >= 0.99f && t.distance_m <= 1.01f);
    for(unsigned i = 0u; i < 22u; ++i) { now += 10u; straight_step(&t, now, good); }
    assert(t.phase == ST_DONE && !strcmp(t.result, "TARGET_REACHED"));
    assert(!straight_command(&t, "GO 1", now, good));
    straight_step(&t, now + 1000u, good);
    assert(t.phase == ST_DONE && t.speed_mps == 0.0f);

    start(&t, now + 1000u);
    assert(!straight_command(&t, "HB 1", now + 1001u, good)); /* old token */
    straight_step(&t, now + 1760u, good);
    assert(t.phase == ST_FAULT);
    start(&t, 30000u);
    assert(straight_command(&t, "HB 3", 30751u, good)); /* optional legacy packet, no lease */
    straight_step(&t, 30751u, good);
    assert(t.phase == ST_FAULT);
    start(&t, 40000u);
    straight_step(&t, 40010u, bad);
    assert(t.phase == ST_RUNNING); /* stationary false by itself is allowed */
    bad.healthy = false;
    straight_step(&t, 40020u, bad);
    assert(t.phase == ST_FAULT && t.speed_mps == 0.0f);
    start(&t, 50000u);
    moving.vx_mps = -0.10f;
    for(uint32_t tick = 50010u; tick <= 50110u; tick += 10u) straight_step(&t, tick, moving);
    assert(t.phase == ST_FAULT);
    start(&t, 51000u);
    moving.vx_mps = -0.031f;
    straight_step(&t, 51010u, moving);
    assert(t.phase == ST_RUNNING);
    moving.vx_mps = 0.0f;
    straight_step(&t, 51020u, moving);
    assert(t.phase == ST_RUNNING && !t.wrong_direction_timing);
    assert(straight_command(&t, "STOP", 51030u, good));
    for(uint32_t tick = 51040u; tick <= 51250u; tick += 10u) straight_step(&t, tick, good);
    assert(t.phase == ST_DONE);
    start(&t, 60000u);
    moving.vx_mps = NAN;
    straight_step(&t, 60010u, moving);
    assert(t.phase == ST_FAULT);
    start(&t, 70000u);
    assert(straight_command(&t, "STOP", 70010u, good));
    assert(t.phase == ST_STOPPING && t.speed_mps == 0.0f);
    moving = good; moving.stationary = false;
    straight_step(&t, 73020u, moving);
    assert(t.phase == ST_FAULT);
    assert(straight_command(&t, "ARM", 80000u, good));
    straight_step(&t, 85001u, good);
    assert(t.phase == ST_FAULT);
    start(&t, 0xfffffff0u);
    assert(straight_command(&t, "STOP", 1u, good));
    straight_step(&t, 10u, good);
    straight_step(&t, 220u, good);
    assert(t.phase == ST_DONE && !strcmp(t.result, "ABORT"));
    start(&t, 1000u);
    t.distance_m = 1.11f;
    straight_step(&t, 1010u, good);
    assert(t.phase == ST_FAULT && !strcmp(t.result, "OVERSHOOT"));
    start(&t, 2000u);
    t.heartbeat_ms = 27001u;
    t.last_ms = 26991u;
    straight_step(&t, 27001u, good);
    assert(t.phase == ST_FAULT && !strcmp(t.result, "RUN_TIMEOUT"));
    start(&t, 50000u);
    assert(straight_command(&t, "STOP", 50010u, good));
    assert(!straight_command(&t, "ARM", 50011u, good));
    straight_step(&t, 50020u, good);
    moving = good; moving.stationary = false;
    straight_step(&t, 50120u, moving);
    straight_step(&t, 50200u, good);
    straight_step(&t, 50350u, good);
    assert(t.phase == ST_STOPPING); /* Requires a continuous stationary window. */
    straight_step(&t, 50401u, good);
    assert(t.phase == ST_DONE);
    puts("STRAIGHT_TEST_OK (one-shot GO without HB, distance stop, RC/feedback loss, timing and token checks)");
    return 0;
}
