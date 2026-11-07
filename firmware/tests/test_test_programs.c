#include "straight_test.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static StraightInput ready(void) {
    return (StraightInput){.healthy = true, .stationary = true, .laser_fresh = true,
        .left_mm = 800, .right_mm = 600, .rear_mm = 1200, .front_mm = 1200, .laser_frame = 1};
}
static void begin(StraightTest* t, const char* arm, StraightInput in) {
    char go[32];
    straight_init(t, 0u);
    assert(straight_command(t, arm, 0u, in));
    snprintf(go, sizeof(go), "GO %lu", (unsigned long)t->token);
    assert(straight_command(t, go, 0u, in));
}
static void hb(StraightTest* t, uint32_t now, StraightInput in) {
    char command[32];
    if(t->phase == ST_RUNNING && now % 200u == 0u) {
        snprintf(command, sizeof(command), "HB %lu", (unsigned long)t->token);
        assert(straight_command(t, command, now, in));
    }
}
static uint32_t complete(StraightTest* t, StraightInput in, bool approach) {
    float plant_vx = 0.0f;
    uint32_t first_wait = 0u, first_back = 0u, now;
    for(now = 10u; now < 220000u && t->phase != ST_DONE && t->phase != ST_FAULT; now += 10u) {
        plant_vx += 0.25f * (t->speed_mps - plant_vx);
        in.vx_mps = plant_vx; in.stationary = fabsf(plant_vx) < 0.002f;
        if(now % 100u == 0u) ++in.laser_frame;
        if(approach) in.front_mm = (uint16_t)(1000.0f - t->distance_m * 1000.0f);
        hb(t, now, in);
        straight_step(t, now, in);
        assert(fabsf(t->speed_mps) <= (approach ? 0.05001f : t->max_speed_mps + 0.00001f));
        assert(fabsf(t->lateral_mps) <= 0.02001f);
        if(t->leg == ST_LEG_WAIT) {
            if(!first_wait) first_wait = now;
            assert(t->speed_mps == 0.0f && t->lateral_mps == 0.0f);
        }
        if(t->leg == ST_LEG_BACK && !first_back) first_back = now;
    }
    assert(t->phase == ST_DONE);
    if(first_back) assert(first_back - first_wait >= 2200u);
    return now;
}
int main(void) {
    StraightTest t;
    StraightInput in = ready();
    const char* plans[] = {"ARM ROUND 1", "ARM ROUND 2", "ARM ROUND 4", "ARM WALL 1", "ARM WALL 4"};
    for(unsigned i = 0; i < sizeof(plans) / sizeof(plans[0]); ++i) {
        begin(&t, plans[i], in);
        const uint32_t elapsed = complete(&t, in, false);
        assert(!strcmp(t.result, "ROUND_COMPLETE") && fabsf(t.distance_m) < 0.02f);
        assert(t.outbound_m >= t.target_m - 0.02f && t.outbound_m < t.target_m + 0.03f);
        if(t.target_m == 4.0f) assert(elapsed > 80000u); /* Must survive >60 s and keep reverse-leg HB. */
        straight_step(&t, elapsed + 10000u, in);
        assert(t.phase == ST_DONE && t.speed_mps == 0.0f);
    }
    begin(&t, "ARM ROUND 4", in);
    assert(!straight_command(&t, "SPEED 0.05", 0u, in));
    straight_init(&t, 0u);
    assert(straight_command(&t, "SPEED 0.05", 0u, in));
    assert(!straight_command(&t, "SPEED 0.90", 0u, in));
    assert(straight_command(&t, "ARM ROUND 4", 0u, in));
    assert(straight_command(&t, "GO 1", 0u, in));
    assert(complete(&t, in, false) > 160000u);
    assert(!strcmp(t.result, "ROUND_COMPLETE"));
    in.front_mm = in.rear_mm = 7000u;
    straight_init(&t, 0u);
    assert(straight_command(&t, "SPEED 0.50", 0u, in));
    assert(straight_command(&t, "ARM ROUND 4", 0u, in));
    assert(straight_command(&t, "GO 1", 0u, in));
    assert(complete(&t, in, false) < 50000u);
    assert(!strcmp(t.result, "ROUND_COMPLETE"));
    in = ready();
    straight_init(&t, 0u);
    t.plan = ST_PLAN_ROUND; t.speed_mps = 0.50f; in.vx_mps = 0.50f;
    uint16_t limits[4];
    straight_guard_limits(&t, in, limits);
    assert(limits[3] >= 1530u && limits[2] == 130u && limits[0] == 130u);
    t.speed_mps = in.vx_mps = -0.50f;
    straight_guard_limits(&t, in, limits);
    assert(limits[2] >= 1530u && limits[3] == 130u);
    in = ready();
    /* STOP during the dwell cancels return; old GO can never restart it. */
    begin(&t, "ARM ROUND 2", in);
    t.distance_m = 1.995f;
    straight_step(&t, 10u, in);
    assert(t.leg == ST_LEG_WAIT);
    assert(straight_command(&t, "STOP", 20u, in));
    for(unsigned now = 30; now <= 250; now += 10) straight_step(&t, now, in);
    assert(t.phase == ST_DONE && !strcmp(t.result, "ABORT") && t.leg != ST_LEG_BACK);
    assert(!straight_command(&t, "GO 1", 300u, in));
    begin(&t, "ARM ROUND", in);
    t.distance_m = 0.995f;
    straight_step(&t, 10u, in);
    for(unsigned now = 20; now <= 760; now += 10) straight_step(&t, now, in);
    assert(t.phase == ST_FAULT && !strcmp(t.result, "LINK_TIMEOUT"));

    /* Body-contact references are stationary-only, bounded, and reset on reboot. */
    straight_init(&t, 0u);
    assert(t.body_edge_mm == 110u && straight_front_target_mm(&t) == 210u);
    assert(!straight_command(&t, "EDGE ALL 69", 0u, in));
    assert(!straight_command(&t, "EDGE ALL -1", 0u, in));
    assert(!straight_command(&t, "EDGE ALL 1001", 0u, in));
    assert(!straight_command(&t, "EDGE ALL 140 x", 0u, in));
    assert(straight_command(&t, "EDGE ALL 140", 0u, in));
    assert(straight_front_target_mm(&t) == 240u && t.speed_mps == 0.0f);
    assert(!straight_command(&t, "ARM ROUND 3", 0u, in));
    assert(straight_command(&t, "ARM", 0u, in));
    assert(!straight_command(&t, "EDGE ALL 150", 0u, in));

    /* Every new test requires fresh four-way data; original FWD stays independent. */
    straight_init(&t, 0u);
    in.laser_fresh = false;
    assert(!straight_command(&t, "ARM ROUND 4", 0u, in));
    assert(straight_command(&t, "ARM", 0u, in));
    in = ready();
    for(unsigned axis = 0; axis < 4u; ++axis) {
        begin(&t, "ARM ROUND", in);
        StraightInput blocked = in;
        uint16_t* mm[] = {&blocked.left_mm, &blocked.right_mm, &blocked.rear_mm, &blocked.front_mm};
        *mm[axis] = 129u;
        straight_step(&t, 10u, blocked);
        assert(t.phase == ST_FAULT && t.speed_mps == 0.0f && !strcmp(t.result, "COLLISION_GUARD"));
        assert(!straight_command(&t, "ARM WALL", 20u, blocked));
    }
    begin(&t, "ARM WALL", in);
    in.laser_fresh = false;
    straight_step(&t, 10u, in);
    assert(t.phase == ST_FAULT && !strcmp(t.result, "LASER_INVALID_OR_STALE"));
    in = ready();
    begin(&t, "ARM WALL", in);
    in.right_mm = 900u;
    straight_step(&t, 10u, in);
    assert(t.phase == ST_FAULT && !strcmp(t.result, "WALL_DISTANCE_JUMP"));

    /* Right-wall lateral sign is the same on both forward and reverse legs. */
    for(unsigned back = 0; back < 2; ++back) for(unsigned close = 0; close < 2; ++close) {
        in = ready(); begin(&t, "ARM WALL", in);
        if(back) { t.leg = ST_LEG_BACK; t.distance_m = 0.5f; }
        in.right_mm = close ? 550u : 650u; ++in.laser_frame;
        straight_step(&t, 10u, in);
        assert(close ? t.lateral_mps > 0.0f : t.lateral_mps < 0.0f);
        assert(fabsf(t.lateral_mps) <= 0.000201f);
        const float filtered = t.right_filtered_mm;
        straight_step(&t, 20u, in);
        assert(t.right_filtered_mm == filtered); /* A 100 Hz loop must not refilter a 10 Hz frame. */
        if(back) {
            in.vx_mps = 0.1f;
            straight_step(&t, 30u, in);
            assert(t.phase == ST_FAULT && !strcmp(t.result, "VELOCITY_RANGE"));
        }
    }
    in = ready(); begin(&t, "ARM APPROACH", in);
    assert(complete(&t, in, true) < 35000u);
    assert(!strcmp(t.result, "FRONT_DISTANCE_REACHED") && t.distance_m < 1.0f);
    in = ready(); in.front_mm = 220u;
    straight_init(&t, 0u);
    assert(!straight_command(&t, "ARM APPROACH", 0u, in));
    in = ready(); begin(&t, "ARM APPROACH", in);
    t.distance_m = 1.001f;
    straight_step(&t, 10u, in);
    assert(t.phase == ST_FAULT && !strcmp(t.result, "APPROACH_TRAVEL_LIMIT"));
    puts("TEST_PROGRAMS_OK (1/2/4m, 0.05/0.50mps round trips, reverse, dwell/HB/STOP, wall signs, directional braking, contact guard, approach, edge)");
    return 0;
}
