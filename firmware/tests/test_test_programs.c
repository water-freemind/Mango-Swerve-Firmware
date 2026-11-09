#include "straight_test.h"
#include "laser.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

uint32_t g_test_tick_ms;

static void test_filtered_drive_spikes(void) {
    for(unsigned back = 0u; back < 2u; ++back) {
        const uint32_t base = 20000u + back * 1000u;
        StraightInput in = {.healthy = true, .stationary = true};
        StraightTest t;
        for(uint32_t tick = 0u; tick <= 600u; tick += 10u) {
            const uint32_t now = base + tick;
            if(tick % 100u == 0u) {
                const uint16_t ranges[4] = {800u, 600u,
                    back && tick == 300u ? 2767u : 6682u,
                    !back && tick == 300u ? 2001u : 4544u};
                uint8_t bytes[8];
                for(unsigned i = 0u; i < 4u; ++i) {
                    bytes[2u * i] = (uint8_t)(ranges[i] >> 8u);
                    bytes[2u * i + 1u] = (uint8_t)ranges[i];
                }
                laser_accept_frame(bytes, now);
            }
            LaserSnapshot sample;
            assert(laser_get_snapshot_at(&sample, now));
            in.laser_fresh = sample.frame_fresh;
            in.laser_frame = sample.frame_count;
            in.laser_updated_mask = sample.updated_mask;
            in.laser_stale_mask = (uint8_t)(~sample.valid_mask & 15u);
            in.left_mm = sample.accepted_mm[0]; in.right_mm = sample.accepted_mm[1];
            in.rear_mm = sample.accepted_mm[2]; in.front_mm = sample.accepted_mm[3];
            if(tick < 100u) continue; /* Prime two independent frames. */
            if(tick == 100u) {
                straight_init(&t, 0u);
                assert(straight_command(&t, "SPEED 1.80", 0u, in));
                assert(straight_command(&t, "ARM WALL 4", 0u, in));
                assert(straight_command(&t, "GO 1", 0u, in));
                t.speed_mps = back ? -0.75f : 0.75f;
                if(back) { t.leg = ST_LEG_BACK; t.distance_m = 3.0f; }
                continue;
            }
            in.vx_mps = back ? -0.75f : 0.75f; in.stationary = false;
            straight_step(&t, tick - 100u, in);
            assert(t.phase == ST_RUNNING && !t.guard_mask && !t.laser_paused);
            assert(fabsf(t.speed_mps) > 0.70f);
        }
    }
}

static StraightInput ready(void) {
    return (StraightInput){.healthy = true, .stationary = true, .laser_fresh = true,
        .left_mm = 800, .right_mm = 600, .rear_mm = 1200, .front_mm = 1200, .laser_frame = 1, .laser_updated_mask = 15u};
}
static void begin(StraightTest* t, const char* arm, StraightInput in) {
    char go[32];
    straight_init(t, 0u);
    assert(straight_command(t, "SPEED 0.10", 0u, in));
    assert(straight_command(t, "ACCEL 0.08", 0u, in));
    assert(straight_command(t, arm, 0u, in));
    snprintf(go, sizeof(go), "GO %lu", (unsigned long)t->token);
    assert(straight_command(t, go, 0u, in));
}
static uint32_t complete(StraightTest* t, StraightInput in, bool approach) {
    float plant_vx = 0.0f;
    uint32_t first_wait = 0u, first_back = 0u, now;
    for(now = 10u; now < 220000u && t->phase != ST_DONE && t->phase != ST_FAULT; now += 10u) {
        plant_vx += 0.25f * (t->speed_mps - plant_vx);
        in.vx_mps = plant_vx; in.stationary = fabsf(plant_vx) < 0.002f;
        if(now % 100u == 0u) ++in.laser_frame;
        if(approach) in.front_mm = (uint16_t)(1000.0f - t->distance_m * 1000.0f);
        straight_step(t, now, in);
        assert(fabsf(t->speed_mps) <= (approach ? 0.05001f : t->max_speed_mps + 0.00001f));
        assert(fabsf(t->lateral_mps) <= STRAIGHT_TEST_WALL_MAX_VY_MPS + 0.00001f);
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

static void test_fast_corridor_without_guard_pauses(void) {
    const float accelerations[] = {0.20f, 0.30f, 0.50f};
    for(unsigned setting = 0u; setting < 3u; ++setting) {
        StraightInput in = ready(); in.front_mm = 5500u; in.rear_mm = 4700u;
        StraightTest t;
        begin(&t, "ARM WALL 4", in);
        t.max_speed_mps = 1.8f; t.accel_mps2 = accelerations[setting];
        float measured = 0.0f, previous_distance = 0.0f;
        for(uint32_t now = 10u; now < 60000u && t.phase != ST_DONE && t.phase != ST_FAULT; now += 10u) {
            measured += 0.20f * (t.speed_mps - measured);
            in.vx_mps = measured; in.stationary = fabsf(measured) < 0.002f;
            if(now % 100u == 0u) {
                ++in.laser_frame;
                in.front_mm = (uint16_t)(5500.0f - previous_distance * 1000.0f);
                in.rear_mm = (uint16_t)(4700.0f + previous_distance * 1000.0f);
                previous_distance = t.distance_m;
            }
            in.laser_age_ms = now % 100u;
            straight_step(&t, now, in);
            assert(!t.guard_mask && !t.laser_paused);
        }
        assert(t.phase == ST_DONE && !strcmp(t.result, "ROUND_COMPLETE"));
    }
}
int main(void) {
    test_filtered_drive_spikes();
    test_fast_corridor_without_guard_pauses();
    StraightTest t;
    StraightInput in = ready();
    const char* plans[] = {"ARM ROUND 1", "ARM ROUND 2", "ARM ROUND 4", "ARM WALL 1", "ARM WALL 4"};
    for(unsigned i = 0; i < sizeof(plans) / sizeof(plans[0]); ++i) {
        begin(&t, plans[i], in);
        const uint32_t elapsed = complete(&t, in, false);
        assert(!strcmp(t.result, "ROUND_COMPLETE") && fabsf(t.distance_m) < 0.02f);
        assert(t.outbound_m >= t.target_m - 0.02f && t.outbound_m < t.target_m + 0.03f);
        if(t.target_m == 4.0f) assert(elapsed > 80000u); /* Entire round trip without host traffic. */
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
    /* Reproduce 5616 -> 109 -> 5983 mm behind forward travel. Pause on the
     * first near sample, never count a repeated/cache-only sample twice. */
    in = ready(); in.front_mm = 3542u;
    begin(&t, "ARM WALL 4", in);
    t.distance_m = 1.98f; t.speed_mps = 0.57f; t.max_speed_mps = 1.00f;
    in.vx_mps = 0.57f; in.rear_mm = 109u;
    straight_step(&t, 10u, in);
    assert(t.phase == ST_RUNNING && t.guard_mask == 4u && t.speed_mps == 0.0f);
    const float paused_distance = t.distance_m;
    in.vx_mps = 0.0f;
    straight_step(&t, 20u, in);
    assert(t.guard_mask == 4u && t.phase == ST_RUNNING);
    ++in.laser_frame; in.laser_updated_mask = 11u;
    straight_step(&t, 30u, in);
    assert(t.guard_mask == 4u && t.phase == ST_RUNNING);
    ++in.laser_frame; in.laser_updated_mask = 15u; in.rear_mm = 5983u;
    in.stationary = false;
    straight_step(&t, 40u, in);
    assert(t.guard_mask && t.phase == ST_RUNNING && t.speed_mps == 0.0f);
    in.stationary = true;
    for(uint32_t now = 50u; now <= 250u; now += 10u) straight_step(&t, now, in);
    assert(!t.guard_mask && t.phase == ST_RUNNING && t.speed_mps > 0.0f);
    assert(t.distance_m == paused_distance);
    in = ready(); begin(&t, "ARM WALL 4", in); in.rear_mm = 109u;
    for(uint32_t now = 10u; now <= 310u; now += 10u) straight_step(&t, now, in);
    assert(t.phase == ST_RUNNING && t.laser_paused && !strcmp(t.result, "LASER_WAIT"));
    ++in.laser_frame; in.rear_mm = 5983u;
    straight_step(&t, 320u, in);
    ++in.laser_frame; straight_step(&t, 330u, in);
    for(uint32_t now = 340u; now <= 530u; now += 10u) straight_step(&t, now, in);
    assert(t.phase == ST_RUNNING && !t.laser_paused && t.speed_mps > 0.0f);
    /* Braking feedback may cross zero during a protective pause. It must not
     * be mistaken for actively driving the wrong leg; resume only after stop. */
    for(unsigned back = 0; back < 2u; ++back) {
        in = ready(); in.front_mm = in.rear_mm = 7000u;
        begin(&t, "ARM WALL 4", in); t.max_speed_mps = 1.50f;
        t.leg = back ? ST_LEG_BACK : ST_LEG_OUT; t.distance_m = 1.40f;
        in.left_mm = 109u;
        straight_step(&t, 10u, in);
        assert(t.guard_mask);
        in.vx_mps = back ? 0.788f : -0.788f; in.stationary = false;
        straight_step(&t, 20u, in);
        assert(t.phase == ST_RUNNING && t.guard_mask && t.speed_mps == 0.0f);
        ++in.laser_frame; in.left_mm = 800u;
        straight_step(&t, 30u, in);
        assert(t.guard_clear && t.guard_mask && t.speed_mps == 0.0f);
        in.vx_mps = 0.0f; in.stationary = true;
        for(uint32_t now = 40u; now <= 240u; now += 10u) straight_step(&t, now, in);
        assert(!t.guard_mask && t.phase == ST_RUNNING);
        assert(back ? t.speed_mps < 0.0f : t.speed_mps > 0.0f);
    }
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
    assert(t.phase == ST_RUNNING && t.leg == ST_LEG_WAIT); /* No heartbeat is required during dwell. */

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
        assert(t.phase == ST_RUNNING && t.speed_mps == 0.0f && t.guard_mask);
        ++blocked.laser_frame;
        straight_step(&t, 20u, blocked);
        assert(t.phase == ST_FAULT && t.speed_mps == 0.0f && !strcmp(t.result, "COLLISION_GUARD"));
        assert(!straight_command(&t, "ARM WALL", 20u, blocked));
    }
    begin(&t, "ARM WALL", in);
    in.laser_fresh = false;
    straight_step(&t, 10u, in);
    assert(t.phase == ST_RUNNING && t.laser_paused && t.speed_mps == 0.0f);
    for(uint32_t now = 20u; now <= 5010u; now += 10u) straight_step(&t, now, in);
    assert(t.phase == ST_FAULT && !strcmp(t.result, "LASER_RECOVERY_TIMEOUT"));
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
        assert(fabsf(t.lateral_mps) <= STRAIGHT_TEST_WALL_ACCEL_MPS2 * 0.01f + 0.000001f);
        const float filtered = t.right_filtered_mm;
        straight_step(&t, 20u, in);
        assert(t.right_filtered_mm == filtered); /* A 100 Hz loop must not refilter a 10 Hz frame. */
        if(back) {
            in.vx_mps = 0.1f;
            for(uint32_t now = 30u; now <= 130u; now += 10u) straight_step(&t, now, in);
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
    /* Far returns behind travel must not abort WALL, but closing directions,
     * the wall reference, sentinel/near readings and real stale frames do. */
    in = ready(); begin(&t, "ARM WALL 4", in);
    in.rear_mm = 15001u;
    straight_step(&t, 10u, in);
    assert(t.phase == ST_RUNNING);
    in.front_mm = 15001u;
    straight_step(&t, 20u, in);
    assert(t.phase == ST_FAULT && !strcmp(t.result, "LASER_INVALID_OR_STALE"));
    in = ready(); begin(&t, "ARM WALL 4", in);
    t.leg = ST_LEG_BACK; t.distance_m = 1.0f;
    in.front_mm = 15001u;
    straight_step(&t, 10u, in);
    assert(t.phase == ST_RUNNING && t.speed_mps < 0.0f);
    in.rear_mm = 15001u;
    straight_step(&t, 20u, in);
    assert(t.phase == ST_FAULT);
    in = ready(); begin(&t, "ARM WALL 4", in);
    t.leg = ST_LEG_WAIT; t.distance_m = 3.99f; in.rear_mm = 15001u;
    straight_step(&t, 10u, in);
    assert(t.phase == ST_RUNNING && t.leg == ST_LEG_WAIT && t.speed_mps == 0.0f);
    for(unsigned now = 20u; now <= 2300u; now += 10u) straight_step(&t, now, in);
    assert(t.leg == ST_LEG_WAIT && t.speed_mps == 0.0f);
    in.rear_mm = 1200u;
    straight_step(&t, 2310u, in);
    assert(t.leg == ST_LEG_BACK && t.speed_mps == 0.0f);
    straight_step(&t, 2320u, in);
    assert(t.speed_mps < 0.0f);
    in = ready(); begin(&t, "ARM WALL 4", in);
    t.leg = ST_LEG_WAIT; in.rear_mm = 65127u;
    for(unsigned now = 10u; now <= 5010u; now += 10u) straight_step(&t, now, in);
    assert(t.phase == ST_FAULT && !strcmp(t.result, "TURN_LASER_TIMEOUT"));
    /* An isolated invalid frame during ARM must not consume the preparation.
     * An invalid GO still cannot move, and recovery requires another GO click. */
    in = ready(); straight_init(&t, 0u);
    assert(straight_command(&t, "ARM WALL 4", 0u, in));
    in.front_mm = 65025u;
    straight_step(&t, 10u, in);
    assert(t.phase == ST_ARMED && t.speed_mps == 0.0f);
    assert(!straight_command(&t, "GO 1", 20u, in));
    in.front_mm = 1200u;
    assert(straight_command(&t, "GO 1", 30u, in));
    in = ready(); begin(&t, "ARM WALL 4", in);
    in.rear_mm = UINT16_MAX;
    straight_step(&t, 10u, in);
    assert(t.phase == ST_FAULT);
    in = ready(); in.front_mm = 15000u; in.rear_mm = 7513u;
    assert(straight_laser_ready(ST_PLAN_WALL, in));
    begin(&t, "ARM WALL 4", in);
    in.laser_stale_mask = 4u; /* Rear is unnecessary on the outgoing leg. */
    straight_step(&t, 10u, in);
    assert(t.phase == ST_RUNNING);
    in.laser_stale_mask = 8u; /* Front sample really expired. */
    straight_step(&t, 20u, in);
    assert(t.phase == ST_RUNNING && t.laser_paused && !strcmp(t.result, "LASER_WAIT"));
    in.laser_stale_mask = 0u;
    ++in.laser_frame; straight_step(&t, 30u, in);
    for(uint32_t now = 40u; now <= 250u; now += 10u) straight_step(&t, now, in);
    assert(t.laser_paused); /* Repeated cached frames cannot resume. */
    ++in.laser_frame; straight_step(&t, 260u, in);
    for(uint32_t now = 270u; now <= 460u; now += 10u) straight_step(&t, now, in);
    assert(t.phase == ST_RUNNING && !t.laser_paused && t.speed_mps > 0.0f);
    /* Rear timeout while reversing stops, integrates braking motion, then
     * resumes only after independently updated clear samples and standstill. */
    in = ready(); begin(&t, "ARM WALL 4", in);
    t.leg = ST_LEG_BACK; t.distance_m = 2.745f;
    t.max_speed_mps = 1.8f;
    in.laser_stale_mask = 4u; in.vx_mps = -0.705f; in.stationary = false;
    straight_step(&t, 10u, in);
    const float before_braking = t.distance_m;
    in.vx_mps = -0.05f; straight_step(&t, 20u, in);
    assert(t.laser_paused && t.distance_m < before_braking && t.speed_mps == 0.0f);
    in.laser_stale_mask = 0u; in.vx_mps = 0.0f; in.stationary = true;
    ++in.laser_frame; straight_step(&t, 30u, in);
    ++in.laser_frame; straight_step(&t, 40u, in);
    for(uint32_t now = 50u; now <= 240u; now += 10u) straight_step(&t, now, in);
    assert(!t.laser_paused && t.phase == ST_RUNNING && t.speed_mps < 0.0f);
    in.laser_stale_mask = 4u; straight_step(&t, 250u, in);
    assert(t.laser_paused);
    in.laser_stale_mask = 0u; in.rear_mm = 109u; ++in.laser_frame;
    straight_step(&t, 260u, in);
    assert(t.phase == ST_RUNNING && t.laser_paused && t.guard_mask == 4u && t.speed_mps == 0.0f);
    straight_step(&t, 270u, in); /* Cached near sample cannot confirm itself. */
    assert(t.phase == ST_RUNNING && t.laser_paused);
    ++in.laser_frame;
    straight_step(&t, 280u, in);
    assert(t.phase == ST_FAULT && !strcmp(t.result, "COLLISION_GUARD"));
    /* Field token8: rear expiry followed by one103mm sample, then clear.
     * Preserve the stop while checking, and resume only after fresh clear frames. */
    in = ready(); begin(&t, "ARM WALL 4", in);
    t.leg = ST_LEG_BACK; t.distance_m = 3.835f;
    in.laser_stale_mask = 4u; straight_step(&t, 10u, in);
    in.laser_stale_mask = 0u; in.rear_mm = 103u; ++in.laser_frame;
    straight_step(&t, 20u, in);
    assert(t.phase == ST_RUNNING && t.laser_paused && t.guard_mask == 4u && t.speed_mps == 0.0f);
    ++in.laser_frame; in.laser_updated_mask = 11u; /* Rejected raw rear value. */
    straight_step(&t, 30u, in);
    assert(t.phase == ST_RUNNING && t.laser_paused && t.guard_mask == 4u);
    ++in.laser_frame; in.laser_updated_mask = 15u; in.rear_mm = 6625u;
    straight_step(&t, 40u, in);
    assert(t.phase == ST_RUNNING && t.laser_paused && !t.guard_mask && t.speed_mps == 0.0f);
    ++in.laser_frame; straight_step(&t, 50u, in);
    for(uint32_t now = 60u; now <= 250u; now += 10u) straight_step(&t, now, in);
    assert(t.phase == ST_RUNNING && !t.laser_paused && t.speed_mps < 0.0f);
    in = ready(); begin(&t, "ARM WALL 4", in);
    in.laser_stale_mask = 8u; straight_step(&t, 10u, in);
    assert(t.laser_paused);
    assert(straight_command(&t, "STOP", 20u, in));
    assert(!t.laser_paused && t.phase == ST_STOPPING);
    in = ready(); begin(&t, "ARM WALL 4", in);
    in.laser_stale_mask = 8u; straight_step(&t, 10u, in);
    in.healthy = false; straight_step(&t, 20u, in);
    assert(t.phase == ST_FAULT && !strcmp(t.result, "FEEDBACK_OR_PERMIT"));
    const char* presets[] = {"0.50", "1.00", "1.50", "1.80", "2.00"};
    const float values[] = {0.50f, 1.00f, 1.50f, 1.80f, 2.00f};
    for(unsigned i = 0; i < 5u; ++i) {
        char command[32];
        in = ready(); in.front_mm = in.rear_mm = 7000u;
        straight_init(&t, 0u);
        snprintf(command, sizeof(command), "SPEED %s", presets[i]);
        assert(straight_command(&t, command, 0u, in));
        assert(t.max_speed_mps == values[i]);
        assert(straight_command(&t, "ARM WALL 4", 0u, in));
        assert(straight_command(&t, "GO 1", 0u, in));
        complete(&t, in, false);
        assert(!strcmp(t.result, "ROUND_COMPLETE"));
    }
    /* Reproduce a 4 m corridor whose forward clearance ends near 0.3 m.
     * Feed actual motion back to all travel-axis ranges, including sensor lag. */
    in = ready(); in.front_mm = 4300u;
    begin(&t, "ARM WALL 4", in); t.max_speed_mps = 0.80f;
    float plant_vx = 0.0f;
    bool slowed_before_turn = false;
    for(uint32_t now = 10u; now < 100000u && t.phase == ST_RUNNING; now += 10u) {
        plant_vx += 0.25f * (t.speed_mps - plant_vx);
        in.vx_mps = plant_vx; in.stationary = fabsf(plant_vx) < 0.002f;
        if(now % 100u == 0u) {
            ++in.laser_frame;
            in.front_mm = (uint16_t)(4300.0f - t.distance_m * 1000.0f);
            in.rear_mm = (uint16_t)(1200.0f + t.distance_m * 1000.0f);
        }
        straight_step(&t, now, in);
        assert(!t.guard_mask && !t.laser_paused); /* Smooth corridor must slow before a guard pause. */
        if(t.leg == ST_LEG_OUT && t.distance_m > 2.7f && t.speed_mps < 0.50f) slowed_before_turn = true;
    }
    assert(t.phase == ST_STOPPING && !strcmp(t.result, "ROUND_COMPLETE"));
    assert(slowed_before_turn && t.outbound_m >= 3.98f && t.distance_m < 0.02f);
    in = ready(); straight_init(&t, 0u);
    assert(t.max_speed_mps == 0.50f && t.accel_mps2 == 0.20f);
    const char* bad_accel[] = {"ACCEL ", "ACCEL nan", "ACCEL inf", "ACCEL -0.1", "ACCEL 0", "ACCEL 0.51", "ACCEL 0.2 extra"};
    for(unsigned i = 0; i < sizeof(bad_accel) / sizeof(bad_accel[0]); ++i)
        assert(!straight_command(&t, bad_accel[i], 0u, in));
    in.stationary = false;
    assert(!straight_command(&t, "ACCEL 0.30", 0u, in));
    in.stationary = true;
    assert(straight_command(&t, "ACCEL 0.30", 0u, in));
    assert(t.accel_mps2 == 0.30f && !strcmp(t.result, "ACCEL_SET"));
    assert(straight_command(&t, "ARM WALL 4", 0u, in));
    assert(!straight_command(&t, "ACCEL 0.20", 0u, in));
    assert(straight_command(&t, "GO 1", 0u, in));
    straight_step(&t, 10u, in);
    assert(fabsf(t.speed_mps - 0.003f) < 0.000001f);
    assert(!straight_command(&t, "ACCEL 0.10", 20u, in));
    /* Synthetic persistent left drift reproduces a saturated 0.02 m/s wall
     * controller. The updated controller must finish instead of reaching the
     * 250 mm tracking-error exit. This is an offline plant, not field proof. */
    in = ready(); in.front_mm = in.rear_mm = 7000u;
    begin(&t, "ARM WALL 4", in); t.max_speed_mps = 0.50f;
    float right_mm = in.right_mm, peak_error_mm = 0.0f;
    plant_vx = 0.0f;
    for(uint32_t now = 10u; now < 100000u && t.phase != ST_DONE && t.phase != ST_FAULT; now += 10u) {
        plant_vx += 0.25f * (t.speed_mps - plant_vx);
        in.vx_mps = plant_vx; in.vy_mps = t.lateral_mps + 0.10f * fabsf(plant_vx);
        in.stationary = fabsf(plant_vx) < 0.002f;
        right_mm += in.vy_mps * 10.0f;
        if(now % 100u == 0u) { ++in.laser_frame; in.right_mm = (uint16_t)right_mm; }
        peak_error_mm = fmaxf(peak_error_mm, fabsf(right_mm - 600.0f));
        straight_step(&t, now, in);
    }
    assert(t.phase == ST_DONE && !strcmp(t.result, "ROUND_COMPLETE") && peak_error_mm < 100.0f);
    in = ready(); begin(&t, "ARM WALL 4", in);
    in.right_mm = 700u;
    for(uint32_t now = 10u; now <= 1000u; now += 10u) { ++in.laser_frame; straight_step(&t, now, in); }
    assert(fabsf(t.lateral_mps + STRAIGHT_TEST_WALL_MAX_VY_MPS) < 0.00001f);
    puts("TEST_PROGRAMS_OK legacy/new speed presets, 1/2/4m round trips, directional far returns, clearance-limited corridor return, braking and faults");
    return 0;
}
