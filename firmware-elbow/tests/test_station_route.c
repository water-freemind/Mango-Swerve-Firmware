/* Real elbow/CAN state machines plus the full-map closed-loop simulator. */
#define main legacy_endpoint_main
#include "test_work_can.c"
#undef main
#define main legacy_map_main
#include "test_full_map.c"
#undef main

static void route_with_work(const char* speed, const char* approach) {
    FullMapTest map;
    FullMapInput input = {0};
    AutoPose actual = {.2f, -.5f, 1.570796327f};
    AutoTwist velocity = {0};
    unsigned completed = 0;
    unsigned alignment_started = 0, alignment_station = 0, longest_alignment = 0;
    char command[32];
    reset_station(0);
    full_map_init(&map, now_ms);
    input.now = now_ms;
    input.healthy = input.odom_ok = input.stationary = input.aligned = true;
    input.work_enabled = input.work_online = input.work_ready = true;
    input.laser = scan(actual, now_ms);
    assert(full_map_command(&map, "MAP ORIGIN", input));
    snprintf(command, sizeof(command), "MAP SPEED %s", speed);
    assert(full_map_command(&map, command, input));
    snprintf(command, sizeof(command), "MAP APPROACH %s", approach);
    assert(full_map_command(&map, command, input));
    assert(full_map_command(&map, "MAP ARM ALL", input));
    assert(full_map_command(&map, "MAP GO 1", input));
    while(now_ms < 850000u && map.phase != MAP_DONE) {
        unsigned region = 0;
        int dx = 9999;
        unsigned before = map.trees_done;
        float target_vx = g_station_work.hold ? 0 : map.command.vx;
        float target_vy = g_station_work.hold ? 0 : map.command.vy;
        AutoVec2 delta;
        velocity.vx += (target_vx - velocity.vx) * .25f;
        velocity.vy += (target_vy - velocity.vy) * .25f;
        velocity.wz += (map.command.wz - velocity.wz) * .25f;
        delta = auto_rotate((AutoVec2){velocity.vx*.01f, velocity.vy*.01f}, actual.yaw);
        actual.x += delta.x;
        actual.y += delta.y;
        actual.yaw = auto_wrap_angle(actual.yaw + velocity.wz*.01f);
        input.stationary = hypotf(velocity.vx, velocity.vy) < .01f && fabsf(velocity.wz) < .02f;
        if(map.target_tree) {
            unsigned index = map.target_tree - 1u;
            float tree_x = map.trees[index/2u][index%2u].x + (index%2u ? .024f : -.024f);
            float distance = fabsf(actual.x - tree_x);
            dx = (int)lroundf((tree_x-actual.x)*2000.0f);
            if(dx > 200) dx=200;
            if(dx < -200) dx=-200;
            region = distance <= .25f ? index%4u + 1u : 0u;
            if(starts > completed) region = 0;
        }
        /* One second of camera silence during entry must leave driving active. */
        int camera = now_ms < 1000u || now_ms >= 2000u;
        run_station(10, region, dx, camera, 1, input.stationary);
        if(starts>completed && Vision_MotorPhase) {
            assert(abs(dx)<=STATION_VISION_RESTART_PX);
            assert(g_station_work.hold && input.stationary);
        }
        input.now = now_ms;
        input.imu_yaw = auto_wrap_angle(actual.yaw - 1.570796327f);
        input.wheel = velocity;
        input.wheel.vx *= 1.032f;
        input.wheel.vy *= 1.032f;
        input.laser = scan(actual, now_ms);
        if(map.segment == 0u) {
            input.laser.accepted_mm[AUTO_REAR] = 80u;
            input.laser.valid_mask |= (uint8_t)(1u << AUTO_REAR);
            input.laser.updated_mask |= (uint8_t)(1u << AUTO_REAR);
            input.laser.sample_age_ms[AUTO_REAR] = 0u;
        }
        input.work_online = g_station_work.online && g_station_work.phase != STATION_LINK_SYNC;
        input.work_ready = station_work_ready();
        input.work_failed = g_station_work.phase == STATION_LINK_FAULT;
        input.work_station = g_station_work.station;
        input.work_slow = g_station_work.slow_requested;
        input.work_wait = g_station_work.stop_requested;
        input.work_active = g_station_work.hold;
        input.work_done = g_station_work.phase == STATION_LINK_DONE;
        input.work_align = g_station_work.phase == STATION_LINK_ALIGN;
        input.work_alignment_fresh = station_work_alignment_fresh(now_ms);
        input.work_centered = g_station_work.aligned;
        input.work_align_direction = g_station_work.align_direction;
        input.work_align_frame = g_station_work.align_updates;
        if(input.work_align && alignment_station!=input.work_station) {
            alignment_station=input.work_station;
            alignment_started=now_ms;
        }
        if(g_station_work.phase==STATION_LINK_WORKING && alignment_started) {
            const unsigned elapsed=now_ms-alignment_started;
            if(elapsed>longest_alignment) longest_alignment=elapsed;
            assert(elapsed<5000u);
            alignment_started=0;
        }
        full_map_step(&map, input);
        if(now_ms >= 1000u && now_ms <= 2000u) {
            assert(map.phase == MAP_RUNNING && !g_station_work.hold);
            assert(map.command.vx > .01f && Vision_Online);
        }
        if(map.phase == MAP_FAULT) {
            fprintf(stderr, "ROUTE_WORK_FAIL speed=%s now=%u reason=%s station=%u phase=%u\n",
                speed, now_ms, map.reason, map.target_tree, g_station_work.phase);
            assert(0);
        }
        if(map.phase == MAP_RUNNING && map.target_tree) {
            if(g_station_work.station != map.target_tree && station_work_ready())
                assert(station_work_begin(map.target_tree, now_ms));
            if(map.tree_hold_stable && now_ms - map.tree_hold_ms >= 200u) {
                if(g_station_work.phase == STATION_LINK_APPROACH)
                    (void)station_work_align(now_ms, input.stationary);
                else if(map.tree_alignment.ready)
                    (void)station_work_start(now_ms, input.stationary);
            }
        }
        if(before != map.trees_done) {
            assert(input.work_done && !g_station_work.hold && Vision_MotorPhase == 0);
            completed++;
        }
        if(map.tree_hold || g_station_work.hold)
            assert(map.command.vx == 0 && map.command.vy == 0 && map.command.wz == 0);
    }
    assert(map.phase == MAP_DONE && map.coverage == 0x7fu && map.trees_done == 0xfffu);
    assert(completed == 12u && starts == 12u);
    printf("FULL_ROUTE_WORK_OK speed=%s approach=%s stations=%u actions=%u time_ms=%u\n",
        speed, approach, completed, starts, now_ms);
    printf("CONTINUOUS_ALIGNMENT_TIME_OK max_ms=%u stations=%u\n",longest_alignment,completed);
}
int main(void) {
    const char* speeds[] = {"0.50", "0.60", "0.70", "0.80", "0.90"};
    const char* approaches[] = {"0.15", "0.20", "0.25", "0.30", "0.35"};
    unsigned index, approach;
    for(index=0; index<5; index++)
        for(approach=0; approach<5; approach++)
            route_with_work(speeds[index], approaches[approach]);
    return 0;
}
