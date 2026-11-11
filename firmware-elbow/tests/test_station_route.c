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
            float distance = fabsf(actual.x - map.trees[index/2u][index%2u].x);
            dx = distance > .25f ? 200 : distance > .07f ? 80 : 20;
            region = distance <= .25f ? index%4u + 1u : 0u;
            if(starts > completed) region = 0;
        }
        run_station(10, region, dx, 1, 1, input.stationary);
        input.now = now_ms;
        input.imu_yaw = auto_wrap_angle(actual.yaw - 1.570796327f);
        input.wheel = velocity;
        input.wheel.vx *= 1.032f;
        input.wheel.vy *= 1.032f;
        input.laser = scan(actual, now_ms);
        input.work_online = g_station_work.online && g_station_work.phase != STATION_LINK_SYNC;
        input.work_ready = station_work_ready();
        input.work_failed = g_station_work.phase == STATION_LINK_FAULT;
        input.work_station = g_station_work.station;
        input.work_slow = g_station_work.slow_requested;
        input.work_wait = g_station_work.stop_requested;
        input.work_active = g_station_work.hold;
        input.work_done = g_station_work.phase == STATION_LINK_DONE;
        full_map_step(&map, input);
        if(map.phase == MAP_FAULT) {
            fprintf(stderr, "ROUTE_WORK_FAIL speed=%s now=%u reason=%s station=%u phase=%u\n",
                speed, now_ms, map.reason, map.target_tree, g_station_work.phase);
            assert(0);
        }
        if(map.phase == MAP_RUNNING && map.target_tree) {
            if(g_station_work.station != map.target_tree && station_work_ready())
                assert(station_work_begin(map.target_tree, now_ms));
            if(map.tree_hold_stable && now_ms - map.tree_hold_ms >= 200u)
                (void)station_work_start(now_ms, input.stationary);
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
}
int main(void) {
    const char* speeds[] = {"0.30", "0.40", "0.50", "0.60", "0.70"};
    const char* approaches[] = {"0.05", "0.10", "0.15", "0.20", "0.25"};
    unsigned index, approach;
    for(index=0; index<5; index++)
        for(approach=0; approach<5; approach++)
            route_with_work(speeds[index], approaches[approach]);
    return 0;
}
