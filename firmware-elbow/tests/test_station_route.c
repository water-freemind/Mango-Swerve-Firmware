/* Real elbow/CAN state machines plus the full-map closed-loop simulator. */
#define main legacy_endpoint_main
#include "test_work_can.c"
#undef main
#define main legacy_map_main
#include "test_full_map.c"
#undef main

static void route_with_work(const char* speed, const char* approach, int unresolved_tree) {
    FullMapTest map;
    FullMapInput input = {0};
    AutoPose actual = {.2f, -.5f, 1.570796327f};
    AutoTwist velocity = {0};
    unsigned completed = 0;
    unsigned fruit_done = 0;
    unsigned station_start_count = 0;
    unsigned alignment_started = 0, alignment_station = 0, longest_alignment = 0;
    bool first_tree_lost = false;
    char command[32];
    reset_station(0);
    full_map_init(&map, now_ms);
    memset(map.trees,0,sizeof(map.trees)); /* Visual work cannot depend on configured tree X. */
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
        unsigned before = map.work_count;
        unsigned physical_slot = 0;
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
            const unsigned row=map.route.segment[map.segment].row;
            const float west=.78f+row*.02f,east=2.02f+row*.015f;
            unsigned local=fabsf(actual.x-west)<fabsf(actual.x-east)?0u:1u;
            unsigned index=(row-1u)*2u+local;
            float tree_x = local?east:west;
            if(unresolved_tree==4 && row==6u && fabsf(actual.x-1.55f)<fabsf(actual.x-tree_x)) {
                tree_x=1.55f;index=12u;
            }
            physical_slot=index;
            float distance = fabsf(actual.x - tree_x);
            dx = (int)lroundf((tree_x-actual.x)*2000.0f);
            if(dx > 200) dx=200;
            if(dx < -200) dx=-200;
            region = distance <= .25f && !(fruit_done&(1u<<index)) ? index%4u + 1u : 0u;
            if(unresolved_tree==3 || (unresolved_tree==2 && index==11u)) region=0;
            if(starts > station_start_count && (unresolved_tree!=1 || index!=11u)) region = 0;
            if(distance>.40f) dx=9999;
            if(unresolved_tree==5 && g_station_work.station==12u &&
               g_station_work.phase==STATION_LINK_ALIGN) first_tree_lost=true;
            if(first_tree_lost && index==11u) {dx=9999;region=0;}
        }
        /* One second of camera silence during entry must leave driving active. */
        int camera = now_ms < 1000u || now_ms >= 2000u;
        run_station(10, region, dx, camera && now_ms%100u==0u, 1, input.stationary);
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
        input.work_result = g_station_work.result;
        input.work_align = g_station_work.phase == STATION_LINK_ALIGN;
        input.work_can_skip_alignment = station_work_can_skip_alignment();
        input.work_target_missing = g_station_work.visual_online && g_station_work.visual_dx==9999 &&
            now_ms-g_station_work.visual_sample_ms<=STATION_VISION_MAX_AGE_MS;
        input.work_target_fresh = g_station_work.visual_online && g_station_work.visual_dx!=9999 &&
            now_ms-g_station_work.visual_sample_ms<=180u;
        input.work_target_dx = g_station_work.visual_dx-STATION_VISION_CENTER_DX;
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
            if(map.work_request_pending && station_work_ready()) {
                station_start_count=starts;
                assert(station_work_begin_visual(map.target_tree, now_ms,
                    map.route.segment[map.segment].line.end.x>map.route.segment[map.segment].line.start.x));
                map.work_request_pending=false;
            }
            if(map.work_scan_ending || map.tree_alignment.skip_pending) (void)station_work_cancel_scan();
            if(!map.tree_alignment.skip_pending && map.tree_hold_stable && now_ms - map.tree_hold_ms >= 200u) {
                if(g_station_work.phase == STATION_LINK_APPROACH)
                    (void)station_work_align(now_ms, input.stationary);
                else if(map.tree_alignment.ready)
                    (void)station_work_start(now_ms, input.stationary);
            }
        }
        if(before != map.work_count) {
            assert(input.work_done && !g_station_work.hold && Vision_MotorPhase == 0);
            completed++;
            if(unresolved_tree!=1 || physical_slot!=11u) fruit_done|=1u<<physical_slot;
        }
        if(map.tree_hold || g_station_work.hold)
            assert(map.command.vx == 0 && map.command.vy == 0 && map.command.wz == 0);
    }
    const unsigned expected=unresolved_tree==2||unresolved_tree==5?11u:unresolved_tree==3?0u:unresolved_tree==4?13u:12u;
    const unsigned done=unresolved_tree==2?0xbffu:unresolved_tree==3?0u:0xfffu;
    if(map.phase!=MAP_DONE || map.trees_done!=done) fprintf(stderr,"ROUTE_FINAL speed=%s phase=%u segment=%u reason=%s trees=%x expected=%x jobs=%u actual=%.3f,%.3f hold=%u motion=%u target=%.3f pose=%.3f station_phase=%u\n",speed,map.phase,map.segment,map.reason,map.trees_done,done,map.work_count,actual.x,actual.y,map.tree_hold,map.motion.out.phase,map.motion.segment.end.x,map.localization.pose.pose.x,g_station_work.phase);
    assert(map.phase == MAP_DONE && map.coverage == 0x7fu && map.trees_done == done);
    assert(completed == expected && map.work_count==expected && starts == (unresolved_tree==1?15u:expected));
    const bool unresolved=unresolved_tree==1 || unresolved_tree==5;
    assert(map.trees_unresolved == (unresolved?(1u<<11):0u));
    assert(!strcmp(map.reason,unresolved?"ROUTE_COMPLETE_WITH_UNRESOLVED":"ROUTE_COMPLETE"));
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
            route_with_work(speeds[index], approaches[approach], 0);
    route_with_work("0.50", "0.30", 1);
    route_with_work("0.50", "0.30", 2);
    route_with_work("0.50", "0.30", 3);
    route_with_work("0.50", "0.30", 4);
    route_with_work("0.50", "0.30", 5);
    puts("VISUAL_ROUTE_OK displaced_tree_positions_greenless_tree_skipped_no_green_entire_map_no_work_third_tree_supported");
    puts("FULL_ROUTE_RETRY_LIMIT_CONTINUE_OK unresolved_G6_first_tree_recorded_all_12_stations_finished");
    puts("FULL_ROUTE_VISION_LOSS_CONTINUE_OK skip_before_first_strike_return_confirmed_remaining_11_stations_finished");
    return 0;
}
