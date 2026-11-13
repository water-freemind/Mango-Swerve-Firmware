/* Exercise the actual mode-6 sensor adapter; no serial/CAN hardware is opened. */
typedef struct { unsigned unused; } UART_HandleTypeDef;
#include "../src/app/straight_test_app.c"
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#undef assert
#define assert(x) do { if(!(x)) { fprintf(stderr,"CHECK_FAILED %s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while(0)

uint32_t g_test_tick_ms;
static uint32_t fake_us;
static bool fake_online = true, fake_calibrated = true, fresh_flag;
static bool fake_can_healthy, fake_laser_ready;
static uint16_t fake_swd = 2000u, fake_vrb = 1500u;
static uint16_t fake_swc = 1500u, fake_ry = 1500u, fake_lx = 1500u;
static uint16_t fake_swb = 1500u, fake_rx = 1500u;
static float fake_wheel_speed;
static uint16_t fake_front_mm = 7000u;
static bool fake_drive_gate = true;
static float fake_wheel_limit;
ChassisErrorCode chassis_set_wheel_speed_limit(float limit) { fake_wheel_limit = limit; return CHASSIS_OK; }
static unsigned prepare_calls;
ChassisErrorCode chassis_prepare_velocity(float vx,float vy,float wz) { (void)vx;(void)vy;(void)wz;++prepare_calls;return CHASSIS_OK; }
void chassis_reset_control_history(void) {}
bool chassis_drive_output_allowed(void) { return fake_drive_gate; }
static DriveVelocity last_velocity;
static unsigned sample_reads;
static unsigned motor_stop_calls;
static unsigned chassis_process_calls;
static float fake_yaw;
static char last_line[1024];
static FILE* trace_test_log;
static uint32_t fake_log_cost_ms;
static unsigned map_report_counts[4];
static bool fake_filtered_laser_ready;
static LaserSnapshot fake_filtered_laser;
FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
UART_HandleTypeDef huart1;
static ChassisErrorCode fake_set_velocity(float vx, float vy, float wz) {
    last_velocity = (DriveVelocity){vx, vy, wz}; return CHASSIS_OK;
}
static ChassisErrorCode fake_chassis_process(void) { ++chassis_process_calls; return CHASSIS_OK; }
const struct ChassisInterface chassis_interface = {.set_velocity = fake_set_velocity, .process = fake_chassis_process};
#define ASSEMBLY_STUB(name) SystemStatus name(void) { return SYSTEM_STATUS_OK; }
ASSEMBLY_STUB(assemble_delay)
ASSEMBLY_STUB(assemble_log)
ASSEMBLY_STUB(assemble_imu)
ASSEMBLY_STUB(assemble_laser)
ASSEMBLY_STUB(assemble_can3)
ASSEMBLY_STUB(assemble_chassis)
ASSEMBLY_STUB(assemble_remote)
bool work_link_init(void) { return true; }
static unsigned station_action_commands;
Can3BusStatus can3_bus_register_endpoint(uint16_t id, Can3EndpointCallback callback, void* user) {
    (void)id; (void)callback; (void)user; return CAN3_BUS_OK;
}
Can3BusStatus can3_bus_send(uint16_t id, const uint8_t* data, uint8_t len) {
    assert(id==WORK_CAN_REPLY_ID && len==8u);
    if(data[1]==STATION_APPROACH || data[1]==STATION_START) ++station_action_commands;
    return CAN3_BUS_OK;
}
void remote_init(void) {}
bool uart_receive_it(UART_HandleTypeDef* h, uint8_t* p, uint16_t n) {
    (void)h; (void)p; (void)n; return true;
}
bool uart_abort_receive_it(UART_HandleTypeDef* h) { (void)h; return true; }
void uart_register_rx_complete_callback(UART_HandleTypeDef* h, void (*cb)(void)) { (void)h; (void)cb; }
void uart_register_error_callback(UART_HandleTypeDef* h, void (*cb)(void)) { (void)h; (void)cb; }
void can_poll_rx(FDCAN_HandleTypeDef* h) { (void)h; }
BspCanStatus can_register_rx_callback(FDCAN_HandleTypeDef* h, STM32HalCanRxCallback cb, void* u) {
    (void)h; (void)cb; (void)u; return STM32_HAL_CAN_OK;
}
LogStatus log_error(const char* fmt, ...) { (void)fmt; return LOG_STATUS_OK; }
LogVofaValue log_vofa_value_f64(double v) { return (LogVofaValue){.type = LOG_VOFA_VALUE_F64, .data.f64 = v}; }
LogVofaValue log_vofa_value_u64(unsigned long long v) { return (LogVofaValue){.type = LOG_VOFA_VALUE_U64, .data.u64 = v}; }
LogStatus log_vofa_write(const char* name, uint32_t n, const LogVofaValue* v) {
    (void)name; (void)n; (void)v;
    g_test_tick_ms+=fake_log_cost_ms;fake_us+=fake_log_cost_ms*1000u;
    return LOG_STATUS_OK;
}
static ImuStatus fake_update(void) { fresh_flag = true; return IMU_STATUS_OK; }
static ImuAngle fake_angle(void) { return (ImuAngle){0.0f, 0.0f, fake_yaw}; }
static const ImuInterface fake_imu = {.update = fake_update, .get_angle = fake_angle,
    .status_str = imu_status_str};
const ImuInterface* imu_instance = &fake_imu;
const char* imu_status_str(ImuStatus s) { return s == IMU_STATUS_OK ? "OK" : "NOT_READY"; }
uint32_t dwt_get_us(void) { return fake_us; }
ImuStatus imu_get_sample(ImuSample* s) {
    ++sample_reads;
    if(!fresh_flag) return IMU_STATUS_NOT_READY;
    fresh_flag = false;
    *s = (ImuSample){.gyro_timestamp_us = fake_us, .gyro = {0, 0, -0.0036f},
        .flags = IMU_SAMPLE_GYRO_NEW | IMU_SAMPLE_GYRO_VALID};
    return IMU_STATUS_OK;
}
ImuGyro imu_get_gyro_corrected(void) { return (ImuGyro){0, 0, -0.0036f}; }
ImuStatus bmi088_get_attitude_debug(Bmi088AttitudeDebug* d) {
    *d = (Bmi088AttitudeDebug){.calibrated = fake_calibrated, .has_angle = true,
        .calib_count = 2000u, .calib_target = 2000u, .temperature = 33.0f};
    return IMU_STATUS_OK;
}
void remote_process(void) {}
bool remote_get_state(RemoteState* r) {
    *r = (RemoteState){.online = fake_online, .manual_request = fake_swd == 2000u,
        .manual_source = REMOTE_MANUAL_SOURCE_CHASSIS};
    for(unsigned i = 0u; i < FS_IA10B_CHANNEL_COUNT; ++i) r->rc_data.channel[i] = 1500u;
    r->rc_data.channel[REMOTE_CH_SWA] = 1000u;
    r->rc_data.channel[REMOTE_CH_SWD] = fake_swd;
    r->rc_data.channel[REMOTE_CH_VRB] = fake_vrb;
    r->rc_data.channel[REMOTE_CH_SWC] = fake_swc;
    r->rc_data.channel[REMOTE_CH_SWB] = fake_swb;
    r->rc_data.channel[REMOTE_CH_RIGHT_X] = fake_rx;
    r->rc_data.channel[REMOTE_CH_RIGHT_Y] = fake_ry;
    r->rc_data.channel[REMOTE_CH_LEFT_X] = fake_lx;
    return true;
}
bool remote_is_online(uint32_t timeout) { (void)timeout; return fake_online; }
static float fake_speed(uint16_t id) { (void)id; return fake_wheel_speed; }
static float fake_position(uint16_t id) { return zeros[id - 1u]; }
static float fake_current(uint16_t id) { (void)id; return -16384.0f; }
static BusMotorStatus fake_motor_stop(uint16_t id) { (void)id; ++motor_stop_calls; return MOTOR_STATUS_OK; }
static BusMotorStatus fake_feedback(uint16_t id, BusMotorFeedback* f) {
    *f = (BusMotorFeedback){.position = zeros[id-1u], .speed = fake_wheel_speed}; return MOTOR_STATUS_OK;
}
static const BusMotorInterface fake_motor = {.get_spd = fake_speed, .get_pos = fake_position,
    .get_tor = fake_current, .stop = fake_motor_stop, .brake = fake_motor_stop,
    .disable = fake_motor_stop, .update_feedback = fake_feedback};
const BusMotorInterface* steer_motor_instance = &fake_motor;
const BusMotorInterface* drive_motor_instance = &fake_motor;
bool can_get_diagnostics(FDCAN_HandleTypeDef* b, STM32HalCanDiagnostics* d) {
    (void)b;
    memset(d, fake_can_healthy ? 0 : 0xff, sizeof(*d)); /* Otherwise stress counter widths in log formats. */
    return true;
}
bool laser_get_snapshot(LaserSnapshot* s) {
    if(fake_filtered_laser_ready) { *s=fake_filtered_laser;return true; }
    *s = (LaserSnapshot){.distance_mm = {65535, 65535, 65535, 65535}, .frame_count = UINT32_MAX};
    if(fake_laser_ready) *s = (LaserSnapshot){.distance_mm = {800, 600, 7000, fake_front_mm},
        .accepted_mm = {800, 600, 7000, fake_front_mm}, .valid_mask = 15u, .updated_mask = 15u,
        .frame_fresh = true, .frame_count = g_test_tick_ms, .last_rx_ms = g_test_tick_ms};
    return true;
}
LogStatus log_info(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    const int n = vsnprintf(last_line, sizeof(last_line), fmt, args);
    va_end(args);
    static const char* const map_reports[]={"TEST_MAP_STATUS ","TEST_MAP_TREE ","TEST_MAP_POSE ","TEST_MAP_AISLE "};
    for(unsigned i=0;i<4u;++i) if(!strncmp(last_line,map_reports[i],strlen(map_reports[i])))
        ++map_report_counts[i];
    if(trace_test_log && !strncmp(last_line,"TEST_TRACE",10)) fprintf(trace_test_log,"[INFO] %s\n",last_line);
    if(n < 0 || (unsigned)n + 18u >= LOG_BUFFER_SIZE) {
        fprintf(stderr, "Log exceeds production buffer (%d) %s\n", n, last_line);
        assert(0);
    }
    g_test_tick_ms+=fake_log_cost_ms;fake_us+=fake_log_cost_ms*1000u;
    return LOG_STATUS_OK;
}
static void feedback(uint32_t now) {
    g_test_tick_ms = now;
    for(unsigned i = 0; i < 8; ++i) motor_ms[i] = now;
    for(unsigned i = 0; i < 4; ++i) motor_state[i] = 1;
    seen = 255u;
}
static void receive_command(const char* command, uint32_t now, StraightInput in) {
    for(const char* p = command; *p; ++p) {
        rx_ring[rx_head] = (uint8_t)*p;
        rx_head = (uint8_t)((rx_head + 1u) & 127u);
    }
    commands(now, in);
}
static void loop_at(uint32_t now) {
    fake_us = now * 1000u;
    feedback(now);
    straight_test_app_loop();
}
static void check_round_heading(const char* arm, float yaw) {
    /* Run the actual sensor adapter and scheduler across WAIT -> BACK. Pure
     * planner tests cannot catch a scheduler clearing the shared yaw reference. */
    const uint32_t start = 20000u;
    initialized = true;
    fake_can_healthy = fake_laser_ready = true;
    fake_online = fake_calibrated = true;
    fake_swc = 1500u; fake_swd = 2000u; fake_vrb = 2000u;
    fake_ry = fake_lx = 1500u; fake_yaw = yaw;
    tx_fault = rx_fault = false;
    control_source = (DriveSource){.mode = DRIVE_SOURCE_HOST};
    last_step = last_imu = last_log = last_plan_log = start;
    fake_us = start * 1000u;
    update_imu_sample(); feedback(start);
    StraightInput in = input(start);
    straight_init(&test, start);
    receive_command(arm, start, in);
    receive_command("GO 1\n", start, in);
    assert(test.phase == ST_RUNNING && chassis_yaw_hold_is_active());
    test.distance_m = 0.995f;
    last_velocity = (DriveVelocity){.vx = 0.10f};
    const unsigned stopped_before = motor_stop_calls;
    loop_at(start + 10u);
    assert(test.phase == ST_RUNNING && test.leg == ST_LEG_WAIT);
    assert(last_velocity.vx == 0.0f && last_velocity.vy == 0.0f && last_velocity.wz == 0.0f);
    assert(motor_stop_calls == stopped_before + 8u);
    assert(chassis_yaw_hold_is_active() && fabsf(chassis_yaw_hold_get_yaw_ref() - yaw) < 0.00001f);
    for(uint32_t now = start + 20u; now <= start + 2220u; now += 10u) {
        loop_at(now);
        assert(test.phase == ST_RUNNING); /* Nonzero start yaw must not cause HEADING_DEVIATION. */
        assert(chassis_yaw_hold_is_active() && fabsf(chassis_yaw_hold_get_yaw_ref() - yaw) < 0.00001f);
    }
    assert(test.leg == ST_LEG_BACK && !strcmp(test.result, "RETURNING"));
    const unsigned processed_before = chassis_process_calls;
    fake_yaw += 0.02f;
    for(uint32_t now = start + 2230u; now <= start + 2500u; now += 10u) loop_at(now);
    assert(test.phase == ST_RUNNING && test.leg == ST_LEG_BACK);
    assert(chassis_process_calls > processed_before && last_velocity.vx < -0.01f);
    assert(chassis_yaw_hold_is_active() && fabsf(last_velocity.wz) > 0.001f);
    assert(fabsf(chassis_yaw_hold_get_yaw_ref() - yaw) < 0.00001f);
    /* Guard pause follows the same stop/heading ownership as WAIT. Fresh clear
     * range alone cannot resume while wheel feedback still reports motion. */
    fake_front_mm = 129u;
    const unsigned guard_stops = motor_stop_calls;
    loop_at(start + 2510u);
    assert(test.guard_mask && last_velocity.vx == 0.0f && motor_stop_calls == guard_stops + 8u);
    assert(chassis_yaw_hold_is_active() && fabsf(chassis_yaw_hold_get_yaw_ref() - yaw) < 0.00001f);
    fake_front_mm = 7000u; fake_wheel_speed = 0.5f;
    loop_at(start + 2520u);
    assert(test.guard_mask && test.guard_clear && last_velocity.vx == 0.0f);
    fake_wheel_speed = 0.0f;
    for(uint32_t now = start + 2530u; now <= start + 2750u; now += 10u) loop_at(now);
    assert(!test.guard_mask && test.phase == ST_RUNNING && last_velocity.vx < 0.0f);
    assert(chassis_yaw_hold_is_active() && fabsf(chassis_yaw_hold_get_yaw_ref() - yaw) < 0.00001f);
    fake_laser_ready = false;
    const unsigned paused_processes = chassis_process_calls;
    loop_at(start + 2760u);
    assert(test.laser_paused && test.phase == ST_RUNNING && last_velocity.vx == 0.0f);
    assert(chassis_process_calls == paused_processes && chassis_yaw_hold_is_active());
    fake_laser_ready = true;
    for(uint32_t now = start + 2770u; now <= start + 3000u; now += 10u) loop_at(now);
    assert(!test.laser_paused && test.phase == ST_RUNNING && last_velocity.vx < 0.0f);
    assert(fabsf(chassis_yaw_hold_get_yaw_ref() - yaw) < 0.00001f);
    in = input(start + 3000u);
    receive_command("STOP\n", start + 3000u, in);
    assert(!chassis_yaw_hold_is_active() && last_velocity.vx == 0.0f);
}
static void check_manual_direct_scheduler(void) {
    uint32_t now = 40000u;
    for(unsigned fault = 0; fault < 7; ++fault) {
        initialized = true;
        tx_fault = rx_fault = false;
        fake_can_healthy = fake_online = fake_calibrated = true;
        fake_drive_gate = true;
        fake_swc = fake_swb = 2000u; fake_swd = 1000u; fake_vrb = 1250u;
        fake_ry = fake_rx = fake_lx = 1500u; fake_wheel_speed = 0;
        control_source = (DriveSource){0};
        last_step = last_imu = last_log = last_plan_log = last_manual_process = now;
        straight_init(&test, now);
        loop_at(now += 10u);
        fake_swd = 2000u; loop_at(now += 10u);
        assert(control_source.manual_armed && manual_velocity.vx == 0);
        fake_ry = 2000u;
        for(unsigned i = 0; i < 100; ++i) loop_at(now += 10u);
        assert(manual_velocity.vx == 1.5f && manual_target.vx == 1.5f);
        assert(!chassis_yaw_hold_is_active() && manual_velocity.wz == 0);
        fake_drive_gate = false;
        for(unsigned i = 0; i < 20; ++i) {
            loop_at(now += 10u);
            assert(manual_velocity.vx == 1.5f);
        }
        fake_drive_gate = true; loop_at(now += 10u);
        assert(manual_velocity.vx == 1.5f);
        /* The chassis service still owns the physical steering alignment gate.
         * A safety stop takes effect before the next 10 ms target update. */
        if(fault == 0) fake_swd = 1000u;
        if(fault == 1) fake_vrb = 1251u;
        if(fault == 2) fake_online = false;
        if(fault == 3) fake_swc = 1000u;
        if(fault == 4) fake_can_healthy = false;
        if(fault == 6) {
            StraightInput in = input(now);
            receive_command("STOP\n", now, in);
        } else loop_at(now += fault == 5 ? 51u : 1u);
        assert(!control_source.manual_armed);
        assert(last_velocity.vx == 0 && last_velocity.vy == 0 && last_velocity.wz == 0);
        assert(manual_target.vx == 0 && manual_velocity.vx == 0);
        /* Recovering permit/fault alone cannot resume a previous command. */
        fake_can_healthy = fake_online = true; fake_swc = 2000u; fake_swd = 2000u; fake_vrb = 1250u;
        loop_at(now += 10u); assert(!control_source.manual_armed);
        if(fault == 2) {
            fake_ry = 1500u; fake_wheel_speed = 0.5f;
            loop_at(now += 10u); assert(!control_source.manual_armed);
            fake_wheel_speed = 0.0f; fake_vrb = 1251u;
            loop_at(now += 10u); assert(!control_source.manual_armed);
            fake_vrb = 1250u;
            loop_at(now += 10u);
            assert(control_source.manual_armed && manual_velocity.vx == 0.0f);
            fake_ry = 2000u; loop_at(now += 10u);
            assert(manual_velocity.vx == 1.5f);
            fake_online = false; loop_at(now += 1u);
            assert(!control_source.manual_armed && manual_velocity.vx == 0.0f);
            fake_online = true; loop_at(now += 10u);
            assert(!control_source.manual_armed); /* Repeated outage again requires neutral. */
        }
        fake_ry = 1500u; fake_swd = 1000u; loop_at(now += 10u);
        fake_swd = 2000u; loop_at(now += 10u);
        assert(control_source.manual_armed && manual_velocity.vx == 0);
        fake_ry = 2000u; loop_at(now += 10u);
        assert(manual_velocity.vx == 1.5f);
        /* Stick reversal and release change the target in one control tick. */
        fake_wheel_speed = 0.5f; fake_ry = 1000u;
        loop_at(now += 10u); assert(manual_velocity.vx == -1.5f);
        fake_wheel_speed = 0; loop_at(now += 10u);
        assert(manual_velocity.vx == -1.5f);
        fake_ry = 1500u; loop_at(now += 10u);
        assert(manual_velocity.vx == 0);
        fake_swd = 1000u; loop_at(now += 1u);
        now += 100u;
    }
    puts("MANUAL_DIRECT_SCHEDULER_TEST_OK (direct throttle, release and reverse, stick-only yaw, SWD/VRB/RC/source/CAN/timing/STOP reset and re-arm)");
}
static void check_manual_logging_budget(void) {
    uint32_t now=100000u,max_block_ms=0;
    initialized=true;tx_fault=rx_fault=false;
    fake_can_healthy=fake_online=fake_calibrated=fake_drive_gate=true;
    fake_swc=2000u;fake_swb=1000u;fake_swd=1000u;fake_vrb=1000u;
    fake_ry=fake_rx=fake_lx=1500u;fake_wheel_speed=0;
    control_source=(DriveSource){0};
    last_step=last_imu=last_log=last_plan_log=last_manual_process=now;
    straight_init(&test,now);
    loop_at(now+=10u);fake_swd=2000u;loop_at(now+=10u);
    assert(control_source.manual_armed);
    fake_ry=2000u;fake_log_cost_ms=6u;
    last_map_log=now-500u;last_plan_log=now-200u;last_log=now-100u;
    for(unsigned i=0;i<5000u;++i) {
        loop_at(now);
        const uint32_t blocked=g_test_tick_ms-now;
        if(blocked>max_block_ms) max_block_ms=blocked;
        assert(control_source.manual_armed && control_source.release_seen);
        assert(test.phase!=ST_FAULT);
        now=g_test_tick_ms+2u;
    }
    assert(max_block_ms<50u && manual_velocity.vx==.4f);
    fake_log_cost_ms=0;
    loop_at(now+=51u);
    assert(!control_source.manual_armed && last_velocity.vx==0);
    fake_swd=1000u;fake_ry=1500u;loop_at(now+=10u);
    printf("MANUAL_LOG_BUDGET_OK iterations=5000 line_cost_ms=6 max_batch_ms=%lu real_51ms_gap_stops\n",
        (unsigned long)max_block_ms);
}
static void check_map_logging_budget(void) {
    uint32_t now=200000u;
    uint32_t maximum=0;
    unsigned iteration;
    initialized=true;
    tx_fault=rx_fault=false;
    fake_can_healthy=fake_online=fake_calibrated=fake_drive_gate=true;
    fake_laser_ready=true;
    fake_swc=1500u;
    fake_swd=2000u;
    fake_vrb=1500u;
    fake_ry=fake_rx=fake_lx=1500u;
    fake_wheel_speed=0;
    control_source=(DriveSource){.mode=DRIVE_SOURCE_HOST};
    straight_init(&test,now);
    full_map_init(&full_map,now);
    assert(field_pose_register(&full_map.localization,(AutoPose){.2f,.2f,1.570796327f},fake_yaw,now));
    full_map.phase=MAP_PAUSED;
    full_map.user_hold=true;
    last_step=last_imu=last_log=last_plan_log=last_map_log=now;
    memset(map_report_counts,0,sizeof(map_report_counts));
    fake_log_cost_ms=8u;
    for(iteration=0;iteration<1000u;iteration++) {
        loop_at(now);
        if(g_test_tick_ms-now>maximum) maximum=g_test_tick_ms-now;
        assert(full_map.phase==MAP_PAUSED && full_map.user_hold);
        now=g_test_tick_ms+2u;
    }
    fake_log_cost_ms=0;
    assert(maximum<50u);
    for(unsigned i=0;i<4u;++i) {
        assert(map_report_counts[i]>0u);
        for(unsigned j=0;j<4u;++j)
            assert(abs((int)map_report_counts[i]-(int)map_report_counts[j])<=1);
    }
    full_map_init(&full_map,now);
    printf("MAP_LOG_BUDGET_OK iterations=1000 line_cost_ms=8 maximum_ms=%lu\n",(unsigned long)maximum);
}

static void check_map_laser_filter(void) {
    fake_filtered_laser_ready=true;
    fake_filtered_laser=(LaserSnapshot){.accepted_mm={179,162,6129,1087},
        .median_mm={179,162,6129,2720},.median_age_ms={0,0,0,100},
        .valid_mask=15,.updated_mask=15,.median_valid_mask=15,.median_updated_mask=7,
        .frame_fresh=true,.frame_count=1,.last_rx_ms=g_test_tick_ms};
    full_map.command=(AutoTwist){.vx=.4f};
    StraightInput in={.vx_mps=.364f};
    FullMapInput m=map_input(g_test_tick_ms,in);
    assert(m.laser.accepted_mm[3]==2720 && m.laser.sample_age_ms[3]==100);
    assert(!(m.laser.updated_mask&8u));
    fake_filtered_laser.accepted_mm[3]=1056;
    fake_filtered_laser.median_mm[3]=1095;fake_filtered_laser.median_updated_mask=15;
    m=map_input(g_test_tick_ms,in);assert(m.laser.accepted_mm[3]==1095);
    fake_filtered_laser.accepted_mm[3]=900;
    m=map_input(g_test_tick_ms,in);assert(m.laser.accepted_mm[3]==900);
    full_map.command=(AutoTwist){.vy=.4f};in=(StraightInput){.vy_mps=.376f};
    fake_filtered_laser.accepted_mm[0]=992;fake_filtered_laser.median_mm[0]=1026;
    m=map_input(g_test_tick_ms,in);assert(m.laser.accepted_mm[0]==1026);
    fake_filtered_laser.accepted_mm[0]=99;
    m=map_input(g_test_tick_ms,in);assert(m.laser.accepted_mm[0]==99);
    full_map.motion.initialized=true;full_map.motion.config.decel=.15f;
    full_map.command=(AutoTwist){.vx=.5f};in=(StraightInput){.vx_mps=.482f};
    fake_filtered_laser.accepted_mm[3]=1145;fake_filtered_laser.median_mm[3]=2794;
    m=map_input(g_test_tick_ms,in);assert(m.laser.accepted_mm[3]==2794);
    /* Two real short returns move the median into the protective envelope. */
    fake_filtered_laser.accepted_mm[3]=1131;fake_filtered_laser.median_mm[3]=1145;
    m=map_input(g_test_tick_ms,in);assert(m.laser.accepted_mm[3]==1145);
    fake_filtered_laser.accepted_mm[3]=600;
    m=map_input(g_test_tick_ms,in);assert(m.laser.accepted_mm[3]==600);
    full_map.motion.initialized=false;
    fake_filtered_laser_ready=false;full_map.command=(AutoTwist){0};
    puts("MAP_LASER_FILTER_ADAPTER_OK recorded_front_and_left_ranges median_age urgent_bypass");
}
static void check_trace_adapter(void) {
    chassis_trace_init(&trace);
    trace_recording=true;trace_dumping=trace_was_active=false;
    control_source=(DriveSource){.mode=DRIVE_SOURCE_MANUAL,.manual_armed=true};
    test.phase=ST_FAULT; /* A cancelled HOST task must not freeze MANUAL trace. */
    tx_fault=rx_fault=false;blockers=0;rc_online=true;
    manual_target=manual_velocity=(DriveVelocity){.1f,0,0};
    StraightInput in={.healthy=true,.stationary=false};
    for(unsigned i=0;i<CHASSIS_TRACE_CAPACITY+3u;i++) {
        trace_sensor=(ChassisTraceSample){.t_ms=i*20u,.steer_valid_mask=15,.drive_valid_mask=15,.imu_valid=1};
        trace_capture(in);
    }
    assert(trace_recording && trace.count==CHASSIS_TRACE_CAPACITY && trace.overwritten==3);
    const unsigned motor_before=motor_stop_calls;
    receive_command("TRACE STOP\n",20000u,in);
    assert(!trace_recording && control_source.manual_armed && motor_stop_calls==motor_before);
    receive_command("TRACE START\n",20001u,in);assert(trace_recording);
    manual_target=manual_velocity=(DriveVelocity){0};in.stationary=true;
    trace_sensor.t_ms+=20; trace_capture(in);
    assert(!trace_recording && control_source.manual_armed); /* ordinary release freezes even with permit on */
    control_source=(DriveSource){.mode=DRIVE_SOURCE_STOP};test.phase=ST_IDLE;
    rc_state.manual_request=false;
    rc_state.rc_data.channel[REMOTE_CH_SWD]=1500;
    receive_command("TRACE DUMP\n",20002u,in);assert(!trace_dumping);
    rc_state.rc_data.channel[REMOTE_CH_SWD]=1000;rc_online=false;
    receive_command("TRACE DUMP\n",20003u,in);assert(!trace_dumping);
    rc_online=true;
    const char* path=getenv("MANGO_TRACE_EXPORT");
    if(path) {trace_test_log=fopen(path,"w");assert(trace_test_log);}
    receive_command("TRACE DUMP\n",20004u,in);assert(trace_dumping);
    for(unsigned i=0;i<6000 && trace_dumping;i++) trace_export(20020u+i*20u,in);
    assert(!trace_dumping && trace_dump_sent==CHASSIS_TRACE_CAPACITY);
    if(trace_test_log) {fclose(trace_test_log);trace_test_log=NULL;}
    assert(motor_stop_calls==motor_before);
    receive_command("TRACE DUMP\n",20005u,in);assert(trace_dumping);
    rc_state.rc_data.channel[REMOTE_CH_SWD]=2000;
    trace_export(150000u,in);assert(!trace_dumping); /* permission change cancels output immediately */
    puts("CHASSIS_TRACE_ADAPTER_OK scope, finite window, no motion, SWD/online gating, buffer bounds and export");
}
static void check_map_adapter(void) {
    uint32_t now=60000u;
    initialized=true;tx_fault=rx_fault=false;
    fake_online=fake_calibrated=fake_can_healthy=fake_laser_ready=true;
    fake_swc=1500u;fake_swd=1000u;fake_vrb=2000u;
    fake_ry=fake_rx=fake_lx=1500u;fake_wheel_speed=0;fake_yaw=.2f;fake_front_mm=5059;
    control_source=(DriveSource){.mode=DRIVE_SOURCE_HOST};
    straight_init(&test,now);full_map_init(&full_map,now);
    last_step=last_imu=last_log=last_plan_log=now;last_velocity=(DriveVelocity){0};
    loop_at(now+=10);
    StraightInput in=input(now);
    receive_command("MAP ORIGIN\n",now,in);
    assert(full_map.phase==MAP_READY && full_map.localization.registered);
    loop_at(now+=10);
    assert(full_map.localization.registered); /* SWD off does not erase registration. */
    fake_swd=2000u;loop_at(now+=10);in=input(now);
    receive_command("MAP ARM NEXT\n",now,in);
    assert(full_map.phase==MAP_READY); /* Removed segmented mode cannot start a route. */
    receive_command("MAP ARM ALL\n",now,in);
    assert(full_map.phase==MAP_ARMED && last_velocity.vx==0);
    receive_command("MAP GO 1\n",now,in);
    assert(full_map.phase==MAP_RUNNING);
    fake_drive_gate=false;const unsigned before=prepare_calls;
    loop_at(now+=10);assert(prepare_calls>before && last_velocity.vx==0);
    receive_command("ARM WALL 4\n",now,input(now));
    assert(test.phase==ST_IDLE && !strcmp(test.result,"FULL_MAP_ACTIVE"));
    fake_drive_gate=true;loop_at(now+=10);
    assert(full_map.phase==MAP_RUNNING && last_velocity.vx>0);
    fake_swd=1000u;loop_at(now+=10);
    assert(full_map.phase==MAP_FAULT && last_velocity.vx==0 && last_velocity.vy==0);
    fake_swc=2000u;loop_at(now+=10);
    assert(!full_map.localization.registered);
    receive_command("MAP ARM ALL\n",now,input(now));
    assert(full_map.phase==MAP_FAULT);
    puts("FULL_MAP_ADAPTER_TEST_OK register_while_SWD_off_prepare_without_drive_owner_isolation_RC_stop");
}
static void check_near_recovery_adapter(void) {
    uint32_t now=150000u;
    initialized=true;tx_fault=rx_fault=false;
    fake_online=fake_calibrated=fake_can_healthy=fake_laser_ready=true;
    fake_swc=1500u;fake_swd=2000u;fake_vrb=2000u;
    fake_ry=fake_rx=fake_lx=1500u;fake_wheel_speed=0;fake_yaw=0;
    control_source=(DriveSource){.mode=DRIVE_SOURCE_HOST};
    straight_init(&test,now);full_map_init(&full_map,now);
    assert(field_pose_register(&full_map.localization,(AutoPose){1.5f,4.4f,1.570796327f},0,now));
    full_map.phase=MAP_RUNNING;full_map.segment=2u;
    full_map.run_ms=full_map.segment_ms=full_map.last_ms=now;
    last_step=last_imu=last_log=last_plan_log=last_map_log=now;last_velocity=(DriveVelocity){0};
    fake_filtered_laser_ready=true;
    fake_filtered_laser=(LaserSnapshot){.accepted_mm={1459,1459,95,159},
        .distance_mm={1459,1459,95,159},.valid_mask=15u,.updated_mask=15u,.frame_fresh=true};
    fake_drive_gate=false;const unsigned before=prepare_calls;
    for(unsigned i=0;i<5u;++i) {
        fake_filtered_laser.frame_count=now+10u;loop_at(now+=10u);
        assert(full_map.phase==MAP_PAUSED && full_map.near_recovery.active);
        assert(last_velocity.vx==0 && last_velocity.vy==0);
    }
    assert(prepare_calls>before);
    fake_drive_gate=true;fake_filtered_laser.frame_count=now+10u;loop_at(now+=10u);
    assert(full_map.phase==MAP_PAUSED && !full_map.near_recovery.preparing);
    assert(last_velocity.vx>.039f && last_velocity.vx<=.04f && last_velocity.vy==0 && last_velocity.wz==0);
    fake_swd=1000u;loop_at(now+=10u);
    assert(full_map.phase==MAP_FAULT && !full_map.near_recovery.active);
    assert(last_velocity.vx==0 && last_velocity.vy==0);
    fake_filtered_laser_ready=false;
    puts("NEAR_RECOVERY_ADAPTER_OK actual_scheduler_zero_drive_prepare_feedback_gate_escape_RC_STOP");
}
static void check_map_mode_selection(void) {
    uint32_t now=90000u;
    initialized=true;tx_fault=rx_fault=false;
    fake_online=fake_calibrated=fake_can_healthy=fake_laser_ready=true;
    fake_swc=1500u;fake_swd=1000u;fake_vrb=2000u;
    fake_ry=fake_rx=fake_lx=1500u;fake_wheel_speed=0;fake_yaw=.2f;fake_front_mm=5059;
    fake_drive_gate=true;
    control_source=(DriveSource){.mode=DRIVE_SOURCE_HOST};
    straight_init(&test,now);full_map_init(&full_map,now);
    assert(station_work_init());station_action_commands=0;
    map_work_enabled=WORK_STATION_ENABLED!=0;
    last_step=last_imu=last_log=last_plan_log=now;last_velocity=(DriveVelocity){0};
    loop_at(now+=10);
    StraightInput in=input(now);
    if(WORK_STATION_ENABLED) {
        receive_command("MAP ORIGIN\n",now,in);
        fake_swd=2000u;loop_at(now+=10);in=input(now);
        receive_command("MAP ARM ALL\n",now,in);
        assert(full_map.phase==MAP_READY && !strcmp(full_map.reason,"WORK_PEER_NOT_READY"));
    }
    receive_command("MAP MODE NAV\n",now,in);
    assert(!map_work_enabled && !strcmp(full_map.reason,"MODE_NAV_SELECTED"));
    if(WORK_STATION_ENABLED) assert(!full_map.localization.registered);
    receive_command("MAP ORIGIN\n",now,in);
    assert(full_map.phase==MAP_READY);
    fake_swd=2000u;loop_at(now+=10);in=input(now);
    receive_command("MAP ARM ALL\n",now,in);
    assert(full_map.phase==MAP_ARMED);
    receive_command("MAP MODE WORK\n",now,in);
    assert(!map_work_enabled && full_map.phase==MAP_ARMED);
    assert(!strcmp(full_map.reason,"MODE_NEEDS_IDLE_STOPPED"));
    receive_command("MAP GO 1\n",now,in);
    for(unsigned sample=0u;sample<100u;++sample) {
        g_station_work.phase=sample%2u?STATION_LINK_FAULT:STATION_LINK_SYNC;
        g_station_work.online=false;
        loop_at(now+=10);
        assert(!map_input(now,input(now)).work_enabled);
        assert(full_map.phase==MAP_RUNNING);
    }
    assert(last_velocity.vx>0 && !station_action_commands && !g_station_work.task);
    receive_command("MAP MODE WORK\n",now,input(now));
    assert(!map_work_enabled && full_map.phase==MAP_RUNNING);
    receive_command("MAP STOP\n",now,input(now));
    fake_wheel_speed=.1f;
    receive_command("MAP MODE WORK\n",now,input(now));
    assert(!strcmp(full_map.reason,"MODE_NEEDS_IDLE_STOPPED"));
    fake_wheel_speed=0;
    g_station_work.hold=true;
    receive_command("MAP MODE WORK\n",now,input(now));
    assert(!strcmp(full_map.reason,WORK_STATION_ENABLED?"MODE_WORK_ACTIVE":"MODE_WORK_UNAVAILABLE"));
    g_station_work.hold=false;
    receive_command("MAP MODE WORK\n",now,input(now));
    assert(map_work_enabled==(WORK_STATION_ENABLED!=0));
    assert(!strcmp(full_map.reason,WORK_STATION_ENABLED?"MODE_WORK_SELECTED":"MODE_WORK_UNAVAILABLE"));
    if(WORK_STATION_ENABLED) {
        assert(!full_map.localization.registered);
        receive_command("MAP ORIGIN\n",now,input(now));
        receive_command("MAP ARM ALL\n",now,input(now));
        assert(!strcmp(full_map.reason,"WORK_PEER_NOT_READY") && full_map.phase==MAP_READY);
    }
    map_work_enabled=false;
    puts("MAP_NAV_MODE_UART_TEST_OK offline_peer_does_not_block_navigation_mode_change_gates_work_mode_checks_peer");
}
int main(void) {
    map_work_enabled=false;
    test.phase = ST_IDLE;
    test.target_m = 4.0f;
    test.body_edge_mm = 110u;
    fake_us = 1000000u;
    update_imu_sample();
    feedback(1000u);
    StraightInput in = input(1000u);
    assert(in.healthy && sample_reads == 1u);
    /* HOST ignores the spring-return VRB wheel; SWD remains the stop gate. */
    fake_vrb = 2000u;
    assert(input(1000u).healthy);
    fake_vrb = 1000u;
    assert(input(1000u).healthy);
    fake_vrb = 1500u;
    fake_swd = 1000u;
    in = input(1000u);
    assert(!in.healthy && (blockers & BLOCK_RC_SWITCH));
    fake_swd = 2000u;
    assert(input(1000u).healthy);
    fake_us += 2000u;
    in = input(1002u);
    assert(in.healthy && sample_reads == 1u); /* A consumed NEW flag is not a stale frame. */
    feedback(1003u); /* A feedback interrupt was newer than the outer loop's tick. */
    in = input(1002u);
    assert(in.healthy && motor_age[0] == 0u);
    fake_online = false;
    in = input(1002u);
    assert(!in.healthy && (blockers & BLOCK_RC_OFFLINE) && imu_debug_valid);
    fake_online = true;
    fake_calibrated = false;
    in = input(1002u);
    assert(!in.healthy && (blockers & BLOCK_IMU_CAL)); /* A finite yaw alone is not ready. */
    fake_calibrated = true;
    fake_us += 100001u;
    feedback(1104u);
    in = input(1104u);
    assert(!in.healthy && (blockers & BLOCK_IMU_STALE));

    /* Five-second observation across the +/-pi boundary, without auto-zeroing. */
    fake_us = 2000000u;
    fake_yaw = -3.14f;
    update_imu_sample();
    feedback(2000u);
    in = input(2000u);
    update_drift(in);
    for(unsigned i = 1u; i <= 1250u; ++i) {
        fake_us += 4000u;
        fake_yaw = remainderf(fake_yaw - 0.0036f * 0.004f, 6.283185307f);
        update_imu_sample();
        feedback(fake_us / 1000u);
        in = input(fake_us / 1000u);
        update_drift(in);
    }
    assert(drift_valid && drift_window_ms == 5000u);
    assert(fabsf(drift_yaw_rate + 0.20626f) < 0.003f);
    assert(fabsf(drift_gyro_rate + 0.20626f) < 0.003f);
    test.phase = ST_RUNNING;
    update_drift(in);
    assert(!drift_valid);
    test.phase = ST_IDLE;
    for(unsigned i = 0; i < 8; ++i) motor_age[i] = UINT32_MAX;
    for(unsigned i = 0; i < 16; ++i) log_detail(UINT32_MAX, in);
    test.plan = ST_PLAN_APPROACH;
    test.distance_m = test.outbound_m = 4.1f;
    test.right_ref_mm = 3000.0f;
    test.body_edge_mm = 1000u;
    log_plan(in);
    /* Exercise production handover and UART dispatch, not just a selector mock. */
    straight_init(&test, 9000u);
    fake_us = 9000000u;
    update_imu_sample(); feedback(9000u);
    in = input(9000u);
    update_control_source(9000u, in);
    assert(control_source.mode == DRIVE_SOURCE_HOST);
    receive_command("ARM\n",9000u,in);
    receive_command("GO 1\n",9000u,in);
    assert(test.phase == ST_RUNNING);
    assert(chassis_yaw_hold_is_active()); /* HOST still acquires a heading reference. */
    fake_vrb = 2000u;
    in = input(9000u); update_control_source(9000u, in);
    assert(in.healthy && test.phase == ST_RUNNING && control_source.mode == DRIVE_SOURCE_HOST);
    detail_slot = 1u; log_detail(9000u, in);
    assert(strstr(last_line, "vrb_required=0"));
    last_velocity = (DriveVelocity){.vx = 0.10f}; /* Cached command from the outgoing owner. */
    rx_ring[rx_head] = 'G'; rx_head = (uint8_t)((rx_head + 1u) & 127u);
    fake_swc = 2000u;
    in = input(9000u); update_control_source(9000u, in);
    assert(test.phase == ST_FAULT && test.speed_mps == 0.0f && rx_head == rx_tail);
    assert(last_velocity.vx == 0.0f && last_velocity.vy == 0.0f && last_velocity.wz == 0.0f);
    assert(!control_source.manual_armed);
    fake_vrb = 1250u;
    fake_swd = 1000u;
    in = input(9000u); update_control_source(9000u, in);
    fake_swd = 2000u;
    in = input(9000u); update_control_source(9000u, in);
    assert(control_source.manual_armed);
    /* A healthy SWC handover needs no extra SWD cycle. The same-frame manual
     * enable starts with fresh ramp timing and cannot inherit HOST output. */
    fake_swc = 1500u;
    in = input(9000u); update_control_source(9000u, in);
    assert(!control_source.manual_armed && control_source.release_seen);
    test.phase = ST_RUNNING;
    last_velocity = (DriveVelocity){.vx = 0.10f};
    last_manual_target = 100u;
    fake_swc = 2000u;
    in = input(9000u); update_control_source(9000u, in);
    assert(control_source.manual_armed && test.phase == ST_FAULT);
    assert(fake_wheel_limit == APP_MANUAL_MAX_LINEAR_MPS);
    assert(last_velocity.vx == 0.0f && manual_velocity.vx == 0.0f);
    assert(last_manual_target == 9000u);
    fake_ry = 2000u;
    in = input(9000u); assert(in.healthy && !rc_centered); /* Joystick movement is valid only for manual. */
    receive_command("ARM\nGO 1\nHB 1\nSPEED 0.50\n",9000u,in);
    assert(test.phase == ST_FAULT && fabsf(test.max_speed_mps - STRAIGHT_TEST_DEFAULT_SPEED_MPS) < 0.00001f && control_source.manual_armed);
    rx_fault = true;
    in = input(9000u); assert(in.healthy); /* A serial error cannot remove manual permit. */
    commands(9000u,in);
    assert(control_source.manual_armed);
    chassis_yaw_hold_set_target(fake_yaw); /* A stale HOST reference cannot affect manual. */
    fake_yaw += 0.2f;
    apply_manual(9010u, in);
    assert(last_velocity.vx == 0.8f && last_velocity.vy == 0.0f);
    assert(last_velocity.wz == 0.0f && !chassis_yaw_hold_is_active());
    fake_yaw -= 0.4f;
    apply_manual(9020u, in);
    assert(last_velocity.wz == 0.0f); /* Yaw/gyro drift cannot create a manual turn. */
    fake_lx = 2000u;
    in = input(9000u); apply_manual(9030u, in);
    assert(last_velocity.wz == -3.0f && !chassis_yaw_hold_is_active());
    fake_lx = 1500u;
    in = input(9000u); apply_manual(9040u, in);
    assert(last_velocity.wz == 0.0f && !chassis_yaw_hold_is_active());
    /* Releasing VRB must stop motor output AND clear the cached velocity. */
    const unsigned before_stops = motor_stop_calls;
    fake_vrb = 1251u;
    in = input(9000u); update_control_source(9000u, in);
    assert(!in.healthy && (blockers & BLOCK_RC_SWITCH) && !rc_switches);
    assert(control_source.mode == DRIVE_SOURCE_MANUAL && !control_source.manual_armed);
    assert(control_source.release_seen && !chassis_yaw_hold_is_active());
    assert(last_velocity.vx == 0.0f && last_velocity.vy == 0.0f && last_velocity.wz == 0.0f);
    assert(manual_velocity.vx == 0.0f && motor_stop_calls == before_stops + 8u);
    assert(strstr(last_line, "vrb_required=1 vrb_ok=0"));
    detail_slot = 1u; log_detail(9000u, in);
    assert(strstr(last_line, "VRB=1251") && strstr(last_line, "vrb_required=1"));
    fake_vrb = 1250u; fake_ry = 1500u;
    in = input(9000u); update_control_source(9000u, in);
    assert(control_source.manual_armed); /* VRB + neutral/stopped re-arms without another SWD cycle. */
    fake_swd = 1000u;
    in = input(9000u); update_control_source(9000u, in);
    fake_swd = 2000u;
    in = input(9000u); update_control_source(9000u, in);
    assert(control_source.manual_armed);
    fake_ry = 2000u;
    in = input(9000u); apply_manual(9010u, in);
    assert(last_velocity.vx > 0.0f);
    receive_command("STOP\n",9000u,in);
    assert(!control_source.manual_armed && last_velocity.vx == 0.0f);
    update_control_source(9000u,in);
    assert(!control_source.manual_armed); /* Requires SWD off/on after explicit STOP. */
    fake_ry = 1500u; fake_swc = 1500u;
    in = input(9000u); update_control_source(9000u,in);
    receive_command("GO 1\n",9000u,in);
    assert(test.phase != ST_RUNNING); /* Prior owner cannot be resumed. */
    fake_vrb = 2000u;
    in = input(9000u); update_control_source(9000u, in);
    receive_command("ARM\n",9000u,in);
    receive_command("GO 2\n",9000u,in);
    assert(test.phase == ST_RUNNING && in.healthy); /* VRB cannot block HOST after manual protection. */
    assert(chassis_yaw_hold_is_active()); /* Returning to HOST reacquires heading hold. */
    check_round_heading("ARM ROUND 1\n", 0.6f);
    check_round_heading("ARM WALL 1\n", -0.6f);
    check_manual_direct_scheduler();
    check_trace_adapter();
    check_map_adapter();
    check_near_recovery_adapter();
    check_manual_logging_budget();
    check_map_laser_filter();
    check_map_logging_budget();
    check_map_mode_selection();
    puts("STRAIGHT_DIAGNOSTICS_TEST_OK (cached samples, calibration gate, stale data, drift wrap, log bounds)");
    puts("DRIVE_ISOLATION_TEST_OK (actual UART dispatch, manual VRB stop/cache clear/re-arm, manual stick-only rotation, HOST ignores VRB and retains heading hold, old commands and STOP)");
    puts("ROUND_HEADING_ADAPTER_TEST_OK (ROUND/WALL scheduler WAIT stops outputs but preserves nonzero heading; BACK corrects yaw; STOP disables)");
    return 0;
}
