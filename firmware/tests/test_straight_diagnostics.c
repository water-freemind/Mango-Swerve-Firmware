/* Exercise the actual mode-6 sensor adapter; no serial/CAN hardware is opened. */
typedef struct { unsigned unused; } UART_HandleTypeDef;
#include "../src/app/straight_test_app.c"
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

uint32_t g_test_tick_ms;
static uint32_t fake_us;
static bool fake_online = true, fake_calibrated = true, fresh_flag;
static bool fake_can_healthy, fake_laser_ready;
static uint16_t fake_swd = 2000u, fake_vrb = 1500u;
static uint16_t fake_swc = 1500u, fake_ry = 1500u, fake_lx = 1500u;
static uint16_t fake_swb = 1500u, fake_rx = 1500u;
static float fake_wheel_speed;
static bool fake_drive_gate = true;
bool chassis_drive_output_allowed(void) { return fake_drive_gate; }
static DriveVelocity last_velocity;
static unsigned sample_reads;
static unsigned motor_stop_calls;
static unsigned chassis_process_calls;
static float fake_yaw;
static char last_line[1024];
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
LogStatus log_vofa_write(const char* name, uint32_t n, const LogVofaValue* v) {
    (void)name; (void)n; (void)v; return LOG_STATUS_OK;
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
    (void)id; *f = (BusMotorFeedback){.position = 0.0f}; return MOTOR_STATUS_OK;
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
    *s = (LaserSnapshot){.distance_mm = {65535, 65535, 65535, 65535}, .frame_count = UINT32_MAX};
    if(fake_laser_ready) *s = (LaserSnapshot){.distance_mm = {800, 600, 7000, 7000},
        .frame_fresh = true, .frame_count = g_test_tick_ms, .last_rx_ms = g_test_tick_ms};
    return true;
}
LogStatus log_info(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    const int n = vsnprintf(last_line, sizeof(last_line), fmt, args);
    va_end(args);
    if(n < 0 || (unsigned)n + 18u >= LOG_BUFFER_SIZE) {
        fprintf(stderr, "Log exceeds production buffer (%d) %s\n", n, last_line);
        assert(0);
    }
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
    in = input(start + 2500u);
    receive_command("STOP\n", start + 2500u, in);
    assert(!chassis_yaw_hold_is_active() && last_velocity.vx == 0.0f);
}
static void check_manual_ramp_scheduler(void) {
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
        assert(fabsf(manual_velocity.vx - 0.2f) < 0.00001f && manual_target.vx == 1.5f);
        assert(!chassis_yaw_hold_is_active() && manual_velocity.wz == 0);
        fake_drive_gate = false;
        for(unsigned i = 0; i < 20; ++i) {
            loop_at(now += 10u);
            assert(manual_velocity.vx <= 0.002001f);
        }
        fake_drive_gate = true; loop_at(now += 10u);
        assert(manual_velocity.vx <= 0.004001f);
        /* A safety stop takes effect immediately, not at the next 10 ms ramp step. */
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
        assert(manual_ramp.output.vx == 0 && manual_ramp.last_motion.vx == 0 && !manual_ramp.reversing);
        assert(manual_target.vx == 0 && manual_velocity.vx == 0);
        /* Recovering permit/fault alone cannot resume a previous command. */
        fake_can_healthy = fake_online = true; fake_swc = 2000u; fake_swd = 2000u; fake_vrb = 1250u;
        loop_at(now += 10u); assert(!control_source.manual_armed);
        fake_ry = 1500u; fake_swd = 1000u; loop_at(now += 10u);
        fake_swd = 2000u; loop_at(now += 10u);
        assert(control_source.manual_armed && manual_velocity.vx == 0);
        fake_ry = 2000u; loop_at(now += 10u);
        assert(fabsf(manual_velocity.vx - 0.002f) < 0.000001f);
        /* Reverse waits for actual wheel-stop feedback after output reaches zero. */
        fake_wheel_speed = 0.5f; fake_ry = 1000u;
        for(unsigned i = 0; i < 20; ++i) { loop_at(now += 10u); assert(manual_velocity.vx == 0); }
        assert(manual_ramp.reversing);
        fake_wheel_speed = 0; loop_at(now += 10u);
        assert(manual_velocity.vx == 0 && !manual_ramp.reversing);
        loop_at(now += 10u); assert(manual_velocity.vx < 0);
        fake_swd = 1000u; loop_at(now += 1u);
        now += 100u;
    }
    puts("MANUAL_RAMP_SCHEDULER_TEST_OK (full throttle, stick-only yaw, SWD/VRB/RC/source/CAN/timing/STOP immediate reset, re-arm and feedback-gated reverse)");
}
int main(void) {
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
    fake_ry = 2000u;
    in = input(9000u); assert(in.healthy && !rc_centered); /* Joystick movement is valid only for manual. */
    receive_command("ARM\nGO 1\nHB 1\nSPEED 0.50\n",9000u,in);
    assert(test.phase == ST_FAULT && fabsf(test.max_speed_mps - 0.10f) < 0.00001f && control_source.manual_armed);
    rx_fault = true;
    in = input(9000u); assert(in.healthy); /* A serial error cannot remove manual permit. */
    commands(9000u,in);
    assert(control_source.manual_armed);
    chassis_yaw_hold_set_target(fake_yaw); /* A stale HOST reference cannot affect manual. */
    fake_yaw += 0.2f;
    apply_manual(9010u, in);
    assert(fabsf(last_velocity.vx - 0.002f) < 0.00001f && last_velocity.vy == 0.0f);
    assert(last_velocity.wz == 0.0f && !chassis_yaw_hold_is_active());
    fake_yaw -= 0.4f;
    apply_manual(9020u, in);
    assert(last_velocity.wz == 0.0f); /* Yaw/gyro drift cannot create a manual turn. */
    fake_lx = 2000u;
    in = input(9000u); apply_manual(9030u, in);
    assert(fabsf(last_velocity.wz + 0.004f) < 0.00001f && !chassis_yaw_hold_is_active());
    fake_lx = 1500u;
    in = input(9000u); apply_manual(9040u, in);
    assert(last_velocity.wz == 0.0f);
    /* Releasing VRB must stop motor output AND clear the cached velocity. */
    const unsigned before_stops = motor_stop_calls;
    fake_vrb = 1251u;
    in = input(9000u); update_control_source(9000u, in);
    assert(!in.healthy && (blockers & BLOCK_RC_SWITCH) && !rc_switches);
    assert(control_source.mode == DRIVE_SOURCE_MANUAL && !control_source.manual_armed);
    assert(!control_source.release_seen && !chassis_yaw_hold_is_active());
    assert(last_velocity.vx == 0.0f && last_velocity.vy == 0.0f && last_velocity.wz == 0.0f);
    assert(manual_velocity.vx == 0.0f && motor_stop_calls == before_stops + 8u);
    assert(strstr(last_line, "vrb_required=1 vrb_ok=0"));
    detail_slot = 1u; log_detail(9000u, in);
    assert(strstr(last_line, "VRB=1251") && strstr(last_line, "vrb_required=1"));
    fake_vrb = 1250u; fake_ry = 1500u;
    in = input(9000u); update_control_source(9000u, in);
    assert(!control_source.manual_armed); /* VRB alone does not re-arm. */
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
    check_manual_ramp_scheduler();
    puts("STRAIGHT_DIAGNOSTICS_TEST_OK (cached samples, calibration gate, stale data, drift wrap, log bounds)");
    puts("DRIVE_ISOLATION_TEST_OK (actual UART dispatch, manual VRB stop/cache clear/re-arm, manual stick-only rotation, HOST ignores VRB and retains heading hold, old commands and STOP)");
    puts("ROUND_HEADING_ADAPTER_TEST_OK (ROUND/WALL scheduler WAIT stops outputs but preserves nonzero heading; BACK corrects yaw; STOP disables)");
    return 0;
}
