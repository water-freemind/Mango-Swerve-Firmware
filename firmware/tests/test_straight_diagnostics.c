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
static uint16_t fake_swd = 2000u, fake_vrb = 1500u;
static unsigned sample_reads;
static float fake_yaw;
static char last_line[1024];
FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
UART_HandleTypeDef huart1;
const struct ChassisInterface chassis_interface = {0}; /* Chassis outputs are never invoked here. */
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
    return true;
}
bool remote_is_online(uint32_t timeout) { (void)timeout; return fake_online; }
static float fake_speed(uint16_t id) { (void)id; return 0.0f; }
static float fake_position(uint16_t id) { return zeros[id - 1u]; }
static float fake_current(uint16_t id) { (void)id; return -16384.0f; }
static const BusMotorInterface fake_motor = {.get_spd = fake_speed, .get_pos = fake_position,
    .get_tor = fake_current};
const BusMotorInterface* steer_motor_instance = &fake_motor;
const BusMotorInterface* drive_motor_instance = &fake_motor;
bool can_get_diagnostics(FDCAN_HandleTypeDef* b, STM32HalCanDiagnostics* d) {
    (void)b;
    memset(d, 0xff, sizeof(*d)); /* Stress counter widths in the actual log formats. */
    return true;
}
bool laser_get_snapshot(LaserSnapshot* s) {
    *s = (LaserSnapshot){.distance_mm = {65535, 65535, 65535, 65535}, .frame_count = UINT32_MAX};
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
int main(void) {
    test.phase = ST_IDLE;
    test.target_m = 4.0f;
    test.body_edge_mm = 110u;
    fake_us = 1000000u;
    update_imu_sample();
    feedback(1000u);
    StraightInput in = input(1000u);
    assert(in.healthy && sample_reads == 1u);
    /* Mode 6 ignores the spring-return VRB wheel; SWD remains the stop gate. */
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
    for(unsigned i = 0; i < 15; ++i) log_detail(UINT32_MAX, in);
    test.plan = ST_PLAN_APPROACH;
    test.distance_m = test.outbound_m = 4.1f;
    test.right_ref_mm = 3000.0f;
    test.body_edge_mm = 1000u;
    log_plan(in);
    puts("STRAIGHT_DIAGNOSTICS_TEST_OK (cached samples, calibration gate, stale data, drift wrap, log bounds)");
    return 0;
}
