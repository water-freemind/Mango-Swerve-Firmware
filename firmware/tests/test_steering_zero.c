#include "steering_zero_test.h"
#include "assemble/assemble.h"
#include "bus_motor/dm_motor.h"
#include "commissioning.h"
#include "log.h"
#include "steering_calibration.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

uint32_t g_test_tick_ms;
FDCAN_GlobalTypeDef g_fake_can_regs[3];
FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
volatile CommissioningSnapshot g_commissioning;
static BusMotorFeedback s_feedback[4];
static float s_last_command[4];
static bool s_enabled[4], s_stale, s_tracking_fault, s_send_failure;
static unsigned s_commands, s_reached;

static void publish(unsigned i) {
    ++g_commissioning.bus[0].frames[i].count;
    g_commissioning.bus[0].frames[i].last_rx_ms = g_test_tick_ms;
}
static BusMotorStatus enable_motor(uint16_t id) {
    for(unsigned i = 0u; i < 4u; ++i) assert(!s_enabled[i]);
    s_enabled[id - 1u] = true;
    if(!s_stale) publish(id - 1u);
    return MOTOR_STATUS_OK;
}
static BusMotorStatus disable_motor(uint16_t id) {
    s_enabled[id - 1u] = false;
    return MOTOR_STATUS_OK;
}
static BusMotorStatus update_motor(uint16_t id, BusMotorFeedback* out) {
    *out = s_feedback[id - 1u];
    return MOTOR_STATUS_OK;
}
static BusMotorStatus command_motor(uint16_t id, float position, float speed) {
    const unsigned i = id - 1u;
    assert(s_enabled[i] && fabsf(position - s_last_command[i]) < 0.00401f);
    assert(fabsf(speed - 0.2f) < 0.000001f);
    s_last_command[i] = position;
    ++s_commands;
    if(s_send_failure) return MOTOR_STATUS_PORT_ERROR;
    if(!s_tracking_fault) s_feedback[i].position = position;
    if(!s_stale) publish(i);
    return MOTOR_STATUS_OK;
}
const BusMotorInterface dm_motor_instance = {
    .enable = enable_motor, .disable = disable_motor,
    .update_feedback = update_motor, .set_pos_vel = command_motor
};
SystemStatus assemble_steering_diagnostics(void) {
    static const float zero[4] = STEERING_FORWARD_ZERO_RAD;
    memset((void*)&g_commissioning, 0, sizeof(g_commissioning));
    g_commissioning.bus[0].id_count = 4u;
    for(unsigned i = 0u; i < 4u; ++i) {
        g_commissioning.bus[0].frames[i].header = (FDCAN_RxHeaderTypeDef){
            .Identifier = 0x11u + i, .DataLength = FDCAN_DLC_BYTES_8
        };
        g_commissioning.bus[0].frames[i].data[0] = (uint8_t)(0x11u + i);
        /* The nearest equivalent zero crosses the +/-pi branch for wheel 2. */
        s_feedback[i] = (BusMotorFeedback){.id = (uint16_t)(i + 1u), .error_code = 1u,
            .position = zero[i] + 0.4f + (i == 1u ? 6.283185307f : 0.0f)};
        s_last_command[i] = s_feedback[i].position;
        publish(i);
    }
    return SYSTEM_STATUS_OK;
}
LogStatus log_info(const char* format, ...) {
    if(strstr(format, "ZERO_TEST reached")) ++s_reached;
    return LOG_STATUS_OK;
}
LogStatus log_error(const char* format, ...) { (void)format; return LOG_STATUS_OK; }

static void run_case(bool stale, bool tracking, bool send_failure, unsigned expected) {
    s_stale = stale;
    s_tracking_fault = tracking;
    s_send_failure = send_failure;
    s_commands = s_reached = 0u;
    g_test_tick_ms = 0xfffff000u;
    memset(s_enabled, 0, sizeof(s_enabled));
    steering_zero_test_init();
    for(unsigned tick = 0u; tick < 6000u; ++tick) {
        g_test_tick_ms += 20u;
        steering_zero_test_process();
    }
    assert(s_reached == expected);
    for(unsigned i = 0u; i < 4u; ++i) assert(!s_enabled[i]);
    if(stale) assert(s_commands == 0u);
    if(expected == 4u) {
        assert(fabsf(s_feedback[1].position - 3.1514618f) < 0.001f);
    }
}
int main(void) {
    run_case(false, false, false, 4u);
    run_case(true, false, false, 0u);
    run_case(false, true, false, 0u);
    run_case(false, false, true, 0u);
    puts("STEERING_ZERO_TEST_OK");
    return 0;
}
