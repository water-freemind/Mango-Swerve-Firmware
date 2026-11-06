#include "chassis.h"
#include "steering_calibration.h"
#include "log.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static float raw_position[4];
static float commanded_position[4];
static float commanded_speed[4];

static BusMotorStatus init_motor(const BusMotorConfig* config) {
    (void)config;
    return MOTOR_STATUS_OK;
}
static BusMotorStatus prepare_motor(uint16_t id) {
    (void)id;
    return MOTOR_STATUS_OK;
}
static BusMotorStatus feedback(uint16_t id, BusMotorFeedback* out) {
    if(out != NULL) {
        *out = (BusMotorFeedback){.id = id, .position = raw_position[id - 1u]};
    }
    return MOTOR_STATUS_OK;
}
static float position(uint16_t id) { return raw_position[id - 1u]; }
static float speed(uint16_t id) { (void)id; return 0.0f; }
static BusMotorStatus set_position(uint16_t id, float target, float limit) {
    assert(limit > 0.0f && fabsf(target) <= 12.4f);
    commanded_position[id - 1u] = target;
    return MOTOR_STATUS_OK;
}
static BusMotorStatus set_speed(uint16_t id, float target) {
    commanded_speed[id - 1u] = target;
    return MOTOR_STATUS_OK;
}
static bool send_frame(uint32_t id, const uint8_t* data, uint8_t len) {
    (void)id; (void)data; (void)len;
    return true;
}
LogStatus log_info(const char* format, ...) { (void)format; return LOG_STATUS_OK; }
LogStatus log_warn(const char* format, ...) { (void)format; return LOG_STATUS_OK; }
LogStatus log_error(const char* format, ...) { (void)format; return LOG_STATUS_OK; }
bool work_link_hold(void) { return false; }
void work_link_limit_velocity(float* vx, float* vy, float* wz) {
    (void)vx; (void)vy; (void)wz;
}

static const BusMotorInterface motor = {
    .init = init_motor, .update_feedback = feedback,
    .get_pos = position, .get_spd = speed,
    .set_pos_vel = set_position, .set_spd = set_speed,
    .brake = prepare_motor, .stop = prepare_motor,
};
static const BusMotorPortOps port = {.send = send_frame};

int main(void) {
    const float zero[4] = STEERING_FORWARD_ZERO_RAD;
    ChassisConfig config = {
        .steer_motor_interface = &motor, .drive_motor_interface = &motor,
        .steer_ops = &port, .drive_ops = &port,
        .prepare_steer_motor = prepare_motor, .prepare_drive_motor = prepare_motor,
        .model = {.length = 0.3f, .width = 0.3f, .wheel_radius = 0.035f, .max_wheel_linear_speed = 0.1f},
        .wheel_drive_ratio = 1.0f,
        .steer_target_mode = CHASSIS_STEER_TARGET_ABS_NEAREST,
        .steer_zero_rad = STEERING_FORWARD_ZERO_RAD,
        .yaw_bias = {.enabled = false, .k_vx = -0.049f, .k_vy = 0.055f, .v_deadband = 0.01f},
    };
    for(unsigned i = 0u; i < 4u; ++i) {
        raw_position[i] = zero[i] + (i == 1u ? 6.283185307f : 0.0f);
    }
    assert(chassis.init(&config) == chassis.OK);
    assert(chassis.set_velocity(0.1f, 0.0f, 0.0f) == chassis.OK);
    assert(chassis.process() == chassis.OK);
    assert(chassis.is_ready());
    assert(chassis.get_control()->wz == 0.0f);
    for(unsigned i = 0u; i < 4u; ++i) {
        assert(fabsf(commanded_position[i] - raw_position[i]) < 0.001f);
        assert(fabsf(chassis.get_state()->cur_wheels[i].steer_angle) < 0.001f);
        assert(fabsf(commanded_speed[i]) > 0.0f);
    }
    assert(chassis.set_velocity(0.0f, 0.1f, 0.0f) == chassis.OK);
    assert(chassis.process() == chassis.OK);
    for(unsigned i = 0u; i < 4u; ++i) {
        /* Equivalent reverse-drive headings must preserve a lateral wheel axis. */
        assert(fabsf(cosf(commanded_position[i] - zero[i])) < 0.001f);
    }
    assert(chassis.brake() == chassis.OK && chassis.process() == chassis.OK);
    for(unsigned i = 0u; i < 4u; ++i) {
        assert(fabsf(fabsf(sinf(commanded_position[i] - zero[i])) - 0.70710678f) < 0.001f);
    }
    config.steer_zero_rad[0] = NAN;
    assert(chassis.init(&config) == chassis.INVALID_PARAM);
    for(unsigned i = 0u; i < 4u; ++i) {
        config.steer_zero_rad[i] = 0.0f;
        raw_position[i] = 0.0f;
    }
    assert(chassis.init(&config) == chassis.OK);
    assert(chassis.set_velocity(0.1f, 0.0f, 0.0f) == chassis.OK);
    assert(chassis.process() == chassis.OK);
    for(unsigned i = 0u; i < 4u; ++i) {
        assert(fabsf(commanded_position[i]) < 0.001f);
    }
    puts("CHASSIS_ZERO_OFFSETS_TEST_OK");
    return 0;
}
