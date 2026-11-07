/* Test the real assembly policy and attitude integrator with a stationary biased gyro. */
typedef struct { unsigned unused; } SPI_HandleTypeDef;
#include "../src/service/assemble/assemble_imu.c"
#include "imu/imu_attitude.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static Bmi088Config captured;
static ImuStatus capture_init(const void* config) { captured = *(const Bmi088Config*)config; return IMU_STATUS_OK; }
const ImuInterface bmi088_blocking_instance = {.init = capture_init};
static const Bmi088PortOps fake_ops = {0};
const Bmi088PortOps* stm32_bmi088_get_ops(void) { return &fake_ops; }
uint16_t stm32_bmi088_get_accel_int_pin(void) { return 0; }
uint16_t stm32_bmi088_get_gyro_int_pin(void) { return 0; }
void stm32_bmi088_register_callbacks(void) {}
Bmi088Error bmi088_get_init_error(void) { return BMI088_ERROR_NO_ERROR; }
const char* bmi088_error_str(Bmi088Error e) { (void)e; return "OK"; }
LogStatus log_error(const char* fmt, ...) { (void)fmt; return LOG_STATUS_OK; }
ImuStatus bmi088_make_config(Bmi088Config* c, const Bmi088PortOps* ops, uint16_t a, uint16_t g) {
    (void)a; (void)g;
    memset(c, 0, sizeof(*c));
    c->ops = ops;
    c->attitude = (ImuAttitudeConfig){.mode = IMU_ATTITUDE_MAHONY_6AXIS, .gyro_calib_samples = 2000u,
        .acc_norm = 9.80665f, .acc_norm_tolerance = 1.5f, .max_acc_age_us = 20000u,
        .gyro_calib_var_threshold = 0.01f, .mahony_kp = 2.0f,
        .gyro_z_temp_coeff = 0.000038f, .gyro_z_bias_offset = 0.00090f};
    return IMU_STATUS_OK;
}
int main(void) {
    assert(assemble_imu() == SYSTEM_STATUS_OK);
#if APP_RUN_MODE == APP_MODE_STRAIGHT_TEST
    assert(captured.attitude.gyro_z_temp_coeff == 0.0f && captured.attitude.gyro_z_bias_offset == 0.0f);
#else
    assert(fabsf(captured.attitude.gyro_z_temp_coeff - 0.000038f) < 1.0e-10f);
    assert(fabsf(captured.attitude.gyro_z_bias_offset - 0.00090f) < 1.0e-9f);
#endif
    ImuAttitude attitude;
    assert(imu_attitude_init(&attitude, &captured.attitude) == IMU_ATTITUDE_STATUS_OK);
    ImuSample sample = {.acc = {0, 0, 9.80665f}, .gyro = {0, 0, -0.00145f}, .temperature = 33.0f,
        .flags = IMU_SAMPLE_GYRO_NEW | IMU_SAMPLE_GYRO_VALID | IMU_SAMPLE_ACC_VALID | IMU_SAMPLE_TEMP_VALID};
    for(unsigned i = 0; i < 2000u; ++i) {
        sample.gyro_timestamp_us += 4000u;
        sample.acc_timestamp_us = sample.gyro_timestamp_us;
        assert(imu_attitude_update(&attitude, &sample) == IMU_ATTITUDE_STATUS_CALIBRATING);
    }
    assert(attitude.calibrated);
    const float before = attitude.angle.yaw;
    for(unsigned i = 0; i < 7500u; ++i) {
        sample.gyro_timestamp_us += 4000u;
        sample.acc_timestamp_us = sample.gyro_timestamp_us;
        assert(imu_attitude_update(&attitude, &sample) == IMU_ATTITUDE_STATUS_OK);
    }
    const float change = remainderf(attitude.angle.yaw - before, 6.283185307f);
#if APP_RUN_MODE == APP_MODE_STRAIGHT_TEST
    assert(fabsf(change) < 0.0001f);
    assert(fabsf(attitude.gyro_filtered.z) < 1.0e-6f);
    puts("IMU_BOOT_BIAS_TEST_OK (mode 6 measured startup mean, 30s stationary simulation)");
#else
    assert(fabsf(change) > 0.09f);
    puts("IMU_LEGACY_POLICY_TEST_OK (other modes keep existing temperature model)");
#endif
    return 0;
}
