#include "steer_wheel_kine.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define sw steer_wheel_interface

#define SW_PI 3.14159265358979323846f
#define SW_2PI (2.0f * SW_PI)
#define SW_HALF_PI (0.5f * SW_PI)
#define SW_EPS 1e-6f

/**
 * @brief 舵轮运动学入口单例定义表
 */
#define X(name, str) .name = STEER_WHEEL_##name,
const struct SteerWheelInterface steer_wheel_interface = {
    {
        STEER_WHEEL_STATUS_TABLE
    },
    .init = steer_wheel_init,
    .fk = steer_wheel_fk,
    .ik = steer_wheel_ik,
    .error_code_to_str = steer_wheel_error_code_to_str
};
#undef X

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void sw_get_wheel_pos(const SteerWheelModel* model, float x[4], float y[4]);
static float sw_wrap_pi(float angle);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化舵轮运动学实例
 * @param steer_wheel 舵轮运动学实例指针
 * @param model 底盘模型参数
 * @return SteelWheelErrorCode 错误码
 */
bool steer_wheel_model_valid(const SteerWheelModel* model) {
    /* Store fractional bounds as float too: host x87 may otherwise compare a
     * stored float to an excess-precision literal differently from Cortex-M. */
    static const float min_span = 0.001f, min_radius = 0.0001f;
    return model != NULL &&
        isfinite(model->length) && model->length >= min_span && model->length <= 100.0f &&
        isfinite(model->width) && model->width >= min_span && model->width <= 100.0f &&
        isfinite(model->wheel_radius) && model->wheel_radius >= min_radius && model->wheel_radius <= 10.0f &&
        isfinite(model->max_wheel_linear_speed) &&
        model->max_wheel_linear_speed >= 0.0f && model->max_wheel_linear_speed <= 100.0f;
}

bool steer_wheel_twist_valid(float vx, float vy, float wz) {
    return isfinite(vx) && fabsf(vx) <= 100.0f &&
        isfinite(vy) && fabsf(vy) <= 100.0f &&
        isfinite(wz) && fabsf(wz) <= 1000.0f;
}

static SteelWheelErrorCode sw_fail(SteerWheel* wheel, SteelWheelErrorCode error) {
    /* Never leave a previous executable command after bad input. Keep the raw
     * feedback intact for diagnosis: the caller must handle this error. */
    memset(&wheel->control, 0, sizeof(wheel->control));
    wheel->state.cur_vx = wheel->state.cur_vy = wheel->state.cur_wz = 0.0f;
    return error;
}

static bool sw_angle_valid(float angle) {
    return isfinite(angle) && fabsf(angle) <= 1000000.0f;
}

SteelWheelErrorCode steer_wheel_init(SteerWheel* wheel, SteerWheelModel model) {
    if(wheel == NULL) return sw.INVALID_PARAM;
    memset(wheel, 0, sizeof(*wheel));
    if(!steer_wheel_model_valid(&model)) return sw.INVALID_MODEL;
    wheel->model = model;
    wheel->initialized = true;
    return sw.OK;
}

SteelWheelErrorCode steer_wheel_fk(SteerWheel* wheel) {
    if(wheel == NULL) return sw.INVALID_PARAM;
    if(!wheel->initialized) return sw_fail(wheel, sw.NOT_INITIALIZE);
    if(!steer_wheel_model_valid(&wheel->model)) return sw_fail(wheel, sw.INVALID_MODEL);
    float x[4], y[4];
    sw_get_wheel_pos(&wheel->model, x, y);
    float sum_vix = 0.0f, sum_viy = 0.0f, sum_w_numer = 0.0f, sum_w_denom = 0.0f;
    for(uint8_t i = 0; i < 4; ++i) {
        const float linear = wheel->state.cur_wheels[i].wheel_omega * wheel->model.wheel_radius;
        const float angle = wheel->state.cur_wheels[i].steer_angle;
        if(!isfinite(linear) || fabsf(linear) > 100.0f || !sw_angle_valid(angle))
            return sw_fail(wheel, sw.INVALID_PARAM);
        const float vix = linear * cosf(angle), viy = linear * sinf(angle);
        sum_vix += vix;
        sum_viy += viy;
        sum_w_numer += -y[i] * vix + x[i] * viy;
        sum_w_denom += x[i] * x[i] + y[i] * y[i];
    }
    const float vx = sum_vix * 0.25f, vy = sum_viy * 0.25f;
    const float wz = sum_w_numer / sum_w_denom;
    if(!steer_wheel_twist_valid(vx, vy, wz)) return sw_fail(wheel, sw.INVALID_PARAM);
    wheel->state.cur_vx = vx;
    wheel->state.cur_vy = vy;
    wheel->state.cur_wz = wz;
    return sw.OK;
}

SteelWheelErrorCode steer_wheel_ik(SteerWheel* wheel) {
    if(wheel == NULL) return sw.INVALID_PARAM;
    if(!wheel->initialized) return sw_fail(wheel, sw.NOT_INITIALIZE);
    if(!steer_wheel_model_valid(&wheel->model)) return sw_fail(wheel, sw.INVALID_MODEL);
    const float vx = wheel->control.vx, vy = wheel->control.vy, wz = wheel->control.wz;
    if(!steer_wheel_twist_valid(vx, vy, wz)) return sw_fail(wheel, sw.INVALID_PARAM);
    float x[4], y[4], linear[4];
    WheelModule output[4];
    sw_get_wheel_pos(&wheel->model, x, y);
    float maximum = 0.0f;
    for(uint8_t i = 0; i < 4; ++i) {
        if(!sw_angle_valid(wheel->state.cur_wheels[i].steer_angle)) return sw_fail(wheel, sw.INVALID_PARAM);
        const float vix = vx - wz * y[i], viy = vy + wz * x[i];
        linear[i] = hypotf(vix, viy);
        if(!isfinite(linear[i])) return sw_fail(wheel, sw.INVALID_PARAM);
        if(linear[i] < SW_EPS) {
            linear[i] = 0.0f;
            output[i].steer_angle = sw_wrap_pi(wheel->state.cur_wheels[i].steer_angle);
        } else output[i].steer_angle = atan2f(viy, vix);
        maximum = fmaxf(maximum, linear[i]);
    }
    const float limit = wheel->model.max_wheel_linear_speed;
    const float scale = limit > 0.0f && maximum > limit ? limit / maximum : 1.0f;
    for(uint8_t i = 0; i < 4; ++i) {
        const float bounded_linear = linear[i] * scale;
        if(!isfinite(bounded_linear) || bounded_linear > 100.0f) return sw_fail(wheel, sw.INVALID_PARAM);
        output[i].wheel_omega = bounded_linear / wheel->model.wheel_radius;
        if(!isfinite(output[i].wheel_omega)) return sw_fail(wheel, sw.INVALID_PARAM);
    }
    /* Commit all four targets together: no mixture of old and partial targets. */
    memcpy(wheel->control.wheels, output, sizeof(output));
    return sw.OK;
}

/**
 * @brief 舵轮运动学错误码转字符串
 * @param status 错误码
 * @return const char* 错误码字符串
 */
#define X(name, str) case STEER_WHEEL_##name: return str;
const char* steer_wheel_error_code_to_str(SteelWheelErrorCode status) {
    switch(status) {
        STEER_WHEEL_STATUS_TABLE
        default: return "UNKNOWN";
    }
}
#undef X

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 根据底盘模型计算四个轮模块相对底盘中心的位置
 * @param model 底盘模型参数
 * @param x 输出四个轮模块 x 坐标，单位 m，顺序为 FL、FR、RR、RL
 * @param y 输出四个轮模块 y 坐标，单位 m，顺序为 FL、FR、RR、RL
 */
static void sw_get_wheel_pos(const SteerWheelModel* model, float x[4], float y[4]) {
    const float hx = model->length * 0.5f;
    const float hy = model->width * 0.5f;

    /* 约定顺序: FL, FR, RR, RL */
    x[0] = hx;  y[0] = -hy;
    x[1] = hx;  y[1] = hy;
    x[2] = -hx;  y[2] = hy;
    x[3] = -hx;  y[3] = -hy;
}

/**
 * @brief 将角度归一化到 (-pi, pi]
 * @param angle 输入角度，单位 rad
 * @return float 归一化后的角度，单位 rad
 */
static float sw_wrap_pi(float angle) {
    if(!isfinite(angle)) {
        return 0.0f;
    }

    angle = fmodf(angle, SW_2PI);
    if(angle > SW_PI) {
        angle -= SW_2PI;
    }
    else if(angle <= -SW_PI) {
        angle += SW_2PI;
    }

    return angle;
}
