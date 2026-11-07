#include "dji_motor.h"

#include <stdbool.h>
#include <math.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define DJI_MOTOR_FEEDBACK_BASE_ID 0x200u
#define DJI_MOTOR_CTRL_FRAME_ID 0x200u
#define DJI_MOTOR_ENCODER_RANGE 8192
#define DJI_MOTOR_ENCODER_HALF 4096
#define DJI_MOTOR_RAD_PER_ROUND 6.28318530718f
#define DJI_MOTOR_RPM_TO_RAD_S (DJI_MOTOR_RAD_PER_ROUND / 60.0f)
#define DJI_MOTOR_RAD_S_TO_RPM (60.0f / DJI_MOTOR_RAD_PER_ROUND)

#define DJI_MOTOR_DEFAULT_KP 8.0f
#define DJI_MOTOR_DEFAULT_KI 0.1f
#define DJI_MOTOR_DEFAULT_KD 0.0f
#define DJI_MOTOR_DEFAULT_MAX_I 5000.0f

typedef struct {
    DjiMotorModel model;
    const char* name;
    float reduction_ratio;
    float max_current_command;
} DjiMotorModelParams;

typedef struct {
    float kp;
    float ki;
    float kd;
    float error;
    float last_error;
    float integral;
    float max_integral;
    float output;
    float max_output;
} DjiMotorPid;

typedef struct {
    uint16_t id;
    uint16_t angle;
    uint16_t last_angle;
    int16_t rpm;
    int16_t current;
    uint8_t temperature;
    int64_t total_tick;
    float target_speed;
    bool has_feedback;
    bool angle_initialized;
    bool control_clock_valid;
    uint32_t control_ms;
    uint32_t feedback_ms;
    BusMotorFeedback feedback;
    DjiMotorPid speed_pid;
} DjiMotorSlot;

static const DjiMotorModelParams s_model_params[] = {
    {
        .model = DJI_MOTOR_MODEL_M3508,
        .name = "M3508",
        .reduction_ratio = (3591.0f / 187.0f),
        .max_current_command = 16384.0f,
    },
    {
        .model = DJI_MOTOR_MODEL_M2006,
        .name = "M2006",
        .reduction_ratio = 36.0f,
        .max_current_command = 10000.0f,
    },
};

static const BusMotorPortOps* s_ops = 0;
static bool s_is_initialized = false;
static DjiMotorSlot s_slots[DJI_MOTOR_MAX_ID];
static uint8_t s_pending_current_mask = 0u;
static DjiMotorModel s_model = DJI_MOTOR_MODEL_M2006;
static const DjiMotorModelParams* s_active_model_params = 0;
static uint32_t s_feedback_timeout_ms = DJI_MOTOR_FEEDBACK_TIMEOUT_MS;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static BusMotorStatus dji_motor_init(const BusMotorConfig* config);
static const char* dji_motor_status_str(BusMotorStatus status);
static const char* dji_motor_mode_str(BusMotorMode mode);
static BusMotorStatus dji_motor_enable(uint16_t id);
static BusMotorStatus dji_motor_disable(uint16_t id);
static BusMotorStatus dji_motor_switch_mode(uint16_t id, BusMotorMode mode);
static BusMotorStatus dji_motor_set_pos(uint16_t id, float position);
static BusMotorStatus dji_motor_set_spd(uint16_t id, float speed);
static BusMotorStatus dji_motor_set_pos_vel(uint16_t id, float position, float speed);
static BusMotorStatus dji_motor_set_tor(uint16_t id, float torque);
static BusMotorStatus dji_motor_set_pd(uint16_t id, float kp, float kd);
static BusMotorStatus dji_motor_update_feedback(uint16_t id, BusMotorFeedback* feedback);
static float dji_motor_get_pos(uint16_t id);
static float dji_motor_get_spd(uint16_t id);
static float dji_motor_get_tor(uint16_t id);
static BusMotorStatus dji_motor_stop(uint16_t id);
static BusMotorStatus dji_motor_brake(uint16_t id);

static bool dji_motor_is_ready(void);
static const DjiMotorModelParams* dji_motor_get_model_params(DjiMotorModel model);
static DjiMotorSlot* dji_motor_get_slot(uint16_t id);
static const DjiMotorSlot* dji_motor_get_slot_const(uint16_t id);
static void dji_motor_reset_slot(DjiMotorSlot* slot,
                                 uint16_t id,
                                 const DjiMotorModelParams* model_params);
static BusMotorStatus dji_motor_send_all_current(void);
static BusMotorStatus dji_motor_update_pid(DjiMotorSlot* slot, uint32_t now_ms);
static void dji_motor_reset_control(DjiMotorSlot* slot);
static void dji_motor_clear_currents(void);
static BusMotorStatus dji_motor_fail(BusMotorStatus status);
static BusMotorStatus dji_motor_feedback_status(const DjiMotorSlot* slot);
static float dji_motor_limit(float value, float limit);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

const BusMotorInterface dji_motor_instance = {
    .init = dji_motor_init,
    .status_str = dji_motor_status_str,
    .mode_str = dji_motor_mode_str,
    .enable = dji_motor_enable,
    .disable = dji_motor_disable,
    .switch_mode = dji_motor_switch_mode,
    .set_pos = dji_motor_set_pos,
    .set_spd = dji_motor_set_spd,
    .set_pos_vel = dji_motor_set_pos_vel,
    .set_tor = dji_motor_set_tor,
    .set_pd = dji_motor_set_pd,
    .update_feedback = dji_motor_update_feedback,
    .get_pos = dji_motor_get_pos,
    .get_spd = dji_motor_get_spd,
    .get_tor = dji_motor_get_tor,
    .stop = dji_motor_stop,
    .brake = dji_motor_brake,
};

/**
 * @brief 初始化 DJI 电机实例
 */
static BusMotorStatus dji_motor_init(const BusMotorConfig* config) {
    const DjiMotorConfig* driver_config;
    const DjiMotorModelParams* model_params;
    DjiMotorModel model;
    uint16_t id;
    bool cleared = true;

    if(dji_motor_is_ready()) {
        const uint8_t zeros[DJI_MOTOR_CMD_LEN] = {0};
        dji_motor_clear_currents();
        cleared = s_ops->send(DJI_MOTOR_CTRL_FRAME_ID, zeros, DJI_MOTOR_CMD_LEN);
    }

    s_ops = 0;
    s_is_initialized = false;
    s_pending_current_mask = 0u;
    s_model = DJI_MOTOR_MODEL_M2006;
    s_active_model_params = 0;
    if(!cleared) return MOTOR_STATUS_PORT_ERROR;

    if(config == 0 || config->ops == 0 || config->timeout_ms > 1000u) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    if(config->ops->send == 0 || config->ops->now_ms == 0) {
        return MOTOR_STATUS_PORT_ERROR;
    }
    s_feedback_timeout_ms = config->timeout_ms != 0u ? config->timeout_ms : DJI_MOTOR_FEEDBACK_TIMEOUT_MS;

    driver_config = (const DjiMotorConfig*)config->driver_config;
    if(driver_config != 0) {
        model = driver_config->model;
    }
    else {
        /* Compatibility default for the current project until upper layers
         * pass DjiMotorConfig explicitly. */
        model = DJI_MOTOR_MODEL_M2006;
    }

    model_params = dji_motor_get_model_params(model);
    if(model_params == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    s_ops = config->ops;
    s_model = model;
    s_active_model_params = model_params;
    for(id = 1u; id <= DJI_MOTOR_MAX_ID; ++id) {
        dji_motor_reset_slot(&s_slots[id - 1u], id, s_active_model_params);
    }
    s_pending_current_mask = 0u;
    s_is_initialized = true;

    return MOTOR_STATUS_OK;
}

/**
 * @brief 将状态码转换为常量字符串
 */
static const char* dji_motor_status_str(BusMotorStatus status) {
    switch(status) {
#define X(name, value)        \
    case MOTOR_STATUS_##name: \
        return #name;
        MOTOR_STATUS_TABLE
#undef X
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief 将模式值转换为常量字符串
 */
static const char* dji_motor_mode_str(BusMotorMode mode) {
    switch((DjiMotorMode)mode) {
        case DJI_MOTOR_MODE_SPEED:
            return "SPEED";
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief 使能 DJI 电机
 */
static BusMotorStatus dji_motor_enable(uint16_t id) {
    if(dji_motor_is_ready() == false) {
        return MOTOR_STATUS_NOT_INITIALIZE;
    }
    if(dji_motor_get_slot(id) == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    return MOTOR_STATUS_OK;
}

/**
 * @brief 失能 DJI 电机
 */
static BusMotorStatus dji_motor_disable(uint16_t id) {
    DjiMotorSlot* slot;

    if(dji_motor_is_ready() == false) {
        return MOTOR_STATUS_NOT_INITIALIZE;
    }

    slot = dji_motor_get_slot(id);
    if(slot == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    dji_motor_reset_control(slot);
    return dji_motor_send_all_current();
}

/**
 * @brief 切换 DJI 电机模式
 */
static BusMotorStatus dji_motor_switch_mode(uint16_t id, BusMotorMode mode) {
    if(dji_motor_is_ready() == false) {
        return MOTOR_STATUS_NOT_INITIALIZE;
    }
    if(dji_motor_get_slot(id) == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    if((DjiMotorMode)mode != DJI_MOTOR_MODE_SPEED) {
        return MOTOR_STATUS_UNSUPPORTED;
    }

    return MOTOR_STATUS_OK;
}

/**
 * @brief DJI 电机当前只支持输出轴速度闭环入口
 */
static BusMotorStatus dji_motor_set_pos(uint16_t id, float position) {
    if(dji_motor_is_ready() == false) {
        return MOTOR_STATUS_NOT_INITIALIZE;
    }

    (void)id;
    (void)position;
    return MOTOR_STATUS_UNSUPPORTED;
}

/**
 * @brief 设定目标速度，单位 rad/s，内部转换为转子 rpm 做电流环输出
 */
static BusMotorStatus dji_motor_set_spd(uint16_t id, float speed) {
    DjiMotorSlot* slot;
    BusMotorStatus status;
    uint32_t now_ms;

    if(dji_motor_is_ready() == false) {
        return MOTOR_STATUS_NOT_INITIALIZE;
    }

    slot = dji_motor_get_slot(id);
    if(slot == 0) {
        return dji_motor_fail(MOTOR_STATUS_INVALID_PARAM);
    }
    /* Bound at the signed rotor-rpm feedback representation, not a claim about
     * rated mechanical speed. Check before the multiplication or integer cast. */
    if(!isfinite(speed) || fabsf(speed) > 32767.0f / (s_active_model_params->reduction_ratio * DJI_MOTOR_RAD_S_TO_RPM))
        return dji_motor_fail(MOTOR_STATUS_INVALID_PARAM);
    status = dji_motor_feedback_status(slot);
    if(status != MOTOR_STATUS_OK) return dji_motor_fail(status);
    now_ms = s_ops->now_ms();
    slot->target_speed = speed;
    status = dji_motor_update_pid(slot, now_ms);
    if(status != MOTOR_STATUS_OK) return dji_motor_fail(status);
    s_pending_current_mask |= (uint8_t)(1u << (id - 1u));
    if(s_pending_current_mask != ((1u << DJI_MOTOR_MAX_ID) - 1u)) {
        return MOTOR_STATUS_OK;
    }

    return dji_motor_send_all_current();
}

/**
 * @brief DJI 电机当前不直接支持位置和速度的复合控制入口
 */
static BusMotorStatus dji_motor_set_pos_vel(uint16_t id, float position, float speed) {
    if(dji_motor_is_ready() == false) {
        return MOTOR_STATUS_NOT_INITIALIZE;
    }

    (void)id;
    (void)position;
    (void)speed;
    return MOTOR_STATUS_UNSUPPORTED;
}

/**
 * @brief DJI 电机当前不直接支持通用扭矩入口
 */
static BusMotorStatus dji_motor_set_tor(uint16_t id, float torque) {
    if(dji_motor_is_ready() == false) {
        return MOTOR_STATUS_NOT_INITIALIZE;
    }

    (void)id;
    (void)torque;
    return MOTOR_STATUS_UNSUPPORTED;
}

/**
 * @brief 设定速度 PID 的 P/D 参数
 */
static BusMotorStatus dji_motor_set_pd(uint16_t id, float kp, float kd) {
    DjiMotorSlot* slot;

    if(dji_motor_is_ready() == false) {
        return MOTOR_STATUS_NOT_INITIALIZE;
    }

    slot = dji_motor_get_slot(id);
    if(slot == 0 || !isfinite(kp) || !isfinite(kd) || kp < 0.0f || kd < 0.0f) {
        return dji_motor_fail(MOTOR_STATUS_INVALID_PARAM);
    }

    slot->speed_pid.kp = kp;
    slot->speed_pid.kd = kd;
    return MOTOR_STATUS_OK;
}

/**
 * @brief 从反馈缓存读取最近反馈
 */
static BusMotorStatus dji_motor_update_feedback(uint16_t id, BusMotorFeedback* feedback) {
    const DjiMotorSlot* slot;
    BusMotorStatus status;

    if(feedback != 0) memset(feedback, 0, sizeof(*feedback));

    if(dji_motor_is_ready() == false) {
        return MOTOR_STATUS_NOT_INITIALIZE;
    }

    slot = dji_motor_get_slot_const(id);
    if(slot == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    status = dji_motor_feedback_status(slot);
    if(status != MOTOR_STATUS_OK) return status;

    if(feedback != 0) {
        *feedback = slot->feedback;
    }

    return MOTOR_STATUS_OK;
}

/**
 * @brief 获取最近位置反馈
 */
static float dji_motor_get_pos(uint16_t id) {
    const DjiMotorSlot* slot = dji_motor_get_slot_const(id);

    if(!dji_motor_is_ready() || slot == 0 || dji_motor_feedback_status(slot) != MOTOR_STATUS_OK) {
        return 0.0f;
    }

    return slot->feedback.position;
}

/**
 * @brief 获取最近速度反馈
 */
static float dji_motor_get_spd(uint16_t id) {
    const DjiMotorSlot* slot = dji_motor_get_slot_const(id);

    if(!dji_motor_is_ready() || slot == 0 || dji_motor_feedback_status(slot) != MOTOR_STATUS_OK) {
        return 0.0f;
    }

    return slot->feedback.speed;
}

/**
 * @brief 获取最近电流反馈
 */
static float dji_motor_get_tor(uint16_t id) {
    const DjiMotorSlot* slot = dji_motor_get_slot_const(id);

    if(!dji_motor_is_ready() || slot == 0 || dji_motor_feedback_status(slot) != MOTOR_STATUS_OK) {
        return 0.0f;
    }

    return slot->feedback.torque;
}

/**
 * @brief 停止 DJI 电机
 */
static BusMotorStatus dji_motor_stop(uint16_t id) {
    DjiMotorSlot* slot;
    if(!dji_motor_is_ready()) return MOTOR_STATUS_NOT_INITIALIZE;
    slot = dji_motor_get_slot(id);
    if(slot == 0) return dji_motor_fail(MOTOR_STATUS_INVALID_PARAM);
    dji_motor_reset_control(slot);
    if(dji_motor_feedback_status(slot) != MOTOR_STATUS_OK)
        return dji_motor_send_all_current();
    return dji_motor_set_spd(id, 0.0f);
}

/**
 * @brief 制动 DJI 电机
 */
static BusMotorStatus dji_motor_brake(uint16_t id) {
    return dji_motor_stop(id);
}

/**
 * @brief 解析一帧 DJI 电机反馈并刷新本地缓存
 */
BusMotorStatus dji_motor_parse_feedback_frame(uint32_t frame_id,
                                              const uint8_t data[DJI_MOTOR_CMD_LEN],
                                              BusMotorFeedback* feedback) {
    DjiMotorSlot* slot;
    float rotor_position_rad;
    float reduction_ratio;
    int16_t delta;
    uint16_t id;

    if(dji_motor_is_ready() == false) {
        return MOTOR_STATUS_NOT_INITIALIZE;
    }
    if(data == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    if(frame_id <= DJI_MOTOR_FEEDBACK_BASE_ID || frame_id > (DJI_MOTOR_FEEDBACK_BASE_ID + DJI_MOTOR_MAX_ID)) {
        return MOTOR_STATUS_ID_MISMATCH;
    }

    id = (uint16_t)(frame_id - DJI_MOTOR_FEEDBACK_BASE_ID);
    slot = dji_motor_get_slot(id);
    if(slot == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    slot->angle = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    if(slot->angle >= DJI_MOTOR_ENCODER_RANGE) {
        slot->has_feedback = false;
        return MOTOR_STATUS_INVALID_PARAM;
    }
    slot->rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    slot->current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    slot->temperature = data[6];

    if(slot->angle_initialized == false) {
        slot->last_angle = slot->angle;
        slot->angle_initialized = true;
    }

    delta = (int16_t)(slot->angle - slot->last_angle);
    if(delta > DJI_MOTOR_ENCODER_HALF) {
        delta -= DJI_MOTOR_ENCODER_RANGE;
    }
    else if(delta < -DJI_MOTOR_ENCODER_HALF) {
        delta += DJI_MOTOR_ENCODER_RANGE;
    }

    slot->total_tick += delta;
    slot->last_angle = slot->angle;
    slot->feedback.id = id;
    slot->feedback.error_code = slot->temperature;
    reduction_ratio = s_active_model_params->reduction_ratio;
    rotor_position_rad =
        ((float)slot->total_tick / (float)DJI_MOTOR_ENCODER_RANGE) * DJI_MOTOR_RAD_PER_ROUND;
    slot->feedback.position = rotor_position_rad / reduction_ratio;
    slot->feedback.speed =
        ((float)slot->rpm * DJI_MOTOR_RPM_TO_RAD_S) / reduction_ratio;
    slot->feedback.torque = (float)slot->current;
    slot->has_feedback = true;
    slot->feedback_ms = s_ops->now_ms();

    if(feedback != 0) {
        *feedback = slot->feedback;
    }

    return MOTOR_STATUS_OK;
}

/**
 * @brief 获取 DJI 电机当前型号
 */
DjiMotorModel dji_motor_get_model(void) {
    return s_model;
}

/**
 * @brief 将 DJI 电机型号转换为字符串
 */
const char* dji_motor_model_str(DjiMotorModel model) {
    const DjiMotorModelParams* model_params = dji_motor_get_model_params(model);

    if(model_params == 0) {
        return "UNKNOWN";
    }

    return model_params->name;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static bool dji_motor_is_ready(void) {
    return (s_is_initialized == true) && (s_ops != 0) && (s_active_model_params != 0);
}

static const DjiMotorModelParams* dji_motor_get_model_params(DjiMotorModel model) {
    uint32_t i;

    for(i = 0u; i < (uint32_t)(sizeof(s_model_params) / sizeof(s_model_params[0])); ++i) {
        if(s_model_params[i].model == model) {
            return &s_model_params[i];
        }
    }

    return 0;
}

static DjiMotorSlot* dji_motor_get_slot(uint16_t id) {
    if(id == 0u || id > DJI_MOTOR_MAX_ID) {
        return 0;
    }

    return &s_slots[id - 1u];
}

static const DjiMotorSlot* dji_motor_get_slot_const(uint16_t id) {
    if(id == 0u || id > DJI_MOTOR_MAX_ID) {
        return 0;
    }

    return &s_slots[id - 1u];
}

static void dji_motor_reset_slot(DjiMotorSlot* slot,
                                 uint16_t id,
                                 const DjiMotorModelParams* model_params) {
    if(slot == 0 || model_params == 0) {
        return;
    }

    memset(slot, 0, sizeof(*slot));
    slot->id = id;
    slot->feedback.id = id;
    slot->speed_pid.kp = DJI_MOTOR_DEFAULT_KP;
    slot->speed_pid.ki = DJI_MOTOR_DEFAULT_KI;
    slot->speed_pid.kd = DJI_MOTOR_DEFAULT_KD;
    slot->speed_pid.max_integral = DJI_MOTOR_DEFAULT_MAX_I;
    slot->speed_pid.max_output = model_params->max_current_command;
}

static BusMotorStatus dji_motor_send_all_current(void) {
    uint8_t data[DJI_MOTOR_CMD_LEN];
    uint16_t i;

    if(dji_motor_is_ready() == false) {
        return MOTOR_STATUS_NOT_INITIALIZE;
    }

    for(i = 0u; i < DJI_MOTOR_MAX_ID; ++i) {
        if(!isfinite(s_slots[i].speed_pid.output) || fabsf(s_slots[i].speed_pid.output) > s_active_model_params->max_current_command)
            return dji_motor_fail(MOTOR_STATUS_INVALID_PARAM);
        int16_t current = (int16_t)s_slots[i].speed_pid.output;
        data[i * 2u] = (uint8_t)(((uint16_t)current >> 8) & 0xFFu);
        data[i * 2u + 1u] = (uint8_t)((uint16_t)current & 0xFFu);
    }

    if(s_ops->send(DJI_MOTOR_CTRL_FRAME_ID, data, DJI_MOTOR_CMD_LEN) == false) {
        dji_motor_clear_currents();
        return MOTOR_STATUS_PORT_ERROR;
    }

    s_pending_current_mask = 0u;
    return MOTOR_STATUS_OK;
}

static BusMotorStatus dji_motor_update_pid(DjiMotorSlot* slot, uint32_t now_ms) {
    DjiMotorPid* pid;
    float target_rpm;
    float reduction_ratio;

    const uint32_t elapsed_ms = slot->control_clock_valid ? now_ms - slot->control_ms : DJI_MOTOR_PID_REFERENCE_MS;
    float scale, integral, derivative, output;
    if(elapsed_ms > DJI_MOTOR_CONTROL_MAX_GAP_MS) return MOTOR_STATUS_TIMEOUT;
    scale = (float)elapsed_ms / (float)DJI_MOTOR_PID_REFERENCE_MS;

    pid = &slot->speed_pid;
    reduction_ratio = s_active_model_params->reduction_ratio;
    target_rpm = slot->target_speed * reduction_ratio * DJI_MOTOR_RAD_S_TO_RPM;
    pid->error = target_rpm - (float)slot->rpm;
    integral = dji_motor_limit(pid->integral + pid->error * scale, pid->max_integral);
    derivative = elapsed_ms != 0u ? (pid->error - pid->last_error) / scale : 0.0f;
    output = pid->kp * pid->error + pid->ki * integral + pid->kd * derivative;
    if(!isfinite(output)) return MOTOR_STATUS_INVALID_PARAM;
    /* Do not accumulate further into current saturation. Allow unwinding. */
    if((output > pid->max_output && pid->error > 0.0f) || (output < -pid->max_output && pid->error < 0.0f)) {
        integral = pid->integral;
        output = pid->kp * pid->error + pid->ki * integral + pid->kd * derivative;
    }
    pid->integral = integral;
    pid->output = dji_motor_limit(output, pid->max_output);
    pid->last_error = pid->error;
    slot->control_ms = now_ms;
    slot->control_clock_valid = true;
    return MOTOR_STATUS_OK;
}

static BusMotorStatus dji_motor_feedback_status(const DjiMotorSlot* slot) {
    if(!slot->has_feedback) return MOTOR_STATUS_NO_FEEDBACK;
    /* Capture the received timestamp BEFORE polling the clock: a CAN interrupt
     * arriving after a caller's older 'now' must not look billions of ms old. */
    const uint32_t stamp = slot->feedback_ms;
    const uint32_t now_ms = s_ops->now_ms();
    if(now_ms - stamp > s_feedback_timeout_ms) return MOTOR_STATUS_TIMEOUT;
    return MOTOR_STATUS_OK;
}

static void dji_motor_reset_control(DjiMotorSlot* slot) {
    slot->target_speed = 0.0f;
    slot->speed_pid.error = slot->speed_pid.last_error = slot->speed_pid.integral = slot->speed_pid.output = 0.0f;
    slot->control_clock_valid = false;
}

static void dji_motor_clear_currents(void) {
    for(uint16_t i=0; i<DJI_MOTOR_MAX_ID; ++i) dji_motor_reset_control(&s_slots[i]);
    s_pending_current_mask = 0u;
}

static BusMotorStatus dji_motor_fail(BusMotorStatus status) {
    const uint8_t zeros[DJI_MOTOR_CMD_LEN] = {0};
    dji_motor_clear_currents();
    /* Attempt a complete zero frame; never flush partially updated targets. */
    if(!s_ops->send(DJI_MOTOR_CTRL_FRAME_ID, zeros, DJI_MOTOR_CMD_LEN)) return MOTOR_STATUS_PORT_ERROR;
    return status;
}

static float dji_motor_limit(float value, float limit) {
    if(value > limit) {
        return limit;
    }
    if(value < -limit) {
        return -limit;
    }

    return value;
}
