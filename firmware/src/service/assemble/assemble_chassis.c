#include "assemble.h"

#include "bus_motor/dji_motor.h"
#include "bus_motor/dm_motor.h"
#include "chassis.h"
#include "chassis_can_map.h"
#include "delay.h"
#include "log.h"
#include "stm32_hal_can.h"
#include "steering_calibration.h"
#include "app_config.h"
#include "straight_test.h"

#include <stddef.h>

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static bool chassis_steer_can_send(uint32_t id, const uint8_t* data, uint8_t len);
static bool chassis_drive_can_send(uint32_t id, const uint8_t* data, uint8_t len);
static BusMotorStatus chassis_prepare_steer_motor(uint16_t id);
static BusMotorStatus chassis_prepare_drive_motor(uint16_t id);
static void chassis_steer_can_rx_callback(FDCAN_HandleTypeDef* hcan, const FDCAN_RxHeaderTypeDef* header, const uint8_t data[8], void* user);
static void chassis_drive_can_rx_callback(FDCAN_HandleTypeDef* hcan, const FDCAN_RxHeaderTypeDef* header, const uint8_t data[8], void* user);

// ! ========================= 变 量 声 明 ========================= ! //

static const BusMotorPortOps chassis_steer_motor_ops = {
    .send = chassis_steer_can_send,
    .now_ms = HAL_GetTick,
    .delay_ms = delay_ms,
};

static const BusMotorPortOps chassis_drive_motor_ops = {
    .send = chassis_drive_can_send,
    .now_ms = HAL_GetTick,
    .delay_ms = delay_ms,
};

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_chassis(void) {
    ChassisConfig chassis_config = {
        .steer_motor_interface = &dm_motor_instance,
        .drive_motor_interface = &dji_motor_instance,
        .steer_ops = &chassis_steer_motor_ops,
        .drive_ops = &chassis_drive_motor_ops,
        .prepare_steer_motor = chassis_prepare_steer_motor,
        .prepare_drive_motor = chassis_prepare_drive_motor,
        .model = {
            .length = 0.26572986916f,
            .width = 0.26572986916f,
            .wheel_radius = 0.035276f,
            .max_wheel_linear_speed = 2.0f },
        .wheel_drive_ratio = 1.0f,
        .steer_target_mode = CHASSIS_STEER_TARGET_ABS_NEAREST,
        .steer_zero_rad = STEERING_FORWARD_ZERO_RAD,
        .yaw_bias = {
            /* Historical chassis fit. Recalibrate after wheel/geometry checks. */
            .enabled = false,
            .k_vx = -0.049f,
            .k_vy = 0.055f,
            .v_deadband = 0.01f,
        }
    };

#if APP_RUN_MODE == APP_MODE_STRAIGHT_TEST
    /* Nominal 102 mm wheel; verify physical distance before claiming 2 m accuracy. */
    chassis_config.model.wheel_radius = STRAIGHT_TEST_WHEEL_RADIUS_M;
    chassis_config.model.max_wheel_linear_speed = 0.10f;
#endif
    log_info("CHASSIS assemble begin");
    if(can_register_rx_callback(CHASSIS_STEER_CAN, chassis_steer_can_rx_callback, NULL) != STM32_HAL_CAN_OK)
        return SYSTEM_STATUS_ERROR;
    if(can_register_rx_callback(CHASSIS_DRIVE_CAN, chassis_drive_can_rx_callback, NULL) != STM32_HAL_CAN_OK)
        return SYSTEM_STATUS_ERROR;

    if(can_filter_init() != STM32_HAL_CAN_OK)
        return SYSTEM_STATUS_ERROR;
    log_info("CHASSIS CAN filter ok");
    if(can_start(&hfdcan1) != STM32_HAL_CAN_OK)
        return SYSTEM_STATUS_ERROR;
    log_info("CHASSIS FDCAN1 start ok");
    if(can_start(&hfdcan2) != STM32_HAL_CAN_OK)
        return SYSTEM_STATUS_ERROR;
    log_info("CHASSIS FDCAN2 start ok");

    delay_ms(100);

    if(chassis_init(&chassis_config) != chassis.OK) {
        return SYSTEM_STATUS_ERROR;
    }

    log_info("CHASSIS service init ok");
    return SYSTEM_STATUS_OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

SystemStatus assemble_steering_diagnostics(void) {
    const BusMotorConfig config = { .ops = &chassis_steer_motor_ops };
    uint16_t id;

    log_info("STEERING_DIAGNOSTICS: restoring steering initialization, IDs 1..4, CAN%u",
             CHASSIS_STEER_CAN == &hfdcan1 ? 1u : 2u);
    if(steer_motor_set_instance(&dm_motor_instance) != MOTOR_STATUS_OK ||
       dm_motor_instance.init(&config) != MOTOR_STATUS_OK ||
       can_register_rx_callback(CHASSIS_STEER_CAN, chassis_steer_can_rx_callback, NULL) != STM32_HAL_CAN_OK) {
        log_error("STEERING_DIAGNOSTICS driver initialization failed");
        return SYSTEM_STATUS_ERROR;
    }
    for(id = 1u; id <= 4u; ++id) {
        BusMotorFeedback feedback;
        log_info("STEERING_INIT id=%u begin", (unsigned)id);
        if(chassis_prepare_steer_motor(id) != MOTOR_STATUS_OK) {
            log_error("STEERING_INIT id=%u command transmission failed", (unsigned)id);
            return SYSTEM_STATUS_ERROR;
        }
        can_poll_rx(CHASSIS_STEER_CAN);
        if(dm_motor_instance.update_feedback(id, &feedback) == MOTOR_STATUS_OK) {
            log_info("STEERING_INIT id=%u feedback state=%u pos=%f speed=%f torque=%f",
                     (unsigned)id, (unsigned)feedback.error_code,
                     (double)feedback.position, (double)feedback.speed, (double)feedback.torque);
        } else {
            log_warn("STEERING_INIT id=%u commands submitted; feedback pending", (unsigned)id);
        }
    }
    log_info("STEERING_DIAGNOSTICS initialization finished; CAN monitoring active");
    return SYSTEM_STATUS_OK;
}

static bool chassis_steer_can_send(uint32_t id, const uint8_t* data, uint8_t len) {
    return can_send(CHASSIS_STEER_CAN, id, data, len) == STM32_HAL_CAN_OK;
}

static bool chassis_drive_can_send(uint32_t id, const uint8_t* data, uint8_t len) {
    return can_send(CHASSIS_DRIVE_CAN, id, data, len) == STM32_HAL_CAN_OK;
}

static BusMotorStatus chassis_prepare_steer_motor(uint16_t id) {
#if APP_RUN_MODE == APP_MODE_STRAIGHT_TEST
    /* Both motor drivers are initialized here. Clear the complete drive group first. */
    for(uint16_t drive_id = 1u; drive_id <= 4u; ++drive_id)
        if(drive_motor.stop(drive_id) != MOTOR_STATUS_OK) return MOTOR_STATUS_PORT_ERROR;
#endif
    if(dm_motor_clear_error(id) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }
    delay_ms(100);
    if(steer_motor.enable(id) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }
    delay_ms(100);
    if(steer_motor.switch_mode(id, DM_MOTOR_MODE_POS_VEL) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }
#if APP_RUN_MODE == APP_MODE_STRAIGHT_TEST
    /* Do not apply the driver's initial raw position=0 while waiting for GO. */
    BusMotorFeedback current;
    can_poll_rx(CHASSIS_STEER_CAN);
    if(steer_motor.update_feedback(id, &current) != MOTOR_STATUS_OK) {
        (void)steer_motor.disable(id);
        return MOTOR_STATUS_NO_FEEDBACK;
    }
#else
    if(steer_motor.set_spd(id, 0.0f) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }
    if(steer_motor.set_tor(id, 0.0f) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }
#endif
    if(steer_motor.brake(id) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }

    delay_ms(100);
    return MOTOR_STATUS_OK;
}

static BusMotorStatus chassis_prepare_drive_motor(uint16_t id) {
    if(drive_motor.enable(id) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }
    if(drive_motor.switch_mode(id, DJI_MOTOR_MODE_SPEED) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }
    if(drive_motor.stop(id) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }

    return MOTOR_STATUS_OK;
}

static void chassis_steer_can_rx_callback(FDCAN_HandleTypeDef* hcan,
                                          const FDCAN_RxHeaderTypeDef* header,
                                          const uint8_t data[8],
                                          void* user) {
    (void)hcan;
    (void)user;

    if(header == NULL) {
        return;
    }

    (void)dm_motor_parse_feedback_frame(header->Identifier, data, NULL);
}

static void chassis_drive_can_rx_callback(FDCAN_HandleTypeDef* hcan,
                                          const FDCAN_RxHeaderTypeDef* header,
                                          const uint8_t data[8],
                                          void* user) {
    (void)hcan;
    (void)user;

    if(header == NULL) {
        return;
    }

    (void)dji_motor_parse_feedback_frame(header->Identifier, data, NULL);
}
