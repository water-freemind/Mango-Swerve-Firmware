#include "commissioning.h"
#include "app_config.h"

#include "chassis_can_map.h"
#include "log.h"

#include <stddef.h>
#include <string.h>

volatile CommissioningSnapshot g_commissioning;

static bool s_initialized;
static uint32_t s_last_log_ms;
static uint8_t s_log_cursor[COMMISSIONING_BUS_COUNT];
static FDCAN_HandleTypeDef* const s_handles[COMMISSIONING_BUS_COUNT] = {
    &hfdcan1, &hfdcan2, &hfdcan3
};

static void commissioning_capture(FDCAN_HandleTypeDef* hcan,
                                   const FDCAN_RxHeaderTypeDef* header,
                                   const uint8_t data[8], void* user) {
    volatile CommissioningBus* bus = NULL;
    uint32_t b;
    uint32_t i;

    (void)user;
    for(b = 0u; b < COMMISSIONING_BUS_COUNT; ++b) {
        if(hcan == s_handles[b]) {
            bus = &g_commissioning.bus[b];
            break;
        }
    }
    if(bus == NULL || header == NULL || data == NULL) {
        return;
    }
    for(i = 0u; i < bus->id_count; ++i) {
        if(bus->frames[i].header.Identifier == header->Identifier &&
           bus->frames[i].header.IdType == header->IdType &&
           bus->frames[i].header.RxFrameType == header->RxFrameType) {
            break;
        }
    }
    if(i == COMMISSIONING_ID_SLOTS) {
        ++bus->table_full_count;
        return;
    }
    bus->frames[i].header = *header;
    for(b = 0u; b < 8u; ++b) {
        bus->frames[i].data[b] = data[b];
    }
    ++bus->frames[i].count;
    bus->frames[i].last_rx_ms = HAL_GetTick();
    if(i == bus->id_count) {
        ++bus->id_count;
    }
}

void commissioning_init(void) {
    BspCanStatus filter_status;
    BspCanStatus status;
    uint32_t b;

    if(s_initialized) {
        return;
    }
    s_initialized = true;
    memset((void*)&g_commissioning, 0, sizeof(g_commissioning));
    for(b = 0u; b < COMMISSIONING_BUS_COUNT; ++b) {
        g_commissioning.bus[b].start_status =
            can_register_rx_callback(s_handles[b], commissioning_capture, NULL);
    }

    filter_status = can_filter_init();
    for(b = 0u; b < 2u; ++b) {
        status = g_commissioning.bus[b].start_status;
        if(status == STM32_HAL_CAN_OK) {
            status = filter_status == STM32_HAL_CAN_OK
                ? can_start(s_handles[b]) : filter_status;
        }
        g_commissioning.bus[b].start_status = status;
        g_commissioning.bus[b].started = status == STM32_HAL_CAN_OK;
    }

    g_commissioning.can3_status = can3_bus_register_endpoint(0x601u, laser_accept_can3_frame, NULL);
    (void)can3_bus_observe_extended(true);
    if(g_commissioning.can3_status == CAN3_BUS_OK &&
       g_commissioning.bus[2].start_status == STM32_HAL_CAN_OK) {
        g_commissioning.can3_status = can3_bus_start();
    }
    g_commissioning.bus[2].started = can3_bus_is_started();

#if APP_RUN_MODE == APP_MODE_STEERING_ZERO_TEST
    log_info("ZERO_TEST: sequential steering return verification follows");
#elif APP_RUN_MODE == APP_MODE_STEERING_DIAGNOSTICS
    log_info("STEERING_DIAGNOSTICS: steering initialization enabled; CAN monitoring active");
#elif APP_RUN_MODE == APP_MODE_CAN2_QUERY
    log_info("STEERING_QUERY CAN%u: read ESC_ID register 8, standard ID 0x7ff, opcode 0x33, motor IDs 1..4",
             CHASSIS_STEER_CAN == &hfdcan1 ? 1u : 2u);
#else
    log_info("CAN_DIAGNOSTICS: application TX disabled; normal CAN mode provides ACK");
#endif
    log_info("UART1 921600 8N1; physical CAN1/2/3 watch: g_commissioning; configured swap=%u",
             (unsigned)CHASSIS_CAN_BUSES_SWAPPED);
#if APP_RUN_MODE == APP_MODE_STEERING_DIAGNOSTICS || APP_RUN_MODE == APP_MODE_STEERING_ZERO_TEST
    log_info("Steering initialization follows; chassis-only firmware");
#else
    log_info("Chassis-only firmware; passive IMU/remote/motor diagnostics");
#endif
}

#if APP_RUN_MODE == APP_MODE_CAN2_QUERY
/* Manufacturer read_motor_param format. This diagnostic only reads ESC_ID.
 * Reference: dmBots/motor-control-routine damiao.cpp, read_motor_param(). */
static void commissioning_query_steering(uint32_t now_ms) {
    static uint32_t last_query_ms;
    static uint8_t next_id = 1u;
    static bool awaiting_tx;
    if(awaiting_tx && (uint32_t)(now_ms - last_query_ms) >= 100u) {
        uint32_t psr = CHASSIS_STEER_CAN->Instance->PSR;
        uint32_t ecr = CHASSIS_STEER_CAN->Instance->ECR;
        uint32_t pending = CHASSIS_STEER_CAN->Instance->TXBRP;
        log_info("STEERING_QUERY motor=%u PSR=0x%lx ECR=0x%lx pending=0x%lx transmitted=0x%lx",
                 (unsigned)(next_id - 1u), (unsigned long)psr, (unsigned long)ecr,
                 (unsigned long)pending, (unsigned long)CHASSIS_STEER_CAN->Instance->TXBTO);
        if(pending != 0u) {
            (void)HAL_FDCAN_AbortTxRequest(CHASSIS_STEER_CAN, pending);
        }
        awaiting_tx = false;
    }
    if(next_id <= 4u && (uint32_t)(now_ms - last_query_ms) >= 2000u) {
        const uint8_t query[8] = {next_id, 0u, 0x33u, 8u, 0u, 0u, 0u, 0u};
        BspCanStatus status = can_send(CHASSIS_STEER_CAN, 0x7FFu, query, 8u);
        log_info("STEERING_QUERY motor=%u read_ESC_ID submit=%u", (unsigned)next_id, (unsigned)status);
        ++next_id;
        last_query_ms = now_ms;
        awaiting_tx = status == STM32_HAL_CAN_OK;
    }
}
#endif

static void commissioning_log_frame(uint32_t b) {
    CommissioningFrame frame;
    uint32_t primask;
    uint8_t index;

    primask = __get_PRIMASK();
    __disable_irq();
    if(g_commissioning.bus[b].id_count == 0u) {
        __set_PRIMASK(primask);
        return;
    }
    index = s_log_cursor[b];
    if(index >= g_commissioning.bus[b].id_count) {
        index = 0u;
    }
    frame = g_commissioning.bus[b].frames[index];
    s_log_cursor[b] = (uint8_t)(index + 1u);
    __set_PRIMASK(primask);

    log_info("CAN%u id=0x%lx type=%lu dlc=0x%lx n=%lu data=%02x %02x %02x %02x %02x %02x %02x %02x",
             (unsigned)(b + 1u), (unsigned long)frame.header.Identifier,
             (unsigned long)frame.header.IdType, (unsigned long)frame.header.DataLength,
             (unsigned long)frame.count, frame.data[0], frame.data[1], frame.data[2], frame.data[3],
             frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
}

void commissioning_process(void) {
    STM32HalCanDiagnostics diagnostics;
    Can3BusDiagnostics can3;
    LaserSnapshot laser;
    uint32_t now_ms;
    uint32_t b;

    for(b = 0u; b < COMMISSIONING_BUS_COUNT; ++b) {
        if(g_commissioning.bus[b].started) {
            can_poll_rx(s_handles[b]);
        }
    }
    now_ms = HAL_GetTick();
#if APP_RUN_MODE == APP_MODE_CAN2_QUERY
    commissioning_query_steering(now_ms);
#endif
    if((uint32_t)(now_ms - s_last_log_ms) < 1000u) {
        return;
    }
    s_last_log_ms = now_ms;
    g_commissioning.uptime_ms = now_ms;
    for(b = 0u; b < COMMISSIONING_BUS_COUNT; ++b) {
        if(can_get_diagnostics(s_handles[b], &diagnostics)) {
            g_commissioning.bus[b].diagnostics = diagnostics;
            log_info("CAN%u up=%u init=%u rx=%lu ids=%u tx_submit=%lu lost=%lu full=%lu BO=%u EP=%u HAL=0x%lx",
                     (unsigned)(b + 1u), (unsigned)g_commissioning.bus[b].started,
                     (unsigned)g_commissioning.bus[b].start_status,
                     (unsigned long)diagnostics.rx_count, (unsigned)g_commissioning.bus[b].id_count,
                     (unsigned long)diagnostics.tx_submit_count,
                     (unsigned long)diagnostics.rx_fifo0_lost_count,
                     (unsigned long)diagnostics.rx_fifo0_full_count,
                     (unsigned)diagnostics.bus_off, (unsigned)diagnostics.error_passive,
                     (unsigned long)diagnostics.hal_error_code);
        }
        commissioning_log_frame(b);
    }
    if(can3_bus_get_diagnostics(&can3)) {
        g_commissioning.can3 = can3;
        log_info("CAN3 status=%u routed=%lu unknown=%lu rejected=%lu",
                 (unsigned)g_commissioning.can3_status, (unsigned long)can3.routed_count,
                 (unsigned long)can3.unknown_count, (unsigned long)can3.rejected_count);
    }
    if(laser_get_snapshot(&laser)) {
        g_commissioning.laser = laser;
        log_info("LASER ports1..4=%u,%u,%u,%u mm frames=%lu age=%lu ms fresh=%u",
                 (unsigned)laser.distance_mm[0], (unsigned)laser.distance_mm[1],
                 (unsigned)laser.distance_mm[2], (unsigned)laser.distance_mm[3],
                 (unsigned long)laser.frame_count, (unsigned long)(now_ms - laser.last_rx_ms),
                 (unsigned)laser.frame_fresh);
    }
}
