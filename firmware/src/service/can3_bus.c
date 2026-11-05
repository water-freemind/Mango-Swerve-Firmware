#include "can3_bus.h"

#include "fdcan.h"

#include <stddef.h>
#include <string.h>

static Can3Router s_router;
static bool s_initialized;
static bool s_started;
static bool s_observe_extended;

static void can3_bus_init_once(void);
static void can3_bus_rx_callback(FDCAN_HandleTypeDef* hcan,
                                 const FDCAN_RxHeaderTypeDef* header,
                                 const uint8_t data[8],
                                 void* user);
static bool can3_bus_dlc_to_len(uint32_t dlc, uint8_t* len);

Can3BusStatus can3_bus_register_endpoint(uint16_t std_id,
                                         Can3EndpointCallback callback,
                                         void* user) {
    Can3RouterStatus status;

    can3_bus_init_once();
    status = can3_router_register(&s_router, std_id, callback, user);
    switch(status) {
        case CAN3_ROUTER_OK: return CAN3_BUS_OK;
        case CAN3_ROUTER_DUPLICATE_ID: return CAN3_BUS_DUPLICATE_ID;
        case CAN3_ROUTER_FULL: return CAN3_BUS_ENDPOINT_FULL;
        case CAN3_ROUTER_LOCKED: return CAN3_BUS_ALREADY_STARTED;
        default: return CAN3_BUS_INVALID_PARAM;
    }
}

Can3BusStatus can3_bus_start(void) {
    FDCAN_FilterTypeDef filter = {0};
    uint16_t std_id;
    uint8_t i;

    can3_bus_init_once();
    if(s_started) {
        return CAN3_BUS_ALREADY_STARTED;
    }
    if(s_router.route_count == 0u ||
       hfdcan3.Init.StdFiltersNbr < s_router.route_count ||
       hfdcan3.Init.TxFifoQueueElmtsNbr == 0u) {
        return CAN3_BUS_CONFIG_ERROR;
    }

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID2 = 0x7FFu;
    for(i = 0u; i < s_router.route_count; ++i) {
        if(!can3_router_get_id(&s_router, i, &std_id)) {
            return CAN3_BUS_CONFIG_ERROR;
        }
        filter.FilterIndex = i;
        filter.FilterID1 = std_id;
        if(HAL_FDCAN_ConfigFilter(&hfdcan3, &filter) != HAL_OK) {
            return CAN3_BUS_HAL_ERROR;
        }
    }

    if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan3,
                                    FDCAN_ACCEPT_IN_RX_FIFO0,
                                    s_observe_extended ? FDCAN_ACCEPT_IN_RX_FIFO0 : FDCAN_REJECT,
                                    FDCAN_REJECT_REMOTE,
                                    FDCAN_REJECT_REMOTE) != HAL_OK ||
       can_register_rx_callback(&hfdcan3, can3_bus_rx_callback, NULL) != STM32_HAL_CAN_OK ||
       can_start(&hfdcan3) != STM32_HAL_CAN_OK) {
        return CAN3_BUS_HAL_ERROR;
    }

    can3_router_lock(&s_router);
    s_started = true;
    return CAN3_BUS_OK;
}

Can3BusStatus can3_bus_observe_extended(bool enabled) {
    if(s_started) {
        return CAN3_BUS_ALREADY_STARTED;
    }
    s_observe_extended = enabled;
    return CAN3_BUS_OK;
}

Can3BusStatus can3_bus_send(uint16_t std_id, const uint8_t* data, uint8_t len) {
    if(std_id > 0x7FFu || data == NULL || len > 8u) {
        return CAN3_BUS_INVALID_PARAM;
    }
    if(!s_started) {
        return CAN3_BUS_NOT_STARTED;
    }
    return can_send(&hfdcan3, std_id, data, len) == STM32_HAL_CAN_OK
        ? CAN3_BUS_OK
        : CAN3_BUS_HAL_ERROR;
}

bool can3_bus_get_diagnostics(Can3BusDiagnostics* diagnostics) {
    uint32_t primask;

    if(diagnostics == NULL) {
        return false;
    }
    memset(diagnostics, 0, sizeof(*diagnostics));
    if(!can_get_diagnostics(&hfdcan3, &diagnostics->hal)) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    diagnostics->routed_count = s_router.routed_count;
    diagnostics->unknown_count = s_router.unknown_count;
    diagnostics->rejected_count = s_router.rejected_count;
    diagnostics->endpoint_count = s_router.route_count;
    diagnostics->started = s_started;
    __set_PRIMASK(primask);
    return true;
}

bool can3_bus_is_started(void) {
    return s_started;
}

static void can3_bus_init_once(void) {
    if(!s_initialized) {
        can3_router_init(&s_router);
        s_initialized = true;
    }
}

static void can3_bus_rx_callback(FDCAN_HandleTypeDef* hcan,
                                 const FDCAN_RxHeaderTypeDef* header,
                                 const uint8_t data[8],
                                 void* user) {
    Can3Frame frame = {0};
    uint8_t len;

    (void)hcan;
    (void)user;
    if(header == NULL || data == NULL || !can3_bus_dlc_to_len(header->DataLength, &len)) {
        ++s_router.rejected_count;
        return;
    }

    frame.id = header->Identifier;
    frame.id_type = header->IdType == FDCAN_STANDARD_ID
        ? CAN3_FRAME_STANDARD : CAN3_FRAME_EXTENDED;
    frame.frame_type = header->RxFrameType == FDCAN_DATA_FRAME
        ? CAN3_FRAME_DATA : CAN3_FRAME_REMOTE;
    frame.format = header->FDFormat == FDCAN_CLASSIC_CAN
        ? CAN3_FORMAT_CLASSIC : CAN3_FORMAT_FD;
    frame.len = len;
    memcpy(frame.data, data, sizeof(frame.data));
    frame.rx_ms = HAL_GetTick();
    (void)can3_router_dispatch(&s_router, &frame);
}

static bool can3_bus_dlc_to_len(uint32_t dlc, uint8_t* len) {
    static const uint32_t dlc_table[9] = {
        FDCAN_DLC_BYTES_0,
        FDCAN_DLC_BYTES_1,
        FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3,
        FDCAN_DLC_BYTES_4,
        FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6,
        FDCAN_DLC_BYTES_7,
        FDCAN_DLC_BYTES_8
    };
    uint8_t i;

    if(len == NULL) {
        return false;
    }
    for(i = 0u; i < 9u; ++i) {
        if(dlc == dlc_table[i]) {
            *len = i;
            return true;
        }
    }
    return false;
}
