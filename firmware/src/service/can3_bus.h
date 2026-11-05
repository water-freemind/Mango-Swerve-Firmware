#ifndef MANGO_SWERVE_CAN3_BUS_H
#define MANGO_SWERVE_CAN3_BUS_H

#include "can3_router.h"
#include "stm32_hal_can.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    CAN3_BUS_OK = 0,
    CAN3_BUS_INVALID_PARAM,
    CAN3_BUS_DUPLICATE_ID,
    CAN3_BUS_ENDPOINT_FULL,
    CAN3_BUS_ALREADY_STARTED,
    CAN3_BUS_NOT_STARTED,
    CAN3_BUS_CONFIG_ERROR,
    CAN3_BUS_HAL_ERROR
} Can3BusStatus;

typedef struct {
    STM32HalCanDiagnostics hal;
    uint32_t routed_count;
    uint32_t unknown_count;
    uint32_t rejected_count;
    uint8_t endpoint_count;
    bool started;
} Can3BusDiagnostics;

Can3BusStatus can3_bus_register_endpoint(uint16_t std_id,
                                         Can3EndpointCallback callback,
                                         void* user);
Can3BusStatus can3_bus_start(void);
/* Before start only: receive extended data frames for raw diagnostic capture. */
Can3BusStatus can3_bus_observe_extended(bool enabled);
Can3BusStatus can3_bus_send(uint16_t std_id, const uint8_t* data, uint8_t len);
bool can3_bus_get_diagnostics(Can3BusDiagnostics* diagnostics);
bool can3_bus_is_started(void);

#endif
