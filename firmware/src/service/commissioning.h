#ifndef MANGO_SWERVE_COMMISSIONING_H
#define MANGO_SWERVE_COMMISSIONING_H

#include "can3_bus.h"
#include "laser.h"

#define COMMISSIONING_BUS_COUNT 3u
#define COMMISSIONING_ID_SLOTS 16u

typedef struct {
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];
    uint32_t count;
    uint32_t last_rx_ms;
} CommissioningFrame;

typedef struct {
    CommissioningFrame frames[COMMISSIONING_ID_SLOTS];
    uint32_t table_full_count;
    uint8_t id_count;
    BspCanStatus start_status;
    bool started;
    STM32HalCanDiagnostics diagnostics;
} CommissioningBus;

typedef struct {
    uint32_t uptime_ms;
    CommissioningBus bus[COMMISSIONING_BUS_COUNT]; /* Physical CAN1, CAN2, CAN3. */
    Can3BusStatus can3_status;
    Can3BusDiagnostics can3;
    LaserSnapshot laser;
} CommissioningSnapshot;

/* SWD watch variable; UART1 also prints a summary every second. */
extern volatile CommissioningSnapshot g_commissioning;

void commissioning_init(void);
void commissioning_process(void);

#endif
