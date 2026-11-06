#include "work_link.h"
#include "work_config.h"

#include <assert.h>
#include <stdio.h>

FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
FDCAN_GlobalTypeDef g_fake_can_regs[3];
uint32_t g_test_tick_ms = 10000u;
static unsigned registrations;
static unsigned transmissions;

Can3BusStatus can3_bus_register_endpoint(uint16_t id, Can3EndpointCallback callback, void* user) {
    (void)id;
    (void)callback;
    (void)user;
    ++registrations;
    return CAN3_BUS_OK;
}

BspCanStatus can_register_rx_callback(FDCAN_HandleTypeDef* bus, STM32HalCanRxCallback callback, void* user) {
    (void)bus;
    (void)callback;
    (void)user;
    ++registrations;
    return STM32_HAL_CAN_OK;
}

Can3BusStatus can3_bus_send(uint16_t id, const uint8_t* data, uint8_t length) {
    (void)id;
    (void)data;
    (void)length;
    ++transmissions;
    return CAN3_BUS_OK;
}

int main(void) {
    float vx = 0.08f, vy = 0.06f, wz = 0.20f;
    assert(!WORK_LINK_ENABLED);
    assert(work_link_hold());
    assert(work_link_init());
    assert(!work_link_hold());
    assert(!g_work_link.paired);
    work_link_process(10000u, false);
    work_link_limit_velocity(&vx, &vy, &wz);
    assert(vx == 0.08f && vy == 0.06f && wz == 0.20f);
    assert(registrations == 0u && transmissions == 0u);
    assert(work_link_init() && !work_link_hold());
    puts("WORK_LINK_STANDALONE_TEST_OK");
    return 0;
}
