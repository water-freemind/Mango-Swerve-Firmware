#include "entry.h"
#include "chassis_can_map.h"
#include "log.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

uint32_t g_test_tick_ms = 1000u;
FDCAN_GlobalTypeDef g_fake_can_regs[3];
FDCAN_HandleTypeDef hfdcan1 = {.Instance = FDCAN1};
FDCAN_HandleTypeDef hfdcan2 = {.Instance = FDCAN2};
FDCAN_HandleTypeDef hfdcan3 = {
    .Instance = FDCAN3, .Init = {.StdFiltersNbr = 8u, .TxFifoQueueElmtsNbr = 8u}
};

typedef struct {
    FDCAN_RxHeaderTypeDef header[32];
    uint8_t data[32][8];
    uint32_t used, read, starts, notifications;
} FakeCan;
static FakeCan s_buses[3];
static unsigned s_tx_calls, s_abort_calls;
static bool s_fail_can1, s_tx_full;

static FakeCan* fake_bus(FDCAN_HandleTypeDef* hcan) {
    return &s_buses[hcan->Instance - g_fake_can_regs];
}

HAL_StatusTypeDef HAL_FDCAN_ConfigInterruptLines(FDCAN_HandleTypeDef* h, uint32_t flags, uint32_t line) {
    (void)h; (void)flags; (void)line; return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef* h, uint32_t flags, uint32_t indexes) {
    (void)indexes; fake_bus(h)->notifications = flags; return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef* h) {
    ++fake_bus(h)->starts;
    if(h == &hfdcan1 && s_fail_can1) return HAL_ERROR;
    h->State = HAL_FDCAN_STATE_BUSY;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef* h, const FDCAN_FilterTypeDef* f) {
    assert(h == &hfdcan3 && f->FilterID1 == 0x601u && f->FilterID2 == 0x7FFu);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef* h, uint32_t a, uint32_t b, uint32_t c, uint32_t d) {
    if(h == &hfdcan3) {
        assert(a == FDCAN_ACCEPT_IN_RX_FIFO0 && b == FDCAN_ACCEPT_IN_RX_FIFO0);
    }
    assert(c == FDCAN_REJECT_REMOTE && d == FDCAN_REJECT_REMOTE);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef* h, FDCAN_ProtocolStatusTypeDef* p) {
    (void)h; memset(p, 0, sizeof(*p)); return HAL_OK;
}
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef* h) {
    (void)h; return s_tx_full ? 0u : 8u;
}
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef* h, uint32_t pending) {
    (void)h; (void)pending; ++s_abort_calls; return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef* h, FDCAN_TxHeaderTypeDef* header, uint8_t* data) {
    (void)h; (void)header; (void)data; ++s_tx_calls; return HAL_OK;
}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef* h, uint32_t fifo) {
    (void)fifo; return fake_bus(h)->used - fake_bus(h)->read;
}
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef* h, uint32_t fifo, FDCAN_RxHeaderTypeDef* header, uint8_t* data) {
    FakeCan* bus = fake_bus(h);
    (void)fifo;
    assert(bus->read < bus->used);
    *header = bus->header[bus->read];
    memcpy(data, bus->data[bus->read], 8u);
    ++bus->read;
    return HAL_OK;
}
void HAL_GPIO_WritePin(uint32_t port, uint32_t pin, uint32_t state) {
    (void)port; (void)pin; (void)state;
}
LogStatus log_info(const char* format, ...) { (void)format; return LOG_STATUS_OK; }
SystemStatus assemble_delay(void) { return SYSTEM_STATUS_OK; }
SystemStatus assemble_log(void) { return SYSTEM_STATUS_OK; }

static void enqueue(FDCAN_HandleTypeDef* h, uint32_t id) {
    FakeCan* bus = fake_bus(h);
    const uint8_t distances[8] = {0, 100, 1, 2, 3, 0xE8, 0, 50};
    bus->header[bus->used] = (FDCAN_RxHeaderTypeDef){
        .Identifier = id, .DataLength = FDCAN_DLC_BYTES_8
    };
    memcpy(bus->data[bus->used], distances, 8u);
    ++bus->used;
}

int main(int argc, char** argv) {
    LaserSnapshot laser;
    unsigned i;
    const uint8_t payload[8] = {0};
    s_fail_can1 = argc > 1 && strcmp(argv[1], "fail-can1") == 0;
    assert(APP_RUN_MODE == APP_MODE_CAN_DIAGNOSTICS);
#if CHASSIS_CAN_BUSES_SWAPPED
    assert(CHASSIS_STEER_CAN == &hfdcan2 && CHASSIS_DRIVE_CAN == &hfdcan1);
#else
    assert(CHASSIS_STEER_CAN == &hfdcan1 && CHASSIS_DRIVE_CAN == &hfdcan2);
#endif

    /* Real entry point is linked without IMU/chassis/remote implementations. */
    entry_init();
    commissioning_init();
    for(i = 0u; i < 3u; ++i) assert(s_buses[i].starts == 1u);
    assert(g_commissioning.bus[0].started == !s_fail_can1);
    assert(g_commissioning.bus[1].started && g_commissioning.bus[2].started);
    assert(s_tx_calls == 0u);

    if(!s_fail_can1) enqueue(&hfdcan1, 0x201u);
    enqueue(&hfdcan2, 0x202u);
    for(i = 0u; i < 15u; ++i) enqueue(&hfdcan3, 0x500u + i);
    enqueue(&hfdcan3, 0x601u);
    HAL_FDCAN_RxFifo0Callback(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    assert(s_buses[2].read == 8u);
    assert(laser_get_snapshot(&laser) && laser.frame_count == 0u);
    entry_loop();
    assert(s_buses[2].read == 16u); /* No new interrupt needed for the remaining eight. */
    assert(laser_get_snapshot(&laser) && laser.frame_count == 1u && laser.frame_fresh);
    assert(laser.distance_mm[0] == 100u && laser.distance_mm[1] == 258u);
    assert(laser.distance_mm[2] == 1000u && laser.distance_mm[3] == 50u);
    assert(g_commissioning.bus[1].frames[0].header.Identifier == 0x202u);
    assert(g_commissioning.bus[2].id_count == 16u);
    assert(g_commissioning.can3.unknown_count == 15u);
    assert(s_tx_calls == 0u && s_abort_calls == 0u);

    g_test_tick_ms = 1301u;
    enqueue(&hfdcan3, 0x555u);
    entry_loop();
    assert(laser_get_snapshot(&laser) && laser.frame_count == 1u && !laser.frame_fresh);
    assert(g_commissioning.bus[2].table_full_count == 1u);
    assert(s_tx_calls == 0u);

    /* An extended frame using the same numeric ID must not refresh S21C. */
    enqueue(&hfdcan3, 0x601u);
    s_buses[2].header[s_buses[2].used - 1u].IdType = FDCAN_EXTENDED_ID;
    entry_loop();
    assert(laser_get_snapshot(&laser) && laser.frame_count == 1u && !laser.frame_fresh);
    assert(can3_bus_observe_extended(false) == CAN3_BUS_ALREADY_STARTED);

    /* Host-only queue-full injection: CAN3 must not abort another consumer. */
    s_tx_full = true;
    hfdcan3.Instance->TXBRP = 3u;
    assert(can3_bus_send(0x601u, payload, 8u) == CAN3_BUS_HAL_ERROR);
    assert(s_abort_calls == 0u && s_tx_calls == 0u);
    puts(s_fail_can1 ? "COMMISSIONING_CAN1_FAILURE_TEST_OK" : "COMMISSIONING_TEST_OK");
    return 0;
}
