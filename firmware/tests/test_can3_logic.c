#include "can3_router.h"
#include "laser.h"
#include "thinning.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

uint32_t g_test_tick_ms;

static bool accept_any(const Can3Frame* frame, void* user) {
    uint32_t* count = user;

    (void)frame;
    ++(*count);
    return true;
}

static Can3Frame make_laser_frame(uint32_t id, uint8_t len, uint32_t rx_ms) {
    Can3Frame frame = {
        .id = id,
        .id_type = CAN3_FRAME_STANDARD,
        .frame_type = CAN3_FRAME_DATA,
        .format = CAN3_FORMAT_CLASSIC,
        .len = len,
        .data = {0x00, 0x64, 0x01, 0x02, 0x03, 0xE8, 0xFF, 0xFF},
        .rx_ms = rx_ms
    };
    return frame;
}

static void test_registry(void) {
    Can3Router router;
    uint32_t count = 0u;
    uint16_t id;
    uint8_t i;

    can3_router_init(&router);
    assert(can3_router_register(&router, 0x601u, accept_any, &count) == CAN3_ROUTER_OK);
    assert(can3_router_register(&router, 0x601u, accept_any, &count) == CAN3_ROUTER_DUPLICATE_ID);
    assert(can3_router_get_id(&router, 0u, &id) && id == 0x601u);
    for(i = 1u; i < CAN3_ROUTER_MAX_ENDPOINTS; ++i) {
        assert(can3_router_register(&router, (uint16_t)(0x601u + i), accept_any, &count) == CAN3_ROUTER_OK);
    }
    assert(can3_router_register(&router, 0x700u, accept_any, &count) == CAN3_ROUTER_FULL);
    can3_router_lock(&router);
    assert(can3_router_register(&router, 0x500u, accept_any, &count) == CAN3_ROUTER_LOCKED);
}

static void test_laser_routing_and_timeout(void) {
    Can3Router router;
    Can3Frame frame;
    LaserSnapshot snapshot;
    uint32_t valid_frame_count;

    can3_router_init(&router);
    assert(can3_router_register(&router, 0x601u, laser_accept_can3_frame, NULL) == CAN3_ROUTER_OK);

    frame = make_laser_frame(0x601u, 8u, 1000u);
    assert(can3_router_dispatch(&router, &frame));
    assert(laser_get_snapshot_at(&snapshot, 1000u));
    assert(snapshot.distance_mm[0] == 100u);
    assert(snapshot.distance_mm[1] == 258u);
    assert(snapshot.distance_mm[2] == 1000u);
    assert(snapshot.distance_mm[3] == 65535u);
    assert(snapshot.frame_fresh);
    valid_frame_count = snapshot.frame_count;

    frame = make_laser_frame(0x602u, 8u, 1100u);
    assert(!can3_router_dispatch(&router, &frame));
    assert(router.unknown_count == 1u);

    frame = make_laser_frame(0x601u, 7u, 1200u);
    assert(!can3_router_dispatch(&router, &frame));
    assert(router.rejected_count == 1u);

    frame = make_laser_frame(0x601u, 8u, 1200u);
    frame.id_type = CAN3_FRAME_EXTENDED;
    assert(!can3_router_dispatch(&router, &frame));
    frame.id_type = CAN3_FRAME_STANDARD;
    frame.frame_type = CAN3_FRAME_REMOTE;
    assert(!can3_router_dispatch(&router, &frame));
    frame.frame_type = CAN3_FRAME_DATA;
    frame.format = CAN3_FORMAT_FD;
    assert(!can3_router_dispatch(&router, &frame));
    assert(router.rejected_count == 4u);

    assert(laser_get_snapshot_at(&snapshot, 1301u));
    assert(snapshot.frame_count == valid_frame_count);
    assert(snapshot.last_rx_ms == 1000u);
    assert(!snapshot.frame_fresh);
}

static void test_thinning_safety_contract(void) {
    ThinningSnapshot snapshot;

    thinning_init();
    assert(!thinning_permit(false, 10u, 100u));
    assert(thinning_permit(true, 10u, 100u));
    assert(thinning_mark_busy(20u));
    thinning_process(109u);
    assert(thinning_get_snapshot(&snapshot) && snapshot.state == THINNING_BUSY);
    thinning_process(110u);
    assert(thinning_get_snapshot(&snapshot) && snapshot.state == THINNING_TIMED_OUT);
    assert(snapshot.chassis_brake_required);
    thinning_reset(120u);
    assert(thinning_get_snapshot(&snapshot) && snapshot.state == THINNING_IDLE);
    assert(!snapshot.chassis_brake_required);
}

int main(void) {
    test_registry();
    test_laser_routing_and_timeout();
    test_thinning_safety_contract();
    puts("CAN3_LOGIC_TEST_OK");
    return 0;
}
