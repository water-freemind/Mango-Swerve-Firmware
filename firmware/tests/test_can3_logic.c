#include "can3_router.h"
#include "laser.h"

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

static void test_laser_boot_snapshot(void) {
    LaserSnapshot snapshot;

    assert(laser_get_snapshot_at(&snapshot, 0u));
    assert(snapshot.frame_count == 0u && !snapshot.frame_fresh);
    assert(!laser_get_snapshot_at(NULL, 0u));
    assert(!laser_accept_can3_frame(NULL, NULL));
    laser_accept_frame(NULL, 100u);
    assert(laser_get_snapshot_at(&snapshot, 100u));
    assert(snapshot.frame_count == 0u && !snapshot.frame_fresh);
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

    assert(laser_get_snapshot_at(&snapshot, 1300u) && snapshot.frame_fresh);
    assert(laser_get_snapshot_at(&snapshot, 1301u));
    assert(snapshot.frame_count == valid_frame_count);
    assert(snapshot.last_rx_ms == 1000u);
    assert(!snapshot.frame_fresh);
}

static void test_laser_ports_and_raw_values(void) {
    uint8_t data[8] = {0};
    const uint16_t expected[LASER_CHANNEL_COUNT] = {1u, 256u, 1000u, 65535u};
    LaserSnapshot snapshot;
    uint32_t port;
    uint32_t channel;
    uint32_t previous_count;

    assert(laser_get_snapshot_at(&snapshot, 2000u));
    previous_count = snapshot.frame_count;
    laser_accept_frame(data, 2000u);
    assert(laser_get_snapshot_at(&snapshot, 2001u));
    assert(snapshot.frame_fresh && snapshot.frame_count == previous_count + 1u);
    for(channel = 0u; channel < LASER_CHANNEL_COUNT; ++channel) {
        assert(snapshot.distance_mm[channel] == 0u);
    }

    /* Sequential sensor connection must update exactly its S21C port slot. */
    for(port = 0u; port < LASER_CHANNEL_COUNT; ++port) {
        data[2u * port] = (uint8_t)(expected[port] >> 8u);
        data[2u * port + 1u] = (uint8_t)expected[port];
        laser_accept_frame(data, 2100u + port * 100u);
        assert(laser_get_snapshot_at(&snapshot, 2100u + port * 100u));
        for(channel = 0u; channel < LASER_CHANNEL_COUNT; ++channel) {
            assert(snapshot.distance_mm[channel] == (channel <= port ? expected[channel] : 0u));
        }
    }
    assert(snapshot.frame_count == previous_count + 5u);

    /* A repeated payload still represents a new board frame. */
    laser_accept_frame(data, 3000u);
    assert(laser_get_snapshot_at(&snapshot, 3000u) && snapshot.frame_fresh);
    assert(snapshot.frame_count == previous_count + 6u);
    assert(snapshot.distance_mm[3] == 65535u);
}

static void test_laser_tick_wraparound(void) {
    const uint8_t data[8] = {0, 100, 0, 200, 0, 250, 1, 0};
    LaserSnapshot snapshot;

    laser_accept_frame(data, UINT32_MAX - 100u);
    assert(laser_get_snapshot_at(&snapshot, 199u) && snapshot.frame_fresh);
    assert(laser_get_snapshot_at(&snapshot, 200u) && !snapshot.frame_fresh);
    assert(snapshot.distance_mm[0] == 100u && snapshot.distance_mm[3] == 256u);
}

int main(void) {
    test_laser_boot_snapshot();
    test_registry();
    test_laser_routing_and_timeout();
    test_laser_ports_and_raw_values();
    test_laser_tick_wraparound();
    puts("LASER_OFFLINE_PREPARATION_TEST_OK");
    puts("CAN3_LOGIC_TEST_OK");
    return 0;
}
