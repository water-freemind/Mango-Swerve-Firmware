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

static void test_laser_accepted_samples(void) {
    const uint8_t valid[8] = {0x1D, 0x59, 0x3A, 0x98, 0x03, 0xE8, 0x01, 0x2C};
    const uint8_t invalid[8] = {0x1D, 0x59, 0x3A, 0x99, 0xFE, 0x01, 0xFE, 0x67};
    LaserSnapshot snapshot;
    laser_accept_frame(valid, 9900u);
    laser_accept_frame(valid, 10000u);
    assert(laser_get_snapshot_at(&snapshot, 10000u));
    assert(snapshot.valid_mask == 15u && snapshot.accepted_mm[0] == 7513u && snapshot.accepted_mm[1] == 15000u);
    laser_accept_frame(invalid, 10050u);
    assert(laser_get_snapshot_at(&snapshot, 10050u));
    assert(snapshot.distance_mm[3] == 65127u && snapshot.accepted_mm[3] == 300u);
    assert(snapshot.accepted_mm[1] == 15000u && snapshot.accepted_mm[2] == 1000u);
    assert(snapshot.valid_mask == 15u && snapshot.sample_age_ms[3] == 50u && snapshot.updated_mask == 1u);
    assert(laser_get_snapshot_at(&snapshot, 10301u));
    assert(snapshot.frame_fresh && snapshot.valid_mask == 1u); /* Raw frames cannot renew rejected channels. */
    laser_accept_frame(valid, 10302u);
    assert(laser_get_snapshot_at(&snapshot, 10302u) && snapshot.valid_mask == 15u);
}

static void test_laser_jump_confirmation(void) {
    uint8_t data[8] = {0x13,0x88,0x13,0x88,0x13,0x88,0x13,0x88};
    LaserSnapshot snapshot;
    laser_accept_frame(data, 11000u);
    laser_accept_frame(data, 11100u);
    data[6] = 0x07; data[7] = 0xD0; /* 5000 -> 2000 mm single spike. */
    laser_accept_frame(data, 11200u);
    assert(laser_get_snapshot_at(&snapshot, 11200u));
    assert(snapshot.distance_mm[3] == 2000u && snapshot.accepted_mm[3] == 5000u);
    assert((snapshot.updated_mask & 8u) && snapshot.sample_age_ms[3] == 0u);
    data[6] = 0x13; data[7] = 0x24; /* Returns to 4900 mm. */
    laser_accept_frame(data, 11300u);
    assert(laser_get_snapshot_at(&snapshot, 11300u) && snapshot.accepted_mm[3] == 4900u);
    laser_accept_frame(data, 11350u);
    data[6] = 0x07; data[7] = 0xD0;
    laser_accept_frame(data, 11400u);
    laser_accept_frame(data, 11400u); /* Same timestamp cannot confirm a jump. */
    assert(laser_get_snapshot_at(&snapshot, 11400u) && snapshot.accepted_mm[3] == 4900u);
    data[6] = 0x07; data[7] = 0x9E; /* Independent 1950 mm confirms new surface. */
    laser_accept_frame(data, 11500u);
    assert(laser_get_snapshot_at(&snapshot, 11500u) && snapshot.accepted_mm[3] == 2000u);
    data[6] = 0; data[7] = 109u; /* Real near returns bypass confirmation. */
    laser_accept_frame(data, 11600u);
    assert(laser_get_snapshot_at(&snapshot, 11600u) && snapshot.accepted_mm[3] == 109u);
    data[6] = 0xFE; data[7] = 0x67;
    laser_accept_frame(data, 11901u);
    assert(laser_get_snapshot_at(&snapshot, 11901u));
    assert(!(snapshot.valid_mask & 8u) && snapshot.accepted_mm[3] == 109u);
    /* Continuous noisy but in-range data must not freeze an old near value. */
    const uint16_t noisy[] = {2400u, 6300u, 3100u, 5000u, 2700u};
    for(unsigned i = 0u; i < sizeof(noisy) / sizeof(noisy[0]); ++i) {
        data[6] = (uint8_t)(noisy[i] >> 8u); data[7] = (uint8_t)noisy[i];
        laser_accept_frame(data, 12100u + i * 100u);
        assert(laser_get_snapshot_at(&snapshot, 12100u + i * 100u));
        assert((snapshot.valid_mask & 8u) && snapshot.accepted_mm[3] >= 2400u);
    }
}

static void test_continuous_median_timestamps(void) {
    uint8_t data[8]={0x0a,0xa0,0x0a,0xa0,0x0a,0xa0,0x0a,0xa0};
    LaserSnapshot s;
    for(unsigned now=20000;now<=20200;now+=100) laser_accept_frame(data,now);
    data[6]=0x04;data[7]=0x3f; /* 1087mm, the recorded short spike. */
    laser_accept_frame(data,20300);
    assert(laser_get_snapshot_at(&s,20300));
    assert(s.accepted_mm[3]==1087 && s.median_mm[3]==2720);
    assert(s.median_age_ms[3]==100 && !(s.median_updated_mask&8u));
    data[6]=0x0a;data[7]=0x4a; /* Recovery to 2634mm. */
    laser_accept_frame(data,20400);assert(laser_get_snapshot_at(&s,20400));
    assert(s.median_mm[3]==2634 && s.median_age_ms[3]==0 && (s.median_updated_mask&8u));
    data[6]=0x05;data[7]=0xdc;
    laser_accept_frame(data,20750); /* A gap resets history. */
    data[6]=0x05;data[7]=0xb4;laser_accept_frame(data,20850); /* 1460. */
    data[6]=0x05;data[7]=0x8c;laser_accept_frame(data,20950); /* 1420. */
    assert(laser_get_snapshot_at(&s,20950));
    assert(s.median_mm[3]==1460 && s.median_age_ms[3]==100 && !(s.median_updated_mask&8u));
    data[6]=0x05;data[7]=0x64;laser_accept_frame(data,21050); /* 1380. */
    assert(laser_get_snapshot_at(&s,21050));
    assert(s.median_mm[3]==1420 && s.median_age_ms[3]==100 && (s.median_updated_mask&8u));
    data[6]=0xfe;data[7]=0x6b;laser_accept_frame(data,21150);
    assert(laser_get_snapshot_at(&s,21150));
    assert(s.median_mm[3]==1420 && s.median_age_ms[3]==200 && !(s.median_updated_mask&8u));
    assert(laser_get_snapshot_at(&s,21351) && !(s.median_valid_mask&8u));
    data[6]=0x03;data[7]=0xe8;laser_accept_frame(data,22000); /* 1000. */
    data[6]=0x04;data[7]=0xb0;laser_accept_frame(data,22100); /* 1200. */
    data[6]=0x04;data[7]=0x4c;laser_accept_frame(data,22200); /* 1100, new median. */
    data[6]=0x05;data[7]=0x14;laser_accept_frame(data,22300); /* 1300 selects the older 1200. */
    assert(laser_get_snapshot_at(&s,22300));
    assert(s.median_mm[3]==1200 && s.median_age_ms[3]==200 && !(s.median_updated_mask&8u));
    puts("LASER_CONTINUOUS_MEDIAN_OK 1087mm_spike source_age independent_updates expiry");
}
int main(void) {
    test_laser_boot_snapshot();
    test_registry();
    test_laser_routing_and_timeout();
    test_laser_ports_and_raw_values();
    test_laser_tick_wraparound();
    test_laser_accepted_samples();
    test_laser_jump_confirmation();
    test_continuous_median_timestamps();
    puts("LASER_OFFLINE_PREPARATION_TEST_OK");
    puts("CAN3_LOGIC_TEST_OK");
    return 0;
}
