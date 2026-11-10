#include "entry.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

uint32_t g_test_tick_ms = 1000u;
static FsIa10bData receiver;
static bool online = true;
static Odom attitude;
static float last_vx, last_vy, last_wz;

ms_t delay_now_ms(void) { return g_test_tick_ms; }
void ibus_maintain(void) {}
bool ibus_get_data(FsIa10bData* out) { *out = receiver; return online; }
bool ibus_is_online(uint32_t timeout_ms) { (void)timeout_ms; return online; }
bool work_link_hold(void) { return false; }
static bool ready(void) { return true; }
static const Odom* get_attitude(void) { return &attitude; }
static ChassisErrorCode set_velocity(float vx, float vy, float wz) {
    last_vx = vx;
    last_vy = vy;
    last_wz = wz;
    return CHASSIS_OK;
}
static ChassisErrorCode stop(void) { return set_velocity(0.0f, 0.0f, 0.0f); }
static ChassisErrorCode steer_first(bool enabled) { (void)enabled; return CHASSIS_OK; }
const struct ChassisInterface chassis_interface = {
    .is_ready = ready, .set_velocity = set_velocity,
    .brake = stop, .stop = stop, .set_steer_then_drive_enabled = steer_first,
};
const struct OdomInterface odom_interface = {.is_ready = ready, .get_state = get_attitude};

int main(void) {
    const struct {
        uint16_t channel;
        float linear;
        float angular;
    } gears[] = {
        {1000u, 0.4f, 1.5f}, {1002u, 0.4f, 1.5f},
        {1500u, 0.8f, 3.0f}, {1998u, 1.5f, 6.0f}, {2000u, 1.5f, 6.0f},
    };
    const ChassisYawHoldConfig heading = {.enabled = false};
    chassis_yaw_hold_init(&heading);
    remote_init();
    receiver.last_update_ms = g_test_tick_ms;
    for(unsigned i = 0u; i < FS_IA10B_CHANNEL_COUNT; ++i) receiver.channel[i] = REMOTE_CENTER;
    receiver.channel[REMOTE_CH_SWA] = REMOTE_SW_HIGH;
    receiver.channel[REMOTE_CH_SWD] = REMOTE_SW_LOW;
    app_apply_remote();
    assert(app_remote_armed);
    receiver.channel[REMOTE_CH_VRB] = 1000u;

    for(unsigned i = 0u; i < sizeof(gears) / sizeof(gears[0]); ++i) {
        receiver.channel[REMOTE_CH_SWB] = gears[i].channel;
        receiver.channel[REMOTE_CH_RIGHT_X] = REMOTE_CENTER;
        receiver.channel[REMOTE_CH_RIGHT_Y] = 2000u;
        receiver.channel[REMOTE_CH_LEFT_X] = REMOTE_CENTER;
        app_apply_remote();
        assert(fabsf(last_vx - gears[i].linear) < 0.00001f);
        assert(last_vy == 0.0f && last_wz == 0.0f);
        receiver.channel[REMOTE_CH_RIGHT_Y] = REMOTE_CENTER;
        receiver.channel[REMOTE_CH_LEFT_X] = 2000u;
        app_apply_remote();
        assert(last_vx == 0.0f && last_vy == 0.0f);
        assert(fabsf(last_wz + gears[i].angular) < 0.00001f);
        receiver.channel[REMOTE_CH_RIGHT_X] = 2000u;
        receiver.channel[REMOTE_CH_RIGHT_Y] = 2000u;
        receiver.channel[REMOTE_CH_LEFT_X] = REMOTE_CENTER;
        app_apply_remote();
        assert(fabsf(hypotf(last_vx, last_vy) - gears[i].linear) < 0.00001f);
    }
    receiver.channel[REMOTE_CH_VRB] = 1500u;
    app_apply_remote();
    assert(last_vx == 0.0f && last_vy == 0.0f && last_wz == 0.0f);
    online = false;
    app_apply_remote();
    assert(!app_remote_armed);
    online = true;
    receiver.channel[REMOTE_CH_VRB] = 1000u;
    app_apply_remote();
    assert(last_vx == 0.0f && last_vy == 0.0f && last_wz == 0.0f);
    puts("MANUAL_SPEED_GEARS_TEST_OK");
    return 0;
}
