#include "remote.h"
#include "delay.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static FsIa10bData s_data;
static uint32_t s_now_ms;
static bool s_online;

ms_t delay_now_ms(void) { return s_now_ms; }
void ibus_maintain(void) {}
bool ibus_get_data(FsIa10bData* out) { *out = s_data; return s_online; }
bool ibus_is_online(uint32_t timeout_ms) { (void)timeout_ms; return s_online; }

int main(void) {
    RemoteState state;
    memset(&s_data, 0, sizeof(s_data));
    remote_init();
    assert(!remote_is_online(100u));
    assert(remote_get_raw_data() == NULL);
    s_online = true;
    s_now_ms = 1000u;
    s_data.last_update_ms = s_now_ms;
    s_data.channel[REMOTE_CH_SWD] = REMOTE_SW_LOW;
    s_data.channel[REMOTE_CH_SWA] = REMOTE_SW_HIGH;
    remote_process();
    assert(remote_get_state(&state) && state.online && state.manual_request);
    assert(state.manual_source == REMOTE_MANUAL_SOURCE_CHASSIS);
    assert(remote_is_online(100u));
    s_now_ms = 1101u;
    assert(!remote_is_online(100u));
    s_data.last_update_ms = s_now_ms;
    const uint16_t swa_values[] = {800u, 999u, 1001u, 1250u, 1251u, 1500u};
    for(unsigned i = 0u; i < sizeof(swa_values) / sizeof(swa_values[0]); ++i) {
        s_data.channel[REMOTE_CH_SWA] = swa_values[i];
        s_data.channel[REMOTE_CH_SWD] = 1999u;
        remote_process();
        assert(remote_get_state(&state) && state.online && state.manual_request);
        assert((state.manual_source == REMOTE_MANUAL_SOURCE_CHASSIS) == (swa_values[i] <= 1250u));
    }
    s_data.channel[REMOTE_CH_SWA] = 1001u;
    const uint16_t swd_values[] = {1000u, 1250u, 1500u, 1749u, 1750u, 2001u, 2200u};
    for(unsigned i = 0u; i < sizeof(swd_values) / sizeof(swd_values[0]); ++i) {
        s_data.channel[REMOTE_CH_SWD] = swd_values[i]; remote_process();
        assert(remote_is_manual_requested() == (swd_values[i] >= 1750u));
    }
    /* Repeated processing cannot refresh old receiver data. */
    s_now_ms += 101u;
    remote_process();
    assert(!remote_is_online(100u) && !remote_is_manual_requested());
    s_data.last_update_ms = s_now_ms;

    /* Former arm selection and autonomous switch positions stay outside chassis control. */
    s_data.channel[REMOTE_CH_SWA] = REMOTE_SW_LOW;
    remote_process();
    assert(remote_get_manual_source() == REMOTE_MANUAL_SOURCE_NONE);
    s_data.channel[REMOTE_CH_SWA] = REMOTE_SW_HIGH;
    s_data.channel[REMOTE_CH_SWD] = REMOTE_SW_HIGH;
    remote_process();
    assert(!remote_is_manual_requested());
    s_online = false;
    remote_process();
    assert(remote_get_state(&state) && !state.online && !state.manual_request);
    assert(remote_get_manual_source() == REMOTE_MANUAL_SOURCE_NONE);
    puts("CHASSIS_REMOTE_TEST_OK");
    return 0;
}
