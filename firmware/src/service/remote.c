/**
 * @file remote.c
 * @brief FS-iA10B manual chassis input service.
 */

#include "remote.h"
#include "delay.h"

#include <string.h>

#define REMOTE_TIMEOUT_MS 100u

static RemoteState s_remote_state;

void remote_init(void) {
    memset(&s_remote_state, 0, sizeof(s_remote_state));
}

void remote_process(void) {
    FsIa10bData rc_data;

    ibus_maintain();
    if(!ibus_get_data(&rc_data) || !ibus_is_online(REMOTE_TIMEOUT_MS)) {
        s_remote_state.online = false;
        s_remote_state.manual_request = false;
        s_remote_state.manual_source = REMOTE_MANUAL_SOURCE_NONE;
        return;
    }

    s_remote_state.online = true;
    s_remote_state.rc_data = rc_data;
    s_remote_state.stamp_ms = delay_now_ms();
    s_remote_state.manual_request = rc_data.channel[REMOTE_CH_SWD] == REMOTE_SW_LOW;
    s_remote_state.manual_source = REMOTE_MANUAL_SOURCE_NONE;
    if(rc_data.channel[REMOTE_CH_SWA] == REMOTE_SW_HIGH) {
        s_remote_state.manual_source = REMOTE_MANUAL_SOURCE_CHASSIS;
    }
}

bool remote_get_state(RemoteState* out) {
    if(out == NULL) {
        return false;
    }
    *out = s_remote_state;
    return true;
}

bool remote_is_online(uint32_t timeout_ms) {
    return s_remote_state.online &&
           (delay_now_ms() - s_remote_state.stamp_ms) <= timeout_ms;
}

bool remote_is_manual_requested(void) {
    return s_remote_state.online && s_remote_state.manual_request;
}

RemoteManualSource remote_get_manual_source(void) {
    if(!s_remote_state.online) {
        return REMOTE_MANUAL_SOURCE_NONE;
    }
    return s_remote_state.manual_source;
}

const FsIa10bData* remote_get_raw_data(void) {
    if(!s_remote_state.online) {
        return NULL;
    }
    return &s_remote_state.rc_data;
}
