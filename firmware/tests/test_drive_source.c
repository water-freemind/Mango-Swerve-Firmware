#include "drive_source.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static RemoteState remote(void) {
    RemoteState r = {.online = true, .manual_request = true, .manual_source = REMOTE_MANUAL_SOURCE_CHASSIS};
    for(unsigned i = 0u; i < FS_IA10B_CHANNEL_COUNT; ++i) r.rc_data.channel[i] = 1500u;
    r.rc_data.channel[REMOTE_CH_SWA] = 1000u;
    r.rc_data.channel[REMOTE_CH_SWC] = 2000u;
    r.rc_data.channel[REMOTE_CH_SWD] = 2000u;
    return r;
}
int main(void) {
    DriveSource s = {0};
    RemoteState r = remote();
    assert(drive_source_update(&s, &r, true, true));
    assert(s.mode == DRIVE_SOURCE_MANUAL && !s.manual_armed); /* Boot with SWD on cannot drive. */
    r.rc_data.channel[REMOTE_CH_SWD] = 1000u; r.manual_request = false;
    drive_source_update(&s, &r, true, true);
    assert(s.release_seen && !s.manual_armed);
    r.rc_data.channel[REMOTE_CH_SWD] = 2000u; r.manual_request = true;
    r.rc_data.channel[REMOTE_CH_RIGHT_Y] = 2000u;
    drive_source_update(&s, &r, true, true);
    assert(!s.manual_armed); /* Deflected stick cannot acquire ownership. */
    r.rc_data.channel[REMOTE_CH_RIGHT_Y] = 1500u;
    drive_source_update(&s, &r, true, false);
    assert(!s.manual_armed); /* Must stop the previous owner's wheels first. */
    drive_source_update(&s, &r, true, true);
    assert(s.manual_armed);
    r.rc_data.channel[REMOTE_CH_RIGHT_Y] = 2000u;
    drive_source_update(&s, &r, true, false);
    assert(s.manual_armed); /* Moving sticks are valid after manual enable. */
    const float linear[] = {0.4f, 0.8f, 1.5f}, angular[] = {1.5f, 3.0f, 6.0f};
    for(unsigned i = 0; i < 3; ++i) {
        r.rc_data.channel[REMOTE_CH_SWB] = (uint16_t)(1000u + 500u * i);
        DriveVelocity v = drive_source_velocity(&r);
        assert(fabsf(v.vx - linear[i]) < 0.00001f && v.vy == 0.0f);
        r.rc_data.channel[REMOTE_CH_RIGHT_X] = 2000u;
        r.rc_data.channel[REMOTE_CH_LEFT_X] = 2000u;
        v = drive_source_velocity(&r);
        assert(fabsf(hypotf(v.vx, v.vy) - linear[i]) < 0.00001f);
        assert(v.vy < 0.0f && fabsf(v.wz + angular[i]) < 0.00001f);
        r.rc_data.channel[REMOTE_CH_RIGHT_X] = r.rc_data.channel[REMOTE_CH_LEFT_X] = 1500u;
    }
    drive_source_stop_manual(&s);
    drive_source_update(&s, &r, true, true);
    assert(!s.manual_armed && !s.release_seen); /* STOP cannot re-enable itself. */
    r.rc_data.channel[REMOTE_CH_SWC] = 1500u;
    assert(drive_source_update(&s, &r, true, true) && s.mode == DRIVE_SOURCE_HOST);
    r.rc_data.channel[REMOTE_CH_SWC] = 2000u;
    assert(drive_source_update(&s, &r, true, true) && !s.manual_armed);
    r.rc_data.channel[REMOTE_CH_SWD] = 1000u; r.manual_request = false;
    drive_source_update(&s, &r, true, true);
    r.rc_data.channel[REMOTE_CH_SWD] = 2000u; r.manual_request = true;
    r.rc_data.channel[REMOTE_CH_RIGHT_Y] = 1500u;
    drive_source_update(&s, &r, true, true);
    assert(s.manual_armed);
    drive_source_update(&s, &r, false, true);
    assert(!s.manual_armed && !s.release_seen);
    drive_source_update(&s, &r, true, true);
    assert(!s.manual_armed); /* A recovered sensor alone does not restart. */
    r.online = false;
    assert(drive_source_update(&s, &r, true, true) && s.mode == DRIVE_SOURCE_STOP);
    r.online = true;
    drive_source_update(&s, &r, true, true);
    assert(!s.manual_armed);
    r.rc_data.channel[REMOTE_CH_SWC] = 1750u;
    assert(drive_source_requested(&r) == DRIVE_SOURCE_STOP);
    r.rc_data.channel[REMOTE_CH_SWC] = 1000u;
    drive_source_update(&s, &r, true, true);
    assert(s.mode == DRIVE_SOURCE_STOP && !s.manual_armed);
    puts("DRIVE_SOURCE_TEST_OK (exclusive SWC ownership, SWD re-enable, centred/stopped handover, fault/RC loss, manual gears)");
    return 0;
}
