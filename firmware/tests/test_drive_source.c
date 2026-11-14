#include "drive_source.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static RemoteState remote(void) {
    RemoteState r = {.online = true, .manual_request = true, .manual_source = REMOTE_MANUAL_SOURCE_CHASSIS};
    for(unsigned i = 0u; i < FS_IA10B_CHANNEL_COUNT; ++i) r.rc_data.channel[i] = 1500u;
    r.rc_data.channel[REMOTE_CH_SWA] = 1000u;
    r.rc_data.channel[REMOTE_CH_SWC] = 2000u;
    r.rc_data.channel[REMOTE_CH_SWD] = 2000u;
    r.rc_data.channel[REMOTE_CH_VRB] = 1250u;
    return r;
}
int main(void) {
    DriveSource s = {0};
    RemoteState r = remote();
    assert(drive_source_update(&s, &r, true, true));
    assert(s.mode == DRIVE_SOURCE_MANUAL && !s.manual_armed); /* Boot with SWD on cannot drive. */
    assert(!strcmp(drive_source_manual_wait_reason(&s,&r,true,true),"SWD_REARM"));
    r.rc_data.channel[REMOTE_CH_SWD] = 1000u; r.manual_request = false;
    drive_source_update(&s, &r, true, true);
    assert(s.release_seen && !s.manual_armed);
    /* SWD off/on is repeatable even if feedback and VRB become ready later. */
    for(unsigned cycle = 0; cycle < 3u; ++cycle) {
        r.rc_data.channel[REMOTE_CH_SWD] = 1000u; r.manual_request = false;
        drive_source_update(&s, &r, false, true);
        assert(s.release_seen && !s.manual_armed);
        r.rc_data.channel[REMOTE_CH_SWD] = 2000u; r.manual_request = true;
        r.rc_data.channel[REMOTE_CH_VRB] = 1500u;
        drive_source_update(&s, &r, false, true);
        assert(s.release_seen && !s.manual_armed);
        drive_source_update(&s, &r, true, true);
        assert(s.release_seen && !s.manual_armed);
        assert(!strcmp(drive_source_manual_wait_reason(&s,&r,true,true),"VRB_RELEASED"));
        r.rc_data.channel[REMOTE_CH_VRB] = 1250u;
        drive_source_update(&s, &r, true, false);
        assert(!s.manual_armed);
        drive_source_update(&s, &r, true, true);
        assert(s.manual_armed);
    }
    r.rc_data.channel[REMOTE_CH_SWD] = 1000u; r.manual_request = false;
    drive_source_update(&s, &r, true, true);
    r.rc_data.channel[REMOTE_CH_SWD] = 2000u; r.manual_request = true;
    r.rc_data.channel[REMOTE_CH_RIGHT_Y] = 2000u;
    drive_source_update(&s, &r, true, true);
    assert(!s.manual_armed); /* Deflected stick cannot acquire ownership. */
    r.rc_data.channel[REMOTE_CH_RIGHT_Y] = 1500u;
    drive_source_update(&s, &r, true, false);
    assert(!s.manual_armed); /* Must stop the previous owner's wheels first. */
    drive_source_update(&s, &r, true, true);
    assert(s.manual_armed);
    /* Normal SWC handovers keep SWD confirmation, but stop the old owner and
     * wait for neutral sticks and stationary wheels before manual takeover. */
    r.rc_data.channel[REMOTE_CH_SWC] = 1500u;
    assert(drive_source_update(&s, &r, true, false));
    assert(s.mode == DRIVE_SOURCE_HOST && !s.manual_armed && s.release_seen);
    r.rc_data.channel[REMOTE_CH_SWC] = 1750u;
    assert(drive_source_update(&s, &r, true, false));
    assert(s.mode == DRIVE_SOURCE_STOP && !s.manual_armed && s.release_seen);
    r.rc_data.channel[REMOTE_CH_SWC] = 2000u;
    r.rc_data.channel[REMOTE_CH_RIGHT_Y] = 2000u;
    assert(drive_source_update(&s, &r, true, true));
    assert(!s.manual_armed && s.release_seen);
    r.rc_data.channel[REMOTE_CH_RIGHT_Y] = 1500u;
    drive_source_update(&s, &r, true, false);
    assert(!s.manual_armed && s.release_seen);
    drive_source_update(&s, &r, true, true);
    assert(s.manual_armed);
    r.rc_data.channel[REMOTE_CH_SWC] = 1500u;
    drive_source_update(&s, &r, true, true);
    r.rc_data.channel[REMOTE_CH_SWC] = 2000u;
    assert(drive_source_update(&s, &r, true, true) && s.manual_armed);
    /* SWD can be released in HOST before selecting MANUAL. */
    r.rc_data.channel[REMOTE_CH_SWC] = 1500u;
    r.rc_data.channel[REMOTE_CH_SWD] = 1000u; r.manual_request = false;
    drive_source_update(&s, &r, true, true);
    r.rc_data.channel[REMOTE_CH_SWD] = 2000u; r.manual_request = true;
    drive_source_update(&s, &r, true, true);
    r.rc_data.channel[REMOTE_CH_SWC] = 2000u;
    assert(drive_source_update(&s, &r, true, true) && s.manual_armed);
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
    /* VRB release stops while moving. Re-press cannot resume a deflected stick,
     * and must wait for centred sticks and a stationary chassis. */
    r.rc_data.channel[REMOTE_CH_VRB] = 1251u;
    drive_source_update(&s, &r, true, false);
    assert(s.mode == DRIVE_SOURCE_MANUAL && !s.manual_armed && s.release_seen);
    r.rc_data.channel[REMOTE_CH_VRB] = 1250u;
    drive_source_update(&s, &r, true, true);
    assert(!s.manual_armed);
    r.rc_data.channel[REMOTE_CH_RIGHT_Y] = 1500u;
    drive_source_update(&s, &r, true, false);
    assert(!s.manual_armed);
    drive_source_update(&s, &r, true, true);
    assert(s.manual_armed);
    const uint16_t vrb_values[] = {1000u, 1249u, 1250u, 1251u, 1500u, 2000u};
    for(unsigned i = 0; i < sizeof(vrb_values) / sizeof(vrb_values[0]); ++i) {
        r.rc_data.channel[REMOTE_CH_VRB] = vrb_values[i];
        r.rc_data.channel[REMOTE_CH_RIGHT_Y] = 1500u;
        r.rc_data.channel[REMOTE_CH_SWD] = 1000u; r.manual_request = false;
        drive_source_update(&s, &r, true, true);
        r.rc_data.channel[REMOTE_CH_SWD] = 2000u; r.manual_request = true;
        drive_source_update(&s, &r, true, true);
        assert(s.manual_armed == (vrb_values[i] <= 1250u));
    }
    r.rc_data.channel[REMOTE_CH_VRB] = 1250u;
    drive_source_stop_manual(&s);
    drive_source_update(&s, &r, true, true);
    assert(!s.manual_armed && !s.release_seen); /* STOP cannot re-enable itself. */
    r.rc_data.channel[REMOTE_CH_SWC] = 1500u;
    assert(drive_source_update(&s, &r, true, true) && s.mode == DRIVE_SOURCE_HOST);
    r.rc_data.channel[REMOTE_CH_SWC] = 2000u;
    assert(drive_source_update(&s, &r, true, true) && s.manual_armed);
    /* An explicit online mode change acknowledges takeover after STOP. */
    drive_source_stop_manual(&s);
    r.rc_data.channel[REMOTE_CH_SWC]=1500u;
    drive_source_update(&s,&r,true,false);
    r.rc_data.channel[REMOTE_CH_SWC]=2000u;
    r.rc_data.channel[REMOTE_CH_RIGHT_Y]=2000u;
    drive_source_update(&s,&r,true,false);
    assert(s.release_seen && !s.manual_armed);
    r.rc_data.channel[REMOTE_CH_RIGHT_Y]=1500u;
    drive_source_update(&s,&r,true,true);
    assert(s.manual_armed);
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
    r = remote(); s = (DriveSource){0};
    r.rc_data.channel[REMOTE_CH_SWD] = 1001u; r.manual_request = false;
    drive_source_update(&s, &r, true, true);
    r.rc_data.channel[REMOTE_CH_SWD] = 1999u; r.manual_request = true;
    drive_source_update(&s, &r, true, true);
    assert(s.manual_armed);
    for(unsigned cycle = 0u; cycle < 100u; ++cycle) {
        r.rc_data.channel[REMOTE_CH_RIGHT_Y] = 2000u;
        drive_source_update(&s, &r, true, false);
        r.online = false;
        drive_source_update(&s, &r, true, false);
        assert(!s.manual_armed && s.release_seen && s.recovery_pending);
        r.online = true;
        drive_source_update(&s, &r, true, true);
        assert(!s.manual_armed); /* Reconnection cannot replay a held stick. */
        r.rc_data.channel[REMOTE_CH_RIGHT_Y] = 1500u;
        drive_source_update(&s, &r, true, false);
        assert(!s.manual_armed);
        r.rc_data.channel[REMOTE_CH_VRB] = 1251u;
        drive_source_update(&s, &r, true, true);
        assert(!s.manual_armed);
        r.rc_data.channel[REMOTE_CH_VRB] = 1250u;
        drive_source_update(&s, &r, true, true);
        assert(s.manual_armed && !s.recovery_pending);
    }
    r.online = false; drive_source_update(&s, &r, true, true);
    drive_source_update(&s, &r, false, true); /* A real fault while offline revokes the old permit. */
    assert(!s.release_seen && !s.recovery_pending);
    r.online = true; drive_source_update(&s, &r, true, true);
    assert(!s.manual_armed);
    puts("DRIVE_SOURCE_TEST_OK (exclusive SWC ownership, preserved SWD confirmation across handovers, manual VRB<=1250 boundary and re-arm, centred/stopped handover, fault/RC loss, manual gears)");
    return 0;
}
