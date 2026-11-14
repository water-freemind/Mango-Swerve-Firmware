#include "drive_source.h"
#include "app_config.h"
#include <math.h>
#include <stdlib.h>

DriveSourceMode drive_source_requested(const RemoteState* r) {
    if(!r->online || r->manual_source != REMOTE_MANUAL_SOURCE_CHASSIS) return DRIVE_SOURCE_STOP;
    const unsigned swc = r->rc_data.channel[REMOTE_CH_SWC];
    if(swc >= 1400u && swc <= 1600u) return DRIVE_SOURCE_HOST;
    if(swc >= 1900u && swc <= 2100u) return DRIVE_SOURCE_MANUAL;
    return DRIVE_SOURCE_STOP;
}
const char* drive_source_name(DriveSourceMode mode) {
    return mode == DRIVE_SOURCE_HOST ? "HOST" : mode == DRIVE_SOURCE_MANUAL ? "MANUAL" : "STOP";
}
bool drive_source_centered(const RemoteState* r) {
    const unsigned ch[] = {REMOTE_CH_RIGHT_X, REMOTE_CH_RIGHT_Y, REMOTE_CH_LEFT_X};
    for(unsigned i = 0u; i < 3u; ++i)
        if((unsigned)abs((int)r->rc_data.channel[ch[i]] - (int)REMOTE_CENTER) > REMOTE_DEADBAND) return false;
    return true;
}
void drive_source_stop_manual(DriveSource* s) {
    s->manual_armed = s->release_seen = false;
    s->recovery_pending = false;
}
bool drive_source_manual_vrb_ok(const RemoteState* r) {
    return r->rc_data.channel[REMOTE_CH_VRB] <= DRIVE_MANUAL_VRB_MAX;
}
const char* drive_source_manual_wait_reason(const DriveSource* s, const RemoteState* r,
                                          bool healthy, bool stationary) {
    if(!r->online) return "RC_OFFLINE";
    if(s->mode != DRIVE_SOURCE_MANUAL) return "SELECT_MANUAL";
    if(!r->manual_request || r->rc_data.channel[REMOTE_CH_VRA] <= REMOTE_VR_LOW_THRESHOLD)
        return "REMOTE_PERMIT";
    if(!healthy) return "HARDWARE";
    if(!drive_source_manual_vrb_ok(r)) return "VRB_RELEASED";
    if(!s->release_seen) return "SWD_REARM";
    if(s->manual_armed) return "NONE";
    if(!drive_source_centered(r)) return "STICKS_NOT_CENTERED";
    if(!stationary) return "WHEELS_MOVING";
    return "NONE";
}
bool drive_source_update(DriveSource* s, const RemoteState* r, bool healthy, bool stationary) {
    const DriveSourceMode mode = drive_source_requested(r);
    const bool changed = s->mode != mode;
    const bool was_armed = s->manual_armed;
    const bool was_online=s->online_seen;
    s->online_seen=r->online;
    if(was_armed) s->recovery_pending = true;
    if(changed) {
        s->mode = mode;
        s->manual_armed = false;
    }
    if(!r->online) {
        s->manual_armed = false;
        if(!healthy && s->recovery_pending) drive_source_stop_manual(s);
        return changed;
    }
    if(remote_switch_matches(r->rc_data.channel[REMOTE_CH_SWD], REMOTE_SW_HIGH)) {
        s->manual_armed = false;
        s->release_seen = true;
        s->recovery_pending = false;
        return changed;
    }
    if(!healthy) {
        if(s->recovery_pending) drive_source_stop_manual(s);
        return changed;
    }
    if(!r->manual_request || r->manual_source != REMOTE_MANUAL_SOURCE_CHASSIS ||
       r->rc_data.channel[REMOTE_CH_VRA] <= REMOTE_VR_LOW_THRESHOLD) {
        drive_source_stop_manual(s);
        return changed;
    }
    /* Keep the observed SWD release across SWC handovers, including intermediate
     * switch positions. Each new manual owner still waits for centre and stop. */
    if(mode != DRIVE_SOURCE_MANUAL) return changed;
    if(!drive_source_manual_vrb_ok(r)) {
        /* Release stops motion immediately. Keep SWD confirmation so a new
         * press can re-enable only after neutral sticks and stationary wheels. */
        s->manual_armed = false;
        return changed;
    }
    if(changed && was_online) s->release_seen=true;
    if(!s->manual_armed && s->release_seen && stationary && drive_source_centered(r)) s->manual_armed = true;
    if(s->manual_armed) s->recovery_pending = false;
    return changed;
}
static float normalized(uint16_t value) {
    int delta = (int)value - (int)REMOTE_CENTER;
    if(abs(delta) <= (int)REMOTE_DEADBAND) return 0.0f;
    return fmaxf(-1.0f, fminf(1.0f, (float)delta / REMOTE_SPAN));
}
DriveVelocity drive_source_velocity(const RemoteState* r) {
    const unsigned gear = r->rc_data.channel[REMOTE_CH_SWB];
    float linear = gear >= 1750u ? 1.5f : gear <= 1250u ? 0.4f : 0.8f;
    float angular = gear >= 1750u ? 6.0f : gear <= 1250u ? 1.5f : 3.0f;
    linear = fminf(linear, APP_MANUAL_MAX_LINEAR_MPS);
    angular = fminf(angular, APP_MANUAL_MAX_ANGULAR_RAD_S);
    DriveVelocity v = {.vx = normalized(r->rc_data.channel[REMOTE_CH_RIGHT_Y]) * linear,
        .vy = -normalized(r->rc_data.channel[REMOTE_CH_RIGHT_X]) * linear,
        .wz = -normalized(r->rc_data.channel[REMOTE_CH_LEFT_X]) * angular};
    const float speed = hypotf(v.vx, v.vy);
    if(speed > linear) { v.vx *= linear / speed; v.vy *= linear / speed; }
    return v;
}
