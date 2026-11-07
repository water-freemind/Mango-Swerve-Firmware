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
}
bool drive_source_update(DriveSource* s, const RemoteState* r, bool healthy, bool stationary) {
    const DriveSourceMode mode = drive_source_requested(r);
    const bool changed = s->mode != mode;
    if(changed) { s->mode = mode; drive_source_stop_manual(s); }
    if(!r->online || !healthy) { drive_source_stop_manual(s); return changed; }
    if(r->rc_data.channel[REMOTE_CH_SWD] == REMOTE_SW_HIGH) {
        s->manual_armed = false;
        s->release_seen = true;
        return changed;
    }
    if(mode != DRIVE_SOURCE_MANUAL || !r->manual_request ||
       r->rc_data.channel[REMOTE_CH_VRA] <= REMOTE_VR_LOW_THRESHOLD) {
        drive_source_stop_manual(s);
        return changed;
    }
    if(!s->manual_armed && s->release_seen && stationary && drive_source_centered(r)) s->manual_armed = true;
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
