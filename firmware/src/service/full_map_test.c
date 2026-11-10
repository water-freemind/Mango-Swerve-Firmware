#include "full_map_test.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const char* full_map_phase_name(FullMapPhase p) {
    static const char* const names[]={"IDLE","READY","ARMED","RUNNING","PAUSED","SEGMENT_DONE","DONE","FAULT"};
    return (unsigned)p<8?names[p]:"INVALID";
}
bool full_map_active(const FullMapTest* m) {
    return m->phase==MAP_ARMED||m->phase==MAP_RUNNING||m->phase==MAP_PAUSED;
}
void full_map_init(FullMapTest* m,uint32_t now) {
    memset(m,0,sizeof(*m));m->initialized=auto_route_build(&m->route,1,1.570796327f);
    m->phase=MAP_IDLE;m->reason="NEED_ORIGIN";m->speed=.10f;m->last_ms=now;
}
void full_map_stop(FullMapTest* m,const char* reason,bool lose_origin) {
    m->command=m->hint=(AutoTwist){0};auto_motion_cancel(&m->motion);
    m->phase=MAP_FAULT;m->reason=reason;m->prediction=(LaserPrediction){0};
    if(lose_origin) m->localization.registered=m->localization.pose.valid=false;
}
static AutoMotionConfig motion_config(float speed) {
    return (AutoMotionConfig){.vmax=speed,.accel=.10f,.decel=.10f,.lateral_max=.03f,.cross_kp=.8f,
        .yaw_kp=1.2f,.wz_max=.06f,.angular_accel=.15f,.position_tolerance=.01f,
        .cross_tolerance=.008f,.yaw_tolerance=.02f,.cross_abort=.08f,.yaw_abort=.15f,
        .settle_s=.4f,.stop_latency_s=.5f,.pose_max_age_ms=50};
}
static float segment_speed(const FullMapTest* m) {
    const AutoRouteKind kind=m->route.segment[m->segment].kind;
    return kind==AUTO_ROUTE_ENTER||kind==AUTO_ROUTE_PARK?fminf(.05f,m->speed):m->speed;
}
static bool begin(FullMapTest* m,uint32_t now) {
    if(m->segment>=AUTO_ROUTE_COUNT) return false;
    AutoMotionConfig c=motion_config(segment_speed(m));
    m->segment_ms=now;
    return auto_motion_begin(&m->motion,&c,m->route.segment[m->segment].line,now);
}
bool full_map_command(FullMapTest* m,const char* cmd,FullMapInput in) {
    if(!m->initialized) full_map_init(m,in.now);
    if(!strcmp(cmd,"MAP STATUS")) return true;
    if(!strcmp(cmd,"MAP STOP")) {full_map_stop(m,"STOP",false);return true;}
    if(m->phase==MAP_FAULT && strcmp(cmd,"MAP ORIGIN")) return false;
    if(!strcmp(cmd,"MAP PAUSE") && (m->phase==MAP_RUNNING||m->phase==MAP_PAUSED)) {
        m->phase=MAP_PAUSED;m->user_hold=true;m->pause_ms=in.now;
        m->command=m->hint=(AutoTwist){0};auto_motion_cancel(&m->motion);m->reason="USER_PAUSE";return true;
    }
    if(!strcmp(cmd,"MAP RESUME") && m->phase==MAP_PAUSED && m->user_hold && in.healthy && in.stationary) {
        m->user_hold=false;m->pause_ms=in.now;m->recovery_frames=0;m->stable=false;
        m->recovery_frame=in.laser.frame_count;m->reason="RESUME_CHECK";return true;
    }
    if(!strcmp(cmd,"MAP ORIGIN")) {
        if(full_map_active(m)||!in.odom_ok||!in.stationary) {m->reason="ORIGIN_NEEDS_STOPPED_FEEDBACK";return false;}
        if(!field_pose_register(&m->localization,(AutoPose){.2f,-.5f,1.570796327f},in.imu_yaw,in.now)) return false;
        m->segment=0;m->coverage=0;m->motion_error=AUTO_MOTION_NO_ERROR;
        m->phase=MAP_READY;m->reason="ORIGIN_SET";return true;
    }
    if(!strncmp(cmd,"MAP SPEED ",10)) {
        if(full_map_active(m)||!in.stationary) return false;
        const char* p=cmd+10;
        if(strcmp(p,"0.05")&&strcmp(p,"0.10")&&strcmp(p,"0.15")&&strcmp(p,"0.20")&&strcmp(p,"0.30")) return false;
        m->speed=strtof(p,NULL);m->reason="SPEED_SET";return true;
    }
    if(!strcmp(cmd,"MAP ARM ALL")||!strcmp(cmd,"MAP ARM NEXT")) {
        if(full_map_active(m)||m->phase==MAP_FAULT||!m->localization.registered||
           m->segment>=AUTO_ROUTE_COUNT||!in.healthy||!in.stationary) {m->reason="ARM_NOT_READY";return false;}
        /* No tree/work peer or competition-start waiver: this is a separate
         * development test, with no valid-work/qualification claims. */
        const AutoPose p=m->localization.pose.pose;
        AutoVec2 start=m->route.segment[m->segment].line.start;
        if(hypotf(p.x-start.x,p.y-start.y)>.05f||fabsf(auto_wrap_angle(p.yaw-1.570796327f))>.08f) {
            m->reason="SEGMENT_START_MISMATCH";return false;
        }
        m->all=!strcmp(cmd,"MAP ARM ALL");++m->token;if(!m->token)++m->token;
        m->phase=MAP_ARMED;m->arm_ms=in.now;m->reason="ARMED";return true;
    }
    if(!strncmp(cmd,"MAP GO ",7)) {
        char expected[32];snprintf(expected,sizeof(expected),"MAP GO %lu",(unsigned long)m->token);
        if(strcmp(cmd,expected)||m->phase!=MAP_ARMED||in.now-m->arm_ms>4000u||!in.healthy||!in.stationary) return false;
        if(!begin(m,in.now)) return false;
        m->prediction=(LaserPrediction){0};m->run_ms=in.now;m->last_ms=in.now;
        m->user_hold=false;m->phase=MAP_RUNNING;m->reason="RUNNING";return true;
    }
    return false;
}
void full_map_step(FullMapTest* m,FullMapInput in) {
    if(!m->initialized) full_map_init(m,in.now);
    const AutoTwist previous_command=m->command;
    m->command=m->hint=(AutoTwist){0};
    if(m->localization.registered && !field_pose_step(&m->localization,in.wheel,in.imu_yaw,in.odom_ok,&in.laser,in.now)) {
        if(full_map_active(m)) full_map_stop(m,"ODOMETRY_INVALID",true);
    }
    if(m->phase==MAP_ARMED) {
        if(!in.healthy||in.now-m->arm_ms>4000u) full_map_stop(m,"ARM_EXPIRED_OR_PERMIT",false);
        return;
    }
    if(m->phase!=MAP_RUNNING&&m->phase!=MAP_PAUSED) return;
    if(!in.healthy||!m->localization.pose.valid||in.now-m->last_ms>50u) {
        full_map_stop(m,"FEEDBACK_OR_PERMIT",!in.odom_ok);return;
    }
    m->last_ms=in.now;
    if(m->phase==MAP_PAUSED && m->user_hold) {m->reason="USER_PAUSE";return;}
    if(in.now-m->run_ms>900000u||in.now-m->segment_ms>120000u) {full_map_stop(m,"RUN_TIMEOUT",false);return;}
    if(!auto_route_pose_clear(&m->route,(AutoEnvelope){.155f,.155f,.155f,.155f},m->localization.pose.pose,.01f)) {
        full_map_stop(m,"MAP_ENVELOPE",false);return;
    }
    LaserPredictionOutput range=laser_prediction_step(&m->prediction,&in.laser,in.now,
        in.wheel.vx,in.wheel.vy,in.imu_yaw,m->phase==MAP_RUNNING);
    m->predicted_mask=range.mask;
    /* Necessary ray follows both measured motion and the next segment direction;
     * entering an open start/gate ray relies on the registered map envelope.
     * Other failed directions do not invalidate localization/navigation. */
    const AutoSegment seg=m->route.segment[m->segment].line;
    AutoVec2 direction=auto_rotate((AutoVec2){seg.end.x-seg.start.x,seg.end.y-seg.start.y},-m->localization.pose.pose.yaw);
    const float len=hypotf(direction.x,direction.y);direction.x/=len;direction.y/=len;
    const float toward[4]={fmaxf(0,in.wheel.vy),fmaxf(0,-in.wheel.vy),fmaxf(0,-in.wheel.vx),fmaxf(0,in.wheel.vx)};
    const float commanded[4]={fmaxf(0,previous_command.vy),fmaxf(0,-previous_command.vy),
        fmaxf(0,-previous_command.vx),fmaxf(0,previous_command.vx)};
    const float ahead[4]={direction.y,-direction.y,-direction.x,direction.x};
    const bool pose_limited=m->localization.uncorrected_m[0]>.4f||m->localization.uncorrected_m[1]>.4f;
    uint8_t required=0;bool safe=!pose_limited;
    for(unsigned i=0;i<4;++i) {
        AutoPlane plane;float predicted;
        const bool wall=field_pose_ray(m->localization.pose.pose,i,&plane,&predicted);
        const uint8_t bit=(uint8_t)(1u<<i);
        /* Visible near returns remain protective even when a map ray has no wall. */
        const float edge=.155f-.04098f;
        if((in.laser.valid_mask&bit)&&in.laser.accepted_mm[i]*.001f<=edge+.02f) safe=false;
        const float speed=fmaxf(toward[i],commanded[i]);
        const float budget=edge+.02f+speed*.5f+speed*speed/.2f;
        if((ahead[i]>.2f||toward[i]>.01f) && (in.laser.valid_mask&bit) &&
           in.laser.accepted_mm[i]*.001f<=budget) safe=false;
        if(!wall) continue;
        if(ahead[i]>.2f||toward[i]>.01f) required|=bit;
        if(!(required&bit)) continue;
        if(!in.laser.frame_fresh||!((in.laser.valid_mask|range.mask)&bit)) {safe=false;continue;}
        if(range.mm[i]*.001f<=budget) safe=false;
    }
    m->required_mask=required;
    if(!safe) {
        if(m->phase!=MAP_PAUSED) {
            m->phase=MAP_PAUSED;m->pause_ms=in.now;m->stable=false;m->recovery_frames=0;
            m->recovery_frame=in.laser.frame_count;auto_motion_cancel(&m->motion);
        }
        m->reason=pose_limited?"POSE_OBSERVATION_PAUSE":"RANGE_PAUSE";
    }
    if(m->phase==MAP_PAUSED) {
        if(in.now-m->pause_ms>=5000u) {full_map_stop(m,"RANGE_TIMEOUT",false);return;}
        if(!safe||!in.stationary) {m->stable=false;m->recovery_frames=0;return;}
        if(in.laser.frame_count!=m->recovery_frame) {
            m->recovery_frame=in.laser.frame_count;
            if((in.laser.updated_mask&required)==required) ++m->recovery_frames;
            else m->recovery_frames=0;
        }
        if(m->recovery_frames<2u) return;
        if(!m->stable) {m->stable=true;m->stable_ms=in.now;}
        if(in.now-m->stable_ms<200u) return;
        if(!begin(m,in.now)) {full_map_stop(m,"MOTION_CONFIG",false);return;}
        m->phase=MAP_RUNNING;m->reason="RESUMED";
    }
    AutoMotionInput input={.pose=m->localization.pose,.measured=in.wheel,.healthy=true,
        .aligned=in.aligned,.stationary=in.stationary,.now_ms=in.now};
    AutoMotionOutput out=auto_motion_step(&m->motion,input);
    if(out.phase==AUTO_MOTION_FAULT) {m->motion_error=out.error;full_map_stop(m,"MOTION_FAULT",false);return;}
    if(out.progress<-.05f) {full_map_stop(m,"WRONG_DIRECTION",false);return;}
    m->command=out.command;m->hint=out.steering_hint;
    if(m->predicted_mask) {
        /* Full-map test max .30m/s. Prediction is a degraded interval; request
         * braking via the motion profile on the next step, not fictitious fixes. */
        m->motion.config.vmax=fminf(segment_speed(m),LASER_PREDICT_SPEED_CAP_MPS);
    } else m->motion.config.vmax=segment_speed(m);
    if(out.phase==AUTO_MOTION_DONE) {
        const unsigned aisle=m->route.segment[m->segment].aisle;
        if(aisle) m->coverage|=(uint8_t)(1u<<(aisle-1));
        ++m->segment;m->command=m->hint=(AutoTwist){0};
        if(m->segment==AUTO_ROUTE_COUNT) {m->phase=MAP_DONE;m->reason="ROUTE_COMPLETE";return;}
        if(!m->all) {m->phase=MAP_SEGMENT_DONE;m->reason="CLICK_ARM_NEXT";return;}
        if(!begin(m,in.now)) {full_map_stop(m,"MOTION_CONFIG",false);return;}
        m->reason="NEXT_SEGMENT";
    }
}
