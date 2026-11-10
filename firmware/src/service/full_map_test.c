#include "full_map_test.h"
#include "full_map_tree_positions.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* User-selected laser reading threshold; do not add the sensor-to-body offset.
 * Map envelope checks independently enforce the modeled vehicle geometry. */
#define FULL_MAP_MIN_LASER_RANGE_M .100f

const char* full_map_phase_name(FullMapPhase p) {
    static const char* const names[]={"IDLE","READY","ARMED","RUNNING","PAUSED","SEGMENT_DONE","DONE","FAULT"};
    return (unsigned)p<8?names[p]:"INVALID";
}
bool full_map_active(const FullMapTest* m) {
    return m->phase==MAP_ARMED||m->phase==MAP_RUNNING||m->phase==MAP_PAUSED;
}
void full_map_init(FullMapTest* m,uint32_t now) {
    memset(m,0,sizeof(*m));m->initialized=auto_route_build(&m->route,1,1.570796327f);
    m->phase=MAP_IDLE;m->reason="NEED_ORIGIN";m->speed=.20f;m->last_ms=now;
    for(unsigned row=1;row<=6;++row) for(unsigned number=1;number<=2;++number)
        if(!full_map_set_tree_x(m,row,number,full_map_tree_x_m[row-1][number-1])) m->initialized=false;
}
void full_map_stop(FullMapTest* m,const char* reason,bool lose_origin) {
    m->tree_hold=m->tree_hold_stable=false;
    m->aisle_correction.active=false;
    m->command=m->hint=(AutoTwist){0};auto_motion_cancel(&m->motion);
    m->phase=MAP_FAULT;m->reason=reason;m->prediction=(LaserPrediction){0};
    if(lose_origin) m->localization.registered=m->localization.pose.valid=false;
}
static AutoMotionConfig motion_config(float speed) {
    return (AutoMotionConfig){.vmax=speed,.accel=.10f,.decel=.10f,.lateral_max=.03f,.cross_kp=.8f,
        .yaw_kp=1.2f,.wz_max=.06f,.angular_accel=.15f,.position_tolerance=.02f,
        .cross_tolerance=.008f,.yaw_tolerance=.02f,.cross_abort=.08f,.yaw_abort=.15f,
        .settle_s=.4f,.stop_latency_s=.5f,.pose_max_age_ms=50,
        .finish_yaw_tolerance=.052359878f,.finish_cross_tolerance=.02f};
}
static float segment_speed(const FullMapTest* m) {
    const AutoRouteKind kind=m->route.segment[m->segment].kind;
    float speed=kind==AUTO_ROUTE_PARK?fminf(.05f,m->speed):m->speed;
    const MapAisleCorrection* a=&m->aisle_correction;
    if(kind==AUTO_ROUTE_SURVEY && a->confirmations && (!a->active || fabsf(a->error_m)>.008f)) {
        const uint16_t nearest=a->front_mm<a->rear_mm?a->front_mm:a->rear_mm;
        speed=fminf(speed,nearest<130u?.08f:.15f);
    }
    return speed;
}
bool full_map_set_tree_x(FullMapTest* m,unsigned row,unsigned number,float x) {
    if(!m||!m->initialized||full_map_active(m)||m->segment!=0u||row<1u||row>6u||number<1u||number>2u||!isfinite(x)) return false;
    const float low=row==4u?.6f:.4f,high=row==4u?2.4f:2.6f;
    if(x<low||x>high) return false;
    const MapTreePosition other=m->trees[row-1u][2u-number];
    if(other.configured && (number==1u?x+.05f>other.x:x-.05f<other.x)) return false;
    m->trees[row-1u][number-1u]=(MapTreePosition){x,true};return true;
}
static bool begin(FullMapTest* m,uint32_t now) {
    if(m->segment>=AUTO_ROUTE_COUNT) return false;
    if(m->tree_hold) return true; /* Resume a dwell without restarting or skipping its timer. */
    m->aisle_correction=(MapAisleCorrection){0};
    AutoMotionConfig c=motion_config(segment_speed(m));
    m->segment_ms=now;
    AutoSegment line=m->route.segment[m->segment].line;
    /* The two collinear geometry records form one continuous initial drive.
     * Do not brake/settle at the former entrance checkpoint. */
    if(m->segment==0u) line.end=m->route.segment[1].line.end;
    m->target_tree=0;
    const AutoRouteSegment route=m->route.segment[m->segment];
    if(route.kind==AUTO_ROUTE_SURVEY && route.row>=1u && route.row<=6u) {
        for(unsigned k=0;k<2u;++k) {
            const unsigned local=line.end.x>line.start.x?k:1u-k;
            const unsigned slot=(route.row-1u)*2u+local;
            if(m->trees[route.row-1u][local].configured && !(m->trees_done&(1u<<slot))) {
                line.end.x=m->trees[route.row-1u][local].x;
                m->target_tree=(uint8_t)(slot+1u);break;
            }
        }
    }
    return auto_motion_begin(&m->motion,&c,line,now);
}
bool full_map_command(FullMapTest* m,const char* cmd,FullMapInput in) {
    if(!m->initialized) full_map_init(m,in.now);
    if(!strcmp(cmd,"MAP STATUS")) return true;
    if(!strcmp(cmd,"MAP STOP")) {full_map_stop(m,"STOP",false);return true;}
    if(m->phase==MAP_FAULT && strcmp(cmd,"MAP ORIGIN")) return false;
    if(!strcmp(cmd,"MAP PAUSE") && (m->phase==MAP_RUNNING||m->phase==MAP_PAUSED)) {
        m->tree_hold_stable=false;
        m->phase=MAP_PAUSED;m->user_hold=true;m->pause_ms=in.now;
        m->command=m->hint=(AutoTwist){0};auto_motion_cancel(&m->motion);m->reason="USER_PAUSE";return true;
    }
    if(!strcmp(cmd,"MAP RESUME") && m->phase==MAP_PAUSED && m->user_hold && in.healthy && in.stationary) {
        m->user_hold=false;m->pause_ms=in.now;m->recovery_frames=0;m->stable=false;
        m->recovery_frame=in.laser.frame_count;m->reason="RESUME_CHECK";return true;
    }
    if(!strcmp(cmd,"MAP ORIGIN")) {
        if(full_map_active(m)||!in.odom_ok||!in.stationary) {m->reason="ORIGIN_NEEDS_STOPPED_FEEDBACK";return false;}
        const uint8_t front_bit=(uint8_t)(1u<<AUTO_FRONT);
        const uint32_t front_age=in.laser.sample_age_ms[AUTO_FRONT];
        if(!in.laser.frame_fresh||!(in.laser.valid_mask&front_bit)||
           !(in.laser.updated_mask&front_bit)||front_age>200u||
           !in.laser.accepted_mm[AUTO_FRONT]||in.laser.accepted_mm[AUTO_FRONT]>7500u) {
            m->reason="ORIGIN_FRONT_NOT_READY";return false;
        }
        AutoPose origin={.2f,-.5f,1.570796327f};AutoPlane front_plane;float expected_front;
        if(!field_pose_ray(origin,AUTO_FRONT,&front_plane,&expected_front)||front_plane.surface_id!=3u) {
            m->reason="ORIGIN_FRONT_WALL_NOT_ASSOCIATED";return false;
        }
        const float forward_projection=sinf(origin.yaw);
        if(!isfinite(forward_projection)||forward_projection<.95f) {
            m->reason="ORIGIN_HEADING_INVALID";return false;
        }
        origin.y+=(expected_front-(float)in.laser.accepted_mm[AUTO_FRONT]*.001f)/forward_projection;
        if(!isfinite(origin.y)||origin.y<-.80f||origin.y>0.0f) {
            m->reason="ORIGIN_OUTSIDE_ENTRY_ZONE";return false;
        }
        FieldPose calibrated;
        if(!field_pose_register(&calibrated,origin,in.imu_yaw,in.now)||
           !auto_route_set_entry_y(&m->route,origin.y)) {
            m->reason="ORIGIN_CALIBRATION_FAILED";return false;
        }
        m->localization=calibrated;m->origin_front_mm=in.laser.accepted_mm[AUTO_FRONT];
        m->segment=0;m->coverage=0;m->motion_error=AUTO_MOTION_NO_ERROR;
        m->trees_done=0;m->target_tree=0;m->tree_hold=m->tree_hold_stable=false;
        m->phase=MAP_READY;m->reason="ORIGIN_FRONT_ANCHORED";return true;
    }
    if(!strncmp(cmd,"MAP SPEED ",10)) {
        if(full_map_active(m)||!in.stationary) return false;
        const char* p=cmd+10;
        if(strcmp(p,"0.20")&&strcmp(p,"0.30")&&strcmp(p,"0.40")&&strcmp(p,"0.50")&&strcmp(p,"0.60")) return false;
        m->speed=strtof(p,NULL);m->reason="SPEED_SET";return true;
    }
    if(!strcmp(cmd,"MAP ARM ALL")) {
        if(full_map_active(m)||m->phase==MAP_FAULT||!m->localization.registered||
           m->segment>=AUTO_ROUTE_COUNT||!in.healthy||!in.stationary) {m->reason="ARM_NOT_READY";return false;}
        /* No tree/work peer or competition-start waiver: this is a separate
         * development test, with no valid-work/qualification claims. */
        const AutoPose p=m->localization.pose.pose;
        AutoVec2 start=m->route.segment[m->segment].line.start;
        if(hypotf(p.x-start.x,p.y-start.y)>.05f||fabsf(auto_wrap_angle(p.yaw-1.570796327f))>.08f) {
            m->reason="SEGMENT_START_MISMATCH";return false;
        }
        m->all=true;++m->token;if(!m->token)++m->token;
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
/* Optional paired ridge observation during horizontal aisle traversal. Empty
 * fields never match these short, opposing returns. Never modify raw ranges. */
static void correct_aisle_position(FullMapTest* m,FullMapInput in) {
    MapAisleCorrection* s=&m->aisle_correction;
    const AutoRouteSegment segment=m->route.segment[m->segment];
    const AutoPose pose=m->localization.pose.pose;
    const LaserSnapshot* l=&in.laser;
    if(segment.kind!=AUTO_ROUTE_SURVEY || segment.aisle==7u || !l->frame_fresh ||
       (l->valid_mask&12u)!=12u || (l->updated_mask&12u)!=12u ||
       l->sample_age_ms[AUTO_FRONT]>200u || l->sample_age_ms[AUTO_REAR]>200u ||
       fabsf(auto_wrap_angle(pose.yaw-1.570796327f))>.087266463f || fabsf(in.wheel.wz)>.1f)
        goto invalid;
    AutoPlane front,rear;float expected_front,expected_rear;
    const AutoLaserMount fm={.origin={.04098f,0},.yaw=0};
    const AutoLaserMount rm={.origin={-.04098f,0},.yaw=3.141592654f};
    if(!auto_route_ray(&m->route,pose,fm,7.5f,&front,&expected_front) ||
       !auto_route_ray(&m->route,pose,rm,7.5f,&rear,&expected_rear) ||
       front.normal.y<.9f || rear.normal.y>-.9f ||
       (front.surface_id<10u && rear.surface_id<10u) ||
       expected_front>.6f || expected_rear>.6f)
        goto invalid;
    /* Keep away from ridge ends: slight yaw or pose error can change targets. */
    for(unsigned row=1;row<=6;++row) {
        if((front.surface_id>=10u+4u*row && front.surface_id<=13u+4u*row) ||
           (rear.surface_id>=10u+4u*row && rear.surface_id<=13u+4u*row)) {
            const float low=row==4u?.6f:.4f, high=row==4u?2.4f:2.6f;
            if(pose.x<low+.08f || pose.x>high-.08f) goto invalid;
        }
    }
    const uint16_t f=l->accepted_mm[AUTO_FRONT],r=l->accepted_mm[AUTO_REAR];
    if(!f||!r||f>600u||r>600u ||
       fabsf((f+r)*.001f-expected_front-expected_rear)>.08f) goto invalid;
    const float dy=((r*.001f-expected_rear)-(f*.001f-expected_front))*.5f*sinf(pose.yaw);
    const float observed_y=pose.y+dy;
    if(fabsf(observed_y-segment.line.start.y)>.08f) goto invalid;
    if(!s->checkpoint_valid || front.surface_id!=s->front_surface || rear.surface_id!=s->rear_surface ||
       abs((int)f-s->checkpoint_front_mm)>5 || abs((int)r-s->checkpoint_rear_mm)>5) {
        s->checkpoint_valid=true;s->frozen=false;
        s->checkpoint_front_mm=f;s->checkpoint_rear_mm=r;
        s->checkpoint_wheel_y=m->localization.wheel_position.y;
    } else if(fabsf(m->localization.wheel_position.y-s->checkpoint_wheel_y)>.03f) s->frozen=true;
    if(s->frozen) goto invalid;
    if(l->frame_count==s->frame || (s->confirmations && l->last_rx_ms==s->stamp_ms)) return;
    const uint32_t elapsed=l->last_rx_ms-s->stamp_ms;
    if(!s->confirmations || elapsed>200u || front.surface_id!=s->front_surface ||
       rear.surface_id!=s->rear_surface || abs((int)f-s->front_mm)>50 || abs((int)r-s->rear_mm)>50)
        s->confirmations=0;
    s->frame=l->frame_count;s->stamp_ms=l->last_rx_ms;
    s->front_surface=front.surface_id;s->rear_surface=rear.surface_id;s->front_mm=f;s->rear_mm=r;
    if(s->confirmations<3u) ++s->confirmations;
    s->error_m=observed_y-segment.line.start.y;s->correction_m=0;
    s->active=s->confirmations>=3u;
    if(!s->active) return;
    /* Limit observation corrections to 20mm/s and 2mm per fresh pair. The
     * existing motion controller then drives back toward the nominal aisle. */
    const float limit=fminf(.002f,.02f*fminf(elapsed*.001f,.2f));
    s->correction_m=fmaxf(-limit,fminf(limit,dy*.2f));
    m->localization.pose.pose.y+=s->correction_m;
    m->localization.used_mask|=12u;m->localization.constrained_axes|=2u;
    m->localization.rejected_mask&=(uint8_t)~12u;
    m->localization.uncorrected_m[1]=0;
    return;
invalid:
    s->active=false;s->confirmations=0;s->error_m=s->correction_m=0;
}
void full_map_step(FullMapTest* m,FullMapInput in) {
    if(!m->initialized) full_map_init(m,in.now);
    const AutoTwist previous_command=m->command;
    m->command=m->hint=(AutoTwist){0};
    LaserSnapshot observation=in.laser;
    /* In G7 the south-facing ray crosses R6 and changes target at its end.
     * It must not become a south-fence position observation during this leg.
     * Keep the north/front fence and east/west observations. The original
     * snapshot remains intact for freshness and collision protection. */
    if(m->segment<AUTO_ROUTE_COUNT && m->route.segment[m->segment].aisle==7u) {
        observation.valid_mask&=(uint8_t)~(1u<<AUTO_REAR);
        observation.updated_mask&=(uint8_t)~(1u<<AUTO_REAR);
    }
    if(m->localization.registered && !field_pose_step(&m->localization,in.wheel,in.imu_yaw,in.odom_ok,&observation,in.now)) {
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
    correct_aisle_position(m,in);
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
        if((in.laser.valid_mask&bit)&&in.laser.accepted_mm[i]*.001f<=FULL_MAP_MIN_LASER_RANGE_M) safe=false;
        const float speed=fmaxf(toward[i],commanded[i]);
        const float budget=FULL_MAP_MIN_LASER_RANGE_M+speed*.5f+speed*speed/.2f;
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
        m->tree_hold_stable=false;
        if(m->phase!=MAP_PAUSED) {
            m->phase=MAP_PAUSED;m->pause_ms=in.now;m->stable=false;m->recovery_frames=0;
            m->recovery_frame=in.laser.frame_count;auto_motion_cancel(&m->motion);
        }
        m->reason=pose_limited?"POSE_OBSERVATION_PAUSE":"RANGE_PAUSE";
    }
    if(m->phase==MAP_PAUSED) {
        if(in.now-m->pause_ms>=5000u) {
            const char* timeout_reason="RANGE_TIMEOUT";
            if(!strcmp(m->reason,"POSE_OBSERVATION_PAUSE")) timeout_reason="POSE_OBSERVATION_TIMEOUT";
            else if(!strcmp(m->reason,"RANGE_PAUSE")) timeout_reason="RANGE_PAUSE_TIMEOUT";
            full_map_stop(m,timeout_reason,false);return;
        }
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
    if(m->tree_hold) {
        const AutoPose p=m->localization.pose.pose;
        const AutoVec2 target=m->motion.segment.end;
        if(fabsf(p.x-target.x)>.02f || fabsf(p.y-target.y)>.02f ||
           fabsf(auto_wrap_angle(p.yaw-m->motion.segment.heading))>.052359878f) {
            m->tree_hold=m->tree_hold_stable=false;
            if(!begin(m,in.now)) full_map_stop(m,"TREE_REPOSITION_FAILED",false);
            else m->reason="TREE_REPOSITION";
            return;
        }
        if(!in.stationary || hypotf(in.wheel.vx,in.wheel.vy)>=.01f || fabsf(in.wheel.wz)>=.02f) {
            m->tree_hold_stable=false;return;
        }
        if(!m->tree_hold_stable) {m->tree_hold_stable=true;m->tree_hold_ms=in.now;}
        m->reason="TREE_DWELL";
        if(in.now-m->tree_hold_ms<2000u) return;
        m->trees_done|=(uint16_t)(1u<<(m->target_tree-1u));
        m->tree_hold=m->tree_hold_stable=false;
        if(!begin(m,in.now)) full_map_stop(m,"TREE_NEXT_FAILED",false);
        else m->reason="TREE_NEXT";
        return;
    }
    AutoMotionInput input={.pose=m->localization.pose,.measured=in.wheel,.healthy=true,
        .aligned=in.aligned,.stationary=in.stationary,.now_ms=in.now};
    AutoMotionOutput out=auto_motion_step(&m->motion,input);
    if(out.phase==AUTO_MOTION_FAULT) {m->motion_error=out.error;full_map_stop(m,"MOTION_FAULT",false);return;}
    if(out.progress<-.05f) {full_map_stop(m,"WRONG_DIRECTION",false);return;}
    m->command=out.command;m->hint=out.steering_hint;
    if(m->predicted_mask) {
        /* Prediction is a degraded interval; request
         * braking via the motion profile on the next step, not fictitious fixes. */
        m->motion.config.vmax=fminf(segment_speed(m),LASER_PREDICT_SPEED_CAP_MPS);
    } else m->motion.config.vmax=segment_speed(m);
    if(out.phase==AUTO_MOTION_DONE) {
        if(m->target_tree) {
            m->tree_hold=true;m->tree_hold_stable=false;
            m->command=m->hint=(AutoTwist){0};m->reason="TREE_DWELL";return;
        }
        const unsigned aisle=m->route.segment[m->segment].aisle;
        if(aisle) m->coverage|=(uint8_t)(1u<<(aisle-1));
        m->segment+=(m->segment==0u?2u:1u);m->command=m->hint=(AutoTwist){0};
        if(m->segment==AUTO_ROUTE_COUNT) {m->phase=MAP_DONE;m->reason="ROUTE_COMPLETE";return;}
        if(!begin(m,in.now)) {full_map_stop(m,"MOTION_CONFIG",false);return;}
        m->reason="NEXT_SEGMENT";
    }
}
