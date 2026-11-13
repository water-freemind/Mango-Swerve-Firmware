#include "full_map_test.h"
#include "full_map_tree_positions.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* User-selected laser reading threshold; do not add the sensor-to-body offset.
 * Map envelope checks independently enforce the modeled vehicle geometry. */
#define FULL_MAP_MIN_LASER_RANGE_M .100f
#define FULL_MAP_ENTRY_APPROACH_M 1.50f

const char* full_map_phase_name(FullMapPhase p) {
    static const char* const names[]={"IDLE","READY","ARMED","RUNNING","PAUSED","SEGMENT_DONE","DONE","FAULT"};
    return (unsigned)p<8?names[p]:"INVALID";
}
bool full_map_active(const FullMapTest* m) {
    return m->phase==MAP_ARMED||m->phase==MAP_RUNNING||m->phase==MAP_PAUSED;
}
void full_map_init(FullMapTest* m,uint32_t now) {
    memset(m,0,sizeof(*m));m->initialized=auto_route_build(&m->route,1,1.570796327f);
    m->phase=MAP_IDLE;m->reason="NEED_ORIGIN";m->speed=.30f;m->last_ms=now;
    m->approach_speed=.10f;
    for(unsigned row=1;row<=6;++row) for(unsigned number=1;number<=2;++number)
        if(!full_map_set_tree_x(m,row,number,full_map_tree_x_m[row-1][number-1])) m->initialized=false;
}
void full_map_stop(FullMapTest* m,const char* reason,bool lose_origin) {
    m->tree_hold=m->tree_hold_stable=false;
    m->aisle_correction.active=false;
    m->aisle_correction.centering=false;
    m->wall_correction=(MapWallCorrection){0};
    m->cross_feedback=(MapCrossFeedback){0};
    m->entry=(MapEntryGuidance){0};m->odom_range_mask=0;
    memset(m->far_handover,0,sizeof(m->far_handover));
    m->command=m->hint=(AutoTwist){0};auto_motion_cancel(&m->motion);
    m->phase=MAP_FAULT;m->reason=reason;m->prediction=(LaserPrediction){0};
    m->laser_motion=(LaserMotionFilter){0};
    if(lose_origin) m->localization.registered=m->localization.pose.valid=false;
}
static AutoMotionConfig motion_config(float speed) {
    /* Field tuning 2026-11-12: faster ramps and earlier heading/aisle correction.
     * The independent laser braking model uses 0.10 m/s^2. Endpoint recovery
     * first settles the outgoing motion, then returns at up to 0.05 m/s. */
    return (AutoMotionConfig){.vmax=speed,.accel=.45f,.decel=.60f,.lateral_max=.10f,.cross_kp=3.2f,
        .yaw_kp=1.6f,.wz_max=.10f,.angular_accel=.30f,.position_tolerance=.02f,
        .cross_tolerance=.005f,.yaw_tolerance=.017453293f,.cross_abort=.08f,.yaw_abort=.15f,
        .settle_s=.4f,.stop_latency_s=.35f,.pose_max_age_ms=50,
        .finish_yaw_tolerance=.069813170f,.finish_cross_tolerance=.02f,.overshoot_recovery=.06f,
        .smooth_correction=true};
}
static float segment_speed(const FullMapTest* m) {
    const AutoRouteKind kind=m->route.segment[m->segment].kind;
    /* Field tuning 2026-11-11: the parking/exit leg carries no ridge work and is
     * only ~1 m through the open entry gap, so 0.10 m/s halves its cost. */
    float speed=kind==AUTO_ROUTE_PARK?fminf(.15f,m->speed):m->speed;
    const MapAisleCorrection* a=&m->aisle_correction;
    if(kind==AUTO_ROUTE_SURVEY && a->confirmations) {
        const uint16_t nearest=a->front_mm<a->rear_mm?a->front_mm:a->rear_mm;
        const float clearance=fmaxf(0,fminf(1,((float)nearest-130.0f)/50.0f));
        /* Field tuning 2026-11-11: 0.08..0.15 m/s made the G1..G6 aisles the
         * dominant cost of a run. 0.12..0.22 still leaves the 0.4 m unobserved
         * budget ~2 s of correction window at the slow end, while the laser
         * braking budget below keeps deciding every genuine approach. */
        const float correction_speed=.20f+.15f*clearance;
        const float error_fraction=a->active ?
            fmaxf(0,fminf(1,(fabsf(a->error_m)-.025f)/.035f)):1.0f;
        speed=fminf(speed,speed+(correction_speed-speed)*error_fraction);
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
    AutoMotionConfig c=motion_config(segment_speed(m));
    if(m->segment==0u) {
        c.yaw_tolerance=.017453293f;c.yaw_kp=2.0f;c.wz_max=.15f;c.angular_accel=.30f;
    }
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
    if(!m->target_tree && m->segment+1u<AUTO_ROUTE_COUNT)
        c.settle_s=.15f;
    if(m->target_tree) {
        const float direction=line.end.x>line.start.x?1.0f:-1.0f;
        const float past=(m->localization.pose.pose.x-line.end.x)*direction;
        /* Reacquiring a nearby tree after a pause may need a short reverse.
         * Use its actual start, while preserving the aisle's nominal Y line. */
        if(past>0 && past<=.06f) line.start.x=m->localization.pose.pose.x;
    }
    return auto_motion_begin(&m->motion,&c,line,now);
}
bool full_map_command(FullMapTest* m,const char* cmd,FullMapInput in) {
    if(!m->initialized) full_map_init(m,in.now);
    if(!strcmp(cmd,"MAP STATUS")) return true;
    if(!strcmp(cmd,"MAP STOP")) {full_map_stop(m,"STOP",false);return true;}
    const bool setting=!strncmp(cmd,"MAP SPEED ",10) || !strncmp(cmd,"MAP APPROACH ",13);
    if(m->phase==MAP_FAULT && strcmp(cmd,"MAP ORIGIN") && !setting) return false;
    if(!strcmp(cmd,"MAP PAUSE") && (m->phase==MAP_RUNNING||m->phase==MAP_PAUSED)) {
        m->tree_hold_stable=false;
        m->phase=MAP_PAUSED;m->user_hold=true;m->pause_ms=in.now;
        m->command=m->hint=(AutoTwist){0};auto_motion_cancel(&m->motion);m->reason="USER_PAUSE";return true;
    }
    if(!strcmp(cmd,"MAP RESUME") && m->phase==MAP_PAUSED && m->user_hold && in.healthy && in.stationary) {
        m->user_hold=false;m->pause_ms=in.now;m->recovery_frames=0;m->recovery_updates=0;m->stable=false;
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
        m->aisle_correction=(MapAisleCorrection){0};
        m->wall_correction=(MapWallCorrection){0};
        m->cross_feedback=(MapCrossFeedback){0};m->entry=(MapEntryGuidance){0};
        m->segment=0;m->coverage=0;m->motion_error=AUTO_MOTION_NO_ERROR;
        m->trees_done=0;m->target_tree=0;m->tree_hold=m->tree_hold_stable=false;
        m->phase=MAP_READY;m->reason="ORIGIN_FRONT_ANCHORED";return true;
    }
    if(!strncmp(cmd,"MAP SPEED ",10)) {
        if(full_map_active(m)||!in.stationary) return false;
        const char* p=cmd+10;
        if(strcmp(p,"0.20")&&strcmp(p,"0.30")&&strcmp(p,"0.40")&&strcmp(p,"0.50")&&strcmp(p,"0.60")&&strcmp(p,"0.70")) return false;
        m->speed=strtof(p,NULL);
        if(m->phase!=MAP_FAULT) m->reason="SPEED_SET";
        return true;
    }
    if(!strncmp(cmd,"MAP APPROACH ",13)) {
        const char* value=cmd+13;
        if(full_map_active(m)||!in.stationary) return false;
        if(strcmp(value,"0.05")&&strcmp(value,"0.10")&&strcmp(value,"0.15")&&
           strcmp(value,"0.20")&&strcmp(value,"0.25")) return false;
        m->approach_speed=strtof(value,NULL);
        if(m->phase!=MAP_FAULT) m->reason="APPROACH_SPEED_SET";
        return true;
    }
    if(!strcmp(cmd,"MAP ARM ALL")) {
        if(in.work_enabled && (!in.work_online || !in.work_ready || in.work_failed)) {
            m->reason="WORK_PEER_NOT_READY";return false;
        }
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
        m->prediction=(LaserPrediction){0};m->laser_motion=(LaserMotionFilter){0};m->run_ms=in.now;m->last_ms=in.now;
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
    if(segment.kind!=AUTO_ROUTE_SURVEY || segment.aisle==7u ||
       fabsf(auto_wrap_angle(pose.yaw-1.570796327f))>.139626340f)
        goto invalid;
    if(!l->frame_fresh ||
       (l->valid_mask&12u)!=12u ||
       l->sample_age_ms[AUTO_FRONT]>LASER_FRAME_TIMEOUT_MS ||
       l->sample_age_ms[AUTO_REAR]>LASER_FRAME_TIMEOUT_MS ||
       fabsf(in.wheel.wz)>.15f)
        goto interrupted;
    AutoPlane front,rear;float expected_front,expected_rear;
    const AutoLaserMount fm={.origin={.04098f,0},.yaw=0};
    const AutoLaserMount rm={.origin={-.04098f,0},.yaw=3.141592654f};
    if(!auto_route_ray(&m->route,pose,fm,7.5f,&front,&expected_front) ||
       !auto_route_ray(&m->route,pose,rm,7.5f,&rear,&expected_rear) ||
       front.normal.y<.9f || rear.normal.y>-.9f ||
       (front.surface_id<10u && rear.surface_id<10u) ||
       expected_front>.6f || expected_rear>.6f)
        goto invalid;
    /* Reserve the projected ray displacement plus 20mm near ridge ends.
     * Straight, short opposing rays can qualify before the old 80mm cutoff. */
    const float edge_margin=.02f+(fmaxf(expected_front,expected_rear)+.04098f)*fabsf(cosf(pose.yaw));
    for(unsigned row=1;row<=6;++row) {
        if((front.surface_id>=10u+4u*row && front.surface_id<=13u+4u*row) ||
           (rear.surface_id>=10u+4u*row && rear.surface_id<=13u+4u*row)) {
            const float low=row==4u?.6f:.4f, high=row==4u?2.4f:2.6f;
            if(pose.x<low+edge_margin || pose.x>high-edge_margin) goto invalid;
        }
    }
    const uint16_t f=l->accepted_mm[AUTO_FRONT],r=l->accepted_mm[AUTO_REAR];
    if(!f||!r||f>600u||r>600u) goto interrupted;
    const AutoVec2 velocity=auto_rotate((AutoVec2){in.wheel.vx,in.wheel.vy},pose.yaw);
    const float projection=sinf(pose.yaw);
    const float front_m=f*.001f-velocity.y*l->sample_age_ms[AUTO_FRONT]*.001f/projection;
    const float rear_m=r*.001f+velocity.y*l->sample_age_ms[AUTO_REAR]*.001f/projection;
    if(front_m<=0||rear_m<=0||fabsf(front_m+rear_m-expected_front-expected_rear)>.08f) goto interrupted;
    const float dy=((rear_m-expected_rear)-(front_m-expected_front))*.5f*projection;
    const float observed_y=pose.y+dy;
    if(fabsf(observed_y-segment.line.start.y)>.08f) goto invalid;
    if(!s->checkpoint_valid || front.surface_id!=s->front_surface || rear.surface_id!=s->rear_surface ||
       abs((int)f-s->checkpoint_front_mm)>5 || abs((int)r-s->checkpoint_rear_mm)>5) {
        s->checkpoint_valid=true;s->frozen=false;
        s->checkpoint_front_mm=f;s->checkpoint_rear_mm=r;
        s->checkpoint_wheel_y=m->localization.wheel_position.y;
    } else if(fabsf(m->localization.wheel_position.y-s->checkpoint_wheel_y)>.03f) s->frozen=true;
    if(s->frozen) goto invalid;
    /* Median source timestamps advance independently. Collect each direction
     * once per board frame, then consume a pair after both have advanced.
     * Freshness and geometry above still apply while waiting for its partner. */
    if(l->frame_count==s->frame) return;
    s->frame=l->frame_count;
    s->pending_updates|=(uint8_t)(l->updated_mask&12u);
    if(s->pending_updates!=12u || (s->confirmations && l->last_rx_ms==s->stamp_ms)) return;
    const uint32_t sample_ms[2]={in.now-l->sample_age_ms[AUTO_FRONT],in.now-l->sample_age_ms[AUTO_REAR]};
    for(unsigned sample=0;sample<2u;++sample) {
        const uint32_t advance=sample_ms[sample]-s->sample_ms[sample];
        if(s->confirmations && (!advance || advance>=0x80000000u)) return;
    }
    s->pending_updates=0;
    const uint32_t elapsed=l->last_rx_ms-s->stamp_ms;
    if(s->confirmations && (front.surface_id!=s->front_surface || rear.surface_id!=s->rear_surface ||
       abs((int)f-s->front_mm)>50 || abs((int)r-s->rear_mm)>50)) {
        s->confirmations=0;s->centering=false;
    }
    s->stamp_ms=l->last_rx_ms;
    s->sample_ms[0]=sample_ms[0];s->sample_ms[1]=sample_ms[1];
    s->front_surface=front.surface_id;s->rear_surface=rear.surface_id;s->front_mm=f;s->rear_mm=r;
    const bool already_confirmed=s->confirmations>=3u;
    if(s->confirmations<3u) ++s->confirmations;
    const float error=observed_y-segment.line.start.y;
    if(!already_confirmed) s->error_m=error;
    else {
        const float dt=fminf(elapsed*.001f,.3f);
        s->error_m+=(dt/(.20f+dt))*(error-s->error_m);
    }
    s->correction_m=0;
    s->active=s->confirmations>=3u;
    if(!s->active) return;
    if(fabsf(s->error_m)>.035f) s->centering=true;
    else if(fabsf(s->error_m)<=.015f) s->centering=false;
    /* Limit observation corrections to 20mm/s and 2mm per fresh pair. The
     * existing motion controller then drives back toward the nominal aisle. */
    const float limit=fminf(.002f,.02f*fminf(elapsed*.001f,.2f));
    const float filtered_delta=segment.line.start.y+s->error_m-pose.y;
    s->correction_m=fmaxf(-limit,fminf(limit,filtered_delta*.2f));
    m->localization.pose.pose.y+=s->correction_m;
    m->localization.used_mask|=12u;m->localization.constrained_axes|=2u;
    m->localization.rejected_mask&=(uint8_t)~12u;
    m->localization.uncorrected_m[1]=0;
    return;
interrupted:
    /* Keep a confirmed correction through one missed producer interval.
     * Do not renew its timestamp or bypass the independent range guards. */
    if(s->active && (uint32_t)(in.now-s->stamp_ms)<=200u &&
       (uint32_t)(in.now-s->sample_ms[0])<=LASER_FRAME_TIMEOUT_MS &&
       (uint32_t)(in.now-s->sample_ms[1])<=LASER_FRAME_TIMEOUT_MS) {
        s->correction_m=0;
        return;
    }
invalid:
    s->active=s->centering=false;s->confirmations=s->pending_updates=0;s->error_m=s->correction_m=0;
}
static void correct_boundary_cross(FullMapTest* m,FullMapInput in) {
    MapWallCorrection* s=&m->wall_correction;
    const AutoSegment line=m->route.segment[m->segment].line;
    const uint8_t axis=fabsf(line.end.y-line.start.y)>fabsf(line.end.x-line.start.x)?0u:1u;
    const AutoPose pose=m->localization.pose.pose;
    const AutoVec2 velocity=auto_rotate((AutoVec2){in.wheel.vx,in.wheel.vy},pose.yaw);
    if(s->confirmations) {
        const float dt=fminf((in.now-s->last_step_ms)*.001f,.05f);
        s->error_m+=(s->axis==0?velocity.x:velocity.y)*dt;
    }
    s->last_step_ms=in.now;
    if(fabsf(in.wheel.wz)>.1f || !in.laser.frame_fresh) goto gap;
    for(unsigned ch=0;ch<4u;++ch) {
        const uint8_t bit=(uint8_t)(1u<<ch);
        if(!(in.laser.valid_mask&bit) || !(in.laser.updated_mask&bit) ||
           in.laser.sample_age_ms[ch]>LASER_FRAME_TIMEOUT_MS ||
           !(m->localization.used_mask&bit)) continue;
        AutoPlane plane;float expected;
        if(!field_pose_ray(pose,ch,&plane,&expected) || expected>.6f ||
           (axis==0?fabsf(plane.normal.x):fabsf(plane.normal.y))<.9f) continue;
        const float angles[4]={1.570796327f,-1.570796327f,3.141592654f,0};
        const float angle=pose.yaw+angles[ch];
        const float incidence=plane.normal.x*cosf(angle)+plane.normal.y*sinf(angle);
        const float corrected=in.laser.accepted_mm[ch]*.001f-
            (plane.normal.x*velocity.x+plane.normal.y*velocity.y)*in.laser.sample_age_ms[ch]*.001f/incidence;
        const float residual=corrected-expected;
        if(corrected<=0 || fabsf(residual)>.15f) continue;
        const float normal=axis==0?plane.normal.x:plane.normal.y;
        const float error=(axis==0?pose.x-line.start.x:pose.y-line.start.y)-normal*residual*incidence;
        if(fabsf(error)>.08f) continue;
        const uint32_t sample=in.now-in.laser.sample_age_ms[ch];
        if(s->confirmations && s->channel==ch && (in.laser.frame_count==s->frame ||
           sample==s->sample_ms || (uint32_t)(sample-s->sample_ms)>=0x80000000u)) goto gap;
        const bool continuing=s->confirmations && s->channel==ch && s->axis==axis &&
            (uint32_t)(in.now-s->stamp_ms)<=300u;
        const float dt=continuing?fminf((in.now-s->stamp_ms)*.001f,.3f):0;
        if(!continuing) {s->confirmations=0;s->error_m=error;}
        else s->error_m+=dt/(.10f+dt)*(error-s->error_m);
        if(s->confirmations<3u) ++s->confirmations;
        s->channel=(uint8_t)ch;s->axis=axis;s->sample_ms=sample;
        s->frame=in.laser.frame_count;s->stamp_ms=in.now;s->active=s->confirmations>=3u;
        return;
    }
gap:
    if((uint32_t)(in.now-s->stamp_ms)>LASER_FRAME_TIMEOUT_MS ||
       (uint32_t)(in.now-s->sample_ms)>LASER_FRAME_TIMEOUT_MS) *s=(MapWallCorrection){0};
}
static void entry_observation(FullMapTest* m,FullMapInput in,LaserSnapshot* observation) {
    /* Long entry: west-wall X and wheel/IMU Y. Neither the right-hand ridge
     * ends nor distant forward returns are position references on this leg. */
    uint8_t excluded=(uint8_t)((1u<<AUTO_RIGHT)|(1u<<AUTO_REAR)|(1u<<AUTO_FRONT));
    AutoPose p=m->localization.pose.pose;
    p.yaw=auto_wrap_angle(in.imu_yaw+m->localization.imu_offset);
    AutoPlane plane;float expected;
    MapEntryGuidance* e=&m->entry;
    if(field_pose_ray(p,AUTO_FRONT,&plane,&expected) && plane.surface_id==3u) {
        if(expected<=FULL_MAP_ENTRY_APPROACH_M) e->approaching=true;
        if(e->approaching) {
            const uint8_t bit=(uint8_t)(1u<<AUTO_FRONT);
            const uint32_t age=observation->sample_age_ms[AUTO_FRONT],sample=in.now-age;
            const bool plausible=observation->frame_fresh && (observation->valid_mask&bit) &&
                age<=LASER_FRAME_TIMEOUT_MS && fabsf(in.wheel.wz)<=.10f &&
                fabsf(observation->accepted_mm[AUTO_FRONT]*.001f-in.wheel.vx*age*.001f-expected)<=.15f;
            if(plausible && (observation->updated_mask&bit) &&
               (!e->confirmations || (sample!=e->sample_ms && (uint32_t)(sample-e->sample_ms)<0x80000000u))) {
                if(!e->confirmations || (uint32_t)(sample-e->sample_ms)>300u) {
                    e->confirmations=0;e->first_ms=sample;
                }
                e->sample_ms=sample;if(e->confirmations<3u) ++e->confirmations;
                if(e->confirmations>=3u && (uint32_t)(sample-e->first_ms)>=180u) e->north_confirmed=true;
            }
            if(plausible && e->north_confirmed) excluded&=(uint8_t)~bit;
        }
    }
    observation->valid_mask&=(uint8_t)~excluded;
    observation->updated_mask&=(uint8_t)~excluded;
}
static float continuous_cross_feedback(FullMapTest* m,FullMapInput in,const AutoSegment* line) {
    const uint8_t axis=fabsf(line->end.y-line->start.y)>fabsf(line->end.x-line->start.x)?0u:1u;
    const AutoPose p=m->localization.pose.pose;
    float target=axis==0?p.x-line->start.x:p.y-line->start.y;
    if(m->wall_correction.active && m->wall_correction.axis==axis) target=m->wall_correction.error_m;
    if(axis==1u && m->aisle_correction.active) target=m->aisle_correction.error_m;
    MapCrossFeedback* s=&m->cross_feedback;
    if(!s->initialized || s->axis!=axis) {
        *s=(MapCrossFeedback){.error_m=target,.last_ms=in.now,.axis=axis,.initialized=true};
        return target;
    }
    const float dt=fminf((in.now-s->last_ms)*.001f,.05f);s->last_ms=in.now;
    const AutoVec2 velocity=auto_rotate((AutoVec2){in.wheel.vx,in.wheel.vy},p.yaw);
    s->error_m+=(axis==0?velocity.x:velocity.y)*dt;
    if(m->wall_correction.active) {
        s->wall_reference=true;s->wall_sample_ms=m->wall_correction.sample_ms;
    } else if(s->wall_reference && (uint32_t)(in.now-s->wall_sample_ms)<=600u) {
        /* Keep the odometry-propagated control reference across reacquisition.
         * A distant opposite fence must not pull the wheels the other way in
         * the middle of a short near-wall producer gap. This grants no fresh
         * localization evidence and no extra collision clearance. */
        return s->error_m;
    }
    /* Source loss changes the measurement weight, never the requested wheel
     * direction in one tick. Odometry propagates the existing error; a bounded
     * innovation blends towards the current real wall/pair or fused fallback.
     * This is command feedback only: it renews no observation or range age. */
    const bool observed=m->wall_correction.active || m->aisle_correction.active;
    const float innovation=(target-s->error_m)*dt/((observed?.08f:.12f)+dt),limit=(observed?.08f:.05f)*dt;
    s->error_m+=fmaxf(-limit,fminf(limit,innovation));
    return s->error_m;
}
static void bridge_far_survey_ray(FullMapTest* m,FullMapInput raw,FullMapInput in,LaserPredictionOutput* range) {
    const AutoRouteSegment route=m->route.segment[m->segment];
    if(route.kind!=AUTO_ROUTE_SURVEY || m->segment==0u) {
        memset(m->far_handover,0,sizeof(m->far_handover));return;
    }
    const AutoPose p=m->localization.pose.pose;
    const AutoSegment line=m->motion.initialized?m->motion.segment:route.line;
    bool east=line.end.x>line.start.x;
    if((line.end.x-p.x)*(east?1.0f:-1.0f)<-.02f) east=!east;
    const unsigned ch=east?AUTO_RIGHT:AUTO_LEFT,other=east?AUTO_LEFT:AUTO_RIGHT;
    const uint8_t bit=(uint8_t)(1u<<ch),opposite_bit=(uint8_t)(1u<<other);
    MapFarRayHandover* h=&m->far_handover[ch];
    if((in.laser.valid_mask&bit) && !(m->laser_rejected_mask&bit)) {h->active=false;return;}
    if(range->mask&bit) return; /* Preserve the ordinary motion predictor's budget first. */
    if(!h->active) {h->active=true;h->lost_ms=in.now;h->lost_path=m->laser_motion.path;}
    if((uint32_t)(in.now-h->lost_ms)>1000u || m->laser_motion.path-h->lost_path>.12f ||
       !raw.laser.frame_fresh || !(raw.laser.valid_mask&bit) ||
       raw.laser.sample_age_ms[ch]>LASER_FRAME_TIMEOUT_MS ||
       !(in.laser.valid_mask&opposite_bit) || (m->laser_rejected_mask&opposite_bit) ||
       in.laser.sample_age_ms[other]>LASER_FRAME_TIMEOUT_MS ||
       in.laser.accepted_mm[other]>600u || m->localization.uncorrected_m[0]>.05f ||
       m->localization.uncorrected_m[1]>.03f || fabsf(p.y-route.line.start.y)>.025f ||
       fabsf(auto_wrap_angle(p.yaw-1.570796327f))>.139626340f) return;
    AutoPlane wall,opposite;float distance,opposite_distance;
    if(!field_pose_ray(p,ch,&wall,&distance) || fabsf(wall.normal.x)<.9f ||
       !field_pose_ray(p,other,&opposite,&opposite_distance) || fabsf(opposite.normal.x)<.9f ||
       fabsf(in.laser.accepted_mm[other]*.001f-opposite_distance)>.08f) return;
    bool cross_supported=false;
    for(unsigned k=AUTO_REAR;k<=AUTO_FRONT;++k) {
        const uint8_t b=(uint8_t)(1u<<k);AutoPlane transverse;float expected;
        const float angle=k==AUTO_REAR?3.141592654f:0;
        const AutoLaserMount mount={.origin={.04098f*cosf(angle),.04098f*sinf(angle)},.yaw=angle};
        if((raw.laser.valid_mask&b) && raw.laser.sample_age_ms[k]<=LASER_FRAME_TIMEOUT_MS &&
           auto_route_ray(&m->route,p,mount,7.5f,&transverse,&expected) && fabsf(transverse.normal.y)>.9f &&
           raw.laser.accepted_mm[k]>100u && fabsf(raw.laser.accepted_mm[k]*.001f-expected)<=.08f)
            cross_supported=true;
    }
    const float closing=fmaxf(0,east?fmaxf(-in.wheel.vy,-m->motion.out.command.vy):
        fmaxf(in.wheel.vy,m->motion.out.command.vy));
    const float budget=FULL_MAP_MIN_LASER_RANGE_M+.5f*closing+closing*closing/.2f;
    if(!cross_supported || raw.laser.accepted_mm[ch]*.001f<=fmaxf(.6f,budget+.10f) ||
       distance<=fmaxf(.6f,budget+.10f)) return;
    /* Independent near opposite fence and transverse return corroborate this
     * short footprint transition. It grants only conservative map clearance,
     * <=0.15m/s, <=0.12m and <=1s beyond ordinary prediction; no valid/update,
     * anchor or observation timestamp is renewed. Real near readings still win. */
    range->mm[ch]=(uint16_t)floorf(fminf(distance-.04f,raw.laser.accepted_mm[ch]*.001f)*1000);
    range->mask|=bit;m->odom_range_mask|=bit;
    if(m->along_speed_limit==0 || m->along_speed_limit>.15f) m->along_speed_limit=.15f;
}
void full_map_step(FullMapTest* m,FullMapInput in) {
    if(!m->initialized) full_map_init(m,in.now);
    const AutoTwist previous_command=m->command;
    m->command=m->hint=(AutoTwist){0};
    const LaserSnapshot raw_laser=in.laser;
    const LaserMotionOutput checked=laser_motion_filter_step(&m->laser_motion,&in.laser,in.now,
        in.wheel.vx,in.wheel.vy,in.imu_yaw,m->motion.initialized?m->motion.config.decel:.10f,
        in.odom_ok && (m->phase==MAP_RUNNING || m->phase==MAP_PAUSED));
    in.laser=checked.checked;m->laser_rejected_mask=checked.rejected_mask;
    const bool entry_leg=m->segment==0u;
    LaserSnapshot observation=in.laser;
    for(unsigned i=0u;i<LASER_CHANNEL_COUNT;++i) {
        const uint8_t bit=(uint8_t)(1u<<i);
        if(entry_leg && i==AUTO_LEFT) {
            /* A motion/geometry-checked nearby wall uses its independent raw
             * source time. A 2-frame-old median must not starve a live X fix. */
            if(checked.rejected_mask&bit) {
                observation.valid_mask&=(uint8_t)~bit;observation.updated_mask&=(uint8_t)~bit;
            }
            continue;
        }
        if(observation.median_valid_mask&bit) {
            observation.accepted_mm[i]=observation.median_mm[i];
            observation.sample_age_ms[i]=observation.median_age_ms[i];
            observation.updated_mask=(uint8_t)((observation.updated_mask&~bit)|(observation.median_updated_mask&bit));
        }
    }
    if(entry_leg && m->localization.registered) entry_observation(m,in,&observation);
    /* In G7 the south-facing ray crosses R6 and changes target at its end.
     * It must not become a south-fence position observation during this leg.
     * Keep the north/front fence and east/west observations. The original
     * snapshot remains intact for freshness and collision protection. */
    if(m->segment<AUTO_ROUTE_COUNT && m->route.segment[m->segment].aisle==7u) {
        observation.valid_mask&=(uint8_t)~(1u<<AUTO_REAR);
        observation.updated_mask&=(uint8_t)~(1u<<AUTO_REAR);
    }
    /* Horizontal ridge aisles already use the independently confirmed pair
     * below. Do not apply a second, faster ridge correction on that axis. */
    const bool paired_aisle=m->segment<AUTO_ROUTE_COUNT &&
        m->route.segment[m->segment].kind==AUTO_ROUTE_SURVEY && m->route.segment[m->segment].row!=0u;
    const AutoRoute* pose_route=paired_aisle || entry_leg ? NULL:&m->route;
    if(m->localization.registered && !field_pose_step_route(&m->localization,in.wheel,in.imu_yaw,in.odom_ok,&observation,in.now,pose_route)) {
        if(full_map_active(m)) full_map_stop(m,"ODOMETRY_INVALID",true);
    }
    if(entry_leg && (m->localization.used_mask&(1u<<AUTO_LEFT))) {
        m->entry.left_observed=true;
        m->entry.left_sample_ms=in.now-observation.sample_age_ms[AUTO_LEFT];
    }
    if(m->phase==MAP_ARMED) {
        if(!in.healthy||in.now-m->arm_ms>4000u) full_map_stop(m,"ARM_EXPIRED_OR_PERMIT",false);
        return;
    }
    if(m->phase!=MAP_RUNNING&&m->phase!=MAP_PAUSED) return;
    if(in.work_enabled && (!in.work_online || in.work_failed)) {
        full_map_stop(m,"WORK_PEER_FAULT",false);return;
    }
    if(!in.healthy||!m->localization.pose.valid||in.now-m->last_ms>50u) {
        full_map_stop(m,"FEEDBACK_OR_PERMIT",!in.odom_ok);return;
    }
    m->last_ms=in.now;
    if(m->phase==MAP_PAUSED && m->user_hold) {m->reason="USER_PAUSE";return;}
    FullMapInput correction_input=in;
    correction_input.laser=observation;
    correct_aisle_position(m,correction_input);
    correct_boundary_cross(m,correction_input);
    if(in.now-m->run_ms>900000u||in.now-m->segment_ms>120000u) {full_map_stop(m,"RUN_TIMEOUT",false);return;}
    const AutoEnvelope body={.155f,.155f,.155f,.155f};
    if(!auto_route_pose_clear(&m->route,body,m->localization.pose.pose,0)) {
        full_map_stop(m,"MAP_ENVELOPE",false);return;
    }
    const bool margin_limited=!auto_route_pose_clear(&m->route,body,m->localization.pose.pose,.02f);
    m->along_speed_limit=margin_limited?.05f:0;
    LaserPredictionOutput range=laser_prediction_step(&m->prediction,&in.laser,in.now,
        in.wheel.vx,in.wheel.vy,in.imu_yaw,m->phase==MAP_RUNNING);
    /* A rejected fresh return is not a new anchor for the legacy dropout
     * predictor. The motion observer supplies its own bounded, aged estimate. */
    range.mask&=(uint8_t)~checked.rejected_mask;
    for(unsigned ch=0;ch<4u;++ch) if(checked.predicted_mask&(1u<<ch))
        range.mm[ch]=checked.estimated_mm[ch];
    range.mask|=checked.predicted_mask;
    /* Registered entry clearance is a map/odometry quantity, not a claimed
     * laser observation. The west fence bounds lateral drift; real front
     * observations become eligible only inside the 1.5m approach zone. */
    m->odom_range_mask=0;
    AutoPlane north_plane;float north_distance;
    const uint8_t front_bit=(uint8_t)(1u<<AUTO_FRONT);
    const float projection=sinf(m->localization.pose.pose.yaw);
    north_distance=(4.6f*m->route.scale-m->localization.pose.pose.y)/fmaxf(.95f,projection)-.04098f;
    if(entry_leg && raw_laser.frame_fresh &&
       fabsf(auto_wrap_angle(m->localization.pose.pose.yaw-1.570796327f))<.139626340f &&
       projection>.95f && north_distance>0) {
        const float uncertainty=.02f+.032f*m->localization.uncorrected_m[1];
        uint16_t distance=(uint16_t)fmaxf(0,floorf((north_distance-uncertainty)*1000));
        const bool north_real=m->entry.north_confirmed && (observation.valid_mask&front_bit) &&
            field_pose_ray(m->localization.pose.pose,AUTO_FRONT,&north_plane,&north_distance) && north_plane.surface_id==3u;
        if((checked.urgent_mask&front_bit) || north_real)
            distance=distance<in.laser.accepted_mm[AUTO_FRONT]?distance:in.laser.accepted_mm[AUTO_FRONT];
        range.mm[AUTO_FRONT]=distance;
        if(north_real) range.mask&=(uint8_t)~front_bit;
        else {range.mask|=front_bit;m->odom_range_mask|=front_bit;}
    }
    if(m->route.segment[m->segment].kind==AUTO_ROUTE_SHIFT) {
        const unsigned side=m->route.segment[m->segment].line.start.x<1.5f?AUTO_LEFT:AUTO_RIGHT;
        const uint8_t side_bit=(uint8_t)(1u<<side),rear_bit=(uint8_t)(1u<<AUTO_REAR);
        const AutoPose p=m->localization.pose.pose;AutoPlane plane;float distance;
        const float closing=fmaxf(0,fmaxf(-in.wheel.vx,-previous_command.vx));
        const float budget=FULL_MAP_MIN_LASER_RANGE_M+closing*.5f+closing*closing/.2f;
        /* A 0.7m side-corridor transfer has a nearby fence fixing X. A far,
         * live rear return may change its footprint at a ridge end; it cannot
         * replace the map with a false near obstacle or revoke the whole leg.
         * Require both the map and the real return beyond the full stopping
         * budget, bounded unobserved Y, and a fresh real side-wall constraint.
         * Missing CAN transport or an actually short rear return never qualify. */
        if(raw_laser.frame_fresh && (raw_laser.valid_mask&rear_bit) &&
           raw_laser.sample_age_ms[AUTO_REAR]<=LASER_FRAME_TIMEOUT_MS &&
           raw_laser.accepted_mm[AUTO_REAR]*.001f>fmaxf(.6f,budget+.10f) &&
           (in.laser.valid_mask&side_bit) && in.laser.sample_age_ms[side]<=LASER_FRAME_TIMEOUT_MS &&
           in.now-m->localization.constrained_ms[0]<=LASER_FRAME_TIMEOUT_MS &&
           m->localization.uncorrected_m[0]<.05f && m->localization.uncorrected_m[1]<1.2f &&
           fabsf(auto_wrap_angle(p.yaw-1.570796327f))<.139626340f &&
           (!(in.laser.valid_mask&rear_bit) || (range.mask&rear_bit)) &&
           field_pose_ray(p,AUTO_REAR,&plane,&distance) && plane.surface_id==4u &&
           distance>fmaxf(.6f,budget+.10f)) {
            const float uncertainty=.02f+.04f*m->localization.uncorrected_m[1];
            range.mm[AUTO_REAR]=(uint16_t)floorf((distance-uncertainty)*1000);
            range.mask|=rear_bit;m->odom_range_mask|=rear_bit;
        }
    }
    FullMapInput raw_input=in;raw_input.laser=raw_laser;
    bridge_far_survey_ray(m,raw_input,in,&range);
    memcpy(m->range_control_mm,range.mm,sizeof(m->range_control_mm));
    m->predicted_mask=range.mask;
    /* Necessary ray follows both measured motion and the next segment direction;
     * entering an open start/gate ray relies on the registered map envelope.
     * Other failed directions do not invalidate localization/navigation. */
    const AutoSegment seg=m->motion.initialized?m->motion.segment:m->route.segment[m->segment].line;
    AutoVec2 direction=auto_rotate((AutoVec2){seg.end.x-seg.start.x,seg.end.y-seg.start.y},-m->localization.pose.pose.yaw);
    const float len=hypotf(direction.x,direction.y);
    const AutoPose current=m->localization.pose.pose;
    const float to_end=(seg.end.x-current.x)*(seg.end.x-seg.start.x)+
        (seg.end.y-current.y)*(seg.end.y-seg.start.y);
    if(m->motion.initialized && to_end < -m->motion.config.position_tolerance*len) {
        direction.x=-direction.x;direction.y=-direction.y;
    }
    direction.x/=len;direction.y/=len;
    const float toward[4]={fmaxf(0,in.wheel.vy),fmaxf(0,-in.wheel.vy),fmaxf(0,-in.wheel.vx),fmaxf(0,in.wheel.vx)};
    const float commanded[4]={fmaxf(0,previous_command.vy),fmaxf(0,-previous_command.vy),
        fmaxf(0,-previous_command.vx),fmaxf(0,previous_command.vx)};
    const float ahead[4]={direction.y,-direction.y,-direction.x,direction.x};
    /* Field tuning 2026-11-11: the entry/exit gap leaves the parking leg without
     * a south fence, so its Y axis can never be corrected; at 0.10 m/s it covers
     * ~1 m, where wheel-odometry drift stays far inside the 80 mm envelope
     * margin. Exempt that leg from the unobserved-travel pause and raise the
     * general budget from 0.40 m to 0.80 m, so one sparse stretch no longer ends
     * a whole run. Envelope and laser protection below are untouched. */
    const bool park_leg=m->route.segment[m->segment].kind==AUTO_ROUTE_PARK;
    /* Entry Y is wheel/IMU travel, while the west wall constrains X. A single
     * rejected current sample must not revoke an unexpired earlier real fix
     * and collapse the Y budget to 1.2m. Preserve its actual source timestamp;
     * no missing sample, prediction or repeated median can renew this lease. */
    const bool entry_side_observed=entry_leg && raw_laser.frame_fresh && m->entry.left_observed &&
        in.now-m->entry.left_sample_ms<=LASER_FRAME_TIMEOUT_MS &&
        in.now-m->localization.constrained_ms[0]<=LASER_FRAME_TIMEOUT_MS &&
        m->localization.uncorrected_m[0]<.05f &&
        fabsf(auto_wrap_angle(m->localization.pose.pose.yaw-1.570796327f))<.139626340f;
    const float entry_length=m->route.segment[1].line.end.y-m->route.segment[0].line.start.y;
    const float y_budget=entry_side_observed?entry_length+.20f:1.2f;
    const bool pose_limited=!park_leg && (m->localization.uncorrected_m[0]>1.2f||
        m->localization.uncorrected_m[1]>y_budget);
    const bool entry_needs_north=entry_leg && m->route.segment[1].line.end.y-m->localization.pose.pose.y<.30f &&
        (!m->entry.north_confirmed || in.now-m->localization.constrained_ms[1]>500u);
    uint8_t required=0;bool safe=!pose_limited && !entry_needs_north;
    float range_speed=segment_speed(m);
    if(in.work_enabled && in.work_slow && in.work_station==m->target_tree)
        range_speed=fminf(range_speed,m->approach_speed);
    uint8_t blocked_directions=0;
    m->range_near_mask=m->range_brake_mask=m->range_stale_mask=0;
    for(unsigned i=0;i<4;++i) {
        AutoPlane plane;float predicted;
        const bool wall=field_pose_ray(m->localization.pose.pose,i,&plane,&predicted);
        const uint8_t bit=(uint8_t)(1u<<i);
        /* Visible near returns remain protective even when a map ray has no wall. */
        if((raw_laser.valid_mask&bit)&&raw_laser.accepted_mm[i]*.001f<=FULL_MAP_MIN_LASER_RANGE_M) {
            m->range_near_mask|=bit;safe=false;
        }
        const float speed=fmaxf(toward[i],commanded[i]);
        const float budget=FULL_MAP_MIN_LASER_RANGE_M+speed*.5f+speed*speed/.2f;
        m->range_budget_m[i]=budget;
        /* The right ray sees changing ridge ends on the entry, not a wall
         * reference. It cannot veto this leg through ordinary range checks. */
        if(entry_leg && i==AUTO_RIGHT) continue;
        const bool entry_front=entry_leg && i==AUTO_FRONT && (range.mask&front_bit);
        if(!entry_front && (ahead[i]>.2f||toward[i]>.01f) && (in.laser.valid_mask&bit) &&
           in.laser.accepted_mm[i]*.001f<=budget) {m->range_brake_mask|=bit;safe=false;}
        if(ahead[i]>.2f && in.laser.frame_fresh && ((in.laser.valid_mask|range.mask)&bit)) {
            /* Brake before entering the existing emergency pause envelope.
             * Include frame age and one producer interval in the lookahead. */
            const float delay=.70f+fminf(.30f,in.laser.sample_age_ms[i]*.001f);
            const float available=fmaxf(0,range.mm[i]*.001f-FULL_MAP_MIN_LASER_RANGE_M-.02f);
            /* Protection uses the conservative 0.10m/s^2 braking model.
             * Leave 0.02m/s for feedback tracking above the requested speed. */
            const float decel=.10f;
            const float closing_limit=sqrtf(decel*decel*delay*delay+2*decel*available)-decel*delay;
            range_speed=fminf(range_speed,fmaxf(0,closing_limit-.02f)/ahead[i]);
        }
        if(!wall) continue;
        if(ahead[i]>.2f||toward[i]>.10f || (fabsf(ahead[i])>.2f && commanded[i]>.0001f)) required|=bit;
        if(ahead[i]<=.2f && !((in.laser.valid_mask|range.mask)&bit)) blocked_directions|=bit;
        if(!(required&bit)) continue;
        if(!in.laser.frame_fresh||!((in.laser.valid_mask|range.mask)&bit)) {m->range_stale_mask|=bit;safe=false;continue;}
        if(range.mm[i]*.001f<=budget) {m->range_brake_mask|=bit;safe=false;}
    }
    if(m->required_mask!=required) {
        m->recovery_frames=0;m->recovery_updates=0;m->stable=false;
    }
    m->required_mask=required;
    if(!safe) {
        m->tree_hold_stable=false;
        if(m->phase!=MAP_PAUSED) {
            m->phase=MAP_PAUSED;m->pause_ms=in.now;m->stable=false;m->recovery_frames=0;
            m->recovery_updates=0;
            m->recovery_frame=in.laser.frame_count;auto_motion_cancel(&m->motion);
        }
        m->reason=pose_limited?"POSE_OBSERVATION_PAUSE":entry_needs_north?"ENTRY_NORTH_PAUSE":"RANGE_PAUSE";
    }
    if(m->phase==MAP_PAUSED) {
        /* Field tuning 2026-11-11: 5 s was shorter than one braking-and-settle
         * cycle plus two independent fresh frames whenever the pause started at
         * the far end of a 0.80 m unobserved stretch, which turned a recoverable
         * pause into a latched FAULT. 8 s keeps the same 2-frame + 200 ms gate. */
        if(in.now-m->pause_ms>=12000u) {
            const char* timeout_reason="RANGE_TIMEOUT";
            if(!strcmp(m->reason,"POSE_OBSERVATION_PAUSE")) timeout_reason="POSE_OBSERVATION_TIMEOUT";
            else if(!strcmp(m->reason,"ENTRY_NORTH_PAUSE")) timeout_reason="ENTRY_NORTH_TIMEOUT";
            else if(!strcmp(m->reason,"RANGE_PAUSE")) timeout_reason="RANGE_PAUSE_TIMEOUT";
            full_map_stop(m,timeout_reason,false);return;
        }
        if(!safe||!in.stationary) {
            m->stable=false;m->recovery_frames=0;m->recovery_updates=0;
            m->recovery_frame=in.laser.frame_count;return;
        }
        if(in.laser.frame_count!=m->recovery_frame) {
            m->recovery_frame=in.laser.frame_count;
            /* A repeated median is fresh but adds no independent evidence.
             * Retain earlier clear samples while the next required rays advance. */
            m->recovery_updates|=(uint8_t)(in.laser.updated_mask&required);
            if((m->recovery_updates&required)==required) {
                ++m->recovery_frames;m->recovery_updates=0;
            }
        }
        if(m->recovery_frames<2u) return;
        if(!m->stable) {m->stable=true;m->stable_ms=in.now;}
        if(in.now-m->stable_ms<200u) return;
        if(!begin(m,in.now)) {full_map_stop(m,"MOTION_CONFIG",false);return;}
        m->phase=MAP_RUNNING;m->reason="RESUMED";
    }
    if(m->tree_hold) {
        if(m->aisle_correction.active && m->aisle_correction.centering) {
            if(in.work_enabled && in.work_active) {
                full_map_stop(m,"WORK_STATION_MOVED",false);return;
            }
            m->tree_hold=m->tree_hold_stable=false;
            if(!begin(m,in.now))
                full_map_stop(m,"TREE_RECENTER_FAILED",false);
            else m->reason="TREE_RECENTER";
            return;
        }
        const AutoPose p=m->localization.pose.pose;
        const AutoVec2 target=m->motion.segment.end;
        if(fabsf(p.x-target.x)>.03f || fabsf(p.y-target.y)>.03f ||
           fabsf(auto_wrap_angle(p.yaw-m->motion.segment.heading))>m->motion.config.finish_yaw_tolerance) {
            if(in.work_enabled && in.work_active) {
                full_map_stop(m,"WORK_STATION_MOVED",false);return;
            }
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
        if(in.work_enabled) {
            m->reason=in.work_wait?"TREE_WORK_WAIT":"TREE_REQUEST_WAIT";
            if(in.work_station!=m->target_tree || !in.work_done) {
                if(!in.work_wait && in.now-m->tree_hold_ms>=5000u)
                    full_map_stop(m,"TREE_REQUEST_TIMEOUT",false);
                return;
            }
        } else if(in.now-m->tree_hold_ms<2000u) return;
        m->trees_done|=(uint16_t)(1u<<(m->target_tree-1u));
        m->tree_hold=m->tree_hold_stable=false;
        if(!begin(m,in.now)) full_map_stop(m,"TREE_NEXT_FAILED",false);
        else m->reason="TREE_NEXT";
        return;
    }
    uint8_t travel_rays=0;
    for(unsigned ch=0;ch<4u;++ch) if(ahead[ch]>.2f) travel_rays|=(uint8_t)(1u<<ch);
    /* Uncertainty in the travel direction caps progress. Orthogonal rays
     * still gate their own closing velocity, without repeatedly slowing an
     * otherwise observed straight drive. */
    m->motion.config.vmax=fmaxf(.001f,(m->predicted_mask&travel_rays) ?
        fminf(range_speed,LASER_PREDICT_SPEED_CAP_MPS):range_speed);
    if((m->predicted_mask&travel_rays&m->odom_range_mask)!=0u)
        m->motion.config.vmax=fmaxf(.001f,range_speed);
    /* The previous segment's alignment flag describes its old wheel targets.
     * Prime the new direction with a zero-drive preparation cycle first. */
    AutoMotionInput input={.pose=m->localization.pose,.measured=in.wheel,.healthy=true,
        .aligned=in.aligned && m->motion.started,.stationary=in.stationary,.now_ms=in.now};
    input.along_speed_limit=m->along_speed_limit;
    input.blocked_directions=blocked_directions;
    input.endpoint_observation_unsettled=m->target_tree &&
        in.now-m->localization.constrained_ms[0]<=LASER_FRAME_TIMEOUT_MS &&
        (fabsf(m->localization.innovation_m[0])>.010f ||
         fabsf(m->localization.pose.pose.x+m->localization.innovation_m[0]-m->motion.segment.end.x)>
            m->motion.config.position_tolerance);
    const float cross=continuous_cross_feedback(m,in,&seg);
    if(fabsf(seg.end.y-seg.start.y)>fabsf(seg.end.x-seg.start.x)) input.pose.pose.x=seg.start.x+cross;
    else input.pose.pose.y=seg.start.y+cross;
    if(m->target_tree && in.now-m->localization.constrained_ms[0]<=LASER_FRAME_TIMEOUT_MS &&
       fabsf(m->localization.pose.pose.x-m->motion.segment.end.x)<=.03f) {
        /* At a tree, control and completion must see the same pending real X
         * observation. Bound this endpoint-only lead to 10mm; retain the slow
         * map/envelope fusion and the unchanged 20/30mm acceptance windows. */
        const float pending=m->localization.innovation_m[0];
        input.pose.pose.x+=fmaxf(-.010f,fminf(.010f,pending));
    }
    if(m->aisle_correction.active) {
        /* Feed the confirmed, filtered pair into continuous cross correction.
         * The separately rate-limited fused pose remains the map/envelope state. */
        /* A large cross error reduces survey speed but must not turn an
         * in-flight correction into a pure sideways stop/90-degree wheel turn.
         * Only hold along at the endpoint, where the confirmed centering
         * hysteresis still has to release before a tree dwell can start. */
        input.hold_along=m->aisle_correction.centering &&
            fabsf(to_end)<=m->motion.config.position_tolerance*len;
        if(input.hold_along) m->reason="AISLE_CENTERING";
        else if(m->aisle_correction.centering) m->reason="AISLE_CORRECTING";
        else if(!strcmp(m->reason,"AISLE_CENTERING") || !strcmp(m->reason,"AISLE_CORRECTING")) m->reason="AISLE_CENTERED";
    } else if(!strcmp(m->reason,"AISLE_CENTERING") || !strcmp(m->reason,"AISLE_CORRECTING")) m->reason="AISLE_CENTERED";
    AutoMotionOutput out=auto_motion_step(&m->motion,input);
    if(out.phase==AUTO_MOTION_FAULT) {m->motion_error=out.error;full_map_stop(m,"MOTION_FAULT",false);return;}
    if(out.progress<-.05f) {full_map_stop(m,"WRONG_DIRECTION",false);return;}
    m->command=out.command;m->hint=out.steering_hint;
    if(out.phase==AUTO_MOTION_DONE) {
        if(m->target_tree) {
            m->tree_hold=true;m->tree_hold_stable=false;
            m->command=m->hint=(AutoTwist){0};m->reason="TREE_DWELL";return;
        }
        const unsigned aisle=m->route.segment[m->segment].aisle;
        if(aisle) m->coverage|=(uint8_t)(1u<<(aisle-1));
        m->segment+=(m->segment==0u?2u:1u);m->command=m->hint=(AutoTwist){0};
        /* Tree targets and pause recovery stay on the same aisle. A new
         * route segment needs its own surface and independent-frame history. */
        m->aisle_correction=(MapAisleCorrection){0};
        m->wall_correction=(MapWallCorrection){0};
        m->cross_feedback=(MapCrossFeedback){0};
        if(m->segment==AUTO_ROUTE_COUNT) {m->phase=MAP_DONE;m->reason="ROUTE_COMPLETE";return;}
        if(!begin(m,in.now)) {full_map_stop(m,"MOTION_CONFIG",false);return;}
        m->reason="NEXT_SEGMENT";
    }
}
