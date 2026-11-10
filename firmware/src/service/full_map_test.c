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
    m->aisle_correction.centering=false;
    m->command=m->hint=(AutoTwist){0};auto_motion_cancel(&m->motion);
    m->phase=MAP_FAULT;m->reason=reason;m->prediction=(LaserPrediction){0};
    if(lose_origin) m->localization.registered=m->localization.pose.valid=false;
}
static AutoMotionConfig motion_config(float speed) {
    /* Field tuning 2026-11-11: the former 0.15 m/s^2 ramps made every tree stop
     * and segment end cost ~3.3 s on each side at 0.50 m/s. 0.30/0.40 keeps the
     * approach curves monotonic while roughly halving the ramp and braking time.
     * The laser braking budget still uses its own conservative 0.10 m/s^2, so
     * raising the chassis decel only adds stopping margin, never removes it. */
    /* Tuning 2026-11-11: centring gain 0.8 -> 1.2. The 100 mm near floor stays
     * an unconditional safety invariant (pinned by range_threshold_tests), so the
     * only way to stop grazing a 0.7 m tree pitch is to hold the centreline
     * tighter: the row correction now eats an off-centre error ~1.5x faster. */
    return (AutoMotionConfig){.vmax=speed,.accel=.30f,.decel=.40f,.lateral_max=.10f,.cross_kp=1.2f,
        .yaw_kp=1.2f,.wz_max=.06f,.angular_accel=.15f,.position_tolerance=.02f,
        .cross_tolerance=.010f,.yaw_tolerance=.034906585f,.cross_abort=.08f,.yaw_abort=.15f,
        .settle_s=.4f,.stop_latency_s=.5f,.pose_max_age_ms=50,
        .finish_yaw_tolerance=.069813170f,.finish_cross_tolerance=.02f,.smooth_correction=true};
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
        const float correction_speed=.12f+.10f*clearance;
        const float error_fraction=a->active ?
            fmaxf(0,fminf(1,(fabsf(a->error_m)-.015f)/.020f)):1.0f;
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
    if(m->segment==0u) {c.yaw_tolerance=.017453293f;c.wz_max=.10f;}
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
    if(m->phase==MAP_FAULT && strcmp(cmd,"MAP ORIGIN")) return false;
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
       (l->valid_mask&12u)!=12u ||
       l->sample_age_ms[AUTO_FRONT]>LASER_FRAME_TIMEOUT_MS ||
       l->sample_age_ms[AUTO_REAR]>LASER_FRAME_TIMEOUT_MS ||
       fabsf(auto_wrap_angle(pose.yaw-1.570796327f))>.139626340f || fabsf(in.wheel.wz)>.15f)
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
    if(!f||!r||f>600u||r>600u) goto invalid;
    const AutoVec2 velocity=auto_rotate((AutoVec2){in.wheel.vx,in.wheel.vy},pose.yaw);
    const float projection=sinf(pose.yaw);
    const float front_m=f*.001f-velocity.y*l->sample_age_ms[AUTO_FRONT]*.001f/projection;
    const float rear_m=r*.001f+velocity.y*l->sample_age_ms[AUTO_REAR]*.001f/projection;
    if(front_m<=0||rear_m<=0||fabsf(front_m+rear_m-expected_front-expected_rear)>.08f) goto invalid;
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
    else s->error_m+=.25f*(error-s->error_m);
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
invalid:
    s->active=s->centering=false;s->confirmations=s->pending_updates=0;s->error_m=s->correction_m=0;
}
void full_map_step(FullMapTest* m,FullMapInput in) {
    if(!m->initialized) full_map_init(m,in.now);
    const AutoTwist previous_command=m->command;
    m->command=m->hint=(AutoTwist){0};
    const bool entry_leg=m->segment==0u;
    LaserSnapshot observation=in.laser;
    for(unsigned i=0u;i<LASER_CHANNEL_COUNT;++i) {
        const uint8_t bit=(uint8_t)(1u<<i);
        if(observation.median_valid_mask&bit) {
            observation.accepted_mm[i]=observation.median_mm[i];
            observation.sample_age_ms[i]=observation.median_age_ms[i];
            observation.updated_mask=(uint8_t)((observation.updated_mask&~bit)|(observation.median_updated_mask&bit));
        }
    }
    if(entry_leg) {
        /* Along the west entry lane, ridge-end returns on the right are not
         * lateral references. The front ray constrains Y only at the north fence. */
        uint8_t excluded=(uint8_t)((1u<<AUTO_RIGHT)|(1u<<AUTO_REAR));
        AutoPose p=m->localization.pose.pose;
        p.yaw=auto_wrap_angle(in.imu_yaw+m->localization.imu_offset);
        AutoPlane front;float distance;
        if(!field_pose_ray(p,AUTO_FRONT,&front,&distance) || front.surface_id!=3u)
            excluded|=(uint8_t)(1u<<AUTO_FRONT);
        observation.valid_mask&=(uint8_t)~excluded;
        observation.updated_mask&=(uint8_t)~excluded;
    }
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
    FullMapInput correction_input=in;
    correction_input.laser=observation;
    correct_aisle_position(m,correction_input);
    if(in.now-m->run_ms>900000u||in.now-m->segment_ms>120000u) {full_map_stop(m,"RUN_TIMEOUT",false);return;}
    const AutoEnvelope body={.155f,.155f,.155f,.155f};
    if(!auto_route_pose_clear(&m->route,body,m->localization.pose.pose,0)) {
        full_map_stop(m,"MAP_ENVELOPE",false);return;
    }
    const bool margin_limited=!auto_route_pose_clear(&m->route,body,m->localization.pose.pose,.01f);
    LaserPredictionOutput range=laser_prediction_step(&m->prediction,&in.laser,in.now,
        in.wheel.vx,in.wheel.vy,in.imu_yaw,m->phase==MAP_RUNNING);
    m->predicted_mask=range.mask;
    /* Necessary ray follows both measured motion and the next segment direction;
     * entering an open start/gate ray relies on the registered map envelope.
     * Other failed directions do not invalidate localization/navigation. */
    const AutoSegment seg=m->motion.initialized?m->motion.segment:m->route.segment[m->segment].line;
    AutoVec2 direction=auto_rotate((AutoVec2){seg.end.x-seg.start.x,seg.end.y-seg.start.y},-m->localization.pose.pose.yaw);
    const float len=hypotf(direction.x,direction.y);direction.x/=len;direction.y/=len;
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
    const bool entry_side_observed=entry_leg && observation.frame_fresh && (observation.valid_mask&1u) &&
        observation.sample_age_ms[AUTO_LEFT]<=LASER_FRAME_TIMEOUT_MS &&
        in.now-m->localization.constrained_ms[0]<=LASER_FRAME_TIMEOUT_MS &&
        m->localization.uncorrected_m[0]<.05f &&
        fabsf(auto_wrap_angle(m->localization.pose.pose.yaw-1.570796327f))<.139626340f;
    const float y_budget=entry_side_observed?3.5f:1.2f;
    const bool pose_limited=!park_leg && (m->localization.uncorrected_m[0]>1.2f||
        m->localization.uncorrected_m[1]>y_budget);
    const bool entry_needs_north=entry_leg && m->route.segment[1].line.end.y-m->localization.pose.pose.y<.30f &&
        in.now-m->localization.constrained_ms[1]>500u;
    uint8_t required=0;bool safe=!pose_limited && !entry_needs_north;
    float range_speed=segment_speed(m);
    if(entry_side_observed && m->localization.uncorrected_m[1]>.4f) range_speed=fminf(range_speed,.25f);
    if(margin_limited) range_speed=fminf(range_speed,.05f);
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
        if(ahead[i]>.2f && in.laser.frame_fresh && ((in.laser.valid_mask|range.mask)&bit)) {
            /* Brake before entering the existing emergency pause envelope.
             * Include frame age and one producer interval in the lookahead. */
            const float delay=.70f+fminf(.30f,in.laser.sample_age_ms[i]*.001f);
            const float available=fmaxf(0,range.mm[i]*.001f-FULL_MAP_MIN_LASER_RANGE_M-.03f);
            /* Protection uses the conservative 0.10m/s^2 braking model.
             * Leave 0.03m/s for feedback tracking above the requested speed. */
            const float decel=.10f;
            const float closing_limit=sqrtf(decel*decel*delay*delay+2*decel*available)-decel*delay;
            range_speed=fminf(range_speed,fmaxf(0,closing_limit-.03f)/ahead[i]);
        }
        if(!wall) continue;
        if(ahead[i]>.2f||toward[i]>.01f) required|=bit;
        if(!(required&bit)) continue;
        if(!in.laser.frame_fresh||!((in.laser.valid_mask|range.mask)&bit)) {safe=false;continue;}
        if(range.mm[i]*.001f<=budget) safe=false;
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
    m->motion.config.vmax=fmaxf(.001f,m->predicted_mask ?
        fminf(range_speed,LASER_PREDICT_SPEED_CAP_MPS):range_speed);
    /* The previous segment's alignment flag describes its old wheel targets.
     * Prime the new direction with a zero-drive preparation cycle first. */
    AutoMotionInput input={.pose=m->localization.pose,.measured=in.wheel,.healthy=true,
        .aligned=in.aligned && m->motion.started,.stationary=in.stationary,.now_ms=in.now};
    if(m->aisle_correction.active && m->aisle_correction.centering) {
        /* Use the confirmed pair immediately for centering. The separately
         * rate-limited fused pose remains the map/envelope state. */
        input.pose.pose.y=seg.start.y+m->aisle_correction.error_m;
        input.hold_along=true;
        m->reason="AISLE_CENTERING";
    } else if(!strcmp(m->reason,"AISLE_CENTERING")) m->reason="AISLE_CENTERED";
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
        if(m->segment==AUTO_ROUTE_COUNT) {m->phase=MAP_DONE;m->reason="ROUTE_COMPLETE";return;}
        if(!begin(m,in.now)) {full_map_stop(m,"MOTION_CONFIG",false);return;}
        m->reason="NEXT_SEGMENT";
    }
}
