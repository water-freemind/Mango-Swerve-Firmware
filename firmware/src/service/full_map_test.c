#include "full_map_test.h"
#include "full_map_tree_positions.h"
#include "work_can_protocol.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* User-selected laser reading threshold; do not add the sensor-to-body offset.
 * Map envelope checks independently enforce the modeled vehicle geometry. */
#define FULL_MAP_MIN_LASER_RANGE_MM 95u
#define FULL_MAP_MIN_LASER_RANGE_M .095f
#define FULL_MAP_ENTRY_APPROACH_M 1.50f
#define G7_HANDOFF_FRONT_MM 165u
#define NEAR_RECOVERY_TIME_MS 1500u
#define NEAR_RECOVERY_STEER_MS 2800u
#define NEAR_RECOVERY_SPEED_MPS .04f
#define NEAR_RECOVERY_PATH_M .04f
#define NEAR_RECOVERY_CLEAR_MM 110u
#define NEAR_OBSERVATION_REJECT_MM 100u
#define NEAR_RECOVERY_HARD_MM 80u
#define FULL_MAP_ARM_TIMEOUT_MS 30000u
#define WORK_POSE_CONFIRM_MS 200u
#define WORK_POSE_HARD_DISTANCE_M .08f
#define WORK_POSE_HARD_YAW_RAD .13962634f
#define WORK_VISION_MISSING_SKIP_MS 2500u
#define WORK_VISION_SKIP_ACK_MS 1500u
#define VISUAL_SURVEY_SPEED_MPS .60f
#define VISUAL_TREE_APPROACH_MPS .35f
#define VISUAL_CLEAR_SURVEY_SPEED_MPS .60f
#define VISUAL_GMG_APPROACH_MPS .20f
#define VISUAL_CLEAR_GMG_APPROACH_MPS .24f
#define VISUAL_TRACKED_APPROACH_MPS .22f
#define VISUAL_CLEAR_TRACKED_APPROACH_MPS .26f
#define VISUAL_NEAR_HOLD_MS 180u
#define VISUAL_STOP_APPROACH_MPS .11f
#define VISUAL_NEAR_ADJUST_MPS .025f
#define VISUAL_FINE_ADJUST_MPS .08f
#define VISUAL_FAR_ADJUST_MPS .12f
#define VISUAL_ALIGN_GAP_MS 200u
#define VISUAL_ALIGN_GAP_MPS .10f
#define SHIFT_CRUISE_CAP_MPS .28f
#define SHIFT_CLEAR_CAP_MPS .30f
#define SHIFT_CENTERED_CAP_MPS .70f
#define G4_SHIFT_WALL_CLOSING_MPS .025f
#define SHIFT_CORRECTION_CAP_MPS .20f
#define SHIFT_LOOKAHEAD_M .12f
#define G7_FRONT_SLOW_M .130f
#define G7_FRONT_CRUISE_M .155f
#define G7_CAUTION_SPEED_MPS .10f
#define SHIFT_WALL_CORRECT_M .040f
#define SHIFT_WALL_STOP_M .012f
#define SHIFT_WALL_RESUME_M .022f
#define SHIFT_WALL_BRAKE_MPS2 .10f
#define SHIFT_WALL_DELAY_S .70f
#define SHIFT_WALL_MAX_AGE_MS 200u
#define SURVEY_ENTRY_CAP_MPS .22f
#define SURVEY_ENTRY_TRACKED_CAP_MPS .30f
#define AISLE_CAUTION_GAP_M .032f
#define AISLE_CRUISE_GAP_M .020f
#define AISLE_CRAWL_GAP_M .008f
#define AISLE_MIN_OPPOSITE_GAP_M .040f
#define AISLE_NEAR_SPEED_MPS .10f
#define AISLE_RANGE_AGE_MS 200u
/* 2026-11-20 field run: G1..G6 paired ridge returns sat at 118..199mm
 * (median 150mm). A 150mm transverse guard therefore clamped the cross speed
 * on exactly the operating distance, stalling the cart before the tool could
 * reach the tree. Shorten the guard and the braking reserve on ridge aisles;
 * the 95mm hard near-stop and the map envelope checks stay in force. */
#define AISLE_WORK_MIN_LASER_RANGE_M .060f
#define AISLE_TRANSVERSE_GUARD_MM 110u
#define G2_ENTRY_OFFSET_M .020f
#define G2_ENTRY_CAP_MPS .18f
#define G4_ENTRY_OFFSET_M .030f
#define G4_ENTRY_CAP_MPS .12f
#define ENTRY_SHIFT_STOP_TOLERANCE_M .008f
#define ENTRY_SHIFT_FINISH_YAW_RAD .026179939f
#define G4_HANDOFF_OVERRUN_M .012f
#define G4_HANDOFF_FINISH_YAW_RAD .039269909f
#define SURVEY_TRACKING_YAW_TOLERANCE_RAD .013089969f /* 0.75 degrees */

typedef struct {
    uint8_t bit;
    float outward, gap_m;
    bool fresh;
} ShiftWallClearance;

static float speed_blend(float near,float far,float distance,float start,float end) {
    const float ratio=fmaxf(0,fminf(1,(distance-start)/(end-start)));
    return near+(far-near)*ratio;
}

const char* full_map_phase_name(FullMapPhase p) {
    static const char* const names[]={"IDLE","READY","ARMED","RUNNING","PAUSED","SEGMENT_DONE","DONE","FAULT"};
    return (unsigned)p<8?names[p]:"INVALID";
}
bool full_map_active(const FullMapTest* m) {
    return m->phase==MAP_ARMED||m->phase==MAP_RUNNING||m->phase==MAP_PAUSED;
}
void full_map_init(FullMapTest* m,uint32_t now) {
    memset(m,0,sizeof(*m));m->initialized=auto_route_build(&m->route,1,1.570796327f);
    m->phase=MAP_IDLE;m->reason="NEED_ORIGIN";m->speed=.90f;m->last_ms=now;
    m->approach_speed=.30f;
    for(unsigned row=1;row<=6;++row) for(unsigned number=1;number<=2;++number)
        if(!full_map_set_tree_x(m,row,number,full_map_tree_x_m[row-1][number-1])) m->initialized=false;
}
void full_map_stop(FullMapTest* m,const char* reason,bool lose_origin) {
    m->work_pose_excursion=false;
    m->visual_hold=(MapVisualHold){0};
    m->envelope_excursion=false;
    m->visual_braking=false;
    m->tree_hold=m->tree_hold_stable=false;
    m->aisle_correction.active=false;
    m->aisle_correction.centering=false;
    m->wall_correction=(MapWallCorrection){0};
    m->cross_feedback=(MapCrossFeedback){0};
    m->entry=(MapEntryGuidance){0};m->odom_range_mask=0;
    memset(m->far_handover,0,sizeof(m->far_handover));
    m->near_recovery=(MapNearRecovery){0};
    m->endpoint=(MapEndpointFeedback){0};
    m->tree_alignment=(MapTreeAlignment){0};
    m->command=m->hint=(AutoTwist){0};auto_motion_cancel(&m->motion);
    m->phase=MAP_FAULT;m->reason=reason;m->prediction=(LaserPrediction){0};
    m->laser_motion=(LaserMotionFilter){0};
    if(lose_origin) m->localization.registered=m->localization.pose.valid=false;
}
static AutoMotionConfig motion_config(float speed) {
    /* Shorter endpoint deceleration, independently bounded by range guards. */
    return (AutoMotionConfig){.vmax=speed,.accel=.45f,.decel=.80f,.lateral_max=.10f,.cross_kp=3.2f,
        .yaw_kp=1.6f,.wz_max=.10f,.angular_accel=.30f,.position_tolerance=.02f,
        .cross_tolerance=.005f,.yaw_tolerance=.017453293f,.cross_abort=.08f,.yaw_abort=.15f,
        .settle_s=.4f,.stop_latency_s=.25f,.pose_max_age_ms=50,
        .finish_yaw_tolerance=.069813170f,.finish_cross_tolerance=.02f,.overshoot_recovery=.06f,
        .smooth_correction=true};
}
static float segment_speed(const FullMapTest* m) {
    const AutoRouteKind kind=m->route.segment[m->segment].kind;
    float speed=m->speed;
    const MapAisleCorrection* a=&m->aisle_correction;
    if(kind==AUTO_ROUTE_SURVEY && a->confirmations) {
        const uint16_t nearest=a->front_mm<a->rear_mm?a->front_mm:a->rear_mm;
        const float clearance=fmaxf(0,fminf(1,((float)nearest-130.0f)/50.0f));
        /* Field tuning 2026-11-20: with the ridge guard shortened to 110mm the
         * confirmed pair no longer clamps the cart at its 150mm operating
         * distance, so a qualified pair may run 0.26..0.44 m/s. A lost or still
         * rebuilding pair keeps the old 0.20..0.35 ceiling: the entry/pair-loss
         * speed protection must not be bypassed by this tuning. */
        const float correction_speed=a->active?
            .26f+.18f*clearance:.20f+.15f*clearance;
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
static float east_entry_offset_m(unsigned aisle) {
    if(aisle==4u) return G4_ENTRY_OFFSET_M;
    return aisle==2u?G2_ENTRY_OFFSET_M:0.0f;
}
static bool begin(FullMapTest* m,uint32_t now) {
    if(m->segment>=AUTO_ROUTE_COUNT) return false;
    if(m->tree_hold) return true; /* Resume a dwell without restarting or skipping its timer. */
    AutoMotionConfig c=motion_config(segment_speed(m));
    if(m->segment==0u) {
        c.yaw_tolerance=.017453293f;c.yaw_kp=2.0f;c.wz_max=.15f;c.angular_accel=.30f;
        c.position_tolerance=.012f;c.finish_yaw_tolerance=.034906585f;
    }
    m->segment_ms=now;
    AutoSegment line=m->route.segment[m->segment].line;
    /* The two collinear geometry records form one continuous initial drive.
     * Do not brake/settle at the former entrance checkpoint. */
    if(m->segment==0u) line.end=m->route.segment[1].line.end;
    m->target_tree=0;
    const AutoRouteSegment route=m->route.segment[m->segment];
    /* 视觉找正的微调段不再套用连续纠偏的最小输出下限：低速下 20mm/s、
     * 40mrad/s 的强制输出会让舵轮在死区边界反复摆动（轮子跳舞）。
     * 其余巡垄段保留滞回，保证有误差时仍能起步修正。 */
    if(route.kind==AUTO_ROUTE_SURVEY && route.row!=1u && !m->tree_alignment.active) {
        c.lateral_min=.020f;c.wz_min=.040f;
    }
    const float entry_offset=route.kind==AUTO_ROUTE_SHIFT && m->segment+1u<AUTO_ROUTE_COUNT?
        east_entry_offset_m(m->route.segment[m->segment+1u].aisle):0.0f;
    if(entry_offset>0) {
        /* The G2 and G4 east approaches produced close rear returns at
         * the first ridge face. Stop the shift farther north before turning. */
        line.end.y+=entry_offset;
        c.position_tolerance=ENTRY_SHIFT_STOP_TOLERANCE_M;
        c.finish_cross_tolerance=.015f;
        c.finish_yaw_tolerance=ENTRY_SHIFT_FINISH_YAW_RAD;
    }
    if(route.aisle==7u) {
        /* The north-wall correction changes quickly after the entry turn.
         * Limit this open aisle's cross response during the launch ramp. */
        c.lateral_max=.04f;
        c.cross_kp=1.6f;
        c.lateral_min=0;
        c.wz_min=0;
    }
    if(route.kind==AUTO_ROUTE_SURVEY && route.row>=1u && route.row<=6u) {
        if(m->visual_work) {
            if(m->visual_segment!=m->segment) {
                m->visual_segment=(uint8_t)m->segment;m->row_work_count=0;
                m->visual_stop=m->visual_braking=m->work_scan_ending=false;m->work_request_pending=true;
            }
            const unsigned local=m->row_work_count==0u?0u:1u;
            m->target_tree=(uint8_t)((route.row-1u)*2u+(line.end.x>line.start.x?local:1u-local)+1u);
            if(!m->visual_stop) m->visual_approach_limit=m->visual_near_limit=0;
            if(m->visual_stop && !m->visual_braking) {
                line.end.x=m->visual_stop_x;
                if(fabsf(line.end.x-line.start.x)<.001f)
                    line.start.x=line.end.x+(route.line.end.x>route.line.start.x?-.05f:.05f);
            }
        } else for(unsigned k=0;k<2u;++k) {
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
        const float control_x=m->visual_hold.motion_reference?
            m->localization.wheel_position.x+m->visual_hold.motion_wheel_offset_x:m->localization.pose.pose.x;
        c.settle_s=.20f;
        if(m->tree_alignment.active && m->tree_alignment.station==m->target_tree) {
            line.end.x=m->tree_alignment.target_x;
            if(fabsf(line.end.x-control_x)>.001f) line.start.x=control_x;
            c.vmax=VISUAL_FINE_ADJUST_MPS;c.position_tolerance=.004f;c.settle_s=.2f;
            c.cross_tolerance=c.finish_cross_tolerance;
            c.yaw_tolerance=c.finish_yaw_tolerance;
        }
        const float direction=line.end.x>line.start.x?1.0f:-1.0f;
        const float past=(control_x-line.end.x)*direction;
        /* Reacquiring a nearby tree after a pause may need a short reverse.
         * Use its actual start, while preserving the aisle's nominal Y line. */
        if(past>0 && past<=.06f) line.start.x=control_x;
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
        m->near_recovery=(MapNearRecovery){0};
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
           front_age>200u||
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
        m->trees_done=m->trees_unresolved=0;
        m->aisle_entry_qualified=m->g4_entry_offset=m->g2_entry_offset=m->g2_wheel_guard=false;
        m->visual_work=m->visual_stop=m->visual_braking=m->work_request_pending=m->work_scan_ending=false;
        m->visual_segment=255u;m->row_work_count=0;m->work_count=m->unresolved_work_count=0;
        m->target_tree=0;m->tree_hold=m->tree_hold_stable=false;
        m->tree_alignment=(MapTreeAlignment){0};
        m->visual_hold=(MapVisualHold){0};
        m->phase=MAP_READY;m->reason="ORIGIN_FRONT_ANCHORED";return true;
    }
    if(!strncmp(cmd,"MAP SPEED ",10)) {
        if(full_map_active(m)||!in.stationary) return false;
        const char* p=cmd+10;
        if(strcmp(p,"0.50")&&strcmp(p,"0.60")&&strcmp(p,"0.70")&&strcmp(p,"0.80")&&strcmp(p,"0.90")) return false;
        m->speed=strtof(p,NULL);
        if(m->phase!=MAP_FAULT) m->reason="SPEED_SET";
        return true;
    }
    if(!strncmp(cmd,"MAP APPROACH ",13)) {
        const char* value=cmd+13;
        if(full_map_active(m)||!in.stationary) return false;
        if(strcmp(value,"0.15")&&strcmp(value,"0.20")&&strcmp(value,"0.25")&&
           strcmp(value,"0.30")&&strcmp(value,"0.35")) return false;
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
        if(strcmp(cmd,expected)||m->phase!=MAP_ARMED||in.now-m->arm_ms>FULL_MAP_ARM_TIMEOUT_MS||!in.healthy||!in.stationary) return false;
        m->visual_work=in.work_enabled;
        if(!begin(m,in.now)) return false;
        m->prediction=(LaserPrediction){0};m->laser_motion=(LaserMotionFilter){0};m->run_ms=in.now;m->last_ms=in.now;
        m->user_hold=false;m->phase=MAP_RUNNING;m->reason="RUNNING";return true;
    }
    return false;
}
/* Optional paired ridge observation during horizontal aisle traversal. Empty
 * fields never match these short, opposing returns. Never modify raw ranges. */
static void correct_aisle_position(FullMapTest* m,FullMapInput in,bool work_hold) {
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
    /* Compare independent range sources with wheel position at their source
     * time, not a cached median against the continuously advancing odometry. */
    const uint32_t oldest_age=in.now-sample_ms[0]>in.now-sample_ms[1]?
        in.now-sample_ms[0]:in.now-sample_ms[1];
    const float source_wheel_y=m->localization.wheel_position.y-velocity.y*oldest_age*.001f;
    if(!s->checkpoint_valid || front.surface_id!=s->front_surface || rear.surface_id!=s->rear_surface ||
       abs((int)f-s->checkpoint_front_mm)>5 || abs((int)r-s->checkpoint_rear_mm)>5) {
        s->checkpoint_valid=true;s->frozen=false;
        s->checkpoint_front_mm=f;s->checkpoint_rear_mm=r;s->checkpoint_wheel_y=source_wheel_y;
    } else if(fabsf(source_wheel_y-s->checkpoint_wheel_y)>.03f) s->frozen=true;
    if(s->frozen) goto invalid;
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
    if(work_hold) return; /* Relative ridge ranges monitor motion during a strike. */
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
static float work_ridge_displacement(FullMapTest* m,uint32_t now) {
    const MapAisleCorrection* pair=&m->aisle_correction;
    if(!pair->active || now-pair->sample_ms[0]>AISLE_RANGE_AGE_MS ||
       now-pair->sample_ms[1]>AISLE_RANGE_AGE_MS)
        return 0;
    MapWorkRidgeReference* reference=&m->work_ridge;
    if(!reference->front_surface) {
        *reference=(MapWorkRidgeReference){.front_surface=pair->front_surface,
            .rear_surface=pair->rear_surface,.front_mm=pair->front_mm,.rear_mm=pair->rear_mm};
        return 0;
    }
    if(reference->front_surface!=pair->front_surface || reference->rear_surface!=pair->rear_surface)
        return 0;
    const int front=(int)pair->front_mm-reference->front_mm;
    const int rear=(int)pair->rear_mm-reference->rear_mm;
    /* A translation changes the two opposing ranges in opposite directions. */
    return front*rear<0?fminf(abs(front),abs(rear))*.001f:0;
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
    /* In G7, FieldPose checks the north ray's bounded rotation error. */
    if(!in.laser.frame_fresh || (m->route.segment[m->segment].aisle!=7u && fabsf(in.wheel.wz)>.1f))
        goto gap;
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
static void recover_entry_north(FullMapTest* m,FullMapInput in,const LaserSnapshot* observation,
    AutoPose pose,float expected) {
    MapEntryGuidance* e=&m->entry;
    AutoPlane left_plane;float left_expected;
    const float front=observation->accepted_mm[AUTO_FRONT]*.001f;
    const float residual=front-expected;
    const bool stopped=in.stationary && hypotf(in.wheel.vx,in.wheel.vy)<.01f && fabsf(in.wheel.wz)<.02f;
    const bool valid=m->phase==MAP_PAUSED && !strcmp(m->reason,"ENTRY_NORTH_PAUSE") && stopped &&
        fabsf(auto_wrap_angle(pose.yaw-1.570796327f))<.069813170f &&
        observation->frame_fresh && (observation->valid_mask&9u)==9u &&
        observation->sample_age_ms[AUTO_FRONT]<=LASER_FRAME_TIMEOUT_MS &&
        observation->sample_age_ms[AUTO_LEFT]<=LASER_FRAME_TIMEOUT_MS &&
        front>=.11f && front<=.8f && fabsf(residual)>.15f && fabsf(residual)<=.35f &&
        e->left_observed && in.now-e->left_sample_ms<=LASER_FRAME_TIMEOUT_MS &&
        field_pose_ray(pose,AUTO_LEFT,&left_plane,&left_expected) && left_plane.surface_id==1u &&
        fabsf(observation->accepted_mm[AUTO_LEFT]*.001f-left_expected)<=.05f;
    if(!valid) {e->recovery_samples=0;return;}
    const uint32_t sample=in.now-observation->sample_age_ms[AUTO_FRONT];
    if(!(observation->updated_mask&(1u<<AUTO_FRONT)) ||
       (e->recovery_samples && (sample==e->recovery_sample_ms ||
        (uint32_t)(sample-e->recovery_sample_ms)>=0x80000000u))) return;
    if(!e->recovery_samples || sample-e->recovery_sample_ms>300u || fabsf(front-e->recovery_front_m)>.03f) {
        e->recovery_samples=0;e->recovery_first_ms=sample;
    }
    e->recovery_sample_ms=sample;e->recovery_front_m=front;
    if(e->recovery_samples<3u) ++e->recovery_samples;
    if(e->recovery_samples<3u || sample-e->recovery_first_ms<180u || e->recovery_m>=.25f) return;
    /* A bounded correction per independent stationary frame brings the ray
     * back into normal admission. Normal fusion supplies all axis evidence. */
    const float limit=fminf(.005f,.25f-e->recovery_m);
    const float correction=copysignf(fminf(limit,fabsf(residual)-.145f),-residual*sinf(pose.yaw));
    m->localization.pose.pose.y+=correction;
    e->recovery_m+=fabsf(correction);
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
        recover_entry_north(m,in,observation,p,expected);
        p.y=m->localization.pose.pose.y;
        (void)field_pose_ray(p,AUTO_FRONT,&plane,&expected);
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
static float continuous_cross_feedback(FullMapTest* m,FullMapInput in,const AutoSegment* line,
    float near_edge_error) {
    const uint8_t axis=fabsf(line->end.y-line->start.y)>fabsf(line->end.x-line->start.x)?0u:1u;
    const AutoPose p=m->localization.pose.pose;
    const bool wall_observed=m->wall_correction.active && m->wall_correction.axis==axis;
    const bool pair_observed=axis==1u && m->aisle_correction.active;
    float target=axis==0?p.x-line->start.x:p.y-line->start.y;
    if(axis==1u && (m->g4_entry_offset || m->g2_entry_offset))
        target-=east_entry_offset_m(m->route.segment[m->segment].aisle);
    if(wall_observed) target=m->wall_correction.error_m;
    if(pair_observed) target=m->aisle_correction.error_m;
    if(near_edge_error>0) target=fmaxf(target,near_edge_error);
    else if(near_edge_error<0) target=fminf(target,near_edge_error);
    MapCrossFeedback* s=&m->cross_feedback;
    if(axis==1u && m->aisle_correction.frozen) {
        /* Explicit rejection is not a missing producer interval. Cancel the
         * held steering reference rather than driving another 300ms on it. */
        *s=(MapCrossFeedback){.axis=axis,.last_ms=in.now,.initialized=true};return 0;
    }
    if(!s->initialized || s->axis!=axis) {
        *s=(MapCrossFeedback){.error_m=target,.last_ms=in.now,.axis=axis,.initialized=true};
        return target;
    }
    const float dt=fminf((in.now-s->last_ms)*.001f,.05f);s->last_ms=in.now;
    const AutoVec2 velocity=auto_rotate((AutoVec2){in.wheel.vx,in.wheel.vy},p.yaw);
    s->error_m+=(axis==0?velocity.x:velocity.y)*dt;
    if(pair_observed) {
        /* A confirmed opposing pair supersedes a former single-wall lease.
         * Keep its oldest real source time; cached pairs cannot renew it. */
        s->wall_reference=false;s->aisle_reference=true;
        const uint32_t a=m->aisle_correction.sample_ms[0],b=m->aisle_correction.sample_ms[1];
        s->aisle_sample_ms=in.now-a>in.now-b?a:b;
    } else if(near_edge_error==0 && s->aisle_reference && (uint32_t)(in.now-s->aisle_sample_ms)<=300u) {
        /* Propagate the accepted control error through a short pair gap.
         * A slow fused pose must not reverse correction toward the close ridge. */
        return s->error_m;
    } else if(wall_observed) {
        s->aisle_reference=false;
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
    const bool ridge_observed=pair_observed || near_edge_error!=0;
    const float innovation=(target-s->error_m)*dt/((ridge_observed?.06f:wall_observed?.08f:.12f)+dt);
    const float limit=(ridge_observed?.16f:wall_observed?.08f:.05f)*dt;
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
       m->localization.uncorrected_m[0]>.08f ||
       m->localization.uncorrected_m[1]>.03f || fabsf(p.y-route.line.start.y)>.05f ||
       fabsf(auto_wrap_angle(p.yaw-1.570796327f))>.139626340f) return;
    AutoPlane wall,opposite;float distance,opposite_distance;
    if(!field_pose_ray(p,ch,&wall,&distance) || fabsf(wall.normal.x)<.9f) return;
    const bool opposite_supported=(in.laser.valid_mask&opposite_bit) &&
        !(m->laser_rejected_mask&opposite_bit) && in.laser.sample_age_ms[other]<=LASER_FRAME_TIMEOUT_MS &&
        in.laser.accepted_mm[other]<=600u && field_pose_ray(p,other,&opposite,&opposite_distance) &&
        fabsf(opposite.normal.x)>.9f && fabsf(in.laser.accepted_mm[other]*.001f-opposite_distance)<=.08f;
    bool cross_supported=false;
    bool ridge_pair=true;
    float expected_span=0;
    for(unsigned k=AUTO_REAR;k<=AUTO_FRONT;++k) {
        const uint8_t b=(uint8_t)(1u<<k);AutoPlane transverse;float expected;
        const float angle=k==AUTO_REAR?3.141592654f:0;
        const AutoLaserMount mount={.origin={.04098f*cosf(angle),.04098f*sinf(angle)},.yaw=angle};
        const bool fresh=(raw.laser.valid_mask&b) && raw.laser.sample_age_ms[k]<=LASER_FRAME_TIMEOUT_MS;
        const bool transverse_hit=fresh && auto_route_ray(&m->route,p,mount,7.5f,&transverse,&expected) &&
            (k==AUTO_FRONT?transverse.normal.y:-transverse.normal.y)>.9f;
        ridge_pair &= transverse_hit && transverse.surface_id>=14u && expected<.60f &&
            raw.laser.accepted_mm[k]>110u;
        if(transverse_hit) {
            expected_span+=expected;
            if(raw.laser.accepted_mm[k]>100u && fabsf(raw.laser.accepted_mm[k]*.001f-expected)<=.08f)
                cross_supported=true;
        }
    }
    const float closing=fmaxf(0,east?fmaxf(-in.wheel.vy,-m->motion.out.command.vy):
        fmaxf(in.wheel.vy,m->motion.out.command.vy));
    const float budget=FULL_MAP_MIN_LASER_RANGE_M+.5f*closing+closing*closing/.2f;
    /* In central G4 the far travel ray jumped between metre-scale returns
     * and the opposite fence was also too far to provide a near reference. */
    const float measured_span=.001f*(raw.laser.accepted_mm[AUTO_REAR]+raw.laser.accepted_mm[AUTO_FRONT]);
    const bool g4_cross_clear=route.aisle==4u && ridge_pair && fabsf(measured_span-expected_span)<=.06f &&
        auto_route_pose_clear(&m->route,(AutoEnvelope){.155f,.155f,.155f,.155f},p,.005f);
    const bool opposite_clear=opposite_supported && m->localization.uncorrected_m[0]<=.05f &&
        fabsf(p.y-route.line.start.y)<=.025f;
    if(!cross_supported || (!opposite_clear && !g4_cross_clear) ||
       raw.laser.accepted_mm[ch]*.001f<=fmaxf(.6f,budget+.10f) ||
       distance<=fmaxf(.6f,budget+.10f)) return;
    /* Independent clearance checks corroborate the short footprint transition.
     * It grants only conservative map clearance,
     * <=0.15m/s, <=0.12m and <=1s beyond ordinary prediction; no valid/update,
     * anchor or observation timestamp is renewed. Real near readings still win. */
    range->mm[ch]=(uint16_t)floorf(fminf(distance-.04f,raw.laser.accepted_mm[ch]*.001f)*1000);
    range->mask|=bit;m->odom_range_mask|=bit;
    if(m->along_speed_limit==0 || m->along_speed_limit>.15f) m->along_speed_limit=.15f;
}
/* Recover by a bounded motion away from a real near return or a fresh,
 * positive but insufficient side-fence gap during a transfer. */
static bool near_recovery_step(FullMapTest* m,FullMapInput in,LaserSnapshot raw,bool pose_limited,
    ShiftWallClearance wall,bool recover_wall,bool other_ranges_safe) {
    MapNearRecovery* r=&m->near_recovery;
    const bool probing_wall=!r->active && recover_wall;
    if(!r->active && !m->range_near_mask && !recover_wall) return false;
    if(!r->active) {
        *r=(MapNearRecovery){.active=true,.start_ms=in.now,.last_ms=in.now,
            .frame=raw.frame_count,.preparing=true,.wall_mask=recover_wall?wall.bit:0};
        if(!probing_wall) {
            m->phase=MAP_PAUSED;m->pause_ms=in.now;
            auto_motion_cancel(&m->motion);
        }
    }
    r->mask|=m->range_near_mask;
    const bool shift_wall=r->wall_mask!=0;
    const uint8_t required_clear=(uint8_t)(r->mask|r->wall_mask);
    const float dt=(float)(in.now-r->last_ms)*.001f;r->last_ms=in.now;
    r->path_m+=hypotf(in.wheel.vx,in.wheel.vy)*dt;
    m->tree_hold_stable=false;m->stable=false;m->recovery_frames=0;m->recovery_updates=0;
    m->reason=shift_wall?"SHIFT_WALL_RECOVERY_WAIT":"NEAR_RECOVERY_WAIT";
    const uint32_t elapsed=r->move_started?in.now-r->move_ms:in.now-r->start_ms;
    const uint32_t budget=r->steering_requested && !r->move_started?
        NEAR_RECOVERY_STEER_MS:NEAR_RECOVERY_TIME_MS;
    if(elapsed>=budget) {
        full_map_stop(m,shift_wall?"SHIFT_WALL_RECOVERY_TIMEOUT":"NEAR_RECOVERY_TIMEOUT",false);return true;
    }
    if(r->path_m>=NEAR_RECOVERY_PATH_M) {
        full_map_stop(m,shift_wall?"SHIFT_WALL_RECOVERY_TRAVEL_LIMIT":"NEAR_RECOVERY_TRAVEL_LIMIT",false);return true;
    }
    bool fresh=raw.frame_fresh && (!shift_wall || (wall.fresh && wall.bit==r->wall_mask));
    bool clear=!shift_wall || wall.gap_m>SHIFT_WALL_RESUME_M;
    for(unsigned i=0;i<4u;++i) {
        const uint8_t bit=(uint8_t)(1u<<i);
        const bool ray_fresh=(raw.valid_mask&bit) && raw.sample_age_ms[i]<=200u;
        if(!ray_fresh) fresh=false;
        if(!(r->mask&bit)) continue;
        if(raw.accepted_mm[i]>=NEAR_RECOVERY_CLEAR_MM) {
            if(ray_fresh && raw.frame_fresh) r->near_seen_mask&=(uint8_t)~bit;
            continue;
        }
        clear=false;
        /* The first near sample stops travel immediately. Only a second real
         * source sample may turn the wheels or start the escape. */
        if(!raw.frame_fresh || !ray_fresh || !(raw.updated_mask&bit)) continue;
        const uint32_t source=in.now-raw.sample_age_ms[i];
        const uint32_t advance=source-r->sample_ms[i];
        if(!(r->near_seen_mask&bit)) r->near_seen_mask|=bit;
        else if(advance && advance<0x80000000u) r->near_confirmed_mask|=bit;
        else continue;
        r->sample_ms[i]=source;
    }
    if(fresh && clear) {
        if(raw.frame_count!=r->frame) {
            r->frame=raw.frame_count;
            for(unsigned i=0;i<4u;++i) if(required_clear&(1u<<i)) {
                const uint32_t source=in.now-raw.sample_age_ms[i];
                const uint32_t advance=source-r->sample_ms[i];
                if((raw.updated_mask&(1u<<i)) && advance && advance<0x80000000u) {
                    r->clear_updates|=(uint8_t)(1u<<i);r->sample_ms[i]=source;
                }
            }
            if((r->clear_updates&required_clear)==required_clear) {++r->clear_frames;r->clear_updates=0;}
        }
        if(r->clear_frames>=2u) {
            *r=(MapNearRecovery){0};m->recovery_frame=raw.frame_count;
            m->reason=shift_wall?"SHIFT_WALL_RECOVERY_CLEAR":"NEAR_RECOVERY_CLEAR";
            return false; /* Ordinary two-frame/200ms stopped recovery follows. */
        }
        r->preparing=true;r->prepared=false;return true;
    }
    r->clear_frames=r->clear_updates=0;
    /* A station action, severe near return, conflicting obstacles, missing
     * telemetry or inadequate localization permits no autonomous escape. */
    bool allowed=fresh && !pose_limited && !in.work_active && !in.work_wait &&
        (r->near_confirmed_mask&r->mask)==r->mask &&
        (required_clear&3u)!=3u && (required_clear&12u)!=12u;
    if(shift_wall && (!other_ranges_safe || r->mask || wall.gap_m<=0)) allowed=false;
    const bool yaw_still=fabsf(in.wheel.wz)<.02f;
    for(unsigned i=0;i<4u;++i)
        if(raw.accepted_mm[i]<=NEAR_RECOVERY_HARD_MM) allowed=false;
    AutoTwist escape={.vx=((required_clear&(1u<<AUTO_REAR))?1.0f:0)-((required_clear&(1u<<AUTO_FRONT))?1.0f:0),
        .vy=((required_clear&(1u<<AUTO_RIGHT))?1.0f:0)-((required_clear&(1u<<AUTO_LEFT))?1.0f:0)};
    const float norm=hypotf(escape.vx,escape.vy);
    if(norm==0) allowed=false;
    else {escape.vx*=NEAR_RECOVERY_SPEED_MPS/norm;escape.vy*=NEAR_RECOVERY_SPEED_MPS/norm;}
    const float toward[4]={escape.vy,-escape.vy,-escape.vx,escape.vx};
    for(unsigned i=0;i<4u;++i) if(toward[i]>.0001f &&
        raw.accepted_mm[i]*.001f<=FULL_MAP_MIN_LASER_RANGE_M+.005f+toward[i]*.5f+toward[i]*toward[i]/.2f) allowed=false;
    const AutoPose p=m->localization.pose.pose;
    const AutoVec2 delta=auto_rotate((AutoVec2){escape.vx,escape.vy},p.yaw);
    const AutoEnvelope body={.155f,.155f,.155f,.155f};
    /* Check the remaining escape corridor against the existing map envelope. */
    float needed_m=.005f;
    for(unsigned i=0u;i<4u;++i) if(r->mask&(1u<<i)) {
        const float away=-toward[i]/NEAR_RECOVERY_SPEED_MPS;
        if(away>.0001f) needed_m=fmaxf(needed_m,
            fmaxf(0,(NEAR_RECOVERY_CLEAR_MM+5.0f-raw.accepted_mm[i])*.001f)/away);
    }
    if(shift_wall) {
        const float inward=-wall.outward*delta.x/NEAR_RECOVERY_SPEED_MPS;
        if(inward<=.0001f) allowed=false;
        else needed_m=fmaxf(needed_m,(SHIFT_WALL_RESUME_M+.005f-wall.gap_m)/inward);
    }
    const float remaining=fminf(needed_m,fmaxf(0,NEAR_RECOVERY_PATH_M-r->path_m))/NEAR_RECOVERY_SPEED_MPS;
    for(unsigned step=0u;step<=8u;++step) {
        AutoPose next=p;next.x+=delta.x*remaining*(float)step/8.0f;next.y+=delta.y*remaining*(float)step/8.0f;
        if(!auto_route_pose_clear(&m->route,body,next,0)) allowed=false;
    }
    if(!allowed) {
        /* Keep the ordinary range pause until there is a safe escape path. */
        if(probing_wall) {*r=(MapNearRecovery){0};return false;}
        r->preparing=true;r->prepared=false;return true;
    }
    if(probing_wall) {
        m->phase=MAP_PAUSED;m->pause_ms=in.now;
        auto_motion_cancel(&m->motion);
    }
    if(fabsf(escape.vx-r->escape.vx)>.0001f || fabsf(escape.vy-r->escape.vy)>.0001f) {
        r->preparing=true;r->prepared=false;
    }
    r->escape=escape;
    if(!yaw_still && !r->preparing) {r->preparing=true;r->prepared=false;}
    if(r->preparing) {
        if(!in.stationary) {r->prepared=false;return true;}
        /* Steering on a soft floor can nudge yaw. Keep zero-drive wheel
         * preparation available, then require settled yaw to start moving. */
        r->steering_requested=true;
        m->hint=escape;m->reason=shift_wall?"SHIFT_WALL_RECOVERY_ALIGN":"NEAR_RECOVERY_ALIGN";
        if(!r->prepared) {r->prepared=true;return true;}
        if(!in.aligned || !yaw_still) return true;
        r->preparing=false;
    }
    if(!r->move_started) {r->move_started=true;r->move_ms=in.now;}
    m->command=m->hint=escape;m->reason=shift_wall?"SHIFT_WALL_RECOVERY_MOVE":"NEAR_RECOVERY_MOVE";
    return true;
}
static float endpoint_feedback(FullMapTest* m,FullMapInput in,const LaserSnapshot* observation) {
    MapEndpointFeedback* e=&m->endpoint;
    const float fused=m->localization.pose.pose.x,target=m->motion.segment.end.x;
    if(!m->target_tree || fabsf(fused-target)>.15f) {
        *e=(MapEndpointFeedback){0};return fused;
    }
    if(!e->active || e->tree!=m->target_tree || in.now-e->last_ms>50u)
        *e=(MapEndpointFeedback){.active=true,.tree=m->target_tree,.last_ms=in.now,.x=fused};
    const AutoVec2 velocity=auto_rotate((AutoVec2){in.wheel.vx,in.wheel.vy},m->localization.pose.pose.yaw);
    e->x+=velocity.x*(in.now-e->last_ms)*.001f;e->last_ms=in.now;
    uint32_t source=0;bool new_x=false;
    if(m->localization.constrained_ms[0]==in.now) for(unsigned ch=0;ch<2u;++ch) {
        const uint8_t bit=(uint8_t)(1u<<ch);
        if((m->localization.used_mask&bit) && (observation->valid_mask&bit) &&
           (observation->updated_mask&bit) && observation->sample_age_ms[ch]<=LASER_FRAME_TIMEOUT_MS) {
            const uint32_t stamp=in.now-observation->sample_age_ms[ch];
            if(!new_x || (uint32_t)(stamp-source)<0x80000000u) source=stamp;
            new_x=true;
        }
    }
    if(new_x && (!e->seen || (source!=e->source_ms && (uint32_t)(source-e->source_ms)<0x80000000u))) {
        const float elapsed=e->seen?fminf((source-e->source_ms)*.001f,.3f):.1f;
        const float observed=fused+m->localization.innovation_m[0];
        const float delta=(observed-e->x)*elapsed/(.10f+elapsed);
        e->x+=fmaxf(-.012f,fminf(.012f,delta));
        e->source_ms=source;e->seen=true;
        if(fabsf(fused-target)<=.02f && fabsf(e->x-target)<=.02f) {
            if(!e->confirmations) e->first_ms=source;
            if(e->confirmations<3u) ++e->confirmations;
            if(e->confirmations>=3u && source-e->first_ms>=180u) e->ready=true;
        } else if(!e->ready) e->confirmations=0;
    }
    const bool fresh=e->seen && in.now-e->source_ms<=LASER_FRAME_TIMEOUT_MS;
    const bool fused_clear=fabsf(fused-target)<=.02f,control_clear=fabsf(e->x-target)<=.02f;
    if(!fresh || !fused_clear || !control_clear) {e->ready=false;e->confirmations=0;}
    e->blockers=(uint16_t)((!fresh?0x200u:0u)|(!fused_clear?0x400u:0u)|
        (!control_clear?0x800u:0u)|(!e->ready?0x1000u:0u));
    return e->x;
}
/* Limit a side-transfer model overlap to 10mm along either map axis. */
static AutoEnvelope shift_tolerance_body(AutoPose pose) {
    const float trim=.010f/(fabsf(cosf(pose.yaw))+fabsf(sinf(pose.yaw)));
    const float side=.155f-trim;
    return (AutoEnvelope){side,side,side,side};
}

static ShiftWallClearance shift_wall_clearance(const AutoRoute* route, AutoSegment line,
    AutoPose pose, AutoEnvelope body, const LaserSnapshot* laser) {
    const float east_fence=3.0f*route->scale;
    const bool east=line.start.x>east_fence*.5f;
    const unsigned channel=east?AUTO_RIGHT:AUTO_LEFT;
    const float outward=east?1.0f:-1.0f;
    const AutoVec2 normal=auto_rotate((AutoVec2){outward,0},-pose.yaw);
    const float extent=auto_envelope_support(body,normal);
    const float incidence=-outward*normal.y;
    ShiftWallClearance wall={.bit=(uint8_t)(1u<<channel),.outward=outward,
        .gap_m=(east?east_fence-pose.x:pose.x)-extent};
    wall.fresh=laser->frame_fresh && (laser->valid_mask&wall.bit) &&
        laser->sample_age_ms[channel]<=SHIFT_WALL_MAX_AGE_MS && incidence>=.95f;
    if(wall.fresh) {
        const float measured=(laser->accepted_mm[channel]*.001f+FIELD_POSE_LASER_ORIGIN_M)*incidence-extent;
        wall.gap_m=fminf(wall.gap_m,measured);
    }
    return wall;
}

static bool aisle_range_fresh(const LaserSnapshot* laser,unsigned channel) {
    return laser->frame_fresh && (laser->valid_mask&(1u<<channel)) &&
        laser->sample_age_ms[channel]<=AISLE_RANGE_AGE_MS;
}

static float aisle_ray_gap(AutoPose pose,AutoEnvelope body,unsigned channel,uint16_t distance_mm) {
    const float direction=channel==AUTO_FRONT?1.0f:-1.0f;
    const AutoVec2 normal=auto_rotate((AutoVec2){0,direction},-pose.yaw);
    return (distance_mm*.001f+FIELD_POSE_LASER_ORIGIN_M)*direction*normal.x-
        auto_envelope_support(body,normal);
}

static void map_cross_limits(const FullMapTest* m,FullMapInput in,ShiftWallClearance wall,
    bool g4_transfer,AutoMotionInput* input) {
    const unsigned axis=fabsf(m->motion.segment.end.y-m->motion.segment.start.y)>
        fabsf(m->motion.segment.end.x-m->motion.segment.start.x)?0u:1u;
    float negative,positive;
    const AutoEnvelope body=m->envelope_excursion?shift_tolerance_body(m->localization.pose.pose):
        (AutoEnvelope){.155f,.155f,.155f,.155f};
    input->limit_cross=true;input->cross_negative_limit=input->cross_positive_limit=0;
    if(!auto_route_axis_clearance(&m->route,body,m->localization.pose.pose,axis,&negative,&positive)) return;
    /* Bound the dangerous normal component, not just the along-route speed.
     * 0.45m/s^2 with 200ms reaction is below the commanded 0.90m/s^2 brake. */
    const float a=.45f,delay=.20f;
    negative=sqrtf(a*a*delay*delay+2*a*fmaxf(0,negative-.005f))-a*delay;
    if(axis==1u && m->g2_wheel_guard) {
        /* A biased pair can pull fused Y south while steering north.
         * Also bound northward clearance by wheel travel from the G2 turn. */
        const float wheel_y=m->g2_wheel_offset_y+m->localization.wheel_position.y;
        positive-=fmaxf(0,wheel_y-m->localization.pose.pose.y);
    }
    const bool g4_survey=axis==1u && m->route.segment[m->segment].aisle==4u;
    const float positive_reserve=g4_survey?.014f:.005f;
    positive=sqrtf(a*a*delay*delay+2*a*fmaxf(0,positive-positive_reserve))-a*delay;
    if(wall.bit) {
        const float delay_speed=SHIFT_WALL_BRAKE_MPS2*SHIFT_WALL_DELAY_S;
        const float space=fmaxf(0,wall.gap_m-SHIFT_WALL_STOP_M-.003f);
        float limit=wall.fresh?sqrtf(delay_speed*delay_speed+2*SHIFT_WALL_BRAKE_MPS2*space)-delay_speed:0;
        /* Rejoining the shift line after recovery must not drive hard back
         * toward the fence while its distance is settling. */
        if(g4_transfer) limit=fminf(limit,G4_SHIFT_WALL_CLOSING_MPS);
        if(wall.outward>0) positive=fminf(positive,limit);
        else negative=fminf(negative,limit);
    }
    if(axis==1u && m->aisle_correction.frozen) negative=positive=0;
    const float normal=axis==0u?m->motion.normal.x:m->motion.normal.y;
    input->cross_negative_limit=normal>=0?negative:positive;
    input->cross_positive_limit=normal>=0?positive:negative;
    if(axis!=1u || in.work_active || !in.laser.frame_fresh) return;
    const AutoRouteSegment route=m->route.segment[m->segment];
    const bool g4_entry=route.aisle==4u && (m->g4_entry_offset ||
        route.line.start.x-m->localization.pose.pose.x<.60f);
    const bool visual_stop=m->visual_work && m->visual_stop && m->target_tree && in.work_enabled;
    const bool ridge_aisle=route.kind==AUTO_ROUTE_SURVEY && route.row!=0u;
    const bool g7=route.aisle==7u;
    if(!g4_entry && !visual_stop && !ridge_aisle && !g7) return;
    /* Limit motion toward either short ray, even while its opposite face is
     * still outside the pair observer's acquisition window. */
    const float yaw=m->localization.pose.pose.yaw;
    const float forward_normal=cosf(yaw)*m->motion.normal.x+sinf(yaw)*m->motion.normal.y;
    const float forward_along=cosf(yaw)*m->motion.tangent.x+sinf(yaw)*m->motion.tangent.y;
    const float along_closing=fabsf(forward_along)*m->motion.config.vmax;
    for(unsigned ch=AUTO_REAR;ch<=AUTO_FRONT;++ch) {
        const bool planned=g7 || g4_entry || (visual_stop && ch==AUTO_FRONT);
        if(!planned && (!ridge_aisle || in.laser.accepted_mm[ch]>AISLE_TRANSVERSE_GUARD_MM)) continue;
        if(!(in.laser.valid_mask&(1u<<ch)) || in.laser.sample_age_ms[ch]>LASER_FRAME_TIMEOUT_MS) continue;
        /* G1..G6 keep a shorter front/rear reserve so the tool can close on
         * the ridge; non-work legs retain the original 95mm reserve. */
        const float reserve=g7?(ch==AUTO_FRONT?G7_FRONT_SLOW_M-.010f:.115f):
            ridge_aisle?AISLE_WORK_MIN_LASER_RANGE_M:FULL_MAP_MIN_LASER_RANGE_M;
        const float available=fmaxf(0,in.laser.accepted_mm[ch]*.001f-reserve-.010f);
        const float reaction=.55f+fminf(.30f,in.laser.sample_age_ms[ch]*.001f);
        const float delay_speed=.10f*reaction;
        const float closing=fmaxf(0,sqrtf(delay_speed*delay_speed+.20f*available)-delay_speed-.012f);
        const float remaining=fmaxf(0,closing-along_closing);
        const float ray_normal=ch==AUTO_FRONT?forward_normal:-forward_normal;
        if(ray_normal>.2f)
            input->cross_positive_limit=fminf(input->cross_positive_limit,remaining/ray_normal);
        else if(ray_normal<-.2f)
            input->cross_negative_limit=fminf(input->cross_negative_limit,remaining/-ray_normal);
    }
}
static void capture_visual_hold(FullMapTest* m) {
    MapVisualHold* hold=&m->visual_hold;
    hold->wheel=m->localization.wheel_position;
    hold->active=true;
    hold->motion_reference=false;
    if(!hold->origin_valid) {
        hold->origin_wheel_x=hold->wheel.x;
        hold->origin_valid=true;
    }
}
/* 停稳后按新视觉反馈微调，在中心候选区制动并确认。
 * 视觉触发允许在首次停车点±180mm内找正，固定树位保留±60mm。 */
static bool refine_tree_position(FullMapTest* m,FullMapInput in) {
    MapTreeAlignment* a=&m->tree_alignment;
    if(!a->active || a->station!=m->target_tree) {
        const unsigned index=m->target_tree-1u;
        *a=(MapTreeAlignment){.active=true,.station=m->target_tree,.started_ms=in.now,
            .nominal_x=m->visual_work?m->visual_stop_x:m->trees[index/2u][index%2u].x,
            .target_x=m->motion.segment.end.x};
    }
    const bool was_ready=a->ready;
    a->ready=in.work_alignment_fresh && in.work_centered;
    if(a->ready && !was_ready && in.stationary) {
        /* The visual stop can settle short of the planned brake endpoint.
         * Record its measured X once before handing the hold to the tool. */
        a->target_x=m->localization.pose.pose.x;
        m->motion.segment.end.x=a->target_x;
    }
    if(a->ready) return false;
    m->tree_hold_stable=false;m->reason="TREE_VISION_WAIT";
    if(in.now-a->started_ms>=12000u) {
        full_map_stop(m,"TREE_VISION_TIMEOUT",false);return true;
    }
    if(!in.work_alignment_fresh || !in.work_align_direction || in.work_align_frame==a->frame ||
       in.work_target_missing ||
       (in.work_target_fresh && in.work_target_dx*in.work_align_direction<=STATION_VISION_STOP_PX))
        return true;
    a->frame=in.work_align_frame;
    const bool new_pass=!a->steps || a->direction!=in.work_align_direction;
    a->direction=in.work_align_direction;
    a->far_target_seen=false;
    const float travel=m->visual_work?.18f:.06f;
    /* Re-express the original physical stop in the current map frame so a
     * correction while parked does not consume the next alignment pass. */
    if(m->visual_work && m->visual_hold.origin_valid)
        a->nominal_x=m->localization.pose.pose.x+m->visual_hold.origin_wheel_x-
            m->localization.wheel_position.x;
    a->target_x=a->nominal_x+travel*a->direction;
    if((new_pass && a->steps>=12u) || fabsf(a->target_x-m->localization.pose.pose.x)<=.005f) {
        full_map_stop(m,"TREE_VISION_TRAVEL_LIMIT",false);return true;
    }
    if(new_pass) ++a->steps;
    if(m->visual_work) {
        m->visual_hold.motion_wheel_offset_x=m->localization.pose.pose.x-m->localization.wheel_position.x;
        m->visual_hold.motion_reference=true;
    }
    m->tree_hold=false;
    if(!begin(m,in.now)) full_map_stop(m,"TREE_VISION_CONFIG",false);
    else m->reason="TREE_VISION_ADJUST";
    return true;
}
static void finish_tree(FullMapTest* m,FullMapInput in,bool skipped) {
    const bool no_target=!skipped && in.work_enabled && in.work_result==WORK_RESULT_SKIPPED;
    const bool unresolved=skipped || no_target || (in.work_enabled && in.work_result==WORK_RESULT_ATTEMPTS);
    const uint16_t tree_bit=(uint16_t)(1u<<(m->target_tree-1u));
    m->trees_done|=tree_bit;
    if(unresolved) m->trees_unresolved|=tree_bit;
    if(m->visual_work) {
        if(!skipped) ++m->work_count;
        if(unresolved) ++m->unresolved_work_count;
        if(m->row_work_count<255u) ++m->row_work_count;
        m->visual_stop=false;m->work_request_pending=true;
        m->tree_alignment=(MapTreeAlignment){0};
    }
    m->tree_hold=m->tree_hold_stable=false;
    if(!begin(m,in.now)) full_map_stop(m,"TREE_NEXT_FAILED",false);
    else m->reason=skipped?"TREE_VISION_SKIPPED":no_target?"TREE_NO_TARGET_CONTINUE":
        unresolved?"TREE_ATTEMPTS_CONTINUE":"TREE_NEXT";
}

void full_map_step(FullMapTest* m,FullMapInput in) {
    if(!m->initialized) full_map_init(m,in.now);
    const bool work_hold=in.work_enabled && in.work_active && m->tree_hold;
    const bool visual_parked=m->visual_work && m->visual_stop && m->tree_hold;
    if(!work_hold) m->work_pose_excursion=false;
    if(!work_hold && !visual_parked) m->work_ridge=(MapWorkRidgeReference){0};
    if(!m->visual_work || !m->visual_stop) m->visual_hold=(MapVisualHold){0};
    else if(!visual_parked) m->visual_hold.active=false;
    const AutoTwist previous_command=m->command;
    m->command=m->hint=(AutoTwist){0};
    const LaserSnapshot raw_laser=in.laser;
    const LaserMotionOutput checked=laser_motion_filter_step(&m->laser_motion,&in.laser,in.now,
        in.wheel.vx,in.wheel.vy,in.imu_yaw,m->motion.initialized?m->motion.config.decel:.10f,
        in.odom_ok && (m->phase==MAP_RUNNING || m->phase==MAP_PAUSED));
    in.laser=checked.checked;m->laser_rejected_mask=checked.rejected_mask;
    const bool entry_leg=m->segment==0u;
    const bool g7=m->segment<AUTO_ROUTE_COUNT && m->route.segment[m->segment].aisle==7u;
    const uint8_t front_bit=(uint8_t)(1u<<AUTO_FRONT);
    LaserSnapshot observation=in.laser;
    for(unsigned i=0u;i<LASER_CHANNEL_COUNT;++i) {
        const uint8_t bit=(uint8_t)(1u<<i);
        if((entry_leg && i==AUTO_LEFT) || (g7 && i==AUTO_FRONT)) {
            /* Use the checked near-wall sample and its own source time.
             * A delayed median must not starve live perimeter fixes. */
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
    /* Reject <=100mm as map-surface evidence before the 95mm guard fires.
     * A close obstacle need not be the mapped fence/ridge. Keep it fully
     * protective in raw_laser, but do not let it drag the pose into that map
     * surface while a bounded escape is trying to restore clearance. */
    for(unsigned i=0u;i<LASER_CHANNEL_COUNT;++i) {
        const uint8_t bit=(uint8_t)(1u<<i);
        if(((raw_laser.valid_mask&bit) && raw_laser.accepted_mm[i]<=NEAR_OBSERVATION_REJECT_MM) ||
           (m->near_recovery.active && ((m->near_recovery.mask|m->near_recovery.wall_mask)&bit))) {
            observation.valid_mask&=(uint8_t)~bit;
            observation.updated_mask&=(uint8_t)~bit;
        }
    }
    if(entry_leg && m->localization.registered) entry_observation(m,in,&observation);
    /* In G7 the south-facing ray crosses R6 and changes target at its end.
     * It must not become a south-fence position observation during this leg.
     * Keep the north/front fence and east/west observations. The original
     * snapshot remains intact for freshness and collision protection. */
    if(g7) {
        observation.valid_mask&=(uint8_t)~(1u<<AUTO_REAR);
        observation.updated_mask&=(uint8_t)~(1u<<AUTO_REAR);
    }
    /* Horizontal ridge aisles already use the independently confirmed pair
     * below. Do not apply a second, faster ridge correction on that axis. */
    const bool paired_aisle=m->segment<AUTO_ROUTE_COUNT &&
        m->route.segment[m->segment].kind==AUTO_ROUTE_SURVEY && m->route.segment[m->segment].row!=0u;
    const bool shift_leg=m->segment<AUTO_ROUTE_COUNT && m->route.segment[m->segment].kind==AUTO_ROUTE_SHIFT;
    const bool g4_transfer=shift_leg && m->segment+1u<AUTO_ROUTE_COUNT &&
        m->route.segment[m->segment+1u].aisle==4u;
    if(shift_leg) {
        /* The inward side alternates between the ridge end and the far fence.
         * Keep the outer wall as the X reference for this short transfer. */
        const float shift_x=m->route.segment[m->segment].line.start.x;
        const uint8_t inward=(uint8_t)(1u<<(shift_x>1.5f*m->route.scale?AUTO_LEFT:AUTO_RIGHT));
        observation.valid_mask&=(uint8_t)~inward;
        observation.updated_mask&=(uint8_t)~inward;
    }
    const AutoRoute* pose_route=paired_aisle || entry_leg || shift_leg || g7 ? NULL:&m->route;
    LaserSnapshot pose_observation=observation;
    if(paired_aisle) {
        /* At ridge ends the north/south rays switch between distant fences
         * and ridge faces. They must not pull Y before the confirmed opposing
         * ridge pair takes authority. Keep raw/observation for range guards
         * and correct_aisle_position; odometry propagates Y across this gap. */
        pose_observation.valid_mask&=(uint8_t)~12u;
        pose_observation.updated_mask&=(uint8_t)~12u;
    }
    if(m->localization.registered && !field_pose_step_route(&m->localization,in.wheel,in.imu_yaw,in.odom_ok,&pose_observation,in.now,pose_route)) {
        if(full_map_active(m)) full_map_stop(m,"ODOMETRY_INVALID",true);
    }
    if(entry_leg && (m->localization.used_mask&(1u<<AUTO_LEFT))) {
        m->entry.left_observed=true;
        m->entry.left_sample_ms=in.now-observation.sample_age_ms[AUTO_LEFT];
    }
    if(m->phase==MAP_ARMED) {
        if(!in.healthy||in.now-m->arm_ms>FULL_MAP_ARM_TIMEOUT_MS) full_map_stop(m,"ARM_EXPIRED_OR_PERMIT",false);
        return;
    }
    if(m->phase!=MAP_RUNNING&&m->phase!=MAP_PAUSED) return;
    const bool visual_alignment=in.work_enabled && in.work_align && m->tree_alignment.active &&
        m->tree_alignment.station==m->target_tree;
    MapTreeAlignment* tree=&m->tree_alignment;
    if(visual_alignment && in.work_target_fresh) {
        tree->far_target_seen=in.work_alignment_fresh &&
            in.work_align_direction==tree->direction && in.work_target_dx*tree->direction>45;
        tree->far_target_ms=in.work_target_sample_ms;
    }
    const bool visual_gap=visual_alignment && !m->tree_hold && in.work_online &&
        in.work_target_missing && !in.work_target_fresh && tree->far_target_seen &&
        in.now-tree->far_target_ms<=VISUAL_ALIGN_GAP_MS &&
        (!in.work_align_direction || in.work_align_direction==tree->direction);
    if(visual_alignment && m->tree_alignment.ready && !in.work_centered) {
        m->tree_alignment.started_ms=in.now;m->tree_alignment.steps=0;
        m->tree_alignment.ready=false;
    }
    if(visual_alignment && !m->tree_alignment.skip_pending && in.now-m->tree_alignment.started_ms>=12000u) {
        full_map_stop(m,"TREE_VISION_TIMEOUT",false);return;
    }
    if(in.work_enabled && (!in.work_online || in.work_failed)) {
        const char* reason="WORK_PEER_FAULT";
        if(!in.work_online) reason="WORK_CAN_TIMEOUT";
        else switch(in.work_result) {
        case STATION_RESULT_CANCELLED: reason="WORK_CANCELLED";break;
        case WORK_RESULT_ATTEMPTS: reason="WORK_ATTEMPTS_EXHAUSTED";break;
        case WORK_RESULT_TREE_TIMEOUT: reason="WORK_TREE_TIMEOUT";break;
        case WORK_RESULT_VISION: reason="WORK_VISION_OFFLINE";break;
        case WORK_RESULT_POSITION: reason="WORK_ALIGNMENT_LOST";break;
        case WORK_RESULT_LINK: reason="WORK_CAN_TIMEOUT";break;
        case WORK_RESULT_FEEDBACK: reason="WORK_STOP_FEEDBACK";break;
        case STATION_RESULT_ENCODER_DIRECTION: reason="WORK_ENCODER_DIRECTION";break;
        case STATION_RESULT_ENCODER_LIMIT: reason="WORK_ENCODER_LIMIT";break;
        case STATION_RESULT_ENCODER_STALL: reason="WORK_ENCODER_STALL";break;
        case STATION_RESULT_ENCODER_TIMEOUT: reason="WORK_ENCODER_TIMEOUT";break;
        case STATION_RESULT_RETURN_NOT_READY: reason="WORK_RETURN_NOT_READY";break;
        default: break;
        }
        full_map_stop(m,reason,false);return;
    }
    if(!in.healthy||!m->localization.pose.valid||in.now-m->last_ms>50u) {
        full_map_stop(m,"FEEDBACK_OR_PERMIT",!in.odom_ok);return;
    }
    if(m->visual_work && m->target_tree && !m->visual_stop && !m->work_request_pending &&
       !m->work_scan_ending && in.work_station==m->target_tree && in.work_wait) {
        /* Brake with the live profile, then anchor visual alignment at the
         * measured stop. A synthetic endpoint can make the chassis creep
         * back and forth before it even starts using the camera feedback. */
        m->visual_stop=m->visual_braking=true;
        m->reason="TREE_VISION_BRAKE";
    }
    const float map_dt=(in.now-m->last_ms)*.001f;
    m->last_ms=in.now;
    if(m->phase==MAP_PAUSED && m->user_hold) {m->reason="USER_PAUSE";return;}
    FullMapInput correction_input=in;
    correction_input.laser=observation;
    correct_aisle_position(m,correction_input,work_hold);
    correct_boundary_cross(m,correction_input);
    if(m->g4_entry_offset || m->g2_entry_offset) {
        const float entered=m->route.segment[m->segment].line.start.x-m->localization.pose.pose.x;
        /* Hand off to the real pair. An open aisle can also release the
         * offset past the staggered ends with two fresh, clear transverse rays. */
        const bool open=entered>=.50f && in.laser.frame_fresh && (in.laser.valid_mask&12u)==12u &&
            in.laser.sample_age_ms[AUTO_FRONT]<=LASER_FRAME_TIMEOUT_MS &&
            in.laser.sample_age_ms[AUTO_REAR]<=LASER_FRAME_TIMEOUT_MS &&
            in.laser.accepted_mm[AUTO_FRONT]>600u && in.laser.accepted_mm[AUTO_REAR]>600u;
        if(m->aisle_correction.active || open) m->g4_entry_offset=m->g2_entry_offset=false;
    }
    if(in.now-m->run_ms>900000u||in.now-m->segment_ms>180000u) {full_map_stop(m,"RUN_TIMEOUT",false);return;}
    const AutoEnvelope body={.155f,.155f,.155f,.155f};
    const AutoPose pose=m->localization.pose.pose;
    float aisle_gap[2]={INFINITY,INFINITY};
    if(paired_aisle && !in.work_active &&
       fabsf(auto_wrap_angle(pose.yaw-m->route.segment[m->segment].line.heading))<=.13962634f) {
        for(unsigned ch=AUTO_REAR;ch<=AUTO_FRONT;++ch) {
            if(!aisle_range_fresh(&raw_laser,ch)) continue;
            const bool settled_pair=m->aisle_entry_qualified &&
                (raw_laser.median_valid_mask&(1u<<ch)) &&
                raw_laser.median_age_ms[ch]<=AISLE_RANGE_AGE_MS;
            /* Keep the conservative raw cap while entering. On an established
             * aisle, use the continuous median for the soft speed plan so one
             * return cannot drop qualification. Raw ranges still guard motion. */
            const uint16_t mm=settled_pair?raw_laser.median_mm[ch]:raw_laser.accepted_mm[ch];
            aisle_gap[ch-AUTO_REAR]=aisle_ray_gap(pose,body,ch,mm);
        }
    }
    const float closest_aisle_gap=fminf(aisle_gap[0],aisle_gap[1]);
    const bool aisle_clear=isfinite(aisle_gap[0]) && isfinite(aisle_gap[1]) &&
        closest_aisle_gap>=AISLE_CRUISE_GAP_M;
    const bool aisle_near_unpaired=!m->aisle_correction.active && closest_aisle_gap<AISLE_CAUTION_GAP_M;
    const bool aisle_tight=closest_aisle_gap<=AISLE_CRAWL_GAP_M;
    if(aisle_near_unpaired || aisle_tight) m->aisle_entry_qualified=false;
    else if(aisle_clear && m->aisle_correction.active &&
       fabsf(m->aisle_correction.error_m)<=.020f &&
       fabsf(auto_wrap_angle(pose.yaw-1.570796327f))<=.034906585f)
        m->aisle_entry_qualified=true;
    const AutoSegment seg=m->motion.initialized?m->motion.segment:m->route.segment[m->segment].line;
    if(!auto_route_pose_clear(&m->route,body,m->localization.pose.pose,0)) {
        const AutoPose pose=m->localization.pose.pose;
        bool recoverable=shift_leg &&
            !in.work_active && raw_laser.frame_fresh && (raw_laser.valid_mask&15u)==15u &&
            auto_envelope_inside(body,pose,0,-m->route.scale,3*m->route.scale,4.6f*m->route.scale,0) &&
            auto_route_pose_clear(&m->route,shift_tolerance_body(pose),pose,0);
        for(unsigned ch=0;ch<4u;++ch)
            if(raw_laser.sample_age_ms[ch]>200u || raw_laser.accepted_mm[ch]<=120u) recoverable=false;
        if(!m->envelope_excursion) m->envelope_excursion_ms=in.now;
        if(!recoverable || in.now-m->envelope_excursion_ms>=2000u) {
            full_map_stop(m,"MAP_ENVELOPE",false);return;
        }
        m->envelope_excursion=true;
        m->reason="SHIFT_ENVELOPE_RECOVER";
    } else {
        m->envelope_excursion=false;
        if(!strcmp(m->reason,"SHIFT_ENVELOPE_RECOVER")) m->reason="SHIFT_ENVELOPE_CLEAR";
    }
    bool margin_limited=!auto_route_pose_clear(&m->route,body,m->localization.pose.pose,.02f);
    if(shift_leg && !margin_limited) {
        /* Slow down before entering a ridge-end overlap, including when the
         * current rectangle is still clear just outside the end. */
        AutoPose preview=m->localization.pose.pose;
        const float dy=seg.end.y-preview.y;
        preview.y+=copysignf(fminf(fabsf(dy),SHIFT_LOOKAHEAD_M),dy);
        margin_limited=!auto_route_pose_clear(&m->route,body,preview,.015f);
    }
    m->along_speed_limit=margin_limited?.05f:0;
    if(margin_limited && paired_aisle && !m->envelope_excursion && !m->visual_stop &&
       (m->aisle_correction.active || m->g4_entry_offset) &&
       aisle_range_fresh(&raw_laser,AUTO_REAR) && aisle_range_fresh(&raw_laser,AUTO_FRONT) &&
       raw_laser.accepted_mm[AUTO_REAR]>=125u && raw_laser.accepted_mm[AUTO_FRONT]>=125u &&
       (raw_laser.accepted_mm[AUTO_REAR]<=210u || raw_laser.accepted_mm[AUTO_FRONT]<=210u) &&
       fabsf(auto_wrap_angle(pose.yaw-seg.heading))<=.034906585f) {
        float west,east,south,north;
        /* The 20mm map padding can trigger on a ridge parallel to travel.
         * Keep the real envelope and side braking limits, but avoid a long
         * 50mm/s crawl if both live rays and the X corridor have clearance. */
        if(auto_route_axis_clearance(&m->route,body,pose,0u,&west,&east) &&
           (seg.end.x>=pose.x?east:west)>=.15f &&
           auto_route_axis_clearance(&m->route,body,pose,1u,&south,&north)) {
            const float side_gap=fminf(south,north);
            const float measured_gap=fminf(
                aisle_ray_gap(pose,body,AUTO_REAR,raw_laser.accepted_mm[AUTO_REAR]),
                aisle_ray_gap(pose,body,AUTO_FRONT,raw_laser.accepted_mm[AUTO_FRONT]));
            m->along_speed_limit=speed_blend(.05f,.10f,fminf(side_gap,measured_gap),.002f,.006f);
        }
    }
    const ShiftWallClearance exterior=shift_leg?
        shift_wall_clearance(&m->route,seg,m->localization.pose.pose,body,&raw_laser):(ShiftWallClearance){0};
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
    if(shift_leg) {
        const unsigned side=m->route.segment[m->segment].line.start.x<1.5f?AUTO_LEFT:AUTO_RIGHT;
        const uint8_t side_bit=(uint8_t)(1u<<side),rear_bit=(uint8_t)(1u<<AUTO_REAR);
        const AutoPose p=m->localization.pose.pose;AutoPlane plane;float distance;
        const float closing=fmaxf(0,fmaxf(-in.wheel.vx,-previous_command.vx));
        const float budget=FULL_MAP_MIN_LASER_RANGE_M+closing*.5f+closing*closing/.2f;
        AutoPlane side_plane;float side_distance;
        const bool side_associated=exterior.fresh && field_pose_ray(p,side,&side_plane,&side_distance) &&
            side_plane.surface_id==(side==AUTO_LEFT?1u:2u) &&
            fabsf(in.laser.accepted_mm[side]*.001f-side_distance)<=.025f;
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
           (in.now-m->localization.constrained_ms[0]<=LASER_FRAME_TIMEOUT_MS || side_associated) &&
           m->localization.uncorrected_m[0]<.05f && m->localization.uncorrected_m[1]<1.2f &&
           fabsf(auto_wrap_angle(p.yaw-1.570796327f))<.139626340f &&
           (!(in.laser.valid_mask&rear_bit) || (range.mask&rear_bit)) &&
           field_pose_ray(p,AUTO_REAR,&plane,&distance) && plane.surface_id==4u &&
           distance>fmaxf(.6f,budget+.10f)) {
            const float uncertainty=.02f+.04f*m->localization.uncorrected_m[1];
            range.mm[AUTO_REAR]=(uint16_t)floorf(fminf(distance-uncertainty,
                raw_laser.accepted_mm[AUTO_REAR]*.001f)*1000);
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
    AutoVec2 direction=auto_rotate((AutoVec2){seg.end.x-seg.start.x,seg.end.y-seg.start.y},-m->localization.pose.pose.yaw);
    const float len=hypotf(direction.x,direction.y);
    const AutoPose current=m->localization.pose.pose;
    float entry_control_y=current.y;
    const uint32_t entry_front_age=in.laser.sample_age_ms[AUTO_FRONT];
    const float entry_front_m=in.laser.accepted_mm[AUTO_FRONT]*.001f;
    const bool entry_front_observed=entry_leg && m->entry.north_confirmed &&
        in.laser.frame_fresh && (in.laser.valid_mask&front_bit) &&
        entry_front_age<=200u && entry_front_m>=.10f && entry_front_m<=.75f &&
        projection>.98f && fabsf(in.wheel.wz)<=.10f &&
        field_pose_ray(current,AUTO_FRONT,&north_plane,&north_distance) && north_plane.surface_id==3u;
    bool entry_endpoint_observed=false;
    if(entry_front_observed) {
        const float north_speed=auto_rotate((AutoVec2){in.wheel.vx,in.wheel.vy},current.yaw).y;
        const float measured_y=4.6f*m->route.scale-(entry_front_m+FIELD_POSE_LASER_ORIGIN_M)*projection+
            north_speed*entry_front_age*.001f;
        if(fabsf(measured_y-current.y)<=.15f && (observation.valid_mask&front_bit)) {
            /* The 5 mm/frame fused update lags braking. Use the confirmed
             * wall directly for this endpoint without changing the fused pose. */
            entry_control_y=measured_y;
            entry_endpoint_observed=true;
        }
    }
    /* Keep local visual moves tied to wheel displacement. Side-wall fusion may
     * move the map pose while the wheels are stationary; the envelope still
     * uses that corrected pose. */
    const bool visual_wheel_move=m->visual_work && m->visual_stop && m->visual_hold.motion_reference;
    const float control_x=visual_wheel_move?
        m->localization.wheel_position.x+m->visual_hold.motion_wheel_offset_x:current.x;
    const float to_end=(seg.end.x-control_x)*(seg.end.x-seg.start.x)+
        (seg.end.y-entry_control_y)*(seg.end.y-seg.start.y);
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
    const bool entry_needs_north=entry_leg && current.y>4.09f*m->route.scale &&
        (!m->entry.north_confirmed || in.now-m->localization.constrained_ms[1]>500u);
    const bool g7_front_missing=g7 && (!raw_laser.frame_fresh || !(raw_laser.valid_mask&front_bit) ||
        raw_laser.sample_age_ms[AUTO_FRONT]>LASER_FRAME_TIMEOUT_MS);
    uint8_t required=0;bool safe=!pose_limited && !entry_needs_north && !g7_front_missing;
    float range_speed=segment_speed(m);
    if(g7) {
        const MapWallCorrection* wall=&m->wall_correction;
        const bool north_tracked=wall->active && wall->channel==AUTO_FRONT &&
            in.now-wall->sample_ms<=LASER_FRAME_TIMEOUT_MS;
        /* The north fence is orthogonal to travel. Slow the lateral ramp
         * independently of the usual along-direction stopping budget. */
        const float front_cap=north_tracked?speed_blend(G7_CAUTION_SPEED_MPS,range_speed,
            raw_laser.accepted_mm[AUTO_FRONT]*.001f,G7_FRONT_SLOW_M,G7_FRONT_CRUISE_M):
            G7_CAUTION_SPEED_MPS;
        range_speed=fminf(range_speed,front_cap);
    }
    if(entry_leg && raw_laser.frame_fresh && (raw_laser.valid_mask&front_bit) &&
       raw_laser.sample_age_ms[AUTO_FRONT]<=LASER_FRAME_TIMEOUT_MS && current.y>3.8f*m->route.scale) {
        range_speed=fminf(range_speed,speed_blend(.04f,.35f,
            raw_laser.accepted_mm[AUTO_FRONT]*.001f,.20f,.65f));
    }
    const MapAisleCorrection* pair=&m->aisle_correction;
    const bool clear_aisle=paired_aisle && m->aisle_entry_qualified &&
        !m->g4_entry_offset && !m->g2_entry_offset &&
        !margin_limited && pair->active && !pair->frozen &&
        closest_aisle_gap>=AISLE_CAUTION_GAP_M && aisle_clear && fabsf(pair->error_m)<=.018f &&
        fabsf(auto_wrap_angle(current.yaw-seg.heading))<=.034906585f;
    if(shift_leg) {
        const float shift_cross=fabsf(current.x-seg.start.x);
        const float shift_yaw=fabsf(auto_wrap_angle(current.yaw-seg.heading));
        const MapWallCorrection* wall=&m->wall_correction;
        float cruise=exterior.fresh && exterior.gap_m>=.032f && !margin_limited?
            SHIFT_CLEAR_CAP_MPS:SHIFT_CRUISE_CAP_MPS;
        AutoPose preview=current;
        const float remaining=seg.end.y-current.y;
        preview.y+=copysignf(fminf(fabsf(remaining),.20f),remaining);
        /* 换垄提速：2026-11-20 实车日志显示 0.70 档整轮未命中，但放宽进入条件
         * 会让 G4 外墙回放误入高速档、破坏已知墙距保护，因此保留原有严格门控。
         * 提速改由提高修正档下限(0.12->0.20)与收窄横差/航向降速区间承担，
         * 基础巡航档与 40mm 墙距修正带保持不变。 */
        if(!margin_limited && exterior.fresh && exterior.gap_m>=.030f && wall->active && wall->axis==0u &&
           in.now-wall->sample_ms<=300u && fabsf(wall->error_m)<=.018f &&
           shift_cross<=.018f && shift_yaw<=.034906585f &&
           auto_route_pose_clear(&m->route,body,current,.03f) &&
           auto_route_pose_clear(&m->route,body,preview,.03f))
            cruise=SHIFT_CENTERED_CAP_MPS;
        /* 2026-11-20: the old 0.12 m/s correction floor plus a 12mm/2deg blend
         * held recorded shift legs at 0.13..0.22 m/s even after centering.
         * Keep the cruise caps unchanged (G4 outer-wall protection), but raise
         * the correction floor and widen the full-speed cross/yaw band. */
        const float cross_cap=speed_blend(cruise,SHIFT_CORRECTION_CAP_MPS,shift_cross,.018f,.040f);
        const float yaw_cap=speed_blend(cruise,SHIFT_CORRECTION_CAP_MPS,shift_yaw,.052359878f,.087266463f);
        const float wall_cap=speed_blend(.05f,cruise,exterior.gap_m,SHIFT_WALL_STOP_M,SHIFT_WALL_CORRECT_M);
        range_speed=fminf(range_speed,fminf(wall_cap,fminf(cross_cap,yaw_cap)));
        if(g4_transfer && exterior.fresh) {
            /* Ease off before a delayed fence update reaches the existing
             * freshness stop. An aged sample cannot release the brake. */
            const float age=(float)raw_laser.sample_age_ms[AUTO_RIGHT];
            range_speed=fminf(range_speed,speed_blend(range_speed,.18f,
                age,120.0f,(float)SHIFT_WALL_MAX_AGE_MS));
        }
    }
    if(paired_aisle && !m->aisle_entry_qualified) {
        const AutoSegment row=m->route.segment[m->segment].line;
        const bool tracking=aisle_clear && !margin_limited && pair->confirmations>=2u &&
            in.now-pair->sample_ms[0]<=200u && in.now-pair->sample_ms[1]<=200u &&
            fabsf(pair->error_m)<=.012f && fabsf(current.y-row.start.y)<=.020f &&
            fabsf(auto_wrap_angle(current.yaw-row.heading))<=.034906585f &&
            auto_route_pose_clear(&m->route,body,current,.03f);
        /* A lost pair must never release the cap just because the chassis
         * passed the nominal end of the entrance zone. */
        range_speed=fminf(range_speed,tracking?SURVEY_ENTRY_TRACKED_CAP_MPS:SURVEY_ENTRY_CAP_MPS);
    }
    /* A raw return at the body edge takes priority over the pair's short
     * dropout lease. Keep enough time for correction before the hard stop. */
    if(aisle_tight) range_speed=fminf(range_speed,AISLE_NEAR_SPEED_MPS);
    float aisle_entry_cross=0;
    if(aisle_near_unpaired && !m->visual_stop && !m->tree_hold) {
        const bool front_close=aisle_gap[1]<aisle_gap[0];
        const unsigned channel=front_close?AUTO_FRONT:AUTO_REAR;
        const unsigned opposite=front_close?AUTO_REAR:AUTO_FRONT;
        const float nearest=aisle_gap[channel-AUTO_REAR];
        const float opposite_gap=aisle_gap[opposite-AUTO_REAR];
        const float crawl=fminf(range_speed,AISLE_NEAR_SPEED_MPS);
        range_speed=speed_blend(crawl,range_speed,nearest,AISLE_CRAWL_GAP_M,AISLE_CAUTION_GAP_M);
        /* Steer away after the independent median agrees and the opposite
         * raw ray is clear. The confirmed pair owns correction when active. */
        if(isfinite(opposite_gap) && opposite_gap>=AISLE_MIN_OPPOSITE_GAP_M &&
           (raw_laser.median_valid_mask&(1u<<channel)) &&
           raw_laser.median_age_ms[channel]<=LASER_FRAME_TIMEOUT_MS) {
            const float median_gap=aisle_ray_gap(current,body,channel,raw_laser.median_mm[channel]);
            const float confirmed_gap=fmaxf(nearest,median_gap);
            if(confirmed_gap<AISLE_CAUTION_GAP_M)
                aisle_entry_cross=copysignf(fminf(.040f,1.5f*(AISLE_CAUTION_GAP_M-confirmed_gap)),
                    front_close?1.0f:-1.0f);
        }
    }
    if(m->g4_entry_offset) range_speed=fminf(range_speed,G4_ENTRY_CAP_MPS);
    if(m->g2_entry_offset) range_speed=fminf(range_speed,G2_ENTRY_CAP_MPS);
    if(park_leg) range_speed=fminf(range_speed,speed_blend(.15f,.20f,to_end/len,.20f,.45f));
    if(paired_aisle && m->route.segment[m->segment].aisle==6u) {
        /* The westbound ray in G6 stayed far until a short return at x=.67m.
         * Brake before reaching that blind portion, even with a far reading. */
        const float exit_gap=current.x-m->route.segment[m->segment].line.end.x;
        range_speed=fminf(range_speed,speed_blend(.10f,range_speed,exit_gap,.55f,.95f));
    }
    const float survey_cap=clear_aisle?VISUAL_CLEAR_SURVEY_SPEED_MPS:VISUAL_SURVEY_SPEED_MPS;
    if(m->visual_work && m->target_tree) range_speed=fminf(range_speed,survey_cap);
    /* Side-corridor Y is odometry until the ridge pair is actually visible.
     * Acquire a centered pair before accelerating through the first tree leg.
     * Keep qualification through ordinary producer gaps and later tree stops. */
    if(paired_aisle && m->route.segment[m->segment].row==1u && !m->aisle_entry_qualified)
        range_speed=fminf(range_speed,.15f);
    if(visual_alignment) {
        float adjust=VISUAL_FINE_ADJUST_MPS;
        if(in.work_target_fresh && in.work_target_dx*in.work_align_direction>0) {
            const float error=fabsf((float)in.work_target_dx);
            /* Allow for the next camera frame and drive coast before the
             * center band. G1 crossed the whole band at the old 0.08 m/s floor. */
            adjust=error<=45.0f?
                speed_blend(VISUAL_NEAR_ADJUST_MPS,VISUAL_FINE_ADJUST_MPS,
                    error,STATION_VISION_STOP_PX,45.0f):
                speed_blend(VISUAL_FINE_ADJUST_MPS,VISUAL_FAR_ADJUST_MPS,error,45.0f,55.0f);
        }
        range_speed=fminf(range_speed,visual_gap?VISUAL_ALIGN_GAP_MPS:adjust);
    }
    if(in.work_enabled && m->target_tree && in.work_station==m->target_tree) {
        if(m->visual_work && !m->visual_stop && !in.work_done) {
            float approach=survey_cap;
            bool close_target=false;
            if(in.work_slow) {
                approach=fminf(clear_aisle?VISUAL_CLEAR_GMG_APPROACH_MPS:VISUAL_GMG_APPROACH_MPS,m->approach_speed);
                const float ahead_px=in.work_target_dx*m->motion.tangent.x;
                if(in.work_target_fresh && ahead_px>=-40.0f) {
                    const float tracked=fminf(clear_aisle?VISUAL_CLEAR_TRACKED_APPROACH_MPS:VISUAL_TRACKED_APPROACH_MPS,m->approach_speed);
                    const float stop=fminf(VISUAL_STOP_APPROACH_MPS,tracked);
                    close_target=ahead_px<80.0f;
                    approach=close_target?speed_blend(stop,tracked,ahead_px,40.0f,80.0f):
                        speed_blend(tracked,fminf(.30f,m->approach_speed),ahead_px,80.0f,130.0f);
                    m->visual_near_limit=close_target?approach:0;
                    m->visual_near_ms=in.now;
                } else if(m->visual_near_limit>0 && in.now-m->visual_near_ms<=VISUAL_NEAR_HOLD_MS) {
                    /* Do not accelerate through a near tree during a short
                     * loss of dx. The held cap grants no alignment evidence. */
                    approach=m->visual_near_limit;
                }
            }
            if(in.work_tree_slow && !in.work_slow) {
                /* A visible tree starts the approach before green is confirmed.
                 * The drive profile still slews to the smaller planning cap. */
                approach=fminf(survey_cap,VISUAL_TREE_APPROACH_MPS);
                close_target=true;
            }
            if(m->visual_approach_limit==0) m->visual_approach_limit=approach;
            else m->visual_approach_limit+=fmaxf(-.50f*map_dt,
                fminf(.30f*map_dt,approach-m->visual_approach_limit));
            if(close_target) m->visual_approach_limit=fminf(m->visual_approach_limit,approach);
            range_speed=fminf(range_speed,m->visual_approach_limit);
        }
        else if(!m->visual_work && in.work_slow && fabsf(m->motion.segment.end.x-m->localization.pose.pose.x)<=.35f)
            range_speed=fminf(range_speed,m->approach_speed);
    }
    uint8_t blocked_directions=0;
    m->range_near_mask=m->range_brake_mask=m->range_stale_mask=0;
    for(unsigned i=0;i<4;++i) {
        /* 入口正向驶离起点时，后激光照向出发区，不参与起步判断。
         * 实际倒车、反向修正及后续路段仍检查后方测距。 */
        if(entry_leg && i==AUTO_REAR && ahead[i]<-.2f &&
           toward[i]<=.01f && commanded[i]<=.01f) {
            m->range_budget_m[i]=0;
            continue;
        }
        AutoPlane plane;float predicted;
        const bool wall=field_pose_ray(m->localization.pose.pose,i,&plane,&predicted);
        const uint8_t bit=(uint8_t)(1u<<i);
        /* Visible near returns remain protective even when a map ray has no wall. */
        if((raw_laser.valid_mask&bit)&&raw_laser.accepted_mm[i]<=FULL_MAP_MIN_LASER_RANGE_MM) {
            m->range_near_mask|=bit;safe=false;
        }
        const float speed=fmaxf(toward[i],commanded[i]);
        /* Entry has separate continuous planning and emergency envelopes.
         * A disputed return can brake continuously before a forced pause. */
        const bool entry_forward=entry_leg && i==AUTO_FRONT;
        const float stop_decel=entry_forward?.90f:ahead[i]>.2f?.20f:.10f;
        const float stop_delay=entry_forward?.10f+fminf(.30f,raw_laser.sample_age_ms[i]*.001f):.50f;
        const float budget=FULL_MAP_MIN_LASER_RANGE_M+speed*stop_delay+speed*speed/(2*stop_decel);
        m->range_budget_m[i]=budget;
        const bool entry_raw=entry_forward && raw_laser.frame_fresh && (raw_laser.valid_mask&bit) &&
            raw_laser.sample_age_ms[i]<=LASER_FRAME_TIMEOUT_MS;
        if(entry_raw && raw_laser.accepted_mm[i]*.001f<=budget) {
            m->range_brake_mask|=bit;safe=false;
        }
        /* The right ray sees changing ridge ends on the entry, not a wall
         * reference. It cannot veto this leg through ordinary range checks. */
        if(entry_leg && i==AUTO_RIGHT) continue;
        const bool entry_front=entry_leg && i==AUTO_FRONT && (range.mask&front_bit);
        if(!entry_front && (ahead[i]>.2f||toward[i]>.01f) && (in.laser.valid_mask&bit) &&
           in.laser.accepted_mm[i]*.001f<=budget) {m->range_brake_mask|=bit;safe=false;}
        if(ahead[i]>.2f && in.laser.frame_fresh && ((in.laser.valid_mask|range.mask)&bit)) {
            /* Brake before entering the existing emergency pause envelope.
             * Include frame age and one producer interval in the lookahead. */
            const float delay=(entry_forward?.20f:.55f)+fminf(.30f,in.laser.sample_age_ms[i]*.001f);
            const uint16_t planning_mm=entry_raw && raw_laser.accepted_mm[i]<range.mm[i]?
                raw_laser.accepted_mm[i]:range.mm[i];
            const float available=fmaxf(0,planning_mm*.001f-FULL_MAP_MIN_LASER_RANGE_M-.02f);
            /* Entry planning uses gentler braking than its emergency envelope.
             * Entry retains 0.04m/s of tracking reserve; other rays 0.02m/s. */
            const float decel=entry_forward?.40f:stop_decel;
            const float closing_limit=sqrtf(decel*decel*delay*delay+2*decel*available)-decel*delay;
            const float tracking_reserve=entry_forward?.04f:.02f;
            const float cap=fmaxf(0,closing_limit-tracking_reserve)/ahead[i];
            if(entry_raw && cap<range_speed) m->range_control_mm[i]=planning_mm;
            range_speed=fminf(range_speed,cap);
        }
        if(!wall) continue;
        if(ahead[i]>.2f||toward[i]>.10f || (fabsf(ahead[i])>.2f && commanded[i]>.0001f)) required|=bit;
        if(ahead[i]<=.2f && !((in.laser.valid_mask|range.mask)&bit)) blocked_directions|=bit;
        if(!(required&bit)) continue;
        if(!in.laser.frame_fresh||!((in.laser.valid_mask|range.mask)&bit)) {m->range_stale_mask|=bit;safe=false;continue;}
        if(range.mm[i]*.001f<=budget) {m->range_brake_mask|=bit;safe=false;}
    }
    const bool other_ranges_safe=safe;
    bool recover_wall=false;
    if(exterior.bit) {
        const AutoVec2 wheel_map=auto_rotate((AutoVec2){in.wheel.vx,in.wheel.vy},current.yaw);
        const AutoVec2 command_map=auto_rotate((AutoVec2){previous_command.vx,previous_command.vy},current.yaw);
        const float closing=fmaxf(0,fmaxf(exterior.outward*wheel_map.x,exterior.outward*command_map.x));
        const bool resuming=m->phase==MAP_PAUSED;
        const float stopping_gap=SHIFT_WALL_STOP_M+SHIFT_WALL_DELAY_S*closing+
            closing*closing/(2*SHIFT_WALL_BRAKE_MPS2);
        const float required_gap=resuming?fmaxf(SHIFT_WALL_RESUME_M,stopping_gap):stopping_gap;
        if(exterior.gap_m<=SHIFT_WALL_CORRECT_M || resuming) {
            required|=exterior.bit;
            if(!exterior.fresh) {m->range_stale_mask|=exterior.bit;safe=false;}
        }
        if(exterior.gap_m<=required_gap) {
            recover_wall=other_ranges_safe && exterior.fresh && !in.work_active && !in.work_wait &&
                exterior.gap_m>0 && exterior.gap_m<=SHIFT_WALL_RESUME_M;
            m->range_brake_mask|=exterior.bit;safe=false;
        }
    }
    if(m->required_mask!=required) {
        m->recovery_frames=0;m->recovery_updates=0;m->stable=false;
    }
    m->required_mask=required;
    if(near_recovery_step(m,in,raw_laser,pose_limited||entry_needs_north,exterior,recover_wall,other_ranges_safe)) return;
    if(!safe) {
        m->tree_hold_stable=false;
        if(m->phase!=MAP_PAUSED) {
            m->phase=MAP_PAUSED;m->pause_ms=in.now;m->stable=false;m->recovery_frames=0;
            m->recovery_updates=0;
            m->recovery_frame=in.laser.frame_count;auto_motion_cancel(&m->motion);
        }
        m->reason=pose_limited?"POSE_OBSERVATION_PAUSE":entry_needs_north?"ENTRY_NORTH_PAUSE":
            g7_front_missing?"G7_FRONT_PAUSE":"RANGE_PAUSE";
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
            else if(!strcmp(m->reason,"G7_FRONT_PAUSE")) timeout_reason="G7_FRONT_TIMEOUT";
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
            const uint8_t updated=(uint8_t)((in.laser.updated_mask&~exterior.bit)|
                (raw_laser.updated_mask&exterior.bit));
            m->recovery_updates|=(uint8_t)(updated&required);
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
    if(tree->skip_pending) {
        m->reason="TREE_VISION_SKIP_WAIT";
        if(in.work_station==m->target_tree && in.work_done && in.work_result==WORK_RESULT_SKIPPED && in.stationary)
            finish_tree(m,in,true);
        else if(in.now-tree->missing_since_ms>=WORK_VISION_SKIP_ACK_MS)
            full_map_stop(m,"TREE_VISION_SKIP_ACK_TIMEOUT",false);
        return;
    }
    if(visual_alignment && in.work_can_skip_alignment && in.work_station==m->target_tree && in.work_target_missing) {
        if(!tree->target_missing) {
            tree->target_missing=true;
            tree->missing_since_ms=in.now;
        }
        if(m->tree_hold && in.stationary && in.now-tree->missing_since_ms>=WORK_VISION_MISSING_SKIP_MS) {
            tree->skip_pending=true;
            tree->missing_since_ms=in.now;
            m->tree_hold_stable=false;
            m->reason="TREE_VISION_SKIP_WAIT";
            return;
        }
    } else tree->target_missing=false;
    if(m->tree_hold) {
        const AutoPose pose=m->localization.pose.pose;
        const AutoVec2 wheel=m->localization.wheel_position;
        if(visual_parked && !m->visual_hold.active) capture_visual_hold(m);
        /* The map can keep accepting wall corrections with the vehicle
         * motionless. Use physical travel relative to a visual stop. */
        const AutoVec2 position=visual_parked?wheel:(AutoVec2){pose.x,pose.y};
        const AutoVec2 target=visual_parked?m->visual_hold.wheel:m->motion.segment.end;
        const float position_delta=fmaxf(fabsf(position.x-target.x),fabsf(position.y-target.y));
        const float ridge_delta=work_hold || visual_parked?work_ridge_displacement(m,in.now):0;
        const float distance=fmaxf(position_delta,ridge_delta);
        const float yaw=fabsf(auto_wrap_angle(pose.yaw-m->motion.segment.heading));
        const bool displaced=distance>.03f || yaw>m->motion.config.finish_yaw_tolerance;
        if(work_hold) {
            if(displaced) {
                if(!m->work_pose_excursion) {
                    m->work_pose_excursion=true;
                    m->work_pose_excursion_ms=in.now;
                }
                if(distance>WORK_POSE_HARD_DISTANCE_M || yaw>WORK_POSE_HARD_YAW_RAD ||
                   in.now-m->work_pose_excursion_ms>=WORK_POSE_CONFIRM_MS)
                    full_map_stop(m,"WORK_STATION_MOVED",false);
                else m->reason="WORK_POSE_SETTLING";
                return;
            }
            m->work_pose_excursion=false;
        }
        const bool recenter=!work_hold && m->aisle_correction.active && m->aisle_correction.centering;
        if(recenter || displaced) {
            if(visual_parked) {
                m->visual_hold.motion_wheel_offset_x=pose.x-wheel.x;
                m->visual_hold.motion_reference=true;
                const float hold_x=pose.x+(target.x-wheel.x);
                if(tree->active && tree->station==m->target_tree) tree->target_x=hold_x;
                else m->visual_stop_x=hold_x;
            }
            m->tree_hold=m->tree_hold_stable=false;
            if(!begin(m,in.now)) full_map_stop(m,recenter?"TREE_RECENTER_FAILED":"TREE_REPOSITION_FAILED",false);
            else m->reason=recenter?"TREE_RECENTER":"TREE_REPOSITION";
            return;
        }
        if(!in.stationary || hypotf(in.wheel.vx,in.wheel.vy)>=.01f || fabsf(in.wheel.wz)>=.02f) {
            m->tree_hold_stable=false;return;
        }
        if(in.work_enabled && in.work_align && in.work_station==m->target_tree &&
           refine_tree_position(m,in)) return;
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
        finish_tree(m,in,false);
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
    if(g4_transfer) {
        const float remaining_south=current.y-seg.end.y;
        /* The turn brakes facing the same heading as the aisle. Near the
         * endpoint, repeating reverse translation and yaw-only preparation
         * wastes the safe handoff when stop error and IMU jitter straddle
         * the tight window. Keep the original early stop and grant a small
         * overrun only with real east-wall clearance and a clear envelope. */
        const bool handoff_clear=remaining_south<=ENTRY_SHIFT_STOP_TOLERANCE_M &&
            remaining_south>=-G4_HANDOFF_OVERRUN_M && exterior.fresh &&
            exterior.gap_m>=SHIFT_WALL_RESUME_M &&
            auto_route_pose_clear(&m->route,body,current,.010f);
        m->motion.config.position_tolerance=handoff_clear?
            G4_HANDOFF_OVERRUN_M:ENTRY_SHIFT_STOP_TOLERANCE_M;
        m->motion.config.finish_yaw_tolerance=handoff_clear?
            G4_HANDOFF_FINISH_YAW_RAD:ENTRY_SHIFT_FINISH_YAW_RAD;
    }
    const float endpoint_x=visual_alignment || m->visual_work?m->localization.pose.pose.x:endpoint_feedback(m,in,&observation);
    input.endpoint_observation_unsettled=m->target_tree && !m->visual_work && !visual_alignment && !m->endpoint.ready;
    input.endpoint_stop_tolerance=m->target_tree && !m->visual_work && !visual_alignment?.015f:0;
    input.stop_requested=m->visual_braking;
    if(visual_alignment) {
        input.stop_requested=in.work_centered ||
            (!visual_gap && (!in.work_alignment_fresh || !in.work_align_direction ||
                            in.work_align_direction!=tree->direction || in.work_target_missing)) ||
            (in.work_target_fresh && in.work_target_dx*tree->direction<=STATION_VISION_STOP_PX);
        tree->ready=false;
    }
    float cross=shift_leg?current.x-seg.start.x:continuous_cross_feedback(m,in,&seg,aisle_entry_cross);
    if(shift_leg && exterior.fresh && exterior.gap_m<SHIFT_WALL_CORRECT_M) {
        /* A short exterior ray can warn ahead of the rate-limited pose fix.
         * Bias correction inwards while the opposite ridge still bounds it. */
        const float wall_error=fminf(.06f,SHIFT_WALL_CORRECT_M-exterior.gap_m);
        cross=exterior.outward*fmaxf(exterior.outward*cross,wall_error);
    }
    if(shift_leg) {
        m->motion.config.cross_kp=speed_blend(3.2f,3.5f,fabsf(cross),.025f,.040f);
        /* At the G7 exit a missed fence frame can remove one correction
         * component. Keep that change within the steering gate's angle. */
        if(m->segment==3u) input.cross_to_along_ratio=.10f;
        else if(!margin_limited && exterior.fresh && exterior.gap_m>=.028f && fabsf(cross)<=.025f)
            input.cross_to_along_ratio=.25f;
        /* A small acceptable cross error should not swing the wheels off the
         * travel direction just as the transfer tapers to its endpoint. */
        if(g4_transfer && fabsf(seg.end.y-current.y)<=SHIFT_LOOKAHEAD_M &&
           exterior.fresh && exterior.gap_m>=SHIFT_WALL_RESUME_M+.003f &&
           fabsf(cross)<=m->motion.config.finish_cross_tolerance)
            input.cross_to_along_ratio=.22f;
    }
    if(paired_aisle && !m->tree_alignment.active) {
        const bool first_row=m->route.segment[m->segment].row==1u;
        const bool acquiring=!m->aisle_entry_qualified && !m->aisle_correction.centering;
        const bool tracking=m->aisle_entry_qualified && !m->visual_stop;
        const bool crawl=!m->aisle_entry_qualified || m->along_speed_limit>0;
        if(first_row) m->motion.config.cross_tolerance=acquiring?.012f:.005f;
        /* Keep the entry policy until the paired ranges confirm the aisle.
         * On the straight, let small yaw errors produce small corrections. */
        m->motion.config.yaw_tolerance=tracking?SURVEY_TRACKING_YAW_TOLERANCE_RAD:
            crawl?.026179939f:.017453293f;
        /* Small corrections stay proportional during entry and map-limited
         * crawl. Fixed minimum commands otherwise turn the wheels sharply
         * each time the error crosses the deadband at low travel speed. */
        m->motion.config.lateral_min=crawl || first_row?0.0f:.020f;
        m->motion.config.wz_min=crawl || tracking || first_row?0.0f:.040f;
        if(crawl && !m->visual_stop && !m->aisle_correction.centering && aisle_clear)
            input.cross_to_along_ratio=.35f;
        if(m->route.segment[m->segment].aisle==4u && margin_limited &&
           !m->visual_stop && fabsf(to_end)>.05f*len) {
            /* Avoid small yaw reversals and lateral steering transients while threading a ridge end. */
            m->motion.config.yaw_tolerance=.043633231f;
            m->motion.config.wz_min=0;
            input.cross_to_along_ratio=.20f;
        }
    }
    if(!entry_leg && !m->visual_stop) {
        float yaw_strength=0;
        if(!margin_limited) {
            if(shift_leg && exterior.fresh)
                yaw_strength=speed_blend(0,1,exterior.gap_m,.025f,.045f);
            else if(paired_aisle && aisle_clear)
                yaw_strength=speed_blend(0,1,closest_aisle_gap,.022f,.042f);
            else if(m->route.segment[m->segment].aisle==7u &&
                aisle_range_fresh(&raw_laser,AUTO_FRONT) && aisle_range_fresh(&raw_laser,AUTO_REAR))
                yaw_strength=speed_blend(0,1,fminf(
                    aisle_ray_gap(current,body,AUTO_FRONT,raw_laser.accepted_mm[AUTO_FRONT]),
                    aisle_ray_gap(current,body,AUTO_REAR,raw_laser.accepted_mm[AUTO_REAR])),.025f,.045f);
        }
        m->motion.config.yaw_kp=1.6f+yaw_strength;
        m->motion.config.angular_accel=.30f+.05f*yaw_strength;
    }
    if(fabsf(seg.end.y-seg.start.y)>fabsf(seg.end.x-seg.start.x)) input.pose.pose.x=seg.start.x+cross;
    else {
        input.pose.pose.x=control_x;
        input.pose.pose.y=seg.start.y+cross;
    }
    if(m->endpoint.active && !visual_alignment && !m->visual_work) input.pose.pose.x=endpoint_x;
    if(entry_leg) {
        input.pose.pose.y=entry_control_y;
        /* At very short range, only a fresh associated ray can request a
         * retreat or confirm enough clearance to launch the eastbound leg. */
        input.endpoint_observation_unsettled=!entry_endpoint_observed ||
            in.laser.accepted_mm[AUTO_FRONT]<G7_HANDOFF_FRONT_MM || fabsf(entry_control_y-current.y)>.020f;
        const bool front_tight=raw_laser.frame_fresh && (raw_laser.valid_mask&front_bit) &&
            raw_laser.sample_age_ms[AUTO_FRONT]<=200u && raw_laser.accepted_mm[AUTO_FRONT]<G7_HANDOFF_FRONT_MM;
        if(front_tight) {
            /* Recover clearance before a small yaw correction swings a corner north. */
            m->motion.config.yaw_tolerance=.087266463f;
            input.hold_along=!entry_endpoint_observed || to_end>=0;
        } else m->motion.config.yaw_tolerance=.017453293f;
    }
    map_cross_limits(m,raw_input,exterior,g4_transfer,&input);
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
    if((m->visual_braking || visual_alignment) && input.stop_requested && in.stationary &&
       hypotf(in.wheel.vx,in.wheel.vy)<.01f && fabsf(in.wheel.wz)<.02f &&
       hypotf(out.command.vx,out.command.vy)<.00001f && fabsf(out.command.wz)<.00001f) {
        const float stopped_x=control_x;
        if(m->visual_braking) {
            m->visual_stop_x=stopped_x;
            m->visual_braking=false;
        } else m->tree_alignment.target_x=stopped_x;
        m->motion.segment.end.x=stopped_x;
        if(m->visual_work) capture_visual_hold(m);
        m->tree_hold=true;m->tree_hold_stable=false;
        m->reason="TREE_VISION_WAIT";
        return;
    }
    if(out.phase==AUTO_MOTION_DONE) {
        if(m->visual_work && m->target_tree && !m->visual_stop) {
            m->work_scan_ending=true;m->command=m->hint=(AutoTwist){0};
            if(in.work_station!=m->target_tree || !in.work_done || m->work_request_pending) {
                m->reason="VISION_SCAN_END";return;
            }
            m->target_tree=0;m->work_scan_ending=false;
        }
        if(m->target_tree) {
            if(m->visual_work && m->visual_stop) capture_visual_hold(m);
            m->tree_hold=true;m->tree_hold_stable=false;
            m->command=m->hint=(AutoTwist){0};m->reason="TREE_DWELL";return;
        }
        const unsigned aisle=m->route.segment[m->segment].aisle;
        if(aisle) m->coverage|=(uint8_t)(1u<<(aisle-1));
        m->segment+=(m->segment==0u?2u:1u);m->command=m->hint=(AutoTwist){0};
        /* Tree targets and pause recovery stay on the same aisle. A new
         * route segment needs its own surface and independent-frame history. */
        m->aisle_correction=(MapAisleCorrection){0};
        m->aisle_entry_qualified=false;
        const unsigned next_aisle=shift_leg && m->segment<AUTO_ROUTE_COUNT?m->route.segment[m->segment].aisle:0u;
        m->g4_entry_offset=next_aisle==4u;m->g2_entry_offset=next_aisle==2u;
        m->g2_wheel_guard=next_aisle==2u;
        if(m->g2_wheel_guard) m->g2_wheel_offset_y=current.y-m->localization.wheel_position.y;
        m->wall_correction=(MapWallCorrection){0};
        m->cross_feedback=(MapCrossFeedback){0};
        if(m->segment==AUTO_ROUTE_COUNT) {
            m->phase=MAP_DONE;
            m->reason=m->trees_unresolved?"ROUTE_COMPLETE_WITH_UNRESOLVED":"ROUTE_COMPLETE";
            return;
        }
        if(!begin(m,in.now)) {full_map_stop(m,"MOTION_CONFIG",false);return;}
        m->reason="NEXT_SEGMENT";
    }
}
