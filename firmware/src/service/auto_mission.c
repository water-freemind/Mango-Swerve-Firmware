#include "auto_mission.h"
#include <math.h>
#include <string.h>

static void fail(AutoMission* m,AutoMissionError error) {
    if(m->out.phase==AUTO_FAULT) return;
    m->out.phase=AUTO_FAULT; m->out.first_fault=error;
    m->out.command=m->out.steering_hint=(AutoTwist){0}; m->out.qualified=false;
    auto_motion_cancel(&m->motion);
    if(m->work.out.phase!=AUTO_WORK_IDLE && m->work.out.phase!=AUTO_WORK_DONE) {
        AutoWorkInput cancel={.cancel=true}; m->out.work=auto_work_step(&m->work,cancel);
    }
}
static bool stationary(AutoMissionInput in) {
    return in.motion.stationary && hypotf(in.motion.measured.vx,in.motion.measured.vy)<.01f && fabsf(in.motion.measured.wz)<.02f;
}
static bool begin_segment(AutoMission* m,uint32_t now) {
    return auto_motion_begin(&m->motion,&m->config.motion,m->config.route.segment[m->out.segment_index].line,now);
}
static float travel_time(float d,AutoMotionConfig c) {
    float inverse=1/c.accel+1/c.decel;
    float peak=fminf(c.vmax,sqrtf(2*fmaxf(0,d)/inverse));
    return peak>0 ? d/peak+peak*.5f*inverse:0;
}
static float remaining(AutoMission* m) {
    float total=m->config.reserve_s;
    for(unsigned i=m->out.segment_index;i<AUTO_ROUTE_COUNT;++i) {
        AutoSegment s=m->config.route.segment[i].line;
        float length=hypotf(s.end.x-s.start.x,s.end.y-s.start.y);
        if(i==m->out.segment_index) length=fmaxf(0,length-fmaxf(0,m->motion.out.progress));
        total+=travel_time(length,m->config.motion)+m->config.motion.settle_s+m->config.segment_allowance_s;
    }
    return total;
}
bool auto_mission_init(AutoMission* m,const AutoMissionConfig* c,uint32_t now) {
    if(!m) return false;
    memset(m,0,sizeof(*m));
    if(!c || !auto_config_valid(&c->geometry) || !auto_route_clear(&c->route,c->geometry.stowed,c->footprint_margin) ||
       !c->deadline_ms || c->deadline_ms>300000 || !c->pause_timeout_ms || !c->pause_stable_ms ||
       c->pause_stable_ms>=c->pause_timeout_ms || !c->stop_work_ms || !c->segment_timeout_ms ||
       !isfinite(c->reserve_s)||c->reserve_s<0||!isfinite(c->segment_allowance_s)||c->segment_allowance_s<0 ||
       (c->simulation_only != c->geometry.simulation_only) || (c->simulation_only != c->work.simulation_only) ||
       (!c->simulation_only && (!auto_config_hardware_ready(&c->geometry)||!c->map_confirmed||!c->work.contract_confirmed))) {
        fail(m,AUTO_MISSION_CONFIG); return false;
    }
    m->config=*c;
    if(!begin_segment(m,now)) { fail(m,AUTO_MISSION_CONFIG); return false; }
    AutoWork probe={0};
    if(!auto_work_begin(&probe,c->work,1,0,1,now)) { fail(m,AUTO_MISSION_CONFIG); return false; }
    auto_motion_cancel(&m->motion); m->out.phase=AUTO_PRECHECK; m->last_ms=now; m->initialized=true;
    return true;
}
AutoMissionOutput auto_mission_step(AutoMission* m,AutoMissionInput in) {
    if(!m) return (AutoMissionOutput){.phase=AUTO_FAULT,.first_fault=AUTO_MISSION_CONFIG};
    if(!m->initialized||m->out.phase==AUTO_FAULT||m->out.phase==AUTO_FINISHED) return m->out;
    uint32_t now=in.motion.now_ms;
    m->out.command=m->out.steering_hint=(AutoTwist){0};
    if(in.start_event!=AUTO_START_NONE && !m->out.started) {
        if(in.start_event!=AUTO_START_LOCAL_APPROVED) { fail(m,AUTO_MISSION_START); return m->out; }
        m->out.started=true; m->start_ms=now; /* includes PRECHECK duration after cold-start event */
    }
    if(m->out.started) {
        m->out.elapsed_s=(uint32_t)(now-m->start_ms)*.001f;
        if((uint32_t)(now-m->start_ms)>=m->config.deadline_ms ||
           in.penalty_ms>=m->config.deadline_ms-(uint32_t)(now-m->start_ms)) {
            fail(m,AUTO_MISSION_DEADLINE); return m->out;
        }
    }
    if(in.cancel||in.motion.hard_stop) { fail(m,AUTO_MISSION_HEALTH); return m->out; }
    uint32_t dt=now-m->last_ms; m->last_ms=now;
    if(m->out.phase!=AUTO_PRECHECK && m->out.phase!=AUTO_WAIT_START && dt>50) {
        fail(m,AUTO_MISSION_TIMING); return m->out;
    }
    if(m->out.phase==AUTO_PRECHECK || m->out.phase==AUTO_WAIT_START) {
        if(!in.precheck_ready || !in.motion.healthy || !in.localization_ok || !in.observation_ok ||
           !in.tools_stowed || !stationary(in)) { m->out.phase=AUTO_PRECHECK; return m->out; }
        m->out.phase=AUTO_WAIT_START;
        if(!m->out.started) return m->out;
        float s=m->config.route.scale;
        if(!in.motion.pose.valid || (uint32_t)(now-in.motion.pose.stamp_ms)>m->config.geometry.pose_max_age_ms ||
           !auto_envelope_inside(m->config.geometry.stowed,in.motion.pose.pose,0,-s,.4f*s,0,m->config.footprint_margin)) {
            fail(m,AUTO_MISSION_START); return m->out;
        }
        if(!begin_segment(m,now)) { fail(m,AUTO_MISSION_CONFIG); return m->out; }
        m->out.phase=AUTO_NAVIGATE; m->phase_ms=m->segment_ms=now;
    }
    if(!in.motion.healthy) { fail(m,AUTO_MISSION_HEALTH); return m->out; }
    if(!in.localization_ok || !in.motion.pose.valid || !isfinite(in.motion.pose.pose.x)||
       !isfinite(in.motion.pose.pose.y)||!isfinite(in.motion.pose.pose.yaw)||
       !isfinite(in.motion.measured.vx)||!isfinite(in.motion.measured.vy)||!isfinite(in.motion.measured.wz)||
       (uint32_t)(now-in.motion.pose.stamp_ms)>m->config.geometry.pose_max_age_ms) {
        fail(m,AUTO_MISSION_POSE); return m->out;
    }
    if(m->out.segment_index>=AUTO_ROUTE_COUNT || m->completed_segments!=((1u<<m->out.segment_index)-1u)) {
        fail(m,AUTO_MISSION_COVERAGE); return m->out;
    }
    m->out.mandatory_remaining_s=remaining(m);
    float available=(m->config.deadline_ms-in.penalty_ms)*.001f-m->out.elapsed_s;
    m->out.budget_insufficient=m->out.mandatory_remaining_s>available;
    if(m->out.phase==AUTO_WORK_HOLD || m->out.phase==AUTO_RETRACT_VERIFY) {
        in.work.now_ms=now; in.work.healthy=in.motion.healthy; in.work.stationary=stationary(in);
        m->out.work=auto_work_step(&m->work,in.work);
        if(m->out.work.phase==AUTO_WORK_FAULT) {
            m->out.work_fault=m->out.work.error; fail(m,AUTO_MISSION_WORK); return m->out;
        }
        if(m->out.work.phase==AUTO_WORK_RETRACT) m->out.phase=AUTO_RETRACT_VERIFY;
        if(m->out.work.phase==AUTO_WORK_DONE) {
            if(!in.tools_stowed) { fail(m,AUTO_MISSION_WORK); return m->out; }
            m->out.actual_actions+=m->out.work.action_seen?1u:0u;
            m->out.handled_targets |= 1u<<m->target.target_id;
            if(!begin_segment(m,now)) { fail(m,AUTO_MISSION_CONFIG); return m->out; }
            m->out.phase=AUTO_NAVIGATE; /* resume original segment, no reset of segment deadline */
        }
        return m->out;
    }
    if(!in.tools_stowed) { fail(m,AUTO_MISSION_WORK); return m->out; }
    if(m->out.phase==AUTO_PAUSE) {
        if((uint32_t)(now-m->phase_ms)>=m->config.pause_timeout_ms) { fail(m,AUTO_MISSION_PAUSE_TIMEOUT); return m->out; }
        if(in.observation_ok && stationary(in)) {
            if(!m->pause_good) { m->pause_good=true; m->pause_good_ms=now; }
            else if((uint32_t)(now-m->pause_good_ms)>=m->config.pause_stable_ms) {
                if(!begin_segment(m,now)) { fail(m,AUTO_MISSION_CONFIG); return m->out; }
                m->out.phase=AUTO_NAVIGATE;
            }
        } else m->pause_good=false;
        return m->out;
    }
    if(!in.observation_ok) {
        if(++m->pause_count>m->config.max_pauses) { fail(m,AUTO_MISSION_PAUSE_LIMIT); return m->out; }
        auto_motion_cancel(&m->motion); m->out.phase=AUTO_PAUSE; m->phase_ms=now; m->pause_good=false; return m->out;
    }
    if((uint32_t)(now-m->segment_ms)>=m->config.segment_timeout_ms) { fail(m,AUTO_MISSION_SEGMENT_TIMEOUT); return m->out; }
    AutoRouteSegment segment=m->config.route.segment[m->out.segment_index];
    if(m->out.phase==AUTO_NAVIGATE && in.target.present) {
        AutoTarget t=in.target;
        float work_budget=m->config.stop_work_ms*.001f+m->config.work.total_ms*.001f;
        bool valid=segment.kind==AUTO_ROUTE_SURVEY&&segment.row&&t.row==segment.row&&t.target_id<32&&
            t.task>m->work.last_task&&t.task&&t.wire_region<=4&&t.classification_ok&&t.reachable&&
            !(m->out.handled_targets&(1u<<t.target_id))&&m->out.mandatory_remaining_s+work_budget<=available;
        if(valid) { m->target=t; m->out.phase=AUTO_STOP_WORK; m->phase_ms=now; }
        else ++m->out.rejected_targets;
    }
    if(m->out.phase==AUTO_STOP_WORK) {
        if((uint32_t)(now-m->phase_ms)>=m->config.stop_work_ms) { fail(m,AUTO_MISSION_WORK); return m->out; }
        in.motion.stop_requested=true;
    }
    AutoMotionOutput motion=auto_motion_step(&m->motion,in.motion);
    m->out.command=motion.command; m->out.steering_hint=motion.steering_hint;
    if(motion.phase==AUTO_MOTION_FAULT) { m->out.motion_fault=motion.error; fail(m,AUTO_MISSION_MOTION); return m->out; }
    if(m->out.phase==AUTO_STOP_WORK && stationary(in) && hypotf(motion.command.vx,motion.command.vy)<.00001f && fabsf(motion.command.wz)<.00001f) {
        if(!auto_work_begin(&m->work,m->config.work,m->target.task,m->target.wire_region,in.work.peer_epoch,now)) {
            fail(m,AUTO_MISSION_WORK); return m->out;
        }
        m->out.work=m->work.out; m->out.phase=AUTO_WORK_HOLD; auto_motion_cancel(&m->motion); return m->out;
    }
    if(m->out.phase==AUTO_NAVIGATE && motion.phase==AUTO_MOTION_DONE) {
        if(segment.aisle) {
            uint8_t bit=(uint8_t)(1u<<(segment.aisle-1));
            uint8_t expected=(uint8_t)(0x7fu & ~((1u<<segment.aisle)-1u));
            if(m->out.coverage!=expected) { fail(m,AUTO_MISSION_COVERAGE); return m->out; }
            m->out.coverage|=bit;
        }
        m->completed_segments|=1u<<m->out.segment_index;
        if(m->out.segment_index==AUTO_ROUTE_COUNT-1) {
            float s=m->config.route.scale;
            if(m->out.coverage!=0x7f) fail(m,AUTO_MISSION_COVERAGE);
            else if(!stationary(in)||!auto_envelope_inside(m->config.geometry.stowed,in.motion.pose.pose,0,-s,.4f*s,0,m->config.footprint_margin)) fail(m,AUTO_MISSION_PARKING);
            else { m->out.phase=AUTO_FINISHED; m->out.qualified=m->out.actual_actions>0; m->out.command=m->out.steering_hint=(AutoTwist){0}; }
        } else {
            ++m->out.segment_index; m->segment_ms=now;
            if(!begin_segment(m,now)) fail(m,AUTO_MISSION_CONFIG);
        }
    }
    return m->out;
}
const char* auto_mission_phase_name(AutoMissionPhase p) {
    static const char* names[]={"PRECHECK","WAIT_START","NAVIGATE","STOP_WORK","WORK_HOLD","RETRACT_VERIFY","PAUSE","FINISHED","FAULT"};
    return (unsigned)p<sizeof(names)/sizeof(names[0])?names[p]:"UNKNOWN";
}
const char* auto_mission_error_name(AutoMissionError e) {
    static const char* names[]={"OK","CONFIG","START","HEALTH","DEADLINE","POSE","MOTION","WORK","PAUSE_TIMEOUT","PAUSE_LIMIT","COVERAGE","PARKING","SEGMENT_TIMEOUT","TIMING"};
    return (unsigned)e<sizeof(names)/sizeof(names[0])?names[e]:"UNKNOWN";
}
