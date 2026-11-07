#include "auto_fixture.h"
#include "auto_observation.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static const char* trace_dir;
typedef enum { NOMINAL_EMPTY, NOMINAL_WORK, RANGE_LOSS, YAW_JUMP, POSE_SLIP,
    PEER_REBOOT, NO_RETRACT, WORK_DEADLINE, LOW_BUDGET, CASE_COUNT } Scenario;
static const char* case_name[]={"mission-empty","mission-work","range-loss","yaw-jump","pose-slip",
    "peer-reboot","no-retract","work-deadline","low-budget"};
static const AutoMissionError expected_fault[]={AUTO_MISSION_OK,AUTO_MISSION_OK,AUTO_MISSION_PAUSE_TIMEOUT,
    AUTO_MISSION_POSE,AUTO_MISSION_POSE,AUTO_MISSION_WORK,AUTO_MISSION_WORK,AUTO_MISSION_DEADLINE,AUTO_MISSION_OK};
static FILE* matrix;
static AutoMission started(void) {
    AutoMission m;AutoMissionConfig c=auto_fixture();assert(auto_mission_init(&m,&c,0));
    AutoMissionInput in=auto_fixture_input((AutoPose){.2f,-.5f,1.5707963268f},10);
    in.start_event=AUTO_START_LOCAL_APPROVED;auto_mission_step(&m,in);
    assert(m.out.phase==AUTO_NAVIGATE);return m;
}
static void route_tests(void) {
    AutoMissionConfig c=auto_fixture();assert(auto_route_valid(&c.route));
    assert(auto_route_clear(&c.route,c.geometry.stowed,.01f));
    unsigned k=7;float distance=0;
    for(unsigned i=0;i<17;++i) {
        AutoRouteSegment s=c.route.segment[i];distance+=hypotf(s.line.end.x-s.line.start.x,s.line.end.y-s.line.start.y);
        if(s.kind==AUTO_ROUTE_SURVEY) assert(s.aisle==k--);
    }
    assert(k==0&&fabsf(distance-30.6f)<.0001f);
    c.route.segment[2].aisle=6;assert(!auto_route_valid(&c.route));
    c=auto_fixture();c.route.segment[4]=c.route.segment[2];assert(!auto_route_valid(&c.route));
    c=auto_fixture();auto_route_build(&c.route,.95f,.7853981634f);
    assert(!auto_route_clear(&c.route,c.geometry.stowed,0));
    c=auto_fixture();AutoPlane p;float range;AutoLaserMount mount={{0,0},0,0};
    assert(!auto_route_ray(&c.route,(AutoPose){.2f,.1f,-1.5707963268f},mount,7.5f,&p,&range)); /* gate opening */
    assert(auto_route_ray(&c.route,(AutoPose){.2f,.5f,0},mount,7.5f,&p,&range));assert(fabsf(range-.2f)<.00001f);
    assert(auto_route_ray(&c.route,(AutoPose){.2f,2.6f,0},mount,7.5f,&p,&range));assert(fabsf(range-.4f)<.00001f); /* short R4 */
}
static void scenario_run(Scenario scenario) {
    bool do_work=scenario==NOMINAL_WORK||scenario>=PEER_REBOOT;
    AutoMissionConfig c=auto_fixture();AutoMission m;assert(auto_mission_init(&m,&c,0));
    if(scenario==LOW_BUDGET) m.config.reserve_s=299;
    AutoObservation monitor={0};FILE* trace=NULL;char path[1024];
    if(trace_dir) {
        int length=snprintf(path,sizeof(path),"%s/%s.csv",trace_dir,case_name[scenario]);
        assert(length>0 && length<(int)sizeof(path));trace=fopen(path,"w");assert(trace);
        fprintf(trace,"t_ms,state,segment,kind,aisle,coverage,actions,qualified,true_x,true_y,pose_x,pose_y,yaw,cmd_vx,cmd_vy,cmd_wz,measured_vx,measured_vy,measured_wz,progress,cross_error,range_valid,opening,stop_mask,first_fault\n");
    }
    AutoPose p={.2f,-.5f,1.5707963268f};AutoTwist vel={0};
    uint32_t offered=0;unsigned last_segment=0,align_ticks=0;unsigned now;
    bool budget_seen=false;
    for(now=10;now<=300000;now+=10) {
        AutoMissionInput in=auto_fixture_input(p,now);
        in.motion.pose.pose.x+=.0003f*sinf(now*.001f);in.motion.pose.pose.y+=.0003f*cosf(now*.001f);
        in.motion.pose.pose.yaw+=.002f*sinf(now*.0007f);
        in.motion.measured=vel;in.motion.stationary=hypotf(vel.vx,vel.vy)<.004f&&fabsf(vel.wz)<.01f;
        if(m.out.segment_index!=last_segment) { last_segment=m.out.segment_index;align_ticks=0; }
        in.motion.aligned=++align_ticks>15;
        if(now==10) in.start_event=AUTO_START_LOCAL_APPROVED;
        unsigned executed_segment=m.out.segment_index;
        AutoRouteSegment s=c.route.segment[executed_segment];
        if(scenario==YAW_JUMP && now>=20000) in.motion.pose.pose.yaw+=.1f;
        if(scenario==POSE_SLIP && now>=20000) in.motion.pose.pose.x+=.08f;
        AutoObservationInput obs={.pose=in.motion.pose,.measured=vel,.commanded=m.out.command,.now_ms=now};
        AutoVec2 tangent=auto_rotate((AutoVec2){s.line.end.x-s.line.start.x,s.line.end.y-s.line.start.y},-p.yaw);
        for(unsigned ch=0;ch<4;++ch) {
            AutoPlane actual_plane,predicted_plane;float actual_range,predicted_range;
            bool hit=auto_route_ray(&c.route,p,c.geometry.laser[ch],7.5f,&actual_plane,&actual_range);
            bool expected=auto_route_ray(&c.route,in.motion.pose.pose,c.geometry.laser[ch],7.5f,&predicted_plane,&predicted_range);
            if(hit) obs.ranges[ch]=(AutoRange){(uint16_t)lroundf(actual_range*1000),now,0,true,true};
            float approach=tangent.x*cosf(c.geometry.laser[ch].yaw)+tangent.y*sinf(c.geometry.laser[ch].yaw);
            if(approach>.1f && expected) obs.required_mask|=(uint8_t)(1u<<ch);
        }
        if(scenario==RANGE_LOSS && now>=20000)
            for(unsigned ch=0;ch<4;++ch) if(obs.required_mask&(1u<<ch)) obs.ranges[ch].valid=false;
        AutoObservationOutput observed=auto_observation_step(&monitor,&c.geometry,&c.route,c.geometry.stowed,obs);
        in.localization_ok=observed.pose_consistent;in.observation_ok=observed.required_ready;
        in.motion.hard_stop=observed.protective_stop;
        /* Known gate-facing openings use the segment cap in auto_fixture, also
         * counted by mission's budget. Map footprint/pose only, not real safety. */
        if(do_work && s.row && m.out.phase==AUTO_NAVIGATE) {
            float length=hypotf(s.line.end.x-s.line.start.x,s.line.end.y-s.line.start.y);
            for(unsigned n=0;n<2;++n) {
                unsigned id=(6-s.row)*2+n;
                if(!(offered&(1u<<id)) && m.motion.out.progress>length*(n+1)/3) {
                    in.target=(AutoTarget){true,true,true,(uint8_t)id,s.row,2,id+1};
                    offered|=1u<<id; break;
                }
            }
        }
        if(m.out.phase==AUTO_WORK_HOLD || m.out.phase==AUTO_RETRACT_VERIFY) {
            AutoWorkPhase w=m.work.out.phase;
            uint8_t state=w==AUTO_WORK_STOPPING?WORK_WAIT:
                w==AUTO_WORK_READY?WORK_WORKING:
                w==AUTO_WORK_ACTIVE && now-m.work.state_ms<500?WORK_WORKING:WORK_GO;
            in.work.packet_present=true;in.work.packet_stamp_ms=now;
            in.work.packet=(WorkCanPacket){m.work.task,state,2,0};
            in.tools_stowed=w==AUTO_WORK_STOPPING||w==AUTO_WORK_READY;
            if(w==AUTO_WORK_ACTIVE) in.work.action=(AutoWorkEvidence){m.work.task,9,now,true};
            if(w==AUTO_WORK_RETRACT && now-m.work.state_ms>=100 && scenario!=NO_RETRACT) {
                in.work.retracted=(AutoWorkEvidence){m.work.task,9,now,true};in.tools_stowed=true;
            }
            if(scenario==PEER_REBOOT) in.work.peer_epoch=10;
            if(scenario==WORK_DEADLINE) in.penalty_ms=299999;
        }
        AutoMissionOutput out=auto_mission_step(&m,in);
        budget_seen=budget_seen||out.budget_insufficient;
        if(trace) fprintf(trace,"%u,%s,%u,%u,%u,%u,%u,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%u,%u,%s\n",
            now,auto_mission_phase_name(out.phase),executed_segment,(unsigned)s.kind,s.aisle,out.coverage,out.actual_actions,out.qualified,
            (double)p.x,(double)p.y,(double)in.motion.pose.pose.x,(double)in.motion.pose.pose.y,(double)p.yaw,
            (double)out.command.vx,(double)out.command.vy,(double)out.command.wz,(double)vel.vx,(double)vel.vy,(double)vel.wz,
            (double)m.motion.out.progress,(double)m.motion.out.cross_error,observed.valid_mask,observed.opening_mask,observed.stop_mask,auto_mission_error_name(out.first_fault));
        if(out.phase==AUTO_FAULT && expected_fault[scenario]!=AUTO_MISSION_OK) break;
        if(out.phase==AUTO_FAULT) printf("FAULT time=%u segment=%u reason=%s motion=%d work=%d\n",now,out.segment_index,auto_mission_error_name(out.first_fault),out.motion_fault,out.work_fault);
        assert(out.phase!=AUTO_FAULT);
        vel.vx+=(out.command.vx-vel.vx)*.25f;vel.vy+=(out.command.vy-vel.vy)*.25f;vel.wz+=(out.command.wz-vel.wz)*.25f;
        AutoVec2 world=auto_rotate((AutoVec2){vel.vx,vel.vy},p.yaw);p.x+=world.x*.01f;p.y+=world.y*.01f;p.yaw+=vel.wz*.01f;
        if(out.segment_index<=2) assert(out.coverage==0); /* outbound isn't aisle survey */
        if(out.phase==AUTO_FINISHED) break;
    }
    if(trace) { assert(ferror(trace)==0);assert(fclose(trace)==0); }
    assert(m.out.first_fault==expected_fault[scenario]);
    if(matrix) fprintf(matrix,"%s,%s,%s,%u,%u,%u,%u,%u\n",case_name[scenario],auto_mission_phase_name(m.out.phase),
        auto_mission_error_name(m.out.first_fault),now,m.out.coverage,m.out.actual_actions,m.out.qualified,m.out.work_fault);
    if(expected_fault[scenario]!=AUTO_MISSION_OK) {
        assert(m.out.phase==AUTO_FAULT && !m.out.qualified);
        assert(m.out.command.vx==0&&m.out.command.vy==0&&m.out.command.wz==0);
        if(scenario==PEER_REBOOT) assert(m.out.work_fault==AUTO_WORK_RESTART);
        if(scenario==NO_RETRACT) assert(m.out.work_fault==AUTO_WORK_RETRACT_TIMEOUT);
        AutoMissionInput resume=auto_fixture_input(p,now+10);
        assert(auto_mission_step(&m,resume).first_fault==expected_fault[scenario]);
        printf("MISSION_FAULT case=%s time_ms=%u first=%s work=%u locked=1\n",case_name[scenario],now,auto_mission_error_name(m.out.first_fault),m.out.work_fault);
        return;
    }
    assert(m.out.phase==AUTO_FINISHED&&m.out.coverage==0x7f);
    assert(m.out.actual_actions==(scenario==NOMINAL_WORK?12u:0u));assert(m.out.qualified==(scenario==NOMINAL_WORK));
    if(scenario==LOW_BUDGET) assert(budget_seen&&m.out.rejected_targets==12);
    assert(auto_envelope_inside(c.geometry.stowed,p,0,-1,.4f,0,.01f));
    printf("MISSION_SIM case=%s total_s=%.2f coverage=0x%02x actions=%u qualified=%u\n",case_name[scenario],now*.001,m.out.coverage,m.out.actual_actions,m.out.qualified);
    AutoMissionInput repeat=auto_fixture_input(p,now+10);repeat.start_event=AUTO_START_LOCAL_APPROVED;
    assert(auto_mission_step(&m,repeat).phase==AUTO_FINISHED&&m.out.command.vx==0);
}
int main(int argc,char** argv) {
    if(argc==2) trace_dir=argv[1];
    if(trace_dir) {
        AutoMissionConfig fixture=auto_fixture();char path[1024];int n=snprintf(path,sizeof(path),"%s/fixture.json",trace_dir);
        assert(n>0&&n<(int)sizeof(path));FILE* meta=fopen(path,"w");assert(meta);
        fprintf(meta,"{\"simulation_only\":true,\"map_scale\":%.6f,\"vmax_mps\":%.6f,\"accel_mps2\":%.6f,\"decel_mps2\":%.6f,\"stop_latency_s\":%.6f,\"tick_ms\":10,\"velocity_response_fraction_per_tick\":0.25,\"pose_noise_m\":0.0003,\"yaw_noise_rad\":0.002,\"segment_vmax\":[",
            (double)fixture.route.scale,(double)fixture.motion.vmax,(double)fixture.motion.accel,(double)fixture.motion.decel,(double)fixture.motion.stop_latency_s);
        for(unsigned i=0;i<AUTO_ROUTE_COUNT;++i) fprintf(meta,"%s%.6f",i?",":"",(double)fixture.segment_vmax[i]);
        fputs("],\"work_model\":\"12 synthetic targets; WORKING 500ms; retract proof after 100ms; not measured hardware\",\"parameters_source\":\"firmware/tests/auto_fixture.h\"}\n",meta);
        assert(ferror(meta)==0);assert(fclose(meta)==0);
    }
    if(trace_dir) { char path[1024];int n=snprintf(path,sizeof(path),"%s/scenarios.csv",trace_dir);
        assert(n>0&&n<(int)sizeof(path));matrix=fopen(path,"w");assert(matrix);
        fputs("scenario,terminal,first_fault,t_ms,coverage,actions,qualified,work_fault\n",matrix); }
    route_tests();for(unsigned s=0;s<CASE_COUNT;++s) scenario_run((Scenario)s);
    if(matrix) { assert(ferror(matrix)==0);assert(fclose(matrix)==0); }
    AutoMissionConfig c=auto_fixture();AutoMission m;
    c.simulation_only=c.geometry.simulation_only=c.work.simulation_only=false;
    assert(!auto_mission_init(&m,&c,0)); /* unconfirmed hardware gate */
    c=auto_fixture();assert(auto_mission_init(&m,&c,0));
    AutoMissionInput in=auto_fixture_input((AutoPose){.2f,-.5f,1.5707963268f},10);
    in.start_event=AUTO_START_HOST_DEBUG;assert(auto_mission_step(&m,in).first_fault==AUTO_MISSION_START);
    c.deadline_ms=100;assert(auto_mission_init(&m,&c,0));
    in.start_event=AUTO_START_LOCAL_APPROVED;in.precheck_ready=false;auto_mission_step(&m,in);
    in.motion.now_ms=110;assert(auto_mission_step(&m,in).first_fault==AUTO_MISSION_DEADLINE); /* precheck counted */
    m=started();in=auto_fixture_input((AutoPose){.2f,-.5f,1.5707963268f},20);in.observation_ok=false;
    assert(auto_mission_step(&m,in).phase==AUTO_PAUSE);
    in.observation_ok=true;
    for(unsigned t=30;t<=130;t+=10) { in.motion.now_ms=in.motion.pose.stamp_ms=t;auto_mission_step(&m,in); }
    assert(m.out.phase==AUTO_NAVIGATE&&m.out.command.vx==0);
    m=started();in=auto_fixture_input((AutoPose){.2f,-.5f,1.5707963268f},20);in.observation_ok=false;
    for(unsigned t=20;t<=1520;t+=10) { in.motion.now_ms=in.motion.pose.stamp_ms=t;auto_mission_step(&m,in); }
    assert(m.out.first_fault==AUTO_MISSION_PAUSE_TIMEOUT);
    m=started();in=auto_fixture_input((AutoPose){.2f,-.5f,1.5707963268f},20);in.localization_ok=false;
    assert(auto_mission_step(&m,in).first_fault==AUTO_MISSION_POSE);
    m=started();in=auto_fixture_input((AutoPose){.2f,-.5f,1.5707963268f},20);m.out.segment_index=3;
    assert(auto_mission_step(&m,in).first_fault==AUTO_MISSION_COVERAGE);
    m=started();in=auto_fixture_input((AutoPose){.2f,-.5f,1.5707963268f},20);in.penalty_ms=299999;
    assert(auto_mission_step(&m,in).first_fault==AUTO_MISSION_DEADLINE);
    m=started();in=auto_fixture_input((AutoPose){.2f,-.5f,1.5707963268f},20);in.cancel=true;
    assert(auto_mission_step(&m,in).first_fault==AUTO_MISSION_HEALTH);in.cancel=false;
    assert(auto_mission_step(&m,in).phase==AUTO_FAULT);
    m=started();unsigned tick=20;
    for(unsigned n=0;n<3;++n) {
        in=auto_fixture_input((AutoPose){.2f,-.5f,1.5707963268f},tick);in.observation_ok=false;
        auto_mission_step(&m,in);tick+=10;
        if(n==2) { assert(m.out.first_fault==AUTO_MISSION_PAUSE_LIMIT);break; }
        for(unsigned j=0;j<12;++j,tick+=10) { in=auto_fixture_input((AutoPose){.2f,-.5f,1.5707963268f},tick);auto_mission_step(&m,in); }
        assert(m.out.phase==AUTO_NAVIGATE);
    }
    c=auto_fixture(); /* Center crossed gate, rear remains in field. */
    assert(!auto_envelope_inside(c.geometry.stowed,(AutoPose){.2f,-.05f,1.5707963268f},0,-1,.4f,0,.01f));
    m=started();m.out.segment_index=16;m.completed_segments=0xffff;m.out.coverage=0x7f;
    m.motion.out.phase=AUTO_MOTION_DONE; /* Even a DONE primitive cannot bypass full-body parking. */
    in=auto_fixture_input((AutoPose){.2f,-.05f,1.5707963268f},20);
    assert(auto_mission_step(&m,in).first_fault==AUTO_MISSION_PARKING);
    assert(!auto_route_pose_clear(&c.route,c.geometry.stowed,(AutoPose){1.5f,.3f,0},.01f)); /* body overlaps R1 */
    m=started();in=auto_fixture_input((AutoPose){1.5f,.3f,0},20);
    assert(auto_mission_step(&m,in).first_fault==AUTO_MISSION_ENVELOPE);
    c.segment_vmax[5]=NAN;assert(!auto_mission_init(&m,&c,0));
    puts("AUTO_MISSION_TEST_OK (full route, 12 work transactions, empty-run qualification, pauses, start/deadline, coverage and safety)");
}
