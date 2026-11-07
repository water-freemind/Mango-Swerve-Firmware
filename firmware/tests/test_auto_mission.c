#include "auto_fixture.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
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
static void normal_run(bool do_work) {
    AutoMissionConfig c=auto_fixture();AutoMission m;assert(auto_mission_init(&m,&c,0));
    AutoPose p={.2f,-.5f,1.5707963268f};AutoTwist vel={0};
    uint32_t offered=0;unsigned last_segment=0,align_ticks=0;unsigned now;
    for(now=10;now<=300000;now+=10) {
        AutoMissionInput in=auto_fixture_input(p,now);
        in.motion.pose.pose.x+=.0003f*sinf(now*.001f);in.motion.pose.pose.y+=.0003f*cosf(now*.001f);
        in.motion.pose.pose.yaw+=.002f*sinf(now*.0007f);
        in.motion.measured=vel;in.motion.stationary=hypotf(vel.vx,vel.vy)<.004f&&fabsf(vel.wz)<.01f;
        if(m.out.segment_index!=last_segment) { last_segment=m.out.segment_index;align_ticks=0; }
        in.motion.aligned=++align_ticks>15;
        if(now==10) in.start_event=AUTO_START_LOCAL_APPROVED;
        AutoRouteSegment s=c.route.segment[m.out.segment_index];
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
            if(w==AUTO_WORK_RETRACT && now-m.work.state_ms>=100) {
                in.work.retracted=(AutoWorkEvidence){m.work.task,9,now,true};in.tools_stowed=true;
            }
        }
        AutoMissionOutput out=auto_mission_step(&m,in);
        if(out.phase==AUTO_FAULT) printf("FAULT time=%u segment=%u reason=%s motion=%d work=%d\n",now,out.segment_index,auto_mission_error_name(out.first_fault),out.motion_fault,out.work_fault);
        assert(out.phase!=AUTO_FAULT);
        vel.vx+=(out.command.vx-vel.vx)*.25f;vel.vy+=(out.command.vy-vel.vy)*.25f;vel.wz+=(out.command.wz-vel.wz)*.25f;
        AutoVec2 world=auto_rotate((AutoVec2){vel.vx,vel.vy},p.yaw);p.x+=world.x*.01f;p.y+=world.y*.01f;p.yaw+=vel.wz*.01f;
        if(out.segment_index<=2) assert(out.coverage==0); /* outbound isn't aisle survey */
        if(out.phase==AUTO_FINISHED) break;
    }
    assert(m.out.phase==AUTO_FINISHED&&m.out.coverage==0x7f);
    assert(m.out.actual_actions==(do_work?12u:0u));assert(m.out.qualified==do_work);
    assert(auto_envelope_inside(c.geometry.stowed,p,0,-1,.4f,0,.01f));
    printf("MISSION_SIM work=%u total_s=%.2f coverage=0x%02x actions=%u qualified=%u\n",do_work,now*.001,m.out.coverage,m.out.actual_actions,m.out.qualified);
    AutoMissionInput repeat=auto_fixture_input(p,now+10);repeat.start_event=AUTO_START_LOCAL_APPROVED;
    assert(auto_mission_step(&m,repeat).phase==AUTO_FINISHED&&m.out.command.vx==0);
}
int main(void) {
    route_tests();normal_run(false);normal_run(true);
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
    puts("AUTO_MISSION_TEST_OK (full route, 12 work transactions, empty-run qualification, pauses, start/deadline, coverage and safety)");
}
