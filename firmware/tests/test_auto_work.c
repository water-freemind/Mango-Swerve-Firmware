#include "auto_work.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static AutoWorkConfig cfg(void) { return (AutoWorkConfig){1000,800,500,3000,100,100,true,false}; }
static AutoWorkInput msg(uint32_t t,uint8_t state) {
    return (AutoWorkInput){.now_ms=t,.peer_epoch=9,.packet_stamp_ms=t,.peer_online=true,
        .healthy=true,.stationary=true,.packet_present=true,.packet={1,state,2,0}};
}
static AutoWork ready(void) {
    AutoWork w={0}; assert(auto_work_begin(&w,cfg(),1,2,9,0));
    auto_work_step(&w,msg(0,WORK_WAIT)); auto_work_step(&w,msg(100,WORK_WAIT));
    assert(w.out.phase==AUTO_WORK_READY && w.out.reply.state==WORK_STOPPED); return w;
}
int main(void) {
    AutoWork w=ready(); AutoWorkInput in=msg(200,WORK_WORKING);
    in.action=(AutoWorkEvidence){1,9,190,true};
    auto_work_step(&w,in); assert(w.out.phase==AUTO_WORK_ACTIVE && !w.out.action_seen);
    auto_work_step(&w,in); assert(w.out.phase==AUTO_WORK_ACTIVE); /* duplicate is idempotent */
    in=msg(250,WORK_GO); in.retracted=(AutoWorkEvidence){1,9,249,true};
    auto_work_step(&w,in); assert(w.out.phase==AUTO_WORK_RETRACT && w.out.hold); /* old proof */
    in=msg(260,WORK_GO); in.retracted=(AutoWorkEvidence){2,9,260,true};
    auto_work_step(&w,in); assert(w.out.hold); /* wrong task */
    in=msg(270,WORK_GO); in.retracted=(AutoWorkEvidence){1,8,270,true};
    auto_work_step(&w,in); assert(w.out.hold); /* old boot session */
    in=msg(280,WORK_GO); in.retracted=(AutoWorkEvidence){1,9,280,true};
    in.action=(AutoWorkEvidence){1,9,210,true}; auto_work_step(&w,in);
    assert(w.out.phase==AUTO_WORK_DONE && !w.out.hold && w.out.action_seen);
    assert(w.out.reply.state==WORK_GO_ACK);
    assert(!auto_work_begin(&w,cfg(),1,2,9,300)); assert(auto_work_begin(&w,cfg(),2,2,9,300));
    in=msg(310,WORK_GO); auto_work_step(&w,in); assert(w.out.phase==AUTO_WORK_STOPPING && w.out.rejected==1);
    w=ready(); in=msg(110,WORK_GO); auto_work_step(&w,in); assert(w.out.rejected==1 && w.out.hold);
    in.packet.result=WORK_RESULT_SKIPPED; auto_work_step(&w,in); assert(w.out.phase==AUTO_WORK_RETRACT);
    in.now_ms=in.packet_stamp_ms=120; in.retracted=(AutoWorkEvidence){1,9,120,true};
    in.action=(AutoWorkEvidence){1,9,120,true}; /* skipped action cannot count */
    auto_work_step(&w,in); assert(w.out.phase==AUTO_WORK_DONE && !w.out.action_seen);
    w=ready();
    for(unsigned t=200;t<=900;t+=100) auto_work_step(&w,msg(t,WORK_WORKING));
    assert(w.out.error==AUTO_WORK_TIMEOUT); /* continuous valid heartbeat doesn't extend deadline */
    w=ready(); auto_work_step(&w,msg(200,WORK_WORKING)); auto_work_step(&w,msg(250,WORK_GO));
    for(unsigned t=300;t<=800;t+=50) auto_work_step(&w,msg(t,WORK_GO));
    assert(w.out.error==AUTO_WORK_RETRACT_TIMEOUT && w.out.hold);
    w=ready(); in=msg(200,WORK_WORKING); in.peer_epoch=10; auto_work_step(&w,in);
    assert(w.out.error==AUTO_WORK_RESTART);
    w=ready(); in=msg(200,WORK_WORKING); in.cancel=true; auto_work_step(&w,in);
    assert(w.out.error==AUTO_WORK_CANCELLED); auto_work_step(&w,msg(210,WORK_GO)); assert(w.out.hold);
    w=ready(); in=msg(200,WORK_WORKING); in.stationary=false; auto_work_step(&w,in);
    assert(w.out.error==AUTO_WORK_FEEDBACK);
    w=ready(); in=msg(600,WORK_WORKING); in.packet_stamp_ms=100; auto_work_step(&w,in);
    assert(w.out.error==AUTO_WORK_LINK);
    w=ready(); in=msg(200,WORK_FAULT); auto_work_step(&w,in); assert(w.out.error==AUTO_WORK_PEER_FAULT);
    w=(AutoWork){0}; AutoWorkConfig c=cfg(); c.simulation_only=false;
    assert(!auto_work_begin(&w,c,1,2,9,0)); /* real contract unconfirmed */
    assert(auto_work_begin(&w,cfg(),1,2,9,0));
    for(unsigned t=0;t<=1000;t+=100) { in=msg(t,WORK_WAIT); in.stationary=false; auto_work_step(&w,in); }
    assert(w.out.error==AUTO_WORK_STOP_TIMEOUT);
    w=(AutoWork){0}; c=cfg(); c.total_ms=300; assert(auto_work_begin(&w,c,1,2,9,0));
    auto_work_step(&w,msg(0,WORK_WAIT)); auto_work_step(&w,msg(100,WORK_WAIT));
    auto_work_step(&w,msg(200,WORK_WORKING)); auto_work_step(&w,msg(300,WORK_GO));
    assert(w.out.error==AUTO_WORK_TOTAL_TIMEOUT);
    uint8_t bytes[8]; WorkCanPacket packet={1,WORK_WAIT,2,0},decoded;
    work_can_encode(bytes,packet); assert(auto_work_decode(bytes,&decoded));
    assert(decoded.task==1 && decoded.region==2); bytes[6]=7; assert(!auto_work_decode(bytes,&decoded));
    puts("AUTO_WORK_TEST_OK (legacy packet adapter, idempotency, stale/wrong task/epoch, independent deadlines, retraction proof, cancellation)");
}
