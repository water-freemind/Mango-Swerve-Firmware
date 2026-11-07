#include "auto_work.h"
#include <string.h>

static void fail(AutoWork* w,AutoWorkError error) {
    if(w->out.phase==AUTO_WORK_FAULT) return;
    w->out.phase=AUTO_WORK_FAULT; w->out.error=error; w->out.hold=true;
    w->out.reply.state=WORK_FAULT;
    w->out.reply.result=error==AUTO_WORK_RESTART?WORK_RESULT_RESET:
        error==AUTO_WORK_LINK?WORK_RESULT_LINK:WORK_RESULT_PROTOCOL;
}
static bool config_valid(AutoWorkConfig c) {
    return c.stop_ms && c.work_ms && c.retract_ms && c.total_ms && c.stable_ms && c.evidence_age_ms &&
        c.stop_ms<60000 && c.work_ms<300000 && c.retract_ms<60000 && c.total_ms<300000 &&
        c.stable_ms<c.stop_ms && c.evidence_age_ms<=1000 && (c.simulation_only||c.contract_confirmed);
}
bool auto_work_begin(AutoWork* w,AutoWorkConfig c,uint32_t task,uint8_t region,uint32_t epoch,uint32_t now) {
    if(!w) return false;
    /* Caller zero-initializes once. A fault is latched until explicit reinit. */
    if(w->out.phase!=AUTO_WORK_IDLE && w->out.phase!=AUTO_WORK_DONE) return false;
    if(!config_valid(c)||!task||task<=w->last_task||region>4||!epoch) return false;
    uint32_t previous=w->last_task;
    memset(w,0,sizeof(*w)); w->last_task=previous; w->config=c; w->task=task; w->epoch=epoch;
    w->begin_ms=w->state_ms=w->peer_ms=now;
    w->out.phase=AUTO_WORK_STOPPING; w->out.hold=true;
    w->out.reply=(WorkCanPacket){task,WORK_STOPPING,region,WORK_RESULT_OK};
    return true;
}
bool auto_work_decode(const uint8_t bytes[8],WorkCanPacket* packet) {
    return bytes && packet && work_can_decode(bytes,packet) && packet->region<=4;
}
static bool evidence(AutoWork* w,AutoWorkEvidence e,uint32_t now) {
    return e.valid && e.task==w->task && e.epoch==w->epoch &&
        (uint32_t)(now-e.stamp_ms)<=w->config.evidence_age_ms;
}
AutoWorkOutput auto_work_step(AutoWork* w,AutoWorkInput in) {
    if(!w) return (AutoWorkOutput){.phase=AUTO_WORK_FAULT,.error=AUTO_WORK_CONFIG,.hold=true};
    if(w->out.phase==AUTO_WORK_IDLE || w->out.phase==AUTO_WORK_DONE || w->out.phase==AUTO_WORK_FAULT) return w->out;
    if(in.cancel) fail(w,AUTO_WORK_CANCELLED);
    else if(!in.peer_online) fail(w,AUTO_WORK_LINK);
    else if(in.peer_epoch!=w->epoch) fail(w,AUTO_WORK_RESTART);
    else if(!in.healthy || (w->out.phase!=AUTO_WORK_STOPPING && !in.stationary)) fail(w,AUTO_WORK_FEEDBACK);
    else if((uint32_t)(in.now_ms-w->begin_ms)>=w->config.total_ms) fail(w,AUTO_WORK_TOTAL_TIMEOUT);
    else if(w->out.phase==AUTO_WORK_STOPPING && (uint32_t)(in.now_ms-w->state_ms)>=w->config.stop_ms) fail(w,AUTO_WORK_STOP_TIMEOUT);
    else if((w->out.phase==AUTO_WORK_READY || w->out.phase==AUTO_WORK_ACTIVE) && (uint32_t)(in.now_ms-w->state_ms)>=w->config.work_ms) fail(w,AUTO_WORK_TIMEOUT);
    else if(w->out.phase==AUTO_WORK_RETRACT && (uint32_t)(in.now_ms-w->state_ms)>=w->config.retract_ms) fail(w,AUTO_WORK_RETRACT_TIMEOUT);
    if(w->out.phase==AUTO_WORK_FAULT) return w->out;
    if(in.packet_present) {
        WorkCanPacket p=in.packet;
        bool valid=(uint32_t)(in.now_ms-in.packet_stamp_ms)<WORK_CAN_TIMEOUT_MS && p.task==w->task &&
            p.region==w->out.reply.region && p.result<=WORK_RESULT_SKIPPED;
        if(p.state==WORK_HELLO && p.task==0 && (uint32_t)(in.now_ms-in.packet_stamp_ms)<WORK_CAN_TIMEOUT_MS) {
            fail(w,AUTO_WORK_RESTART); return w->out;
        }
        if(valid && p.state==WORK_FAULT) { fail(w,AUTO_WORK_PEER_FAULT); return w->out; }
        if(valid && p.state==WORK_WAIT && (w->out.phase==AUTO_WORK_STOPPING || w->out.phase==AUTO_WORK_READY)) {
            w->peer_ms=in.packet_stamp_ms;
        } else if(valid && p.state==WORK_WORKING && (w->out.phase==AUTO_WORK_READY || w->out.phase==AUTO_WORK_ACTIVE)) {
            /* Keep deadline set when STOPPED was first acknowledged. */
            if(!w->work_started) { w->work_started=true; w->action_start_ms=in.now_ms; }
            w->out.phase=AUTO_WORK_ACTIVE; w->peer_ms=in.packet_stamp_ms;
        } else if(valid && p.state==WORK_GO && (w->out.phase==AUTO_WORK_ACTIVE || w->out.phase==AUTO_WORK_RETRACT ||
                  (w->out.phase==AUTO_WORK_READY && p.result==WORK_RESULT_SKIPPED))) {
            if(w->out.phase!=AUTO_WORK_RETRACT) w->state_ms=in.now_ms;
            w->out.phase=AUTO_WORK_RETRACT; w->out.reply.result=p.result;
            w->peer_ms=in.packet_stamp_ms;
        } else ++w->out.rejected;
    }
    if((uint32_t)(in.now_ms-w->peer_ms)>=WORK_CAN_TIMEOUT_MS) { fail(w,AUTO_WORK_LINK); return w->out; }
    if(w->out.phase==AUTO_WORK_STOPPING) {
        if(!in.stationary) w->stopped_timing=false;
        else if(!w->stopped_timing) { w->stopped_timing=true; w->stopped_ms=in.now_ms; }
        else if((uint32_t)(in.now_ms-w->stopped_ms)>=w->config.stable_ms) {
            w->out.phase=AUTO_WORK_READY; w->out.reply.state=WORK_STOPPED; w->state_ms=in.now_ms;
        }
    }
    if(w->work_started && (w->out.phase==AUTO_WORK_ACTIVE || w->out.phase==AUTO_WORK_RETRACT) && evidence(w,in.action,in.now_ms) &&
       (uint32_t)(in.action.stamp_ms-w->action_start_ms)<0x80000000u)
        w->out.action_seen=true;
    if(w->out.phase==AUTO_WORK_RETRACT && evidence(w,in.retracted,in.now_ms) &&
       (uint32_t)(in.retracted.stamp_ms-w->state_ms)<0x80000000u) {
        w->out.phase=AUTO_WORK_DONE; w->out.reply.state=WORK_GO_ACK;
        w->out.hold=false; w->last_task=w->task;
    }
    return w->out;
}
