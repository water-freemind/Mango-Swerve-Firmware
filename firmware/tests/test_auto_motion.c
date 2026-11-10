#include "auto_motion.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static AutoMotionConfig config(void) {
    return (AutoMotionConfig){.vmax=.2f,.accel=.2f,.decel=.3f,.lateral_max=.03f,.cross_kp=.8f,
        .yaw_kp=1,.wz_max=.1f,.angular_accel=.3f,.position_tolerance=.01f,.cross_tolerance=.005f,
        .yaw_tolerance=.01f,.cross_abort=.15f,.yaw_abort=.25f,.settle_s=.15f,.pose_max_age_ms=50};
}
static AutoMotionInput input(AutoPose p,uint32_t now) {
    return (AutoMotionInput){.pose={p,now,true},.healthy=true,.aligned=true,.stationary=true,.now_ms=now};
}
static void simulate(float dx,float dy) {
    AutoMotion m; AutoMotionConfig c=config();
    AutoSegment s={{.2f,.2f},{.2f+dx,.2f+dy},1.5707963268f};
    assert(auto_motion_begin(&m,&c,s,0));
    AutoPose actual={s.start.x,s.start.y,s.heading+.025f};
    AutoTwist measured={0}; AutoMotionOutput out={0};
    /* Lagged actuator (40ms), +/-0.5mm pose noise, 0.003rad yaw error.
       Positions are integrated from lagged velocity, never assigned to target. */
    unsigned tick;
    for(tick=1;tick<3000;++tick) {
        AutoMotionInput in=input(actual,tick*10);
        in.pose.pose.x+=.0005f*sinf(tick*.1f); in.pose.pose.y+=.0005f*cosf(tick*.1f);
        in.pose.pose.yaw+=.003f*sinf(tick*.03f);
        in.measured=measured;
        in.stationary=hypotf(measured.vx,measured.vy)<.005f&&fabsf(measured.wz)<.01f;
        in.aligned=tick>20;
        out=auto_motion_step(&m,in);
        assert(out.phase!=AUTO_MOTION_FAULT);
        if(tick<=20) assert(out.command.vx==0 && out.command.vy==0);
        measured.vx+=(out.command.vx-measured.vx)*.25f;
        measured.vy+=(out.command.vy-measured.vy)*.25f;
        measured.wz+=(out.command.wz-measured.wz)*.25f;
        AutoVec2 world=auto_rotate((AutoVec2){measured.vx,measured.vy},actual.yaw);
        actual.x+=world.x*.01f; actual.y+=world.y*.01f; actual.yaw+=measured.wz*.01f;
        if(out.phase==AUTO_MOTION_DONE) break;
    }
    assert(tick<3000);
    assert(hypotf(actual.x-s.end.x,actual.y-s.end.y)<.013f);
    assert(fabsf(auto_wrap_angle(actual.yaw-s.heading))<.014f);
    printf("MOTION_SIM dx=%.1f dy=%.1f time_s=%.2f error_m=%.5f\n",(double)dx,(double)dy,tick*.01,
        (double)hypotf(actual.x-s.end.x,actual.y-s.end.y));
}
int main(void) {
    simulate(1,0); simulate(-1,0); simulate(0,1); simulate(0,-1);
    AutoMotion m; AutoMotionConfig c=config(); AutoSegment s={{0,0},{1,0},0};
    assert(auto_motion_begin(&m,&c,s,0));
    AutoMotionInput in=input((AutoPose){0},10); in.stationary=false; in.measured.vx=.2f;
    assert(auto_motion_step(&m,in).command.vx==0); /* a new segment cannot inherit motion */
    in=input((AutoPose){0},20); assert(auto_motion_step(&m,in).command.vx>0);
    for(unsigned t=30;t<=1000;t+=10) { in=input((AutoPose){0},t); auto_motion_step(&m,in); }
    in=input((AutoPose){0},1010); in.hard_stop=true;
    assert(auto_motion_step(&m,in).phase==AUTO_MOTION_FAULT); assert(m.out.command.vx==0);
    in.hard_stop=false; in.now_ms=1020; assert(auto_motion_step(&m,in).phase==AUTO_MOTION_FAULT);
    auto_motion_cancel(&m); assert(m.out.phase==AUTO_MOTION_IDLE && m.out.command.vx==0);
    assert(auto_motion_begin(&m,&c,s,0)); in=input((AutoPose){0},51);
    assert(auto_motion_step(&m,in).error==AUTO_MOTION_BAD_INPUT);
    assert(auto_motion_begin(&m,&c,s,0)); in=input((AutoPose){0},10); in.pose.stamp_ms=100;
    assert(auto_motion_step(&m,in).error==AUTO_MOTION_STALE_POSE);
    assert(auto_motion_begin(&m,&c,s,0)); in=input((AutoPose){0,.2f,0},10);
    assert(auto_motion_step(&m,in).error==AUTO_MOTION_DEVIATION);
    assert(auto_motion_begin(&m,&c,s,0)); in=input((AutoPose){0,0,.3f},10);
    assert(auto_motion_step(&m,in).error==AUTO_MOTION_DEVIATION);
    assert(auto_motion_begin(&m,&c,s,0)); in=input((AutoPose){1.02f,0,0},10);
    assert(auto_motion_step(&m,in).error==AUTO_MOTION_OVERSHOOT);
    assert(auto_motion_begin(&m,&c,s,0)); in=input((AutoPose){NAN,0,0},10);
    assert(auto_motion_step(&m,in).error==AUTO_MOTION_BAD_INPUT);
    assert(auto_motion_begin(&m,&c,s,0));
    for(unsigned t=10;t<5000;t+=10) { in=input((AutoPose){0},t); in.aligned=false; auto_motion_step(&m,in); }
    in=input((AutoPose){0},5000); auto_motion_step(&m,in); assert(m.out.command.vx<=.00201f);
    for(unsigned t=5010;t<=6000;t+=10) { in=input((AutoPose){.3f,0,0},t); auto_motion_step(&m,in); }
    float previous=m.out.command.vx;
    in=input((AutoPose){.3f,0,0},6010); in.stop_requested=true; auto_motion_step(&m,in);
    assert(m.out.phase==AUTO_MOTION_HOLD && m.out.command.vx<previous && m.out.command.vx>0);
    for(unsigned t=6020;t<=8000;t+=10) { in=input((AutoPose){.3f,0,0},t); in.stop_requested=true; auto_motion_step(&m,in); }
    assert(m.out.command.vx==0 && m.out.phase==AUTO_MOTION_HOLD);
    /* Slow accumulated yaw bias must not evade the absolute heading guard. */
    assert(auto_motion_begin(&m,&c,s,0));
    for(unsigned tick=1;tick<=300;++tick) {
        in=input((AutoPose){0,0,tick*.001f},tick*10);
        auto_motion_step(&m,in);
        if(m.out.phase==AUTO_MOTION_FAULT) break;
    }
    assert(m.out.error==AUTO_MOTION_DEVIATION && m.out.command.wz==0);
    /* Reverse an endpoint lateral correction while feedback still moves in
     * the old direction: demand must reach zero and wait for measured stop. */
    assert(auto_motion_begin(&m,&c,s,0));
    for(unsigned t=10;t<=500;t+=10) { in=input((AutoPose){1,.03f,0},t);auto_motion_step(&m,in); }
    assert(m.out.command.vy<0);
    for(unsigned t=510;t<=1000;t+=10) {
        in=input((AutoPose){1,-.03f,0},t);in.measured.vy=-.02f;in.stationary=false;
        auto_motion_step(&m,in);assert(m.out.command.vy<=0);
    }
    assert(m.reversing && m.out.command.vy==0);
    in=input((AutoPose){1,-.03f,0},1010);auto_motion_step(&m,in);
    assert(!m.reversing && m.out.command.vy==0);
    in=input((AutoPose){1,-.03f,0},1020);auto_motion_step(&m,in);assert(m.out.command.vy>0);
    /* Relax only endpoint acceptance, keeping in-flight heading correction. */
    c.finish_yaw_tolerance=.052359878f;
    assert(auto_motion_begin(&m,&c,s,0));
    in=input((AutoPose){.5f,0,.04f},10);auto_motion_step(&m,in);
    assert(m.out.command.wz<0 && m.out.phase!=AUTO_MOTION_DONE);
    assert(auto_motion_begin(&m,&c,s,0));
    for(unsigned t=10;t<=300;t+=10) {
        in=input((AutoPose){1,0,.04f},t);auto_motion_step(&m,in);
    }
    assert(m.out.phase==AUTO_MOTION_DONE && m.out.command.wz==0);
    assert(auto_motion_begin(&m,&c,s,0));
    for(unsigned t=10;t<=300;t+=10) {
        in=input((AutoPose){1,0,.06f},t);auto_motion_step(&m,in);
    }
    assert(m.out.phase!=AUTO_MOTION_DONE && m.out.command.wz<0);
    auto_motion_cancel(&m);assert(m.out.command.vx==0&&m.out.command.vy==0&&m.out.command.wz==0);
    puts("AUTO_MOTION_TEST_OK (four noisy lagged runs, alignment, safety, restart, stale/NaN/deviation, normal hold)");
}
