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
static void heading_reversal_while_translating(void) {
    AutoMotion m;AutoMotionConfig c=config();
    assert(auto_motion_begin(&m,&c,(AutoSegment){{0,0},{2,0},0},0));
    for(unsigned now=10;now<=1000;now+=10) {
        AutoMotionInput in=input((AutoPose){.2f,0,.03f},now);
        auto_motion_step(&m,in);
    }
    assert(m.out.command.wz<0);
    const float previous_speed=hypotf(m.out.command.vx,m.out.command.vy);
    AutoMotionInput in=input((AutoPose){.2f,0,-.03f},1010);
    in.stationary=false;in.measured=(AutoTwist){.vx=.2f,.wz=-.03f};
    auto_motion_step(&m,in);
    assert(!m.reversing);
    assert(hypotf(m.out.command.vx,m.out.command.vy)>=previous_speed-.00001f);
    assert(fabsf(m.out.command.wz+.03f)<=c.angular_accel*.01f+.00001f);
    puts("MOTION_HEADING_REVERSAL_OK translation_continuous angular_slew_bounded");
}
static void endpoint_speed_envelope(void) {
    AutoMotionConfig c=config();c.accel=10;float previous=0;
    for(unsigned sample=0;sample<3;++sample) {
        AutoMotion m;
        assert(auto_motion_begin(&m,&c,(AutoSegment){{0,0},{1,0},0},0));
        m.started=true;m.velocity_map=(AutoVec2){.1f,0};
        AutoMotionInput in=input((AutoPose){.90f,0,0},10);
        in.measured.vx=sample*.1f;in.stationary=false;
        m.config.stop_latency_s=.5f;
        auto_motion_step(&m,in);
        if(sample) assert(fabsf(m.out.command.vx-previous)<.000001f);
        previous=m.out.command.vx;
        assert(previous>0 && previous<=c.vmax);
    }
    puts("MOTION_ENDPOINT_ENVELOPE_OK feedback_noise_does_not_switch_target");
}
static void along_hold_centering_test(void) {
    AutoMotion m;const AutoMotionConfig c=config();
    assert(auto_motion_begin(&m,&c,(AutoSegment){{0,0},{1,0},0},0));
    for(unsigned now=10;now<=500;now+=10) {
        AutoMotionInput in=input((AutoPose){.5f,.03f,0},now);in.hold_along=true;
        auto_motion_step(&m,in);
        assert(m.out.command.vx==0 && m.out.command.vy<0);
        assert(m.out.phase!=AUTO_MOTION_DONE && m.out.phase!=AUTO_MOTION_FAULT);
    }
    AutoMotionInput in=input((AutoPose){.5f,.005f,0},510);
    auto_motion_step(&m,in);assert(m.out.command.vx>0 && m.out.command.vx<=.00201f);
    in.now_ms=520;in.hard_stop=true;auto_motion_step(&m,in);
    assert(m.out.phase==AUTO_MOTION_FAULT && m.out.command.vx==0 && m.out.command.vy==0);
    puts("MOTION_ALONG_HOLD_OK lateral_correction bounded_release hard_stop");
}
static void endpoint_centering_hold_test(void) {
    for(int sign=-1;sign<=1;sign+=2) {
        AutoMotion m;AutoMotionConfig c=config();
        c.cross_tolerance=.008f;c.finish_cross_tolerance=.02f;
        assert(auto_motion_begin(&m,&c,(AutoSegment){{0,0},{1,0},0},0));
        for(unsigned now=10;now<=500;now+=10) {
            AutoMotionInput in=input((AutoPose){1,sign*.0175f,0},now);
            in.hold_along=true;auto_motion_step(&m,in);
            assert(sign*m.out.command.vy<0 && m.out.command.vx==0);
            assert(m.out.phase!=AUTO_MOTION_DONE);
        }
        for(unsigned now=510;now<=1500;now+=10) {
            AutoMotionInput in=input((AutoPose){1,sign*.009f,0},now);
            auto_motion_step(&m,in);
        }
        assert(m.out.phase==AUTO_MOTION_DONE);
    }
    puts("MOTION_ENDPOINT_CENTERING_OK 17_5mm_both_directions correction_then_completion");
}
static void prepared_wheel_direction_test(void) {
    for(unsigned run=0;run<2;++run) {
        AutoMotion m;AutoMotionConfig c=config();
        c.vmax=run?.02f:.5f;c.accel=c.decel=c.angular_accel=.15f;
        assert(auto_motion_begin(&m,&c,(AutoSegment){{0,0},{2,0},0},0));
        AutoMotionInput in=input((AutoPose){0,0,.04f},10);in.aligned=false;
        const AutoMotionOutput prepare=auto_motion_step(&m,in);
        assert(prepare.command.vx==0 && prepare.command.wz==0);
        in.now_ms=20;in.pose.stamp_ms=20;in.aligned=true;
        const AutoMotionOutput launch=auto_motion_step(&m,in);
        const float position[4][2]={{.12f,.12f},{.12f,-.12f},{-.12f,-.12f},{-.12f,.12f}};
        for(unsigned wheel=0;wheel<4;++wheel) {
            const AutoTwist h=prepare.steering_hint,v=launch.command;
            const float x=position[wheel][0],y=position[wheel][1];
            const float prepared_angle=atan2f(h.vy+h.wz*x,h.vx-h.wz*y);
            const float launch_angle=atan2f(v.vy+v.wz*x,v.vx-v.wz*y);
            assert(fabsf(auto_wrap_angle(launch_angle-prepared_angle))<.0001f);
        }
        assert(hypotf(launch.command.vx,launch.command.vy)<=c.accel*.01f+.000001f);
        assert(fabsf(launch.command.wz)<=c.angular_accel*.01f+.000001f);
    }
    puts("MOTION_PREPARED_WHEEL_DIRECTION_OK four_modules fast_and_slow_launch shared_slew");
}
static void smooth_correction_boundary_test(void) {
    AutoMotion m;AutoMotionConfig c=config();c.smooth_correction=true;
    for(int sign=-1;sign<=1;sign+=2) {
        assert(auto_motion_begin(&m,&c,(AutoSegment){{0,0},{1,0},0},0));
        AutoMotionInput in=input((AutoPose){.5f,sign*(c.cross_tolerance+.0001f),
            sign*(c.yaw_tolerance+.0001f)},10);
        in.aligned=false;
        const AutoMotionOutput out=auto_motion_step(&m,in);
        assert(fabsf(out.steering_hint.wz)<.0002f);
        const AutoVec2 map_hint=auto_rotate((AutoVec2){out.steering_hint.vx,out.steering_hint.vy},in.pose.pose.yaw);
        assert(fabsf(map_hint.y)<.0002f);
    }
    puts("MOTION_SMOOTH_CORRECTION_OK no_deadband_boundary_step");
}
static void stopped_goal_alignment_test(void) {
    AutoMotion m;const AutoMotionConfig c=config();
    assert(auto_motion_begin(&m,&c,(AutoSegment){{0,0},{1,0},0},0));
    for(unsigned now=10;now<=300;now+=10) {
        AutoMotionInput in=input((AutoPose){1,0,0},now);in.aligned=false;
        const AutoMotionOutput out=auto_motion_step(&m,in);
        assert(out.command.vx==0 && out.command.vy==0 && out.command.wz==0);
    }
    assert(m.out.phase==AUTO_MOTION_DONE);
    puts("MOTION_STOPPED_GOAL_OK stationary_zero_target_completes_with_closed_gate");
}
static void stable_preparation_test(void) {
    AutoMotion m;AutoMotionConfig c=config();c.smooth_correction=true;c.vmax=.03f;
    assert(auto_motion_begin(&m,&c,(AutoSegment){{0,0},{1,0},0},0));
    AutoTwist first={0};
    for(unsigned now=10;now<=200;now+=10) {
        AutoMotionInput in=input((AutoPose){.5f,.02f+(now%20u?.001f:0),.04f+(now%20u?.005f:0)},now);
        in.aligned=false;
        const AutoMotionOutput out=auto_motion_step(&m,in);
        if(now==10u) first=out.steering_hint;
        assert(out.command.vx==0 && out.command.vy==0 && out.command.wz==0);
        assert(fabsf(out.steering_hint.vx-first.vx)<.000001f);
        assert(fabsf(out.steering_hint.vy-first.vy)<.000001f);
        assert(fabsf(out.steering_hint.wz-first.wz)<.000001f);
    }
    /* A changed correction direction needs a new preparation despite the old gate. */
    AutoMotionInput in=input((AutoPose){.5f,.02f,-.04f},210);
    const AutoMotionOutput changed=auto_motion_step(&m,in);
    assert(changed.phase==AUTO_MOTION_ALIGN && changed.command.wz==0);
    assert(changed.steering_hint.wz>0);
    in.now_ms=in.pose.stamp_ms=220;
    const AutoMotionOutput launch=auto_motion_step(&m,in);
    assert(launch.command.vx>0 && launch.command.wz>0);
    const float hint_angle=atan2f(changed.steering_hint.vy+.1f*changed.steering_hint.wz,
        changed.steering_hint.vx-.1f*changed.steering_hint.wz);
    const float launch_angle=atan2f(launch.command.vy+.1f*launch.command.wz,
        launch.command.vx-.1f*launch.command.wz);
    assert(fabsf(auto_wrap_angle(hint_angle-launch_angle))<.00001f);
    assert(auto_motion_begin(&m,&c,(AutoSegment){{0,0},{1,0},0},0));
    in=input((AutoPose){.5f,.02f,.04f},10);in.aligned=false;auto_motion_step(&m,in);
    m.config.vmax=.001f;
    in.now_ms=in.pose.stamp_ms=20;in.aligned=true;
    const AutoMotionOutput limited=auto_motion_step(&m,in);
    assert(hypotf(limited.command.vx,limited.command.vy)<=.001001f);
    puts("MOTION_STABLE_PREPARATION_OK hold_200ms direction_change_reprepare launch_matches");
}
int main(void) {
    stable_preparation_test();
    smooth_correction_boundary_test();
    stopped_goal_alignment_test();
    prepared_wheel_direction_test();
    endpoint_centering_hold_test();
    along_hold_centering_test();
    heading_reversal_while_translating();endpoint_speed_envelope();
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
