#include "chassis.h"
#include "app_config.h"
#include "log.h"
#include "auto_motion.h"
#include "full_map_test.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#undef assert
#define assert(x) do { if(!(x)) { fprintf(stderr,"CHECK_FAILED %s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while(0)

static uint32_t tick;
static float raw[4], measured_speed, angle[4], velocity[4], steer_limit[4];
static float plant_drive[4];
static bool plant_feedback;
static bool seen = true, output_error, stop_error;
static unsigned stops, writes;
static uint32_t now_ms(void) { return tick; }
static BusMotorStatus init_motor(const BusMotorConfig* c) { (void)c; return MOTOR_STATUS_OK; }
static BusMotorStatus prepare(uint16_t id) { (void)id; return MOTOR_STATUS_OK; }
static BusMotorStatus feedback(uint16_t id, BusMotorFeedback* f) {
    if(!seen) return MOTOR_STATUS_NO_FEEDBACK;
    if(f) *f = (BusMotorFeedback){.id=id, .position=raw[id-1]};
    return MOTOR_STATUS_OK;
}
static float position(uint16_t id) { return raw[id-1]; }
static float speed(uint16_t id) { return plant_feedback?plant_drive[id-1]:measured_speed; }
static BusMotorStatus set_position(uint16_t id, float v, float limit) {
    assert(isfinite(v) && isfinite(limit));
    angle[id-1] = v; steer_limit[id-1] = limit; ++writes;
    return output_error ? MOTOR_STATUS_PORT_ERROR : MOTOR_STATUS_OK;
}
static BusMotorStatus set_speed(uint16_t id, float v) {
    assert(isfinite(v)); velocity[id-1] = v;
    return output_error ? MOTOR_STATUS_PORT_ERROR : MOTOR_STATUS_OK;
}
static BusMotorStatus stop(uint16_t id) { velocity[id-1] = 0; ++stops; return stop_error ? MOTOR_STATUS_PORT_ERROR : MOTOR_STATUS_OK; }
static bool send_frame(uint32_t id, const uint8_t* p, uint8_t n) { (void)id; (void)p; (void)n; return true; }
LogStatus log_info(const char* f, ...) { (void)f; return LOG_STATUS_OK; }
LogStatus log_warn(const char* f, ...) { (void)f; return LOG_STATUS_OK; }
LogStatus log_error(const char* f, ...) { (void)f; return LOG_STATUS_OK; }
bool work_link_hold(void) { return false; }
void work_link_limit_velocity(float* x,float* y,float* w) { (void)x; (void)y; (void)w; }
static const BusMotorInterface motor = {
    .init=init_motor, .update_feedback=feedback, .get_pos=position, .get_spd=speed,
    .set_pos_vel=set_position, .set_spd=set_speed, .stop=stop, .brake=stop
};
static const BusMotorPortOps port = {.send=send_frame, .now_ms=now_ms};
static ChassisConfig config(void) {
    return (ChassisConfig){.steer_motor_interface=&motor, .drive_motor_interface=&motor,
        .steer_ops=&port, .drive_ops=&port, .prepare_steer_motor=prepare, .prepare_drive_motor=prepare,
        .model={.length=.3f,.width=.3f,.wheel_radius=.051f,.max_wheel_linear_speed=1.5f},
        .wheel_drive_ratio=1, .steer_target_mode=CHASSIS_STEER_TARGET_ABS_NEAREST};
}
static void setup(void) {
    memset(raw,0,sizeof raw); memset(velocity,0,sizeof velocity);
    tick=1000; measured_speed=0; plant_feedback=false; seen=true; output_error=stop_error=false; stops=writes=0;
    ChassisConfig c=config(); assert(chassis.init(&c)==chassis.OK);
    assert(chassis.process()==chassis.OK);
}
static void all_zero(void) { for(unsigned i=0;i<4;i++) assert(velocity[i]==0); }
static void move_forward(void) {
    assert(chassis.set_velocity(.1f,0,0)==chassis.OK);
    tick+=2; assert(chassis.process()==chassis.OK);
}
static void invalid_tests(void) {
    const float bad[]={NAN,INFINITY,-INFINITY,FLT_MAX};
    for(unsigned n=0;n<4;n++) {
        setup(); move_forward();
        assert(chassis.set_velocity(bad[n],0,0)!=chassis.OK); all_zero();
        tick+=2; assert(chassis.process()==chassis.OK); all_zero();
        assert(chassis.get_control()->vx==0);
    }
    setup(); move_forward(); raw[2]=NAN; tick+=2;
    assert(chassis.process()!=chassis.OK); all_zero();
    raw[2]=0; tick+=2; assert(chassis.process()==chassis.OK); all_zero();
    setup(); move_forward(); measured_speed=INFINITY; tick+=2;
    assert(chassis.process()!=chassis.OK); all_zero();
    setup(); move_forward(); output_error=true; tick+=2;
    assert(chassis.process()!=chassis.OK); all_zero();
    setup(); move_forward(); seen=false; tick+=2;
    (void)chassis.process(); all_zero();
    seen=true; tick+=2; assert(chassis.process()==chassis.OK); all_zero();
    setup(); move_forward(); ChassisConfig c=config(); c.model.width=NAN;
    assert(chassis.init(&c)!=chassis.OK); all_zero(); assert(!chassis.is_ready());
    setup(); move_forward(); stop_error=true; stops=0;
    assert(chassis.stop()==chassis.MOTOR_OUTPUT_FAILED); assert(stops==8); all_zero();
    assert(chassis.get_control()->vx==0);
}
static float turn_after_100ms(unsigned step) {
    setup(); assert(chassis.set_velocity(0,.1f,0)==chassis.OK);
    for(unsigned t=0;t<100;t+=step) { tick+=step; assert(chassis.process()==chassis.OK); }
    return steer_limit[0];
}
static void timing_tests(void) {
    const float fast=turn_after_100ms(2), slow=turn_after_100ms(10);
    assert(fabsf(fast-slow)<.001f);
    setup(); move_forward(); unsigned count=writes;
    assert(chassis.process()==chassis.OK); assert(writes==count);
#if APP_RUN_MODE == APP_MODE_STRAIGHT_TEST
    assert(chassis_drive_output_allowed());
#endif
    tick+=51; assert(chassis.process()!=chassis.OK); all_zero();
    tick+=2; assert(chassis.process()==chassis.OK); all_zero();
    move_forward(); tick-=10; assert(chassis.process()!=chassis.OK); all_zero();
    setup(); assert(chassis.stop()==chassis.OK);
    tick=UINT32_MAX-1; move_forward(); /* restart clock after explicit stop */
    tick+=2; assert(chassis.process()==chassis.OK); /* unsigned clock rollover */
}
static void steering_tests(void) {
    setup(); move_forward();
    for(unsigned n=0;n<30;n++) {
        assert(chassis.set_velocity(1e-5f,(n&1)?1e-5f:-1e-5f,0)==chassis.OK);
        tick+=2; assert(chassis.process()==chassis.OK); all_zero();
        assert(fabsf(angle[0])<.001f);
    }
    setup(); assert(chassis.set_velocity(0,.1f,0)==chassis.OK);
    tick+=2; assert(chassis.process()==chassis.OK); all_zero();
    /* Enter tighter than exit: old single threshold must not chatter. */
    for(unsigned i=0;i<4;i++) raw[i]=angle[i]-.14f;
    tick+=2; assert(chassis.process()==chassis.OK); all_zero();
    for(unsigned i=0;i<4;i++) raw[i]=angle[i]-.10f;
    tick+=2; assert(chassis.process()==chassis.OK);
    assert(fabsf(velocity[0])>0);
    for(unsigned i=0;i<4;i++) raw[i]=angle[i]-.14f;
    tick+=2; assert(chassis.process()==chassis.OK); assert(fabsf(velocity[0])>0);
    for(unsigned i=0;i<4;i++) raw[i]=angle[i]-.17f;
    tick+=2; assert(chassis.process()==chassis.OK); all_zero();
    setup(); for(unsigned i=0;i<4;i++) raw[i]=3.13f;
    assert(chassis.set_velocity(-.1f,-.001f,0)==chassis.OK);
    tick+=2; assert(chassis.process()==chassis.OK);
    assert(fabsf(angle[0]-raw[0])<.04f);
}
#if APP_RUN_MODE == APP_MODE_STRAIGHT_TEST
static LaserSnapshot plant_scan(const AutoRoute* route,AutoPose p,uint32_t now) {
    LaserSnapshot l={.frame_fresh=true,.last_rx_ms=now,.frame_count=now+1u};
    const float yaw[4]={1.570796327f,-1.570796327f,3.141592654f,0};
    for(unsigned i=0;i<4;++i) {
        const AutoLaserMount mount={.origin={.04098f*cosf(yaw[i]),.04098f*sinf(yaw[i])},.yaw=yaw[i]};
        AutoPlane plane;float distance;
        if(auto_route_ray(route,p,mount,7.5f,&plane,&distance)) {
            l.accepted_mm[i]=l.distance_mm[i]=(uint16_t)lroundf(distance*1000);
            l.valid_mask|=(uint8_t)(1u<<i);l.updated_mask|=(uint8_t)(1u<<i);
        } else {l.accepted_mm[i]=65535u;l.sample_age_ms[i]=UINT32_MAX;}
    }
    return l;
}
static void full_map_steering_plant_test(int side) {
    setup();plant_feedback=true;memset(plant_drive,0,sizeof plant_drive);
    ChassisConfig cc=config();cc.model.length=cc.model.width=.265729869f;
    assert(chassis.init(&cc)==chassis.OK);
    for(unsigned i=0;i<4;++i) raw[i]=angle[i]=1.570796327f;
    SteerWheel plant;assert(swheel.init(&plant,cc.model)==swheel.OK);
    FullMapTest m;full_map_init(&m,tick);
    AutoPose actual={2.35f,2.3f+side*.038f,1.570796327f};
    assert(field_pose_register(&m.localization,(AutoPose){actual.x,2.3f+side*.013f,actual.yaw},0,tick));
    m.segment=8;m.phase=MAP_ARMED;m.token=1;m.arm_ms=tick;m.speed=.70f;
    FullMapInput in={.now=tick,.healthy=true,.odom_ok=true,.stationary=true,
        .laser=plant_scan(&m.route,actual,tick)};
    assert(full_map_command(&m,"MAP GO 1",in));
    unsigned moving_correction=0,reclosed=0;bool previous_gate=false;float minimum_range=1000;
    for(unsigned step=1;step<=1500;++step) {
        tick+=10;
        const float drive_sign[4]={1,-1,-1,1};
        for(unsigned i=0;i<4;++i) {
            const float steering=(angle[i]-raw[i])*.12f;
            raw[i]+=fmaxf(-steer_limit[i]*.01f,fminf(steer_limit[i]*.01f,steering));
            plant_drive[i]+=(velocity[i]-plant_drive[i])*.20f;
            plant.state.cur_wheels[i]=(WheelModule){drive_sign[i]*plant_drive[i],raw[i]};
        }
        assert(swheel.fk(&plant)==swheel.OK);
        /* The production chassis reflects internal Y/yaw into its external
         * body frame before the map/IMU adapter reads it. */
        in.wheel=(AutoTwist){plant.state.cur_vx,-plant.state.cur_vy,-plant.state.cur_wz};
        const AutoVec2 delta=auto_rotate((AutoVec2){in.wheel.vx*.01f,in.wheel.vy*.01f},actual.yaw);
        actual.x+=delta.x;actual.y+=delta.y;actual.yaw+=in.wheel.wz*.01f;
        in.now=tick;in.imu_yaw=actual.yaw-1.570796327f;
        in.stationary=hypotf(in.wheel.vx,in.wheel.vy)<.01f && fabsf(in.wheel.wz)<.02f;
        in.aligned=chassis_drive_output_allowed();
        if(step%10u==0u) in.laser=plant_scan(&m.route,actual,tick);
        else for(unsigned i=0;i<4;++i) if(in.laser.sample_age_ms[i]!=UINT32_MAX) in.laser.sample_age_ms[i]+=10u;
        full_map_step(&m,in);
        if(m.phase!=MAP_RUNNING) fprintf(stderr,"G4_PLANT_FAILURE side=%d phase=%u reason=%s t=%u actual=%.4f,%.4f\n",
            side,m.phase,m.reason,step*10,actual.x,actual.y);
        assert(m.phase==MAP_RUNNING);
        const bool prepare=m.motion.out.phase==AUTO_MOTION_ALIGN;
        const AutoTwist v=prepare?m.hint:m.command;
        assert((prepare?chassis_prepare_velocity(v.vx,v.vy,v.wz):chassis.set_velocity(v.vx,v.vy,v.wz))==chassis.OK);
        assert(chassis.process()==chassis.OK);
        const bool gate=chassis_drive_output_allowed();
        if(previous_gate && !gate && !m.tree_hold) ++reclosed;
        previous_gate=gate;
        if(m.aisle_correction.centering && fabsf(actual.x-m.motion.segment.end.x)>.05f && m.command.vy>.01f)
            ++moving_correction;
        const unsigned near=in.laser.accepted_mm[AUTO_FRONT]<in.laser.accepted_mm[AUTO_REAR]?
            in.laser.accepted_mm[AUTO_FRONT]:in.laser.accepted_mm[AUTO_REAR];
        if(near<minimum_range) minimum_range=(float)near;
        if(m.trees_done&(1u<<7)) {
            printf("G4_CHASSIS_PLANT side=%d time_s=%.2f gate_reclosures=%u moving_correction_ticks=%u min_range_mm=%.0f cross_mm=%.1f\n",
                side,step*.01,reclosed,moving_correction,minimum_range,(actual.y-2.3f)*1000);
            assert(moving_correction>=20u && reclosed<=3u && minimum_range>100 && fabsf(actual.y-2.3f)<.025f);
            return;
        }
    }
    assert(0);
}
static void entry_wall_steering_plant_test(float speed) {
    setup();plant_feedback=true;memset(plant_drive,0,sizeof plant_drive);
    ChassisConfig cc=config();cc.model.length=cc.model.width=.265729869f;
    assert(chassis.init(&cc)==chassis.OK);
    for(unsigned i=0;i<4;++i) raw[i]=angle[i]=0;
    SteerWheel plant;assert(swheel.init(&plant,cc.model)==swheel.OK);
    FullMapTest m;full_map_init(&m,tick);
    AutoPose actual={.20f,.30f,1.570796327f};
    assert(field_pose_register(&m.localization,actual,0,tick));
    m.phase=MAP_ARMED;m.token=1;m.arm_ms=tick;m.speed=speed;
    FullMapInput in={.now=tick,.healthy=true,.odom_ok=true,.stationary=true,
        .laser=plant_scan(&m.route,actual,tick)};
    assert(full_map_command(&m,"MAP GO 1",in));
    float peak_cross=0,peak_y=0;unsigned reclosed=0;bool previous_gate=false,wall_seen=false;
    for(unsigned step=1;step<=5000;++step) {
        tick+=10;
        const float drive_sign[4]={1,-1,-1,1};
        for(unsigned i=0;i<4;++i) {
            const float steering=(angle[i]-raw[i])*.12f;
            raw[i]+=fmaxf(-steer_limit[i]*.01f,fminf(steer_limit[i]*.01f,steering));
            plant_drive[i]+=(velocity[i]-plant_drive[i])*.20f;
            plant.state.cur_wheels[i]=(WheelModule){drive_sign[i]*plant_drive[i],raw[i]};
        }
        assert(swheel.fk(&plant)==swheel.OK);
        in.wheel=(AutoTwist){plant.state.cur_vx,-plant.state.cur_vy,-plant.state.cur_wz+.01f};
        /* Unobserved lateral tyre slip pushes west by 20mm/s. Wheel odometry
         * does not receive that displacement; the wall sensor has to reject it. */
        const float slip=in.wheel.vx>.05f?.020f:0;
        const AutoVec2 delta=auto_rotate((AutoVec2){in.wheel.vx*.01f,(in.wheel.vy+slip)*.01f},actual.yaw);
        actual.x+=delta.x;actual.y+=delta.y;actual.yaw+=in.wheel.wz*.01f;
        if(fabsf(actual.x-.20f)>peak_cross) {peak_cross=fabsf(actual.x-.20f);peak_y=actual.y;}
        in.now=tick;in.imu_yaw=actual.yaw-1.570796327f;
        in.stationary=hypotf(in.wheel.vx,in.wheel.vy)<.01f && fabsf(in.wheel.wz)<.02f;
        in.aligned=chassis_drive_output_allowed();
        if(step%10u==0u) in.laser=plant_scan(&m.route,actual,tick);
        else for(unsigned i=0;i<4;++i) if(in.laser.sample_age_ms[i]!=UINT32_MAX) in.laser.sample_age_ms[i]+=10u;
        full_map_step(&m,in);wall_seen|=m.wall_correction.active;
        if(m.phase!=MAP_RUNNING) fprintf(stderr,"ENTRY_PLANT_FAILURE speed=%.2f reason=%s time=%u actual=%.4f,%.4f wheel=%.4f,%.4f,%.4f masks=%x,%x,%x predicted=%x mm=%u,%u,%u,%u budget=%.0f,%.0f,%.0f,%.0f wall=%u/%.1f\n",speed,m.reason,step*10,actual.x,actual.y,
            in.wheel.vx,in.wheel.vy,in.wheel.wz,m.range_near_mask,m.range_brake_mask,m.range_stale_mask,m.predicted_mask,
            m.range_control_mm[0],m.range_control_mm[1],m.range_control_mm[2],m.range_control_mm[3],
            m.range_budget_m[0]*1000,m.range_budget_m[1]*1000,m.range_budget_m[2]*1000,m.range_budget_m[3]*1000,
                m.wall_correction.active,m.wall_correction.error_m*1000);
        if(m.phase!=MAP_RUNNING) fprintf(stderr,"ENTRY_OBSERVATION used=%x rejected=%x constrained_age=%u,%u uncorrected=%.3f,%.3f source_age=%u wall_ref=%u motion_error=%u yaw=%.2f\n",
            m.localization.used_mask,m.localization.rejected_mask,tick-m.localization.constrained_ms[0],tick-m.localization.constrained_ms[1],
            m.localization.uncorrected_m[0],m.localization.uncorrected_m[1],in.laser.sample_age_ms[AUTO_LEFT],m.cross_feedback.wall_reference,m.motion_error,in.imu_yaw*57.29578f);
        if(getenv("MANGO_SHIFT_DEBUG") && step%100u==0u) fprintf(stderr,"ENTRY_END step=%u seg=%u y=%.3f fused=%.3f yaw=%.2f phase=%u align=%u cmd=%.4f,%.4f,%.4f hint=%.3f,%.3f,%.3f block=%x cross=%.1f\n",
            step,m.segment,actual.y,m.localization.pose.pose.y,in.imu_yaw*57.29578f,m.motion.out.phase,in.aligned,
            m.command.vx,m.command.vy,m.command.wz,m.hint.vx,m.hint.vy,m.hint.wz,m.required_mask,m.cross_feedback.error_m*1000);
        assert(m.phase==MAP_RUNNING);
        const bool prepare=m.motion.out.phase==AUTO_MOTION_ALIGN;
        const AutoTwist v=prepare?m.hint:m.command;
        assert((prepare?chassis_prepare_velocity(v.vx,v.vy,v.wz):chassis.set_velocity(v.vx,v.vy,v.wz))==chassis.OK);
        assert(chassis.process()==chassis.OK);
        const bool gate=chassis_drive_output_allowed();
        if(previous_gate && !gate && actual.y>.6f && actual.y<3.9f) ++reclosed;
        previous_gate=gate;
        if(m.segment==2u) {
            printf("ENTRY_WALL_CHASSIS_PLANT speed=%.2f time_s=%.2f peak_cross_mm=%.1f peak_y=%.2f gate_reclosures=%u wall_seen=%u\n",
                speed,step*.01,peak_cross*1000,peak_y,reclosed,wall_seen);
            assert(wall_seen && peak_cross<.030f && reclosed<=1u);
            return;
        }
    }
    assert(0);
}
static void shift_wall_gap_steering_plant_test(int side,float speed) {
    setup();plant_feedback=true;memset(plant_drive,0,sizeof plant_drive);
    ChassisConfig cc=config();cc.model.length=cc.model.width=.265729869f;
    assert(chassis.init(&cc)==chassis.OK);
    for(unsigned i=0;i<4;++i) raw[i]=angle[i]=1.570796327f;
    SteerWheel plant;assert(swheel.init(&plant,cc.model)==swheel.OK);
    FullMapTest m;full_map_init(&m,tick);
    const unsigned seg=side>0?3u:5u;
    const float nominal=side>0?2.8f:.2f;
    AutoPose actual={nominal+side*-.033f,m.route.segment[seg].line.start.y,1.570796327f};
    assert(field_pose_register(&m.localization,(AutoPose){nominal+side*-.020f,actual.y,actual.yaw},0,tick));
    m.segment=seg;m.phase=MAP_ARMED;m.token=1;m.arm_ms=tick;m.speed=speed;
    FullMapInput in={.now=tick,.healthy=true,.odom_ok=true,.stationary=true,
        .laser=plant_scan(&m.route,actual,tick)};
    assert(full_map_command(&m,"MAP GO 1",in));
    unsigned reclosed=0,align_ticks=0,pauses=0;bool previous_gate=false,previous_pause=false;
    LaserSnapshot last=in.laser;
    for(unsigned step=1;step<=2500;++step) {
        tick+=10;const float drive_sign[4]={1,-1,-1,1};
        for(unsigned i=0;i<4;++i) {
            const float steering=(angle[i]-raw[i])*.12f;
            raw[i]+=fmaxf(-steer_limit[i]*.01f,fminf(steer_limit[i]*.01f,steering));
            plant_drive[i]+=(velocity[i]-plant_drive[i])*.20f;
            plant.state.cur_wheels[i]=(WheelModule){drive_sign[i]*plant_drive[i],raw[i]};
        }
        assert(swheel.fk(&plant)==swheel.OK);
        in.wheel=(AutoTwist){plant.state.cur_vx,-plant.state.cur_vy,-plant.state.cur_wz};
        const AutoVec2 delta=auto_rotate((AutoVec2){in.wheel.vx*.01f,in.wheel.vy*.01f},actual.yaw);
        actual.x+=delta.x;actual.y+=delta.y;actual.yaw+=in.wheel.wz*.01f;
        in.now=tick;in.imu_yaw=actual.yaw-1.570796327f+.010f*sinf(step*.07f);
        in.stationary=hypotf(in.wheel.vx,in.wheel.vy)<.01f && fabsf(in.wheel.wz)<.02f;
        in.aligned=chassis_drive_output_allowed();
        if(step%10u==0u) {
            in.laser=plant_scan(&m.route,actual,tick);
            const unsigned near=side>0?AUTO_RIGHT:AUTO_LEFT,far=side>0?AUTO_LEFT:AUTO_RIGHT;
            /* Opposite fence adds a 55mm bias to the fused coordinate, while
             * the nearby fence intermittently misses two producer frames. */
            in.laser.accepted_mm[far]+=55u;
            if(step%70u<25u) {
                in.laser.accepted_mm[near]=last.accepted_mm[near];
                in.laser.sample_age_ms[near]=last.sample_age_ms[near]+100u;
                in.laser.updated_mask&=(uint8_t)~(1u<<near);
                if(in.laser.sample_age_ms[near]>300u) in.laser.valid_mask&=(uint8_t)~(1u<<near);
            }
            last=in.laser;
        } else for(unsigned i=0;i<4;++i) if(in.laser.sample_age_ms[i]!=UINT32_MAX) in.laser.sample_age_ms[i]+=10u;
        full_map_step(&m,in);
        if(m.phase==MAP_FAULT) fprintf(stderr,"SHIFT_PLANT_FAULT side=%d speed=%.2f reason=%s actual=%.4f,%.4f\n",side,speed,m.reason,actual.x,actual.y);
        assert(m.phase!=MAP_FAULT);
        const bool paused=m.phase==MAP_PAUSED;
        if(paused && !previous_pause) ++pauses;
        previous_pause=paused;
        const bool prepare=m.motion.out.phase==AUTO_MOTION_ALIGN;
        align_ticks+=prepare;
        const AutoTwist v=prepare?m.hint:m.command;
        assert((prepare?chassis_prepare_velocity(v.vx,v.vy,v.wz):chassis.set_velocity(v.vx,v.vy,v.wz))==chassis.OK);
        assert(chassis.process()==chassis.OK);
        const bool gate=chassis_drive_output_allowed();
        if(previous_gate && !gate) {
            ++reclosed;
            if(getenv("MANGO_SHIFT_DEBUG")) fprintf(stderr,"SHIFT_GATE step=%u pos=%.3f,%.3f yaw=%.2f limit=%.3f pred=%x phase=%u cmd=%.4f,%.4f,%.4f wall=%u/%.1f cross=%.1f\n",
                step,actual.x,actual.y,in.imu_yaw*57.29578f,m.motion.config.vmax,m.predicted_mask,m.motion.out.phase,
                m.command.vx,m.command.vy,m.command.wz,m.wall_correction.active,m.wall_correction.error_m*1000,m.motion.out.cross_error*1000);
        }
        previous_gate=gate;
        if(getenv("MANGO_SHIFT_DEBUG") && step%100u==0u) fprintf(stderr,"SHIFT_PROGRESS side=%d speed=%.2f step=%u y=%.4f fused=%.4f phase=%u limit=%.3f pred=%x odom=%x cmd=%.4f,%.4f,%.4f wall=%u/%.1f feedback=%.1f\n",
            side,speed,step,actual.y,m.localization.pose.pose.y,m.motion.out.phase,m.motion.config.vmax,m.predicted_mask,m.odom_range_mask,
            m.command.vx,m.command.vy,m.command.wz,m.wall_correction.active,m.wall_correction.error_m*1000,m.cross_feedback.error_m*1000);
        if(m.segment!=seg) {
            printf("SHIFT_WALL_GAP_PLANT side=%d speed=%.2f time_s=%.2f gate_reclosures=%u align_ticks=%u pauses=%u cross_mm=%.1f\n",
                side,speed,step*.01,reclosed,align_ticks,pauses,(actual.x-nominal)*1000);
            assert(step<850u && reclosed<=3u && pauses==0u && fabsf(actual.x-nominal)<.030f);
            return;
        }
    }
    assert(0);
}
static void endpoint_motion_alignment_test(void) {
    setup();ChassisConfig cc=config();cc.model.length=cc.model.width=.16f;
    assert(chassis.init(&cc)==chassis.OK);assert(chassis.process()==chassis.OK);
    const AutoMotionConfig c={.vmax=.20f,.accel=.30f,.decel=.40f,.lateral_max=.10f,.cross_kp=1.2f,
        .yaw_kp=1.2f,.wz_max=.06f,.angular_accel=.15f,.position_tolerance=.02f,
        .cross_tolerance=.010f,.yaw_tolerance=.034906585f,.cross_abort=.08f,.yaw_abort=.15f,
        .settle_s=.4f,.stop_latency_s=.5f,.pose_max_age_ms=50,
        .finish_yaw_tolerance=.069813170f,.finish_cross_tolerance=.02f,.smooth_correction=true};
    AutoMotion m;assert(auto_motion_begin(&m,&c,(AutoSegment){{0,0},{1,0},0},tick));
    AutoPose actual={1,.022f,.034906585f};AutoTwist measured={0},applied={0};
    unsigned blocked=0,reclosed=0;bool previous_gate=false;
    for(unsigned step=1;step<=500u;++step) {
        tick+=10;
        for(unsigned wheel=0;wheel<4u;++wheel) {
            const float correction=(angle[wheel]-raw[wheel])*.25f;
            raw[wheel]+=fmaxf(-.06f,fminf(.06f,correction));
        }
        measured.vx+=(applied.vx-measured.vx)*.25f;
        measured.vy+=(applied.vy-measured.vy)*.25f;
        measured.wz+=(applied.wz-measured.wz)*.25f;
        const AutoVec2 delta=auto_rotate((AutoVec2){measured.vx*.01f,measured.vy*.01f},actual.yaw);
        actual.x+=delta.x;actual.y+=delta.y;actual.yaw+=measured.wz*.01f;
        AutoPose observed=actual;
        observed.y+=.003f*sinf(step*.63f);observed.yaw+=.012f*sinf(step*.47f);
        AutoMotionInput in={.pose={observed,tick,true},.measured=measured,.healthy=true,
            .aligned=chassis_drive_output_allowed(),
            .stationary=hypotf(measured.vx,measured.vy)<.01f && fabsf(measured.wz)<.02f,.now_ms=tick};
        const AutoMotionOutput out=auto_motion_step(&m,in);
        if(out.phase==AUTO_MOTION_ALIGN)
            assert(chassis_prepare_velocity(out.steering_hint.vx,out.steering_hint.vy,out.steering_hint.wz)==chassis.OK);
        else assert(chassis.set_velocity(out.command.vx,out.command.vy,out.command.wz)==chassis.OK);
        assert(chassis.process()==chassis.OK);
        const bool gate=chassis_drive_output_allowed();
        blocked+=out.phase==AUTO_MOTION_ALIGN;
        if(previous_gate && !gate) ++reclosed;
        previous_gate=gate;
        applied=gate && out.phase!=AUTO_MOTION_ALIGN?out.command:(AutoTwist){0};
        assert(out.phase!=AUTO_MOTION_FAULT);
        if(out.phase==AUTO_MOTION_DONE) {
            assert(reclosed<=2u && fabsf(actual.y)<.02f);
            printf("CHASSIS_MOTION_ALIGNMENT_OK time_s=%.2f align_ticks=%u gate_reclosures=%u cross_mm=%.1f\n",
                step*.01,blocked,reclosed,actual.y*1000);return;
        }
    }
    assert(0);
}
#endif
int main(void) {
#if APP_RUN_MODE == APP_MODE_STRAIGHT_TEST
    shift_wall_gap_steering_plant_test(1,.30f);
    shift_wall_gap_steering_plant_test(-1,.30f);
    shift_wall_gap_steering_plant_test(1,.50f);
    shift_wall_gap_steering_plant_test(-1,.50f);
    shift_wall_gap_steering_plant_test(1,.70f);
    shift_wall_gap_steering_plant_test(-1,.70f);
    entry_wall_steering_plant_test(.5f);entry_wall_steering_plant_test(.7f);
    full_map_steering_plant_test(-1);full_map_steering_plant_test(1);
    endpoint_motion_alignment_test();
#endif
    invalid_tests(); timing_tests(); steering_tests();
    setup();
    assert(chassis_prepare_velocity(.1f,0,0)==chassis.OK);
    for(unsigned step=0;step<30;step++) {
        tick+=2;assert(chassis.process()==chassis.OK);all_zero();
        for(unsigned i=0;i<4;i++) raw[i]=angle[i];
    }
    assert(chassis.set_velocity(.1f,0,0)==chassis.OK);
    tick+=2;assert(chassis.process()==chassis.OK);assert(fabsf(velocity[0])>0);
    setup();
    assert(chassis_set_wheel_speed_limit(1.0f) == chassis.OK);
    assert(chassis.get_chassis()->kine.model.max_wheel_linear_speed == 1.0f);
    assert(chassis_set_wheel_speed_limit(NAN) == chassis.INVALID_PARAM);
    assert(chassis_set_wheel_speed_limit(0.0f) == chassis.INVALID_PARAM);
    assert(chassis_set_wheel_speed_limit(2.0f) == chassis.INVALID_PARAM);
    assert(chassis.get_chassis()->kine.model.max_wheel_linear_speed == 1.0f);
    assert(chassis_set_wheel_speed_limit(1.5f) == chassis.OK);
    puts("CHASSIS_CONTROL_TEST_OK numeric, timing, stale-output, alignment and angle-wrap");
    return 0;
}
