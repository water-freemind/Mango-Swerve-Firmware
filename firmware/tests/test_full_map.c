#include "full_map_test.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static LaserSnapshot scan(AutoPose p,uint32_t stamp) {
    LaserSnapshot l={.frame_count=stamp+1u,.last_rx_ms=stamp,.frame_fresh=true};
    for(unsigned i=0;i<4;i++) {
        AutoPlane plane;float distance;
        if(field_pose_ray(p,i,&plane,&distance)) {
            l.distance_mm[i]=l.accepted_mm[i]=(uint16_t)lroundf(distance*1000);
            l.valid_mask|=(uint8_t)(1u<<i);l.updated_mask|=(uint8_t)(1u<<i);
        } else {l.distance_mm[i]=l.accepted_mm[i]=65535;l.sample_age_ms[i]=UINT32_MAX;}
    }
    return l;
}
static void localization_tests(void) {
    AutoPose start={.2f,-.5f,1.570796327f};AutoPlane plane;float distance;
    assert(!field_pose_ray(start,AUTO_LEFT,&plane,&distance));
    assert(!field_pose_ray(start,AUTO_RIGHT,&plane,&distance));
    assert(!field_pose_ray(start,AUTO_REAR,&plane,&distance));
    assert(field_pose_ray(start,AUTO_FRONT,&plane,&distance)&&distance>5);
    AutoPose entered={.2f,.2f,1.570796327f};
    assert(field_pose_ray(entered,AUTO_LEFT,&plane,&distance)&&plane.surface_id==1);
    assert(distance>.15f&&distance<.17f);
    FieldPose f;assert(field_pose_register(&f,entered,0,0));
    AutoPose actual=entered;actual.x+=.03f;
    LaserSnapshot l=scan(actual,10);
    l.accepted_mm[AUTO_RIGHT]=65135; /* One bad channel cannot discard left/front. */
    assert(field_pose_step(&f,(AutoTwist){0},0,true,&l,10));
    assert(f.pose.pose.x>entered.x && f.pose.pose.x<=entered.x+.00501f);
    assert(f.used_mask&(1u<<AUTO_LEFT));assert(f.rejected_mask&(1u<<AUTO_RIGHT));
    const float previous=f.pose.pose.x;
    for(unsigned i=0;i<4;i++) if(l.valid_mask&(1u<<i)) l.sample_age_ms[i]=11;
    assert(field_pose_step(&f,(AutoTwist){0},0,true,&l,20));
    assert(f.pose.pose.x==previous); /* Same receipt stamp never corrects twice, even across a time/age race. */
    l=(LaserSnapshot){0};
    assert(field_pose_step(&f,(AutoTwist){.vx=.1f},0,true,&l,30));
    assert(f.pose.pose.y>entered.y && f.used_mask==0); /* Odom remains continuous. */
    assert(!field_pose_step(&f,(AutoTwist){.vx=NAN},0,true,&l,40));
    assert(!f.registered);
    /* Re-transmitted frozen distances have fresh CAN receipt stamps. Compare
     * against independent wheel travel, so filter feedback cannot hide them. */
    const AutoPose inside={1.5f,.2f,1.570796327f};
    assert(field_pose_register(&f,inside,0,0));l=scan(inside,0);
    for(unsigned now=10;now<=2000;now+=10) {
        l.frame_count=now;l.last_rx_ms=now; /* Deliberately fresh-but-frozen board output. */
        assert(field_pose_step(&f,(AutoTwist){.vx=.1f},0,true,&l,now));
    }
    assert(f.rejected_mask&(1u<<AUTO_FRONT));
    assert(f.rejected_mask&(1u<<AUTO_REAR));
    assert(f.uncorrected_m[1]>.15f && f.used_mask&(1u<<AUTO_LEFT));
}
static void run_empty_field(bool all,bool bad_rear) {
    FullMapTest m;full_map_init(&m,0);
    AutoPose actual={.2f,-.5f,1.570796327f};AutoTwist actual_v={0};
    FullMapInput in={.healthy=true,.odom_ok=true,.stationary=true,.aligned=true,.laser=scan(actual,0)};
    assert(full_map_command(&m,"MAP ORIGIN",in));
    assert(full_map_command(&m,all?"MAP ARM ALL":"MAP ARM NEXT",in));
    assert(full_map_command(&m,"MAP GO 1",in));
    bool rear_injected=false;unsigned segment_stops=0;
    for(unsigned now=10;now<850000;now+=10) {
        actual_v.vx+=(m.command.vx-actual_v.vx)*.25f;
        actual_v.vy+=(m.command.vy-actual_v.vy)*.25f;
        actual_v.wz+=(m.command.wz-actual_v.wz)*.25f;
        const AutoVec2 delta=auto_rotate((AutoVec2){actual_v.vx*.01f,actual_v.vy*.01f},actual.yaw);
        actual.x+=delta.x;actual.y+=delta.y;actual.yaw=auto_wrap_angle(actual.yaw+actual_v.wz*.01f);
        in.now=now;in.imu_yaw=auto_wrap_angle(actual.yaw-1.570796327f);in.wheel=actual_v;
        in.wheel.vx*=1.032f;in.wheel.vy*=1.032f; /* Independent wheel scale error. */
        in.stationary=hypotf(actual_v.vx,actual_v.vy)<.01f&&fabsf(actual_v.wz)<.02f;
        if(now%100==0) {
            in.laser=scan(actual,now);
            if(bad_rear && m.segment==2 && m.motion.out.progress>.3f && m.motion.out.progress<.7f) {
                in.laser.distance_mm[2]=65135;in.laser.accepted_mm[2]=65135;
                in.laser.valid_mask&=11u;in.laser.updated_mask&=11u;rear_injected=true;
            }
        } else for(unsigned i=0;i<4;i++) if(in.laser.sample_age_ms[i]!=UINT32_MAX) in.laser.sample_age_ms[i]+=10;
        full_map_step(&m,in);
        if(m.phase==MAP_FAULT) {
            fprintf(stderr,"map failure %s segment=%u x=%.4f y=%.4f actual=%.4f,%.4f now=%u\n",
                m.reason,m.segment,m.localization.pose.pose.x,m.localization.pose.pose.y,actual.x,actual.y,now);
            assert(0);
        }
        if(m.phase==MAP_SEGMENT_DONE) {
            assert(!all);++segment_stops;
            assert(full_map_command(&m,"MAP ARM NEXT",in));
            char go[32];snprintf(go,sizeof(go),"MAP GO %lu",(unsigned long)m.token);
            assert(full_map_command(&m,go,in));
        }
        if(m.phase==MAP_DONE) {
            assert(m.coverage==127 && m.segment==17);
            assert(hypotf(actual.x-.2f,actual.y+.5f)<.05f);
            assert(!bad_rear||rear_injected);
            assert(all||segment_stops==16);
            printf("FULL_MAP_REPLAY_OK all=%u bad_rear=%u time_s=%.2f parking_error_m=%.4f\n",
                all,bad_rear,now*.001,hypotf(actual.x-.2f,actual.y+.5f));return;
        }
    }
    assert(0);
}
static void command_tests(void) {
    FullMapTest m;full_map_init(&m,0);
    FullMapInput in={.odom_ok=true,.healthy=true,.stationary=true,.aligned=true};
    assert(!full_map_command(&m,"MAP ARM ALL",in));
    assert(full_map_command(&m,"MAP ORIGIN",in));
    assert(!full_map_command(&m,"MAP SPEED 2.00",in));
    assert(full_map_command(&m,"MAP ARM NEXT",in));
    assert(!full_map_command(&m,"MAP GO 2",in));
    assert(!full_map_command(&m,"MAP ORIGIN",in));
    in.now=4001;assert(!full_map_command(&m,"MAP GO 1",in));
    full_map_step(&m,in);assert(m.phase==MAP_FAULT&&m.command.vx==0);
    assert(full_map_command(&m,"MAP ORIGIN",in));
    assert(full_map_command(&m,"MAP ARM ALL",in));
    assert(full_map_command(&m,"MAP GO 2",in));
    in.healthy=false;in.now+=10;full_map_step(&m,in);
    assert(m.phase==MAP_FAULT && m.command.vx==0 && m.command.vy==0);
}
static void protection_tests(void) {
    FullMapTest m;full_map_init(&m,0);
    FullMapInput in={.odom_ok=true,.healthy=true,.stationary=true,.aligned=true,
        .laser=scan((AutoPose){.2f,-.5f,1.570796327f},1)};
    assert(full_map_command(&m,"MAP ORIGIN",in));
    assert(full_map_command(&m,"MAP ARM ALL",in));assert(full_map_command(&m,"MAP GO 1",in));
    in.laser.frame_fresh=false;in.now=10;full_map_step(&m,in);
    assert(m.phase==MAP_PAUSED && m.command.vx==0);
    in.laser=scan(m.localization.pose.pose,2);in.now=20;full_map_step(&m,in);
    assert(m.phase==MAP_PAUSED); /* One received frame is not recovery proof. */
    for(unsigned now=30;now<=200;now+=10) {in.now=now;full_map_step(&m,in);}
    assert(m.phase==MAP_PAUSED); /* Cached repeated packet cannot count twice. */
    for(unsigned now=210;now<=450;now+=10) {
        in.now=now;in.laser=scan(m.localization.pose.pose,now);full_map_step(&m,in);
    }
    assert(m.phase==MAP_RUNNING);
    assert(full_map_command(&m,"MAP PAUSE",in));
    for(unsigned now=460;now<=600;now+=10) {in.now=now;full_map_step(&m,in);}
    assert(m.phase==MAP_PAUSED&&m.user_hold&&m.command.vx==0);
    assert(full_map_command(&m,"MAP RESUME",in));
    for(unsigned now=610;now<=880;now+=10) {in.now=now;in.laser=scan(m.localization.pose.pose,now);full_map_step(&m,in);}
    assert(m.phase==MAP_RUNNING);
    in.laser=scan(m.localization.pose.pose,900);in.laser.accepted_mm[0]=10;in.now=890;
    in.laser.valid_mask|=1;in.laser.updated_mask|=1;
    full_map_step(&m,in);assert(m.phase==MAP_PAUSED); /* Contact remains protective at an open ray. */
}
int main(void) {
    AutoRoute route;assert(auto_route_build(&route,1,1.570796327f));
    for(unsigned i=0;i<AUTO_ROUTE_COUNT;i++) {
        const AutoRouteSegment s=route.segment[i];
        if(s.aisle==7) assert(s.row==0);
        if(s.aisle>=1&&s.aisle<=6) assert(s.row==s.aisle);
    }
    localization_tests();command_tests();protection_tests();
    run_empty_field(true,false);run_empty_field(true,true);run_empty_field(false,false);
    puts("FULL_MAP_TEST_OK simulation_only_no_hardware_or_work_actions");return 0;
}
