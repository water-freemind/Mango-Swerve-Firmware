#include "full_map_test.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#undef assert
#define assert(x) do { if(!(x)) { fprintf(stderr,"CHECK_FAILED %s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while(0)

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
    if(!all) assert(full_map_command(&m,"MAP SPEED 0.70",in));
    assert(full_map_command(&m,"MAP ARM ALL",in));
    assert(full_map_command(&m,"MAP GO 1",in));
    bool rear_injected=false,entry_crossed_moving=false;unsigned segment_stops=0,tree_count=0;
    const unsigned tree_order[12]={12,11,9,10,8,7,5,6,4,3,1,2};
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
        const uint16_t done_before=m.trees_done;
        full_map_step(&m,in);
        assert(fabsf(m.motion.config.settle_s-(m.target_tree || m.segment>=16u ? .4f:.15f))<.00001f);
        if(m.tree_hold) assert(m.command.vx==0 && m.command.vy==0 && m.command.wz==0);
        if(m.trees_done!=done_before) {
            assert(tree_count<12 && (m.trees_done^done_before)==(1u<<(tree_order[tree_count]-1u)));
            assert(now-m.tree_hold_ms>=2000u);++tree_count;
        }
        assert(m.segment!=1u); /* Initial drive skips the obsolete stop record. */
        if(actual.y>.20f && actual.y<.30f && actual.x<.3f && m.segment==0u) {
            assert(actual_v.vx>.02f && m.command.vx>.02f);
            entry_crossed_moving=true;
        }
        if(m.phase==MAP_FAULT) {
            fprintf(stderr,"map failure %s segment=%u x=%.4f y=%.4f actual=%.4f,%.4f now=%u\n",
                m.reason,m.segment,m.localization.pose.pose.x,m.localization.pose.pose.y,actual.x,actual.y,now);
            assert(0);
        }
        if(m.phase==MAP_SEGMENT_DONE) {
            assert(0); /* Full-map mode never waits for per-segment clicks. */
            ++segment_stops;
            assert(full_map_command(&m,"MAP ARM ALL",in));
            char go[32];snprintf(go,sizeof(go),"MAP GO %lu",(unsigned long)m.token);
            assert(full_map_command(&m,go,in));
        }
        if(m.phase==MAP_DONE) {
            assert(m.coverage==127 && m.segment==17);
            assert(tree_count==12 && m.trees_done==4095u);
            assert(hypotf(actual.x-.2f,actual.y+.5f)<.05f);
            assert(!bad_rear||rear_injected);
            assert(entry_crossed_moving);
            assert(segment_stops==0);
            printf("FULL_MAP_REPLAY_OK all=%u bad_rear=%u time_s=%.2f parking_error_m=%.4f\n",
                all,bad_rear,now*.001,hypotf(actual.x-.2f,actual.y+.5f));return;
        }
    }
    assert(0);
}
static void command_tests(void) {
    FullMapTest m;full_map_init(&m,0);
    FullMapInput in={.odom_ok=true,.healthy=true,.stationary=true,.aligned=true,
        .laser=scan((AutoPose){.2f,-.5f,1.570796327f},0)};
    assert(!full_map_command(&m,"MAP ARM ALL",in));
    assert(full_map_command(&m,"MAP ORIGIN",in));
    assert(fabsf(m.speed-.30f)<.00001f);
    assert(!full_map_command(&m,"MAP ARM NEXT",in));
    assert(!full_map_command(&m,"MAP SPEED 0.10",in));
    const char* presets[]={"0.30","0.40","0.50","0.60","0.70"};
    for(unsigned i=0;i<5;++i) {char speed[32];snprintf(speed,sizeof(speed),"MAP SPEED %s",presets[i]);assert(full_map_command(&m,speed,in));}
    assert(full_map_command(&m,"MAP SPEED 0.20",in));
    assert(!full_map_command(&m,"MAP SPEED 2.00",in));
    assert(full_map_command(&m,"MAP ARM ALL",in));
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
static void range_threshold_tests(void) {
    for(unsigned mm=99;mm<=101;++mm) {
        FullMapTest m;full_map_init(&m,0);
        const AutoPose actual={.2f,-.2f,1.570796327f};
        FullMapInput in={.odom_ok=true,.healthy=true,.stationary=true,.aligned=true,
            .laser=scan(actual,0)};
        assert(full_map_command(&m,"MAP ORIGIN",in));
        assert(fabsf(m.localization.pose.pose.y-actual.y)<.001f);
        assert(fabsf(m.route.segment[0].line.start.y-actual.y)<.001f);
        assert(full_map_command(&m,"MAP ARM ALL",in));
        assert(full_map_command(&m,"MAP GO 1",in));
        /* A close side return protects even where the map has no side wall.
         * 101mm must not acquire an extra sensor-to-body offset. */
        in.laser.accepted_mm[AUTO_LEFT]=(uint16_t)mm;
        in.laser.valid_mask|=1;in.laser.updated_mask|=1;
        in.now=10;full_map_step(&m,in);
        assert(m.phase==(mm<=100?MAP_PAUSED:MAP_RUNNING));
        if(mm<=100) assert(m.command.vx==0 && m.command.vy==0);
    }
}
static void endpoint_acceptance_tests(void) {
    FullMapTest map;full_map_init(&map,0);
    FullMapInput in={.odom_ok=true,.healthy=true,.stationary=true,.aligned=true,
        .laser=scan((AutoPose){.2f,-.2f,1.570796327f},0)};
    assert(full_map_command(&map,"MAP ORIGIN",in));
    assert(full_map_command(&map,"MAP ARM ALL",in));
    assert(full_map_command(&map,"MAP GO 1",in));
    AutoMotion motion;const AutoSegment line={{0,0},{1,0},0};
    assert(auto_motion_begin(&motion,&map.motion.config,line,0));
    for(unsigned now=10;now<=600;now+=10) {
        AutoMotionInput sample={.pose={{1.015f,.015f,.04f},now,true},
            .healthy=true,.stationary=true,.aligned=true,.now_ms=now};
        auto_motion_step(&motion,sample);
    }
    assert(motion.out.phase==AUTO_MOTION_DONE);
    assert(auto_motion_begin(&motion,&map.motion.config,line,0));
    for(unsigned now=10;now<=600;now+=10) {
        AutoMotionInput sample={.pose={{1,.025f,0},now,true},
            .healthy=true,.stationary=true,.aligned=true,.now_ms=now};
        auto_motion_step(&motion,sample);
    }
    assert(motion.out.phase!=AUTO_MOTION_DONE && motion.out.command.vy<0);
    assert(auto_motion_begin(&motion,&map.motion.config,line,0));
    AutoMotionInput sample={.pose={{1.025f,0,0},10,true},
        .healthy=true,.stationary=true,.aligned=true,.now_ms=10};
    auto_motion_step(&motion,sample);
    assert(motion.out.error==AUTO_MOTION_NO_ERROR && motion.out.command.vx<0);
    assert(auto_motion_begin(&motion,&map.motion.config,line,0));
    sample.pose.pose.x=1.065f;
    auto_motion_step(&motion,sample);
    assert(motion.out.error==AUTO_MOTION_OVERSHOOT);
}
static LaserSnapshot ridge_scan(AutoPose p,uint32_t stamp) {
    AutoRoute route;assert(auto_route_build(&route,1,1.570796327f));
    LaserSnapshot l=scan(p,stamp);
    const float a[4]={1.570796327f,-1.570796327f,3.141592654f,0};
    for(unsigned i=0;i<4;i++) {
        AutoLaserMount mount={.origin={.04098f*cosf(a[i]),.04098f*sinf(a[i])},.yaw=a[i]};
        AutoPlane plane;float distance;
        if(auto_route_ray(&route,p,mount,7.5f,&plane,&distance)) {
            l.distance_mm[i]=l.accepted_mm[i]=(uint16_t)lroundf(distance*1000);
            l.valid_mask|=(uint8_t)(1u<<i);l.updated_mask|=(uint8_t)(1u<<i);
        }
    }
    return l;
}
static FullMapInput setup_aisle(FullMapTest* m,unsigned segment,float x) {
    full_map_init(m,0);
    FullMapInput in={.healthy=true,.odom_ok=true,.stationary=true,.aligned=true,
        .laser=scan((AutoPose){.2f,-.5f,1.570796327f},0)};
    assert(full_map_command(m,"MAP ORIGIN",in));
    assert(full_map_command(m,"MAP ARM ALL",in));assert(full_map_command(m,"MAP GO 1",in));
    m->segment=segment;
    AutoPose p={x,m->route.segment[segment].line.start.y,1.570796327f};
    assert(field_pose_register(&m->localization,p,0,0));
    const AutoMotionConfig config=m->motion.config;
    assert(auto_motion_begin(&m->motion,&config,m->route.segment[segment].line,0));
    in.laser=ridge_scan(p,0);
    return in;
}
static void g7_ridge_exit_test(void) {
    FullMapTest m;FullMapInput in=setup_aisle(&m,2,2.5f);
    AutoPose actual={2.5f,4.4f,1.570796327f};AutoTwist v={0};
    bool exited=false;
    for(unsigned now=10;now<30000;now+=10) {
        v.vx+=(m.command.vx-v.vx)*.25f;v.vy+=(m.command.vy-v.vy)*.25f;
        v.wz+=(m.command.wz-v.wz)*.25f;
        AutoVec2 delta=auto_rotate((AutoVec2){v.vx*.01f,v.vy*.01f},actual.yaw);
        actual.x+=delta.x;actual.y+=delta.y;actual.yaw=auto_wrap_angle(actual.yaw+v.wz*.01f);
        in.now=now;in.wheel=v;in.imu_yaw=auto_wrap_angle(actual.yaw-1.570796327f);
        in.stationary=hypotf(v.vx,v.vy)<.01f && fabsf(v.wz)<.02f;
        if(now%100u==0) {
            in.laser=ridge_scan(actual,now);
            if(actual.x>2.6f) { /* Target changes from R6 to a biased far return. */
                in.laser.accepted_mm[AUTO_REAR]=(uint16_t)lroundf((actual.y-.04098f+.08f)*1000);
                exited=true;
            }
        } else for(unsigned ch=0;ch<4;++ch) in.laser.sample_age_ms[ch]+=10;
        const unsigned before=m.segment;
        full_map_step(&m,in);
        if(before==2u) assert(!(m.localization.used_mask&(1u<<AUTO_REAR)));
        assert(m.phase!=MAP_FAULT && m.phase!=MAP_PAUSED);
        if(m.segment==3u) {
            assert(exited && fabsf(actual.y-4.4f)<.02f);
            assert(!m.aisle_correction.active && m.target_tree==0u);
            /* Mask is local to G7. Rear is still a collision input everywhere. */
            in.now+=10;in.laser=ridge_scan(actual,in.now);
            in.laser.accepted_mm[AUTO_REAR]=99;
            full_map_step(&m,in);assert(m.phase==MAP_PAUSED);
            puts("G7_RIDGE_EXIT_TEST_OK target switch, biased far rear, next leg, rear near-stop");return;
        }
    }
    assert(0);
}
static void aisle_correction_tests(void) {
    for(int sign=-1;sign<=1;sign+=2) {
        FullMapTest m;FullMapInput in=setup_aisle(&m,12,2);
        AutoPose actual={2,.9f+sign*.04f,1.570796327f};
        for(unsigned now=10;now<=1600;now+=10) {
            in.now=now;
            if(now%100==0) in.laser=ridge_scan(actual,now);
            else for(unsigned i=0;i<4;++i) in.laser.sample_age_ms[i]+=10;
            full_map_step(&m,in);assert(m.phase==MAP_RUNNING);
        }
        assert(m.aisle_correction.active);
        assert(sign*(m.localization.pose.pose.y-.9f)>.015f);
        assert(sign*m.command.vx<0); /* Front close -> backward; rear close -> forward. */
        assert(fabsf(m.aisle_correction.correction_m)<=.00201f);
        /* No repeated frame may acquire confirmation or move the observation. */
        const float y=m.localization.pose.pose.y;
        in.now+=10;full_map_step(&m,in);
        assert(fabsf(m.localization.pose.pose.y-y)<.00001f);
        in.laser.sample_age_ms[2]=in.laser.sample_age_ms[3]=301;
        in.now+=10;full_map_step(&m,in);assert(m.aisle_correction.active);
        for(unsigned tick=0;tick<21;++tick) {in.now+=10;full_map_step(&m,in);}
        assert(!m.aisle_correction.active);
    }
    FullMapTest m;FullMapInput in=setup_aisle(&m,12,2);
    in.laser=ridge_scan((AutoPose){2,.94f,1.570796327f},0);
    for(unsigned now=10;now<=100;now+=10){in.now=now;full_map_step(&m,in);}
    assert(!m.aisle_correction.active && m.aisle_correction.confirmations==1);
    ++in.laser.frame_count;in.now+=10;full_map_step(&m,in);
    assert(m.aisle_correction.confirmations==1); /* Same receipt time is not a new sample. */
    in=setup_aisle(&m,12,2);
    for(unsigned now=10;now<=1400;now+=10) {
        in.now=now;in.wheel.vx=.04f;in.stationary=false;
        /* Keep the fused estimate fixed to isolate the independent wheel
         * checkpoint from the map-envelope guard in this frozen-data test. */
        m.localization.pose.pose.y=.9f;
        /* Fresh CAN stamps, but identical sensor distances despite north travel. */
        in.laser=ridge_scan((AutoPose){2,.94f,1.570796327f},now);
        full_map_step(&m,in);
    }
    assert(m.aisle_correction.frozen && !m.aisle_correction.active);
    in=setup_aisle(&m,12,2);in.now=10;
    /* Implausible gap and real near returns must not be smoothed away. */
    in.now+=10;in.laser=ridge_scan((AutoPose){2,.9f,1.570796327f},in.now);
    in.laser.accepted_mm[2]=in.laser.accepted_mm[3]=500;
    full_map_step(&m,in);assert(!m.aisle_correction.active);
    in.now+=10;in.laser=ridge_scan((AutoPose){2,.97f,1.570796327f},in.now);
    full_map_step(&m,in);assert(m.phase==MAP_PAUSED && m.command.vx==0 && m.command.vy==0);
    /* Short R4 does not exist at x=.5; do not pair it with another ridge. */
    in=setup_aisle(&m,8,.5f);
    for(unsigned now=10;now<=500;now+=10) {
        in.now=now;in.laser=ridge_scan((AutoPose){.5f,2.3f,1.570796327f},now);
        full_map_step(&m,in);assert(!m.aisle_correction.active);
    }
    puts("AISLE_CORRECTION_TEST_OK direction, freshness, gap, short-R4, near-stop");
}
static void aisle_closed_loop_test(void) {
    FullMapTest m;FullMapInput in=setup_aisle(&m,12,2.4f);
    AutoPose actual={2.4f,.935f,1.570796327f};AutoTwist measured={0};bool corrected=false;
    for(unsigned now=10;now<60000;now+=10) {
        measured.vx+=(m.command.vx-measured.vx)*.25f;
        measured.vy+=(m.command.vy-measured.vy)*.25f;
        measured.wz+=(m.command.wz-measured.wz)*.25f;
        const AutoVec2 delta=auto_rotate((AutoVec2){measured.vx*.01f,measured.vy*.01f},actual.yaw);
        actual.x+=delta.x;actual.y+=delta.y+.00005f; /* 5mm/s unmeasured north slip. */
        actual.yaw=auto_wrap_angle(actual.yaw+measured.wz*.01f);
        in.now=now;in.wheel=measured;in.imu_yaw=auto_wrap_angle(actual.yaw-1.570796327f);
        in.stationary=hypotf(measured.vx,measured.vy)<.01f && fabsf(measured.wz)<.02f;
        if(now%100==0) in.laser=ridge_scan(actual,now);
        else for(unsigned i=0;i<4;++i) in.laser.sample_age_ms[i]+=10;
        full_map_step(&m,in);corrected|=m.aisle_correction.active;
        if(m.phase==MAP_FAULT || m.phase==MAP_PAUSED) {
            fprintf(stderr,"aisle closed-loop failed %s y=%f map_y=%f\n",m.reason,actual.y,m.localization.pose.pose.y);
            assert(0);
        }
        if(m.segment==13) {
            assert(corrected && fabsf(actual.y-.9f)<.025f);
            printf("AISLE_CLOSED_LOOP_OK slip_mm_s=5 initial_mm=35 final_mm=%.1f time_s=%.2f\n",
                (actual.y-.9f)*1000,now*.001f);return;
        }
    }
    assert(0);
}
static void tree_config_and_pause_tests(void) {
    FullMapTest m;FullMapInput in=setup_aisle(&m,12,2.8f);
    assert(!full_map_set_tree_x(&m,2,1,.8f)); /* No editing an active plan. */
    m.phase=MAP_READY;m.segment=0;
    assert(!full_map_set_tree_x(&m,0,1,1));assert(!full_map_set_tree_x(&m,7,1,1));
    assert(!full_map_set_tree_x(&m,4,1,.5f));assert(!full_map_set_tree_x(&m,4,2,NAN));
    assert(!full_map_set_tree_x(&m,2,1,1.7f)); /* Left cannot cross right. */
    const float expected[6][2]={{1.23f,2.15f},{.73f,1.63f},{.68f,2.22f},{1.19f,2.10f},{1.30f,2.20f},{.665f,1.875f}};
    for(unsigned r=0;r<6;r++) for(unsigned n=0;n<2;n++)
        assert(m.trees[r][n].configured && fabsf(m.trees[r][n].x-expected[r][n])<.00001f);
    m.segment=12;
    assert(full_map_command(&m,"MAP ARM ALL",in));
    char go[32];snprintf(go,sizeof(go),"MAP GO %lu",(unsigned long)m.token);
    assert(full_map_command(&m,go,in));assert(m.target_tree==4); /* G2 is right-to-left. */
    AutoPose at_tree={1.63f,.9f,1.570796327f};
    for(unsigned now=10;now<=1400;now+=10) {
        m.localization.pose.pose=at_tree;in.now=now;in.laser=scan(at_tree,now);full_map_step(&m,in);
    }
    assert(m.tree_hold && m.trees_done==0);
    assert(full_map_command(&m,"MAP PAUSE",in));
    for(unsigned now=1410;now<=4500;now+=10){in.now=now;in.laser=scan(at_tree,now);full_map_step(&m,in);}
    assert(m.tree_hold && m.trees_done==0 && m.phase==MAP_PAUSED);
    assert(full_map_command(&m,"MAP RESUME",in));
    for(unsigned now=4510;now<=6400;now+=10){in.now=now;in.laser=scan(at_tree,now);full_map_step(&m,in);assert(m.trees_done==0);}
    for(unsigned now=6410;now<=7100;now+=10){in.now=now;in.laser=scan(at_tree,now);full_map_step(&m,in);}
    assert(m.trees_done==8u && m.target_tree==3 && !m.tree_hold);
    at_tree.x=.73f;
    for(unsigned now=7110;now<=11000 && !m.tree_hold;now+=10) {
        m.localization.pose.pose=at_tree;in.now=now;in.laser=scan(at_tree,now);full_map_step(&m,in);
    }
    assert(m.tree_hold);
    in.now+=10;in.laser=scan(at_tree,in.now);in.laser.accepted_mm[AUTO_FRONT]=99;
    full_map_step(&m,in);
    assert(m.phase==MAP_PAUSED && m.tree_hold && m.trees_done==8u && m.command.vx==0 && m.command.vy==0);
    puts("TREE_STOP_TEST_OK measured coordinates, reverse order, 2s dwell and pause reset");
}
static void speed_transition_tests(void) {
    float previous=0;
    for(unsigned sample=0;sample<2;++sample) {
        FullMapTest m;FullMapInput in=setup_aisle(&m,12,2);
        m.speed=.4f;
        const AutoPose p={2,.9f+(sample?.026f:.024f),1.570796327f};
        for(unsigned now=10;now<=30;now+=10) {
            m.localization.pose.pose=p;in.now=now;in.laser=ridge_scan(p,now);
            full_map_step(&m,in);
        }
        assert(m.aisle_correction.active && m.phase==MAP_RUNNING);
        if(sample) {
            assert(m.motion.config.vmax<previous);
            assert(previous-m.motion.config.vmax<.08f);
        }
        previous=m.motion.config.vmax;
    }
    FullMapTest m;FullMapInput in=setup_aisle(&m,2,2);
    m.speed=.6f;in.now=10;in.laser=scan(m.localization.pose.pose,10);
    full_map_step(&m,in);
    assert(m.phase==MAP_RUNNING);
    assert(m.motion.config.vmax>.3f && m.motion.config.vmax<.45f);
    /* Immediate near protection and permission loss still stop the task. */
    in.now=20;in.laser=scan(m.localization.pose.pose,20);
    in.laser.accepted_mm[AUTO_RIGHT]=99;
    full_map_step(&m,in);assert(m.phase==MAP_PAUSED && m.command.vy==0);
    in.now=30;in.healthy=false;full_map_step(&m,in);
    assert(m.phase==MAP_FAULT && m.command.vx==0 && m.command.vy==0);
    puts("MAP_SPEED_TRANSITION_OK continuous_aisle_limit anticipatory_braking safety_retained");
}
static void corner_preparation_test(void) {
    FullMapTest m;FullMapInput in=setup_aisle(&m,2,.2f);
    in.now=10;in.laser=scan(m.localization.pose.pose,10);
    in.aligned=true; /* Still reporting the previous northbound wheel target. */
    full_map_step(&m,in);
    assert(m.motion.out.phase==AUTO_MOTION_ALIGN);
    assert(m.command.vx==0 && m.command.vy==0 && m.command.wz==0);
    assert(m.hint.vy<-.01f);
    for(unsigned now=20;now<=300;now+=10) {
        in.now=now;in.aligned=false;in.laser=scan(m.localization.pose.pose,now);
        full_map_step(&m,in);
        assert(m.command.vx==0 && m.command.vy==0 && m.hint.vy<-.01f);
    }
    in.now=310;in.aligned=true;in.laser=scan(m.localization.pose.pose,310);
    full_map_step(&m,in);
    assert(m.motion.out.phase==AUTO_MOTION_RUN && m.command.vy<0);
    /* Restart stays bounded by one configured acceleration step. */
    assert(hypotf(m.command.vx,m.command.vy)<=m.motion.config.accel*.01f+.00001f);
    in.now=320;in.healthy=false;full_map_step(&m,in);
    assert(m.phase==MAP_FAULT && m.command.vx==0 && m.command.vy==0);
    puts("MAP_CORNER_PREP_OK old_alignment_not_reused gate_wait_no_drive bounded_restart");
}
static void g1_margin_recovery_test(void) {
    for(unsigned run=0;run<2;++run) {
        FullMapTest m;FullMapInput in=setup_aisle(&m,14,.425f);
        m.speed=run?.30f:.20f;
        const float yaw=(run?88.69f:89.29f)*.017453293f;
        AutoPose actual={.425f,.2f+(206.0f-(run?119.0f:116.0f))*.0005f,yaw};
        AutoTwist measured={.vy=-m.speed};bool centering_seen=false,margin_seen=false;
        m.command=measured;m.motion.velocity_map=(AutoVec2){m.speed,0};m.motion.started=true;
        in.laser=ridge_scan(actual,0);
        for(unsigned now=10;now<40000;now+=10) {
            measured.vx+=(m.command.vx-measured.vx)*.25f;
            measured.vy+=(m.command.vy-measured.vy)*.25f;
            measured.wz+=(m.command.wz-measured.wz)*.25f;
            const AutoVec2 delta=auto_rotate((AutoVec2){measured.vx*.01f,measured.vy*.01f},actual.yaw);
            actual.x+=delta.x;actual.y+=delta.y;actual.yaw+=measured.wz*.01f;
            in.now=now;in.wheel=measured;in.imu_yaw=actual.yaw-1.570796327f;
            in.stationary=hypotf(measured.vx,measured.vy)<.01f&&fabsf(measured.wz)<.02f;
            if(now%100==0) in.laser=ridge_scan(actual,now);
            else for(unsigned i=0;i<4;++i) in.laser.sample_age_ms[i]+=10;
            full_map_step(&m,in);
            centering_seen|=m.aisle_correction.centering;
            margin_seen|=!auto_route_pose_clear(&m.route,(AutoEnvelope){.155f,.155f,.155f,.155f},m.localization.pose.pose,.01f);
            if(m.phase==MAP_FAULT) {
                fprintf(stderr,"G1 recovery failed run=%u reason=%s actual=%.3f,%.3f fused=%.3f,%.3f now=%u\n",
                    run,m.reason,actual.x,actual.y,m.localization.pose.pose.x,m.localization.pose.pose.y,now);
                assert(0);
            }
            if(actual.x>1.0f && fabsf(actual.y-.2f)<.015f) {
                assert(centering_seen);
                printf("G1_MARGIN_RECOVERY_OK run=%u margin_seen=%u final_y=%.4f time_s=%.2f\n",run,margin_seen,actual.y,now*.001);
                break;
            }
            assert(now<39990);
        }
    }
}
static void margin_and_collision_test(void) {
    FullMapTest m;FullMapInput in=setup_aisle(&m,14,1);
    m.localization.pose.pose.y=.238f;
    in.now=10;in.laser=scan(m.localization.pose.pose,10);
    full_map_step(&m,in);
    assert(m.phase==MAP_RUNNING && m.along_speed_limit==.05f);
    assert(hypotf(m.command.vx,m.command.vy)<=m.motion.config.accel*.01f+.00001f);
    m.localization.pose.pose.y=.251f;
    in.now=20;in.laser=scan(m.localization.pose.pose,20);
    full_map_step(&m,in);
    assert(m.phase==MAP_FAULT && !strcmp(m.reason,"MAP_ENVELOPE"));
    assert(m.command.vx==0 && m.command.vy==0);
    in=setup_aisle(&m,14,1);
    in.now=10;in.laser=ridge_scan(m.localization.pose.pose,10);
    in.laser.accepted_mm[AUTO_FRONT]=99;
    full_map_step(&m,in);
    assert(m.phase==MAP_PAUSED && m.command.vx==0 && m.command.vy==0);
    puts("MAP_MARGIN_LAYER_OK buffer_limited actual_envelope_fault near_range_stop");
}
static void centering_hysteresis_test(void) {
    FullMapTest m;FullMapInput in=setup_aisle(&m,14,1);
    for(unsigned now=10;now<=30;now+=10) {
        m.localization.pose.pose.y=.2f;in.now=now;
        in.laser=ridge_scan((AutoPose){1,.242f,1.570796327f},now);
        full_map_step(&m,in);
        if(now<30) assert(!m.aisle_correction.centering);
    }
    assert(m.aisle_correction.active && m.aisle_correction.centering);
    m.localization.pose.pose.y=.2f;in.now=40;
    in.laser=ridge_scan((AutoPose){1,.234f,1.570796327f},40);
    full_map_step(&m,in);assert(m.aisle_correction.centering);
    m.localization.pose.pose.y=.2f;in.now=50;
    in.laser=ridge_scan((AutoPose){1,.214f,1.570796327f},50);
    full_map_step(&m,in);assert(m.aisle_correction.centering);
    for(unsigned now=60;now<=1000;now+=10) {
        m.localization.pose.pose.y=.2f;in.now=now;
        in.laser=ridge_scan((AutoPose){1,.214f,1.570796327f},now);
        full_map_step(&m,in);
    }
    assert(!m.aisle_correction.centering);
    puts("AISLE_CENTERING_HYSTERESIS_OK independent_confirmation enter_35mm exit_15mm");
}
static void tree_endpoint_centering_test(bool late_observation) {
    FullMapTest m;FullMapInput in=setup_aisle(&m,8,1.2f);
    const float target_x=m.trees[3][0].x;
    AutoSegment line=m.motion.segment;line.end.x=target_x;
    AutoMotionConfig c=m.motion.config;
    c.settle_s=.4f;
    assert(auto_motion_begin(&m.motion,&c,line,0));
    m.target_tree=7;m.trees_done|=1u<<7;m.aisle_correction.centering=true;
    if(late_observation) {m.tree_hold=true;m.motion.out.phase=AUTO_MOTION_DONE;}
    AutoPose actual={target_x,2.282f,1.570796327f};AutoTwist measured={0};
    assert(field_pose_register(&m.localization,actual,0,0));
    for(unsigned now=10;now<=10000;now+=10) {
        measured.vx+=(m.command.vx-measured.vx)*.25f;
        measured.vy+=(m.command.vy-measured.vy)*.25f;
        measured.wz+=(m.command.wz-measured.wz)*.25f;
        const AutoVec2 delta=auto_rotate((AutoVec2){measured.vx*.01f,measured.vy*.01f},actual.yaw);
        actual.x+=delta.x;actual.y+=delta.y;actual.yaw+=measured.wz*.01f;
        in.now=now;in.wheel=measured;in.imu_yaw=actual.yaw-1.570796327f;
        in.stationary=hypotf(measured.vx,measured.vy)<.01f&&fabsf(measured.wz)<.02f;
        if(now%100==0) in.laser=ridge_scan(actual,now);
        else for(unsigned i=0;i<4;++i) in.laser.sample_age_ms[i]+=10;
        full_map_step(&m,in);assert(m.phase!=MAP_FAULT && m.phase!=MAP_PAUSED);
        if(m.trees_done&(1u<<6)) {
            if(fabsf(actual.y-2.3f)>=.02f || fabsf(actual.x-target_x)>=.01f)
                fprintf(stderr,"tree centering finish actual=%.5f,%.5f target=%.5f\n",actual.x,actual.y,target_x);
            assert(fabsf(actual.y-2.3f)<.02f && fabsf(actual.x-target_x)<.01f);
            assert(!m.aisle_correction.centering);
            printf("TREE_ENDPOINT_CENTERING_OK G4_tree1 late_observation=%u initial_error_mm=18 completed_s=%.2f\n",
                (unsigned)late_observation,now*.001);
            return;
        }
    }
    assert(0);
}
static void delayed_range_braking_test(void) {
    for(unsigned run=0;run<2;++run) {
        FullMapTest m;FullMapInput in=setup_aisle(&m,run?15u:0u,run?1.5f:.2f);
        AutoPose actual={run?1.5f:.2f,run?.2f:3.1f,1.570796327f};
        AutoTwist measured=run?(AutoTwist){.vy=.4f}:(AutoTwist){.vx=.4f};
        AutoSegment line=m.route.segment[m.segment].line;
        if(!run) line.end.y=4.4f;
        AutoMotionConfig c=m.motion.config;c.vmax=.4f;
        assert(c.accel==.45f && c.decel==.60f);
        assert(auto_motion_begin(&m.motion,&c,line,0));
        assert(field_pose_register(&m.localization,actual,0,0));
        m.speed=.4f;m.command=measured;m.motion.started=true;
        m.motion.velocity_map=run?(AutoVec2){-.4f,0}:(AutoVec2){0,.4f};
        AutoPose previous=actual;
        AutoPose initial_delayed=actual;
        if(run) initial_delayed.x+=.04f;else initial_delayed.y-=.04f;
        in.laser=scan(initial_delayed,0);
        for(unsigned i=0;i<4;++i) in.laser.sample_age_ms[i]=100;
        bool finished=false;
        for(unsigned now=10;now<=45000;now+=10) {
            measured.vx+=(m.command.vx-measured.vx)*.2f;
            measured.vy+=(m.command.vy-measured.vy)*.2f;
            measured.wz+=(m.command.wz-measured.wz)*.2f;
            const AutoVec2 delta=auto_rotate((AutoVec2){measured.vx*.01f,measured.vy*.01f},actual.yaw);
            actual.x+=delta.x;actual.y+=delta.y;actual.yaw+=measured.wz*.01f;
            in.now=now;in.wheel=measured;in.imu_yaw=actual.yaw-1.570796327f;
            in.stationary=hypotf(measured.vx,measured.vy)<.01f&&fabsf(measured.wz)<.02f;
            if(now%100==0) {
                in.laser=scan(previous,now);previous=actual;
                for(unsigned i=0;i<4;++i) if(in.laser.valid_mask&(1u<<i)) in.laser.sample_age_ms[i]=100;
            } else for(unsigned i=0;i<4;++i) if(in.laser.sample_age_ms[i]!=UINT32_MAX) in.laser.sample_age_ms[i]+=10;
            full_map_step(&m,in);
            if(m.phase==MAP_PAUSED || m.phase==MAP_FAULT) {
                fprintf(stderr,"delayed braking failed run=%u reason=%s at=%u actual=%.3f,%.3f speed=%.3f\n",
                    run,m.reason,now,actual.x,actual.y,hypotf(measured.vx,measured.vy));assert(0);
            }
            if(m.segment==(run?16u:2u)) {
                finished=true;
                printf("MAP_DELAYED_BRAKING_OK run=%u speed=0.40 lag_ms=50 median_age_ms=100 time_s=%.2f\n",run,now*.001);
                break;
            }
        }
        assert(finished);
    }
}
static void filtered_aisle_observation_test(void) {
    FullMapTest m;FullMapInput in=setup_aisle(&m,12,1.5f);
    const AutoPose p={1.5f,.9f,1.570796327f};
    for(unsigned now=100;now<=600;now+=100) {
        /* Exercise the filter/centering state without advancing the scheduler
         * by more than its normal 10ms deadline between received samples. */
        for(unsigned tick=now-90;tick<=now;tick+=10) {
            in.now=tick;in.laser=ridge_scan(p,tick);
            for(unsigned ch=0;ch<4;++ch) in.laser.median_mm[ch]=in.laser.accepted_mm[ch];
            in.laser.median_valid_mask=15;
            in.laser.median_updated_mask=now<=300?15u:(now==400?4u:0u);
            in.laser.accepted_mm[AUTO_FRONT]=now%200?110u:190u;
            in.laser.accepted_mm[AUTO_REAR]=now%200?210u:130u;
            full_map_step(&m,in);
            assert(m.phase==MAP_RUNNING && !m.aisle_correction.centering);
        }
        if(now>=300) assert(m.aisle_correction.active && m.aisle_correction.confirmations==3);
    }
    assert(fabsf(m.localization.pose.pose.y-.9f)<.003f);
    in.now+=10;in.laser.accepted_mm[AUTO_FRONT]=99;
    full_map_step(&m,in);
    assert(m.phase==MAP_PAUSED && m.command.vx==0 && m.command.vy==0);
    puts("FILTERED_AISLE_OBSERVATION_OK near_jitter stable_pair partial_updates hard_near_stop");
}
static void staggered_aisle_observation_test(void) {
    FullMapTest m;FullMapInput in=setup_aisle(&m,12,1.5f);
    const AutoPose actual={1.5f,.940f,1.570796327f};
    for(unsigned now=10;now<=1200;now+=10) {
        const unsigned stamp=now/100u*100u;
        in.now=now;in.laser=ridge_scan(actual,stamp);
        for(unsigned ch=0;ch<4;++ch) {
            in.laser.median_mm[ch]=in.laser.accepted_mm[ch];
            in.laser.median_age_ms[ch]=now-stamp;
        }
        in.laser.median_valid_mask=15;
        in.laser.median_updated_mask=stamp%200u?4u:8u;
        in.laser.median_age_ms[stamp%200u?AUTO_FRONT:AUTO_REAR]+=100u;
        full_map_step(&m,in);
        assert(m.phase==MAP_RUNNING);
        if(now<100u) assert(m.aisle_correction.confirmations==0u);
        if(now>=600u) {
            assert(m.aisle_correction.active && m.aisle_correction.confirmations==3u);
            assert(m.aisle_correction.centering && m.command.vx<0);
        }
    }
    /* A selected median can be two board periods old and still fresh. */
    const uint32_t pair_stamp=m.aisle_correction.stamp_ms;
    in.now+=10;in.laser.median_age_ms[AUTO_FRONT]=250;
    ++in.laser.frame_count;in.laser.last_rx_ms=in.now;in.laser.median_updated_mask=4;
    full_map_step(&m,in);assert(m.aisle_correction.active);
    assert(m.aisle_correction.stamp_ms==pair_stamp); /* Older sources cannot reconfirm. */
    in.now+=10;in.laser.median_age_ms[AUTO_FRONT]=301;
    full_map_step(&m,in);assert(m.aisle_correction.active);
    for(unsigned tick=0;tick<21;++tick) {
        in.now+=10;full_map_step(&m,in);
    }
    assert(!m.aisle_correction.active);
    puts("STAGGERED_AISLE_OBSERVATION_OK independent_pair cached_frame median_age stale_reset");
}
static void tree_observation_continuity_test(void) {
    FullMapTest m;FullMapInput in=setup_aisle(&m,8,2.1f);
    const AutoPose actual={2.1f,2.3f,1.570796327f};
    for(unsigned now=10;now<=30;now+=10) {
        in.now=now;in.laser=ridge_scan(actual,now);full_map_step(&m,in);
    }
    assert(m.aisle_correction.active);
    m.target_tree=8;m.tree_hold=m.tree_hold_stable=true;
    m.tree_hold_ms=in.now-2000u;
    m.motion.segment.end=(AutoVec2){2.1f,2.3f};
    m.motion.out.phase=AUTO_MOTION_DONE;
    in.now+=10;in.laser=ridge_scan(actual,in.now);full_map_step(&m,in);
    assert((m.trees_done&(1u<<7)) && m.target_tree==7u && !m.tree_hold);
    assert(m.aisle_correction.active && m.aisle_correction.confirmations==3u);
    puts("TREE_OBSERVATION_CONTINUITY_OK next_tree_keeps_confirmed_pair");
}
static void intermittent_recovery_observation_test(void) {
    FullMapTest m;FullMapInput in=setup_aisle(&m,12,2.2f);
    const AutoPose actual={2.2f,.9f,1.570796327f};
    in.now=10;in.laser=ridge_scan(actual,10);in.laser.accepted_mm[AUTO_REAR]=99;
    full_map_step(&m,in);assert(m.phase==MAP_PAUSED);
    for(unsigned now=20;now<=600;now+=10) {
        const unsigned stamp=now/100u*100u;
        in.now=now;in.laser=ridge_scan(actual,stamp);
        for(unsigned ch=0;ch<4u;++ch) in.laser.sample_age_ms[ch]=now-stamp;
        /* Required left medians advance on alternate board frames. */
        in.laser.updated_mask=stamp%200u?1u:0u;
        full_map_step(&m,in);
        if(now<300u) assert(m.phase==MAP_PAUSED && m.recovery_frames<=1u);
    }
    assert(m.phase==MAP_RUNNING);
    in.now+=10;in.laser=ridge_scan(actual,in.now);in.laser.accepted_mm[AUTO_REAR]=99;
    full_map_step(&m,in);assert(m.phase==MAP_PAUSED && m.recovery_frames==0u);
    puts("INTERMITTENT_RECOVERY_OK fresh_independent_samples cached_frames near_stop_reset");
}
static void relaxed_aisle_correction_test(void) {
    FullMapTest m;FullMapInput in=setup_aisle(&m,12,1.5f);
    const AutoPose actual={1.5f,.925f,1.675516082f}; /* 25mm offset and six degrees yaw. */
    in.imu_yaw=actual.yaw-1.570796327f;
    for(unsigned now=10;now<=30;now+=10) {
        in.now=now;in.laser=ridge_scan(actual,now);full_map_step(&m,in);
    }
    assert(m.phase==MAP_RUNNING && m.aisle_correction.active && !m.aisle_correction.centering);
    assert(m.command.vy>0);
    assert(m.motion.config.smooth_correction);
    assert(fabsf(m.motion.config.finish_yaw_tolerance-.069813170f)<.000001f);
    puts("RELAXED_AISLE_CORRECTION_OK six_degree_observation 25mm_moving_correction four_degree_finish");
}
static FullMapInput setup_entry_observer(FullMapTest* m,float y) {
    FullMapInput in=setup_aisle(m,0,.184f);
    const AutoPose p={.184f,y,1.570796327f};
    assert(field_pose_register(&m->localization,p,0,0));
    AutoSegment line=m->route.segment[0].line;line.end=m->route.segment[1].line.end;
    const AutoMotionConfig c=m->motion.config;
    assert(auto_motion_begin(&m->motion,&c,line,0));
    m->speed=.40f;in.imu_yaw=.077493f;
    in.laser=(LaserSnapshot){.accepted_mm={143,210,1768,7031},
        .distance_mm={143,210,1768,7031},.valid_mask=15,.updated_mask=15,.frame_fresh=true};
    return in;
}
static void entry_observer_role_test(void) {
    FullMapTest m;FullMapInput in=setup_entry_observer(&m,1.296f);
    for(unsigned now=10;now<=1600;now+=10) {
        in.now=now;in.laser.frame_count=now;in.laser.last_rx_ms=now;
        full_map_step(&m,in);
        assert(m.phase==MAP_RUNNING && !(m.localization.used_mask&(1u<<AUTO_RIGHT)));
    }
    assert(m.localization.pose.pose.x>.18f);
    in.now+=10;in.laser.accepted_mm[AUTO_RIGHT]=99;
    full_map_step(&m,in);assert(m.phase==MAP_PAUSED);
    puts("ENTRY_OBSERVER_ROLE_OK recorded_conflicting_right_range left_reference right_near_stop");
}
static void entry_observation_bridge_test(void) {
    FullMapTest m;FullMapInput in=setup_entry_observer(&m,1.2f);
    in.now=10;in.laser.frame_count=10;in.laser.last_rx_ms=10;full_map_step(&m,in);
    m.localization.uncorrected_m[1]=1.21f;
    in.now=20;in.laser.frame_count=20;in.laser.last_rx_ms=20;full_map_step(&m,in);
    assert(m.phase==MAP_RUNNING && m.motion.config.vmax>.35f);
    in.laser.valid_mask&=(uint8_t)~1u;in.laser.updated_mask&=(uint8_t)~1u;
    for(unsigned now=30;now<=350;now+=10) {
        in.now=now;in.laser.frame_count=now;in.laser.last_rx_ms=now;full_map_step(&m,in);
    }
    assert(m.phase==MAP_PAUSED && !strcmp(m.reason,"POSE_OBSERVATION_PAUSE"));
    puts("ENTRY_OBSERVATION_BRIDGE_OK odometry_progress fresh_left_required expiry_pauses");
}
static void entry_north_confirmation_test(void) {
    FullMapTest m;FullMapInput in=setup_entry_observer(&m,4.15f);
    m.localization.constrained_ms[1]=0;in.imu_yaw=0;
    for(unsigned now=10;now<=600;now+=10) {
        in.now=now;in.laser.frame_count=now;in.laser.last_rx_ms=now;full_map_step(&m,in);
    }
    assert(m.phase==MAP_PAUSED && !strcmp(m.reason,"ENTRY_NORTH_PAUSE"));
    for(unsigned now=610;now<=1400;now+=10) {
        in.now=now;in.laser=scan(m.localization.pose.pose,now);full_map_step(&m,in);
    }
    assert(m.phase==MAP_RUNNING && m.segment==0u);
    puts("ENTRY_NORTH_CONFIRMATION_OK cannot_turn_on_unobserved_y fresh_north_resumes");
}
static void tree_dwell_observation_hysteresis_test(void) {
    FullMapTest m;FullMapInput in=setup_aisle(&m,4,.644f);
    const AutoPose p={.644f,3.712f,1.570796327f};
    assert(field_pose_register(&m.localization,p,0,0));
    m.target_tree=11;m.trees_done=1u<<11;
    m.motion.segment.end=(AutoVec2){.665f,3.7f};m.motion.out.phase=AUTO_MOTION_DONE;
    m.tree_hold=m.tree_hold_stable=true;m.tree_hold_ms=0;
    for(unsigned now=10;now<=1900;now+=10) {
        in.now=now;in.laser=ridge_scan(p,now);full_map_step(&m,in);
        assert(m.phase==MAP_RUNNING && m.tree_hold && m.target_tree==11u);
    }
    assert(m.trees_done==(1u<<11));
    puts("TREE_DWELL_HYSTERESIS_OK recorded_21mm_shift_keeps_confirmed_dwell");
}
static void tree_reposition_past_target_test(void) {
    FullMapTest m;FullMapInput in=setup_aisle(&m,4,.620f);
    const AutoPose p={.620f,3.7f,1.570796327f};
    assert(field_pose_register(&m.localization,p,0,0));
    m.target_tree=11;m.trees_done=1u<<11;m.tree_hold=true;
    m.motion.segment.end=(AutoVec2){.665f,3.7f};m.motion.out.phase=AUTO_MOTION_DONE;
    for(unsigned now=10;now<=100;now+=10) {
        in.now=now;in.laser=ridge_scan(p,now);full_map_step(&m,in);
        assert(m.phase==MAP_RUNNING);
    }
    assert(!m.tree_hold && m.target_tree==11u && m.motion.tangent.x>0 && m.command.vy<0);
    assert(m.required_mask&(1u<<AUTO_RIGHT));
    in.now+=10;in.laser=ridge_scan(p,in.now);in.laser.accepted_mm[AUTO_RIGHT]=99;
    full_map_step(&m,in);assert(m.phase==MAP_PAUSED && m.command.vy==0);
    puts("TREE_REPOSITION_PAST_TARGET_OK 45mm_local_reverse actual_direction_range_guard");
}
static void entry_missing_north_closed_loop_test(void) {
    FullMapTest m;full_map_init(&m,0);
    AutoPose actual={.2f,-.5f,1.570796327f};AutoTwist measured={0};
    FullMapInput in={.healthy=true,.odom_ok=true,.aligned=true,.stationary=true,.laser=scan(actual,0)};
    assert(full_map_command(&m,"MAP ORIGIN",in));assert(full_map_command(&m,"MAP SPEED 0.40",in));
    assert(full_map_command(&m,"MAP ARM ALL",in));assert(full_map_command(&m,"MAP GO 1",in));
    bool bridged=false;
    for(unsigned now=10;now<=60000;now+=10) {
        measured.vx+=(m.command.vx-measured.vx)*.25f;
        measured.vy+=(m.command.vy-measured.vy)*.25f;
        measured.wz+=(m.command.wz+.01f-measured.wz)*.25f;
        const AutoVec2 delta=auto_rotate((AutoVec2){measured.vx*.01f,measured.vy*.01f},actual.yaw);
        actual.x+=delta.x;actual.y+=delta.y;actual.yaw+=measured.wz*.01f;
        in.now=now;in.wheel=measured;in.wheel.vx*=1.032f;in.wheel.vy*=1.032f;
        in.imu_yaw=actual.yaw-1.570796327f;
        in.stationary=hypotf(measured.vx,measured.vy)<.01f && fabsf(measured.wz)<.02f;
        if(now%100u==0u) {
            in.laser=scan(actual,now);
            if(actual.y>.3f && actual.y<2.2f) {
                in.laser.accepted_mm[AUTO_FRONT]=7031;in.laser.accepted_mm[AUTO_RIGHT]=210;
                in.laser.valid_mask|=10u;in.laser.updated_mask|=10u;
            }
        } else for(unsigned ch=0;ch<4u;++ch) if(in.laser.sample_age_ms[ch]!=UINT32_MAX) in.laser.sample_age_ms[ch]+=10;
        full_map_step(&m,in);
        bridged|=m.localization.uncorrected_m[1]>1.2f;
        if(m.phase==MAP_PAUSED || m.phase==MAP_FAULT) {
            fprintf(stderr,"entry closed-loop failed %s actual=%.3f,%.3f map=%.3f,%.3f time=%u\n",
                m.reason,actual.x,actual.y,m.localization.pose.pose.x,m.localization.pose.pose.y,now);
            assert(0);
        }
        if(m.segment==2u) {
            assert(bridged && fabsf(actual.x-.2f)<.03f && fabsf(actual.y-4.4f)<.03f);
            assert(now-m.localization.constrained_ms[1]<=500u);
            printf("ENTRY_MISSING_NORTH_CLOSED_LOOP_OK time_s=%.2f final_y=%.4f wheel_scale=1.032 gyro_disturbance=.01 no_pauses\n",now*.001,actual.y);
            return;
        }
    }
    assert(0);
}
static void ridge_localization_tests(void) {
    AutoRoute route;assert(auto_route_build(&route,1,1.570796327f));
    const AutoPose p={1.5f,.92f,1.570796327f};
    FieldPose fence,ridges;
    assert(field_pose_register(&fence,p,0,0));ridges=fence;
    LaserSnapshot l=ridge_scan(p,10);
    assert(field_pose_step(&fence,(AutoTwist){0},0,true,&l,10));
    assert(!(fence.used_mask&(1u<<AUTO_FRONT)));
    assert(field_pose_step_route(&ridges,(AutoTwist){0},0,true,&l,10,&route));
    assert(ridges.used_mask&(1u<<AUTO_FRONT));
    assert(ridges.constrained_axes&2u);
    const float y=ridges.pose.pose.y;
    assert(field_pose_step_route(&ridges,(AutoTwist){0},0,true,&l,20,&route));
    assert(ridges.pose.pose.y==y && ridges.used_mask==0);
    l=ridge_scan(p,30);l.accepted_mm[AUTO_FRONT]=1979;
    assert(field_pose_step_route(&ridges,(AutoTwist){0},0,true,&l,30,&route));
    assert(ridges.rejected_mask&(1u<<AUTO_FRONT));
    /* At the entry, the observed 2.2m return has no matching mapped face.
     * It must not turn into a fabricated longitudinal position correction. */
    assert(field_pose_register(&ridges,(AutoPose){.188f,.225f,1.6015f},0,0));
    l=scan(ridges.pose.pose,10);l.accepted_mm[AUTO_FRONT]=2230;
    assert(field_pose_step_route(&ridges,(AutoTwist){0},0,true,&l,10,&route));
    assert(ridges.rejected_mask&(1u<<AUTO_FRONT));
    puts("RIDGE_LOCALIZATION_OK known_faces repeated_stamp unknown_entry_return_rejected");
}
static void station_work_gate_test(void) {
    FullMapTest map;
    FullMapInput input = setup_aisle(&map, 4, .665f);
    const AutoPose pose = {.665f, 3.7f, 1.570796327f};
    unsigned now;
    assert(field_pose_register(&map.localization, pose, 0, 0));
    map.target_tree = 11;
    map.trees_done = 1u << 11;
    map.motion.segment.end = (AutoVec2){.665f, 3.7f};
    map.motion.out.phase = AUTO_MOTION_DONE;
    map.tree_hold = map.tree_hold_stable = true;
    input.work_enabled = input.work_online = true;
    for(now=10; now<=3000; now+=10) {
        input.now = now;
        input.laser = ridge_scan(pose, now);
        full_map_step(&map, input);
        assert(map.phase == MAP_RUNNING && map.tree_hold);
        assert(map.trees_done == (1u << 11));
        assert(map.command.vx == 0 && map.command.vy == 0);
    }
    input.work_wait = input.work_active = true;
    input.work_done = true;
    input.work_station = 12; /* Completion for a different tree cannot release. */
    input.now += 10;
    input.laser = ridge_scan(pose, input.now);
    full_map_step(&map, input);
    assert(map.tree_hold && map.trees_done == (1u << 11));
    input.work_station = 11;
    input.work_done = false;
    map.localization.pose.pose.x += .05f;
    input.now += 10;
    input.laser = ridge_scan(map.localization.pose.pose, input.now);
    full_map_step(&map, input);
    assert(map.phase == MAP_FAULT && !strcmp(map.reason, "WORK_STATION_MOVED"));
    assert(map.command.vx == 0 && map.command.vy == 0);
    puts("STATION_MAP_GATE_OK no_timer_release wrong_tree_done active_tool_blocks_reposition");
}

static void tree_overshoot_recovery_test(void) {
    FullMapTest m;FullMapInput in=setup_aisle(&m,4,.644f);
    AutoPose actual={.644f,3.7f,1.570796327f};AutoTwist velocity={0};
    const AutoMotionConfig c=m.motion.config;
    const AutoSegment line={{1.875f,3.7f},{.665f,3.7f},1.570796327f};
    assert(auto_motion_begin(&m.motion,&c,line,0));
    m.target_tree=11;m.trees_done=1u<<11;
    FullMapTest blocked=m;FullMapInput invalid=in;
    invalid.now=10;invalid.laser=ridge_scan(actual,10);
    invalid.laser.valid_mask&=(uint8_t)~(1u<<AUTO_RIGHT);
    invalid.laser.accepted_mm[AUTO_RIGHT]=65535;
    full_map_step(&blocked,invalid);
    assert(blocked.phase==MAP_PAUSED && (blocked.required_mask&(1u<<AUTO_RIGHT)));
    bool returning=false,done=false;
    for(unsigned now=10;now<=12000;now+=10) {
        in.now=now;in.wheel=velocity;in.stationary=hypotf(velocity.vx,velocity.vy)<.005f;
        in.laser=ridge_scan(actual,now);
        full_map_step(&m,in);
        assert(m.phase==MAP_RUNNING);
        if(m.command.vy<-.0001f) {
            returning=true;assert(m.required_mask&(1u<<AUTO_RIGHT));
            assert(hypotf(m.command.vx,m.command.vy)<=.05001f);
        }
        velocity.vx+=(m.command.vx-velocity.vx)*.25f;
        velocity.vy+=(m.command.vy-velocity.vy)*.25f;
        const AutoVec2 world=auto_rotate((AutoVec2){velocity.vx,velocity.vy},actual.yaw);
        actual.x+=world.x*.01f;actual.y+=world.y*.01f;
        if(m.trees_done&(1u<<10)) {done=true;break;}
    }
    assert(returning && done && fabsf(actual.x-.665f)<=.022f);
    puts("G6_TREE_OVERSHOOT_RECOVERY_OK 21mm_return_reverse_laser_required_dwell_continues");
}
static void continuous_aisle_correction_test(void) {
    FullMapTest m;FullMapInput in=setup_aisle(&m,8,2.3f);
    AutoPose actual={2.3f,2.272f,1.570796327f};AutoTwist velocity={0};
    float minimum_rear=1000;
    for(unsigned now=10;now<=3000;now+=10) {
        const float disturbance=m.motion.started?-.01f:0;
        in.now=now;in.wheel=velocity;in.wheel.vx+=disturbance;
        in.stationary=hypotf(in.wheel.vx,in.wheel.vy)<.005f;
        in.laser=ridge_scan(actual,now);
        in.laser.accepted_mm[AUTO_FRONT]-=23;in.laser.accepted_mm[AUTO_REAR]-=23;
        in.laser.distance_mm[AUTO_FRONT]=in.laser.accepted_mm[AUTO_FRONT];
        in.laser.distance_mm[AUTO_REAR]=in.laser.accepted_mm[AUTO_REAR];
        if(in.laser.accepted_mm[AUTO_REAR]<minimum_rear) minimum_rear=in.laser.accepted_mm[AUTO_REAR];
        full_map_step(&m,in);
        assert(m.phase==MAP_RUNNING && !m.aisle_correction.centering);
        velocity.vx+=(m.command.vx-velocity.vx)*.25f;
        velocity.vy+=(m.command.vy-velocity.vy)*.25f;
        const AutoVec2 world=auto_rotate((AutoVec2){velocity.vx+disturbance,velocity.vy},actual.yaw);
        actual.x+=world.x*.01f;actual.y+=world.y*.01f;
    }
    assert(minimum_rear>100 && fabsf(actual.y-2.3f)<.025f);
    FullMapTest gap;FullMapInput g=setup_aisle(&gap,12,1.5f);
    const AutoPose p={1.5f,.925f,1.570796327f};
    for(unsigned now=10;now<=400;now+=10) {
        g.now=now;g.laser=ridge_scan(p,now);full_map_step(&gap,g);
    }
    assert(gap.aisle_correction.active && gap.command.vx<0);
    const uint32_t stamp=gap.aisle_correction.stamp_ms;
    for(unsigned now=410;now<=590;now+=10) {
        g.now=now;g.laser=ridge_scan(p,now);g.laser.valid_mask&=(uint8_t)~(1u<<AUTO_REAR);
        full_map_step(&gap,g);
        assert(gap.phase==MAP_RUNNING && gap.aisle_correction.active && gap.command.vx<0);
        assert(gap.aisle_correction.stamp_ms==stamp);
    }
    g.now=610;g.laser=ridge_scan(p,610);g.laser.valid_mask&=(uint8_t)~(1u<<AUTO_REAR);
    full_map_step(&gap,g);assert(!gap.aisle_correction.active);
    puts("CONTINUOUS_AISLE_CORRECTION_OK G4_narrow_pair_drift_corrects_before_100mm_gap_hold_200ms");
}
static void entry_speed_continuity_test(void) {
    float speeds[2];
    for(unsigned sample=0;sample<2;++sample) {
        FullMapTest m;FullMapInput in=setup_entry_observer(&m,1.2f);
        in.imu_yaw=0;m.localization.uncorrected_m[1]=sample?.41f:.39f;
        in.now=10;in.laser.frame_count=10;in.laser.last_rx_ms=10;
        full_map_step(&m,in);assert(m.phase==MAP_RUNNING);
        speeds[sample]=m.motion.config.vmax;
    }
    assert(speeds[0]>.35f && speeds[1]>.35f && fabsf(speeds[0]-speeds[1])<.02f);
    FullMapTest m;FullMapInput in=setup_aisle(&m,12,1.5f);
    full_map_stop(&m,"MOTION_FAULT",false);
    assert(full_map_command(&m,"MAP SPEED 0.50",in) && m.speed==.50f);
    assert(m.phase==MAP_FAULT && !strcmp(m.reason,"MOTION_FAULT"));
    assert(!full_map_command(&m,"MAP ARM ALL",in));
    puts("ENTRY_SPEED_CONTINUITY_OK smooth_04m_boundary_fault_speed_setting_preserves_fault");
}
static void moving_aisle_centering_test(void) {
    for(int sign=-1;sign<=1;sign+=2) {
        FullMapTest m;FullMapInput in=setup_aisle(&m,12,1.5f);
        const AutoPose measured={1.5f,.9f+sign*.04f,1.570796327f};
        for(unsigned now=10;now<=1600;now+=10) {
            in.now=now;in.laser=ridge_scan(measured,now);full_map_step(&m,in);
            assert(m.phase==MAP_RUNNING);
        }
        assert(m.aisle_correction.active && m.aisle_correction.centering);
        const AutoVec2 v=auto_rotate((AutoVec2){m.command.vx,m.command.vy},measured.yaw);
        assert(v.x<-.005f && sign*v.y<-.005f);
        assert(!strcmp(m.reason,"AISLE_CORRECTING"));
    }
    puts("AISLE_MOVING_CENTERING_OK 40mm_both_sides_keeps_along_motion_before_endpoint");
}
static void recorded_entry_pause_cause_test(void) {
    const float speed[2]={.580f,.523f};
    const uint16_t ranges[2][4]={{148,197,6178,1245},{146,403,6847,1065}};
    for(unsigned run=0;run<2;++run) {
        FullMapTest m;FullMapInput in=setup_entry_observer(&m,run?2.5f:1.8f);
        assert(field_pose_register(&m.localization,(AutoPose){.2f,run?2.5f:1.8f,1.570796327f},0,0));
        in.now=10;in.stationary=false;in.wheel=(AutoTwist){.vx=speed[run],.vy=-.02f};
        in.laser=scan(m.localization.pose.pose,10);
        for(unsigned ch=0;ch<4;++ch) in.laser.accepted_mm[ch]=ranges[run][ch];
        in.laser.valid_mask=in.laser.updated_mask=15u;
        full_map_step(&m,in);
        if(m.phase!=MAP_RUNNING) fprintf(stderr,"ENTRY_REPLAY run=%u reason=%s near=%x brake=%x stale=%x odom=%x pose=%.3f,%.3f\n",
            run,m.reason,m.range_near_mask,m.range_brake_mask,m.range_stale_mask,m.odom_range_mask,m.localization.pose.pose.x,m.localization.pose.pose.y);
        assert(m.phase==MAP_RUNNING);
        assert(m.range_near_mask==0 && m.range_stale_mask==0 && m.range_brake_mask==0);
        assert(m.odom_range_mask&(1u<<AUTO_FRONT));
        assert(m.range_budget_m[AUTO_FRONT]>ranges[run][AUTO_FRONT]*.001f);
    }
    puts("RECORDED_ENTRY_PAUSE_CAUSE_OK registered_entry_clearance_rejects_far_front_braking_jump");
}
static void entry_fresh_jump_fusion_test(float speed) {
    FullMapTest m;FullMapInput in=setup_entry_observer(&m,1.0f);
    in.imu_yaw=0;in.wheel=(AutoTwist){.vx=speed};in.stationary=false;
    m.speed=speed;m.motion.started=true;m.motion.velocity_map=(AutoVec2){0,speed};
    m.motion.out.phase=AUTO_MOTION_RUN;m.command=(AutoTwist){.vx=speed};
    for(unsigned now=10;now<=190;now+=10) {
        in.now=now;in.laser=scan(m.localization.pose.pose,now);
        in.laser.accepted_mm[AUTO_FRONT]=(uint16_t)(6795-speed*now);
        full_map_step(&m,in);assert(m.phase==MAP_RUNNING);
    }
    const uint32_t stamp=m.laser_motion.channel[AUTO_FRONT].anchor.stamp;
    const uint16_t jump=speed<.6f?1065u:2767u;
    in.now=200;in.laser=scan(m.localization.pose.pose,200);in.laser.accepted_mm[AUTO_FRONT]=jump;
    full_map_step(&m,in);
    assert(m.phase==MAP_RUNNING && m.command.vx>0);
    assert((m.laser_rejected_mask&8u) && (m.predicted_mask&8u));
    assert(m.range_control_mm[AUTO_FRONT]>3200u && m.range_control_mm[AUTO_FRONT]<3600u && !(m.range_brake_mask&8u));
    assert(m.laser_motion.channel[AUTO_FRONT].anchor.stamp==stamp);
    in.now=210;in.laser=scan(m.localization.pose.pose,210);in.laser.accepted_mm[AUTO_FRONT]=99;
    full_map_step(&m,in);assert(m.phase==MAP_PAUSED && (m.range_near_mask&8u));
    printf("ENTRY_FRESH_JUMP_FUSION_OK speed=%.2f recorded_6795_to_%u no_zero_stop urgent_99mm_still_pauses\n",speed,jump);
}
static void entry_single_wall_policy_test(void) {
    FullMapTest m;FullMapInput in=setup_entry_observer(&m,1.2f);
    in.imu_yaw=0;m.speed=.50f;m.motion.started=true;
    m.motion.velocity_map=(AutoVec2){.015f,.50f};
    m.motion.out.phase=AUTO_MOTION_RUN;m.command=(AutoTwist){.50f,-.015f,0};
    in.stationary=false;in.wheel=m.command;
    const uint16_t right[4]={2775,198,6228,399};
    for(unsigned now=10;now<=800;now+=10) {
        in.now=now;in.laser=scan(m.localization.pose.pose,now);
        /* Recorded entry right-ray target changes; far forward returns also
         * disagree with the registered fence. Neither is a lateral reference. */
        in.laser.accepted_mm[AUTO_RIGHT]=right[(now/100u)%4u];
        in.laser.accepted_mm[AUTO_FRONT]=(now/100u)%2u?4300u:6500u;
        full_map_step(&m,in);
        assert(m.phase==MAP_RUNNING);
        assert(!(m.required_mask&(1u<<AUTO_RIGHT)));
        assert(!(m.localization.used_mask&((1u<<AUTO_RIGHT)|(1u<<AUTO_FRONT))));
        assert(m.command.vx>.05f);
    }
    in.now+=10;in.laser=scan(m.localization.pose.pose,in.now);
    in.laser.accepted_mm[AUTO_FRONT]=150; /* Real close front obstacle still wins. */
    full_map_step(&m,in);assert(m.phase==MAP_PAUSED);
    puts("ENTRY_SINGLE_WALL_POLICY_OK right_target_changes_unused far_front_odom close_front_stop");
}
static void low_speed_wall_countersteer_test(void) {
    for(unsigned along=0;along<2u;++along) {
        FieldPose f;AutoPose actual={.2f,1.0f,1.570796327f};
        assert(field_pose_register(&f,actual,0,0));
        for(unsigned now=10;now<=2000;now+=10) {
            const float forward=along?.07f:0;
            actual.y+=forward*.01f;
            const LaserSnapshot l=scan(actual,now);
            assert(field_pose_step(&f,(AutoTwist){.vx=forward,.vy=-.02f},0,true,&l,now));
        }
        if(along) {
            assert(f.used_mask&(1u<<AUTO_LEFT));
            assert(fabsf(f.pose.pose.x-.2f)<.005f);
        } else assert(f.rejected_mask&(1u<<AUTO_LEFT));
    }
    puts("WALL_COUNTERSTEER_OK slow_tangential_motion_accepts_constant_wall_pure_sideways_still_frozen");
}
static void entry_near_fence_source_test(void) {
    FullMapTest m;FullMapInput in=setup_entry_observer(&m,2.5f);in.imu_yaw=0;
    for(unsigned now=10;now<=400;now+=10) {
        in.now=now;in.laser=scan(m.localization.pose.pose,now);full_map_step(&m,in);
        assert(m.phase==MAP_RUNNING && !m.entry.approaching);
        assert(!(m.localization.used_mask&(1u<<AUTO_FRONT)));
    }
    assert(field_pose_register(&m.localization,(AutoPose){.184f,3.2f,1.570796327f},0,400));
    /* Start the separate near-zone fixture with its own temporal range track;
     * the preceding far-zone fixture did not physically travel this 0.7m. */
    m.laser_motion=(LaserMotionFilter){0};
    for(unsigned now=410;now<=700;now+=10) {
        in.now=now;in.laser=scan(m.localization.pose.pose,now);
        /* Three independent 10Hz source samples, not 30 cached control ticks. */
        const unsigned source=400u+(now-400u)/100u*100u;
        in.laser.frame_count=source;in.laser.last_rx_ms=source;
        for(unsigned ch=0;ch<4u;++ch) if(in.laser.sample_age_ms[ch]!=UINT32_MAX) in.laser.sample_age_ms[ch]=now-source;
        full_map_step(&m,in);
        assert(m.entry.approaching);
        if(now<600u) assert(!m.entry.north_confirmed);
    }
    assert(m.entry.north_confirmed && m.localization.constrained_ms[1]>=600u);
    puts("ENTRY_NEAR_FENCE_SOURCE_OK 1p5m_latched_three_independent_samples_no_far_front_localization");
}
static void shift_far_return_change_test(void) {
    FullMapTest m;FullMapInput in=setup_aisle(&m,3,2.79f);
    m.speed=.5f;m.motion.started=true;m.motion.out.phase=AUTO_MOTION_RUN;
    m.motion.velocity_map=(AutoVec2){0,-.3f};m.command=(AutoTwist){-.3f,0,0};
    in.stationary=false;in.wheel=m.command;
    bool inferred=false;
    for(unsigned now=10;now<=1200;now+=10) {
        in.now=now;in.laser=scan(m.localization.pose.pose,now);
        if(now>=100u) in.laser.accepted_mm[AUTO_REAR]=(now/100u)%2u?2971u:6711u;
        full_map_step(&m,in);
        assert(m.phase==MAP_RUNNING && m.command.vx<-.01f);
        inferred|=(m.odom_range_mask&(1u<<AUTO_REAR))!=0u;
    }
    assert(inferred && !(m.localization.used_mask&(1u<<AUTO_REAR)));
    in.now+=10;in.laser=scan(m.localization.pose.pose,in.now);in.laser.accepted_mm[AUTO_REAR]=150;
    full_map_step(&m,in);assert(m.phase==MAP_PAUSED && !m.odom_range_mask);
    puts("SHIFT_FAR_RETURN_CHANGE_OK known_corridor_live_far_return_fresh_side_required_real_150mm_stop");
}
static void entry_real_constraint_gap_test(void) {
    FullMapTest m;FullMapInput in=setup_entry_observer(&m,1.2f);in.imu_yaw=0;
    for(unsigned now=10;now<=400;now+=10) {
        in.now=now;in.laser=scan(m.localization.pose.pose,now);
        full_map_step(&m,in);assert(m.phase==MAP_RUNNING);
    }
    m.localization.uncorrected_m[1]=1.5f;
    const uint32_t constrained=m.localization.constrained_ms[0];
    for(unsigned now=410;now<=700;now+=10) {
        in.now=now;in.laser=scan(m.localization.pose.pose,now);
        in.laser.valid_mask&=(uint8_t)~1u;in.laser.updated_mask&=(uint8_t)~1u;
        full_map_step(&m,in);
        /* A current rejected sample is not the expiry of an earlier real fix.
         * No gap tick may renew its time or claim a new lateral observation. */
        assert(m.phase==MAP_RUNNING);
        assert(m.localization.constrained_ms[0]==constrained);
        assert(!(m.localization.used_mask&1u));
    }
    in.now=710;in.laser=scan(m.localization.pose.pose,in.now);
    in.laser.valid_mask&=(uint8_t)~1u;in.laser.updated_mask&=(uint8_t)~1u;
    full_map_step(&m,in);
    assert(m.phase==MAP_PAUSED && !strcmp(m.reason,"POSE_OBSERVATION_PAUSE"));
    puts("ENTRY_REAL_CONSTRAINT_GAP_OK current_mask_gap_does_not_revoke_real_fix_300ms_expiry_unchanged");
}
static void entry_raw_wall_source_test(float speed) {
    FullMapTest m;FullMapInput in=setup_entry_observer(&m,.3f);in.imu_yaw=0;
    m.speed=speed;m.localization.uncorrected_m[1]=1.4f;
    in.stationary=false;in.wheel=(AutoTwist){.vx=speed};
    for(unsigned now=10;now<=1200;now+=10) {
        const unsigned source=now/110u*110u,median_source=source>=220u?source-220u:0u;
        in.now=now;in.laser=scan(m.localization.pose.pose,source);
        in.laser.frame_count=source+1u;in.laser.last_rx_ms=source;
        for(unsigned ch=0;ch<4u;++ch) if(in.laser.sample_age_ms[ch]!=UINT32_MAX)
            in.laser.sample_age_ms[ch]=now-source;
        in.laser.median_mm[AUTO_LEFT]=in.laser.accepted_mm[AUTO_LEFT];
        in.laser.median_valid_mask=1u;in.laser.median_updated_mask=1u;
        in.laser.median_age_ms[AUTO_LEFT]=now-median_source;
        full_map_step(&m,in);
        assert(m.phase==MAP_RUNNING);
        assert(in.now-m.entry.left_sample_ms<=110u);
    }
    in.now+=10;in.laser=scan(m.localization.pose.pose,in.now);in.laser.accepted_mm[AUTO_LEFT]=99;
    full_map_step(&m,in);assert(m.phase==MAP_PAUSED && (m.range_near_mask&1u));
    printf("ENTRY_RAW_WALL_SOURCE_OK speed=%.2f raw_110ms_median_220ms_no_false_expiry_real_99mm_stop\n",speed);
}
static void opposite_wall_handover_test(void) {
    FullMapTest m;FullMapInput in=setup_aisle(&m,2,.25f);m.speed=.70f;
    bool bridged=false;
    for(unsigned now=10;now<=900;now+=10) {
        in.now=now;in.wheel=m.command;in.stationary=hypotf(in.wheel.vx,in.wheel.vy)<.01f;
        in.laser=scan(m.localization.pose.pose,now);
        if(now>=200u) in.laser.accepted_mm[AUTO_RIGHT]=(now/100u)%2u?3533:6228;
        full_map_step(&m,in);assert(m.phase==MAP_RUNNING);
        bridged|=(m.odom_range_mask&2u)!=0;
        if(m.odom_range_mask&2u) {
            assert(!(m.localization.used_mask&2u));
            assert(m.along_speed_limit>0 && m.along_speed_limit<=.15f);
            assert(m.laser_motion.channel[AUTO_RIGHT].anchor.stamp<200u);
        }
    }
    assert(bridged);
    in.now+=10;in.laser=scan(m.localization.pose.pose,in.now);in.laser.accepted_mm[AUTO_RIGHT]=150;
    full_map_step(&m,in);assert(m.phase==MAP_PAUSED);
    puts("OPPOSITE_WALL_HANDOVER_OK fresh_near_opposite_bounds_far_primary_real_150mm_stops");
}
static void tree_observer_settling_test(void) {
    FullMapTest m;FullMapInput in=setup_aisle(&m,6,2.2f);in.imu_yaw=0;
    m.target_tree=10;m.trees_done=0xd00;
    AutoPose actual=m.localization.pose.pose;actual.x=2.235f;
    AutoSegment line=m.motion.segment;line.start.x=1.3f;line.end.x=2.2f;
    AutoMotionConfig c=m.motion.config;assert(auto_motion_begin(&m.motion,&c,line,0));
    AutoTwist velocity={0};unsigned holds=0,previous_hold=0;
    for(unsigned now=10;now<=10000;now+=10) {
        velocity.vx+=(m.command.vx-velocity.vx)*.25f;velocity.vy+=(m.command.vy-velocity.vy)*.25f;
        velocity.wz+=(m.command.wz-velocity.wz)*.25f;
        AutoVec2 d=auto_rotate((AutoVec2){velocity.vx*.01f,velocity.vy*.01f},actual.yaw);
        actual.x+=d.x;actual.y+=d.y;actual.yaw=auto_wrap_angle(actual.yaw+velocity.wz*.01f);
        in.now=now;in.wheel=velocity;in.imu_yaw=auto_wrap_angle(actual.yaw-1.570796327f);
        in.stationary=hypotf(velocity.vx,velocity.vy)<.01f && fabsf(velocity.wz)<.02f;
        if(now%100u==0) in.laser=ridge_scan(actual,now);
        else for(unsigned ch=0;ch<4u;++ch) if(in.laser.sample_age_ms[ch]!=UINT32_MAX) in.laser.sample_age_ms[ch]+=10u;
        full_map_step(&m,in);assert(m.phase==MAP_RUNNING);
        if(m.tree_hold && !previous_hold) ++holds;
        previous_hold=m.tree_hold;
        if(m.trees_done&0x200u) {
            printf("TREE_OBSERVER_METRICS holds=%u elapsed_ms=%u actual_x=%.4f\n",holds,now,actual.x);
            assert(holds==1u && fabsf(actual.x-2.2f)<=.021f);
            printf("TREE_OBSERVER_SETTLING_OK holds=%u elapsed_ms=%u actual_x=%.4f\n",holds,now,actual.x);return;
        }
    }
    fprintf(stderr,"TREE_SETTLE_FAILED actual=%.5f fused=%.5f cmd=%.5f,%.5f phase=%u settled=%.3f holds=%u\n",
        actual.x,m.localization.pose.pose.x,m.command.vx,m.command.vy,
        m.motion.out.phase,m.motion.settled_s,holds);
    assert(0);
}
static void far_handover_boundaries_test(void) {
    for(unsigned trial=0;trial<5u;++trial) {
        const unsigned segment=trial==1u?6u:trial==2u?8u:2u;
        FullMapTest m;FullMapInput in=setup_aisle(&m,segment,segment==8u?2.8f:.25f);m.speed=.70f;
        const unsigned main_ray=segment==8u?AUTO_LEFT:AUTO_RIGHT,other=main_ray==AUTO_LEFT?AUTO_RIGHT:AUTO_LEFT;
        bool seen=false,paused=false;
        for(unsigned now=10;now<=2600u;now+=10) {
            in.now=now;in.wheel=m.command;in.stationary=hypotf(in.wheel.vx,in.wheel.vy)<.01f;
            in.laser=ridge_scan(m.localization.pose.pose,now);
            if(now>=200u) {
                in.laser.accepted_mm[main_ray]=(now/100u)%2u?3533:6228;
                if(trial==3u) in.laser.valid_mask&=(uint8_t)~(1u<<other);
                if(trial==4u) in.laser.frame_fresh=false;
            }
            full_map_step(&m,in);
            seen|=(m.odom_range_mask&(1u<<main_ray))!=0;
            if(m.phase==MAP_PAUSED) {paused=true;break;}
            assert(m.phase==MAP_RUNNING);
        }
        assert(paused); /* Never extend an untrusted primary indefinitely. */
        if(trial<3u) assert(seen); else assert(!seen);
    }
    puts("FAR_HANDOVER_BOUNDARIES_OK G7_G5_G4_left_and_right_finite_budget_missing_opposite_and_board_never_bridge");
}
int main(void) {
    entry_raw_wall_source_test(.5f);entry_raw_wall_source_test(.6f);entry_raw_wall_source_test(.7f);
    opposite_wall_handover_test();
    far_handover_boundaries_test();
    tree_observer_settling_test();
    entry_real_constraint_gap_test();
    low_speed_wall_countersteer_test();
    shift_far_return_change_test();
    entry_near_fence_source_test();
    entry_single_wall_policy_test();
    entry_fresh_jump_fusion_test(.5f);entry_fresh_jump_fusion_test(.7f);
    moving_aisle_centering_test();
    recorded_entry_pause_cause_test();
    tree_overshoot_recovery_test();
    continuous_aisle_correction_test();
    entry_speed_continuity_test();
    entry_observer_role_test();
    entry_observation_bridge_test();
    entry_north_confirmation_test();
    tree_dwell_observation_hysteresis_test();
    tree_reposition_past_target_test();
    entry_missing_north_closed_loop_test();
    relaxed_aisle_correction_test();
    intermittent_recovery_observation_test();
    staggered_aisle_observation_test();
    tree_observation_continuity_test();
    filtered_aisle_observation_test();
    delayed_range_braking_test();
    tree_endpoint_centering_test(false);tree_endpoint_centering_test(true);
    centering_hysteresis_test();
    margin_and_collision_test();
    g1_margin_recovery_test();
    corner_preparation_test();
    ridge_localization_tests();
    AutoRoute route;assert(auto_route_build(&route,1,1.570796327f));
    for(unsigned i=0;i<AUTO_ROUTE_COUNT;i++) {
        const AutoRouteSegment s=route.segment[i];
        if(s.aisle==7) assert(s.row==0);
        if(s.aisle>=1&&s.aisle<=6) assert(s.row==s.aisle);
    }
    station_work_gate_test();
    localization_tests();command_tests();protection_tests();range_threshold_tests();endpoint_acceptance_tests();
    g7_ridge_exit_test();aisle_correction_tests();
    aisle_closed_loop_test();
    tree_config_and_pause_tests();
    speed_transition_tests();
    run_empty_field(true,false);run_empty_field(true,true);run_empty_field(false,false);
    puts("FULL_MAP_TEST_OK simulation_only_no_hardware_or_work_actions");return 0;
}
