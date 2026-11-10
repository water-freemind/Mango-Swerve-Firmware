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
    if(!all) assert(full_map_command(&m,"MAP SPEED 0.60",in));
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
    assert(fabsf(m.speed-.20f)<.00001f);
    assert(!full_map_command(&m,"MAP ARM NEXT",in));
    assert(!full_map_command(&m,"MAP SPEED 0.10",in));
    const char* presets[]={"0.20","0.30","0.40","0.50","0.60"};
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
        in.laser.sample_age_ms[2]=in.laser.sample_age_ms[3]=201;
        in.now+=10;full_map_step(&m,in);assert(!m.aisle_correction.active);
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
    const float expected[6][2]={{1.24f,2.16f},{.74f,1.64f},{.69f,2.23f},{1.20f,2.11f},{1.31f,2.21f},{.675f,1.885f}};
    for(unsigned r=0;r<6;r++) for(unsigned n=0;n<2;n++)
        assert(m.trees[r][n].configured && fabsf(m.trees[r][n].x-expected[r][n])<.00001f);
    m.segment=12;
    assert(full_map_command(&m,"MAP ARM ALL",in));
    char go[32];snprintf(go,sizeof(go),"MAP GO %lu",(unsigned long)m.token);
    assert(full_map_command(&m,go,in));assert(m.target_tree==4); /* G2 is right-to-left. */
    AutoPose at_tree={1.64f,.9f,1.570796327f};
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
    at_tree.x=.74f;
    for(unsigned now=7110;now<=11000 && !m.tree_hold;now+=10) {
        m.localization.pose.pose=at_tree;in.now=now;in.laser=scan(at_tree,now);full_map_step(&m,in);
    }
    assert(m.tree_hold);
    in.now+=10;in.laser=scan(at_tree,in.now);in.laser.accepted_mm[AUTO_FRONT]=99;
    full_map_step(&m,in);
    assert(m.phase==MAP_PAUSED && m.tree_hold && m.trees_done==8u && m.command.vx==0 && m.command.vy==0);
    puts("TREE_STOP_TEST_OK measured coordinates, reverse order, 2s dwell and pause reset");
}
int main(void) {
    AutoRoute route;assert(auto_route_build(&route,1,1.570796327f));
    for(unsigned i=0;i<AUTO_ROUTE_COUNT;i++) {
        const AutoRouteSegment s=route.segment[i];
        if(s.aisle==7) assert(s.row==0);
        if(s.aisle>=1&&s.aisle<=6) assert(s.row==s.aisle);
    }
    localization_tests();command_tests();protection_tests();range_threshold_tests();endpoint_acceptance_tests();
    g7_ridge_exit_test();aisle_correction_tests();
    aisle_closed_loop_test();
    tree_config_and_pause_tests();
    run_empty_field(true,false);run_empty_field(true,true);run_empty_field(false,false);
    puts("FULL_MAP_TEST_OK simulation_only_no_hardware_or_work_actions");return 0;
}
