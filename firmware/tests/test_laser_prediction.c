#include "laser_prediction.h"
#include "straight_test.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static LaserSnapshot measured(uint32_t stamp) {
    LaserSnapshot l = {.accepted_mm={3800,450,3000,6000}, .frame_fresh=true,
        .valid_mask=15,.updated_mask=15,.frame_count=stamp+1u,.last_rx_ms=stamp};
    return l;
}
static LaserPredictionOutput step(LaserPrediction* s, LaserSnapshot* l, uint32_t t, float vx) {
    return laser_prediction_step(s,l,t,vx,0,0,true);
}
int main(void) {
    LaserPrediction s={0}; LaserSnapshot l=measured(1000);
    assert(step(&s,&l,1000,-0.2f).mask==0);
    l.updated_mask=11;
    LaserPredictionOutput o={0};
    for(unsigned t=1010;t<=1400;t+=10) {
        l.sample_age_ms[2]=t-1000;
        if(t>1300) l.valid_mask=11;
        o=step(&s,&l,t,-0.2f);
        assert(o.mask==4);
    }
    assert(o.mm[2]<=2900 && o.mm[2]>2800); /* 80mm travel plus uncertainty */
    for(unsigned t=1410;t<=1610;t+=10) {l.sample_age_ms[2]=t-1000;o=step(&s,&l,t,-0.2f);}
    assert(o.mask==0); /* Repeated estimates cannot renew real age. */

    s=(LaserPrediction){0}; l=measured(0);step(&s,&l,0,0.2f);
    l.updated_mask=11; l.sample_age_ms[2]=10;o=step(&s,&l,10,0.2f);
    assert(o.mask==4 && o.mm[2]<=2980); /* Moving away never enlarges clearance. */
    l.frame_fresh=false; assert(step(&s,&l,20,0.2f).mask==0);
    l.frame_fresh=true; assert(step(&s,&l,30,0.6f).mask==0);
    assert(step(&s,&l,40,0.2f).mask==0); /* No inherited anchor after bad feedback. */

    s=(LaserPrediction){0};l=measured(0);step(&s,&l,0,0.2f);
    l.updated_mask=7;l.sample_age_ms[3]=10;
    assert(step(&s,&l,10,0.2f).mask==8);
    assert(laser_prediction_step(&s,&l,20,0.2f,0,0.04f,true).mask==0);
    s=(LaserPrediction){0};l=measured(0);step(&s,&l,0,0.5f);l.updated_mask=7;
    for(unsigned t=10;t<=410;t+=10) {l.sample_age_ms[3]=t;o=step(&s,&l,t,0.5f);}
    assert(o.mask==0); /* 205mm exceeds path budget before time budget. */

    s=(LaserPrediction){0};l=measured(0);step(&s,&l,0,-0.2f);
    l.accepted_mm[2]=100; l.updated_mask=15; l.sample_age_ms[2]=0;l.last_rx_ms=10;
    assert(step(&s,&l,10,-0.2f).mm[2]==100);
    l.updated_mask=11;l.sample_age_ms[2]=10;
    assert(step(&s,&l,20,-0.2f).mask==0); /* Near true samples are never masked. */
    assert(laser_prediction_step(&s,&l,30,-0.2f,0,0,false).mask==0);

    s=(LaserPrediction){0};l=measured(UINT32_MAX-20u);step(&s,&l,UINT32_MAX-20u,-0.2f);
    l.updated_mask=11;l.sample_age_ms[2]=30;
    assert(step(&s,&l,9,-0.2f).mask==4); /* Tick wrap */
    assert(step(&s,&l,100,-0.2f).mask==0); /* Scheduler gap invalidates integration */
    s=(LaserPrediction){0};l=measured(0);step(&s,&l,0,-0.2f);l.updated_mask=11;l.sample_age_ms[2]=10;
    assert(laser_prediction_step(&s,&l,10,NAN,0,0,true).mask==0);

    StraightTest t; straight_init(&t,0);
    StraightInput in={.healthy=true,.stationary=true,.laser_fresh=true,
        .left_mm=3800,.right_mm=450,.rear_mm=3000,.front_mm=6000,.laser_updated_mask=15,.laser_frame=1};
    assert(straight_command(&t,"ARM WALL 4",0,in));
    assert(straight_command(&t,"GO 1",1,in));
    t.leg=ST_LEG_BACK;t.outbound_m=t.distance_m=3.99f;t.speed_mps=-0.2f;
    in.stationary=false;in.vx_mps=-0.2f;in.laser_updated_mask=11;in.laser_predicted_mask=4;
    straight_step(&t,11,in);
    assert(t.phase==ST_RUNNING && !t.laser_paused && t.speed_mps<0);
    assert(!straight_laser_ready(ST_PLAN_WALL,in)); /* Predicted data cannot ARM. */
    in.laser_predicted_mask=0;in.laser_stale_mask=4;
    straight_step(&t,21,in);
    assert(t.laser_paused && t.speed_mps==0);
    in.laser_stale_mask=0;in.laser_predicted_mask=4;in.laser_frame++;
    straight_step(&t,31,in);
    assert(t.laser_paused && t.laser_recovery_frames==0); /* Not a real recovery frame. */
    puts("LASER_PREDICTION_TEST_OK");
    return 0;
}
