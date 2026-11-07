/* Synthetic planar fixtures only. This does not connect to a sensor, infer
 * missing calibration or prove the real chassis' braking/rotation envelope. */
#include "auto_geometry.h"
#include "straight_test.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static FILE* csv;
static const float PI=3.14159265359f;
static const AutoVec2 normal[4]={{0,1},{0,-1},{-1,0},{1,0}};
static const char* reason(AutoGeomResult r) {
    const char* names[]={"OK","CONFIG","UNCONFIRMED","STALE","RANGE","SURFACE","RESIDUAL"};
    return (unsigned)r<sizeof(names)/sizeof(names[0])?names[r]:"UNKNOWN";
}
static AutoConfig fixture(void) {
    AutoConfig c={0};
    c.wheel_radius=.051f;
    c.wheel[0]=(AutoVec2){.08f,.08f}; c.wheel[1]=(AutoVec2){.08f,-.08f};
    c.wheel[2]=(AutoVec2){-.08f,-.08f}; c.wheel[3]=(AutoVec2){-.08f,.08f};
    c.stowed=(AutoEnvelope){.14f,.15f,.13f,.16f};
    c.extended=(AutoEnvelope){.34f,.18f,.17f,.19f};
    const float yaw[4]={1.57079632679f,-1.57079632679f,3.14159265359f,0};
    const float mount[4]={.03f,.04f,.02f,.05f};
    for(unsigned i=0;i<4;i++) {
        c.laser[i]=(AutoLaserMount){{normal[i].x*mount[i],normal[i].y*mount[i]},.03f,yaw[i]};
        c.laser_evidence[i]=(AutoEvidence){"synthetic asymmetric clearance fixture",false};
    }
    c.brake_decel=.3f; c.latency_s=.1f; c.clearance_margin=.03f; c.residual_limit=.05f;
    c.range_max_age_ms=200; c.pose_max_age_ms=50;
    c.wheels_evidence=c.envelope_evidence=c.brake_evidence=c.start_evidence=c.work_contract_evidence=
        (AutoEvidence){"synthetic fixture, never hardware calibration",false};
    c.simulation_only=true;
    assert(auto_config_valid(&c)); assert(!auto_config_hardware_ready(&c));
    return c;
}
static float dot(AutoVec2 a,AutoVec2 b) { return a.x*b.x+a.y*b.y; }
static AutoRange measure(const AutoConfig* c,unsigned ch,AutoPose pose,AutoPlane plane,uint32_t stamp) {
    const AutoLaserMount m=c->laser[ch];
    AutoVec2 origin=auto_rotate(m.origin,pose.yaw);
    origin.x+=pose.x; origin.y+=pose.y;
    AutoVec2 ray=auto_rotate((AutoVec2){cosf(m.yaw),sinf(m.yaw)},pose.yaw);
    const float raw=(plane.offset-dot(plane.normal,origin))/dot(plane.normal,ray);
    assert(raw>0 && raw<65.0f);
    return (AutoRange){(uint16_t)lroundf(raw*1000),stamp,plane.surface_id,true,true};
}
static void record(const char* scene,unsigned ch,uint32_t t,AutoRange range,AutoTwist v,
                   float true_gap,AutoGeomResult result,AutoClearance out,const char* first) {
    if(csv) fprintf(csv,"%s,%u,%lu,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s,%u,%s\n",
        scene,ch,(unsigned long)t,range.raw_mm,true_gap,out.edge_offset_m,out.clearance_m,out.required_m,
        out.sample_age_s,out.response_budget_s,hypotf(v.vx,v.vy),v.wz,reason(result),out.protective_stop,first);
}
static void approach(unsigned ch,float reaction,float decel,const char* scene) {
    AutoConfig c=fixture(); c.latency_s=reaction; c.brake_decel=decel;
    const AutoVec2 n=normal[ch];
    AutoPlane wall={n,1.0f+auto_envelope_support(c.stowed,n),1,true};
    AutoPose history[16]={{0}};
    const unsigned delay_ticks=10; /* 100ms observed range age. */
    float displacement=0,speed=.3f,min_gap=100;
    uint32_t first_stop=0;
    for(unsigned k=0;k<1000;k++) {
        const uint32_t t=1000u+k*10u;
        AutoPose p={n.x*displacement,n.y*displacement,0}; history[k%16]=p;
        unsigned old=k>=delay_ticks?k-delay_ticks:0;
        AutoRange r=measure(&c,ch,history[old%16],wall,1000u+old*10u);
        AutoTwist v={n.x*speed,n.y*speed,0};
        AutoClearance out;
        AutoGeomResult result=auto_laser_clearance(&c,ch,c.stowed,(AutoPoseSample){p,t,true},v,r,wall,t,&out);
        assert(result==AUTO_GEOM_OK);
        const float gap=1.0f-displacement; min_gap=fminf(min_gap,gap);
        if(out.protective_stop && !first_stop) first_stop=t;
        record(scene,ch,t,r,v,gap,result,out,first_stop?"PROTECTIVE_STOP":"NONE");
        if(first_stop && t-first_stop>=(uint32_t)lroundf(reaction*1000)) speed=fmaxf(0,speed-decel*.01f);
        displacement+=speed*.01f;
        if(first_stop && speed==0) break;
    }
    assert(first_stop && min_gap>.015f); /* Synthetic ground truth stays outside body. */
    printf("CLEARANCE_APPROACH channel=%u scene=%s first_stop_ms=%lu min_ground_truth_gap_m=%.6f\n",ch,scene,(unsigned long)first_stop,min_gap);
}
static void scenario_matrix(void) {
    AutoConfig c=fixture(); AutoClearance out;
    const float gap[4]={.055f,.065f,.055f,.065f};
    const float offsets[4]={.10f,.12f,.13f,.09f};
    for(unsigned ch=0;ch<4;ch++) {
        AutoPlane wall={normal[ch],auto_envelope_support(c.stowed,normal[ch])+gap[ch],10u+ch,true};
        AutoPoseSample p={{0},1000,true}; AutoRange r=measure(&c,ch,p.pose,wall,1000);
        AutoGeomResult result=auto_laser_clearance(&c,ch,c.stowed,p,(AutoTwist){0},r,wall,1000,&out);
        assert(result==AUTO_GEOM_OK && !out.protective_stop);
        assert(fabsf(out.clearance_m-gap[ch])<.00051f && fabsf(out.edge_offset_m-offsets[ch])<.00001f);
        record("scenario55_65",ch,1000,r,(AutoTwist){0},gap[ch],result,out,"NONE");
    }
    AutoPoseSample p={{0},1000,true}; AutoPlane wall={{1,0},.3f,21,true};
    AutoRange r=measure(&c,AUTO_FRONT,p.pose,wall,1000);
    AutoTwist v={.1f,0,0};
    assert(auto_laser_clearance(&c,3,c.stowed,p,v,r,wall,1000,&out)==AUTO_GEOM_OK && !out.protective_stop);
    record("speed_transition",3,1000,r,v,.16f,AUTO_GEOM_OK,out,"NONE");
    v.vx=.4f; p.stamp_ms=r.stamp_ms=1010;
    assert(auto_laser_clearance(&c,3,c.stowed,p,v,r,wall,1010,&out)==AUTO_GEOM_OK && out.protective_stop);
    record("speed_transition",3,1010,r,v,.16f,AUTO_GEOM_OK,out,"PROTECTIVE_STOP");
    assert(auto_laser_clearance(&c,3,c.extended,p,(AutoTwist){0},r,wall,1010,&out)==AUTO_GEOM_OK && out.collision);
    record("extended",3,1010,r,(AutoTwist){0},-.04f,AUTO_GEOM_OK,out,"COLLISION");

    /* Rotation increases required space before the sampled footprint collides.
     * Checking sampled yaw is NOT a continuous sweep or an angular-decel proof. */
    c=fixture(); c.stowed=(AutoEnvelope){.25f,.10f,.10f,.10f}; wall.offset=.34f;
    for(unsigned k=0;k<=9;k++) {
        p=(AutoPoseSample){{0,0,k*.05f},1100u+k*10u,true};
        r=measure(&c,3,p.pose,wall,p.stamp_ms);
        v=(AutoTwist){0,0,1};
        AutoGeomResult result=auto_laser_clearance(&c,3,c.stowed,p,v,r,wall,p.stamp_ms,&out);
        assert(result==AUTO_GEOM_OK && out.protective_stop);
        record("rotation_sampled",3,p.stamp_ms,r,v,NAN,result,out,"PROTECTIVE_STOP");
    }
    c=fixture(); p=(AutoPoseSample){{0,0,PI/6},1300,true}; wall.offset=.5f;
    r=measure(&c,3,p.pose,wall,1300);
    assert(auto_laser_clearance(&c,3,c.stowed,p,(AutoTwist){0},r,wall,1300,&out)==AUTO_GEOM_OK);
    assert(out.clearance_m<out.raw_m-out.edge_offset_m-.04f); /* raw ray != perpendicular gap */
    record("oblique",3,1300,r,(AutoTwist){0},NAN,AUTO_GEOM_OK,out,"NONE");
    AutoPoseSample grazing=p; grazing.pose.yaw=1.4f;
    AutoRange grazing_range=measure(&c,3,grazing.pose,wall,1300);
    assert(auto_laser_clearance(&c,3,c.stowed,grazing,(AutoTwist){0},grazing_range,wall,1300,&out)==AUTO_GEOM_SURFACE);
    grazing.pose.yaw=PI;
    assert(auto_laser_clearance(&c,3,c.stowed,grazing,(AutoTwist){0},r,wall,1300,&out)==AUTO_GEOM_SURFACE);
    wall.associated=false;
    AutoGeomResult result=auto_laser_clearance(&c,3,c.stowed,p,(AutoTwist){0},r,wall,1300,&out);
    assert(result==AUTO_GEOM_SURFACE);
    record("opening",3,1300,r,(AutoTwist){0},NAN,result,out,"SURFACE");
    wall.associated=true; r.surface_id=999;
    assert(auto_laser_clearance(&c,3,c.stowed,p,(AutoTwist){0},r,wall,1300,&out)==AUTO_GEOM_SURFACE);
    r.surface_id=21; r.stamp_ms=1099;
    result=auto_laser_clearance(&c,3,c.stowed,p,(AutoTwist){0},r,wall,1300,&out);
    assert(result==AUTO_GEOM_STALE); record("stale",3,1300,r,(AutoTwist){0},NAN,result,out,"STALE");
    r.stamp_ms=1300; r.channel_verified=false;
    result=auto_laser_clearance(&c,3,c.stowed,p,(AutoTwist){0},r,wall,1300,&out);
    assert(result==AUTO_GEOM_UNCONFIRMED); record("unknown_channel",3,1300,r,(AutoTwist){0},NAN,result,out,"UNCONFIRMED");
    c.simulation_only=false;
    assert(!auto_config_hardware_ready(&c)); /* unknown calibration still blocks hardware */
}
static void age_and_legacy_contract(void) {
    AutoConfig c=fixture(); c.stowed=(AutoEnvelope){.15f,.15f,.15f,.15f};
    for(unsigned i=0;i<4;i++) c.laser[i].origin=(AutoVec2){normal[i].x*.04f,normal[i].y*.04f};
    c.brake_decel=STRAIGHT_TEST_BRAKE_DECEL_MPS2;
    c.latency_s=STRAIGHT_TEST_SENSOR_DELAY_S;
    c.clearance_margin=STRAIGHT_TEST_COLLISION_MARGIN_MM*.001f;
    StraightTest old; straight_init(&old,1000);
    assert(old.body_edge_mm==110); assert(straight_front_target_mm(&old)==210);
    const AutoTwist cases[]={{.2f,0,0},{-.3f,0,0},{0,.1f,0},{0,-.25f,0}};
    for(unsigned k=0;k<4;k++) {
        StraightInput in={.vx_mps=cases[k].vx,.vy_mps=cases[k].vy}; uint16_t limits[4];
        straight_guard_limits(&old,in,limits);
        for(unsigned ch=0;ch<4;ch++) {
            AutoPlane plane={normal[ch],1.5f,50u+ch,true};
            AutoPoseSample p={{0},1000,true}; AutoRange r=measure(&c,ch,p.pose,plane,1000); AutoClearance out;
            assert(auto_laser_clearance(&c,ch,c.stowed,p,cases[k],r,plane,1000,&out)==AUTO_GEOM_OK);
            assert(abs((int)limits[ch]-(int)ceilf(1000*(out.edge_offset_m+out.required_m)))<=1);
        }
    }
    /* Preserve old FWD opt-out; this offline fixture does not change policy. */
    assert(straight_laser_ready(ST_PLAN_FWD,(StraightInput){0}));
    AutoPlane plane={{1,0},.5f,90,true}; AutoPoseSample p={{0},1000,true};
    AutoRange r=measure(&c,3,p.pose,plane,1000); AutoClearance fresh,aged,both;
    assert(auto_laser_clearance(&c,3,c.stowed,p,(AutoTwist){.2f,0,0},r,plane,1000,&fresh)==AUTO_GEOM_OK);
    r.stamp_ms=960;
    assert(auto_laser_clearance(&c,3,c.stowed,p,(AutoTwist){.2f,0,0},r,plane,1000,&aged)==AUTO_GEOM_OK);
    p.stamp_ms=970;
    assert(auto_laser_clearance(&c,3,c.stowed,p,(AutoTwist){.2f,0,0},r,plane,1000,&both)==AUTO_GEOM_OK);
    assert(fabsf(aged.required_m-fresh.required_m-.008f)<.00001f);
    assert(fabsf(both.required_m-aged.required_m)<.00001f); /* max, not double-counted sum */
    record("age_budget",3,1000,r,(AutoTwist){.2f,0,0},.35f,AUTO_GEOM_OK,both,"NONE");
}
int main(int argc,char** argv) {
    if(argc>1) { csv=fopen(argv[1],"w"); assert(csv); }
    if(csv) fputs("scenario,channel,t_ms,raw_mm,ground_truth_gap_m,edge_offset_m,clearance_m,required_m,sample_age_s,response_budget_s,speed_mps,wz_rad_s,result,protective_stop,first_reason\n",csv);
    for(unsigned i=0;i<4;i++) approach(i,.1f,.3f,"approach_base");
    approach(AUTO_FRONT,.2f,.15f,"approach_long_latency_weak_brake");
    scenario_matrix(); age_and_legacy_contract();
    if(csv) fclose(csv);
    puts("CLEARANCE_REPLAY_TEST_OK (synthetic only; no hardware validation)");
    return 0;
}
