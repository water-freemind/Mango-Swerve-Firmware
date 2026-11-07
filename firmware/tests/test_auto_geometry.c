#include "auto_geometry.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#define NEAR(a,b) assert(fabsf((a)-(b))<0.00002f)
static AutoConfig fixture(void) {
    AutoConfig c={0};
    c.wheel_radius=.051f;
    c.wheel[0]=(AutoVec2){.08f,.08f}; c.wheel[1]=(AutoVec2){.08f,-.08f};
    c.wheel[2]=(AutoVec2){-.08f,-.08f}; c.wheel[3]=(AutoVec2){-.08f,.08f};
    c.stowed=(AutoEnvelope){.14f,.15f,.13f,.16f};
    c.extended=(AutoEnvelope){.34f,.15f,.13f,.16f};
    const float angles[4]={1.5707963268f,-1.5707963268f,3.1415926536f,0};
    for(unsigned i=0;i<4;++i) {
        c.laser[i]=(AutoLaserMount){{.04f*cosf(angles[i]),.04f*sinf(angles[i])},.03f,angles[i]};
        c.laser_evidence[i]=(AutoEvidence){"TEST fixture only, not measured",false};
    }
    c.brake_decel=.3f; c.latency_s=.3f; c.clearance_margin=.02f; c.residual_limit=.03f;
    c.range_max_age_ms=200; c.pose_max_age_ms=50;
    c.wheels_evidence=c.envelope_evidence=c.brake_evidence=c.start_evidence=c.work_contract_evidence=
        (AutoEvidence){"TEST fixture only, not measured",false};
    c.simulation_only=true;
    return c;
}
static AutoClearance range_case(AutoConfig c,unsigned ch,float gap,AutoTwist vel) {
    AutoLaserMount mount=c.laser[ch];
    AutoVec2 n={cosf(mount.yaw),sinf(mount.yaw)};
    float edge=auto_envelope_support(c.stowed,n);
    float offset=mount.origin.x*n.x+mount.origin.y*n.y;
    AutoRange r={(uint16_t)lroundf(1000*(edge-offset+gap)),100,11,true,true};
    AutoPlane plane={n,edge+gap,11,true};
    AutoPoseSample p={{0,0,0},100,true};
    AutoClearance out;
    assert(auto_laser_clearance(&c,ch,c.stowed,p,vel,r,plane,100,&out)==AUTO_GEOM_OK);
    NEAR(out.clearance_m,gap);
    return out;
}
int main(void) {
    AutoConfig c=fixture();
    assert(auto_config_valid(&c)); assert(!auto_config_hardware_ready(&c));
    c.wheels_evidence.confirmed=c.envelope_evidence.confirmed=c.brake_evidence.confirmed=
        c.start_evidence.confirmed=c.work_contract_evidence.confirmed=true;
    for(unsigned i=0;i<4;++i) c.laser_evidence[i].confirmed=true;
    assert(!auto_config_hardware_ready(&c)); /* even all flags cannot promote simulation */
    c.simulation_only=false; assert(auto_config_hardware_ready(&c));
    c.laser_evidence[2].confirmed=false; assert(!auto_config_hardware_ready(&c));
    c=fixture();
    const float gaps[4]={.055f,.065f,.055f,.065f};
    const float offsets[4]={.09f,.12f,.11f,.10f};
    for(unsigned i=0;i<4;++i) {
        AutoClearance out=range_case(c,i,gaps[i],(AutoTwist){0});
        NEAR(out.edge_offset_m,offsets[i]); assert(!out.protective_stop);
    }
    assert(range_case(c,AUTO_FRONT,.055f,(AutoTwist){.5f,0,0}).protective_stop);
    assert(!range_case(c,AUTO_FRONT,.055f,(AutoTwist){-.5f,0,0}).protective_stop);
    assert(range_case(c,AUTO_REAR,.055f,(AutoTwist){-.5f,0,0}).protective_stop);
    assert(range_case(c,AUTO_RIGHT,.055f,(AutoTwist){0,-.5f,0}).protective_stop);
    assert(range_case(c,AUTO_LEFT,.055f,(AutoTwist){0,.5f,0}).protective_stop);
    assert(range_case(c,AUTO_FRONT,.055f,(AutoTwist){0,0,2}).protective_stop);
    AutoPoseSample p={{0,0,0},100,true};
    AutoRange r={155,100,11,true,true};
    AutoPlane wall={{1,0},.195f,11,true};
    AutoClearance out;
    assert(auto_laser_clearance(&c,3,c.extended,p,(AutoTwist){0},r,wall,100,&out)==AUTO_GEOM_OK);
    assert(out.collision); NEAR(out.clearance_m,-.145f);
    r.raw_mm=50; wall.offset=.09f; /* below 7 cm is accepted, but body overlaps plane */
    assert(auto_laser_clearance(&c,3,c.stowed,p,(AutoTwist){0},r,wall,100,&out)==AUTO_GEOM_OK);
    assert(out.collision);
    r.raw_mm=0;
    assert(auto_laser_clearance(&c,3,c.stowed,p,(AutoTwist){0},r,wall,100,&out)==AUTO_GEOM_RANGE);
    r.raw_mm=65535;
    assert(auto_laser_clearance(&c,3,c.stowed,p,(AutoTwist){0},r,wall,100,&out)==AUTO_GEOM_RANGE);
    r=(AutoRange){155,100,11,true,true}; wall.offset=.195f;
    r.channel_verified=false;
    assert(auto_laser_clearance(&c,3,c.stowed,p,(AutoTwist){0},r,wall,100,&out)==AUTO_GEOM_UNCONFIRMED);
    r.channel_verified=true; r.surface_id=12;
    assert(auto_laser_clearance(&c,3,c.stowed,p,(AutoTwist){0},r,wall,100,&out)==AUTO_GEOM_SURFACE);
    r.surface_id=11; wall.associated=false;
    assert(auto_laser_clearance(&c,3,c.stowed,p,(AutoTwist){0},r,wall,100,&out)==AUTO_GEOM_SURFACE);
    wall.associated=true; wall.offset=1.0f; /* opening / wrong wall */
    assert(auto_laser_clearance(&c,3,c.stowed,p,(AutoTwist){0},r,wall,100,&out)==AUTO_GEOM_RESIDUAL);
    wall.offset=.195f;
    assert(auto_laser_clearance(&c,3,c.stowed,p,(AutoTwist){0},r,wall,151,&out)==AUTO_GEOM_STALE);
    p.stamp_ms=301;
    assert(auto_laser_clearance(&c,3,c.stowed,p,(AutoTwist){0},r,wall,301,&out)==AUTO_GEOM_STALE);
    p.stamp_ms=100; p.pose.yaw=.1f;
    wall.offset=.195f*cosf(.1f); /* identical ray length at nonzero yaw */
    assert(auto_laser_clearance(&c,3,c.stowed,p,(AutoTwist){0},r,wall,100,&out)==AUTO_GEOM_OK);
    NEAR(out.clearance_m,.055f*cosf(.1f)-.16f*sinf(.1f));
    p.pose.yaw=NAN;
    assert(auto_laser_clearance(&c,3,c.stowed,p,(AutoTwist){0},r,wall,100,&out)==AUTO_GEOM_CONFIG);
    AutoPoseSample mapped;
    p=(AutoPoseSample){{1,0,.1f},UINT32_MAX-9,true};
    assert(auto_pose_to_map(p,(AutoPose){.2f,-.5f,1.5707963268f},10,50,&mapped));
    NEAR(mapped.pose.x,.2f); NEAR(mapped.pose.y,.5f); NEAR(mapped.pose.yaw,1.6707963268f);
    assert(mapped.stamp_ms==p.stamp_ms);
    assert(!auto_pose_to_map(p,(AutoPose){0},100,50,&mapped)); assert(!mapped.valid);
    assert(!auto_envelope_inside(c.stowed,(AutoPose){.2f,-.05f,1.5707963268f},0,-1,.4f,0,0));
    assert(auto_envelope_inside(c.stowed,(AutoPose){.2f,-.5f,1.5707963268f},0,-1,.4f,0,.01f));
    assert(!auto_envelope_inside(c.extended,(AutoPose){.2f,-.3f,1.5707963268f},0,-1,.4f,0,0));
    AutoConfig bad=c; bad.brake_decel=0; assert(!auto_config_valid(&bad));
    bad=c; bad.wheel[2]=bad.wheel[1]; assert(!auto_config_valid(&bad));
    bad=c; bad.laser[0].yaw=NAN; assert(!auto_config_valid(&bad));
    puts("AUTO_GEOMETRY_TEST_OK (four offsets, 55/65 gaps, yaw, extension, closing speed, validity, map pose and whole-body parking)");
    return 0;
}
