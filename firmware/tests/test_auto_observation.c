#include "auto_fixture.h"
#include "auto_observation.h"
#include <assert.h>
#include <stdio.h>

static AutoObservationInput sample(AutoMissionConfig c,AutoPose actual,AutoPose estimate,uint32_t t) {
    AutoObservationInput in={.pose={estimate,t,true},.now_ms=t};
    for(unsigned i=0;i<4;++i) {
        AutoPlane p;float range;
        if(auto_route_ray(&c.route,actual,c.geometry.laser[i],7.5f,&p,&range)) {
            in.ranges[i]=(AutoRange){(uint16_t)lroundf(range*1000),t,0,true,true};
            in.required_mask|=(uint8_t)(1u<<i);
        }
    }
    return in;
}
int main(void) {
    (void)auto_fixture_input;
    AutoMissionConfig c=auto_fixture();AutoObservation m={0};
    AutoPose p={1.5f,.2f,0};
    AutoObservationInput in=sample(c,p,p,1000);
    AutoObservationOutput out=auto_observation_step(&m,&c.geometry,&c.route,c.geometry.stowed,in);
    assert(out.required_ready && out.valid_mask==15 && !out.protective_stop);
    in.ranges[AUTO_LEFT].valid=false;
    out=auto_observation_step(&m,&c.geometry,&c.route,c.geometry.stowed,in);
    assert(!out.required_ready && !(out.valid_mask&(1u<<AUTO_LEFT)));
    in=sample(c,p,p,1000);in.ranges[AUTO_LEFT].channel_verified=false;
    assert(!auto_observation_step(&m,&c.geometry,&c.route,c.geometry.stowed,in).required_ready);
    in=sample(c,p,p,1000);in.ranges[AUTO_LEFT].stamp_ms=799;
    out=auto_observation_step(&m,&c.geometry,&c.route,c.geometry.stowed,in);
    assert(!out.required_ready && out.reason[AUTO_LEFT]==AUTO_GEOM_STALE);
    in=sample(c,p,p,1000);in.ranges[AUTO_LEFT].surface_id=9999;
    assert(auto_observation_step(&m,&c.geometry,&c.route,c.geometry.stowed,in).reason[AUTO_LEFT]==AUTO_GEOM_SURFACE);

    /* Frozen range / smooth wheel slip: a plausible pose with an independent
     * range disagreement is not fixed by a fresh aggregate packet timestamp. */
    m=(AutoObservation){0};AutoPose slipped=p;slipped.y+=.06f;
    in=sample(c,p,slipped,1000);
    out=auto_observation_step(&m,&c.geometry,&c.route,c.geometry.stowed,in);
    assert(out.pose_consistent && !out.required_ready && out.reason[AUTO_LEFT]==AUTO_GEOM_RESIDUAL);
    m=(AutoObservation){0};
    for(unsigned tick=0;tick<60;++tick) {
        slipped=p;slipped.y+=tick*.001f;
        in=sample(c,p,slipped,1000+tick*10);
        out=auto_observation_step(&m,&c.geometry,&c.route,c.geometry.stowed,in);
        assert(out.pose_consistent); /* each delta is below the jump threshold */
        if(!out.required_ready) break;
    }
    assert(!out.required_ready && (out.reason[AUTO_LEFT]==AUTO_GEOM_RESIDUAL || out.reason[AUTO_RIGHT]==AUTO_GEOM_RESIDUAL));
    m=(AutoObservation){0};in=sample(c,p,p,1000);
    auto_observation_step(&m,&c.geometry,&c.route,c.geometry.stowed,in);
    in.pose.pose.x+=.1f; /* same stamp, different sample */
    assert(!auto_observation_step(&m,&c.geometry,&c.route,c.geometry.stowed,in).pose_consistent);
    in=sample(c,p,p,1010);in.pose.pose.x+=.1f;
    assert(!auto_observation_step(&m,&c.geometry,&c.route,c.geometry.stowed,in).pose_consistent);
    in=sample(c,p,p,1010);in.pose.pose.yaw+=.1f;
    assert(!auto_observation_step(&m,&c.geometry,&c.route,c.geometry.stowed,in).pose_consistent);
    in=sample(c,p,p,1010);in.commanded.vx=NAN;
    out=auto_observation_step(&m,&c.geometry,&c.route,c.geometry.stowed,in);
    assert(!out.required_ready && out.reason[0]==AUTO_GEOM_CONFIG);

    /* Slowly accumulating yaw drift is bounded by the segment yaw gate; a
     * single wall cannot observe every yaw/slip mode. Discontinuities are caught. */
    m=(AutoObservation){0};in=sample(c,p,p,1000);in.commanded.vy=.3f;
    out=auto_observation_step(&m,&c.geometry,&c.route,c.geometry.stowed,in);
    assert(out.required_ready && out.protective_stop && (out.stop_mask&(1u<<AUTO_LEFT)));
    m=(AutoObservation){0};in=sample(c,p,p,1000);
    AutoEnvelope enlarged=c.geometry.stowed;enlarged.left=.21f;
    out=auto_observation_step(&m,&c.geometry,&c.route,enlarged,in);
    assert(out.clearance[AUTO_LEFT].collision);
    p=(AutoPose){.2f,.1f,-1.5707963268f};m=(AutoObservation){0};in=sample(c,p,p,1000);
    in.required_mask|=1u<<AUTO_FRONT;
    out=auto_observation_step(&m,&c.geometry,&c.route,c.geometry.stowed,in);
    assert(out.opening_mask&(1u<<AUTO_FRONT));assert(!(out.valid_mask&(1u<<AUTO_FRONT)) && !out.required_ready);
    /* A side ray changes row face at the short R4, and an old association fails. */
    AutoPlane near,short_row;float a,b;
    assert(auto_route_ray(&c.route,(AutoPose){.2f,.5f,0},c.geometry.laser[AUTO_FRONT],7.5f,&near,&a));
    assert(auto_route_ray(&c.route,(AutoPose){.2f,2.6f,0},c.geometry.laser[AUTO_FRONT],7.5f,&short_row,&b));
    assert(near.surface_id!=short_row.surface_id && fabsf(b-a-.2f)<.00001f);
    p=(AutoPose){.2f,2.6f,0};m=(AutoObservation){0};in=sample(c,p,p,1000);
    in.ranges[AUTO_FRONT].surface_id=near.surface_id;
    assert(auto_observation_step(&m,&c.geometry,&c.route,c.geometry.stowed,in).reason[AUTO_FRONT]==AUTO_GEOM_SURFACE);
    puts("AUTO_OBSERVATION_TEST_OK (individual health, stale/frozen ranges, slip residual, pose/yaw jumps, opening, short row, command stopping budget)");
}
