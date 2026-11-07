#ifndef MANGO_AUTO_TEST_FIXTURE_H
#define MANGO_AUTO_TEST_FIXTURE_H
#include "auto_mission.h"
#include <math.h>
/* Fictional commissioning scenario. NEVER linked as a production profile. */
static AutoMissionConfig auto_fixture(void) {
    AutoMissionConfig c={0}; AutoConfig* g=&c.geometry;
    g->simulation_only=c.simulation_only=c.work.simulation_only=true;
    g->wheel_radius=.051f;
    g->wheel[0]=(AutoVec2){.08f,.08f};g->wheel[1]=(AutoVec2){.08f,-.08f};
    g->wheel[2]=(AutoVec2){-.08f,-.08f};g->wheel[3]=(AutoVec2){-.08f,.08f};
    g->stowed=(AutoEnvelope){.135f,.135f,.135f,.135f};
    g->extended=(AutoEnvelope){.34f,.135f,.135f,.135f};
    const float yaw[4]={1.5707963268f,-1.5707963268f,3.1415926536f,0};
    for(unsigned i=0;i<4;++i) {
        g->laser[i]=(AutoLaserMount){{.04f*cosf(yaw[i]),.04f*sinf(yaw[i])},.035f,yaw[i]};
        g->laser_evidence[i]=(AutoEvidence){"SIMULATED, no hardware calibration",false};
    }
    g->brake_decel=.3f;g->latency_s=.1f;g->clearance_margin=.01f;g->residual_limit=.04f;
    g->range_max_age_ms=200;g->pose_max_age_ms=50;
    g->wheels_evidence=g->envelope_evidence=g->brake_evidence=g->start_evidence=g->work_contract_evidence=
        (AutoEvidence){"SIMULATED, no hardware calibration",false};
    c.motion=(AutoMotionConfig){.vmax=.3f,.accel=.3f,.decel=.3f,.lateral_max=.03f,.cross_kp=.8f,
        .yaw_kp=1,.wz_max=.1f,.angular_accel=.3f,.position_tolerance=.008f,.cross_tolerance=.005f,
        .yaw_tolerance=.01f,.cross_abort=.15f,.yaw_abort=.25f,.settle_s=.15f,.stop_latency_s=.08f,.pose_max_age_ms=50};
    c.work=(AutoWorkConfig){1000,2000,1000,4000,100,100,true,false};
    auto_route_build(&c.route,1,1.5707963268f);
    c.deadline_ms=300000;c.pause_timeout_ms=1500;c.pause_stable_ms=100;c.stop_work_ms=2000;
    c.segment_timeout_ms=90000;c.max_pauses=2;c.reserve_s=15;c.segment_allowance_s=.3f;c.footprint_margin=.01f;
    return c;
}
static AutoMissionInput auto_fixture_input(AutoPose pose,uint32_t now) {
    return (AutoMissionInput){.motion={.pose={pose,now,true},.healthy=true,.aligned=true,.stationary=true,.now_ms=now},
        .precheck_ready=true,.tools_stowed=true,.observation_ok=true,.localization_ok=true,
        .work={.peer_online=true,.peer_epoch=9,.healthy=true,.stationary=true,.now_ms=now}};
}
#endif
