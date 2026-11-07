#include "auto_geometry.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static bool finite_vec(AutoVec2 v) { return isfinite(v.x) && isfinite(v.y); }
static bool finite_pose(AutoPose p) { return isfinite(p.x) && isfinite(p.y) && isfinite(p.yaw); }
static bool positive(float x) { return isfinite(x) && x > 0; }
static bool nonnegative(float x) { return isfinite(x) && x >= 0; }
static bool envelope_valid(AutoEnvelope b) {
    return positive(b.front) && positive(b.rear) && positive(b.left) && positive(b.right);
}
static bool documented(AutoEvidence e) { return e.source != NULL && e.source[0] != '\0'; }
static bool confirmed(AutoEvidence e) { return documented(e) && e.confirmed; }
static float dot(AutoVec2 a, AutoVec2 b) { return a.x*b.x + a.y*b.y; }

bool auto_config_valid(const AutoConfig* c) {
    if(!c || !positive(c->wheel_radius) || !envelope_valid(c->stowed) || !envelope_valid(c->extended) ||
       !positive(c->brake_decel) || !nonnegative(c->latency_s) || !nonnegative(c->clearance_margin) ||
       !positive(c->residual_limit) || !c->range_max_age_ms || !c->pose_max_age_ms ||
       c->range_max_age_ms > 10000u || c->pose_max_age_ms > 10000u ||
       !documented(c->wheels_evidence) || !documented(c->envelope_evidence) ||
       !documented(c->brake_evidence) || !documented(c->start_evidence) ||
       !documented(c->work_contract_evidence)) return false;
    for(unsigned i=0; i<4; ++i) {
        if(!finite_vec(c->wheel[i]) || !finite_vec(c->laser[i].origin) ||
           !isfinite(c->laser[i].yaw) || !nonnegative(c->laser[i].z) ||
           !documented(c->laser_evidence[i])) return false;
        for(unsigned j=0; j<i; ++j)
            if(hypotf(c->wheel[i].x-c->wheel[j].x,c->wheel[i].y-c->wheel[j].y)<0.001f) return false;
    }
    return true;
}
bool auto_config_hardware_ready(const AutoConfig* c) {
    if(!auto_config_valid(c) || c->simulation_only || !confirmed(c->wheels_evidence) ||
       !confirmed(c->envelope_evidence) || !confirmed(c->brake_evidence) ||
       !confirmed(c->start_evidence) || !confirmed(c->work_contract_evidence)) return false;
    for(unsigned i=0; i<4; ++i) if(!confirmed(c->laser_evidence[i])) return false;
    return true;
}
float auto_wrap_angle(float a) { return remainderf(a, 6.28318530718f); }
AutoVec2 auto_rotate(AutoVec2 v, float yaw) {
    float c=cosf(yaw), s=sinf(yaw);
    return (AutoVec2){c*v.x-s*v.y,s*v.x+c*v.y};
}
bool auto_pose_to_map(AutoPoseSample in, AutoPose t, uint32_t now, uint32_t age, AutoPoseSample* out) {
    if(!out) return false;
    memset(out,0,sizeof(*out));
    if(!in.valid || !finite_pose(in.pose) || !finite_pose(t) || !age ||
       (uint32_t)(now-in.stamp_ms)>age) return false;
    AutoVec2 xy=auto_rotate((AutoVec2){in.pose.x,in.pose.y},t.yaw);
    out->pose=(AutoPose){xy.x+t.x,xy.y+t.y,auto_wrap_angle(in.pose.yaw+t.yaw)};
    out->stamp_ms=in.stamp_ms;
    out->valid=finite_pose(out->pose);
    return out->valid;
}
float auto_envelope_support(AutoEnvelope b, AutoVec2 n) {
    return (n.x>=0 ? b.front*n.x : -b.rear*n.x) + (n.y>=0 ? b.left*n.y : -b.right*n.y);
}
float auto_envelope_swept_support(AutoEnvelope b, AutoVec2 n, float angle) {
    const float turn=6.28318530718f;
    const float normal_length=hypotf(n.x,n.y);
    if(!envelope_valid(b) || !finite_vec(n) || !isfinite(angle) ||
       fabsf(normal_length-1.0f)>.001f) return NAN;
    const AutoVec2 corners[4]={{b.front,b.left},{b.front,-b.right},{-b.rear,b.left},{-b.rear,-b.right}};
    const float low=fminf(0,angle), high=fmaxf(0,angle), normal_angle=atan2f(n.y,n.x);
    float maximum=0;
    for(unsigned i=0;i<4;i++) {
        const AutoVec2 p=corners[i]; const float radius=hypotf(p.x,p.y)*normal_length;
        if(!isfinite(radius)) return NAN;
        float candidate;
        if(fabsf(angle)>=turn) candidate=radius;
        else {
            candidate=fmaxf(dot(n,p),dot(n,auto_rotate(p,angle)));
            const float phase=normal_angle-atan2f(p.y,p.x);
            const float critical=phase+ceilf((low-phase)/turn)*turn;
            if(critical<=high) candidate=fmaxf(candidate,radius);
        }
        maximum=fmaxf(maximum,candidate);
    }
    return maximum;
}
bool auto_envelope_inside(AutoEnvelope b, AutoPose p, float xmin,float ymin,float xmax,float ymax,float margin) {
    if(!envelope_valid(b) || !finite_pose(p) || !isfinite(xmin) || !isfinite(ymin) ||
       !isfinite(xmax) || !isfinite(ymax) || xmin>=xmax || ymin>=ymax || !nonnegative(margin)) return false;
    const AutoVec2 axes[4]={{1,0},{-1,0},{0,1},{0,-1}};
    const float limits[4]={xmax,-xmin,ymax,-ymin};
    for(unsigned i=0;i<4;++i) {
        float edge=dot(axes[i],(AutoVec2){p.x,p.y})+auto_envelope_support(b,auto_rotate(axes[i],-p.yaw));
        if(edge+margin>limits[i]) return false;
    }
    return true;
}
AutoGeomResult auto_laser_clearance(const AutoConfig* c,unsigned ch,AutoEnvelope body,
    AutoPoseSample pose,AutoTwist v,AutoRange r,AutoPlane plane,uint32_t now,AutoClearance* out) {
    if(!out) return AUTO_GEOM_CONFIG;
    memset(out,0,sizeof(*out));
    if(!auto_config_valid(c) || ch>=4 || !envelope_valid(body) || !finite_pose(pose.pose) ||
       !isfinite(v.vx) || !isfinite(v.vy) || !isfinite(v.wz)) return AUTO_GEOM_CONFIG;
    if(!pose.valid || (uint32_t)(now-pose.stamp_ms)>c->pose_max_age_ms ||
       (uint32_t)(now-r.stamp_ms)>c->range_max_age_ms) return AUTO_GEOM_STALE;
    if(!r.valid || !r.raw_mm || r.raw_mm==UINT16_MAX) return AUTO_GEOM_RANGE;
    if(!r.channel_verified) return AUTO_GEOM_UNCONFIRMED;
    if(!plane.associated || !plane.surface_id || r.surface_id!=plane.surface_id ||
       !finite_vec(plane.normal) || !isfinite(plane.offset) ||
       fabsf(hypotf(plane.normal.x,plane.normal.y)-1.0f)>0.001f) return AUTO_GEOM_SURFACE;
    AutoLaserMount m=c->laser[ch];
    AutoVec2 n=auto_rotate(plane.normal,-pose.pose.yaw);
    AutoVec2 u={cosf(m.yaw),sinf(m.yaw)};
    float incidence=dot(n,u);
    if(incidence<0.2f) return AUTO_GEOM_SURFACE; /* grazing/back-facing ray */
    AutoVec2 sensor_map=auto_rotate(m.origin,pose.pose.yaw);
    sensor_map.x+=pose.pose.x; sensor_map.y+=pose.pose.y;
    out->raw_m=r.raw_mm*0.001f;
    out->expected_m=(plane.offset-dot(plane.normal,sensor_map))/incidence;
    out->residual_m=out->raw_m-out->expected_m;
    if(!positive(out->expected_m) || fabsf(out->residual_m)>c->residual_limit) return AUTO_GEOM_RESIDUAL;
    out->edge_offset_m=auto_envelope_support(body,n)-dot(n,m.origin);
    out->clearance_m=out->raw_m*incidence-out->edge_offset_m;
    float radius=hypotf(fmaxf(body.front,body.rear),fmaxf(body.left,body.right));
    out->closing_mps=fmaxf(0,dot(n,(AutoVec2){v.vx,v.vy})+fabsf(v.wz)*radius);
    const uint32_t range_age=now-r.stamp_ms, pose_age=now-pose.stamp_ms;
    out->sample_age_s=(range_age>pose_age?range_age:pose_age)*.001f;
    out->response_budget_s=c->latency_s+out->sample_age_s;
    out->required_m=out->closing_mps*out->response_budget_s+
        out->closing_mps*out->closing_mps/(2*c->brake_decel)+c->clearance_margin;
    if(!isfinite(out->clearance_m) || !isfinite(out->required_m)) return AUTO_GEOM_CONFIG;
    out->collision=out->clearance_m<=0;
    out->protective_stop=out->clearance_m<=out->required_m;
    return AUTO_GEOM_OK;
}
