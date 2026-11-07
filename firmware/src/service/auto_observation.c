#include "auto_observation.h"
#include <math.h>
AutoObservationOutput auto_observation_step(AutoObservation* m,const AutoConfig* c,
    const AutoRoute* route,AutoEnvelope body,AutoObservationInput in) {
    AutoObservationOutput out={0};
    for(unsigned i=0;i<4;++i) out.reason[i]=AUTO_GEOM_CONFIG;
    if(!m || !auto_config_valid(c) || !auto_route_valid(route) || in.required_mask>15 ||
       !in.pose.valid||!isfinite(in.pose.pose.x)||!isfinite(in.pose.pose.y)||!isfinite(in.pose.pose.yaw)||
       !isfinite(in.measured.vx)||!isfinite(in.measured.vy)||!isfinite(in.measured.wz)||
       !isfinite(in.commanded.vx)||!isfinite(in.commanded.vy)||!isfinite(in.commanded.wz)||
       (uint32_t)(in.now_ms-in.pose.stamp_ms)>c->pose_max_age_ms) return out;
    out.pose_consistent=true;
    if(m->seen && in.pose.stamp_ms==m->previous.stamp_ms &&
       (in.pose.pose.x!=m->previous.pose.x || in.pose.pose.y!=m->previous.pose.y || in.pose.pose.yaw!=m->previous.pose.yaw))
        out.pose_consistent=false; /* A timestamp identifies one immutable sample. */
    if(m->seen && in.pose.stamp_ms!=m->previous.stamp_ms) {
        float dt=(uint32_t)(in.pose.stamp_ms-m->previous.stamp_ms)*.001f;
        AutoVec2 expected=auto_rotate((AutoVec2){in.measured.vx*dt,in.measured.vy*dt},m->previous.pose.yaw);
        float dx=in.pose.pose.x-m->previous.pose.x-expected.x,dy=in.pose.pose.y-m->previous.pose.y-expected.y;
        /* Diagnostic bounds, not localization accuracy guarantees. Smooth wheel
         * slip needs independent wall residuals; this catches discontinuities. */
        float linear_allow=.01f+.5f*dt;
        float yaw_delta=auto_wrap_angle(in.pose.pose.yaw-m->previous.pose.yaw);
        if(dt>.2f||hypotf(dx,dy)>linear_allow||fabsf(yaw_delta-in.measured.wz*dt)>.03f+dt*.2f)
            out.pose_consistent=false;
    }
    if(out.pose_consistent) { m->seen=true; m->previous=in.pose; }
    for(unsigned i=0;i<4;++i) {
        float expected;
        if(!auto_route_ray(route,in.pose.pose,c->laser[i],7.5f,&out.plane[i],&expected)) {
            out.opening_mask|=(uint8_t)(1u<<i);out.reason[i]=AUTO_GEOM_SURFACE;continue;
        }
        AutoRange r=in.ranges[i];
        /* Actual S21C has no surface id. The adapter supplies proposed association
         * from map raycast; residual must still pass using the independent range. */
        if(!r.surface_id) r.surface_id=out.plane[i].surface_id;
        AutoClearance measured,commanded;
        out.reason[i]=auto_laser_clearance(c,i,body,in.pose,in.measured,r,out.plane[i],in.now_ms,&measured);
        if(out.reason[i]!=AUTO_GEOM_OK) continue;
        out.reason[i]=auto_laser_clearance(c,i,body,in.pose,in.commanded,r,out.plane[i],in.now_ms,&commanded);
        if(out.reason[i]!=AUTO_GEOM_OK) continue;
        out.clearance[i]=measured.required_m>=commanded.required_m?measured:commanded;
        out.valid_mask|=(uint8_t)(1u<<i);
        if(out.clearance[i].protective_stop) out.stop_mask|=(uint8_t)(1u<<i);
    }
    out.required_ready=out.pose_consistent && (out.valid_mask&in.required_mask)==in.required_mask;
    out.protective_stop=out.stop_mask!=0;
    return out;
}
