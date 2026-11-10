#include "auto_motion.h"
#include <math.h>
#include <string.h>

static bool positive(float x) { return isfinite(x) && x>0; }
static float clamp(float x,float limit) { return fmaxf(-limit,fminf(limit,x)); }
static bool finite_twist(AutoTwist t) { return isfinite(t.vx)&&isfinite(t.vy)&&isfinite(t.wz); }
static void zero(AutoMotion* m) { m->velocity_map=(AutoVec2){0}; m->out.command=m->out.steering_hint=(AutoTwist){0}; m->settled_s=0; }
static void fault(AutoMotion* m,AutoMotionError e) { zero(m); m->out.phase=AUTO_MOTION_FAULT; m->out.error=e; }
bool auto_motion_begin(AutoMotion* m,const AutoMotionConfig* c,AutoSegment s,uint32_t now) {
    if(!m) return false;
    memset(m,0,sizeof(*m));
    if(!c || !positive(c->vmax)||!positive(c->accel)||!positive(c->decel)||!positive(c->lateral_max)||
       !positive(c->cross_kp)||!positive(c->yaw_kp)||!positive(c->wz_max)||!positive(c->angular_accel)||
       !positive(c->position_tolerance)||!positive(c->cross_tolerance)||!positive(c->yaw_tolerance)||
       !positive(c->cross_abort)||!positive(c->yaw_abort)||!positive(c->settle_s)||
       !isfinite(c->stop_latency_s)||c->stop_latency_s<0||c->stop_latency_s>2||
       !isfinite(c->finish_yaw_tolerance)||c->finish_yaw_tolerance<0||
       !isfinite(c->finish_cross_tolerance)||c->finish_cross_tolerance<0||
       (c->finish_cross_tolerance>0 && (c->finish_cross_tolerance<c->cross_tolerance||c->finish_cross_tolerance>=c->cross_abort))||
       (c->finish_yaw_tolerance>0 && (c->finish_yaw_tolerance<c->yaw_tolerance||c->finish_yaw_tolerance>=c->yaw_abort))||
       c->cross_abort<=c->cross_tolerance||c->yaw_abort<=c->yaw_tolerance||
       !c->pose_max_age_ms||c->pose_max_age_ms>10000||!isfinite(s.heading)||
       !isfinite(s.start.x)||!isfinite(s.start.y)||!isfinite(s.end.x)||!isfinite(s.end.y)) {
        fault(m,AUTO_MOTION_BAD_CONFIG); return false;
    }
    float dx=s.end.x-s.start.x,dy=s.end.y-s.start.y;
    m->length=hypotf(dx,dy);
    if(!positive(m->length)) { fault(m,AUTO_MOTION_BAD_CONFIG); return false; }
    m->config=*c; m->segment=s; m->tangent=(AutoVec2){dx/m->length,dy/m->length};
    m->normal=(AutoVec2){-m->tangent.y,m->tangent.x}; m->last_ms=now;
    m->out.phase=AUTO_MOTION_ALIGN; m->initialized=true; return true;
}
void auto_motion_cancel(AutoMotion* m) {
    if(!m) return;
    zero(m); m->initialized=false; m->out.phase=AUTO_MOTION_IDLE; m->out.error=AUTO_MOTION_NO_ERROR;
}
AutoMotionOutput auto_motion_step(AutoMotion* m,AutoMotionInput in) {
    if(!m) return (AutoMotionOutput){.phase=AUTO_MOTION_FAULT,.error=AUTO_MOTION_BAD_CONFIG};
    if(!m->initialized || m->out.phase==AUTO_MOTION_FAULT || m->out.phase==AUTO_MOTION_DONE) return m->out;
    if(in.hard_stop || !in.healthy) { fault(m,AUTO_MOTION_STOPPED_BY_SAFETY); return m->out; }
    uint32_t ms=(uint32_t)(in.now_ms-m->last_ms);
    if(ms>50u || !finite_twist(in.measured) || !isfinite(in.pose.pose.x)||
       !isfinite(in.pose.pose.y)||!isfinite(in.pose.pose.yaw)) { fault(m,AUTO_MOTION_BAD_INPUT); return m->out; }
    if(!in.pose.valid || (uint32_t)(in.now_ms-in.pose.stamp_ms)>m->config.pose_max_age_ms) {
        fault(m,AUTO_MOTION_STALE_POSE); return m->out;
    }
    float dx=in.pose.pose.x-m->segment.start.x,dy=in.pose.pose.y-m->segment.start.y;
    m->out.progress=dx*m->tangent.x+dy*m->tangent.y;
    m->out.cross_error=dx*m->normal.x+dy*m->normal.y;
    m->out.heading_error=auto_wrap_angle(m->segment.heading-in.pose.pose.yaw);
    if(!isfinite(m->out.progress)||!isfinite(m->out.cross_error)||!isfinite(m->out.heading_error)) {
        fault(m,AUTO_MOTION_BAD_INPUT); return m->out;
    }
    if(fabsf(m->out.cross_error)>m->config.cross_abort || fabsf(m->out.heading_error)>m->config.yaw_abort) {
        fault(m,AUTO_MOTION_DEVIATION); return m->out;
    }
    float remaining=m->length-m->out.progress;
    if(remaining < -m->config.position_tolerance) { fault(m,AUTO_MOTION_OVERSHOOT); return m->out; }
    m->last_ms=in.now_ms;
    if(!ms) return m->out;
    float dt=ms*.001f;
    bool measured_zero=hypotf(in.measured.vx,in.measured.vy)<.01f && fabsf(in.measured.wz)<.02f;
    if(!m->started) {
        if(!in.stationary || !measured_zero) { zero(m); m->out.phase=AUTO_MOTION_ALIGN; return m->out; }
        m->started=true;
    }
    const float brake_distance=fmaxf(0,remaining-.5f*m->config.position_tolerance);
    const float delay_speed=m->config.decel*m->config.stop_latency_s;
    /* Solve d = v*T + v*v/(2*a) for the target speed. Subtracting the
     * current speed's delay distance made the target oscillate near a stop. */
    float along=fminf(m->config.vmax,
        sqrtf(delay_speed*delay_speed+2*m->config.decel*brake_distance)-delay_speed);
    along=fminf(along,2*fmaxf(0,remaining)); /* low-gain endpoint approach */
    if(fabsf(remaining)<=m->config.position_tolerance) along=0;
    if(in.hold_along) along=0;
    float cross_error=m->out.cross_error,heading_error=m->out.heading_error;
    if(m->config.smooth_correction) {
        cross_error=copysignf(fmaxf(0,fabsf(cross_error)-m->config.cross_tolerance),cross_error);
        heading_error=copysignf(fmaxf(0,fabsf(heading_error)-m->config.yaw_tolerance),heading_error);
    }
    float lateral=clamp(-m->config.cross_kp*cross_error,m->config.lateral_max);
    if(fabsf(m->out.cross_error)<=m->config.cross_tolerance) lateral=0;
    float wz=clamp(m->config.yaw_kp*heading_error,m->config.wz_max);
    if(fabsf(m->out.heading_error)<=m->config.yaw_tolerance) wz=0;
    const float finish_yaw=m->config.finish_yaw_tolerance>0?m->config.finish_yaw_tolerance:m->config.yaw_tolerance;
    const float finish_cross=m->config.finish_cross_tolerance>0?m->config.finish_cross_tolerance:m->config.cross_tolerance;
    /* A centering hold must keep correcting to cross_tolerance even inside
     * the wider endpoint acceptance window. Completion waits for release. */
    const bool position_reached=!in.hold_along && fabsf(remaining)<=m->config.position_tolerance &&
        fabsf(m->out.cross_error)<=finish_cross;
    if(position_reached) lateral=0;
    if(position_reached && fabsf(m->out.heading_error)<=finish_yaw) wz=0;
    AutoVec2 target={along*m->tangent.x+lateral*m->normal.x,along*m->tangent.y+lateral*m->normal.y};
    float target_norm=hypotf(target.x,target.y);
    if(target_norm>m->config.vmax) { target.x*=m->config.vmax/target_norm; target.y*=m->config.vmax/target_norm; }
    if(in.stop_requested) { target=(AutoVec2){0}; wz=0; }
    /* Heading feedback can change sign while translation continues. Its own
     * angular slew crosses zero without stopping the translational profile. */
    if(target.x*m->velocity_map.x+target.y*m->velocity_map.y<0) m->reversing=true;
    if(m->reversing) { target=(AutoVec2){0}; wz=0; }
    const bool zero_target=target.x==0 && target.y==0 && wz==0;
    const AutoVec2 requested_body=auto_rotate(target,-in.pose.pose.yaw);
    const AutoTwist requested_hint={requested_body.x,requested_body.y,wz};
    const AutoTwist prepared=m->out.steering_hint;
    const bool prepared_translation=prepared.vx!=0 || prepared.vy!=0;
    const bool prepared_valid=m->config.smooth_correction && m->out.phase==AUTO_MOTION_ALIGN &&
        (prepared_translation || prepared.wz!=0);
    const bool compatible=prepared_translation==(target.x!=0 || target.y!=0) &&
        requested_body.x*prepared.vx+requested_body.y*prepared.vy>=0 && wz*prepared.wz>=0;
    if((!in.aligned || (prepared_valid && !compatible)) && !zero_target) {
        AutoTwist hint=requested_hint;
        /* Hold a feasible direction long enough for the wheel servos to catch
         * up. Reversals immediately replace it and require a new zero-drive cycle. */
        if(prepared_valid && compatible && in.now_ms-m->prepare_ms<250u) hint=prepared;
        else m->prepare_ms=in.now_ms;
        zero(m); m->out.phase=AUTO_MOTION_ALIGN;
        m->out.steering_hint=hint;
        return m->out; /* never accumulate a command behind closed wheel gate */
    }
    /* A stationary goal needs no new wheel direction. Closed alignment still
     * enforces zero drive while allowing the ordinary settle timer to finish. */
    if(!in.aligned) {m->velocity_map=(AutoVec2){0};m->out.command=(AutoTwist){0};}
    if(prepared_valid && !zero_target) {
        const float norm=hypotf(prepared.vx,prepared.vy);
        const float scale=norm>m->config.vmax?m->config.vmax/norm:1;
        target=auto_rotate((AutoVec2){prepared.vx*scale,prepared.vy*scale},in.pose.pose.yaw);
        wz=prepared.wz*scale;
    }
    m->out.steering_hint=(AutoTwist){0};
    m->out.phase=in.stop_requested?AUTO_MOTION_HOLD:AUTO_MOTION_RUN;
    float limit=(hypotf(target.x,target.y)<hypotf(m->velocity_map.x,m->velocity_map.y)?m->config.decel:m->config.accel)*dt;
    float dvx=target.x-m->velocity_map.x,dvy=target.y-m->velocity_map.y,dv=hypotf(dvx,dvy);
    float scale=dv>limit ? limit/dv:1;
    const float dw=wz-m->out.command.wz;
    const float angular_limit=m->config.angular_accel*dt;
    if(fabsf(dw)>angular_limit) scale=fminf(scale,angular_limit/fabsf(dw));
    /* A shared progress fraction preserves the prepared wheel directions on
     * launch, while satisfying both linear and angular acceleration limits. */
    m->velocity_map.x+=dvx*scale; m->velocity_map.y+=dvy*scale;
    AutoVec2 body=auto_rotate(m->velocity_map,-in.pose.pose.yaw);
    m->out.command=(AutoTwist){body.x,body.y,m->out.command.wz+dw*scale};
    if(!finite_twist(m->out.command)) { fault(m,AUTO_MOTION_BAD_INPUT); return m->out; }
    bool reached=position_reached && fabsf(m->out.heading_error)<=finish_yaw;
    bool command_zero=hypotf(body.x,body.y)<.00001f && fabsf(m->out.command.wz)<.00001f;
    if(m->reversing && command_zero && in.stationary && measured_zero) m->reversing=false;
    if(!in.stop_requested && !in.hold_along && reached && command_zero && in.stationary && measured_zero) m->settled_s+=dt;
    else m->settled_s=0;
    if(m->settled_s>=m->config.settle_s) { zero(m); m->out.phase=AUTO_MOTION_DONE; }
    return m->out;
}
