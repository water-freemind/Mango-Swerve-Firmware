#include "field_pose.h"
#include <math.h>
#include <string.h>

static float laser_yaw(unsigned ch) {
    const float yaw[4]={1.570796327f,-1.570796327f,3.141592654f,0};
    return yaw[ch];
}
static AutoVec2 sensor_offset(unsigned ch, float yaw) {
    /* User CAD centre-to-sensor 40.98mm; effective optical origin still needs
     * field verification. This profile is not the former 153mm layout. */
    const float a=yaw+laser_yaw(ch);
    return (AutoVec2){.04098f*cosf(a),.04098f*sinf(a)};
}
static void hit(AutoVec2 p,AutoVec2 u,AutoVec2 n,float offset,float low,float high,
    unsigned id,AutoPlane* best,float* nearest) {
    const float denominator=n.x*u.x+n.y*u.y;
    if(denominator<.2f) return;
    const float t=(offset-n.x*p.x-n.y*p.y)/denominator;
    const float tangent=n.x!=0 ? p.y+t*u.y:p.x+t*u.x;
    if(t>0 && t<*nearest && tangent>=low && tangent<=high) {
        *nearest=t;*best=(AutoPlane){n,offset,id,true};
    }
}
bool field_pose_ray(AutoPose p,unsigned ch,AutoPlane* plane,float* distance) {
    if(!plane||!distance||ch>=4||!isfinite(p.x)||!isfinite(p.y)||!isfinite(p.yaw)) return false;
    const AutoVec2 o=sensor_offset(ch,p.yaw);
    const AutoVec2 sensor={p.x+o.x,p.y+o.y};
    const float a=p.yaw+laser_yaw(ch);
    const AutoVec2 u={cosf(a),sinf(a)};
    *plane=(AutoPlane){0};*distance=7.5f;
    hit(sensor,u,(AutoVec2){-1,0},0,0,4.6f,1,plane,distance);
    hit(sensor,u,(AutoVec2){1,0},3,0,4.6f,2,plane,distance);
    hit(sensor,u,(AutoVec2){0,1},4.6f,0,3,3,plane,distance);
    hit(sensor,u,(AutoVec2){0,-1},0,.4f,3,4,plane,distance);
    return plane->associated;
}
bool field_pose_register(FieldPose* s,AutoPose p,float yaw,uint32_t now) {
    if(!s||!isfinite(p.x)||!isfinite(p.y)||!isfinite(p.yaw)||!isfinite(yaw)) return false;
    memset(s,0,sizeof(*s));s->pose=(AutoPoseSample){p,now,true};
    s->wheel_position=(AutoVec2){p.x,p.y};
    s->imu_offset=auto_wrap_angle(p.yaw-yaw);s->last_ms=now;s->registered=true;
    s->constrained_ms[0]=s->constrained_ms[1]=now;
    return true;
}
bool field_pose_step_route(FieldPose* s,AutoTwist v,float yaw,bool healthy,const LaserSnapshot* l,uint32_t now,const AutoRoute* route) {
    if(!s||!l||!s->registered) return false;
    s->used_mask=s->rejected_mask=s->constrained_axes=0;
    const uint32_t ms=now-s->last_ms;
    if(!healthy||ms>50||!isfinite(v.vx)||!isfinite(v.vy)||!isfinite(v.wz)||!isfinite(yaw)||
       hypotf(v.vx,v.vy)>2.2f) {
        s->pose.valid=false;s->registered=false;return false;
    }
    const float heading=auto_wrap_angle(yaw+s->imu_offset);
    const float middle=s->pose.pose.yaw+.5f*auto_wrap_angle(heading-s->pose.pose.yaw);
    const AutoVec2 velocity=auto_rotate((AutoVec2){v.vx,v.vy},middle);
    AutoPose prior=s->pose.pose;
    prior.x+=velocity.x*ms*.001f;prior.y+=velocity.y*ms*.001f;prior.yaw=heading;
    s->wheel_position.x+=velocity.x*ms*.001f;s->wheel_position.y+=velocity.y*ms*.001f;
    s->uncorrected_m[0]+=fabsf(velocity.x)*ms*.001f;
    s->uncorrected_m[1]+=fabsf(velocity.y)*ms*.001f;
    s->last_ms=now;
    float dx=0,dy=0,wx=0,wy=0;
    for(unsigned ch=0;ch<4;ch++) {
        const uint8_t bit=(uint8_t)(1u<<ch);const uint32_t age=l->sample_age_ms[ch];
        const uint32_t stamp=l->last_rx_ms; /* Immutable receipt time of updated_mask. */
        if(!l->frame_fresh||!(l->valid_mask&bit)||!(l->updated_mask&bit)||age>LASER_FRAME_TIMEOUT_MS||
           ((s->seen_mask&bit)&&stamp==s->used_stamp[ch])) continue;
        s->seen_mask|=bit;s->used_stamp[ch]=stamp;
        if(!l->accepted_mm[ch]||l->accepted_mm[ch]>7500u||fabsf(v.wz)>.1f) {s->rejected_mask|=bit;continue;}
        AutoPlane plane;float expected;
        bool associated=field_pose_ray(prior,ch,&plane,&expected);
        if(route) {
            const float angle=laser_yaw(ch);
            const AutoLaserMount mount={.origin={.04098f*cosf(angle),.04098f*sinf(angle)},.yaw=angle};
            AutoPlane ridge;float ridge_distance;
            if(auto_route_ray(route,prior,mount,7.5f,&ridge,&ridge_distance) && ridge.surface_id>=10u) {
                const float incidence=ridge.normal.x*cosf(prior.yaw+angle)+ridge.normal.y*sinf(prior.yaw+angle);
                if(incidence>=.2f) {
                    const float corrected=l->accepted_mm[ch]*.001f-
                        (ridge.normal.x*velocity.x+ridge.normal.y*velocity.y)*age*.001f/incidence;
                    if(corrected>0 && fabsf(corrected-ridge_distance)<=.15f) {
                        plane=ridge;expected=ridge_distance;associated=true;
                    }
                }
            }
        }
        if(!associated) continue;
        if((s->checkpoint_mask&bit) && s->checkpoint_surface[ch]==plane.surface_id &&
           fabsf((float)l->accepted_mm[ch]-s->checkpoint_mm[ch])<=5.0f) {
            const AutoVec2 old=s->checkpoint_position[ch];
            const float travel=plane.normal.x*(s->wheel_position.x-old.x)+plane.normal.y*(s->wheel_position.y-old.y);
            if(fabsf(travel)>.03f) {s->rejected_mask|=bit;continue;}
        } else {
            s->checkpoint_mask|=bit;s->checkpoint_surface[ch]=plane.surface_id;
            s->checkpoint_mm[ch]=l->accepted_mm[ch];s->checkpoint_position[ch]=s->wheel_position;
        }
        const float a=prior.yaw+laser_yaw(ch);
        const float incidence=plane.normal.x*cosf(a)+plane.normal.y*sinf(a);
        /* Project a delayed range with measured translation, not commanded speed.
         * Large rotations are rejected above; this is a bounded small-delay model. */
        const float corrected=l->accepted_mm[ch]*.001f-
            (plane.normal.x*velocity.x+plane.normal.y*velocity.y)*age*.001f/incidence;
        const float residual=corrected-expected;
        if(corrected<=0||fabsf(residual)>.15f) {s->rejected_mask|=bit;continue;}
        const float correction=-residual*incidence;
        dx+=plane.normal.x*correction;dy+=plane.normal.y*correction;
        wx+=fabsf(plane.normal.x);wy+=fabsf(plane.normal.y);
        s->used_mask|=bit;
    }
    /* Parallel observations do not double the gain. No invented constraint along
     * a wall; all rejected samples leave a pure wheel/IMU prediction. */
    if(wx>0) {prior.x+=fmaxf(-.005f,fminf(.005f,.15f*dx/wx));s->constrained_axes|=1;s->uncorrected_m[0]=0;s->constrained_ms[0]=now;}
    if(wy>0) {prior.y+=fmaxf(-.005f,fminf(.005f,.15f*dy/wy));s->constrained_axes|=2;s->uncorrected_m[1]=0;s->constrained_ms[1]=now;}
    s->pose=(AutoPoseSample){prior,now,true};return true;
}
bool field_pose_step(FieldPose* s,AutoTwist v,float yaw,bool healthy,const LaserSnapshot* l,uint32_t now) {
    return field_pose_step_route(s,v,yaw,healthy,l,now,NULL);
}
