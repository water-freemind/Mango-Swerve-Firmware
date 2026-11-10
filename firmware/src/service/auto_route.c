#include "auto_route.h"
#include <math.h>
#include <string.h>
#define AUTO_ROUTE_ENTRY_MIN_Y_M (-.80f)
static AutoRouteSegment segment(float x0,float y0,float x1,float y1,float heading,AutoRouteKind kind,unsigned aisle,unsigned row) {
    return (AutoRouteSegment){{{x0,y0},{x1,y1},heading},kind,(uint8_t)aisle,(uint8_t)row};
}
bool auto_route_build(AutoRoute* r,float scale,float heading) {
    if(!r || !isfinite(scale)||scale<.5f||scale>2||!isfinite(heading)) return false;
    memset(r,0,sizeof(*r)); r->scale=scale;r->entry_y=-.5f*scale;
    float l=.2f*scale, right=2.8f*scale;
    unsigned k=0;
    r->segment[k++]=segment(l,r->entry_y,l,.2f*scale,heading,AUTO_ROUTE_ENTER,0,0);
    r->segment[k++]=segment(l,.2f*scale,l,4.4f*scale,heading,AUTO_ROUTE_OUTBOUND,0,0);
    for(int aisle=7;aisle>=1;--aisle) {
        float y=(.2f+.7f*(aisle-1))*scale;
        float x0=aisle%2?l:right, x1=aisle%2?right:l;
        r->segment[k++]=segment(x0,y,x1,y,heading,AUTO_ROUTE_SURVEY,aisle,aisle<7?aisle:0);
        if(aisle>1) r->segment[k++]=segment(x1,y,x1,y-.7f*scale,heading,AUTO_ROUTE_SHIFT,0,0);
    }
    r->segment[k++]=segment(right,.2f*scale,l,.2f*scale,heading,AUTO_ROUTE_RETURN,0,0);
    r->segment[k++]=segment(l,.2f*scale,l,-.5f*scale,heading,AUTO_ROUTE_PARK,0,0);
    return k==AUTO_ROUTE_COUNT;
}
bool auto_route_set_entry_y(AutoRoute* r,float entry_y) {
    if(!r||!isfinite(r->scale)||r->scale<.5f||r->scale>2||!isfinite(entry_y)||
       entry_y<AUTO_ROUTE_ENTRY_MIN_Y_M*r->scale||entry_y>0.0f) return false;
    r->entry_y=entry_y;
    r->segment[0].line.start.y=entry_y;
    return true;
}
bool auto_route_valid(const AutoRoute* r) {
    if(!r||!isfinite(r->entry_y)||r->entry_y<AUTO_ROUTE_ENTRY_MIN_Y_M*r->scale||r->entry_y>0.0f) return false;
    AutoRoute expected;
    if(!auto_route_build(&expected,r->scale,r->segment[0].line.heading)) return false;
    expected.entry_y=r->entry_y;
    expected.segment[0].line.start.y=r->entry_y;
    for(unsigned i=0;i<AUTO_ROUTE_COUNT;++i) {
        AutoRouteSegment a=r->segment[i],b=expected.segment[i];
        const float av[]={a.line.start.x,a.line.start.y,a.line.end.x,a.line.end.y,a.line.heading};
        const float bv[]={b.line.start.x,b.line.start.y,b.line.end.x,b.line.end.y,b.line.heading};
        for(unsigned j=0;j<5;++j) if(!isfinite(av[j])||fabsf(av[j]-bv[j])>.0001f) return false;
        if(a.kind!=b.kind||a.aisle!=b.aisle||a.row!=b.row) return false;
    }
    return true;
}
bool auto_route_clear(const AutoRoute* r,AutoEnvelope body,float margin) {
    if(!auto_route_valid(r)||!isfinite(margin)||margin<0||
       !isfinite(body.front)||!isfinite(body.rear)||!isfinite(body.left)||!isfinite(body.right)||
       body.front<=0||body.rear<=0||body.left<=0||body.right<=0) return false;
    const AutoVec2 axes[4]={{1,0},{-1,0},{0,1},{0,-1}};
    float extent[4],s=r->scale;
    for(unsigned j=0;j<4;++j) extent[j]=auto_envelope_support(body,auto_rotate(axes[j],-r->segment[0].line.heading))+margin;
    for(unsigned i=0;i<AUTO_ROUTE_COUNT;++i) {
        AutoSegment p=r->segment[i].line;
        float xmin=fminf(p.start.x,p.end.x)-extent[1],xmax=fmaxf(p.start.x,p.end.x)+extent[0];
        float ymin=fminf(p.start.y,p.end.y)-extent[3],ymax=fmaxf(p.start.y,p.end.y)+extent[2];
        if(xmin<0||xmax>3*s||ymin< -s||ymax>4.6f*s || (ymin<0&&xmax>.4f*s)) return false;
        for(unsigned row=1;row<=6;++row) {
            float x0=(row==4?.6f:.4f)*s,x1=(row==4?2.4f:2.6f)*s;
            float y0=(.4f+.7f*(row-1))*s,y1=y0+.3f*s;
            if(xmax>=x0&&xmin<=x1&&ymax>=y0&&ymin<=y1) return false;
        }
    }
    return true;
}
bool auto_route_pose_clear(const AutoRoute* r,AutoEnvelope body,AutoPose pose,float margin) {
    if(!auto_route_valid(r)) return false;
    float s=r->scale;
    if(!auto_envelope_inside(body,pose,0,-s,3*s,4.6f*s,margin)) return false;
    float xmin=pose.x-auto_envelope_support(body,auto_rotate((AutoVec2){-1,0},-pose.yaw))-margin;
    float xmax=pose.x+auto_envelope_support(body,auto_rotate((AutoVec2){1,0},-pose.yaw))+margin;
    float ymin=pose.y-auto_envelope_support(body,auto_rotate((AutoVec2){0,-1},-pose.yaw))-margin;
    float ymax=pose.y+auto_envelope_support(body,auto_rotate((AutoVec2){0,1},-pose.yaw))+margin;
    if(ymin<0 && xmax>.4f*s) return false;
    for(unsigned row=1;row<=6;++row) {
        float x0=(row==4?.6f:.4f)*s,x1=(row==4?2.4f:2.6f)*s;
        float y0=(.4f+.7f*(row-1))*s,y1=y0+.3f*s;
        if(xmax>=x0 && xmin<=x1 && ymax>=y0 && ymin<=y1) return false;
    }
    return true;
}
static void intersect(AutoVec2 sensor,AutoVec2 dir,AutoVec2 n,float offset,float low,float high,
                      uint32_t id,float* nearest,AutoPlane* plane) {
    float denom=n.x*dir.x+n.y*dir.y;
    if(denom<=.00001f) return;
    float t=(offset-n.x*sensor.x-n.y*sensor.y)/denom;
    float tangent=n.x!=0?sensor.y+t*dir.y:sensor.x+t*dir.x;
    if(t>0&&t<*nearest&&tangent>=low&&tangent<=high) {
        *nearest=t; *plane=(AutoPlane){n,offset,id,true};
    }
}
bool auto_route_ray(const AutoRoute* r,AutoPose p,AutoLaserMount m,float max_range,AutoPlane* plane,float* distance) {
    if(!plane||!distance) return false;
    *plane=(AutoPlane){0}; *distance=0;
    if(!auto_route_valid(r)||!isfinite(p.x)||!isfinite(p.y)||!isfinite(p.yaw)||
       !isfinite(m.origin.x)||!isfinite(m.origin.y)||!isfinite(m.yaw)||!isfinite(max_range)||max_range<=0) return false;
    AutoVec2 sensor=auto_rotate(m.origin,p.yaw); sensor.x+=p.x; sensor.y+=p.y;
    AutoVec2 dir={cosf(p.yaw+m.yaw),sinf(p.yaw+m.yaw)};
    float s=r->scale,nearest=max_range;
    intersect(sensor,dir,(AutoVec2){-1,0},0,0,4.6f*s,1,&nearest,plane);
    intersect(sensor,dir,(AutoVec2){1,0},3*s,0,4.6f*s,2,&nearest,plane);
    intersect(sensor,dir,(AutoVec2){0,1},4.6f*s,0,3*s,3,&nearest,plane);
    intersect(sensor,dir,(AutoVec2){0,-1},0,.4f*s,3*s,4,&nearest,plane);
    for(unsigned row=1;row<=6;++row) {
        float x0=(row==4?.6f:.4f)*s,x1=(row==4?2.4f:2.6f)*s;
        float y0=(.4f+.7f*(row-1))*s,y1=y0+.3f*s;
        intersect(sensor,dir,(AutoVec2){1,0},x0,y0,y1,10+4*row,&nearest,plane);
        intersect(sensor,dir,(AutoVec2){-1,0},-x1,y0,y1,11+4*row,&nearest,plane);
        intersect(sensor,dir,(AutoVec2){0,1},y0,x0,x1,12+4*row,&nearest,plane);
        intersect(sensor,dir,(AutoVec2){0,-1},-y1,x0,x1,13+4*row,&nearest,plane);
    }
    if(!plane->associated) return false;
    *distance=nearest; return true;
}
