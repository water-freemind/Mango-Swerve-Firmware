#include "steer_wheel_kine.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#undef assert
#define assert(e) do { if(!(e)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#e); exit(1); } } while(0)

static const SteerWheelModel baseline = {.length=.26572986916f,.width=.26572986916f,.wheel_radius=.051f,.max_wheel_linear_speed=1.5f};
static void cleared(const SteerWheel *s) {
    assert(s->control.vx == 0 && s->control.vy == 0 && s->control.wz == 0);
    assert(s->state.cur_vx == 0 && s->state.cur_vy == 0 && s->state.cur_wz == 0);
    for(unsigned i=0;i<4;++i) {
        assert(s->control.wheels[i].wheel_omega == 0);
        assert(isfinite(s->control.wheels[i].steer_angle));
    }
}
static void seed(SteerWheel *s) {
    assert(steer_wheel_init(s, baseline)==STEER_WHEEL_OK);
    s->control.vx=.5f;
    assert(steer_wheel_ik(s)==STEER_WHEEL_OK);
    s->state.cur_vx=.5f;
}
static void invalid_values(void) {
    const float bad[]={NAN,INFINITY,-INFINITY,FLT_MAX,-FLT_MAX};
    SteerWheel s;
    for(unsigned b=0;b<sizeof(bad)/sizeof(*bad);++b) {
        for(unsigned field=0;field<3;++field) {
            seed(&s);
            if(field==0) s.control.vx=bad[b];
            if(field==1) s.control.vy=bad[b];
            if(field==2) s.control.wz=bad[b];
            assert(steer_wheel_ik(&s)==STEER_WHEEL_INVALID_PARAM);
            cleared(&s);
        }
        for(unsigned field=0;field<4;++field) {
            seed(&s);
            SteerWheelModel m=baseline;
            if(field==0) m.length=bad[b];
            if(field==1) m.width=bad[b];
            if(field==2) m.wheel_radius=bad[b];
            if(field==3) m.max_wheel_linear_speed=bad[b];
            assert(steer_wheel_init(&s,m)==STEER_WHEEL_INVALID_MODEL);
            assert(!s.initialized);
            cleared(&s);
            assert(steer_wheel_ik(&s)==STEER_WHEEL_NOT_INITIALIZE);
        }
        for(unsigned wheel=0;wheel<4;++wheel) {
            seed(&s);
            s.state.cur_wheels[wheel].wheel_omega=bad[b];
            assert(steer_wheel_fk(&s)==STEER_WHEEL_INVALID_PARAM);
            cleared(&s);
            seed(&s);
            s.state.cur_wheels[wheel].steer_angle=bad[b];
            assert(steer_wheel_fk(&s)==STEER_WHEEL_INVALID_PARAM);
            cleared(&s);
            seed(&s);
            s.state.cur_wheels[wheel].steer_angle=bad[b];
            s.control.vx=0;
            assert(steer_wheel_ik(&s)==STEER_WHEEL_INVALID_PARAM);
            cleared(&s);
        }
    }
    seed(&s);
    s.model.length=NAN;
    assert(steer_wheel_fk(&s)==STEER_WHEEL_INVALID_MODEL);
    cleared(&s);
    seed(&s);
    s.model.wheel_radius=0;
    assert(steer_wheel_ik(&s)==STEER_WHEEL_INVALID_MODEL);
    cleared(&s);
    seed(&s);
    s.control.vx=NAN;
    assert(steer_wheel_ik(&s)==STEER_WHEEL_INVALID_PARAM);
    /* An explicit new valid command recovers; the old .5m/s does not. */
    assert(steer_wheel_ik(&s)==STEER_WHEEL_OK);
    cleared(&s);
    s.control.vy=.25f;
    assert(steer_wheel_ik(&s)==STEER_WHEEL_OK);
    assert(s.control.wheels[0].wheel_omega>0);
    assert(steer_wheel_init(NULL,baseline)==STEER_WHEEL_INVALID_PARAM);
    assert(steer_wheel_ik(NULL)==STEER_WHEEL_INVALID_PARAM);
    assert(steer_wheel_fk(NULL)==STEER_WHEEL_INVALID_PARAM);
}
static uint32_t rng=789123;
static float sample(void) { rng=rng*1664525u+1013904223u; return ((float)(rng>>8)/16777216.0f)*4.0f-2.0f; }
static void round_trip(void) {
    SteerWheel s;
    for(unsigned n=0;n<1200;++n) {
        SteerWheelModel m=baseline;
        m.length=.1f+.025f*(float)(n%13);
        m.width=.12f+.021f*(float)(n%11);
        m.wheel_radius=.025f+.003f*(float)(n%9);
        m.max_wheel_linear_speed=n%2?0:1.5f;
        assert(steer_wheel_init(&s,m)==STEER_WHEEL_OK);
        float vx=sample(),vy=sample(),wz=sample();
        s.control.vx=vx;s.control.vy=vy;s.control.wz=wz;
        assert(steer_wheel_ik(&s)==STEER_WHEEL_OK);
        float maximum=0;
        for(unsigned i=0;i<4;++i) {
            s.state.cur_wheels[i]=s.control.wheels[i];
            float speed=fabsf(s.control.wheels[i].wheel_omega*m.wheel_radius);
            maximum=fmaxf(maximum,speed);
        }
        assert(m.max_wheel_linear_speed==0 || maximum<=m.max_wheel_linear_speed+1e-5f);
        assert(steer_wheel_fk(&s)==STEER_WHEEL_OK);
        if(maximum<1.49999f || m.max_wheel_linear_speed==0) {
            assert(fabsf(s.state.cur_vx-vx)<1e-5f);
            assert(fabsf(s.state.cur_vy-vy)<1e-5f);
            assert(fabsf(s.state.cur_wz-wz)<2e-5f);
        } else {
            /* Uniform wheel limiting must preserve twist direction and curvature. */
            const float scale=s.state.cur_vx/vx;
            assert(scale>0 && scale<=1.00001f);
            assert(fabsf(s.state.cur_vy-vy*scale)<1e-4f);
            assert(fabsf(s.state.cur_wz-wz*scale)<1e-4f);
        }
    }
}
static void axes_and_stopped_angle(void) {
    SteerWheel s;
    assert(steer_wheel_init(&s,baseline)==STEER_WHEEL_OK);
    s.control.vy=1;
    assert(steer_wheel_ik(&s)==STEER_WHEEL_OK);
    for(unsigned i=0;i<4;++i) assert(fabsf(s.control.wheels[i].steer_angle-1.570796327f)<1e-6f);
    s.control.vy=0;s.control.wz=1;
    assert(steer_wheel_ik(&s)==STEER_WHEEL_OK);
    /* Internal +y is right: positive wz yields (+,+) at FL. Service changes signs. */
    assert(s.control.wheels[0].steer_angle>0 && s.control.wheels[0].steer_angle<1.5708f);
    assert(s.control.wheels[1].steer_angle>1.5708f);
    s.control.wz=0;
    const float a[]={3.1415f,-3.1415f,12.0f,-12.0f};
    for(unsigned i=0;i<4;++i) s.state.cur_wheels[i].steer_angle=a[i];
    assert(steer_wheel_ik(&s)==STEER_WHEEL_OK);
    for(unsigned i=0;i<4;++i) {
        assert(s.control.wheels[i].wheel_omega==0);
        assert(fabsf(remainderf(s.control.wheels[i].steer_angle-a[i],6.283185307f))<1e-5f);
    }
}
static void numeric_boundaries(void) {
    SteerWheel s;
    SteerWheelModel m=baseline;
    const float bad_span[]={0,-.1f,.0001f,100.1f};
    for(unsigned i=0;i<4;++i) {
        m.length=bad_span[i];
        assert(!steer_wheel_model_valid(&m));
    }
    m=baseline;m.wheel_radius=.00001f;
    assert(!steer_wheel_model_valid(&m));
    m=baseline;m.max_wheel_linear_speed=101;
    assert(!steer_wheel_model_valid(&m));
    assert(!steer_wheel_model_valid(NULL));
    m=baseline;m.wheel_radius=.0001f;
    assert(steer_wheel_model_valid(&m));
    m.length=m.width=100.0f;m.wheel_radius=10.0f;m.max_wheel_linear_speed=100.0f;
    assert(steer_wheel_model_valid(&m));
    assert(!steer_wheel_twist_valid(100.01f,0,0));
    assert(!steer_wheel_twist_valid(0,-100.01f,0));
    assert(!steer_wheel_twist_valid(0,0,1000.1f));
    assert(steer_wheel_twist_valid(100,-100,1000));
    /* Very small positive limits still apply, zero explicitly disables limiting. */
    m=baseline;m.max_wheel_linear_speed=.0000001f;
    assert(steer_wheel_init(&s,m)==STEER_WHEEL_OK);
    s.control.vx=1;
    assert(steer_wheel_ik(&s)==STEER_WHEEL_OK);
    for(unsigned i=0;i<4;++i) assert(s.control.wheels[i].wheel_omega*m.wheel_radius<=.00000011f);
    /* Small accepted geometry must yield a finite and correct angular inverse. */
    m=baseline;m.length=m.width=.001f;m.max_wheel_linear_speed=0;
    assert(steer_wheel_init(&s,m)==STEER_WHEEL_OK);
    s.control.wz=1;
    assert(steer_wheel_ik(&s)==STEER_WHEEL_OK);
    memcpy(s.state.cur_wheels,s.control.wheels,sizeof(s.state.cur_wheels));
    assert(steer_wheel_fk(&s)==STEER_WHEEL_OK);
    assert(fabsf(s.state.cur_wz-1)<1e-5f);
    seed(&s);
    s.state.cur_wheels[0].wheel_omega=101/baseline.wheel_radius;
    assert(steer_wheel_fk(&s)==STEER_WHEEL_INVALID_PARAM);
    cleared(&s);
    /* Failed init on an already live object cannot preserve executable targets. */
    seed(&s);m=baseline;m.width=0;
    assert(steer_wheel_init(&s,m)==STEER_WHEEL_INVALID_MODEL);
    cleared(&s);assert(!s.initialized);
}
int main(void) { invalid_values();round_trip();axes_and_stopped_angle();numeric_boundaries();puts("STEER_WHEEL_KINE_OK");return 0; }
