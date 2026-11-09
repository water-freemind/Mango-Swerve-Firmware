#include "chassis.h"
#include "app_config.h"
#include "log.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#undef assert
#define assert(x) do { if(!(x)) { fprintf(stderr,"CHECK_FAILED %s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while(0)

static uint32_t tick;
static float raw[4], measured_speed, angle[4], velocity[4], steer_limit[4];
static bool seen = true, output_error, stop_error;
static unsigned stops, writes;
static uint32_t now_ms(void) { return tick; }
static BusMotorStatus init_motor(const BusMotorConfig* c) { (void)c; return MOTOR_STATUS_OK; }
static BusMotorStatus prepare(uint16_t id) { (void)id; return MOTOR_STATUS_OK; }
static BusMotorStatus feedback(uint16_t id, BusMotorFeedback* f) {
    if(!seen) return MOTOR_STATUS_NO_FEEDBACK;
    if(f) *f = (BusMotorFeedback){.id=id, .position=raw[id-1]};
    return MOTOR_STATUS_OK;
}
static float position(uint16_t id) { return raw[id-1]; }
static float speed(uint16_t id) { (void)id; return measured_speed; }
static BusMotorStatus set_position(uint16_t id, float v, float limit) {
    assert(isfinite(v) && isfinite(limit));
    angle[id-1] = v; steer_limit[id-1] = limit; ++writes;
    return output_error ? MOTOR_STATUS_PORT_ERROR : MOTOR_STATUS_OK;
}
static BusMotorStatus set_speed(uint16_t id, float v) {
    assert(isfinite(v)); velocity[id-1] = v;
    return output_error ? MOTOR_STATUS_PORT_ERROR : MOTOR_STATUS_OK;
}
static BusMotorStatus stop(uint16_t id) { velocity[id-1] = 0; ++stops; return stop_error ? MOTOR_STATUS_PORT_ERROR : MOTOR_STATUS_OK; }
static bool send_frame(uint32_t id, const uint8_t* p, uint8_t n) { (void)id; (void)p; (void)n; return true; }
LogStatus log_info(const char* f, ...) { (void)f; return LOG_STATUS_OK; }
LogStatus log_warn(const char* f, ...) { (void)f; return LOG_STATUS_OK; }
LogStatus log_error(const char* f, ...) { (void)f; return LOG_STATUS_OK; }
bool work_link_hold(void) { return false; }
void work_link_limit_velocity(float* x,float* y,float* w) { (void)x; (void)y; (void)w; }
static const BusMotorInterface motor = {
    .init=init_motor, .update_feedback=feedback, .get_pos=position, .get_spd=speed,
    .set_pos_vel=set_position, .set_spd=set_speed, .stop=stop, .brake=stop
};
static const BusMotorPortOps port = {.send=send_frame, .now_ms=now_ms};
static ChassisConfig config(void) {
    return (ChassisConfig){.steer_motor_interface=&motor, .drive_motor_interface=&motor,
        .steer_ops=&port, .drive_ops=&port, .prepare_steer_motor=prepare, .prepare_drive_motor=prepare,
        .model={.length=.3f,.width=.3f,.wheel_radius=.051f,.max_wheel_linear_speed=1.5f},
        .wheel_drive_ratio=1, .steer_target_mode=CHASSIS_STEER_TARGET_ABS_NEAREST};
}
static void setup(void) {
    memset(raw,0,sizeof raw); memset(velocity,0,sizeof velocity);
    tick=1000; measured_speed=0; seen=true; output_error=stop_error=false; stops=writes=0;
    ChassisConfig c=config(); assert(chassis.init(&c)==chassis.OK);
    assert(chassis.process()==chassis.OK);
}
static void all_zero(void) { for(unsigned i=0;i<4;i++) assert(velocity[i]==0); }
static void move_forward(void) {
    assert(chassis.set_velocity(.1f,0,0)==chassis.OK);
    tick+=2; assert(chassis.process()==chassis.OK);
}
static void invalid_tests(void) {
    const float bad[]={NAN,INFINITY,-INFINITY,FLT_MAX};
    for(unsigned n=0;n<4;n++) {
        setup(); move_forward();
        assert(chassis.set_velocity(bad[n],0,0)!=chassis.OK); all_zero();
        tick+=2; assert(chassis.process()==chassis.OK); all_zero();
        assert(chassis.get_control()->vx==0);
    }
    setup(); move_forward(); raw[2]=NAN; tick+=2;
    assert(chassis.process()!=chassis.OK); all_zero();
    raw[2]=0; tick+=2; assert(chassis.process()==chassis.OK); all_zero();
    setup(); move_forward(); measured_speed=INFINITY; tick+=2;
    assert(chassis.process()!=chassis.OK); all_zero();
    setup(); move_forward(); output_error=true; tick+=2;
    assert(chassis.process()!=chassis.OK); all_zero();
    setup(); move_forward(); seen=false; tick+=2;
    (void)chassis.process(); all_zero();
    seen=true; tick+=2; assert(chassis.process()==chassis.OK); all_zero();
    setup(); move_forward(); ChassisConfig c=config(); c.model.width=NAN;
    assert(chassis.init(&c)!=chassis.OK); all_zero(); assert(!chassis.is_ready());
    setup(); move_forward(); stop_error=true; stops=0;
    assert(chassis.stop()==chassis.MOTOR_OUTPUT_FAILED); assert(stops==8); all_zero();
    assert(chassis.get_control()->vx==0);
}
static float turn_after_100ms(unsigned step) {
    setup(); assert(chassis.set_velocity(0,.1f,0)==chassis.OK);
    for(unsigned t=0;t<100;t+=step) { tick+=step; assert(chassis.process()==chassis.OK); }
    return steer_limit[0];
}
static void timing_tests(void) {
    const float fast=turn_after_100ms(2), slow=turn_after_100ms(10);
    assert(fabsf(fast-slow)<.001f);
    setup(); move_forward(); unsigned count=writes;
    assert(chassis.process()==chassis.OK); assert(writes==count);
#if APP_RUN_MODE == APP_MODE_STRAIGHT_TEST
    assert(chassis_drive_output_allowed());
#endif
    tick+=51; assert(chassis.process()!=chassis.OK); all_zero();
    tick+=2; assert(chassis.process()==chassis.OK); all_zero();
    move_forward(); tick-=10; assert(chassis.process()!=chassis.OK); all_zero();
    setup(); assert(chassis.stop()==chassis.OK);
    tick=UINT32_MAX-1; move_forward(); /* restart clock after explicit stop */
    tick+=2; assert(chassis.process()==chassis.OK); /* unsigned clock rollover */
}
static void steering_tests(void) {
    setup(); move_forward();
    for(unsigned n=0;n<30;n++) {
        assert(chassis.set_velocity(1e-5f,(n&1)?1e-5f:-1e-5f,0)==chassis.OK);
        tick+=2; assert(chassis.process()==chassis.OK); all_zero();
        assert(fabsf(angle[0])<.001f);
    }
    setup(); assert(chassis.set_velocity(0,.1f,0)==chassis.OK);
    tick+=2; assert(chassis.process()==chassis.OK); all_zero();
    /* Enter tighter than exit: old single threshold must not chatter. */
    for(unsigned i=0;i<4;i++) raw[i]=angle[i]-.14f;
    tick+=2; assert(chassis.process()==chassis.OK); all_zero();
    for(unsigned i=0;i<4;i++) raw[i]=angle[i]-.10f;
    tick+=2; assert(chassis.process()==chassis.OK);
    assert(fabsf(velocity[0])>0);
    for(unsigned i=0;i<4;i++) raw[i]=angle[i]-.14f;
    tick+=2; assert(chassis.process()==chassis.OK); assert(fabsf(velocity[0])>0);
    for(unsigned i=0;i<4;i++) raw[i]=angle[i]-.17f;
    tick+=2; assert(chassis.process()==chassis.OK); all_zero();
    setup(); for(unsigned i=0;i<4;i++) raw[i]=3.13f;
    assert(chassis.set_velocity(-.1f,-.001f,0)==chassis.OK);
    tick+=2; assert(chassis.process()==chassis.OK);
    assert(fabsf(angle[0]-raw[0])<.04f);
}
int main(void) {
    invalid_tests(); timing_tests(); steering_tests();
    setup();
    assert(chassis_set_wheel_speed_limit(1.0f) == chassis.OK);
    assert(chassis.get_chassis()->kine.model.max_wheel_linear_speed == 1.0f);
    assert(chassis_set_wheel_speed_limit(NAN) == chassis.INVALID_PARAM);
    assert(chassis_set_wheel_speed_limit(0.0f) == chassis.INVALID_PARAM);
    assert(chassis_set_wheel_speed_limit(2.0f) == chassis.INVALID_PARAM);
    assert(chassis.get_chassis()->kine.model.max_wheel_linear_speed == 1.0f);
    assert(chassis_set_wheel_speed_limit(1.5f) == chassis.OK);
    puts("CHASSIS_CONTROL_TEST_OK numeric, timing, stale-output, alignment and angle-wrap");
    return 0;
}
