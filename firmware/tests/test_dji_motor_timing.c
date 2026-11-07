#include "dji_motor.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t clock_ms;
static uint8_t sent[8];
static unsigned tx_count;
static bool tx_ok = true;
static uint32_t now(void) { return clock_ms; }
static bool send_frame(uint32_t id, const uint8_t* data, uint8_t len) {
    assert(id == 0x200 && len == 8);
    memcpy(sent, data, 8);
    ++tx_count;
    return tx_ok;
}
static const BusMotorPortOps port = {.send=send_frame, .now_ms=now};
static const DjiMotorConfig model = {.model=DJI_MOTOR_MODEL_M3508};
static const BusMotorConfig config = {.ops=&port, .driver_config=&model};
static int current(unsigned id) {
    unsigned ix = (id - 1u) * 2u;
    return (int16_t)((uint16_t)sent[ix] << 8 | sent[ix+1u]);
}
static float shaft_speed(float rotor_rpm) { return rotor_rpm * (6.28318530718f/60.0f) / (3591.0f/187.0f); }
static void feedback(unsigned id, int rpm) {
    uint8_t data[8] = {0};
    data[2] = (uint8_t)((uint16_t)rpm >> 8);
    data[3] = (uint8_t)rpm;
    assert(dji_motor_parse_feedback_frame(0x200u+id,data,NULL)==MOTOR_STATUS_OK);
}
static void all_feedback(int rpm) { for(unsigned i=1;i<=4;i++) feedback(i,rpm); }
static void setup(void) {
    clock_ms=100u; tx_count=0; tx_ok=true;
    memset(sent,0,sizeof sent);
    assert(dji_motor_instance.init(&config)==MOTOR_STATUS_OK);
    all_feedback(0);
}
static void command(float rpm) {
    for(unsigned i=1;i<=4;i++) assert(dji_motor_instance.set_spd(i,shaft_speed(rpm))==MOTOR_STATUS_OK);
}
static void zero_current(void) { for(unsigned i=1;i<=4;i++) assert(current(i)==0); }

static void duplicate_and_baseline(void) {
    setup(); command(10);
    assert(abs(current(1)-81)<=1); /* Existing 2ms gain: 8*10 + .1*10. */
    for(unsigned n=0;n<100;n++) command(10);
    assert(abs(current(1)-81)<=1); /* No time elapsed: must not wind up. */
    clock_ms+=2; all_feedback(0); command(10);
    assert(abs(current(1)-82)<=1);
    command(-10); /* Changed target at same timestamp: proportional response now. */
    assert(current(1)<0);
}
static int run_trace(unsigned period, bool jitter) {
    setup(); command(10);
    unsigned elapsed=0, ix=0;
    const unsigned steps[]={1,3,7,9};
    while(elapsed<80) {
        unsigned step=jitter ? steps[ix++%4] : period;
        if(step>80-elapsed) step=80-elapsed;
        elapsed+=step; clock_ms+=step; all_feedback(0); command(10);
    }
    return current(1);
}
static void elapsed_invariance(void) {
    const int a=run_trace(2,false), b=run_trace(10,false), c=run_trace(0,true);
    assert(abs(a-b)<=1 && abs(a-c)<=1);
    assert(a>=120 && a<=122);
}
static void derivative_normalization(void) {
    setup(); assert(dji_motor_instance.set_pd(1,0,100)==MOTOR_STATUS_OK);
    command(10); clock_ms+=2; command(20);
    assert(abs(current(1)-1003)<=1);
    setup(); assert(dji_motor_instance.set_pd(1,0,100)==MOTOR_STATUS_OK);
    command(10); clock_ms+=10; command(20);
    assert(abs(current(1)-211)<=1);
    command(20); /* Same timestamp: D=0 and no extra integral. */
    assert(abs(current(1)-11)<=1);
}
static void stale_and_gaps(void) {
    setup(); command(10);
    clock_ms+=101;
    BusMotorFeedback out={0};
    assert(dji_motor_instance.update_feedback(1,&out)==MOTOR_STATUS_TIMEOUT);
    assert(dji_motor_instance.get_spd(1)==0);
    assert(dji_motor_instance.set_spd(1,shaft_speed(10))==MOTOR_STATUS_TIMEOUT);
    zero_current();
    all_feedback(0);
    command(10); assert(abs(current(1)-81)<=1); /* No old integral after fault. */
    clock_ms+=51; all_feedback(0);
    assert(dji_motor_instance.set_spd(1,shaft_speed(10))==MOTOR_STATUS_TIMEOUT);
    zero_current(); command(10); assert(abs(current(1)-81)<=1);
    clock_ms-=1; all_feedback(0);
    assert(dji_motor_instance.set_spd(1,shaft_speed(10))==MOTOR_STATUS_TIMEOUT);
    zero_current();
    setup(); clock_ms=UINT32_MAX-1; all_feedback(0);
    for(unsigned i=1;i<=4;i++) assert(dji_motor_instance.stop(i)==MOTOR_STATUS_OK);
    command(10); clock_ms+=2; all_feedback(0); command(10);
    assert(abs(current(1)-82)<=1); /* Natural timestamp wrap is a valid 2ms. */
}
static void stop_disable_restart(void) {
    setup(); command(10);
    for(unsigned i=0;i<10;i++) { clock_ms+=2; all_feedback(0); command(10); }
    for(unsigned i=1;i<=4;i++) assert(dji_motor_instance.stop(i)==MOTOR_STATUS_OK);
    zero_current(); clock_ms+=2; command(10); assert(abs(current(1)-81)<=1);
    for(unsigned i=1;i<=4;i++) assert(dji_motor_instance.disable(i)==MOTOR_STATUS_OK);
    zero_current(); clock_ms+=2; command(10); assert(abs(current(1)-81)<=1);
    clock_ms+=2; all_feedback(100);
    for(unsigned i=1;i<=4;i++) assert(dji_motor_instance.brake(i)==MOTOR_STATUS_OK);
    assert(current(1)<0); /* Stop remains closed-loop braking when feedback exists. */
    clock_ms+=101;
    for(unsigned i=1;i<=4;i++) assert(dji_motor_instance.stop(i)==MOTOR_STATUS_OK);
    zero_current();
}
static void invalid_and_tx(void) {
    setup(); command(10);
    assert(dji_motor_instance.set_spd(1,NAN)==MOTOR_STATUS_INVALID_PARAM); zero_current();
    assert(dji_motor_instance.set_spd(1,INFINITY)==MOTOR_STATUS_INVALID_PARAM); zero_current();
    assert(dji_motor_instance.set_spd(1,FLT_MAX)==MOTOR_STATUS_INVALID_PARAM); zero_current();
    assert(dji_motor_instance.set_pd(1,NAN,0)==MOTOR_STATUS_INVALID_PARAM);
    assert(dji_motor_instance.set_pd(1,1,-1)==MOTOR_STATUS_INVALID_PARAM);
    uint8_t bad[8]={0xff,0xff,0,0,0,0,0,0};
    assert(dji_motor_parse_feedback_frame(0x201,bad,NULL)==MOTOR_STATUS_INVALID_PARAM);
    assert(dji_motor_instance.set_spd(1,shaft_speed(10))==MOTOR_STATUS_NO_FEEDBACK);
    zero_current(); all_feedback(0); command(10);
    tx_ok=false; clock_ms+=2;
    for(unsigned i=1;i<=3;i++) assert(dji_motor_instance.set_spd(i,shaft_speed(10))==MOTOR_STATUS_OK);
    assert(dji_motor_instance.set_spd(4,shaft_speed(10))==MOTOR_STATUS_PORT_ERROR);
    tx_ok=true; clock_ms+=2; command(10); assert(abs(current(1)-81)<=1);
    BusMotorPortOps no_clock=port; no_clock.now_ms=NULL;
    BusMotorConfig bad_config=config; bad_config.ops=&no_clock;
    assert(dji_motor_instance.init(&bad_config)==MOTOR_STATUS_PORT_ERROR);
    zero_current(); /* Reinitialization cannot leave the previous current frame active. */
    assert(dji_motor_instance.set_spd(1,0)==MOTOR_STATUS_NOT_INITIALIZE);
    assert(dji_motor_instance.get_pos(1)==0);
    setup();
    const unsigned before=tx_count;
    for(unsigned i=1;i<=3;i++) assert(dji_motor_instance.set_spd(i,shaft_speed(10))==MOTOR_STATUS_OK);
    assert(tx_count==before); /* Four-current frame is still pending. */
    assert(dji_motor_instance.set_spd(4,NAN)==MOTOR_STATUS_INVALID_PARAM);
    assert(tx_count==before+1); zero_current(); /* Invalid fourth target must not flush first three. */
}
static void saturation_and_repeated_feedback(void) {
    setup(); command(5000); assert(current(1)==16384);
    for(unsigned k=0;k<10;k++) { clock_ms+=2; all_feedback(0); command(5000); }
    clock_ms+=2; command(0); assert(current(1)==0); /* No accumulated integral while saturated. */
    setup(); command(10);
    for(unsigned k=0;k<10;k++) { clock_ms+=2; command(10); }
    assert(abs(current(1)-91)<=1); /* Bounded sample-and-hold feedback is allowed until expiry. */
}
int main(void) {
    duplicate_and_baseline(); elapsed_invariance(); derivative_normalization(); stale_and_gaps();
    stop_disable_restart(); invalid_and_tx(); saturation_and_repeated_feedback();
    puts("DJI_MOTOR_TIMING_TEST_OK");
    return 0;
}
