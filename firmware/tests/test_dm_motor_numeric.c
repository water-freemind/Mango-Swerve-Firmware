#include "dm_motor.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint8_t frame[8], length;
static uint32_t frame_id;
static unsigned frames;
static bool port_ok=true;
static bool send_frame(uint32_t id,const uint8_t* data,uint8_t len) {
    frame_id=id; length=len; memcpy(frame,data,len); ++frames; return port_ok;
}
static const BusMotorPortOps ops={.send=send_frame};
static const BusMotorConfig config={.ops=&ops};
static void setup(void) {
    port_ok=true; frames=0; memset(frame,0,sizeof frame);
    assert(dm_motor_instance.init(&config)==MOTOR_STATUS_OK);
}
static void expect(uint32_t id,const uint8_t* bytes,unsigned size) {
    assert(frame_id==id && length==size && !memcmp(frame,bytes,size));
}
static void normal_packets(void) {
    setup();
    const uint8_t posvel[8]={0,0,0x80,0x3f,0,0,0,0x40};
    assert(dm_motor_instance.set_pos_vel(1,1,2)==MOTOR_STATUS_OK); expect(0x101,posvel,8);
    assert(dm_motor_instance.set_spd(1,12.56f)==MOTOR_STATUS_OK);
    const float requested_speed=12.56f; assert(!memcmp(frame+4,&requested_speed,4)); /* POS_VEL is not MIT's +/-10 encoding. */
    const uint8_t mode[8]={1,0,0x55,10,3,0,0,0};
    assert(dm_motor_instance.switch_mode(1,DM_MOTOR_MODE_SPEED)==MOTOR_STATUS_OK); expect(0x7ff,mode,8);
    const uint8_t speed[4]={0,0,0,0xc0};
    assert(dm_motor_instance.set_spd(1,-2)==MOTOR_STATUS_OK); expect(0x201,speed,4);
    setup();
    assert(dm_motor_instance.switch_mode(1,DM_MOTOR_MODE_MIT)==MOTOR_STATUS_OK);
    const uint8_t mit[8]={0x7f,0xff,0x7f,0xf0,0x18,0x01,0xa7,0xff};
    assert(dm_motor_instance.set_pos(1,0)==MOTOR_STATUS_OK); expect(1,mit,8);
    assert(dm_motor_instance.set_pos_vel(1,FLT_MAX,-FLT_MAX)==MOTOR_STATUS_OK);
    assert(dm_motor_instance.set_tor(1,FLT_MAX)==MOTOR_STATUS_OK);
    assert(dm_motor_instance.set_pd(1,FLT_MAX,-FLT_MAX)==MOTOR_STATUS_OK);
    const uint8_t clamped[8]={0xff,0xff,0,0x0f,0xff,0,0x0f,0xff}; expect(1,clamped,8);
    setup();
    assert(dm_motor_instance.switch_mode(1,DM_MOTOR_MODE_POS_VEL_TOR)==MOTOR_STATUS_OK);
    assert(dm_motor_instance.set_pos_vel(1,1,1.25f)==MOTOR_STATUS_OK);
    assert(dm_motor_instance.set_tor(1,.25f)==MOTOR_STATUS_OK);
    const uint8_t pvt[8]={0,0,0x80,0x3f,0x7d,0,0xc4,9}; expect(0x301,pvt,8);
}
static void invalid_no_send_no_poison(void) {
    setup(); assert(dm_motor_instance.set_pos_vel(1,1,2)==MOTOR_STATUS_OK);
    const unsigned before=frames;
    assert(dm_motor_instance.set_pos(1,NAN)==MOTOR_STATUS_INVALID_PARAM);
    assert(dm_motor_instance.set_spd(1,INFINITY)==MOTOR_STATUS_INVALID_PARAM);
    assert(dm_motor_instance.set_pos_vel(1,3,-INFINITY)==MOTOR_STATUS_INVALID_PARAM);
    assert(dm_motor_instance.set_tor(1,NAN)==MOTOR_STATUS_INVALID_PARAM);
    assert(dm_motor_instance.set_pd(1,NAN,1)==MOTOR_STATUS_INVALID_PARAM);
    assert(dm_motor_instance.set_pd(1,1,INFINITY)==MOTOR_STATUS_INVALID_PARAM);
    assert(frames==before);
    assert(dm_motor_instance.set_spd(1,2)==MOTOR_STATUS_OK);
    const uint8_t original[8]={0,0,0x80,0x3f,0,0,0,0x40}; expect(0x101,original,8);
    assert(dm_motor_instance.switch_mode(1,DM_MOTOR_MODE_MIT)==MOTOR_STATUS_OK);
    unsigned count=frames;
    assert(dm_motor_instance.set_pos(1,NAN)==MOTOR_STATUS_INVALID_PARAM); assert(frames==count);
    assert(dm_motor_instance.set_pos(1,0)==MOTOR_STATUS_OK); /* finite candidate not poisoned */
}
static void scaled_representation_and_modes(void) {
    setup(); assert(dm_motor_instance.switch_mode(1,DM_MOTOR_MODE_POS_VEL_TOR)==MOTOR_STATUS_OK);
    unsigned before=frames;
    assert(dm_motor_instance.set_spd(1,327.68f)==MOTOR_STATUS_INVALID_PARAM);
    assert(dm_motor_instance.set_spd(1,-327.69f)==MOTOR_STATUS_INVALID_PARAM);
    assert(dm_motor_instance.set_spd(1,FLT_MAX)==MOTOR_STATUS_INVALID_PARAM);
    assert(dm_motor_instance.set_tor(1,3.2768f)==MOTOR_STATUS_INVALID_PARAM);
    assert(dm_motor_instance.set_tor(1,-3.2769f)==MOTOR_STATUS_INVALID_PARAM);
    assert(dm_motor_instance.set_tor(1,FLT_MAX)==MOTOR_STATUS_INVALID_PARAM);
    assert(frames==before);
    assert(dm_motor_instance.set_pos_vel(1,0,-327.68f)==MOTOR_STATUS_OK);
    assert(dm_motor_instance.set_tor(1,-3.2768f)==MOTOR_STATUS_OK);
    const int16_t s=(int16_t)((uint16_t)frame[4]|(uint16_t)frame[5]<<8);
    const int16_t t=(int16_t)((uint16_t)frame[6]|(uint16_t)frame[7]<<8);
    assert((s==-32768 || s==-32767) && (t==-32768 || t==-32767)); /* float quantization may truncate one count. */
    setup(); assert(dm_motor_instance.switch_mode(1,DM_MOTOR_MODE_MIT)==MOTOR_STATUS_OK);
    assert(dm_motor_instance.set_spd(1,1000)==MOTOR_STATUS_OK); /* MIT clamps finite values */
    before=frames;
    assert(dm_motor_instance.switch_mode(1,DM_MOTOR_MODE_POS_VEL_TOR)==MOTOR_STATUS_INVALID_PARAM);
    assert(frames==before);
    assert(dm_motor_instance.set_spd(1,0)==MOTOR_STATUS_OK); assert(frame_id==1); /* still MIT */
    assert(dm_motor_instance.switch_mode(1,DM_MOTOR_MODE_POS_VEL_TOR)==MOTOR_STATUS_OK);
    before=frames;
    assert(dm_motor_instance.switch_mode(1,99)==MOTOR_STATUS_UNSUPPORTED); assert(frames==before);
}
static void failed_send_not_committed(void) {
    setup(); assert(dm_motor_instance.set_pos_vel(1,1,2)==MOTOR_STATUS_OK);
    port_ok=false;
    assert(dm_motor_instance.set_pos(1,3)==MOTOR_STATUS_PORT_ERROR);
    assert(dm_motor_instance.switch_mode(1,DM_MOTOR_MODE_SPEED)==MOTOR_STATUS_PORT_ERROR);
    port_ok=true;
    assert(dm_motor_instance.set_spd(1,2)==MOTOR_STATUS_OK);
    const uint8_t original[8]={0,0,0x80,0x3f,0,0,0,0x40}; expect(0x101,original,8);
    assert(dm_motor_instance.stop(1)==MOTOR_STATUS_OK);
    const uint8_t stopped[8]={0,0,0x80,0x3f,0,0,0,0}; expect(0x101,stopped,8);
}
int main(void) {
    normal_packets(); invalid_no_send_no_poison(); scaled_representation_and_modes(); failed_send_not_committed();
    puts("DM_MOTOR_NUMERIC_TEST_OK (normal wire bytes, finite MIT clamps, nonfinite rejection, PVT bounds, transactional modes)");
    return 0;
}
