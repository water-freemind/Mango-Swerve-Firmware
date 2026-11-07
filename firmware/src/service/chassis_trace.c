#include "chassis_trace.h"
#include <stdio.h>
#include <string.h>

void chassis_trace_init(ChassisTrace* trace) {
    if(trace) { memset(trace,0,sizeof(*trace)); trace->next_sequence=1u; }
}
uint32_t chassis_trace_oldest(const ChassisTrace* trace) {
    return trace ? trace->next_sequence-trace->count : 0;
}
bool chassis_trace_get(const ChassisTrace* trace,uint32_t sequence,ChassisTraceSample* sample) {
    if(!trace || !sample || sequence-chassis_trace_oldest(trace)>=trace->count) return false;
    *sample=trace->samples[sequence%CHASSIS_TRACE_CAPACITY];
    return sample->sequence==sequence;
}
bool chassis_trace_push(ChassisTrace* trace,const ChassisTraceSample* sample) {
    if(!trace || !sample) return false;
    if(trace->count) {
        const ChassisTraceSample* last=&trace->samples[(trace->next_sequence-1u)%CHASSIS_TRACE_CAPACITY];
        const bool event=sample->mode!=last->mode || sample->permitted!=last->permitted ||
            sample->blocker!=last->blocker || sample->fault!=last->fault ||
            sample->imu_valid!=last->imu_valid || sample->steer_valid_mask!=last->steer_valid_mask || sample->drive_valid_mask!=last->drive_valid_mask;
        if(!event && sample->t_ms-last->t_ms<CHASSIS_TRACE_PERIOD_MS) return false;
    }
    ChassisTraceSample* dst=&trace->samples[trace->next_sequence%CHASSIS_TRACE_CAPACITY];
    *dst=*sample;dst->sequence=trace->next_sequence++;
    if(trace->count<CHASSIS_TRACE_CAPACITY) ++trace->count;
    else ++trace->overwritten;
    return true;
}
const char* chassis_trace_csv_header(void) {
    return "sequence,t_ms,length_m,width_m,radius_m,target_vx,target_vy,target_wz,limited_vx,limited_vy,limited_wz,omega_fl,omega_fr,omega_rr,omega_rl,angle_fl,angle_fr,angle_rr,angle_rl,yaw_rad,gyro_z_rad_s,imu_age_us,imu_valid,steer_age_fl,steer_age_fr,steer_age_rr,steer_age_rl,drive_age_fl,drive_age_fr,drive_age_rr,drive_age_rl,control_dt_ms,steer_valid_mask,drive_valid_mask,mode,permitted,blocker,fault";
}
bool chassis_trace_csv_row(const ChassisTraceSample* s,const SteerWheelModel* m,char* out,size_t cap) {
    if(!out || cap==0) return false;
    out[0]='\0';
    if(!s || !steer_wheel_model_valid(m)) return false;
    const int n=snprintf(out,cap,
        "%lu,%lu,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%lu,%lu",
        (unsigned long)s->sequence,(unsigned long)s->t_ms,m->length,m->width,m->wheel_radius,
        s->target[0],s->target[1],s->target[2],s->limited[0],s->limited[1],s->limited[2],
        s->wheel_rad_s[0],s->wheel_rad_s[1],s->wheel_rad_s[2],s->wheel_rad_s[3],
        s->steer_rad[0],s->steer_rad[1],s->steer_rad[2],s->steer_rad[3],s->yaw_rad,s->gyro_z_rad_s,
        (unsigned long)s->imu_age_us,s->imu_valid,
        s->steer_age_ms[0],s->steer_age_ms[1],s->steer_age_ms[2],s->steer_age_ms[3],
        s->drive_age_ms[0],s->drive_age_ms[1],s->drive_age_ms[2],s->drive_age_ms[3],
        s->control_dt_ms,s->steer_valid_mask,s->drive_valid_mask,s->mode,s->permitted,
        (unsigned long)s->blocker,(unsigned long)s->fault);
    if(n<0 || (size_t)n>=cap) {out[0]='\0';return false;}
    return true;
}
