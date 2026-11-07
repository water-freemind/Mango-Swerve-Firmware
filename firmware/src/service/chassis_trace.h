#ifndef CHASSIS_TRACE_H
#define CHASSIS_TRACE_H
#include "steer_wheel_kine.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define CHASSIS_TRACE_CAPACITY 512u
#define CHASSIS_TRACE_PERIOD_MS 20u
#define CHASSIS_TRACE_CSV_CAPACITY 1024u

/* One coherent producer snapshot. Target/limited use external +x forward,
 * +y left, +wz CCW. Wheel angles use internal IK +y right convention, FL FR RR RL.
 * Raw invalid measurements are retained for diagnosis; valid masks and ages
 * describe each measurement. Snapshot time is NOT the individual sample time. */
typedef struct {
    uint32_t sequence, t_ms;
    float target[3], limited[3];
    float wheel_rad_s[4], steer_rad[4];
    float yaw_rad, gyro_z_rad_s;
    uint32_t imu_age_us;
    uint8_t imu_valid;
    uint16_t steer_age_ms[4], drive_age_ms[4];
    uint16_t control_dt_ms;
    uint8_t steer_valid_mask, drive_valid_mask, mode, permitted;
    uint32_t blocker, fault;
} ChassisTraceSample;
typedef struct {
    ChassisTraceSample samples[CHASSIS_TRACE_CAPACITY];
    uint32_t next_sequence, count, overwritten;
} ChassisTrace;

/* Clears history/counters. To pause/resume without clearing, the caller simply
 * stops/resumes push calls. Event edges may shorten the 10.24s nominal window. */
void chassis_trace_init(ChassisTrace* trace);
/* Main-loop only (not ISR-safe). Event edges bypass periodic decimation. */
bool chassis_trace_push(ChassisTrace* trace, const ChassisTraceSample* sample);
uint32_t chassis_trace_oldest(const ChassisTrace* trace);
bool chassis_trace_get(const ChassisTrace* trace, uint32_t sequence, ChassisTraceSample* sample);
const char* chassis_trace_csv_header(void);
/* Bounded row, no newline; false means no valid row, buffer is cleared. Caller
 * exports on demand while stopped; do not stream the whole ring in a control tick. */
bool chassis_trace_csv_row(const ChassisTraceSample* sample, const SteerWheelModel* model,
                           char* output, size_t capacity);
#endif
