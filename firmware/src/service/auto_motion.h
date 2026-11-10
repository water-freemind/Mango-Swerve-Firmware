#ifndef MANGO_AUTO_MOTION_H
#define MANGO_AUTO_MOTION_H
#include "auto_geometry.h"
typedef struct {
    float vmax, accel, decel, lateral_max, cross_kp;
    float yaw_kp, wz_max, angular_accel;
    float position_tolerance, cross_tolerance, yaw_tolerance;
    float cross_abort, yaw_abort, settle_s, stop_latency_s;
    uint32_t pose_max_age_ms;
    float finish_yaw_tolerance; /* Optional endpoint-only acceptance; zero uses yaw_tolerance. */
    float finish_cross_tolerance; /* Optional endpoint-only lateral acceptance. */
} AutoMotionConfig;
typedef struct { AutoVec2 start, end; float heading; } AutoSegment;
typedef enum { AUTO_MOTION_IDLE, AUTO_MOTION_ALIGN, AUTO_MOTION_RUN,
    AUTO_MOTION_HOLD, AUTO_MOTION_DONE, AUTO_MOTION_FAULT } AutoMotionPhase;
typedef enum { AUTO_MOTION_NO_ERROR, AUTO_MOTION_BAD_CONFIG, AUTO_MOTION_BAD_INPUT,
    AUTO_MOTION_STALE_POSE, AUTO_MOTION_DEVIATION, AUTO_MOTION_OVERSHOOT,
    AUTO_MOTION_STOPPED_BY_SAFETY } AutoMotionError;
typedef struct {
    AutoPoseSample pose; AutoTwist measured;
    bool healthy, aligned, stationary, stop_requested, hard_stop;
    uint32_t now_ms;
} AutoMotionInput;
typedef struct {
    AutoMotionPhase phase; AutoMotionError error;
    AutoTwist command, steering_hint; /* Hint is NOT authorization to drive. */
    float progress, cross_error, heading_error;
} AutoMotionOutput;
typedef struct {
    AutoMotionConfig config; AutoSegment segment; AutoMotionOutput out;
    AutoVec2 tangent, normal, velocity_map;
    float length, settled_s; uint32_t last_ms;
    bool initialized, started, reversing;
} AutoMotion;
bool auto_motion_begin(AutoMotion* motion,const AutoMotionConfig* config,AutoSegment segment,uint32_t now_ms);
void auto_motion_cancel(AutoMotion* motion);
AutoMotionOutput auto_motion_step(AutoMotion* motion,AutoMotionInput input);
#endif
