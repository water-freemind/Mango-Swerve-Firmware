#ifndef MANGO_FULL_MAP_TEST_H
#define MANGO_FULL_MAP_TEST_H
#include "auto_route.h"
#include "field_pose.h"
#include "laser_prediction.h"
typedef enum { MAP_IDLE, MAP_READY, MAP_ARMED, MAP_RUNNING, MAP_PAUSED,
    MAP_SEGMENT_DONE, MAP_DONE, MAP_FAULT } FullMapPhase;
typedef struct {
    uint32_t now; AutoTwist wheel; float imu_yaw;
    bool healthy, odom_ok, stationary, aligned;
    LaserSnapshot laser;
} FullMapInput;
typedef struct {
    FullMapPhase phase; FieldPose localization; AutoRoute route; AutoMotion motion;
    LaserPrediction prediction;
    AutoTwist command, hint;
    uint32_t token, arm_ms, run_ms, last_ms, segment_ms, pause_ms, stable_ms, recovery_frame;
    unsigned segment, recovery_frames; uint8_t coverage, predicted_mask, required_mask;
    float speed; bool all, stable, initialized, user_hold;
    const char* reason;
    AutoMotionError motion_error;
} FullMapTest;
void full_map_init(FullMapTest* map,uint32_t now);
bool full_map_active(const FullMapTest* map);
void full_map_stop(FullMapTest* map,const char* reason,bool lose_origin);
bool full_map_command(FullMapTest* map,const char* command,FullMapInput input);
void full_map_step(FullMapTest* map,FullMapInput input);
const char* full_map_phase_name(FullMapPhase phase);
#endif
