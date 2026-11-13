#ifndef MANGO_FULL_MAP_TEST_H
#define MANGO_FULL_MAP_TEST_H
#include "auto_route.h"
#include "field_pose.h"
#include "laser_prediction.h"
typedef enum { MAP_IDLE, MAP_READY, MAP_ARMED, MAP_RUNNING, MAP_PAUSED,
    MAP_SEGMENT_DONE, MAP_DONE, MAP_FAULT } FullMapPhase;
typedef struct {
    bool active, centering;
    uint8_t confirmations, pending_updates;
    uint32_t frame, stamp_ms, front_surface, rear_surface;
    uint32_t sample_ms[2]; /* Source timestamps of the last confirmed front/rear pair. */
    uint16_t front_mm, rear_mm;
    float error_m, correction_m;
    bool checkpoint_valid, frozen;
    uint16_t checkpoint_front_mm, checkpoint_rear_mm;
    float checkpoint_wheel_y;
} MapAisleCorrection;
typedef struct {
    bool active;
    uint8_t confirmations,channel,axis;
    uint32_t frame,stamp_ms,sample_ms,last_step_ms;
    float error_m;
} MapWallCorrection;

typedef struct {
    float error_m;
    uint32_t last_ms;
    uint32_t wall_sample_ms;
    uint8_t axis;
    bool initialized,wall_reference;
} MapCrossFeedback;

typedef struct {
    bool approaching, north_confirmed, left_observed;
    uint8_t confirmations;
    uint32_t sample_ms, first_ms, left_sample_ms;
} MapEntryGuidance;

typedef struct {
    uint32_t lost_ms;
    float lost_path;
    bool active;
} MapFarRayHandover;
typedef struct {
    uint32_t now; AutoTwist wheel; float imu_yaw;
    bool healthy, odom_ok, stationary, aligned;
    LaserSnapshot laser;
    bool work_enabled, work_online, work_ready, work_done, work_failed;
    bool work_slow, work_wait, work_active;
    uint8_t work_station;
} FullMapInput;
typedef struct {
    float x;
    bool configured;
} MapTreePosition;
typedef struct {
    FullMapPhase phase; FieldPose localization; AutoRoute route; AutoMotion motion;
    LaserPrediction prediction;
    LaserMotionFilter laser_motion;
    MapAisleCorrection aisle_correction;
    MapWallCorrection wall_correction;
    MapCrossFeedback cross_feedback;
    MapEntryGuidance entry;
    MapFarRayHandover far_handover[2]; /* Left/right only; bounded extra handover, no invented samples. */
    AutoTwist command, hint;
    uint32_t token, arm_ms, run_ms, last_ms, segment_ms, pause_ms, stable_ms, recovery_frame;
    unsigned segment, recovery_frames; uint8_t coverage, predicted_mask, required_mask, recovery_updates;
    uint8_t range_near_mask, range_brake_mask, range_stale_mask;
    float range_budget_m[4]; /* Last evaluated stop thresholds, indexed like LaserSnapshot. */
    uint16_t range_control_mm[4];
    uint8_t laser_rejected_mask;
    uint8_t odom_range_mask; /* Map clearance with bounded odometry; never a real laser-valid bit. */
    uint16_t origin_front_mm;
    float speed, approach_speed, along_speed_limit; bool all, stable, initialized, user_hold;
    const char* reason;
    AutoMotionError motion_error;
    MapTreePosition trees[6][2]; /* row 1..6, tree 1 west / tree 2 east */
    uint16_t trees_done;
    uint8_t target_tree; /* 0 none; (row-1)*2 + local tree number */
    bool tree_hold, tree_hold_stable;
    uint32_t tree_hold_ms;
} FullMapTest;
/* Station vehicle-center X in metres from the west map boundary. No guessed defaults. */
bool full_map_set_tree_x(FullMapTest* map,unsigned row,unsigned number,float x);
void full_map_init(FullMapTest* map,uint32_t now);
bool full_map_active(const FullMapTest* map);
void full_map_stop(FullMapTest* map,const char* reason,bool lose_origin);
bool full_map_command(FullMapTest* map,const char* command,FullMapInput input);
void full_map_step(FullMapTest* map,FullMapInput input);
const char* full_map_phase_name(FullMapPhase phase);
#endif
