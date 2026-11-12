#ifndef MANGO_FIELD_POSE_H
#define MANGO_FIELD_POSE_H
#include "auto_geometry.h"
#include "laser.h"
#include "auto_route.h"

typedef struct {
    AutoPoseSample pose;
    float imu_offset;
    AutoVec2 wheel_position, checkpoint_position[4];
    uint16_t checkpoint_mm[4];
    uint32_t checkpoint_surface[4];
    uint8_t checkpoint_mask;
    float uncorrected_m[2]; /* Axis travel since last independent map correction. */
    float innovation_m[2]; /* Independent residual still pending after the bounded fusion correction. */
    uint32_t last_ms, used_stamp[4];
    uint32_t constrained_ms[2]; /* Last accepted independent X/Y correction, or registered prior. */
    uint8_t seen_mask, used_mask, rejected_mask, constrained_axes;
    bool registered;
} FieldPose;
/* Empty nominal 3 x 4.6m field only. No fence around the start zone;
 * south fence has the 0..0.4m entry gap. Returns optical ray distance. */
bool field_pose_ray(AutoPose pose, unsigned channel, AutoPlane* plane, float* distance);
bool field_pose_register(FieldPose* state, AutoPose start, float imu_yaw, uint32_t now);
/* Wheel velocity is body forward/left; IMU yaw must be the same physical sample
 * source as register(). Range receipt times are not sensor quality guarantees. */
bool field_pose_step(FieldPose* state, AutoTwist wheel, float imu_yaw, bool healthy,
    const LaserSnapshot* laser, uint32_t now);
/* Also associate short returns with configured ridge faces. Fence-only
 * observations retain their residual/freshness/frozen-data checks. */
bool field_pose_step_route(FieldPose* state,AutoTwist wheel,float imu_yaw,bool healthy,
    const LaserSnapshot* laser,uint32_t now,const AutoRoute* route);
#endif
