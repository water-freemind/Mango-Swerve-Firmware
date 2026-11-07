#ifndef MANGO_AUTO_GEOMETRY_H
#define MANGO_AUTO_GEOMETRY_H
#include <stdbool.h>
#include <stdint.h>

/* Pure C, no HAL/actuator dependencies. SI units: m, s, rad.
 * Body +x forward, +y left; map +X right, +Y into field. */
typedef struct { float x, y; } AutoVec2;
typedef struct { float x, y, yaw; } AutoPose;
typedef struct { float vx, vy, wz; } AutoTwist;
typedef struct { const char* source; bool confirmed; } AutoEvidence;
typedef struct {
    float front, rear, left, right; /* Positive extents from body origin. */
} AutoEnvelope;
typedef struct {
    AutoVec2 origin; float z, yaw; /* Effective ranging reference, not housing. */
} AutoLaserMount;
enum { AUTO_LEFT, AUTO_RIGHT, AUTO_REAR, AUTO_FRONT, AUTO_LASER_COUNT };
typedef struct {
    AutoVec2 wheel[4]; float wheel_radius;
    AutoLaserMount laser[AUTO_LASER_COUNT];
    AutoEnvelope stowed, extended;
    float brake_decel, latency_s, clearance_margin, residual_limit;
    uint32_t range_max_age_ms, pose_max_age_ms;
    AutoEvidence wheels_evidence, envelope_evidence, brake_evidence;
    AutoEvidence laser_evidence[AUTO_LASER_COUNT];
    AutoEvidence start_evidence, work_contract_evidence;
    bool simulation_only;
} AutoConfig;
typedef struct { AutoPose pose; uint32_t stamp_ms; bool valid; } AutoPoseSample;
typedef struct {
    AutoVec2 normal; float offset; /* map plane n.dot(point)=offset, n toward obstacle */
    uint32_t surface_id; bool associated;
} AutoPlane;
typedef struct {
    uint16_t raw_mm; uint32_t stamp_ms, surface_id;
    bool valid, channel_verified; /* Aggregate CAN freshness alone is insufficient. */
} AutoRange;
typedef enum {
    AUTO_GEOM_OK, AUTO_GEOM_CONFIG, AUTO_GEOM_UNCONFIRMED,
    AUTO_GEOM_STALE, AUTO_GEOM_RANGE, AUTO_GEOM_SURFACE, AUTO_GEOM_RESIDUAL
} AutoGeomResult;
typedef struct {
    float raw_m, expected_m, residual_m, edge_offset_m;
    float clearance_m, closing_mps, required_m;
    bool collision, protective_stop;
} AutoClearance;

bool auto_config_valid(const AutoConfig* config);
/* No operational default: simulation profiles can never pass this gate. */
bool auto_config_hardware_ready(const AutoConfig* config);
float auto_wrap_angle(float rad);
AutoVec2 auto_rotate(AutoVec2 v, float yaw);
/* Transform an existing odom pose; does NOT consume IMU or integrate position. */
bool auto_pose_to_map(AutoPoseSample odom_pose, AutoPose map_from_odom,
                      uint32_t now_ms, uint32_t max_age_ms, AutoPoseSample* result);
float auto_envelope_support(AutoEnvelope body, AutoVec2 normal_body);
bool auto_envelope_inside(AutoEnvelope body, AutoPose pose,
                          float xmin, float ymin, float xmax, float ymax, float margin);
AutoGeomResult auto_laser_clearance(const AutoConfig* config, unsigned channel,
    AutoEnvelope body, AutoPoseSample pose, AutoTwist velocity,
    AutoRange range, AutoPlane plane, uint32_t now_ms, AutoClearance* result);
#endif
