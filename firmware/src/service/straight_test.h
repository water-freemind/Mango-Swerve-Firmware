#ifndef MANGO_STRAIGHT_TEST_H
#define MANGO_STRAIGHT_TEST_H
#include <stdbool.h>
#include <stdint.h>
#define STRAIGHT_TEST_WHEEL_RADIUS_M 0.051f /* nominal; verify using an external ruler */
#define STRAIGHT_TEST_TARGET_M 1.0f
#define STRAIGHT_TEST_OVERSHOOT_M (STRAIGHT_TEST_TARGET_M + 0.10f)
#define STRAIGHT_TEST_RUN_TIMEOUT_MS 25000u
#define STRAIGHT_TEST_FRONT_GAP_MM 100u
#define STRAIGHT_TEST_BODY_EDGE_MM 110u /* Left-side contact measured 108..109 mm; common initial envelope. */
#define STRAIGHT_TEST_COLLISION_MARGIN_MM 20u
#define STRAIGHT_TEST_FRONT_TOL_MM 10u
#define STRAIGHT_TEST_APPROACH_TIMEOUT_MS 35000u
#define STRAIGHT_TEST_WALL_MAX_VY_MPS 0.02f
#define STRAIGHT_TEST_WALL_KP 0.5f
#define STRAIGHT_TEST_TURN_DWELL_MS 2000u
#define STRAIGHT_TEST_SPEED_CEILING_MPS 0.50f
#define STRAIGHT_TEST_BRAKE_DECEL_MPS2 0.10f /* Initial command/braking budget; needs field validation. */
#define STRAIGHT_TEST_SENSOR_DELAY_S 0.30f

typedef enum { ST_IDLE, ST_ARMED, ST_RUNNING, ST_STOPPING, ST_DONE, ST_FAULT } StraightPhase;
typedef enum { ST_PLAN_FWD, ST_PLAN_ROUND, ST_PLAN_WALL, ST_PLAN_APPROACH } StraightPlan;
typedef enum { ST_LEG_OUT, ST_LEG_WAIT, ST_LEG_BACK } StraightLeg;
typedef struct {
    StraightPhase phase;
    StraightPlan plan;
    StraightLeg leg;
    uint16_t body_edge_mm;
    uint32_t token, arm_ms, heartbeat_ms, start_ms, last_ms, stopped_ms;
    uint32_t leg_start_ms, turn_ms, laser_frame;
    float max_speed_mps, target_m, distance_m, speed_mps, lateral_mps, outbound_m, right_ref_mm, right_filtered_mm;
    bool stopped_timing;
    const char* result;
} StraightTest;
/* healthy includes fresh motors/IMU, remote permit and CAN/UART health. */
typedef struct {
    bool healthy, stationary;
    float vx_mps, vy_mps;
    bool laser_fresh;
    uint16_t left_mm, right_mm, rear_mm, front_mm;
    uint32_t laser_frame, laser_age_ms;
} StraightInput;
const char* straight_plan_name(StraightPlan plan);
const char* straight_leg_name(StraightLeg leg);
bool straight_laser_ready(StraightPlan plan, StraightInput input);
uint16_t straight_front_target_mm(const StraightTest* test);
void straight_guard_limits(const StraightTest* test, StraightInput input, uint16_t limits_mm[4]);
void straight_init(StraightTest* test, uint32_t now);
bool straight_command(StraightTest* test, const char* line, uint32_t now, StraightInput input);
void straight_step(StraightTest* test, uint32_t now, StraightInput input);
void straight_fault(StraightTest* test, const char* reason);
#endif
