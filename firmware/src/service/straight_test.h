#ifndef MANGO_STRAIGHT_TEST_H
#define MANGO_STRAIGHT_TEST_H
#include <stdbool.h>
#include <stdint.h>
#define STRAIGHT_TEST_WHEEL_RADIUS_M 0.051f /* nominal; verify using an external ruler */

typedef enum { ST_IDLE, ST_ARMED, ST_RUNNING, ST_STOPPING, ST_DONE, ST_FAULT } StraightPhase;
typedef struct {
    StraightPhase phase;
    uint32_t token, arm_ms, heartbeat_ms, start_ms, last_ms, stopped_ms;
    float distance_m, speed_mps;
    bool stopped_timing;
    const char* result;
} StraightTest;
/* healthy includes fresh motors/IMU, remote permit and CAN/UART health. */
typedef struct { bool healthy, stationary; float vx_mps; } StraightInput;
void straight_init(StraightTest* test, uint32_t now);
bool straight_command(StraightTest* test, const char* line, uint32_t now, StraightInput input);
void straight_step(StraightTest* test, uint32_t now, StraightInput input);
void straight_fault(StraightTest* test, const char* reason);
#endif
