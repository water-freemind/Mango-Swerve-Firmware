#ifndef MANGO_DRIVE_SOURCE_H
#define MANGO_DRIVE_SOURCE_H
#include "remote.h"

#define DRIVE_MANUAL_VRB_MAX 1250u

typedef enum { DRIVE_SOURCE_STOP, DRIVE_SOURCE_HOST, DRIVE_SOURCE_MANUAL } DriveSourceMode;
typedef struct {
    DriveSourceMode mode;
    bool manual_armed, release_seen;
    bool recovery_pending;
    bool online_seen;
} DriveSource;
typedef struct { float vx, vy, wz; } DriveVelocity;

DriveSourceMode drive_source_requested(const RemoteState* remote);
const char* drive_source_name(DriveSourceMode mode);
bool drive_source_centered(const RemoteState* remote);
bool drive_source_manual_vrb_ok(const RemoteState* remote);
const char* drive_source_manual_wait_reason(const DriveSource* source, const RemoteState* remote,
                                          bool hardware_healthy, bool stationary);
/* First boot requires an observed SWD-off confirmation. An online switch to
 * MANUAL also confirms takeover with neutral sticks and stationary wheels.
 * SWC handovers preserve confirmation. VRB <= DRIVE_MANUAL_VRB_MAX is required.
 * SWD off is recorded even during hardware faults.
 * Initial hardware/VRB readiness may arrive after SWD on. VRB release stops
 * motion, then re-press requires neutral/stationary takeover. Hardware faults,
 * explicit stops and loss of SWA/VRA permit revoke confirmation until SWD
 * off/on or a new online MANUAL selection. RC loss
 * stops immediately but retains confirmation. Reconnection waits for neutral,
 * stationary wheels and VRB. A hardware fault while waiting clears confirmation. */
bool drive_source_update(DriveSource* source, const RemoteState* remote,
                         bool hardware_healthy, bool stationary);
void drive_source_stop_manual(DriveSource* source);
DriveVelocity drive_source_velocity(const RemoteState* remote);
#endif
