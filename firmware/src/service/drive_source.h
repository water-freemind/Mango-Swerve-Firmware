#ifndef MANGO_DRIVE_SOURCE_H
#define MANGO_DRIVE_SOURCE_H
#include "remote.h"

#define DRIVE_MANUAL_VRB_MAX 1250u

typedef enum { DRIVE_SOURCE_STOP, DRIVE_SOURCE_HOST, DRIVE_SOURCE_MANUAL } DriveSourceMode;
typedef struct {
    DriveSourceMode mode;
    bool manual_armed, release_seen;
    bool recovery_pending;
} DriveSource;
typedef struct { float vx, vy, wz; } DriveVelocity;

DriveSourceMode drive_source_requested(const RemoteState* remote);
const char* drive_source_name(DriveSourceMode mode);
bool drive_source_centered(const RemoteState* remote);
bool drive_source_manual_vrb_ok(const RemoteState* remote);
/* SWD must be observed off before manual enable. SWC handovers preserve this
 * confirmation, then wait for centred sticks, stationary fresh wheels and
 * VRB <= DRIVE_MANUAL_VRB_MAX. SWD off is recorded even during hardware faults.
 * Initial hardware/VRB readiness may arrive after SWD on. VRB release stops
 * motion, then re-press requires neutral/stationary takeover. Hardware faults,
 * explicit stops and loss of SWA/VRA permit require SWD off/on again. RC loss
 * stops immediately but retains confirmation. Reconnection waits for neutral,
 * stationary wheels and VRB. A hardware fault while waiting clears confirmation. */
bool drive_source_update(DriveSource* source, const RemoteState* remote,
                         bool hardware_healthy, bool stationary);
void drive_source_stop_manual(DriveSource* source);
DriveVelocity drive_source_velocity(const RemoteState* remote);
#endif
