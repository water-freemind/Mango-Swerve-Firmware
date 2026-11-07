#ifndef MANGO_DRIVE_SOURCE_H
#define MANGO_DRIVE_SOURCE_H
#include "remote.h"

typedef enum { DRIVE_SOURCE_STOP, DRIVE_SOURCE_HOST, DRIVE_SOURCE_MANUAL } DriveSourceMode;
typedef struct {
    DriveSourceMode mode;
    bool manual_armed, release_seen;
} DriveSource;
typedef struct { float vx, vy, wz; } DriveVelocity;

DriveSourceMode drive_source_requested(const RemoteState* remote);
const char* drive_source_name(DriveSourceMode mode);
bool drive_source_centered(const RemoteState* remote);
/* A mode change invalidates manual enable; SWD must be observed off in the
 * selected mode, then on with centred sticks and stationary fresh wheels. */
bool drive_source_update(DriveSource* source, const RemoteState* remote,
                         bool hardware_healthy, bool stationary);
void drive_source_stop_manual(DriveSource* source);
DriveVelocity drive_source_velocity(const RemoteState* remote);
#endif
