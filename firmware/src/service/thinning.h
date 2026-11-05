#ifndef MANGO_SWERVE_THINNING_H
#define MANGO_SWERVE_THINNING_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    THINNING_IDLE = 0,
    THINNING_PERMITTED,
    THINNING_BUSY,
    THINNING_SUCCESS,
    THINNING_FAILED,
    THINNING_FAULT,
    THINNING_TIMED_OUT
} ThinningState;

typedef struct {
    ThinningState state;
    uint32_t state_since_ms;
    uint32_t deadline_ms;
    bool chassis_brake_required;
} ThinningSnapshot;

void thinning_init(void);
bool thinning_permit(bool chassis_stopped_and_braked, uint32_t now_ms, uint32_t timeout_ms);
bool thinning_mark_busy(uint32_t now_ms);
bool thinning_finish(bool success, uint32_t now_ms);
void thinning_fault(uint32_t now_ms);
void thinning_process(uint32_t now_ms);
void thinning_reset(uint32_t now_ms);
bool thinning_get_snapshot(ThinningSnapshot* snapshot);

#endif
