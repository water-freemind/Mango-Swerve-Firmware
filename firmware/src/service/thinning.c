#include "thinning.h"

#include "main.h"

#include <stddef.h>

static volatile ThinningSnapshot s_thinning;

static bool thinning_deadline_reached(uint32_t now_ms, uint32_t deadline_ms) {
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static void thinning_set_state(ThinningState state, uint32_t now_ms) {
    s_thinning.state = state;
    s_thinning.state_since_ms = now_ms;
    s_thinning.chassis_brake_required = state != THINNING_IDLE;
}

void thinning_init(void) {
    s_thinning.state = THINNING_IDLE;
    s_thinning.state_since_ms = 0u;
    s_thinning.deadline_ms = 0u;
    s_thinning.chassis_brake_required = false;
}

bool thinning_permit(bool chassis_stopped_and_braked, uint32_t now_ms, uint32_t timeout_ms) {
    if(!chassis_stopped_and_braked || timeout_ms == 0u || s_thinning.state != THINNING_IDLE) {
        return false;
    }
    s_thinning.deadline_ms = now_ms + timeout_ms;
    thinning_set_state(THINNING_PERMITTED, now_ms);
    return true;
}

bool thinning_mark_busy(uint32_t now_ms) {
    if(s_thinning.state != THINNING_PERMITTED) {
        return false;
    }
    thinning_set_state(THINNING_BUSY, now_ms);
    return true;
}

bool thinning_finish(bool success, uint32_t now_ms) {
    if(s_thinning.state != THINNING_BUSY) {
        return false;
    }
    thinning_set_state(success ? THINNING_SUCCESS : THINNING_FAILED, now_ms);
    return true;
}

void thinning_fault(uint32_t now_ms) {
    thinning_set_state(THINNING_FAULT, now_ms);
}

void thinning_process(uint32_t now_ms) {
    if((s_thinning.state == THINNING_PERMITTED || s_thinning.state == THINNING_BUSY) &&
       thinning_deadline_reached(now_ms, s_thinning.deadline_ms)) {
        thinning_set_state(THINNING_TIMED_OUT, now_ms);
    }
}

void thinning_reset(uint32_t now_ms) {
    s_thinning.deadline_ms = 0u;
    thinning_set_state(THINNING_IDLE, now_ms);
}

bool thinning_get_snapshot(ThinningSnapshot* snapshot) {
    uint32_t primask;

    if(snapshot == NULL) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *snapshot = s_thinning;
    __set_PRIMASK(primask);
    return true;
}
