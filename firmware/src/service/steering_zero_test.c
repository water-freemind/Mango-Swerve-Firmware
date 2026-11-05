#include "steering_zero_test.h"

#include "assemble/assemble.h"
#include "bus_motor/dm_motor.h"
#include "chassis_can_map.h"
#include "commissioning.h"
#include "log.h"

#include <math.h>

#define ZERO_TEST_SPEED 0.2f
#define ZERO_TEST_PERIOD_MS 20u
#define ZERO_TEST_FEEDBACK_MS 300u
#define ZERO_TEST_TIMEOUT_MS 30000u

/* Mechanical forward positions recorded separately on 2026-11-05. */
static const float s_zero[4] = {0.001335164f, -3.131723507f, 3.127908751f, 2.075799191f};
typedef enum { ZERO_DELAY, ZERO_WAIT, ZERO_MOVE, ZERO_DONE, ZERO_FAULT } ZeroPhase;
static ZeroPhase s_phase = ZERO_FAULT;
static unsigned s_wheel;
static uint32_t s_start_ms, s_step_ms, s_log_ms, s_count;
static float s_target, s_command;
static unsigned s_settled;

static void zero_disable_all(void) {
    for(uint16_t id = 1u; id <= 4u; ++id) {
        if(dm_motor_instance.disable(id) != MOTOR_STATUS_OK) {
            log_error("ZERO_TEST disable transmission failed id=%u", (unsigned)id);
        }
    }
}

static void zero_fail(const char* reason) {
    s_phase = ZERO_FAULT;
    zero_disable_all();
    log_error("ZERO_TEST FAULT id=%u reason=%s", s_wheel + 1u, reason);
}

/* Read frame identity, timestamp and driver cache under the same IRQ lock. */
static bool zero_feedback(BusMotorFeedback* feedback, uint32_t* age, uint32_t* count) {
    const unsigned bus = CHASSIS_STEER_CAN == &hfdcan1 ? 0u : 1u;
    const uint32_t primask = __get_PRIMASK();
    bool found = false;
    __disable_irq();
    for(unsigned i = 0u; i < g_commissioning.bus[bus].id_count; ++i) {
        const volatile CommissioningFrame* frame = &g_commissioning.bus[bus].frames[i];
        if(frame->header.Identifier == 0x11u + s_wheel &&
           frame->header.IdType == FDCAN_STANDARD_ID &&
           frame->header.RxFrameType == FDCAN_DATA_FRAME &&
           frame->header.FDFormat == FDCAN_CLASSIC_CAN &&
           frame->header.DataLength == FDCAN_DLC_BYTES_8 &&
           (frame->data[0] & 0x0fu) == s_wheel + 1u) {
            *age = HAL_GetTick() - frame->last_rx_ms;
            *count = frame->count;
            found = dm_motor_instance.update_feedback((uint16_t)(s_wheel + 1u), feedback) == MOTOR_STATUS_OK;
            break;
        }
    }
    __set_PRIMASK(primask);
    return found;
}

static void zero_start_wheel(uint32_t now) {
    BusMotorFeedback feedback;
    uint32_t age;
    s_count = 0u;
    (void)zero_feedback(&feedback, &age, &s_count);
    s_start_ms = now;
    s_settled = 0u;
    s_phase = ZERO_WAIT;
    if(dm_motor_instance.enable((uint16_t)(s_wheel + 1u)) != MOTOR_STATUS_OK) {
        zero_fail("enable transmission");
    }
}

void steering_zero_test_init(void) {
    s_wheel = 0u;
    s_phase = ZERO_FAULT;
    if(assemble_steering_diagnostics() != SYSTEM_STATUS_OK) {
        zero_fail("initialization");
        return;
    }
    zero_disable_all();
    s_start_ms = HAL_GetTick();
    s_step_ms = s_start_ms;
    s_log_ms = s_start_ms;
    s_phase = ZERO_DELAY;
    log_info("ZERO_TEST armed; starts in 5s; sequential IDs 1..4, speed=0.2 rad/s; release after each wheel");
}

void steering_zero_test_process(void) {
    const uint32_t now = HAL_GetTick();
    BusMotorFeedback feedback;
    uint32_t age = 0u, count = 0u;
    bool available;
    if((uint32_t)(now - s_log_ms) >= 1000u) {
        s_log_ms = now;
        log_info("ZERO_TEST phase=%u id=%u target=%f command=%f", (unsigned)s_phase,
                 s_wheel + 1u, (double)s_target, (double)s_command);
    }
    if(s_phase == ZERO_DONE || s_phase == ZERO_FAULT) {
        return;
    }
    if(s_phase == ZERO_DELAY) {
        if((uint32_t)(now - s_start_ms) >= 5000u) zero_start_wheel(now);
        return;
    }
    if((uint32_t)(now - s_step_ms) < ZERO_TEST_PERIOD_MS) return;
    s_step_ms = now;
    available = zero_feedback(&feedback, &age, &count);
    if(s_phase == ZERO_WAIT) {
        if(!available || age > ZERO_TEST_FEEDBACK_MS || count == s_count) {
            if((uint32_t)(now - s_start_ms) > ZERO_TEST_FEEDBACK_MS) zero_fail("no new enable feedback");
            return;
        }
        if(feedback.error_code != 1u || !isfinite(feedback.position)) {
            zero_fail("motor state");
            return;
        }
        s_command = feedback.position;
        s_target = s_zero[s_wheel] + 6.283185307f * roundf((s_command - s_zero[s_wheel]) / 6.283185307f);
        if(s_target < -12.5f || s_target > 12.5f) {
            zero_fail("target range");
            return;
        }
        s_phase = ZERO_MOVE;
        log_info("ZERO_TEST begin id=%u position=%f target=%f", s_wheel + 1u,
                 (double)s_command, (double)s_target);
    }
    if(!available || age > ZERO_TEST_FEEDBACK_MS || feedback.error_code != 1u ||
       !isfinite(feedback.position) || !isfinite(feedback.speed)) {
        zero_fail("feedback stale or fault");
        return;
    }
    if((uint32_t)(now - s_start_ms) > ZERO_TEST_TIMEOUT_MS || fabsf(feedback.position - s_command) > 0.15f) {
        zero_fail("timeout or tracking error");
        return;
    }
    if(count != s_count) {
        s_count = count;
        if(fabsf(s_target - feedback.position) < 0.02f && fabsf(feedback.speed) < 0.05f && s_command == s_target) {
            ++s_settled;
        } else {
            s_settled = 0u;
        }
    }
    if(s_settled >= 10u) {
        log_info("ZERO_TEST reached id=%u pos=%f error=%f", s_wheel + 1u,
                 (double)feedback.position, (double)(feedback.position - s_target));
        if(dm_motor_instance.disable((uint16_t)(s_wheel + 1u)) != MOTOR_STATUS_OK) {
            zero_fail("disable transmission");
            return;
        }
        if(++s_wheel == 4u) {
            s_wheel = 3u;
            s_phase = ZERO_DONE;
            log_info("ZERO_TEST COMPLETE all four reached; steering released");
        } else {
            zero_start_wheel(now);
        }
        return;
    }
    const float delta = s_target - s_command;
    const float step = ZERO_TEST_SPEED * (float)ZERO_TEST_PERIOD_MS * 0.001f;
    s_command += fmaxf(-step, fminf(step, delta));
    if(fabsf(s_target - s_command) < 0.00001f) s_command = s_target;
    if(dm_motor_instance.set_pos_vel((uint16_t)(s_wheel + 1u), s_command, ZERO_TEST_SPEED) != MOTOR_STATUS_OK) {
        zero_fail("position transmission");
    }
}
