#ifndef MANGO_SWERVE_TEST_MAIN_H
#define MANGO_SWERVE_TEST_MAIN_H

#include <stdint.h>

#ifdef TEST_CAN_HAL
#include "can_hal_fake.h"
#endif

extern uint32_t g_test_tick_ms;
#ifdef TEST_RX_TIME_RACE
extern uint32_t g_test_irq_mask;
uint32_t test_rx_tick(void);
#endif

static inline uint32_t HAL_GetTick(void) {
#ifdef TEST_RX_TIME_RACE
    return test_rx_tick();
#else
    return g_test_tick_ms;
#endif
}

static inline uint32_t __get_PRIMASK(void) {
#ifdef TEST_RX_TIME_RACE
    return g_test_irq_mask;
#else
    return 0u;
#endif
}

static inline void __disable_irq(void) {
#ifdef TEST_RX_TIME_RACE
    g_test_irq_mask = 1u;
#endif
}

static inline void __set_PRIMASK(uint32_t primask) {
#ifdef TEST_RX_TIME_RACE
    g_test_irq_mask = primask;
#else
    (void)primask;
#endif
}

static inline void __enable_irq(void) { __set_PRIMASK(0u); }

#endif
