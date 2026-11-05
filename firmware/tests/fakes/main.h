#ifndef MANGO_SWERVE_TEST_MAIN_H
#define MANGO_SWERVE_TEST_MAIN_H

#include <stdint.h>

#ifdef TEST_CAN_HAL
#include "can_hal_fake.h"
#endif

extern uint32_t g_test_tick_ms;

static inline uint32_t HAL_GetTick(void) {
    return g_test_tick_ms;
}

static inline uint32_t __get_PRIMASK(void) {
    return 0u;
}

static inline void __disable_irq(void) {
}

static inline void __set_PRIMASK(uint32_t primask) {
    (void)primask;
}

#endif
