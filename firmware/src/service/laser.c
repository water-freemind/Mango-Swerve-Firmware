#include "laser.h"

#include "main.h"

#include <stddef.h>

static volatile uint16_t s_distance_mm[LASER_CHANNEL_COUNT];
static volatile uint32_t s_last_rx_ms;
static volatile uint32_t s_frame_count;

void laser_accept_frame(const uint8_t data[8], uint32_t rx_ms) {
    uint32_t i;

    if(data == NULL) {
        return;
    }
    for(i = 0u; i < LASER_CHANNEL_COUNT; ++i) {
        s_distance_mm[i] = (uint16_t)(((uint16_t)data[2u * i] << 8u) | data[2u * i + 1u]);
    }
    s_last_rx_ms = rx_ms;
    ++s_frame_count;
}

bool laser_get_snapshot(LaserSnapshot* snapshot) {
    uint32_t primask;
    uint32_t i;

    if(snapshot == NULL) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    for(i = 0u; i < LASER_CHANNEL_COUNT; ++i) {
        snapshot->distance_mm[i] = s_distance_mm[i];
    }
    snapshot->last_rx_ms = s_last_rx_ms;
    snapshot->frame_count = s_frame_count;
    __set_PRIMASK(primask);

    snapshot->frame_fresh = snapshot->frame_count != 0u &&
        (uint32_t)(HAL_GetTick() - snapshot->last_rx_ms) <= LASER_FRAME_TIMEOUT_MS;
    return true;
}
