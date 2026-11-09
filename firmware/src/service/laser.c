#include "laser.h"

#include "main.h"

#include <stddef.h>

static volatile uint16_t s_distance_mm[LASER_CHANNEL_COUNT];
static volatile uint16_t s_accepted_mm[LASER_CHANNEL_COUNT];
static volatile uint32_t s_accepted_ms[LASER_CHANNEL_COUNT];
static volatile uint8_t s_accepted_mask;
static uint16_t s_history[LASER_CHANNEL_COUNT][3];
static uint8_t s_history_count[LASER_CHANNEL_COUNT];
static volatile uint8_t s_updated_mask;
static volatile uint32_t s_last_rx_ms;
static volatile uint32_t s_frame_count;

static uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
    if(a > b) { const uint16_t swap = a; a = b; b = swap; }
    if(b > c) { b = c; }
    return a > b ? a : b;
}

void laser_accept_frame(const uint8_t data[8], uint32_t rx_ms) {
    uint32_t i;

    if(data == NULL) {
        return;
    }
    s_updated_mask = 0u;
    for(i = 0u; i < LASER_CHANNEL_COUNT; ++i) {
        const uint8_t bit = (uint8_t)(1u << i);
        s_distance_mm[i] = (uint16_t)(((uint16_t)data[2u * i] << 8u) | data[2u * i + 1u]);
        if(s_distance_mm[i] <= LASER_ACCEPT_MAX_MM) {
            const uint16_t value = s_distance_mm[i];
            const uint32_t elapsed = rx_ms - s_accepted_ms[i];
            if((s_accepted_mask & bit) && elapsed == 0u) continue;
            if(elapsed > LASER_FILTER_CONFIRM_MS) s_history_count[i] = 0u;
            s_history[i][2] = s_history[i][1];
            s_history[i][1] = s_history[i][0];
            s_history[i][0] = value;
            if(s_history_count[i] < 3u) ++s_history_count[i];
            uint16_t filtered = value;
            if(value > LASER_FILTER_NEAR_MM && s_history_count[i] == 3u)
                filtered = median3(s_history[i][0], s_history[i][1], s_history[i][2]);
            else if(value > LASER_FILTER_NEAR_MM && s_history_count[i] == 2u)
                filtered = value < s_history[i][1] ? value : s_history[i][1];
            s_accepted_mm[i] = filtered;
            s_accepted_ms[i] = rx_ms;
            s_accepted_mask |= bit;
            s_updated_mask |= bit;
        }
    }
    s_last_rx_ms = rx_ms;
    ++s_frame_count;
}

bool laser_accept_can3_frame(const Can3Frame* frame, void* user) {
    (void)user;

    if(frame == NULL || frame->id != 0x601u || frame->len != 8u) {
        return false;
    }
    laser_accept_frame(frame->data, frame->rx_ms);
    return true;
}

bool laser_get_snapshot(LaserSnapshot* snapshot) {
    /* Sample time and RX stamp under the same IRQ mask. A CAN interrupt after
     * reading time must not make a fresh frame appear UINT32_MAX ms old. */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool ok = laser_get_snapshot_at(snapshot, HAL_GetTick());
    __set_PRIMASK(primask);
    return ok;
}

bool laser_get_snapshot_at(LaserSnapshot* snapshot, uint32_t now_ms) {
    uint32_t primask;
    uint32_t i;

    if(snapshot == NULL) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    snapshot->valid_mask = 0u;
    snapshot->updated_mask = 0u;
    for(i = 0u; i < LASER_CHANNEL_COUNT; ++i) {
        snapshot->distance_mm[i] = s_distance_mm[i];
        if(s_updated_mask & (1u << i))
            snapshot->updated_mask |= (uint8_t)(1u << i);
        snapshot->accepted_mm[i] = s_accepted_mm[i];
        snapshot->sample_age_ms[i] = (s_accepted_mask & (1u << i)) ?
            (uint32_t)(now_ms - s_accepted_ms[i]) : UINT32_MAX;
        if((s_accepted_mask & (1u << i)) && snapshot->sample_age_ms[i] <= LASER_FRAME_TIMEOUT_MS)
            snapshot->valid_mask |= (uint8_t)(1u << i);
    }
    snapshot->last_rx_ms = s_last_rx_ms;
    snapshot->frame_count = s_frame_count;
    __set_PRIMASK(primask);

    snapshot->frame_fresh = snapshot->frame_count != 0u &&
        (uint32_t)(now_ms - snapshot->last_rx_ms) <= LASER_FRAME_TIMEOUT_MS;
    return true;
}
