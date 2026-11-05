#ifndef MANGO_SWERVE_LASER_H
#define MANGO_SWERVE_LASER_H

#include <stdbool.h>
#include <stdint.h>

#include "can3_router.h"

/* S21C connector order 1..4; physical directions are assigned after installation. */
#define LASER_CHANNEL_COUNT 4u
#define LASER_FRAME_TIMEOUT_MS 300u

typedef struct {
    uint16_t distance_mm[LASER_CHANNEL_COUNT];
    uint32_t last_rx_ms;
    uint32_t frame_count;
    bool frame_fresh;
} LaserSnapshot;

/* Called only after the complete 0x601 frame header has been validated. */
void laser_accept_frame(const uint8_t data[8], uint32_t rx_ms);

/* CAN3 endpoint for S21C standard ID 0x601, classic data frame, DLC 8. */
bool laser_accept_can3_frame(const Can3Frame* frame, void* user);

/* Copies an IRQ-safe snapshot; frame_fresh describes the aggregate CAN frame only. */
bool laser_get_snapshot(LaserSnapshot* snapshot);

/* Deterministic variant used by offline tests and time-controlled callers. */
bool laser_get_snapshot_at(LaserSnapshot* snapshot, uint32_t now_ms);

#endif
