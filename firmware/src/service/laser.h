#ifndef MANGO_SWERVE_LASER_H
#define MANGO_SWERVE_LASER_H

#include <stdbool.h>
#include <stdint.h>

#include "can3_router.h"

/* S21C connector order 1..4; physical directions are assigned after installation. */
#define LASER_CHANNEL_COUNT 4u
#define LASER_FRAME_TIMEOUT_MS 300u
#define LASER_ACCEPT_MAX_MM 15000u
#define LASER_FILTER_CONFIRM_MS 200u
#define LASER_FILTER_NEAR_MM 1100u /* Covers the maximum configured 1000 mm body envelope plus margin. */

typedef struct {
    /* Raw S21C millimetres in connector order. Range validity is established on the installed sensors. */
    uint16_t distance_mm[LASER_CHANNEL_COUNT];
    uint16_t accepted_mm[LASER_CHANNEL_COUNT]; /* Three-sample median, with immediate near returns. */
    uint32_t sample_age_ms[LASER_CHANNEL_COUNT];
    uint8_t valid_mask; /* Fresh accepted samples, one bit per channel. */
    uint8_t updated_mask; /* Accepted channels in the latest raw board frame. */
    uint16_t median_mm[LASER_CHANNEL_COUNT]; /* Continuous median, including short ranges. */
    uint32_t median_age_ms[LASER_CHANNEL_COUNT]; /* Age of the selected source sample. */
    uint8_t median_valid_mask, median_updated_mask;
    uint32_t last_rx_ms;
    uint32_t frame_count;
    bool frame_fresh; /* Freshness of the board CAN frame, evaluated with the 300 ms timeout. */
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
