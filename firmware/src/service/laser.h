#ifndef MANGO_SWERVE_LASER_H
#define MANGO_SWERVE_LASER_H

#include <stdbool.h>
#include <stdint.h>

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

/* Copies an IRQ-safe snapshot; frame_fresh describes the aggregate CAN frame only. */
bool laser_get_snapshot(LaserSnapshot* snapshot);

#endif
