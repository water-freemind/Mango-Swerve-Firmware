#ifndef MANGO_STATION_WORK_H
#define MANGO_STATION_WORK_H

#include "can3_bus.h"
#include "work_can_protocol.h"

typedef enum {
    STATION_LINK_SYNC, STATION_LINK_READY, STATION_LINK_APPROACH, STATION_LINK_WORKING,
    STATION_LINK_RELEASE, STATION_LINK_DONE, STATION_LINK_FAULT, STATION_LINK_ALIGN
} StationLinkPhase;

typedef struct {
    StationLinkPhase phase;
    uint32_t task, rx_count, tx_count, rejected_count;
    uint8_t station, result;
    bool online, hold, returned, slow_requested, stop_requested;
    bool align_seen, aligned;
    bool home_pending;
    int8_t align_direction;
    uint8_t align_sequence;
    uint32_t align_sample_ms, align_updates;
    uint32_t encoder_rx_count, encoder_sample_ms;
    uint8_t encoder_motor, encoder_phase, encoder_result;
    int16_t encoder_progress, encoder_target;
} StationWorkStatus;

extern StationWorkStatus g_station_work;
bool station_work_init(void);
void station_work_process(uint32_t now, bool healthy, bool stationary);
bool station_work_begin(uint8_t station, uint32_t now);
bool station_work_start(uint32_t now, bool stationary);
bool station_work_align(uint32_t now, bool stationary);
bool station_work_alignment_fresh(uint32_t now);
bool station_work_confirm_home(bool stationary);
bool station_work_reset(bool stationary);
void station_work_abort(uint8_t result);
bool station_work_ready(void);

#endif
