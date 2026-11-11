#include "station_work.h"
#include "work_config.h"
#include <string.h>

StationWorkStatus g_station_work;
static volatile WorkCanPacket inbox;
static volatile uint32_t inbox_ms;
static volatile bool pending;
static uint32_t peer_ms, sent_ms, started_ms, next_task;
static bool initialized, dirty, seen_peer, task_acknowledged;

static bool receive_station(const Can3Frame* frame, void* user) {
    WorkCanPacket packet;
    (void)user;
    if(frame->len != 8u || !station_can_decode(frame->data, &packet) ||
       !station_can_is_reply(packet.state)) return false;
    inbox = packet;
    inbox_ms = frame->rx_ms;
    pending = true;
    return true;
}

bool station_work_init(void) {
    memset(&g_station_work, 0, sizeof(g_station_work));
    pending = seen_peer = task_acknowledged = false;
    peer_ms = sent_ms = started_ms = next_task = 0u;
    dirty = true;
    initialized = true;
    if(WORK_STATION_ENABLED) {
        initialized = can3_bus_register_endpoint(WORK_CAN_REQUEST_ID,
            receive_station, NULL) == CAN3_BUS_OK;
    }
    return initialized;
}

bool station_work_ready(void) {
    return initialized && g_station_work.online &&
        (g_station_work.phase == STATION_LINK_READY ||
         g_station_work.phase == STATION_LINK_DONE);
}

void station_work_abort(uint8_t result) {
    if(g_station_work.phase == STATION_LINK_FAULT) return;
    g_station_work.hold = g_station_work.phase == STATION_LINK_WORKING ||
        g_station_work.phase == STATION_LINK_RELEASE;
    g_station_work.phase = STATION_LINK_FAULT;
    g_station_work.result = result;
    g_station_work.returned = false;
    dirty = true;
}

bool station_work_reset(bool stationary) {
    if(!stationary || g_station_work.phase == STATION_LINK_APPROACH ||
       g_station_work.phase == STATION_LINK_WORKING ||
       g_station_work.phase == STATION_LINK_RELEASE ||
       (g_station_work.hold && !g_station_work.returned)) return false;
    g_station_work.phase = STATION_LINK_SYNC;
    g_station_work.task = 0u;
    g_station_work.station = 0u;
    g_station_work.result = WORK_RESULT_OK;
    g_station_work.online = g_station_work.hold = false;
    g_station_work.returned = g_station_work.slow_requested = g_station_work.stop_requested = false;
    pending = seen_peer = task_acknowledged = false;
    dirty = true;
    return true;
}

bool station_work_begin(uint8_t station, uint32_t now) {
    if(!station_work_ready() || station < 1u || station > 12u ||
       next_task == UINT32_MAX) return false;
    g_station_work.task = ++next_task;
    g_station_work.station = station;
    g_station_work.result = WORK_RESULT_OK;
    g_station_work.phase = STATION_LINK_APPROACH;
    g_station_work.hold = false;
    g_station_work.returned = false;
    g_station_work.slow_requested = g_station_work.stop_requested = false;
    task_acknowledged = false;
    started_ms = now;
    dirty = true;
    return true;
}

bool station_work_start(uint32_t now, bool stationary) {
    if(g_station_work.phase != STATION_LINK_APPROACH ||
       !g_station_work.online || !g_station_work.stop_requested || !stationary)
        return false;
    g_station_work.phase = STATION_LINK_WORKING;
    g_station_work.hold = true;
    started_ms = now;
    dirty = true;
    return true;
}

static bool accept_reply(WorkCanPacket packet, uint32_t stamp) {
    if(packet.state == STATION_HELLO && packet.task == 0u && packet.region == 0u) {
        if(g_station_work.hold || g_station_work.phase == STATION_LINK_APPROACH)
            station_work_abort(WORK_RESULT_RESET);
        else if(g_station_work.phase != STATION_LINK_FAULT) {
            g_station_work.phase = STATION_LINK_SYNC;
            dirty = true;
        }
    } else if(packet.state == STATION_READY && packet.task == 0u && packet.region == 0u) {
        /* An idle READY may already be in flight when APPROACH is created.
         * Ignore it without extending freshness until this task is acknowledged. */
        if(g_station_work.phase == STATION_LINK_APPROACH && !task_acknowledged)
            return false;
        if(g_station_work.phase == STATION_LINK_APPROACH ||
           g_station_work.phase == STATION_LINK_WORKING ||
           g_station_work.phase == STATION_LINK_RELEASE) {
            station_work_abort(WORK_RESULT_RESET);
        } else if(g_station_work.phase == STATION_LINK_SYNC) {
            g_station_work.phase = STATION_LINK_READY;
        } else if(g_station_work.phase != STATION_LINK_READY) return false;
    } else {
        if(packet.task != g_station_work.task || packet.region != g_station_work.station)
            return false;
        if(g_station_work.phase == STATION_LINK_APPROACH &&
           (packet.state == STATION_TRACKING || packet.state == STATION_SLOW ||
            packet.state == STATION_WAIT)) {
            task_acknowledged = true;
            g_station_work.slow_requested = g_station_work.slow_requested ||
                packet.state == STATION_SLOW || packet.state == STATION_WAIT;
            g_station_work.stop_requested = g_station_work.stop_requested || packet.state == STATION_WAIT;
        } else if(packet.state == STATION_ABORTING || packet.state == STATION_FAULT) {
            station_work_abort(packet.result ? packet.result : WORK_RESULT_PROTOCOL);
            g_station_work.returned = packet.state == STATION_FAULT;
        } else if(packet.state == STATION_WORKING &&
                  g_station_work.phase == STATION_LINK_WORKING) {
            /* Retransmitted progress refreshes the peer watchdog. */
        } else if(packet.state == STATION_DONE &&
                  (g_station_work.phase == STATION_LINK_WORKING ||
                   g_station_work.phase == STATION_LINK_RELEASE)) {
            if(packet.result != WORK_RESULT_OK) station_work_abort(packet.result);
            else {
                g_station_work.phase = STATION_LINK_RELEASE;
                g_station_work.returned = true;
                dirty = true;
            }
        } else if(packet.state == STATION_RELEASED &&
                  (g_station_work.phase == STATION_LINK_RELEASE ||
                   g_station_work.phase == STATION_LINK_DONE)) {
            g_station_work.phase = STATION_LINK_DONE;
            g_station_work.hold = false;
        } else return false;
    }
    peer_ms = stamp;
    seen_peer = true;
    ++g_station_work.rx_count;
    return true;
}

void station_work_process(uint32_t now, bool healthy, bool stationary) {
    WorkCanPacket packet, command;
    uint32_t stamp;
    bool received;
    uint8_t bytes[8];
    uint32_t key;
    if(!WORK_STATION_ENABLED || !initialized) return;
    key = __get_PRIMASK();
    __disable_irq();
    received = pending;
    packet = inbox;
    stamp = inbox_ms;
    pending = false;
    now = HAL_GetTick();
    __set_PRIMASK(key);
    /* A completion acknowledgement cannot bypass a failed stop interlock. */
    if((g_station_work.phase == STATION_LINK_WORKING ||
        g_station_work.phase == STATION_LINK_RELEASE) && (!healthy || !stationary))
        station_work_abort(WORK_RESULT_FEEDBACK);
    if(received && ((uint32_t)(now - stamp) >= WORK_CAN_TIMEOUT_MS ||
       !accept_reply(packet, stamp))) ++g_station_work.rejected_count;
    g_station_work.online = seen_peer && (uint32_t)(now - peer_ms) < WORK_CAN_TIMEOUT_MS;
    if(g_station_work.phase == STATION_LINK_APPROACH &&
       (!healthy || !g_station_work.online || (uint32_t)(now - started_ms) >= 60000u))
        station_work_abort(!healthy ? WORK_RESULT_FEEDBACK : WORK_RESULT_LINK);
    if(g_station_work.phase == STATION_LINK_WORKING ||
       g_station_work.phase == STATION_LINK_RELEASE) {
        if(!healthy || !stationary) station_work_abort(WORK_RESULT_FEEDBACK);
        else if(!g_station_work.online) station_work_abort(WORK_RESULT_LINK);
        else if((uint32_t)(now - started_ms) >= WORK_STATION_TIMEOUT_MS)
            station_work_abort(WORK_RESULT_TREE_TIMEOUT);
    }
    command = (WorkCanPacket){g_station_work.task, STATION_START,
        g_station_work.station, g_station_work.result};
    if(g_station_work.phase == STATION_LINK_SYNC ||
       g_station_work.phase == STATION_LINK_READY) command.state = STATION_SYNC;
    else if(g_station_work.phase == STATION_LINK_APPROACH) command.state = STATION_APPROACH;
    else if(g_station_work.phase == STATION_LINK_RELEASE ||
            g_station_work.phase == STATION_LINK_DONE) command.state = STATION_RELEASE;
    else if(g_station_work.phase == STATION_LINK_FAULT) command.state = STATION_ABORT;
    if(dirty || (uint32_t)(now - sent_ms) >= WORK_CAN_PERIOD_MS) {
        station_can_encode(bytes, command);
        if(can3_bus_send(WORK_CAN_REPLY_ID, bytes, 8u) == CAN3_BUS_OK) {
            ++g_station_work.tx_count;
            sent_ms = now;
            dirty = false;
        }
    }
}
