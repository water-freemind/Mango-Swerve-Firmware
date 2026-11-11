#include "work_link.h"
#include "work_config.h"
#include "chassis_can_map.h"
#include "station_work.h"

#include <math.h>
#include <string.h>

WorkLinkStatus g_work_link;
static bool initialized, seen_peer, sync_started, stopped_timing;
static volatile bool pending;
static bool reply_dirty;
static volatile WorkCanPacket inbox;
static volatile uint32_t inbox_ms;
static uint32_t last_peer_ms, wait_started, stopped_since, last_tx_ms;
static volatile uint32_t wheel_ms[4];
static volatile int16_t wheel_rpm[4];
static volatile uint8_t wheel_seen;

static void fail(uint8_t result) {
    g_work_link.reply.state = WORK_FAULT;
    g_work_link.reply.result = result;
    g_work_link.hold = true;
    reply_dirty = true;
}

static bool receive_request(const Can3Frame* frame, void* user) {
    WorkCanPacket packet;
    (void)user;
    if(frame->len != 8u || !work_can_decode(frame->data, &packet) || packet.region > 4u) {
        return false;
    }
    inbox = packet;
    inbox_ms = frame->rx_ms;
    pending = true;
    return true;
}

static void receive_wheel(FDCAN_HandleTypeDef* bus,
                          const FDCAN_RxHeaderTypeDef* header,
                          const uint8_t data[8], void* user) {
    uint32_t index;
    (void)bus;
    (void)user;
    if(header->IdType != FDCAN_STANDARD_ID || header->RxFrameType != FDCAN_DATA_FRAME ||
       header->FDFormat != FDCAN_CLASSIC_CAN || header->DataLength != FDCAN_DLC_BYTES_8 ||
       header->Identifier < 0x201u || header->Identifier > 0x204u) {
        return;
    }
    index = header->Identifier - 0x201u;
    wheel_rpm[index] = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    wheel_ms[index] = HAL_GetTick();
    wheel_seen |= (uint8_t)(1u << index);
}

bool work_link_init(void) {
    uint32_t i;
    memset(&g_work_link, 0, sizeof(g_work_link));
    for(i = 0u; i < 4u; ++i) {
        wheel_ms[i] = 0u;
        wheel_rpm[i] = 0;
    }
    wheel_seen = 0u;
    pending = seen_peer = sync_started = stopped_timing = false;
    last_peer_ms = wait_started = stopped_since = last_tx_ms = 0u;
    g_work_link.hold = true;
    g_work_link.reply.state = WORK_SYNC;
    reply_dirty = true;
    if(!WORK_LINK_ENABLED) {
        initialized = true;
        g_work_link.hold = false;
        g_work_link.reply.state = WORK_READY;
        return true;
    }
    initialized = can3_bus_register_endpoint(WORK_CAN_REQUEST_ID, receive_request, NULL) == CAN3_BUS_OK &&
        can_register_rx_callback(CHASSIS_DRIVE_CAN, receive_wheel, NULL) == STM32_HAL_CAN_OK;
    return initialized;
}

static bool accept_request(WorkCanPacket packet, uint32_t rx_ms) {
    uint8_t state = g_work_link.reply.state;
    if(state == WORK_FAULT) {
        return false;
    }
    if(packet.state == WORK_HELLO && packet.task == 0u) {
        if(g_work_link.paired) {
            fail(WORK_RESULT_RESET);
            return true;
        }
        sync_started = true;
        g_work_link.reply.state = WORK_SYNC;
    } else if(!g_work_link.paired) {
        if(!sync_started || packet.state != WORK_READY || packet.task != 0u) {
            return false;
        }
        g_work_link.paired = true;
        g_work_link.hold = false;
        g_work_link.reply.state = WORK_READY;
    } else if(packet.state == WORK_FAULT) {
        fail(packet.result);
    } else if((packet.state == WORK_SLOW || packet.state == WORK_WAIT) && !g_work_link.active &&
              packet.task == g_work_link.reply.task + 1u && packet.task != 0u) {
        g_work_link.reply = packet;
        g_work_link.active = true;
        if(packet.state == WORK_WAIT) {
            g_work_link.reply.state = WORK_STOPPING;
            g_work_link.hold = true;
            wait_started = rx_ms;
            stopped_timing = false;
        } else {
            g_work_link.reply.state = WORK_SLOW_ACK;
        }
    } else if(packet.task != g_work_link.reply.task) {
        return false;
    } else if(packet.state == WORK_READY && !g_work_link.active) {
        g_work_link.reply.state = WORK_READY;
    } else if(packet.state == WORK_SLOW && state == WORK_SLOW_ACK) {
        /* Repeated approach request. */
    } else if(packet.state == WORK_WAIT && state == WORK_SLOW_ACK) {
        g_work_link.reply.state = WORK_STOPPING;
        g_work_link.hold = true;
        wait_started = rx_ms;
        stopped_timing = false;
    } else if(packet.state == WORK_WAIT && (state == WORK_STOPPING || state == WORK_STOPPED)) {
        /* A lost STOPPED reply may leave the requester retransmitting WAIT. */
    } else if(packet.state == WORK_WORKING && state == WORK_STOPPED) {
        /* Keep the stop interlock for the entire tree. */
    } else if(packet.state == WORK_GO &&
              (state == WORK_STOPPED || state == WORK_GO_ACK ||
               (state == WORK_SLOW_ACK && packet.result == WORK_RESULT_SKIPPED))) {
        g_work_link.reply.state = WORK_GO_ACK;
        g_work_link.reply.result = packet.result;
        g_work_link.active = false;
        g_work_link.hold = false;
    } else {
        return false;
    }
    g_work_link.reply.region = packet.region;
    ++g_work_link.rx_count;
    reply_dirty = reply_dirty || state != g_work_link.reply.state;
    last_peer_ms = rx_ms;
    seen_peer = true;
    return true;
}

void work_link_process(uint32_t now_ms, bool chassis_ready) {
    WorkCanPacket packet;
    uint32_t received_ms = 0u, key, i;
    bool have_packet, wheels_fresh, wheels_stopped;
    uint8_t data[8];
    if(!WORK_LINK_ENABLED || !initialized) return;

    key = __get_PRIMASK();
    __disable_irq();
    now_ms = HAL_GetTick();
    have_packet = pending;
    packet = inbox;
    received_ms = inbox_ms;
    pending = false;
    wheels_fresh = wheel_seen == 0x0Fu;
    wheels_stopped = true;
    for(i = 0u; i < 4u; ++i) {
        wheels_fresh = wheels_fresh && (uint32_t)(now_ms - wheel_ms[i]) < WORK_FEEDBACK_TIMEOUT_MS;
        wheels_stopped = wheels_stopped && wheel_rpm[i] >= -WORK_STOP_MOTOR_RPM &&
            wheel_rpm[i] <= WORK_STOP_MOTOR_RPM;
    }
    __set_PRIMASK(key);

    /* Validate the existing stop before a GO packet can release its interlock. */
    if((g_work_link.reply.state == WORK_STOPPING || g_work_link.reply.state == WORK_STOPPED) &&
       (!chassis_ready || !wheels_fresh ||
        (g_work_link.reply.state == WORK_STOPPED && !wheels_stopped))) {
        fail(WORK_RESULT_FEEDBACK);
    }
    if(have_packet && !accept_request(packet, received_ms)) {
        ++g_work_link.rejected_count;
    }
    if(g_work_link.paired && seen_peer &&
       (uint32_t)(now_ms - last_peer_ms) >= WORK_CAN_TIMEOUT_MS &&
       g_work_link.reply.state != WORK_FAULT) {
        fail(WORK_RESULT_LINK);
    }
    if(g_work_link.reply.state == WORK_STOPPING || g_work_link.reply.state == WORK_STOPPED) {
        if(!chassis_ready || !wheels_fresh) {
            fail(WORK_RESULT_FEEDBACK);
        } else if(!wheels_stopped) {
            stopped_timing = false;
            if(g_work_link.reply.state == WORK_STOPPED) fail(WORK_RESULT_FEEDBACK);
        } else if(!stopped_timing) {
            stopped_timing = true;
            stopped_since = now_ms;
        } else if(g_work_link.reply.state == WORK_STOPPING &&
                  (uint32_t)(now_ms - stopped_since) >= WORK_STOP_STABLE_MS) {
            g_work_link.reply.state = WORK_STOPPED;
            reply_dirty = true;
        }
        if(g_work_link.reply.state == WORK_STOPPING &&
           (uint32_t)(now_ms - wait_started) >= WORK_STOP_TIMEOUT_MS) {
            fail(WORK_RESULT_STOP_TIMEOUT);
        }
    }
    g_work_link.reply.region &= 0x7Fu;
    if(wheels_fresh && !wheels_stopped && chassis_ready) {
        g_work_link.reply.region |= WORK_CAN_MOVING_FLAG;
    }
    if((sync_started || g_work_link.paired) &&
       (reply_dirty || (uint32_t)(now_ms - last_tx_ms) >= WORK_CAN_PERIOD_MS)) {
        work_can_encode(data, g_work_link.reply);
        if(can3_bus_send(WORK_CAN_REPLY_ID, data, 8u) == CAN3_BUS_OK) {
            ++g_work_link.tx_count;
            last_tx_ms = now_ms;
            reply_dirty = false;
        }
    }
}

bool work_link_hold(void) {
    return !initialized || (WORK_LINK_ENABLED && g_work_link.hold) ||
        (WORK_STATION_ENABLED && g_station_work.hold);
}

void work_link_limit_velocity(float* vx, float* vy, float* wz) {
    float speed;
    if(work_link_hold()) {
        *vx = *vy = *wz = 0.0f;
    } else if(g_work_link.active) {
        speed = sqrtf(*vx * *vx + *vy * *vy);
        if(speed > WORK_APPROACH_SPEED_MPS) {
            *vx *= WORK_APPROACH_SPEED_MPS / speed;
            *vy *= WORK_APPROACH_SPEED_MPS / speed;
        }
        *wz = 0.0f;
    }
}
