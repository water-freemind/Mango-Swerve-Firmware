#ifndef WORK_CAN_PROTOCOL_H
#define WORK_CAN_PROTOCOL_H

#include <stdint.h>

#define WORK_CAN_VERSION 1u
#define WORK_CAN_REQUEST_ID 0x610u
#define WORK_CAN_REPLY_ID 0x611u
#define WORK_CAN_PERIOD_MS 100u
#define WORK_CAN_TIMEOUT_MS 500u
#define WORK_CAN_MOVING_FLAG 0x80u

typedef enum {
    WORK_HELLO = 1, WORK_SYNC, WORK_READY, WORK_SLOW, WORK_SLOW_ACK,
    WORK_WAIT, WORK_STOPPING, WORK_STOPPED, WORK_WORKING,
    WORK_GO, WORK_GO_ACK, WORK_FAULT
} WorkCanState;

typedef enum {
    WORK_RESULT_OK = 0, WORK_RESULT_ATTEMPTS, WORK_RESULT_TREE_TIMEOUT,
    WORK_RESULT_VISION, WORK_RESULT_POSITION, WORK_RESULT_LINK,
    WORK_RESULT_STOP_TIMEOUT, WORK_RESULT_FEEDBACK, WORK_RESULT_RESET,
    WORK_RESULT_PROTOCOL, WORK_RESULT_SKIPPED
} WorkResult;

typedef struct {
    uint32_t task;
    uint8_t state, region, result;
} WorkCanPacket;

static inline void work_can_encode(uint8_t data[8], WorkCanPacket packet) {
    data[0] = WORK_CAN_VERSION;
    data[1] = packet.state;
    data[2] = (uint8_t)packet.task;
    data[3] = (uint8_t)(packet.task >> 8);
    data[4] = (uint8_t)(packet.task >> 16);
    data[5] = (uint8_t)(packet.task >> 24);
    data[6] = packet.region;
    data[7] = packet.result;
}

static inline uint8_t work_can_decode(const uint8_t data[8], WorkCanPacket* packet) {
    if(data[0] != WORK_CAN_VERSION || data[1] < WORK_HELLO ||
       data[1] > WORK_FAULT || (data[6] & 0x7Fu) > 4u || data[7] > WORK_RESULT_SKIPPED) {
        return 0u;
    }
    packet->state = data[1];
    packet->task = (uint32_t)data[2] | ((uint32_t)data[3] << 8) |
        ((uint32_t)data[4] << 16) | ((uint32_t)data[5] << 24);
    packet->region = data[6];
    packet->result = data[7];
    return 1u;
}

/* Version 3 is chassis-led station work. region carries station 1..12.
 * Commands travel on 0x611, replies on 0x610. Version 1 remains distinct. */
#define STATION_CAN_VERSION 3u
#define STATION_RESULT_CANCELLED 11u
#define STATION_RESULT_ENCODER_DIRECTION 12u
#define STATION_RESULT_ENCODER_LIMIT 13u
#define STATION_RESULT_ENCODER_STALL 14u
#define STATION_RESULT_ENCODER_TIMEOUT 15u
#define STATION_RESULT_MAX STATION_RESULT_ENCODER_TIMEOUT
#define STATION_VISION_CENTER_DX (-35)
#define STATION_VISION_STOP_PX 15
#define STATION_VISION_RESTART_PX 25
#define STATION_VISION_STABLE_MS 300u
#define STATION_VISION_MAX_AGE_MS 300u
typedef enum {
    STATION_SYNC = 1, STATION_READY, STATION_START, STATION_WORKING,
    STATION_DONE, STATION_RELEASE, STATION_RELEASED, STATION_ABORT,
    STATION_FAULT, STATION_ABORTING, STATION_HELLO, STATION_APPROACH,
    STATION_SLOW, STATION_WAIT, STATION_TRACKING,
    STATION_ALIGN, STATION_ALIGN_EAST, STATION_ALIGN_WEST, STATION_CENTERED,
    STATION_HOME, STATION_HOME_READY, STATION_ENCODER_DIAG
} StationCanState;

/* 对中反馈的最后一字节为视觉新帧序号，重复心跳保持原序号。 */
static inline uint8_t station_can_is_alignment(uint8_t state) {
    return state >= STATION_ALIGN_EAST && state <= STATION_CENTERED;
}

static inline uint8_t station_can_is_reply(uint8_t state) {
    return state == STATION_READY || state == STATION_WORKING ||
        state == STATION_DONE || state == STATION_RELEASED ||
        state == STATION_FAULT || state == STATION_ABORTING || state == STATION_HELLO || state == STATION_SLOW ||
        state == STATION_WAIT || state == STATION_TRACKING || state == STATION_HOME_READY ||
        state == STATION_ENCODER_DIAG || station_can_is_alignment(state);
}
static inline void station_can_encode(uint8_t data[8], WorkCanPacket packet) {
    work_can_encode(data, packet);
    data[0] = STATION_CAN_VERSION;
}
static inline uint8_t station_can_decode(const uint8_t data[8], WorkCanPacket* packet) {
    if(data[0] != STATION_CAN_VERSION || data[1] < STATION_SYNC ||
       data[1] > STATION_ENCODER_DIAG ||
       (data[1] == STATION_ENCODER_DIAG ?
        ((data[6]&7u)<1u || (data[6]&7u)>4u ||
         ((data[6]>>3)!=1u && (data[6]>>3)!=3u)) : data[6]>12u) ||
       (!station_can_is_alignment(data[1]) && data[7] > STATION_RESULT_MAX))
        return 0u;
    packet->state = data[1];
    packet->task = (uint32_t)data[2] | ((uint32_t)data[3] << 8) |
        ((uint32_t)data[4] << 16) | ((uint32_t)data[5] << 24);
    packet->region = data[6];
    packet->result = data[7];
    return 1u;
}

#endif
