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

#endif
