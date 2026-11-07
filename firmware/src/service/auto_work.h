#ifndef MANGO_AUTO_WORK_H
#define MANGO_AUTO_WORK_H
#include <stdbool.h>
#include <stdint.h>
#include "work_can_protocol.h"

/* Uses existing v1 packet values. Epoch, action and retract evidence are local
 * adapter inputs, NOT invented v1 CAN fields. Wire contract is still unconfirmed. */
typedef struct {
    uint32_t stop_ms, work_ms, retract_ms, total_ms, stable_ms, evidence_age_ms;
    bool simulation_only, contract_confirmed;
} AutoWorkConfig;
typedef enum { AUTO_WORK_IDLE, AUTO_WORK_STOPPING, AUTO_WORK_READY,
    AUTO_WORK_ACTIVE, AUTO_WORK_RETRACT, AUTO_WORK_DONE, AUTO_WORK_FAULT } AutoWorkPhase;
typedef enum { AUTO_WORK_OK, AUTO_WORK_CONFIG, AUTO_WORK_LINK, AUTO_WORK_RESTART,
    AUTO_WORK_FEEDBACK, AUTO_WORK_STOP_TIMEOUT, AUTO_WORK_TIMEOUT,
    AUTO_WORK_RETRACT_TIMEOUT, AUTO_WORK_TOTAL_TIMEOUT, AUTO_WORK_CANCELLED,
    AUTO_WORK_PEER_FAULT } AutoWorkError;
typedef struct { uint32_t task, epoch, stamp_ms; bool valid; } AutoWorkEvidence;
typedef struct {
    uint32_t now_ms, peer_epoch, packet_stamp_ms;
    bool peer_online, healthy, stationary, cancel, packet_present;
    WorkCanPacket packet;
    AutoWorkEvidence retracted, action;
} AutoWorkInput;
typedef struct {
    AutoWorkPhase phase; AutoWorkError error; WorkCanPacket reply;
    bool hold, action_seen; uint32_t rejected;
} AutoWorkOutput;
typedef struct {
    AutoWorkConfig config; AutoWorkOutput out;
    uint32_t task, epoch, begin_ms, state_ms, peer_ms, stopped_ms, last_task, action_start_ms;
    bool stopped_timing, work_started;
} AutoWork;
bool auto_work_begin(AutoWork* work,AutoWorkConfig config,uint32_t task,uint8_t region,uint32_t epoch,uint32_t now);
AutoWorkOutput auto_work_step(AutoWork* work,AutoWorkInput input);
/* Parsing adapter only; this does not register CAN or transmit anything. */
bool auto_work_decode(const uint8_t bytes[8],WorkCanPacket* packet);
#endif
