#ifndef MANGO_WORK_LINK_H
#define MANGO_WORK_LINK_H

#include "can3_bus.h"
#include "work_can_protocol.h"

typedef struct {
    WorkCanPacket reply;
    uint32_t rx_count, tx_count, rejected_count;
    bool paired, active, hold;
} WorkLinkStatus;

extern WorkLinkStatus g_work_link;

/* Register before starting CAN3. Only the operational chassis entry calls this. */
bool work_link_init(void);
void work_link_process(uint32_t now_ms, bool chassis_ready);
bool work_link_hold(void);
void work_link_limit_velocity(float* vx, float* vy, float* wz);

#endif
