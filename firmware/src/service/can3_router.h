#ifndef MANGO_SWERVE_CAN3_ROUTER_H
#define MANGO_SWERVE_CAN3_ROUTER_H

#include <stdbool.h>
#include <stdint.h>

#define CAN3_ROUTER_MAX_ENDPOINTS 8u

typedef enum {
    CAN3_FRAME_STANDARD = 0,
    CAN3_FRAME_EXTENDED
} Can3FrameIdType;

typedef enum {
    CAN3_FRAME_DATA = 0,
    CAN3_FRAME_REMOTE
} Can3FrameType;

typedef enum {
    CAN3_FORMAT_CLASSIC = 0,
    CAN3_FORMAT_FD
} Can3FrameFormat;

typedef struct {
    uint32_t id;
    Can3FrameIdType id_type;
    Can3FrameType frame_type;
    Can3FrameFormat format;
    uint8_t len;
    uint8_t data[8];
    uint32_t rx_ms;
} Can3Frame;

typedef bool (*Can3EndpointCallback)(const Can3Frame* frame, void* user);

typedef enum {
    CAN3_ROUTER_OK = 0,
    CAN3_ROUTER_INVALID_PARAM,
    CAN3_ROUTER_DUPLICATE_ID,
    CAN3_ROUTER_FULL,
    CAN3_ROUTER_LOCKED
} Can3RouterStatus;

typedef struct {
    uint16_t std_id;
    Can3EndpointCallback callback;
    void* user;
} Can3Route;

typedef struct {
    Can3Route routes[CAN3_ROUTER_MAX_ENDPOINTS];
    uint8_t route_count;
    bool locked;
    uint32_t routed_count;
    uint32_t unknown_count;
    uint32_t rejected_count;
} Can3Router;

void can3_router_init(Can3Router* router);
Can3RouterStatus can3_router_register(Can3Router* router,
                                      uint16_t std_id,
                                      Can3EndpointCallback callback,
                                      void* user);
void can3_router_lock(Can3Router* router);
bool can3_router_get_id(const Can3Router* router, uint8_t index, uint16_t* std_id);
bool can3_router_dispatch(Can3Router* router, const Can3Frame* frame);

#endif
