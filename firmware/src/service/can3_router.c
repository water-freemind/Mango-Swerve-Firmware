#include "can3_router.h"

#include <stddef.h>
#include <string.h>

void can3_router_init(Can3Router* router) {
    if(router != NULL) {
        memset(router, 0, sizeof(*router));
    }
}

Can3RouterStatus can3_router_register(Can3Router* router,
                                      uint16_t std_id,
                                      Can3EndpointCallback callback,
                                      void* user) {
    uint8_t i;

    if(router == NULL || callback == NULL || std_id > 0x7FFu) {
        return CAN3_ROUTER_INVALID_PARAM;
    }
    if(router->locked) {
        return CAN3_ROUTER_LOCKED;
    }
    for(i = 0u; i < router->route_count; ++i) {
        if(router->routes[i].std_id == std_id) {
            return CAN3_ROUTER_DUPLICATE_ID;
        }
    }
    if(router->route_count >= CAN3_ROUTER_MAX_ENDPOINTS) {
        return CAN3_ROUTER_FULL;
    }

    router->routes[router->route_count].std_id = std_id;
    router->routes[router->route_count].callback = callback;
    router->routes[router->route_count].user = user;
    ++router->route_count;
    return CAN3_ROUTER_OK;
}

void can3_router_lock(Can3Router* router) {
    if(router != NULL) {
        router->locked = true;
    }
}

bool can3_router_get_id(const Can3Router* router, uint8_t index, uint16_t* std_id) {
    if(router == NULL || std_id == NULL || index >= router->route_count) {
        return false;
    }
    *std_id = router->routes[index].std_id;
    return true;
}

bool can3_router_dispatch(Can3Router* router, const Can3Frame* frame) {
    uint8_t i;

    if(router == NULL || frame == NULL ||
       frame->id_type != CAN3_FRAME_STANDARD ||
       frame->frame_type != CAN3_FRAME_DATA ||
       frame->format != CAN3_FORMAT_CLASSIC ||
       frame->id > 0x7FFu || frame->len > 8u) {
        if(router != NULL) {
            ++router->rejected_count;
        }
        return false;
    }

    for(i = 0u; i < router->route_count; ++i) {
        if(router->routes[i].std_id == frame->id) {
            if(router->routes[i].callback(frame, router->routes[i].user)) {
                ++router->routed_count;
                return true;
            }
            ++router->rejected_count;
            return false;
        }
    }

    ++router->unknown_count;
    return false;
}
