#include "assemble.h"

#include "can3_bus.h"

SystemStatus assemble_can3(void) {
    return can3_bus_start() == CAN3_BUS_OK
        ? SYSTEM_STATUS_OK
        : SYSTEM_STATUS_ERROR;
}
