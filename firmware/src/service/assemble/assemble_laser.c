#include "assemble.h"

#include "can3_bus.h"
#include "laser.h"

SystemStatus assemble_laser(void) {
    return can3_bus_register_endpoint(0x601u, laser_accept_can3_frame, NULL) == CAN3_BUS_OK
        ? SYSTEM_STATUS_OK
        : SYSTEM_STATUS_ERROR;
}
