#ifndef MANGO_SWERVE_CHASSIS_CAN_MAP_H
#define MANGO_SWERVE_CHASSIS_CAN_MAP_H

#include "stm32_hal_can.h"

/* Set only after physical bus identification. This changes both RX and TX. */
#ifndef CHASSIS_CAN_BUSES_SWAPPED
#define CHASSIS_CAN_BUSES_SWAPPED 0
#endif

#if CHASSIS_CAN_BUSES_SWAPPED == 0
#define CHASSIS_STEER_CAN (&hfdcan1)
#define CHASSIS_DRIVE_CAN (&hfdcan2)
#elif CHASSIS_CAN_BUSES_SWAPPED == 1
#define CHASSIS_STEER_CAN (&hfdcan2)
#define CHASSIS_DRIVE_CAN (&hfdcan1)
#else
#error "CHASSIS_CAN_BUSES_SWAPPED must be 0 or 1"
#endif

#endif
