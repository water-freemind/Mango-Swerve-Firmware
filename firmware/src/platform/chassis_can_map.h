#ifndef MANGO_SWERVE_CHASSIS_CAN_MAP_H
#define MANGO_SWERVE_CHASSIS_CAN_MAP_H

#include "stm32_hal_can.h"

/* Wiring exchanged on 2026-11-05: C620 feedback verified on CAN2.
 * Steering is connected to CAN1 according to the user's wiring check.
 * This changes both RX and TX. */
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
