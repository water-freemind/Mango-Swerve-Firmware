#ifndef MANGO_SWERVE_APP_CONFIG_H
#define MANGO_SWERVE_APP_CONFIG_H

#define APP_MODE_CAN_DIAGNOSTICS 0
#define APP_MODE_MANUAL_DRIVE 1

/* Commissioning default: receive CAN traffic, without sending motor commands. */
#ifndef APP_RUN_MODE
#define APP_RUN_MODE APP_MODE_CAN_DIAGNOSTICS
#endif

#if APP_RUN_MODE != APP_MODE_CAN_DIAGNOSTICS && APP_RUN_MODE != APP_MODE_MANUAL_DRIVE
#error "Invalid APP_RUN_MODE"
#endif

#endif
