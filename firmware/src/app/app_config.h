#ifndef MANGO_SWERVE_APP_CONFIG_H
#define MANGO_SWERVE_APP_CONFIG_H

#define APP_MODE_CAN_DIAGNOSTICS 0
#define APP_MODE_MANUAL_DRIVE 1
#define APP_MODE_STEERING_DIAGNOSTICS 2
#define APP_MODE_CAN2_QUERY 3
#define APP_MODE_STEERING_ZERO_TEST 4
#define APP_MODE_MANUAL_STANDALONE 5

/* Fast-gear caps. SWB selects the slow/medium/fast speed limits. */
#define APP_MANUAL_MAX_LINEAR_MPS 1.5f
#define APP_MANUAL_MAX_ANGULAR_RAD_S 6.0f

/* Initial heading loop values. Output is a chassis angular velocity in rad/s. */
#define APP_YAW_HOLD_KP 0.8f
#define APP_YAW_HOLD_KD 0.1f
#define APP_YAW_HOLD_MAX_WZ 0.03f

/* Commissioning default: receive CAN traffic, without sending motor commands. */
#ifndef APP_RUN_MODE
#define APP_RUN_MODE APP_MODE_CAN_DIAGNOSTICS
#endif

#if APP_RUN_MODE != APP_MODE_CAN_DIAGNOSTICS && APP_RUN_MODE != APP_MODE_MANUAL_DRIVE && APP_RUN_MODE != APP_MODE_STEERING_DIAGNOSTICS && APP_RUN_MODE != APP_MODE_CAN2_QUERY && APP_RUN_MODE != APP_MODE_STEERING_ZERO_TEST && APP_RUN_MODE != APP_MODE_MANUAL_STANDALONE
#error "Invalid APP_RUN_MODE"
#endif

#endif
