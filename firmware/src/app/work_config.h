#ifndef MANGO_WORK_CONFIG_H
#define MANGO_WORK_CONFIG_H

#include "app_config.h"

/* Standalone commissioning is selected explicitly by its own firmware image. */
#define WORK_LINK_ENABLED (APP_RUN_MODE != APP_MODE_MANUAL_STANDALONE && APP_RUN_MODE != APP_MODE_STRAIGHT_TEST)

/* FullMapWork uses the mode-6 navigation entry with a version-3 work peer. */
#ifndef WORK_STATION_ENABLED
#define WORK_STATION_ENABLED 0
#endif
#if WORK_STATION_ENABLED && APP_RUN_MODE != APP_MODE_STRAIGHT_TEST
#error "Station work requires the full-map mode-6 entry"
#endif
#define WORK_STATION_TIMEOUT_MS 75000u

/* Initial commissioning values. Calibrate with the wheels unloaded first. */
#define WORK_APPROACH_SPEED_MPS 0.10f
/* C620 motor-shaft RPM, before the gearbox. All four wheels must qualify. */
#define WORK_STOP_MOTOR_RPM 30
#define WORK_FEEDBACK_TIMEOUT_MS 100u
#define WORK_STOP_STABLE_MS 200u
#define WORK_STOP_CONTINUE_WHEEL_RAD_S 0.16f
/* Strike impulses may exceed the continued-stop band briefly. Bound both
 * duration and estimated travel. Confirm a fast spike for 20ms while
 * retaining an immediate guard for larger wheel speeds. */
#define WORK_STOP_HARD_WHEEL_RAD_S 1.0f
#define WORK_STOP_HARD_CONFIRM_MS 20u
#define WORK_STOP_IMMEDIATE_WHEEL_RAD_S 3.0f
#define WORK_STOP_NOISE_MS 120u
#define WORK_STOP_EXCURSION_M 0.002f
#define WORK_STOP_TIMEOUT_MS 2000U

#endif
