#ifndef MANGO_WORK_CONFIG_H
#define MANGO_WORK_CONFIG_H

#include "app_config.h"

/* Standalone commissioning is selected explicitly by its own firmware image. */
#define WORK_LINK_ENABLED (APP_RUN_MODE != APP_MODE_MANUAL_STANDALONE)

/* Initial commissioning values. Calibrate with the wheels unloaded first. */
#define WORK_APPROACH_SPEED_MPS 0.10f
/* C620 motor-shaft RPM, before the gearbox. All four wheels must qualify. */
#define WORK_STOP_MOTOR_RPM 30
#define WORK_FEEDBACK_TIMEOUT_MS 100u
#define WORK_STOP_STABLE_MS 200u
#define WORK_STOP_TIMEOUT_MS 2000U

#endif
