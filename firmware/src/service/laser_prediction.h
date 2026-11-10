#ifndef MANGO_LASER_PREDICTION_H
#define MANGO_LASER_PREDICTION_H
#include "laser.h"

/* Total age since real measurement, not an extension renewed by predictions. */
#define LASER_PREDICT_MAX_MS 600u
#define LASER_PREDICT_MAX_PATH_M 0.20f
#define LASER_PREDICT_MAX_SPEED_MPS 0.50f
#define LASER_PREDICT_SPEED_CAP_MPS 0.15f
typedef struct {
    uint32_t last_ms, stamp[2];
    float x, path, turn, yaw;
    float anchor_x[2], anchor_path[2], anchor_turn[2];
    uint16_t anchor_mm[2];
    uint8_t anchors;
    bool initialized;
} LaserPrediction;
typedef struct {
    uint16_t mm[4];
    uint8_t mask;
} LaserPredictionOutput;

/* Only front/rear far measurements; no fresh bits or sensor timestamps invented.
 * Caller enables only healthy HOST running legs, never ARM/WAIT/guard recovery. */
LaserPredictionOutput laser_prediction_step(LaserPrediction* state, const LaserSnapshot* laser,
    uint32_t now, float vx, float vy, float yaw, bool enabled);
#endif
