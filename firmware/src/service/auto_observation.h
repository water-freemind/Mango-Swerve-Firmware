#ifndef MANGO_AUTO_OBSERVATION_H
#define MANGO_AUTO_OBSERVATION_H
#include "auto_route.h"
typedef struct { AutoPoseSample previous; bool seen; } AutoObservation;
typedef struct {
    AutoPoseSample pose; AutoTwist measured, commanded;
    AutoRange ranges[4]; uint8_t required_mask; uint32_t now_ms;
} AutoObservationInput;
typedef struct {
    bool pose_consistent, required_ready, protective_stop;
    uint8_t valid_mask, opening_mask, stop_mask;
    AutoGeomResult reason[4]; AutoClearance clearance[4]; AutoPlane plane[4];
} AutoObservationOutput;
/* No integration. Checks consecutive pose vs one producer's measured velocity.
 * required_mask is route-phase policy; no ray at an opening is NOT valid clearance. */
AutoObservationOutput auto_observation_step(AutoObservation* monitor,const AutoConfig* config,
    const AutoRoute* route,AutoEnvelope body,AutoObservationInput input);
#endif
