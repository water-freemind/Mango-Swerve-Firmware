#ifndef MANGO_AUTO_ROUTE_H
#define MANGO_AUTO_ROUTE_H
#include "auto_motion.h"
#define AUTO_ROUTE_COUNT 17u
typedef enum { AUTO_ROUTE_ENTER, AUTO_ROUTE_OUTBOUND, AUTO_ROUTE_SURVEY,
    AUTO_ROUTE_SHIFT, AUTO_ROUTE_RETURN, AUTO_ROUTE_PARK } AutoRouteKind;
typedef struct { AutoSegment line; AutoRouteKind kind; uint8_t aisle, row; } AutoRouteSegment;
typedef struct { AutoRouteSegment segment[AUTO_ROUTE_COUNT]; float scale, entry_y; } AutoRoute;
bool auto_route_build(AutoRoute* route,float scale,float heading);
/* Calibrates only the out-of-field entry point; all field waypoints stay fixed. */
bool auto_route_set_entry_y(AutoRoute* route,float entry_y);
/* Conservative fixed-heading AABB continuous sweep, not endpoint-only check. */
bool auto_route_clear(const AutoRoute* route,AutoEnvelope body,float margin);
/* Conservative live AABB of the yawed footprint, including start/gate. */
bool auto_route_pose_clear(const AutoRoute* route,AutoEnvelope body,AutoPose pose,float margin);
bool auto_route_valid(const AutoRoute* route);
/* First front-facing intersection with known field fence/ridge faces. False at openings. */
bool auto_route_ray(const AutoRoute* route,AutoPose pose,AutoLaserMount mount,float max_range,
                    AutoPlane* plane,float* expected_range);
#endif
