#ifndef MANGO_AUTO_MISSION_H
#define MANGO_AUTO_MISSION_H
#include "auto_route.h"
#include "auto_work.h"
typedef enum { AUTO_PRECHECK, AUTO_WAIT_START, AUTO_NAVIGATE, AUTO_STOP_WORK,
    AUTO_WORK_HOLD, AUTO_RETRACT_VERIFY, AUTO_PAUSE, AUTO_FINISHED, AUTO_FAULT } AutoMissionPhase;
typedef enum { AUTO_MISSION_OK, AUTO_MISSION_CONFIG, AUTO_MISSION_START,
    AUTO_MISSION_HEALTH, AUTO_MISSION_DEADLINE, AUTO_MISSION_POSE,
    AUTO_MISSION_MOTION, AUTO_MISSION_WORK, AUTO_MISSION_PAUSE_TIMEOUT,
    AUTO_MISSION_PAUSE_LIMIT, AUTO_MISSION_COVERAGE, AUTO_MISSION_PARKING,
    AUTO_MISSION_SEGMENT_TIMEOUT, AUTO_MISSION_TIMING } AutoMissionError;
typedef enum { AUTO_START_NONE, AUTO_START_LOCAL_APPROVED, AUTO_START_HOST_DEBUG } AutoStartKind;
typedef struct {
    AutoConfig geometry; AutoMotionConfig motion; AutoWorkConfig work;
    AutoRoute route;
    uint32_t deadline_ms, pause_timeout_ms, pause_stable_ms, stop_work_ms, segment_timeout_ms;
    unsigned max_pauses; float reserve_s, segment_allowance_s, footprint_margin;
    bool simulation_only, map_confirmed;
} AutoMissionConfig;
typedef struct {
    bool present, classification_ok, reachable;
    uint8_t target_id, row, wire_region; uint32_t task;
} AutoTarget;
typedef struct {
    AutoMotionInput motion;
    bool precheck_ready, tools_stowed, observation_ok, localization_ok, cancel;
    AutoStartKind start_event;
    uint32_t penalty_ms;
    AutoTarget target;
    AutoWorkInput work;
} AutoMissionInput;
typedef struct {
    AutoMissionPhase phase; AutoMissionError first_fault;
    AutoTwist command, steering_hint;
    unsigned segment_index; uint8_t coverage;
    uint32_t handled_targets, actual_actions, rejected_targets;
    float elapsed_s, mandatory_remaining_s;
    bool budget_insufficient, qualified, started;
    AutoMotionError motion_fault; AutoWorkError work_fault;
    AutoWorkOutput work;
} AutoMissionOutput;
typedef struct {
    AutoMissionConfig config; AutoMissionOutput out; AutoMotion motion; AutoWork work;
    AutoTarget target; uint32_t start_ms, last_ms, phase_ms, segment_ms, pause_good_ms, completed_segments;
    unsigned pause_count; bool initialized, pause_good;
} AutoMission;
bool auto_mission_init(AutoMission* mission,const AutoMissionConfig* config,uint32_t now);
AutoMissionOutput auto_mission_step(AutoMission* mission,AutoMissionInput input);
const char* auto_mission_phase_name(AutoMissionPhase phase);
const char* auto_mission_error_name(AutoMissionError error);
#endif
