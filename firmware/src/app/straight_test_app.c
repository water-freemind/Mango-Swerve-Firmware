#include "app_config.h"
#if APP_RUN_MODE == APP_MODE_STRAIGHT_TEST
#include "straight_test_app.h"
#include "straight_test.h"
#include "assemble/assemble.h"
#include "chassis.h"
#include "chassis_yaw_hold.h"
#include "steering_calibration.h"
#include "chassis_can_map.h"
#include "stm32_hal_can.h"
#include "stm32_hal_uart.h"
#include "stm32_hal_dwt.h"
#include "imu/imu.h"
#include "imu/bmi088.h"
#include "remote.h"
#include "drive_source.h"
#include "chassis_profile.h"
#include "chassis_trace.h"
#include "work_link.h"
#include "laser.h"
#include "laser_prediction.h"
#include "full_map_test.h"
#include "log.h"
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

static StraightTest test;
static FullMapTest full_map;
static uint32_t last_map_log;
static LaserPrediction laser_prediction;
static LaserPredictionOutput predicted_laser;
static uint8_t prediction_logged_mask;
static bool initialized;
static uint32_t last_step, last_log, last_imu, last_plan_log;
static volatile uint32_t motor_ms[8];
static volatile uint8_t seen, motor_state[4];
static uint8_t byte;
static volatile uint8_t rx_ring[128], rx_head, rx_tail;
static volatile bool rx_fault;
static char line[48];
static unsigned line_len;
static const float zeros[4] = STEERING_FORWARD_ZERO_RAD;
static bool tx_fault;
static DriveSource control_source;
static DriveVelocity manual_velocity;
static DriveVelocity manual_target;
static ChassisTrace trace;
static ChassisTraceSample trace_sensor;
static bool trace_recording, trace_was_active, trace_dumping, trace_header;
static uint32_t trace_dump_sequence, trace_dump_end, trace_dump_sent, trace_last_export;
static unsigned trace_chunk_offset;
static char trace_row[CHASSIS_TRACE_CSV_CAPACITY];
static const char* trace_stop_reason = "BOOT";
static uint32_t last_manual_target;
static uint32_t last_manual_process;

/* Capture each sensor frame once. BMI088 get_sample() consumes its NEW flags. */
static ImuSample imu_sample;
static bool imu_seen;
static ImuStatus imu_update_status = IMU_STATUS_NOT_READY;
static uint32_t imu_dt_us, imu_dt_max_us, imu_samples, imu_errors;
static uint32_t control_dt_ms, control_dt_max_ms, detail_slot;
static uint32_t motor_age[8], blockers;
static uint8_t feedback_seen, enabled_mask;
static RemoteState rc_state;
static bool rc_online, rc_switches, rc_centered;
static Bmi088AttitudeDebug imu_debug;
static bool imu_debug_valid;
static uint32_t imu_age_us;
static bool drift_tracking, drift_valid;
static uint32_t drift_start_us, drift_last_us, drift_window_ms;
static float drift_last_yaw, drift_yaw_sum, drift_gyro_sum, drift_yaw_rate, drift_gyro_rate;
enum {
    BLOCK_STEER_RX = 1u << 0, BLOCK_DRIVE_RX = 1u << 1, BLOCK_STEER_STATE = 1u << 2,
    BLOCK_RC_OFFLINE = 1u << 3, BLOCK_RC_SWITCH = 1u << 4, BLOCK_RC_STICK = 1u << 5,
    BLOCK_IMU_SAMPLE = 1u << 6, BLOCK_IMU_STALE = 1u << 7, BLOCK_IMU_CAL = 1u << 8,
    BLOCK_NONFINITE = 1u << 9, BLOCK_UART = 1u << 10, BLOCK_TX = 1u << 11,
    BLOCK_CAN = 1u << 12
};
static const char* first_blocker(void) {
    static const char* const names[] = {"STEER_FEEDBACK", "DRIVE_FEEDBACK", "STEER_STATE",
        "RC_OFFLINE", "RC_SWITCH", "RC_STICK", "IMU_SAMPLE", "IMU_STALE", "IMU_CALIB",
        "NONFINITE", "UART_RX", "CAN_TX", "CAN_BUS"};
    for(unsigned i = 0u; i < sizeof(names) / sizeof(names[0]); ++i)
        if(blockers & (1u << i)) return names[i];
    return "NONE";
}
static void update_imu_sample(void) {
    ImuSample fresh;
    imu_update_status = imu.update();
    if(imu_update_status != IMU_STATUS_OK) { ++imu_errors; return; }
    if(imu_get_sample(&fresh) != IMU_STATUS_OK || !(fresh.flags & IMU_SAMPLE_GYRO_VALID)) return;
    if(imu_seen) {
        imu_dt_us = fresh.gyro_timestamp_us - imu_sample.gyro_timestamp_us;
        if(imu_dt_us > imu_dt_max_us) imu_dt_max_us = imu_dt_us;
    }
    imu_sample = fresh;
    imu_seen = true;
    ++imu_samples;
}

static void stop_outputs(void) {
    manual_target = manual_velocity = (DriveVelocity){0};
    last_manual_target = HAL_GetTick();
    /* Clear the cached chassis command as well as the physical motor outputs,
     * so the next owner cannot process the previous owner's velocity. */
    (void)chassis.set_velocity(0.0f, 0.0f, 0.0f);
    chassis_reset_control_history();
    for(uint16_t id = 1u; id <= 4u; ++id) {
        BusMotorFeedback f;
        if(drive_motor.stop(id) != MOTOR_STATUS_OK) tx_fault = true;
        /* No position command until this boot has actually received the motor angle. */
        if(steer_motor.update_feedback(id, &f) == MOTOR_STATUS_OK && isfinite(f.position)) {
            if(steer_motor.brake(id) != MOTOR_STATUS_OK) tx_fault = true;
        } else if(steer_motor.disable(id) != MOTOR_STATUS_OK) tx_fault = true;
    }
}

static void received(void) {
    const uint8_t next = (uint8_t)((rx_head + 1u) & 127u);
    if(next == rx_tail) rx_fault = true;
    else { rx_ring[rx_head] = byte; rx_head = next; }
    if(!uart_receive_it(&huart1, &byte, 1u)) rx_fault = true;
}
static void uart_error(void) { rx_fault = true; }
static void motor_received(FDCAN_HandleTypeDef* bus, const FDCAN_RxHeaderTypeDef* h,
                           const uint8_t data[8], void* user) {
    unsigned index;
    (void)user;
    if(h->IdType != FDCAN_STANDARD_ID || h->RxFrameType != FDCAN_DATA_FRAME ||
       h->FDFormat != FDCAN_CLASSIC_CAN || h->DataLength != FDCAN_DLC_BYTES_8) return;
    if(bus == CHASSIS_STEER_CAN && h->Identifier >= 0x11u && h->Identifier <= 0x14u) {
        index = h->Identifier - 0x11u;
        if((data[0] & 15u) != index + 1u) return;
        motor_state[index] = data[0] >> 4;
    } else if(bus == CHASSIS_DRIVE_CAN && h->Identifier >= 0x201u && h->Identifier <= 0x204u) {
        index = 4u + h->Identifier - 0x201u;
    } else return;
    motor_ms[index] = HAL_GetTick();
    seen |= (uint8_t)(1u << index);
}
static bool remote_permit(void) {
    remote_process();
    rc_online = remote_get_state(&rc_state) && remote_is_online(100u);
    rc_state.online = rc_online;
    rc_switches = rc_state.manual_request && rc_state.manual_source == REMOTE_MANUAL_SOURCE_CHASSIS &&
        drive_source_requested(&rc_state) != DRIVE_SOURCE_STOP &&
        rc_state.rc_data.channel[REMOTE_CH_VRA] > REMOTE_VR_LOW_THRESHOLD &&
        (drive_source_requested(&rc_state) != DRIVE_SOURCE_MANUAL || drive_source_manual_vrb_ok(&rc_state));
    /* Manual additionally requires the spring-return VRB permit; HOST does not. */
    rc_centered = true;
    const unsigned channels[] = {REMOTE_CH_RIGHT_X, REMOTE_CH_RIGHT_Y, REMOTE_CH_LEFT_X};
    for(unsigned i = 0u; i < 3u; ++i)
        if((unsigned)abs((int)rc_state.rc_data.channel[channels[i]] - (int)REMOTE_CENTER) > REMOTE_DEADBAND)
            rc_centered = false;
    if(!rc_online) blockers |= BLOCK_RC_OFFLINE;
    else {
        if(!rc_switches) blockers |= BLOCK_RC_SWITCH;
        if(!rc_centered && drive_source_requested(&rc_state) == DRIVE_SOURCE_HOST) blockers |= BLOCK_RC_STICK;
    }
    return rc_online && rc_switches && rc_centered;
}
static StraightInput input(uint32_t now) {
    StraightInput in = {.healthy = true, .stationary = true, .vx_mps = 0.0f};
    uint32_t stamp[8];
    BusMotorFeedback steer_sample[4], drive_sample[4];
    BusMotorStatus steer_status[4], drive_status[4];
    blockers = 0u;
    const uint32_t key = __get_PRIMASK();
    __disable_irq();
    feedback_seen = seen;
    enabled_mask = 0u;
    for(unsigned i = 0u; i < 8u; ++i) stamp[i] = motor_ms[i];
    for(unsigned i = 0u; i < 4u; ++i)
        if(motor_state[i] == 1u) enabled_mask |= (uint8_t)(1u << i);
    /* Cache-only reads: keep value/status pairs with their CAN receipt stamps.
     * No SPI, CAN transmit, formatting or floating-point trigonometry here. */
    for(unsigned i=0u;i<4u;++i) {
        steer_status[i]=steer_motor.update_feedback((uint16_t)(i+1u),&steer_sample[i]);
        drive_status[i]=drive_motor.update_feedback((uint16_t)(i+1u),&drive_sample[i]);
    }
    __set_PRIMASK(key);
    /* Read time after copying stamps: an RX interrupt during SPI work may be
     * newer than the loop's original 'now', which must not underflow its age. */
    now = HAL_GetTick();
    trace_sensor = (ChassisTraceSample){.t_ms = now};
    for(unsigned i = 0u; i < 8u; ++i) motor_age[i] = now - stamp[i];
    for(unsigned i = 0u; i < 8u; ++i)
        if(!(feedback_seen & (1u << i)) || motor_age[i] > 100u)
            blockers |= i < 4u ? BLOCK_STEER_RX : BLOCK_DRIVE_RX;
    if(enabled_mask != 15u) blockers |= BLOCK_STEER_STATE;
    (void)remote_permit(); /* Evaluate all conditions, even when another one fails. */
    if(rx_fault && drive_source_requested(&rc_state) == DRIVE_SOURCE_HOST) blockers |= BLOCK_UART;
    if(tx_fault) blockers |= BLOCK_TX;
    imu_age_us = imu_seen ? (uint32_t)(dwt_get_us() - imu_sample.gyro_timestamp_us) : UINT32_MAX;
    if(!imu_seen || imu_update_status != IMU_STATUS_OK) blockers |= BLOCK_IMU_SAMPLE;
    if(imu_age_us > 100000u) blockers |= BLOCK_IMU_STALE;
    imu_debug_valid = bmi088_get_attitude_debug(&imu_debug) == IMU_STATUS_OK;
    if(!imu_debug_valid || !imu_debug.calibrated || !imu_debug.has_angle) blockers |= BLOCK_IMU_CAL;
    for(uint16_t id = 1u; id <= 4u; ++id) {
        const float sign = (id == 2u || id == 3u) ? -1.0f : 1.0f;
        const float speed = drive_status[id-1u] == MOTOR_STATUS_OK ? drive_sample[id-1u].speed : NAN;
        const float angle = steer_status[id-1u] == MOTOR_STATUS_OK ? steer_sample[id-1u].position - zeros[id-1u] : NAN;
        trace_sensor.wheel_rad_s[id-1u] = sign * speed;
        trace_sensor.steer_rad[id-1u] = angle;
        trace_sensor.steer_age_ms[id-1u] = (uint16_t)(motor_age[id-1u] > 65535u ? 65535u : motor_age[id-1u]);
        trace_sensor.drive_age_ms[id-1u] = (uint16_t)(motor_age[id+3u] > 65535u ? 65535u : motor_age[id+3u]);
        if((feedback_seen & (1u << (id-1u))) && motor_age[id-1u] <= 100u && isfinite(angle))
            trace_sensor.steer_valid_mask |= (uint8_t)(1u << (id-1u));
        if((feedback_seen & (1u << (id+3u))) && motor_age[id+3u] <= 100u && isfinite(speed))
            trace_sensor.drive_valid_mask |= (uint8_t)(1u << (id-1u));
        if(!isfinite(speed) || !isfinite(angle)) blockers |= BLOCK_NONFINITE;
        in.stationary = in.stationary && fabsf(speed) < 0.05f;
        in.vx_mps += sign * speed * STRAIGHT_TEST_WHEEL_RADIUS_M * cosf(angle) * 0.25f;
        in.vy_mps -= sign * speed * STRAIGHT_TEST_WHEEL_RADIUS_M * sinf(angle) * 0.25f;
    }
    const ImuAngle attitude = imu.get_angle();
    const ImuGyro gyro = imu_get_gyro_corrected();
    trace_sensor.yaw_rad = attitude.yaw;
    trace_sensor.gyro_z_rad_s = gyro.z;
    trace_sensor.imu_age_us = imu_age_us;
    trace_sensor.imu_valid = imu_debug_valid && imu_debug.calibrated && imu_debug.has_angle && imu_seen && imu_update_status == IMU_STATUS_OK &&
        imu_age_us <= 100000u && isfinite(attitude.yaw) && isfinite(gyro.z);
    if(!isfinite(attitude.yaw) || !isfinite(gyro.z)) blockers |= BLOCK_NONFINITE;
    LaserSnapshot laser = {0};
    if(laser_get_snapshot(&laser)) {
        const bool predict_enabled = blockers == 0u && drive_source_requested(&rc_state) == DRIVE_SOURCE_HOST &&
            test.phase == ST_RUNNING && test.plan == ST_PLAN_WALL && test.leg != ST_LEG_WAIT &&
            !test.guard_mask && !test.laser_paused;
        predicted_laser = laser_prediction_step(&laser_prediction, &laser, now, in.vx_mps, in.vy_mps,
            attitude.yaw, predict_enabled);
        in.laser_fresh = laser.frame_fresh && laser.frame_count != 0u;
        in.left_mm = predicted_laser.mm[0];
        in.right_mm = predicted_laser.mm[1];
        in.rear_mm = predicted_laser.mm[2];
        in.front_mm = predicted_laser.mm[3];
        in.laser_predicted_mask = predicted_laser.mask;
        in.laser_stale_mask = (uint8_t)(~(laser.valid_mask | predicted_laser.mask) & 15u);
        in.laser_updated_mask = laser.updated_mask;
        in.laser_frame = laser.frame_count;
        in.laser_age_ms = HAL_GetTick() - laser.last_rx_ms;
    }
    in.healthy = blockers == 0u;
    if(prediction_logged_mask != in.laser_predicted_mask) {
        prediction_logged_mask = in.laser_predicted_mask;
        log_info("TEST_PREDICT mask=0x%x rear_mm=%u front_mm=%u real_age_ms=%lu,%lu max_age_ms=%u max_path_mm=200 speed_cap=0.15",
            prediction_logged_mask, in.rear_mm, in.front_mm, (unsigned long)laser.sample_age_ms[2],
            (unsigned long)laser.sample_age_ms[3], LASER_PREDICT_MAX_MS);
    }
    return in;
}
static bool trace_safe_to_export(StraightInput in) {
    const unsigned swd = rc_state.rc_data.channel[REMOTE_CH_SWD];
    return in.stationary && rc_online && swd >= 900u && swd <= 1100u &&
        !control_source.manual_armed && !rc_state.manual_request &&
        (blockers & (BLOCK_STEER_RX | BLOCK_DRIVE_RX | BLOCK_NONFINITE)) == 0u &&
        test.phase != ST_RUNNING && test.phase != ST_ARMED && test.phase != ST_STOPPING;
}
static void trace_capture(StraightInput in) {
    const bool manual_motion = control_source.mode == DRIVE_SOURCE_MANUAL &&
        (manual_target.vx != 0 || manual_target.vy != 0 || manual_target.wz != 0 ||
         manual_velocity.vx != 0 || manual_velocity.vy != 0 || manual_velocity.wz != 0);
    const bool active = manual_motion || !in.stationary || test.phase == ST_RUNNING || test.phase == ST_STOPPING;
    if(trace_recording && !trace_dumping) {
        ChassisTraceSample s = trace_sensor;
        s.mode = (uint8_t)control_source.mode;
        s.permitted = (uint8_t)(control_source.manual_armed || (control_source.mode == DRIVE_SOURCE_HOST && rc_switches && rc_online));
        s.blocker = blockers;
        s.fault = (control_source.mode == DRIVE_SOURCE_HOST && test.phase == ST_FAULT ? 1u : 0u) |
            (tx_fault ? 2u : 0u) | (control_source.mode == DRIVE_SOURCE_HOST && rx_fault ? 4u : 0u);
        s.control_dt_ms = (uint16_t)(control_dt_ms > 65535u ? 65535u : control_dt_ms);
        if(control_source.mode == DRIVE_SOURCE_MANUAL) {
            s.target[0]=manual_target.vx; s.target[1]=manual_target.vy; s.target[2]=manual_target.wz;
            s.limited[0]=manual_velocity.vx; s.limited[1]=manual_velocity.vy; s.limited[2]=manual_velocity.wz;
        } else if(control_source.mode == DRIVE_SOURCE_HOST && test.phase == ST_RUNNING && test.leg != ST_LEG_WAIT) {
            s.target[0]=s.limited[0]=test.speed_mps;
            s.target[1]=s.limited[1]=test.lateral_mps;
            s.target[2]=0.0f; s.limited[2]=chassis_yaw_hold_get_output_wz();
        }
        (void)chassis_trace_push(&trace,&s);
        if(trace_was_active && (!active || s.fault)) {
            trace_recording=false;
            trace_stop_reason = control_source.mode == DRIVE_SOURCE_HOST &&
                (test.phase == ST_FAULT || test.phase == ST_DONE) ? test.result : "MANUAL_RELEASE";
            log_info("TEST_TRACE_STATE recording=0 reason=%s count=%lu",trace_stop_reason,(unsigned long)trace.count);
        }
    }
    trace_was_active=active;
}
static void trace_command(const char* cmd, StraightInput in) {
    if(!strcmp(cmd,"TRACE START") && !trace_dumping) {
        trace_recording=true; trace_stop_reason="CAPTURE_RUNNING";
    } else if(!strcmp(cmd,"TRACE STOP")) {
        trace_recording=false; trace_stop_reason="CAPTURE_STOP";
    } else if(!strcmp(cmd,"TRACE CLEAR") && trace_safe_to_export(in) && !trace_dumping) {
        chassis_trace_init(&trace); trace_stop_reason="CLEARED";
    } else if(!strcmp(cmd,"TRACE DUMP") && trace_safe_to_export(in) && !trace_dumping) {
        trace_recording=false; trace_dumping=true; trace_header=true;
        trace_dump_sequence=chassis_trace_oldest(&trace); trace_dump_end=trace.next_sequence;
        trace_dump_sent=0; trace_chunk_offset=0; trace_row[0]='\0';
        log_info("TEST_TRACE_META expected_count=%lu first=%lu next=%lu overwritten=%lu reason=%s",
            (unsigned long)trace.count,(unsigned long)trace_dump_sequence,(unsigned long)trace_dump_end,
            (unsigned long)trace.overwritten,trace_stop_reason);
    } else {
        log_info("TEST_TRACE_STATE rejected=INVALID_BUSY_OR_NOT_STOPPED require=SWD_OFF_FRESH_STATIONARY");
        return;
    }
    log_info("TEST_TRACE_STATE recording=%u dumping=%u count=%lu reason=%s motor_command=NONE",
        (unsigned)trace_recording,(unsigned)trace_dumping,(unsigned long)trace.count,trace_stop_reason);
}
static void trace_export(uint32_t now,StraightInput in) {
    if(!trace_dumping) return;
    if(!trace_safe_to_export(in)) {
        trace_dumping=false;
        log_info("TEST_TRACE_STATE rejected=EXPORT_CANCELLED_NOT_STOPPED");
        return;
    }
    if(now-trace_last_export<20u) return;
    trace_last_export=now;
    if(trace_row[0]=='\0') {
        if(trace_header) {
            const char* header=chassis_trace_csv_header();
            if(strlen(header)>=sizeof(trace_row)) {trace_dumping=false;return;}
            strcpy(trace_row,header);
        } else if(trace_dump_sequence==trace_dump_end) {
            trace_dumping=false;
            log_info("TEST_TRACE_END sent_count=%lu",(unsigned long)trace_dump_sent);
            return;
        } else {
            ChassisTraceSample sample;
            const ChassisProfile* profile=chassis_profile_for_mode(APP_RUN_MODE);
            if(!profile || !chassis_trace_get(&trace,trace_dump_sequence,&sample) ||
               !chassis_trace_csv_row(&sample,&profile->model,trace_row,sizeof(trace_row))) {
                trace_dumping=false; log_info("TEST_TRACE_STATE rejected=EXPORT_FORMAT_OR_DATA"); return;
            }
        }
    }
    const unsigned length=(unsigned)strlen(trace_row), total=(length+149u)/150u;
    const unsigned part=trace_chunk_offset/150u+1u;
    const unsigned size=length-trace_chunk_offset>150u ? 150u : length-trace_chunk_offset;
    char chunk[151]; memcpy(chunk,trace_row+trace_chunk_offset,size);chunk[size]='\0';
    if(trace_header) log_info("TEST_TRACE seq=HEADER part=%u total=%u data=%s",part,total,chunk);
    else log_info("TEST_TRACE seq=%lu part=%u total=%u data=%s",(unsigned long)trace_dump_sequence,part,total,chunk);
    trace_chunk_offset+=size;
    if(trace_chunk_offset>=length) {
        trace_row[0]='\0';trace_chunk_offset=0;
        if(trace_header) trace_header=false;
        else {++trace_dump_sequence;++trace_dump_sent;}
    }
}
static bool hardware_healthy(void);
static FullMapInput map_input(uint32_t now,StraightInput in) {
    FullMapInput m={.now=now,.wheel={in.vx_mps,in.vy_mps,imu_get_gyro_corrected().z},
        .imu_yaw=imu.get_angle().yaw,.healthy=in.healthy,.odom_ok=hardware_healthy(),
        .stationary=in.stationary,.aligned=chassis_drive_output_allowed()};
    (void)laser_get_snapshot(&m.laser);
    return m;
}
static void commands(uint32_t now, StraightInput in) {
    if(rx_fault) {
        if(full_map_active(&full_map)) full_map_stop(&full_map,"UART_ERROR_OR_OVERFLOW",false);
        straight_fault(&test, "UART_ERROR_OR_OVERFLOW");
        rx_tail = rx_head;
        line_len = 0u;
        (void)uart_abort_receive_it(&huart1);
        rx_fault = false;
        if(!uart_receive_it(&huart1, &byte, 1u)) rx_fault = true;
        return;
    }
    /* Bound work per loop; sustained flooding cannot starve the control watchdog. */
    for(unsigned n = 0u; n < 128u && rx_tail != rx_head; ++n) {
        const uint8_t ch = rx_ring[rx_tail];
        rx_tail = (uint8_t)((rx_tail + 1u) & 127u);
        if(ch == '\r' || ch == '\n') {
            if(line_len == 0u) continue;
            line[line_len] = '\0';
            if(!strncmp(line,"TRACE ",6u)) {
                trace_command(line,in); line_len=0u; continue;
            }
            if(!strncmp(line,"MAP ",4u)) {
                bool ok=false;
                const bool read_or_stop=!strcmp(line,"MAP STATUS")||!strcmp(line,"MAP STOP");
                const bool straight_busy=test.phase==ST_ARMED||test.phase==ST_RUNNING||test.phase==ST_STOPPING;
                const bool allowed=read_or_stop || (control_source.mode==DRIVE_SOURCE_HOST && !straight_busy);
                if(allowed)
                    ok=full_map_command(&full_map,line,map_input(now,in));
                log_info("TEST_MAP_ACK ok=%u phase=%s token=%lu segment=%u reason=%s cmd=%s",
                    (unsigned)ok,full_map_phase_name(full_map.phase),(unsigned long)full_map.token,
                    full_map.segment,allowed?full_map.reason:"OWNER_OR_STRAIGHT_ACTIVE",line+4);
                if(!strcmp(line,"MAP STATUS")) {
                    const AutoPoseSample p=full_map.localization.pose;
                    const uint32_t age=now-p.stamp_ms;
                    const bool valid=full_map.localization.registered&&p.valid&&hardware_healthy()&&age<=50u;
                    log_info("TEST_MAP_SNAPSHOT valid=%u age_ms=%lu x=%.3f y=%.3f yaw_deg=%.2f phase=%s segment=%u coverage=0x%x",
                        (unsigned)valid,(unsigned long)age,(double)p.pose.x,(double)p.pose.y,(double)p.pose.yaw*57.295779513,
                        full_map_phase_name(full_map.phase),full_map.segment,full_map.coverage);
                }
                line_len=0u;continue;
            }
            const StraightPhase before = test.phase;
            bool ok;
            if(!strcmp(line, "STOP")) {
                if(full_map_active(&full_map)) full_map_stop(&full_map,"STOP",false);
                drive_source_stop_manual(&control_source);
                manual_velocity = (DriveVelocity){0};
                chassis_yaw_hold_disable();
                stop_outputs();
            }
            if(full_map_active(&full_map) && strcmp(line,"PING") && strcmp(line,"STOP")) {
                ok=false;test.result="FULL_MAP_ACTIVE";
            } else if(control_source.mode != DRIVE_SOURCE_HOST &&
               (!strncmp(line, "ARM", 3u) || !strncmp(line, "GO ", 3u) || !strncmp(line, "HB ", 3u))) {
                ok = false;
                test.result = "HOST_NOT_SELECTED";
            } else if(control_source.manual_armed &&
                      (!strncmp(line, "SPEED ", 6u) || !strncmp(line, "ACCEL ", 6u) || !strncmp(line, "EDGE ", 5u))) {
                ok = false;
                test.result = "MANUAL_ACTIVE";
            } else ok = straight_command(&test, line, now, in);
            if(before == ST_ARMED && test.phase == ST_RUNNING) {
                const ImuAngle a = imu.get_angle();
                chassis_yaw_hold_set_target(a.yaw);
            }
            const char* command_name = !strncmp(line, "SPEED ", 6u) ? "SPEED" : !strncmp(line, "ACCEL ", 6u) ? "ACCEL" : !strncmp(line, "EDGE ", 5u) ? "EDGE" :
                !strncmp(line, "ARM", 3u) ? "ARM" : !strncmp(line, "GO ", 3u) ? "GO" : !strcmp(line, "STOP") ? "STOP" : "PING";
            if(line[0] != 'H') log_info("TEST_ACK ok=%u phase=%u token=%lu reason=%s plan=%s target_m=%.2f body_edge_mm=%u vmax=%.2f accel=%.2f cmd=%s",
                (unsigned)ok, (unsigned)test.phase, (unsigned long)test.token, test.result,
                straight_plan_name(test.plan), (double)test.target_m, test.body_edge_mm, (double)test.max_speed_mps, (double)test.accel_mps2, command_name);
            line_len = 0u;
        } else if(ch < 32u || ch > 126u || line_len >= sizeof(line) - 1u) {
            rx_fault = true;
            break;
        } else line[line_len++] = (char)ch;
    }
}
static bool hardware_healthy(void) {
    return (blockers & ~(BLOCK_RC_OFFLINE | BLOCK_RC_SWITCH | BLOCK_RC_STICK | BLOCK_UART)) == 0u;
}
static void log_control(uint32_t now) {
    log_info("TEST_CONTROL t_ms=%lu source=%s manual_armed=%u release_seen=%u centered=%u hw_ok=%u vx=%.3f vy=%.3f wz=%.3f hb_required=0 vrb_required=%u vrb_ok=%u",
        (unsigned long)now, drive_source_name(control_source.mode), (unsigned)control_source.manual_armed,
        (unsigned)control_source.release_seen, (unsigned)rc_centered, (unsigned)hardware_healthy(),
        (double)manual_velocity.vx, (double)manual_velocity.vy, (double)manual_velocity.wz,
        (unsigned)(control_source.mode == DRIVE_SOURCE_MANUAL), (unsigned)drive_source_manual_vrb_ok(&rc_state));
}
static void update_control_source(uint32_t now, StraightInput in) {
    const bool was_manual = control_source.manual_armed;
    if(drive_source_update(&control_source, &rc_state, hardware_healthy(), in.stationary)) {
        if(full_map_active(&full_map)||full_map.localization.registered)
            full_map_stop(&full_map,"CONTROL_MODE_CHANGED",true);
        const float wheel_limit = control_source.mode == DRIVE_SOURCE_HOST ?
            STRAIGHT_TEST_SPEED_CEILING_MPS : APP_MANUAL_MAX_LINEAR_MPS;
        if(chassis_set_wheel_speed_limit(wheel_limit) != chassis.OK) {
            drive_source_stop_manual(&control_source);
            tx_fault = true;
        }
        /* Invalidate the previous owner's task and queued bytes before the
         * next owner can write motor outputs. No task resumes on switching. */
        if(test.phase == ST_ARMED || test.phase == ST_RUNNING || test.phase == ST_STOPPING)
            straight_fault(&test, "CONTROL_MODE_CHANGED");
        rx_tail = rx_head;
        line_len = 0u;
        manual_velocity = (DriveVelocity){0};
        last_manual_process = now;
        chassis_yaw_hold_disable();
        stop_outputs();
        log_control(now);
    } else if(was_manual && !control_source.manual_armed) {
        manual_velocity = (DriveVelocity){0};
        chassis_yaw_hold_disable();
        stop_outputs();
        log_control(now);
    }
    if(!was_manual && control_source.manual_armed) {
        manual_target = manual_velocity = (DriveVelocity){0};
        last_manual_target = now;
    }
}
static void apply_manual(uint32_t now, StraightInput in) {
    (void)in;
    manual_target = drive_source_velocity(&rc_state);
    const uint32_t dt_ms = now - last_manual_target;
    last_manual_target = now;
    /* Free manual motion: only the sticks command rotation. HOST owns the
     * automatic heading hold used by the test programs. */
    chassis_yaw_hold_disable();
    if(dt_ms > 50u) {
        drive_source_stop_manual(&control_source);
        straight_fault(&test, "MANUAL_CONTROL_TIMING");
        stop_outputs();
        return;
    }
    manual_velocity = manual_target;
    if(chassis.set_velocity(manual_velocity.vx, manual_velocity.vy, manual_velocity.wz) != chassis.OK) {
        drive_source_stop_manual(&control_source);
        straight_fault(&test, "MANUAL_CONTROL");
        stop_outputs();
    }
}
static bool can_healthy(void) {
    STM32HalCanDiagnostics a, b;
    return can_get_diagnostics(CHASSIS_STEER_CAN, &a) && can_get_diagnostics(CHASSIS_DRIVE_CAN, &b) &&
        !a.bus_off && !b.bus_off && !a.error_passive && !b.error_passive &&
        a.tx_failed_count == 0u && b.tx_failed_count == 0u;
}
static void update_drift(StraightInput in) {
    const uint32_t bad_data = BLOCK_STEER_RX | BLOCK_DRIVE_RX | BLOCK_IMU_SAMPLE |
        BLOCK_IMU_STALE | BLOCK_IMU_CAL | BLOCK_NONFINITE;
    /* Wheel feedback cannot detect someone rotating/lifting the entire robot.
     * This is an observation window, never an automatic bias adjustment. */
    if((blockers & bad_data) || !in.stationary ||
       (test.phase != ST_IDLE && test.phase != ST_DONE && test.phase != ST_FAULT)) {
        drift_tracking = drift_valid = false;
        return;
    }
    const uint32_t stamp = imu_sample.gyro_timestamp_us;
    const float yaw = imu.get_angle().yaw;
    if(!drift_tracking) {
        drift_tracking = true;
        drift_start_us = drift_last_us = stamp;
        drift_last_yaw = yaw;
        drift_yaw_sum = drift_gyro_sum = 0.0f;
        return;
    }
    if(stamp == drift_last_us) return;
    const uint32_t dt = stamp - drift_last_us;
    drift_yaw_sum += remainderf(yaw - drift_last_yaw, 6.283185307f);
    drift_gyro_sum += imu_get_gyro_corrected().z * (float)dt * 1.0e-6f;
    drift_last_us = stamp;
    drift_last_yaw = yaw;
    const uint32_t elapsed = stamp - drift_start_us;
    if(elapsed >= 5000000u) {
        const float factor = 57.295779513f * 1.0e6f / (float)elapsed;
        drift_yaw_rate = drift_yaw_sum * factor;
        drift_gyro_rate = drift_gyro_sum * factor;
        drift_window_ms = elapsed / 1000u;
        drift_valid = true;
        drift_start_us = stamp;
        drift_yaw_sum = drift_gyro_sum = 0.0f;
    }
}
static void log_detail(uint32_t now, StraightInput in) {
    /* One bounded line per 100 ms, rather than a burst that blocks the controller.
     * All formats fit the existing 256-byte logger at documented sensor ranges. */
    const unsigned slot = detail_slot++ % 16u;
    if(slot == 0u) {
        log_info("TEST_HEALTH t_ms=%lu blocked=0x%04lx first=%s seen=0x%02x steer_enabled=0x%x rc_ok=%u imu_ok=%u uart_fault=%u tx_fault=%u",
            (unsigned long)now, (unsigned long)blockers, first_blocker(), (unsigned)feedback_seen,
            (unsigned)enabled_mask, (unsigned)(rc_online && rc_switches &&
                (control_source.mode == DRIVE_SOURCE_MANUAL || rc_centered)),
            (unsigned)((blockers & (BLOCK_IMU_SAMPLE | BLOCK_IMU_STALE | BLOCK_IMU_CAL)) == 0u),
            (unsigned)rx_fault, (unsigned)tx_fault);
    } else if(slot == 1u) {
        const uint16_t* ch = rc_state.rc_data.channel;
        log_info("TEST_REMOTE t_ms=%lu online=%u switches_ok=%u centered=%u source=%u SWA=%u SWB=%u SWC=%u SWD=%u VRA=%u VRB=%u RX=%u RY=%u LX=%u vrb_required=%u",
            (unsigned long)now, (unsigned)rc_online, (unsigned)rc_switches, (unsigned)rc_centered,
            (unsigned)rc_state.manual_source, ch[REMOTE_CH_SWA], ch[REMOTE_CH_SWB], ch[REMOTE_CH_SWC],
            ch[REMOTE_CH_SWD], ch[REMOTE_CH_VRA], ch[REMOTE_CH_VRB], ch[REMOTE_CH_RIGHT_X],
            ch[REMOTE_CH_RIGHT_Y], ch[REMOTE_CH_LEFT_X], (unsigned)(control_source.mode == DRIVE_SOURCE_MANUAL));
    } else if(slot == 2u) {
        const ImuAngle a = imu.get_angle();
        log_info("TEST_IMU t_ms=%lu ready=%u cal=%u/%u age_us=%lu temp_C=%.2f ref_C=%.2f yaw_deg=%.3f pitch_deg=%.3f roll_deg=%.3f gz_rad_s=%.6f",
            (unsigned long)now, (unsigned)(imu_debug_valid && imu_debug.calibrated && imu_debug.has_angle),
            (unsigned)imu_debug.calib_count, (unsigned)imu_debug.calib_target, (unsigned long)imu_age_us,
            (double)imu_debug.temperature, (double)imu_debug.gyro_temp_ref,
            (double)a.yaw * 57.295779513, (double)a.pitch * 57.295779513, (double)a.roll * 57.295779513,
            (double)imu_debug.gyro_corrected.z);
    } else if(slot == 3u) {
        log_info("TEST_IMU_RAW t_ms=%lu unit=rad/s raw=%.6f,%.6f,%.6f startup_bias=%.6f,%.6f,%.6f corrected=%.6f,%.6f,%.6f",
            (unsigned long)now, (double)imu_sample.gyro.x, (double)imu_sample.gyro.y, (double)imu_sample.gyro.z,
            (double)imu_debug.gyro_bias.x, (double)imu_debug.gyro_bias.y, (double)imu_debug.gyro_bias.z,
            (double)imu_debug.gyro_corrected.x, (double)imu_debug.gyro_corrected.y, (double)imu_debug.gyro_corrected.z);
    } else if(slot == 4u) {
        log_info("TEST_IMU_BIAS t_ms=%lu applied_z_rad_s=%.6f temp_z_rad_s=%.6f slope_rad_s_C=%.7f intercept_rad_s=%.6f zru=%u active=%u policy=BOOT_MEAN",
            (unsigned long)now, (double)imu_debug.gyro_z_bias_effective, (double)imu_debug.gyro_temp_comp.z,
            (double)imu_debug.gyro_temp_coeff.z, (double)imu_debug.gyro_z_temp_intercept,
            (unsigned)imu_debug.zru_enabled, (unsigned)imu_debug.zru_active);
    } else if(slot <= 8u) {
        const uint16_t id = (uint16_t)(slot - 4u);
        static const char* const wheel[] = {"FL", "FR", "RR", "RL"};
        const unsigned i = id - 1u;
        log_info("TEST_MOTOR t_ms=%lu id=%u wheel=%s steer_seen=%u drive_seen=%u steer_age_ms=%lu drive_age_ms=%lu state=%u raw_rad=%.4f wheel_rad=%.4f drive_rad_s=%.4f current_raw=%.0f",
            (unsigned long)now, (unsigned)id, wheel[i], (unsigned)((feedback_seen >> i) & 1u),
            (unsigned)((feedback_seen >> (i + 4u)) & 1u), (unsigned long)motor_age[i],
            (unsigned long)motor_age[i + 4u], (unsigned)motor_state[i], (double)steer_motor.get_pos(id),
            (double)remainderf(steer_motor.get_pos(id) - zeros[i], 6.283185307f),
            (double)drive_motor.get_spd(id), (double)drive_motor.get_tor(id));
    } else if(slot <= 11u) {
        STM32HalCanDiagnostics d = {0};
        const unsigned bus = slot - 8u;
        FDCAN_HandleTypeDef* const handles[] = {&hfdcan1, &hfdcan2, &hfdcan3};
        const bool ok = can_get_diagnostics(handles[bus - 1u], &d);
        log_info("TEST_CAN t_ms=%lu bus=%u ok=%u rx=%lu tx=%lu done=%lu tx_fail=%lu rx_fail=%lu lost=%lu off=%u passive=%u proto_err=%lu hal=0x%lx",
            (unsigned long)now, bus, (unsigned)ok, (unsigned long)d.rx_count, (unsigned long)d.tx_submit_count,
            (unsigned long)d.tx_complete_count, (unsigned long)d.tx_failed_count, (unsigned long)d.rx_failed_count,
            (unsigned long)d.rx_fifo0_lost_count, (unsigned)d.bus_off, (unsigned)d.error_passive,
            (unsigned long)d.protocol_error_count, (unsigned long)d.hal_error_code);
    } else if(slot == 12u) {
        LaserSnapshot laser = {0};
        (void)laser_get_snapshot(&laser);
        log_info("TEST_LASER t_ms=%lu fresh=%u frames=%lu age_ms=%lu left_mm=%u right_mm=%u rear_mm=%u front_mm=%u control=%s",
            (unsigned long)now, (unsigned)laser.frame_fresh, (unsigned long)laser.frame_count,
            (unsigned long)(now - laser.last_rx_ms), laser.distance_mm[0], laser.distance_mm[1],
            laser.distance_mm[2], laser.distance_mm[3], test.plan == ST_PLAN_WALL ? "RIGHT_HOLD" :
                test.plan == ST_PLAN_APPROACH ? "FRONT_STOP" : test.plan == ST_PLAN_ROUND ? "COLLISION_GUARD" : "OBSERVE_ONLY");
        log_info("TEST_LASER_VALID mask=0x%x age_ms=%lu,%lu,%lu,%lu max_mm=15000 updated=0x%x",
            laser.valid_mask, (unsigned long)laser.sample_age_ms[0], (unsigned long)laser.sample_age_ms[1],
            (unsigned long)laser.sample_age_ms[2], (unsigned long)laser.sample_age_ms[3], laser.updated_mask);
    } else if(slot == 13u) {
        log_info("TEST_TIMING t_ms=%lu control_dt_ms=%lu max_ms=%lu imu_dt_us=%lu imu_max_us=%lu imu_frames=%lu imu_errors=%lu update=%s hb_required=0",
            (unsigned long)now, (unsigned long)control_dt_ms, (unsigned long)control_dt_max_ms,
            (unsigned long)imu_dt_us, (unsigned long)imu_dt_max_us, (unsigned long)imu_samples,
            (unsigned long)imu_errors, imu.status_str(imu_update_status));
    } else if(slot == 14u) {
        log_info("TEST_DRIFT t_ms=%lu valid=%u wheel_still=%u window_ms=%lu yaw_deg_s=%.6f gyro_deg_s=%.6f yaw_deg_min=%.3f body_static=USER_CONFIRM auto_zero=0",
            (unsigned long)now, (unsigned)drift_valid, (unsigned)in.stationary, (unsigned long)drift_window_ms,
            (double)drift_yaw_rate, (double)drift_gyro_rate, (double)drift_yaw_rate * 60.0);
    } else {
        if(control_source.mode == DRIVE_SOURCE_MANUAL)
            log_info("TEST_RAMP t_ms=%lu target=%.3f,%.3f,%.3f output=%.3f,%.3f,%.3f reversing=0 direct=1",
                (unsigned long)now, (double)manual_target.vx, (double)manual_target.vy, (double)manual_target.wz,
                (double)manual_velocity.vx, (double)manual_velocity.vy, (double)manual_velocity.wz);
        log_control(now);
    }
}
static void log_plan(StraightInput in) {
    log_info("TEST_PLAN proto=2 plan=%s leg=%s target_m=%.2f x=%.3f out=%.3f vy=%.4f right_ref_mm=%.1f laser_ok=%u body_edge_mm=%u front_target_mm=%u vmax=%.2f accel=%.2f hb_required=0",
        straight_plan_name(test.plan), straight_leg_name(test.leg), (double)test.target_m,
        (double)test.distance_m, (double)test.outbound_m, (double)test.lateral_mps,
        (double)test.right_ref_mm, (unsigned)straight_motion_laser_ready(&test, in), test.body_edge_mm,
        straight_front_target_mm(&test), (double)test.max_speed_mps, (double)test.accel_mps2);
    if(test.plan == ST_PLAN_WALL)
        log_info("TEST_WALL ref_mm=%.1f right_mm=%u filtered_mm=%.1f error_mm=%.1f vy_cmd=%.4f vy_meas=%.4f cap=0.08",
            (double)test.right_ref_mm, in.right_mm, (double)test.right_filtered_mm,
            (double)(test.right_ref_mm - test.right_filtered_mm), (double)test.lateral_mps, (double)in.vy_mps);
    log_info("TEST_RANGE fresh=%u age_ms=%lu left_mm=%u right_mm=%u rear_mm=%u front_mm=%u",
        (unsigned)in.laser_fresh, (unsigned long)in.laser_age_ms, in.left_mm, in.right_mm, in.rear_mm, in.front_mm);
    uint16_t guards[4];
    straight_guard_limits(&test, in, guards);
    log_info("TEST_BRAKE left_mm=%u right_mm=%u rear_mm=%u front_mm=%u decel_mps2=0.10 delay_ms=500 active=%u",
        guards[0], guards[1], guards[2], guards[3],
        (unsigned)(control_source.mode == DRIVE_SOURCE_HOST && test.plan != ST_PLAN_FWD));
}
void straight_test_app_init(void) {
    initialized = false;
    chassis_trace_init(&trace);
    trace_recording=true;trace_dumping=trace_was_active=false;
    trace_stop_reason="BOOT";
    control_source = (DriveSource){0};
    (void)assemble_delay();
    if(assemble_log() != SYSTEM_STATUS_OK) return;
    straight_init(&test, HAL_GetTick());
    full_map_init(&full_map, HAL_GetTick());
    if(assemble_imu() != SYSTEM_STATUS_OK || assemble_laser() != SYSTEM_STATUS_OK ||
       !work_link_init() || assemble_can3() != SYSTEM_STATUS_OK ||
       can_register_rx_callback(CHASSIS_STEER_CAN, motor_received, NULL) != STM32_HAL_CAN_OK ||
       can_register_rx_callback(CHASSIS_DRIVE_CAN, motor_received, NULL) != STM32_HAL_CAN_OK ||
       assemble_chassis() != SYSTEM_STATUS_OK || assemble_remote() != SYSTEM_STATUS_OK) {
        straight_fault(&test, "INIT_FAILED");
        if(chassis.get_chassis()->initialized) stop_outputs();
        log_error("TEST_INIT_FAILED");
        return;
    }
    remote_init();
    const ChassisYawHoldConfig heading = {.enabled = true, .kp = STRAIGHT_TEST_YAW_KP,
        .kd = APP_YAW_HOLD_KD, .v_deadband = 0.01f, .wz_deadband = 0.0f, .wz_limit = STRAIGHT_TEST_YAW_MAX_WZ};
    chassis_yaw_hold_init(&heading);
    uart_register_rx_complete_callback(&huart1, received);
    uart_register_error_callback(&huart1, uart_error);
    rx_fault = !uart_receive_it(&huart1, &byte, 1u);
    (void)chassis.set_steer_then_drive_enabled(true);
    stop_outputs();
    initialized = true;
    last_step = last_imu = HAL_GetTick();
    log_info("TEST_READY mode=6 target=%.2fm default_speed=0.50 accel=0.20 ceiling=2.00 radius=0.051 NOMINAL FWD_laser_limit=0; PING ARM GO <token> HB <token> STOP",
        (double)STRAIGHT_TEST_TARGET_M);
    log_info("TEST_READY imu_bias=BOOT_MEAN keep_body_static_until_ready=1 calibration_samples=2000 detail_period_ms=1600 FW_period_ms=100");
    log_info("TEST_READY remote_policy=SWD_LATCH SWD_permit=2000 SWD_stop=1000 host_vrb_required=0 manual_vrb_max=%u manual_heading_hold=0", DRIVE_MANUAL_VRB_MAX);
    log_info("TEST_READY proto=2 programs=FWD,ROUND,WALL,APPROACH round_leg_m=1,2,4 body_edge_mm=110 guard_mm=130 front_gap_mm=100 max_approach_m=1");
    log_info("TEST_READY control=SWC_1000_STOP_1500_HOST_2000_MANUAL SWD_1000_STOP_2000_PERMIT hb_required=0 manual_reenable=SWD_OFF_CENTER_ON");
    log_info("TEST_READY manual_control=direct ramp=0");
    log_info("TEST_READY full_map_proto=1 field=EMPTY_3x4.6 start=0.20,-0.50,90 default_speed=0.10 tools=DISABLED auto_start=0");
    log_info("TEST_READY laser_filter=MEDIAN3 window_gap_ms=200 near_bypass_mm=1100 laser_recovery_ms=5000 brake_delay_ms=500");
    log_info("TEST_READY manual_jerk linear=0.80 angular=1.60 target_lateral_budget=0.30 transient_budget_not_guaranteed=1");
}
void straight_test_app_loop(void) {
    const uint32_t now = HAL_GetTick();
    if(!initialized) return;
    can_poll_rx(CHASSIS_STEER_CAN);
    can_poll_rx(CHASSIS_DRIVE_CAN);
    can_poll_rx(&hfdcan3);
    if(now - last_imu >= 4u) {
        last_imu = now;
        update_imu_sample();
    }
    StraightInput in = input(now);
    if(!can_healthy()) blockers |= BLOCK_CAN;
    in.healthy = blockers == 0u;
    update_control_source(now, in);
    if(control_source.mode != DRIVE_SOURCE_HOST) in.healthy = false;
    commands(now, in);
    update_drift(in);
    if(rx_fault) straight_fault(&test, "UART_ERROR_OR_OVERFLOW");
    if(control_source.mode == DRIVE_SOURCE_HOST && test.phase == ST_RUNNING) {
        const ImuAngle a = imu.get_angle();
        const float error = remainderf(chassis_yaw_hold_get_yaw_ref() - a.yaw, 6.283185307f);
        if(!isfinite(error) || fabsf(error) > 0.15f) straight_fault(&test, "HEADING_DEVIATION");
    }
    if(now - last_step >= 10u) {
        control_dt_ms = now - last_step;
        if(control_dt_ms > control_dt_max_ms) control_dt_max_ms = control_dt_ms;
        last_step = now;
        if(control_source.mode==DRIVE_SOURCE_HOST) {
            const FullMapPhase before=full_map.phase;
            full_map_step(&full_map,map_input(now,in));
            if(before!=full_map.phase) log_info("TEST_MAP_EVENT t_ms=%lu phase=%s segment=%u reason=%s",
                (unsigned long)now,full_map_phase_name(full_map.phase),full_map.segment,full_map.reason);
        }
        if(control_source.mode == DRIVE_SOURCE_HOST && !full_map_active(&full_map)) {
            const StraightPhase before = test.phase;
            const bool was_laser_paused = test.laser_paused;
            const uint8_t previous_guard = test.guard_mask;
            uint16_t guards[4];
            straight_guard_limits(&test, in, guards);
            straight_step(&test, now, in);
            if(test.phase == ST_RUNNING && (was_laser_paused != test.laser_paused ||
               (!previous_guard && test.guard_mask))) {
                log_info("TEST_PAUSE token=%lu result=%s laser_wait=%u guard_mask=0x%x stale_mask=0x%x x=%.3f vx=%.3f",
                    (unsigned long)test.token, test.result, (unsigned)test.laser_paused, test.guard_mask,
                    in.laser_stale_mask, (double)test.distance_m, (double)in.vx_mps);
                log_info("TEST_PAUSE_RANGE mm=%u,%u,%u,%u limits=%u,%u,%u,%u frame=%lu updated=0x%x",
                    in.left_mm, in.right_mm, in.rear_mm, in.front_mm, guards[0], guards[1], guards[2], guards[3],
                    (unsigned long)in.laser_frame, in.laser_updated_mask);
            }
            if(before != ST_FAULT && test.phase == ST_FAULT) {
                log_error("TEST_FAULT token=%lu reason=%s leg=%s x=%.3f fresh=%u age_ms=%lu frame=%lu stale_mask=0x%x",
                    (unsigned long)test.token, test.result, straight_leg_name(test.leg), (double)test.distance_m,
                    (unsigned)in.laser_fresh, (unsigned long)in.laser_age_ms, (unsigned long)in.laser_frame, in.laser_stale_mask);
                log_error("TEST_FAULT_RANGE left_mm=%u right_mm=%u rear_mm=%u front_mm=%u guards=%u,%u,%u,%u vx=%.3f vy=%.3f",
                    in.left_mm, in.right_mm, in.rear_mm, in.front_mm,
                    guards[0], guards[1], guards[2], guards[3], (double)in.vx_mps, (double)in.vy_mps);
            }
        }
        const bool map_running=control_source.mode==DRIVE_SOURCE_HOST && full_map.phase==MAP_RUNNING;
        const bool map_paused=control_source.mode==DRIVE_SOURCE_HOST &&
            (full_map.phase==MAP_ARMED||full_map.phase==MAP_PAUSED);
        const bool host_active = !map_running && !map_paused && control_source.mode == DRIVE_SOURCE_HOST && test.phase == ST_RUNNING &&
            test.leg != ST_LEG_WAIT && !test.guard_mask && !test.laser_paused;
        const bool host_paused = control_source.mode == DRIVE_SOURCE_HOST && test.phase == ST_RUNNING && !host_active;
        if(control_source.mode == DRIVE_SOURCE_MANUAL && control_source.manual_armed) apply_manual(now, in);
        else if(map_running) {
            chassis_yaw_hold_disable();
            const AutoTwist v=full_map.command,h=full_map.hint;
            const bool prepare=full_map.motion.out.phase==AUTO_MOTION_ALIGN;
            const ChassisErrorCode status=prepare ? chassis_prepare_velocity(h.vx,h.vy,h.wz):chassis.set_velocity(v.vx,v.vy,v.wz);
            if(status!=chassis.OK||chassis.process()!=chassis.OK) {
                full_map_stop(&full_map,"CHASSIS_CONTROL",true);stop_outputs();
            }
        }
        else if(host_active) {
            const ImuAngle a = imu.get_angle();
            const ImuGyro g = imu_get_gyro_corrected();
            const float wz = chassis_yaw_hold_apply(test.speed_mps, test.lateral_mps, 0.0f, a.yaw, g.z, (float)control_dt_ms * 0.001f);
            if(chassis.set_velocity(test.speed_mps, test.lateral_mps, wz) != chassis.OK || chassis.process() != chassis.OK)
                straight_fault(&test, "CHASSIS_CONTROL");
        } else if(!host_paused) {
            chassis_yaw_hold_disable();
        }
        /* WAIT and guard pauses stop outputs while retaining GO's heading. */
        if(!(control_source.mode == DRIVE_SOURCE_MANUAL && control_source.manual_armed) &&
           !host_active && !map_running) {
            manual_velocity = (DriveVelocity){0};
            stop_outputs();
        }
        if(tx_fault || !can_healthy()) {
            if(full_map_active(&full_map)) full_map_stop(&full_map,"CAN_TX_OR_BUS",true);
            straight_fault(&test, "CAN_TX_OR_BUS");
            if(control_source.manual_armed) {
                drive_source_stop_manual(&control_source);
                stop_outputs();
            }
        }
    }
    if(control_source.mode == DRIVE_SOURCE_MANUAL && control_source.manual_armed && now - last_manual_process >= 2u) {
        if(now - last_manual_process > 50u || chassis.process() != chassis.OK) {
            drive_source_stop_manual(&control_source);
            straight_fault(&test, "MANUAL_CONTROL_TIMING_OR_OUTPUT");
            manual_velocity = (DriveVelocity){0};
            stop_outputs();
        }
        last_manual_process = now;
    } else if(!control_source.manual_armed) last_manual_process = now;
    trace_capture(in);
    trace_export(now,in);
    if(now-last_map_log>=500u) {
        last_map_log=now;
        const AutoPose p=full_map.localization.pose.pose;
        log_info("TEST_MAP_STATUS proto=1 phase=%s token=%lu segment=%u total=17 coverage=0x%x origin=%u x=%.3f y=%.3f yaw_deg=%.2f speed=%.2f reason=%s",
            full_map_phase_name(full_map.phase),(unsigned long)full_map.token,full_map.segment,full_map.coverage,
            (unsigned)full_map.localization.registered,(double)p.x,(double)p.y,(double)p.yaw*57.295779513,
            (double)full_map.speed,full_map.reason);
        log_info("TEST_MAP_POSE used=0x%x rejected=0x%x axes=0x%x required=0x%x predicted=0x%x vx=%.3f vy=%.3f motion_error=%u uncorrected_m=%.3f,%.3f",
            full_map.localization.used_mask,full_map.localization.rejected_mask,full_map.localization.constrained_axes,
            full_map.required_mask,full_map.predicted_mask,(double)full_map.command.vx,(double)full_map.command.vy,
            (unsigned)full_map.motion_error,(double)full_map.localization.uncorrected_m[0],
            (double)full_map.localization.uncorrected_m[1]);
    }
    if(now - last_log >= 100u) {
        last_log = now;
        const ImuAngle a = imu.get_angle();
        log_info("TEST_STATUS phase=%u token=%lu healthy=%u stopped=%u target_m=%.2f s=%.3f vx=%.3f cmd=%.3f yaw_deg=%.2f ref_deg=%.2f wz=%.4f blocked=%s result=%s control=%s",
            (unsigned)test.phase, (unsigned long)test.token, (unsigned)in.healthy, (unsigned)in.stationary,
            (double)test.target_m,
            (double)test.distance_m, (double)in.vx_mps, (double)test.speed_mps, (double)a.yaw * 57.295779513,
            (double)chassis_yaw_hold_get_yaw_ref() * 57.295779513, (double)chassis_yaw_hold_get_output_wz(),
            control_source.mode == DRIVE_SOURCE_HOST ? first_blocker() : "HOST_NOT_SELECTED", test.result,
            drive_source_name(control_source.mode));
        /* Dedicated numeric frame; human-readable status is not a FireWater channel. */
        LaserSnapshot laser = {0};
        (void)laser_get_snapshot(&laser);
        const LogVofaValue wave[13] = {
            log_vofa_value_f64((double)a.yaw * 57.295779513),
            log_vofa_value_f64((double)chassis_yaw_hold_get_yaw_ref() * 57.295779513),
            log_vofa_value_f64((double)chassis_yaw_hold_get_yaw_error() * 57.295779513),
            log_vofa_value_f64((double)chassis_yaw_hold_get_output_wz()),
            log_vofa_value_u64(laser.distance_mm[0]),
            log_vofa_value_u64(laser.distance_mm[1]),
            log_vofa_value_u64(laser.distance_mm[2]),
            log_vofa_value_u64(laser.distance_mm[3]),
            log_vofa_value_u64(laser.accepted_mm[0]),
            log_vofa_value_u64(laser.accepted_mm[1]),
            log_vofa_value_u64(laser.accepted_mm[2]),
            log_vofa_value_u64(laser.accepted_mm[3]),
            log_vofa_value_u64(laser.valid_mask),
        };
        (void)log_vofa_write("FW", 13u, wave);
        log_detail(now, in);
    }
    if(now - last_plan_log >= 200u) { last_plan_log = now; log_plan(in); }
}
#endif
