#include "main.h"
#include "tim.h"
#include "vision_motor.h"
#include "work_can.h"
#include "can_test.h"
#include "work_link.h"
#include "work_config.h"
#include "station_work.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

FakeTimer timer;
GPIO_TypeDef portc, portd;
CAN_HandleTypeDef hcan2;
FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
FDCAN_GlobalTypeDef g_fake_can_regs[3];
static uint32_t now_ms, last_camera, last_tx_time;
static unsigned pins_c, pins_d, starts, last_phase;
static int raw_rpm, wheel_enabled, drop_requests, drop_replies, drop_go;
static Can3EndpointCallback request_callback;
static STM32HalCanRxCallback wheel_callback;
static uint8_t reply_queue[32][8];
static unsigned queue_read, queue_write;
static uint8_t laser_pending, laser_data[8];
static CAN_RxHeaderTypeDef laser_header;

uint32_t HAL_GetTick(void) { return now_ms; }
uint32_t __get_PRIMASK(void) { return 0; }
void __disable_irq(void) {}
void __set_PRIMASK(uint32_t key) { (void)key; }
void HAL_GPIO_WritePin(GPIO_TypeDef* port, uint16_t pin, int state) {
    unsigned* pins = port == GPIOC ? &pins_c : &pins_d;
    if(state) *pins |= pin; else *pins &= ~pin;
}

Can3BusStatus can3_bus_register_endpoint(uint16_t id, Can3EndpointCallback cb, void* user) {
    assert(id == WORK_CAN_REQUEST_ID && user == NULL);
    request_callback = cb;
    return CAN3_BUS_OK;
}
BspCanStatus can_register_rx_callback(FDCAN_HandleTypeDef* bus, STM32HalCanRxCallback cb, void* user) {
    assert(bus == &hfdcan2 && user == NULL);
    wheel_callback = cb;
    return STM32_HAL_CAN_OK;
}
Can3BusStatus can3_bus_send(uint16_t id, const uint8_t* data, uint8_t len) {
    assert(id == WORK_CAN_REPLY_ID && len == 8);
    if(drop_replies) return CAN3_BUS_OK;
    if(data[1] == (ELBOW_TEST_MODE == 2 ? STATION_RELEASE : WORK_GO_ACK) && drop_go) { drop_go--; return CAN3_BUS_OK; }
    assert(queue_write - queue_read < 32);
    memcpy(reply_queue[queue_write++ % 32], data, 8);
    return CAN3_BUS_OK;
}
int HAL_CAN_ConfigFilter(CAN_HandleTypeDef* can, CAN_FilterTypeDef* f) {
    (void)can;
    assert((f->FilterBank == 14 && f->FilterIdHigh == WORK_CAN_REPLY_ID << 5) ||
           (f->FilterBank == 15 && f->FilterIdHigh == CAN_LASER_ID << 5));
    assert(f->FilterMaskIdLow == 6 && f->FilterMaskIdHigh == 0x7FFu << 5);
    return HAL_OK;
}
int HAL_CAN_Start(CAN_HandleTypeDef* can) { (void)can; return HAL_OK; }
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef* can) { (void)can; return 0; }
int HAL_CAN_AbortTxRequest(CAN_HandleTypeDef* can, uint32_t box) { (void)can; (void)box; return HAL_OK; }
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef* can, uint32_t box) { (void)can; (void)box; return 0; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef* can) { (void)can; return 3; }
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef* can, uint32_t fifo) {
    (void)can; (void)fifo; return queue_write - queue_read + laser_pending;
}
int HAL_CAN_GetRxMessage(CAN_HandleTypeDef* can, uint32_t fifo, CAN_RxHeaderTypeDef* h, uint8_t* data) {
    (void)can; (void)fifo;
    if (laser_pending) {
        *h = laser_header;
        memcpy(data, laser_data, 8);
        laser_pending = 0;
        return HAL_OK;
    }
    h->IDE = CAN_ID_STD; h->RTR = CAN_RTR_DATA; h->DLC = 8; h->StdId = WORK_CAN_REPLY_ID;
    memcpy(data, reply_queue[queue_read++ % 32], 8);
    return HAL_OK;
}
int HAL_CAN_AddTxMessage(CAN_HandleTypeDef* can, CAN_TxHeaderTypeDef* h, uint8_t* data, uint32_t* box) {
    Can3Frame frame = {0};
    (void)can;
    assert(h->StdId == WORK_CAN_REQUEST_ID && h->DLC == 8);
    *box = 1;
    last_tx_time = now_ms;
    if(drop_requests) return HAL_OK;
    frame.id = WORK_CAN_REQUEST_ID; frame.len = 8; frame.rx_ms = now_ms;
    memcpy(frame.data, data, 8);
    assert(request_callback(&frame, NULL));
    return HAL_OK;
}

static void emit_wheels(void) {
    unsigned i;
    uint8_t data[8] = {0};
    FDCAN_RxHeaderTypeDef header = {0};
    header.IdType = FDCAN_STANDARD_ID; header.RxFrameType = FDCAN_DATA_FRAME;
    header.FDFormat = FDCAN_CLASSIC_CAN; header.DataLength = FDCAN_DLC_BYTES_8;
    data[2] = (uint8_t)((uint16_t)raw_rpm >> 8); data[3] = (uint8_t)raw_rpm;
    for(i = 0; i < 4; i++) {
        header.Identifier = 0x201u + i;
        wheel_callback(&hfdcan2, &header, data, NULL);
    }
}
static void run(unsigned duration, unsigned region, int dx, int camera) {
    unsigned t;
    char report[40];
    for(t = 0; t < duration; t += 10) {
        now_ms += 10;
        if(camera && (uint32_t)(now_ms - last_camera) >= 50) {
            int len = snprintf(report, sizeof(report), "$VIS,%u,%d\r\n", region, dx);
            Vision_Motor_Receive((const uint8_t*)report, (uint16_t)len);
            last_camera = now_ms;
        }
        if(wheel_enabled) emit_wheels();
        CAN_Test_Task();
        Vision_Motor_Task();
        if(Vision_MotorPhase == 1 && last_phase != 1) {
            assert(g_work_link.hold && g_work_link.reply.state == WORK_STOPPED);
            assert(Work_CAN_GetReply(Vision_WorkTask) == WORK_STOPPED);
            starts++;
        }
        last_phase = Vision_MotorPhase;
        CAN_Test_Task();
        work_link_process(now_ms, true);
    }
}
static void reset_at(uint32_t tick) {
    now_ms = tick; last_camera = tick - 100;
    queue_read = queue_write = starts = last_phase = pins_c = pins_d = 0;
    raw_rpm = drop_requests = drop_replies = drop_go = 0;
    wheel_enabled = 1;
    memset(&timer, 0, sizeof(timer));
    assert(work_link_init());
    Vision_Motor_Init();
    CAN_Test_Init();
    run(100, 0, 9999, 1);
    assert(g_work_link.paired && Vision_WorkState == VISION_WORK_IDLE);
}
static void approach(void) {
    float vx = 1.0f, vy = 1.0f, wz = 2.0f;
    run(250, 1, 80, 1);
    assert(Vision_WorkState == VISION_WORK_APPROACH && g_work_link.active);
    work_link_limit_velocity(&vx, &vy, &wz);
    assert(fabsf(sqrtf(vx*vx + vy*vy) - WORK_APPROACH_SPEED_MPS) < 0.0001f && wz == 0);
    run(100, 1, 20, 1);
    assert(Vision_WorkState == VISION_WORK_WAIT_STOP);
    assert(g_work_link.hold && starts == 0);
}
static void start_action(void) {
    unsigned i;
    approach();
    for(i = 0; i < 100 && starts == 0; i++) run(10, 1, 20, 1);
    assert(starts == 1 && Vision_MotorRegion == 2);
    assert(timer.CCR2 == VISION_MOTOR_PWM && timer.CCR1 == 0);
    assert((pins_c & GPIO_PIN_2) && !(pins_c & GPIO_PIN_3));
}
static void inject_request(uint8_t state, uint32_t task) {
    Can3Frame frame = {0};
    WorkCanPacket packet = {0};
    packet.state = state; packet.task = task;
    frame.id = WORK_CAN_REQUEST_ID; frame.len = 8; frame.rx_ms = now_ms;
    work_can_encode(frame.data, packet);
    assert(request_callback(&frame, NULL));
    work_link_process(now_ms, true);
}

static void test_full_tree(void) {
    unsigned i;
    uint32_t first_task;
    reset_at(0);
    start_action();
    first_task = Vision_WorkTask;
    run(VISION_CW_MS + VISION_REVERSE_PAUSE_MS + 30, 2, 20, 1);
    assert(Vision_MotorRegion == 2 && Vision_MotorPhase == 3);
    assert(!(pins_c & GPIO_PIN_2) && (pins_c & GPIO_PIN_3));
    for(i = 0; i < 250 && starts < 2; i++) run(10, 2, 20, 1);
    assert(starts == 2 && Vision_MotorRegion == 4);
    assert(timer.CCR4 == VISION_MOTOR_PWM && timer.CCR2 == 0);
    assert(!(pins_d & GPIO_PIN_3) && (pins_d & GPIO_PIN_5));
    for(i = 0; i < 250 && Vision_WorkState != VISION_WORK_CHECK; i++) run(10, 0, 20, 1);
    assert(Vision_WorkState == VISION_WORK_CHECK);
    run(400, 0, 20, 1);
    assert(g_work_link.hold && Vision_WorkState == VISION_WORK_CHECK);
    drop_go = 2;
    for(i = 0; i < 150 && Vision_WorkState != VISION_WORK_LEAVING; i++) run(10, 0, 20, 1);
    assert(Vision_WorkState == VISION_WORK_LEAVING && !g_work_link.hold);
    assert(starts == 2 && Vision_WorkResult == WORK_RESULT_OK);
    inject_request(WORK_WAIT, first_task);
    assert(!g_work_link.hold && g_work_link.reply.state == WORK_GO_ACK);
    run(500, 1, 20, 1);
    assert(Vision_WorkState == VISION_WORK_LEAVING && starts == 2);
    run(800, 0, 9999, 1);
    assert(Vision_WorkState == VISION_WORK_LEAVING); /* No movement, no rearm. */
    raw_rpm = 100;
    run(1000, 0, 9999, 1);
    assert(Vision_WorkState == VISION_WORK_IDLE);
    raw_rpm = 0;
    run(250, 1, 80, 1);
    assert(Vision_WorkTask == first_task + 1 && Vision_WorkState == VISION_WORK_APPROACH);
    puts("WORK_FULL_TREE_AND_REARM_OK");
}
static void test_faults(void) {
    reset_at(0); approach(); wheel_enabled = 0;
    run(150, 1, 20, 1);
    assert(g_work_link.hold && g_work_link.reply.state == WORK_FAULT && starts == 0);
    reset_at(0); start_action();
    now_ms += 150; /* GO cannot bypass expired wheel feedback. */
    inject_request(WORK_GO, Vision_WorkTask);
    assert(g_work_link.hold && g_work_link.reply.state == WORK_FAULT);
    reset_at(0); start_action(); drop_requests = drop_replies = 1;
    run(600, 2, 20, 1);
    assert(Vision_WorkState == VISION_WORK_FAULT && g_work_link.hold);
    run(2200, 2, 20, 1);
    assert(Vision_MotorPhase == 0 && starts == 1 && !timer.CCR2);
    reset_at(0); approach();
    run(200, 1, 20, 1);
    assert(Vision_WorkState == VISION_WORK_CHECK);
    run(2500, 0, 9999, 0);
    assert(Vision_WorkState == VISION_WORK_FAULT && g_work_link.hold);
    reset_at(0); start_action();
    CAN_Test_Init(); Vision_Motor_Init();
    run(200, 0, 9999, 1);
    assert(g_work_link.reply.state == WORK_FAULT && g_work_link.hold);
    reset_at(0); start_action();
    assert(work_link_init());
    run(700, 1, 20, 1);
    assert(Vision_WorkState == VISION_WORK_FAULT && g_work_link.hold);
    puts("WORK_LINK_FEEDBACK_VISION_RESET_FAULTS_OK");
}
static void test_limits(void) {
    unsigned i;
    reset_at(0); start_action();
    for(i = 0; i < 750 && Vision_WorkState != VISION_WORK_LEAVING; i++) run(10, 1, 20, 1);
    assert(starts == 2 && Vision_WorkResult == WORK_RESULT_ATTEMPTS);
    assert(Vision_WorkState == VISION_WORK_LEAVING);
    reset_at(0);
    run(300, 0, 80, 1); run(300, 0, -130, 1);
    assert(starts == 0 && Vision_WorkResult == WORK_RESULT_SKIPPED && !g_work_link.hold);
    reset_at(0); approach();
    raw_rpm = 100; run(1800, 1, 20, 1);
    assert(starts == 0 && g_work_link.hold);
    run(300, 1, 20, 1);
    assert(g_work_link.reply.state == WORK_FAULT);
    reset_at(UINT32_MAX - 500);
    start_action(); run(2500, 0, 20, 1); run(1000, 0, 20, 1);
    assert(Vision_WorkState == VISION_WORK_LEAVING && starts == 1);
    puts("WORK_RETRY_SKIP_STOP_TIMEOUT_WRAP_OK");
}
static void test_wait_without_slow(void) {
    reset_at(0);
    inject_request(WORK_WAIT, 1);
    assert(g_work_link.hold && g_work_link.reply.state == WORK_STOPPING);
    puts("WORK_WAIT_WITH_LOST_SLOW_OK");
}
static void test_parser(void) {
    uint32_t count;
    reset_at(0);
    count = Vision_FrameCount;
    Vision_Motor_Receive((const uint8_t*)"$VIS,5,0\r\n", 10);
    Vision_Motor_Receive((const uint8_t*)"$VIS,1,3000\r\n", 13);
    assert(Vision_FrameCount == count && Vision_BadFrameCount == 2);
    Vision_Motor_Receive((const uint8_t*)"noise$VIS,3,", 12);
    Vision_Motor_Receive((const uint8_t*)"-12\r\n", 5);
    Vision_Motor_Task();
    assert(Vision_Region == 3 && Vision_TreeDX == -12 && starts == 0);
    assert(last_tx_time <= now_ms);
    puts("WORK_VISION_PARSER_OK");
}
static void test_laser_display_input(void) {
    uint16_t mm = 0;
    uint32_t rx_count;
    const uint8_t sample[8] = {0, 0, 0x3a, 0x98, 0x3a, 0x99, 0xff, 0xff};
    reset_at(UINT32_MAX - 200u);
    assert(CAN_Laser_Read(0, &mm) == 0);
    memcpy(laser_data, sample, 8);
    laser_header.IDE = CAN_ID_STD;
    laser_header.RTR = CAN_RTR_DATA;
    laser_header.DLC = 7;
    laser_header.StdId = CAN_LASER_ID;
    laser_pending = 1;
    CAN_Test_Task();
    assert(CAN_Laser_Read(0, &mm) == 0);
    laser_header.DLC = 8;
    laser_header.IDE = 1; /* Extended frames are rejected. */
    laser_pending = 1;
    CAN_Test_Task();
    assert(CAN_Laser_Read(0, &mm) == 0);
    laser_header.IDE = CAN_ID_STD;
    laser_header.RTR = 1; /* Remote frames are rejected. */
    laser_pending = 1;
    CAN_Test_Task();
    assert(CAN_Laser_Read(0, &mm) == 0);
    laser_header.RTR = CAN_RTR_DATA;
    rx_count = CAN_Test_RxCount;
    laser_pending = 1;
    CAN_Test_Task();
    assert(CAN_Test_RxCount == rx_count); /* Laser cannot refresh chassis heartbeat. */
    assert(CAN_Laser_Read(0, &mm) == 1 && mm == 0);
    assert(CAN_Laser_Read(1, &mm) == 1 && mm == 15000);
    assert(CAN_Laser_Read(2, &mm) == 2 && mm == 15001);
    assert(CAN_Laser_Read(3, &mm) == 2 && mm == 65535);
    assert(CAN_Laser_Read(4, &mm) == 0 && CAN_Laser_Read(0, NULL) == 0);
    now_ms += CAN_LASER_TIMEOUT_MS;
    assert(CAN_Laser_Read(0, &mm) == 1);
    now_ms++;
    assert(CAN_Laser_Read(0, &mm) == 3);
    laser_pending = 1;
    CAN_Test_Task();
    assert(CAN_Laser_Read(1, &mm) == 1 && mm == 15000);
    CAN_Test_Init();
    assert(CAN_Laser_Read(0, &mm) == 0);
    puts("LASER_INPUT_ENDIAN_RANGE_TIMEOUT_WRAP_RECOVERY_OK");
}
static void run_standalone(uint32_t duration, uint8_t region, int dx, int camera) {
    uint32_t start = now_ms;
    while ((uint32_t)(now_ms - start) < duration) {
        char frame[32];
        now_ms += 10;
        if (camera && (uint32_t)(now_ms - last_camera) >= 100) {
            int size = snprintf(frame, sizeof(frame), "$VIS,%u,%d\r\n", region, dx);
            Vision_Motor_Receive((const uint8_t*)frame, (uint16_t)size);
            last_camera = now_ms;
        }
        Vision_Motor_Task();
        if (Vision_MotorPhase == 1 && last_phase != 1) starts++;
        last_phase = Vision_MotorPhase;
        CAN_Test_Task();
        assert(CAN_Test_TxCount == 0); /* Standalone never commands the chassis. */
    }
}

static void test_standalone(void) {
    now_ms = UINT32_MAX - 100;
    last_camera = now_ms - 100;
    starts = last_phase = queue_read = queue_write = 0;
    memset(&timer, 0, sizeof(timer));
    Vision_Motor_Init();
    CAN_Test_Init();
    assert(Vision_WorkState == VISION_WORK_TEST);
    run_standalone(1000, 0, 9999, 1);
    assert(starts == 0);
    run_standalone(200, 1, 9999, 1);
    assert(starts == 0);
    run_standalone(150, 1, 9999, 1);
    assert(starts == 1 && Vision_MotorRegion == 2); /* No tree/CAN required. */
    run_standalone(3000, 1, 9999, 1);
    assert(starts == 1 && Vision_MotorPhase == 0); /* Held target fires once. */
    run_standalone(1000, 1, 9999, 0);
    run_standalone(1000, 1, 9999, 1);
    assert(starts == 1); /* Reconnect alone cannot rearm the same target. */
    run_standalone(300, 0, 9999, 1);
    run_standalone(500, 1, 9999, 1);
    assert(starts == 1); /* Brief empty detection cannot rearm. */
    run_standalone(700, 0, 9999, 1);
    run_standalone(350, 1, 9999, 1);
    assert(starts == 2 && Vision_MotorRegion == 2);
    run_standalone(3000, 1, 9999, 1);
    run_standalone(350, 2, 1200, 1);
    assert(starts == 3 && Vision_MotorRegion == 4); /* Different region, any dx. */
    run_standalone(2000, 2, 1200, 0);
    assert(Vision_MotorPhase == 0 && !Vision_Online);
    run_standalone(700, 0, 9999, 1);
    run_standalone(350, 3, 0, 1);
    assert(starts == 4 && Vision_MotorRegion == 3);
    run_standalone(3000, 3, 0, 1);
    run_standalone(350, 4, 0, 1);
    assert(starts == 5 && Vision_MotorRegion == 1);
    assert(CAN_Test_RxCount == 0 && Vision_WorkTask == starts);
    puts("STANDALONE_STABILITY_MAPPING_LATCH_REARM_TIMEOUT_WRAP_OK");
}

static void run_station(unsigned duration, unsigned region, int dx, int camera,
                        int healthy, int stationary) {
    unsigned elapsed;
    for(elapsed = 0; elapsed < duration; elapsed += 10) {
        char report[40];
        now_ms += 10;
        if(camera && (uint32_t)(now_ms - last_camera) >= 50u) {
            int size = snprintf(report, sizeof(report), "$VIS,%u,%d\r\n", region, dx);
            Vision_Motor_Receive((const uint8_t*)report, (uint16_t)size);
            last_camera = now_ms;
        }
        CAN_Test_Task();
        Vision_Motor_Task();
        if(Vision_MotorPhase == 1 && last_phase != 1) {
            assert(g_station_work.hold);
            assert(g_station_work.phase == STATION_LINK_WORKING);
            assert(healthy && stationary);
            starts++;
        }
        last_phase = Vision_MotorPhase;
        CAN_Test_Task();
        station_work_process(now_ms, healthy != 0, stationary != 0);
    }
}
static void reset_station(uint32_t tick) {
    now_ms = tick;
    last_camera = tick - 100u;
    queue_read = queue_write = starts = last_phase = 0u;
    pins_c = pins_d = drop_requests = drop_replies = drop_go = 0;
    memset(&timer, 0, sizeof(timer));
    Vision_Motor_Init();
    CAN_Test_Init();
    assert(work_link_init());
    assert(station_work_init());
    run_station(150, 0, 9999, 1, 1, 1);
    assert(station_work_ready());
}
static void request_station(unsigned station, unsigned region) {
    assert(station_work_begin((uint8_t)station, now_ms));
    run_station(300, region, 80, 1, 1, 0);
    assert(g_station_work.slow_requested && !g_station_work.hold);
    run_station(300, region, 20, 1, 1, 0);
    assert(g_station_work.stop_requested && Vision_MotorPhase == 0);
    assert(!station_work_start(now_ms, false));
    run_station(150, region, 20, 1, 1, 1);
    assert(Vision_MotorPhase == 0);
    assert(station_work_start(now_ms, true));
}
static void test_station_sequence(void) {
    unsigned station;
    reset_station(UINT32_MAX - 300u);
    for(station = 1; station <= 12; station++) {
        unsigned before = starts;
        unsigned region = (station - 1u) % 4u + 1u;
        request_station(station, region);
        run_station(600, region, 20, 1, 1, 1);
        assert(starts == before + 1u && g_station_work.hold);
        drop_go = 2;
        run_station(3000, 0, 9999, 1, 1, 1);
        assert(g_station_work.phase == STATION_LINK_DONE && !g_station_work.hold);
        assert(g_station_work.station == station && Vision_MotorPhase == 0);
        run_station(400, region, 20, 1, 1, 1);
        assert(starts == before + 1u);
    }
    assert(starts == 12u);
    puts("STATION_12_TREES_SLOW_WAIT_FIXED_STOP_RETURN_RELEASE_WRAP_OK");
}
static void test_station_faults(void) {
    unsigned before;
    reset_station(0);
    request_station(1, 1);
    run_station(600, 1, 20, 1, 1, 1);
    before = starts;
    drop_requests = 1;
    run_station(600, 1, 20, 1, 1, 1);
    assert(g_station_work.phase == STATION_LINK_FAULT && g_station_work.hold);
    assert(!station_work_reset(true));
    run_station(1800, 1, 20, 1, 1, 1);
    assert(Vision_MotorPhase == 0 && starts == before);
    drop_requests = 0;
    run_station(150, 1, 20, 1, 1, 1);
    assert(g_station_work.returned && station_work_reset(true));
    run_station(150, 0, 9999, 1, 1, 1);
    assert(station_work_ready());
    request_station(2, 2);
    run_station(600, 2, 20, 1, 1, 1);
    run_station(600, 2, 20, 0, 1, 1);
    assert(g_station_work.phase == STATION_LINK_FAULT && g_station_work.hold);
    reset_station(0);
    request_station(3, 3);
    run_station(500, 3, 20, 1, 1, 1);
    Vision_Motor_Init();
    CAN_Test_Init();
    run_station(150, 0, 9999, 1, 1, 1);
    assert(g_station_work.phase == STATION_LINK_FAULT && g_station_work.hold);
    reset_station(0);
    request_station(4, 4);
    run_station(500, 4, 20, 1, 1, 1);
    run_station(100, 4, 20, 1, 0, 1);
    assert(g_station_work.phase == STATION_LINK_FAULT);
    puts("STATION_CAN_VISION_RESTART_FEEDBACK_ABORT_RETURN_RESET_OK");
}
static void test_station_replay_and_release_gate(void) {
    Can3Frame frame = {0};
    WorkCanPacket packet;
    uint32_t rejected;
    reset_station(0);
    request_station(1, 1);
    run_station(500, 1, 20, 1, 1, 1);
    frame.len = 8;
    frame.rx_ms = now_ms;
    packet = (WorkCanPacket){g_station_work.task+1u, STATION_DONE, 1, WORK_RESULT_OK};
    station_can_encode(frame.data, packet);
    assert(request_callback(&frame, NULL));
    rejected = g_station_work.rejected_count;
    station_work_process(now_ms, true, true);
    assert(g_station_work.rejected_count == rejected+1u && g_station_work.hold);
    packet.task = g_station_work.task;
    packet.region = 2;
    station_can_encode(frame.data, packet);
    assert(request_callback(&frame, NULL));
    station_work_process(now_ms, true, true);
    assert(g_station_work.phase == STATION_LINK_WORKING && g_station_work.hold);
    packet.region = 1;
    packet.state = STATION_RELEASED;
    station_can_encode(frame.data, packet);
    assert(request_callback(&frame, NULL));
    station_work_process(now_ms, true, true);
    assert(g_station_work.phase == STATION_LINK_WORKING && g_station_work.hold);
    g_station_work.phase = STATION_LINK_RELEASE;
    assert(request_callback(&frame, NULL));
    station_work_process(now_ms, true, false);
    assert(g_station_work.phase == STATION_LINK_FAULT && g_station_work.hold);
    assert(g_station_work.result == WORK_RESULT_FEEDBACK);
    reset_station(0);
    request_station(1, 1);
    run_station(500, 1, 20, 1, 1, 1);
    assert(station_work_init()); /* Chassis reboot ends the old route. */
    run_station(200, 0, 9999, 1, 1, 1);
    assert(!station_work_ready());
    run_station(1800, 0, 9999, 1, 1, 1);
    assert(station_work_ready() && Vision_MotorPhase == 0 && starts == 1);
    puts("STATION_STALE_TASK_WRONG_TREE_EARLY_RELEASE_STOP_GATE_AND_CHASSIS_RESET_OK");
}

static void test_station_laser_without_camera(void) {
    uint16_t distance;
    unsigned channel, iteration;
    reset_station(UINT32_MAX - 300u);
    run_station(600, 0, 9999, 0, 1, 1);
    assert(!Vision_Online && !station_work_ready());
    assert(!station_work_begin(1, now_ms));
    laser_header = (CAN_RxHeaderTypeDef){0};
    laser_header.StdId = CAN_LASER_ID;
    laser_header.IDE = CAN_ID_STD;
    laser_header.RTR = CAN_RTR_DATA;
    laser_header.DLC = 8;
    for(iteration=0; iteration<20; iteration++) {
        for(channel=0; channel<4; channel++) {
            unsigned sample=1000u + iteration*10u + channel;
            laser_data[channel*2u]=(uint8_t)(sample >> 8);
            laser_data[channel*2u+1u]=(uint8_t)sample;
        }
        laser_pending=1;
        run_station(100, 0, 9999, 0, 1, 1);
        for(channel=0; channel<4; channel++) {
            assert(CAN_Laser_Read((uint8_t)channel, &distance)==1);
            assert(distance==1000u+iteration*10u+channel);
        }
        assert(!station_work_ready() && Vision_MotorPhase==0);
    }
    run_station(400, 0, 9999, 0, 1, 1);
    assert(CAN_Laser_Read(0, &distance)==3);
    laser_pending=1;
    run_station(100, 0, 9999, 0, 1, 1);
    assert(CAN_Laser_Read(0, &distance)==1);
    run_station(200, 0, 9999, 1, 1, 1);
    assert(station_work_ready() && starts==0);
    puts("STATION_CAMERA_OFF_LASER_REFRESH_TIMEOUT_RECOVERY_NO_FALSE_READY_OK");
}

static void test_station_empty_and_limits(void) {
    reset_station(0);
    request_station(1, 0);
    run_station(1500, 0, 9999, 1, 1, 1);
    assert(g_station_work.phase == STATION_LINK_DONE && starts == 0);
    reset_station(0);
    request_station(2, 1);
    run_station(6000, 1, 20, 1, 1, 1);
    assert(starts == 2u && g_station_work.phase == STATION_LINK_FAULT);
    assert(Vision_WorkResult == WORK_RESULT_ATTEMPTS && g_station_work.hold);
    puts("STATION_NO_TARGET_AND_ATTEMPTS_NEVER_REPORT_FALSE_SUCCESS_OK");
}

int main(void) {
    setbuf(stdout, NULL);
    if (ELBOW_TEST_MODE == 2) {
        test_station_sequence();
        test_station_faults();
        test_station_replay_and_release_gate();
        test_station_empty_and_limits();
        test_station_laser_without_camera();
    } else if (ELBOW_TEST_MODE == 1) {
        test_standalone();
    } else {
        test_parser(); test_full_tree(); test_faults(); test_limits(); test_wait_without_slow();
        test_laser_display_input();
    }
    puts("WORK_CAN_INTEROP_TEST_OK");
    return 0;
}
