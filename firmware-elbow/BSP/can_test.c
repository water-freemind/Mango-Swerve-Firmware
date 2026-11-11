/* CAN2 正常模式收发，业务包含两类帧。
 * 0x610/0x611 用于肘击与底盘协同，0x601 用于读取同总线激光距离。
 * 主循环先收 CAN，再处理视觉/作业逻辑，再发 CAN，最后刷新 OLED。
 * 本文件沿用 can_test 名称，当前执行的是正常业务通信。
 */
#include "can_test.h"
#include "can.h"
#include "work_can.h"
#include "vision_motor.h"

volatile uint32_t CAN_Test_TxCount, CAN_Test_RxCount, CAN_Test_ErrorCount;
volatile uint32_t CAN_Test_LastHalError;
volatile uint8_t CAN_Test_State;
static WorkCanPacket request, reply; /* request 是待发送请求，reply 是最近有效底盘应答。 */
/* ready 表示 CAN 启动成功，have_reply 表示收到过底盘应答。
 * dirty 表示请求已变化，tx_pending 表示硬件发送邮箱内还有未完成的帧。
 */
static uint8_t ready, have_reply, dirty, tx_pending;
static uint32_t last_tx, last_rx, mailbox, pending_since;
static uint16_t laser_mm[4]; /* 左、右、后、前的原始毫米值，主循环收帧后更新。 */
static uint32_t laser_rx_ms;
static uint8_t laser_received;

/* 保存最近错误，供 OLED 的 C 和串口诊断查看。 */
static void record_error(uint8_t state)
{
    CAN_Test_State = state;
    CAN_Test_ErrorCount++;
    CAN_Test_LastHalError = HAL_CAN_GetError(&hcan2);
}

void CAN_Test_Init(void)
{
    CAN_FilterTypeDef filter = {0};
    ready = have_reply = tx_pending = 0;
    dirty = 1;
    last_tx = last_rx = 0;
    CAN_Test_TxCount = CAN_Test_RxCount = CAN_Test_ErrorCount = 0;
    CAN_Test_LastHalError = 0;
    CAN_Test_State = 0;
    laser_received = 0;
    request.state = ELBOW_TEST_MODE == 2 ? STATION_HELLO : WORK_HELLO;
    request.task = 0;
    request.region = request.result = 0;
    filter.FilterBank = 14;
    /* CAN1/CAN2 共用过滤器组，从 14 开始分给 CAN2。
     * 第 14 组精确接收底盘 0x611，第 15 组精确接收激光 0x601。
     */
    filter.SlaveStartFilterBank = 14;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.FilterIdHigh = WORK_CAN_REPLY_ID << 5;
    filter.FilterMaskIdHigh = 0x7FFu << 5;
    filter.FilterMaskIdLow = 6u; /* 同时检查 IDE/RTR 位，只接收标准数据帧。 */
    if (HAL_CAN_ConfigFilter(&hcan2, &filter) != HAL_OK)
    {
        record_error(2);
        return;
    }
    filter.FilterBank = 15;
    filter.FilterIdHigh = CAN_LASER_ID << 5;
    if (HAL_CAN_ConfigFilter(&hcan2, &filter) != HAL_OK ||
        HAL_CAN_Start(&hcan2) != HAL_OK)
    {
        record_error(2);
        return;
    }
    ready = 1;
}

void Work_CAN_SetRequest(uint8_t state, uint32_t task, uint8_t region, uint8_t result)
{
    if (request.state == state && request.task == task &&
        request.region == region && request.result == result) return;
    request.state = state;
    request.task = task;
    request.region = region;
    request.result = result;
    dirty = 1;
    /* 请求状态变化时取消尚未发完的旧状态，避免旧 WAIT/GO 在新状态后到达。 */
    if (tx_pending)
    {
        (void)HAL_CAN_AbortTxRequest(&hcan2, mailbox);
        tx_pending = 0;
    }
}

uint8_t Work_CAN_Online(void)
{
    return ready && have_reply &&
        (uint32_t)(HAL_GetTick() - last_rx) < WORK_CAN_TIMEOUT_MS;
}

uint8_t Work_CAN_GetReply(uint32_t task)
{
    /* 正常应答必须属于当前树任务；底盘故障即使任务编号不同也需要处理。 */
    if (!Work_CAN_Online()) return 0;
    if (reply.state == WORK_FAULT) return WORK_FAULT;
    return reply.task == task ? reply.state : 0;
}

uint8_t Work_CAN_Result(void) { return reply.result; }
uint8_t Work_CAN_StationCommand(WorkCanPacket *command)
{
    if (!command || !Work_CAN_Online()) return 0;
    *command = reply;
    return 1;
}
uint8_t Work_CAN_Moving(void)
{
    return Work_CAN_Online() && (reply.region & WORK_CAN_MOVING_FLAG) != 0;
}

uint8_t CAN_Laser_Read(uint8_t channel, uint16_t *distance_mm)
{
    /* 超时先返回状态，避免屏幕把上一次距离继续当作实时读数。 */
    if (channel >= 4 || distance_mm == 0 || !laser_received) return 0;
    if ((uint32_t)(HAL_GetTick() - laser_rx_ms) > CAN_LASER_TIMEOUT_MS) return 3;
    *distance_mm = laser_mm[channel];
    return *distance_mm <= CAN_LASER_MAX_MM ? 1 : 2;
}

void CAN_Test_Task(void)
{
    CAN_RxHeaderTypeDef rx;
    CAN_TxHeaderTypeDef tx = {0};
    WorkCanPacket packet;
    uint8_t data[8], count = 0, i;
    uint32_t now = HAL_GetTick();
    if (!ready) return;
    /* 每轮最多处理 8 帧，给视觉解析、动作计时和显示刷新留出执行机会。 */
    while (count++ < 8 && HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0))
    {
        if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx, data) != HAL_OK)
        {
            record_error(4);
            break;
        }
        if (rx.IDE != CAN_ID_STD || rx.RTR != CAN_RTR_DATA || rx.DLC != 8)
            continue;
        if (rx.StdId == CAN_LASER_ID)
        {
            /* 激光采用大端：高字节在前，低字节在后。
             * 四路保存后直接跳过底盘应答处理，所以激光在线不等于底盘在线。
             */
            for (i = 0; i < 4; i++)
                laser_mm[i] = (uint16_t)(((uint16_t)data[2 * i] << 8) | data[2 * i + 1]);
            laser_rx_ms = now;
            laser_received = 1;
            continue;
        }
        if (rx.StdId != WORK_CAN_REPLY_ID) continue;
#if ELBOW_TEST_MODE == 2
        {
            if (!station_can_decode(data, &packet) || station_can_is_reply(packet.state))
                continue;
            if (packet.state == STATION_SYNC)
            {
                if (packet.task || packet.region) continue;
            }
            else
            {
                if (!packet.task || !packet.region) continue;
                if (packet.state == STATION_APPROACH)
                {
                    if (packet.task < request.task ||
                        (packet.task == request.task && packet.region != request.region)) continue;
                    if (packet.task > request.task && request.state != STATION_READY &&
                        request.state != STATION_RELEASED && request.state != STATION_HELLO) continue;
                }
                else if (packet.task != request.task || packet.region != request.region) continue;
            }
            reply = packet;
            have_reply = 1;
            last_rx = now;
            CAN_Test_RxCount++;
            CAN_Test_State = 1;
            continue;
        }
#else
        if (!work_can_decode(data, &packet))
            continue;
        /* 过滤过期任务和不属于底盘应答方向的状态，再更新时间戳。 */
        if (packet.task != request.task && packet.state != WORK_FAULT) continue;
        if (packet.state != WORK_SYNC && packet.state != WORK_READY &&
            packet.state != WORK_SLOW_ACK && packet.state != WORK_STOPPING &&
            packet.state != WORK_STOPPED && packet.state != WORK_GO_ACK &&
            packet.state != WORK_FAULT) continue;
        reply = packet;
        have_reply = 1;
        last_rx = now;
        CAN_Test_RxCount++;
        CAN_Test_State = 1;
#endif
    }
    if (have_reply && !Work_CAN_Online()) CAN_Test_State = 5;
    /* 邮箱中的帧超过 100 ms 仍未发完，取消并记录发送失败，后续继续重试。 */
    if (tx_pending)
    {
        if (!HAL_CAN_IsTxMessagePending(&hcan2, mailbox)) tx_pending = 0;
        else if ((uint32_t)(now - pending_since) >= WORK_CAN_PERIOD_MS)
        {
            (void)HAL_CAN_AbortTxRequest(&hcan2, mailbox);
            tx_pending = 0;
            record_error(3);
        }
    }
    /* 状态变化后立即安排发送；状态未变时每 100 ms 重发，作为业务心跳。
     * 邮箱未空时返回主循环，不在这里阻塞等待。
     */
    /* 独立测试保留激光/状态接收，收帧后返回，不向底盘发布作业请求。 */
    if (ELBOW_TEST_MODE != 1)
    {
        if (tx_pending || (!dirty && (uint32_t)(now - last_tx) < WORK_CAN_PERIOD_MS)) return;
        if (!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)) return;
        if (ELBOW_TEST_MODE == 2) station_can_encode(data, request);
        else work_can_encode(data, request);
        tx.StdId = WORK_CAN_REQUEST_ID;
        tx.IDE = CAN_ID_STD;
        tx.RTR = CAN_RTR_DATA;
        tx.DLC = 8;
        tx.TransmitGlobalTime = DISABLE;
        if (HAL_CAN_AddTxMessage(&hcan2, &tx, data, &mailbox) != HAL_OK)
        {
            record_error(3);
            return;
        }
        CAN_Test_TxCount++;
        last_tx = pending_since = now;
        tx_pending = 1;
        dirty = 0;
    }
}
