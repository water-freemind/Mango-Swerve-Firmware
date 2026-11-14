/* 肘击触发流程的主文件，建议先阅读 Work_Control_Task() 中的 switch。
 * 主循环通过 Vision_Motor_Task() 反复调用这里，每次只推进当前阶段。
 *
 * ELBOW_TEST_MODE=1 时使用 process_test() 独立测试视觉与肘击。
 * ELBOW_TEST_MODE=0 时使用下面的正常协同流程，与屏幕 W 显示对应。
 * BOOT 握手 → IDLE 等树 → APPROACH 减速接近 → WAIT_STOP 等底盘停稳
 * → CHECK 新图像复检 → ACTING 执行一个区域 → CHECK 再次复检
 * → RELEASE 请求放行 → LEAVING 等当前树离开 → IDLE 等下一棵树。
 * 任何锁存故障进入 FAULT，需要复位控制器后重新握手。
 *
 * 判断依据是视觉区域、树偏移 dx 和底盘 CAN 应答。
 * 激光数据在 can_test.c 接收，当前用于 OLED 显示。
 * 修改区间、等待时间和动作次数，请查看 vision_motor.h。
 */
#include "work_control.h"
#include "work_can.h"
#include "vision_motor.h"
#include <string.h>

volatile uint8_t Vision_WorkState, Vision_WorkResult; /* 当前作业阶段、结果码，供 OLED/串口读取。 */
volatile uint32_t Vision_WorkTask; /* 每遇到一棵新树加 1，用于匹配本次底盘应答。 */
/* syncing 握手已开始，paired 握手已完成；attempts[1..4] 记录本棵树各区域次数。 */
static uint8_t syncing, paired, attempts[5], stable_region, stable_frames;
/* stable_region/stable_frames 记录同一区域的连续视觉结果。
 * empty_frames 统计无目标新帧，departed_frames 统计越过离树位置的新帧。
 * 各 timing 标志表示对应计时已经开始，避免时间戳为 0 时含义不明确。
 */
static uint8_t empty_frames, departed_frames, moving_timing, missing_timing;
static uint8_t empty_timing, saw_movement;
static uint32_t state_since, tree_since, stable_since, empty_since, frame_floor;
static uint32_t check_since, check_delay, moving_since, missing_since, last_frame;
static uint8_t test_latched_region; /* 独立测试中已动作的区域，避免同一目标连续打击。 */
static uint8_t station_number;
static uint8_t alignment_centered, alignment_reply;

/* 切换作业阶段，同时清除不能跨阶段累计的无目标/离树计数。 */
static void change_state(uint8_t state, uint32_t now)
{
    Vision_WorkState = state;
    state_since = now;
    empty_frames = empty_timing = departed_frames = 0;
}

/* 锁存故障原因。当前电机动作仍由 vision_motor.c 完成定时返回。 */
static void fail(uint8_t reason, uint32_t now)
{
    Vision_WorkResult = reason;
    change_state(VISION_WORK_FAULT, now);
}

/* 把肘击端内部阶段转换成底盘认识的 CAN 请求。
 * CHECK/ACTING 持续发布 WORKING，让底盘保持停车。
 * RELEASE/LEAVING 持续发布 GO，丢帧后还会周期重发。
 */
static void publish(void)
{
    uint8_t state;
    switch (Vision_WorkState)
    {
    case VISION_WORK_BOOT: state = syncing && Vision_Online ? WORK_READY : WORK_HELLO; break;
    case VISION_WORK_IDLE: state = WORK_READY; break;
    case VISION_WORK_APPROACH: state = WORK_SLOW; break;
    case VISION_WORK_WAIT_STOP: state = WORK_WAIT; break;
    case VISION_WORK_CHECK:
    case VISION_WORK_ACTING: state = WORK_WORKING; break;
    case VISION_WORK_RELEASE:
    case VISION_WORK_LEAVING: state = WORK_GO; break;
    default: state = WORK_FAULT; break;
    }
    Work_CAN_SetRequest(state, Vision_WorkTask, Vision_Region, Vision_WorkResult);
}

/* 上电清空任务和历史次数，测试模式进入 TEST，协同模式从 BOOT 开始握手。 */
void Work_Control_Init(void)
{
    Vision_WorkState = ELBOW_TEST_MODE == 1 ? VISION_WORK_TEST : VISION_WORK_BOOT;
    Vision_WorkResult = WORK_RESULT_OK;
    Vision_WorkTask = 0;
    syncing = paired = stable_region = stable_frames = 0;
    empty_frames = empty_timing = departed_frames = 0;
    moving_timing = missing_timing = saw_movement = 0;
    state_since = tree_since = stable_since = empty_since = frame_floor = 0;
    check_since = check_delay = moving_since = missing_since = last_frame = 0;
    test_latched_region = 0;
    station_number = 0;
    alignment_centered = 0;
    alignment_reply = STATION_TRACKING;
    memset(attempts, 0, sizeof(attempts));
}

/* 开始停稳后/返回后的图像复检。
 * 首次停稳后等待 200 ms；返回后从返回计时结束处等待至少 300 ms。
 * frame_floor 记录等待期内最后看到的帧编号，后续必须有更新的帧。
 * 这些等待用于避开传输中的旧图像，需按实际视觉延迟标定。
 */
static void begin_check(uint32_t now, uint32_t frame_count, uint8_t returned)
{
    change_state(VISION_WORK_CHECK, now);
    frame_floor = frame_count;
    check_since = now;
    check_delay = WORK_FRESH_VISION_MS;
    if (returned)
    {
        check_since = Vision_Motor_ReturnedAt();
        if (WORK_SETTLE_MS > check_delay) check_delay = WORK_SETTLE_MS;
    }
    stable_region = stable_frames = 0;
    missing_timing = 0;
}

/* 只有新视觉帧才能累计稳定次数，同一个旧帧被主循环读取多次不会重复计数。
 * 树超出减速区、视觉失效或区域变为 0 时清除稳定计数。
 * 区域变化后重新计时，开始动作前要求至少 3 帧且持续至少 200 ms。
 */
static void update_target(uint32_t now, uint8_t new_frame, uint8_t tree_valid, int32_t offset)
{
    if (!tree_valid || offset < -WORK_SLOW_DX || offset > WORK_SLOW_DX || !Vision_Region)
    {
        stable_region = stable_frames = 0;
        return;
    }
    if (!new_frame) return;
    if (stable_region != Vision_Region)
    {
        stable_region = Vision_Region;
        stable_since = now;
        stable_frames = 1;
    }
    else if (stable_frames < 255) stable_frames++;
}

/* CHECK 阶段决定能否真正启动肘击。调用这里前必须已收到本任务 STOPPED。 */
static void process_check(uint32_t now, uint32_t frame_count,
                          uint8_t new_frame, uint8_t tree_valid, int32_t offset)
{
    if (!tree_valid)
    {
        /* 无树或视觉离线期间不能累计“没有目标”。等 2 秒仍无树则故障。 */
        empty_timing = empty_frames = 0;
        if (!missing_timing) { missing_timing = 1; missing_since = now; }
        if ((uint32_t)(now - missing_since) >= WORK_VISION_WAIT_MS)
            fail(WORK_RESULT_VISION, now);
        return;
    }
    missing_timing = 0;
    if (offset < -(ELBOW_TEST_MODE == 2 ? STATION_VISION_RESTART_PX : WORK_STOP_DX) ||
        offset > (ELBOW_TEST_MODE == 2 ? STATION_VISION_RESTART_PX : WORK_STOP_DX))
    {
        /* 停稳后偏移仍超过 ±30 像素，保持故障停车，避免错位肘击。 */
        fail(WORK_RESULT_POSITION, now);
        return;
    }
    if ((uint32_t)(now - tree_since) >= WORK_TREE_TIMEOUT_MS)
    {
        /* 从首次请求停车起达到 8 秒，记录超时并放行，不再启动下一次。 */
        if (ELBOW_TEST_MODE == 2) fail(WORK_RESULT_TREE_TIMEOUT, now);
        else
        {
            Vision_WorkResult = WORK_RESULT_TREE_TIMEOUT;
            change_state(VISION_WORK_RELEASE, now);
        }
        return;
    }
    if ((uint32_t)(now - check_since) < check_delay)
    {
        /* 等待期间到达的帧也划入旧帧范围，延时结束后再取新结果。 */
        frame_floor = frame_count;
        return;
    }
    if (!new_frame || frame_count == frame_floor) return;
    if (Vision_Region)
    {
        /* 视觉区域为 1～4 才有待处理目标，0 表示当前无目标。 */
        empty_timing = empty_frames = 0;
        if (stable_frames < 3 || (uint32_t)(now - stable_since) < WORK_TARGET_STABLE_MS) return;
        if (attempts[Vision_Region] >= WORK_MAX_ATTEMPTS_PER_REGION)
        {
            /* 当前区域在本棵树已执行 2 次，结束本次作业并携带次数上限结果。 */
            if (ELBOW_TEST_MODE == 2) fail(WORK_RESULT_ATTEMPTS, now);
            else
            {
                Vision_WorkResult = WORK_RESULT_ATTEMPTS;
                change_state(VISION_WORK_RELEASE, now);
            }
            return;
        }
        if (Vision_Motor_Start(Vision_Region))
        {
            /* 机构空闲时才会启动成功；成功后锁定该区域并增加实际动作次数。 */
            attempts[Vision_Region]++;
            change_state(VISION_WORK_ACTING, now);
        }
        return;
    }
    /* 树有效、位置合格、区域为 0，连续至少 500 ms 且至少 3 帧才正常放行。 */
    if (!empty_timing) { empty_timing = 1; empty_since = now; }
    if (empty_frames < 255) empty_frames++;
    if (empty_frames >= WORK_NO_TARGET_FRAMES &&
        (uint32_t)(now - empty_since) >= WORK_NO_TARGET_MS)
        change_state(VISION_WORK_RELEASE, now);
}

/* 放行后屏蔽同一棵树的重复触发。
 * 先观察到底盘运动标志连续存在至少 300 ms，再判断树已离开。
 * 当前离树方向为 -1，所以 dx <= -120 连续 3 帧可重新等待下一棵树。
 * 也允许在视觉仍在线时，连续 600 ms 报告无树后重新等待。
 */
static void process_departure(uint32_t now, uint8_t new_frame, uint8_t tree_valid, int32_t offset)
{
    if (!Vision_Online) { fail(WORK_RESULT_VISION, now); return; }
    if (Work_CAN_Moving())
    {
        if (!moving_timing) { moving_timing = 1; moving_since = now; }
        if ((uint32_t)(now - moving_since) >= WORK_LEAVE_MIN_MOVING_MS) saw_movement = 1;
    }
    else moving_timing = 0;
    if (new_frame && tree_valid)
    {
        missing_timing = 0;
        if (offset * WORK_LEAVE_DIRECTION >= WORK_LEAVE_DX)
        {
            if (departed_frames < 255) departed_frames++;
        }
        else departed_frames = 0;
        if (departed_frames >= 3 && saw_movement) change_state(VISION_WORK_IDLE, now);
    }
    else if (new_frame && !tree_valid)
    {
        departed_frames = 0;
        if (!missing_timing) { missing_timing = 1; missing_since = now; }
        if (saw_movement && (uint32_t)(now - missing_since) >= WORK_LEAVE_MISSING_MS)
            change_state(VISION_WORK_IDLE, now);
    }
}

/* 独立测试只处理相机给出的区域，复用原来的电机动作函数。
 * 每轮结束后重新统计目标稳定性，防止动作过程中收到的旧目标直接触发。
 * 断线清除稳定计数，保留已打击区域，恢复通信后须换区域或先确认无目标。
 */
static void process_test(uint32_t now, uint32_t frame_count)
{
    /* 第 1 步：比较视觉帧编号，同一帧在主循环中重复读取只算一次。 */
    uint8_t new_frame = frame_count != last_frame;
    last_frame = frame_count;
    Vision_WorkState = Vision_MotorPhase ? VISION_WORK_ACTING : VISION_WORK_TEST;
    if (Vision_MotorPhase || !Vision_Online)
    {
        /* 正在动作或视觉离线时清除本轮稳定计数。
         * test_latched_region 保留，避免通信恢复后对同一目标自动再打一遍。
         */
        stable_region = stable_frames = empty_frames = empty_timing = 0;
        return;
    }
    if (!new_frame) return;
    if (!Vision_Region)
    {
        /* 第 2 步：相机在线且区域为 0，说明本帧没有待处理 Gmg。
         * 连续至少 500 ms、至少 3 帧都无目标，才解除上次区域的触发锁。
         */
        stable_region = stable_frames = 0;
        if (!empty_timing) { empty_timing = 1; empty_since = now; }
        if (empty_frames < 255) empty_frames++;
        if (empty_frames >= WORK_NO_TARGET_FRAMES &&
            (uint32_t)(now - empty_since) >= WORK_NO_TARGET_MS)
            test_latched_region = 0;
        return;
    }
    empty_frames = empty_timing = 0;
    /* 第 3 步：区域改变就从第 1 帧重新计时，同一区域继续累计稳定帧数。 */
    if (stable_region != Vision_Region)
    {
        stable_region = Vision_Region;
        stable_frames = 1;
        stable_since = now;
    }
    else if (stable_frames < 255) stable_frames++;
    /* 第 4 步：已经打过且未解除锁、帧数不足 3 或持续不足 200 ms，继续等待。
     * 稳定换到另一区域可触发下一路，此处不检查 Tree、dx 或底盘应答。
     */
    if (stable_region == test_latched_region || stable_frames < 3 ||
        (uint32_t)(now - stable_since) < WORK_TARGET_STABLE_MS) return;
    if (Vision_Motor_Start(stable_region))
    {
        /* 第 5 步：启动成功后记住本次区域。打击、换向、返回仍由电机文件计时。 */
        test_latched_region = stable_region;
        Vision_WorkTask++; /* 测试模式下仅用于记录实际动作次数。 */
        Vision_WorkState = VISION_WORK_ACTING;
    }
}

static void process_link(uint32_t now, uint32_t frame_count)
{
    /* dx 是相机给出的像素偏移，减去机构作业中心后再与区间比较。 */
    int32_t offset = Vision_TreeDX - WORK_DX_CENTER;
    uint8_t tree_valid = Vision_Online && Vision_TreeDX != 9999;
    uint8_t new_frame = frame_count != last_frame;
    uint8_t reply = Work_CAN_GetReply(Vision_WorkTask);
    last_frame = frame_count;
    update_target(now, new_frame, tree_valid, offset);
    if (paired && Vision_WorkState != VISION_WORK_FAULT)
    {
        /* 握手完成后监视断线和底盘故障。
         * 复检或肘击时底盘必须持续回复 STOPPED，否则锁存协议故障。
         */
        if (!Work_CAN_Online()) fail(WORK_RESULT_LINK, now);
        else if (reply == WORK_FAULT) fail(Work_CAN_Result(), now);
        else if ((Vision_WorkState == VISION_WORK_CHECK || Vision_WorkState == VISION_WORK_ACTING) &&
                 reply != WORK_STOPPED) fail(WORK_RESULT_PROTOCOL, now);
    }
    switch (Vision_WorkState)
    {
    case VISION_WORK_BOOT:
        /* 上电先收到 SYNC，再收到 READY，才认为与底盘配对完成。 */
        if (reply == WORK_SYNC) syncing = 1;
        else if (reply == WORK_READY && syncing)
        {
            paired = 1;
            change_state(VISION_WORK_IDLE, now);
        }
        break;
    case VISION_WORK_IDLE:
        /* 等待在线视觉的新帧。树进入 ±100 像素即建立新任务并请求减速。 */
        if (!Vision_Online)
        {
            fail(WORK_RESULT_VISION, now);
            break;
        }
        if (new_frame && tree_valid && offset >= -WORK_SLOW_DX && offset <= WORK_SLOW_DX)
        {
            if (Vision_WorkTask == UINT32_MAX) { fail(WORK_RESULT_PROTOCOL, now); break; }
            Vision_WorkTask++;
            Vision_WorkResult = WORK_RESULT_OK;
            memset(attempts, 0, sizeof(attempts));
            change_state(VISION_WORK_APPROACH, now);
        }
        break;
    case VISION_WORK_APPROACH:
        /* 等待 SLOW_ACK。区域稳定且树在 ±30 像素内时发 WAIT 请求停车。 */
        if (!tree_valid)
        {
            /* 接近时丢树也先请求停车，停稳后再等待视觉恢复。 */
            tree_since = now;
            change_state(VISION_WORK_WAIT_STOP, now);
            break;
        }
        if (reply != WORK_SLOW_ACK) break;
        if (Vision_Region && offset * WORK_LEAVE_DIRECTION > WORK_STOP_DX)
            /* 当前方向下 dx < -30 且仍有目标，认为已越过作业区，故障停车。 */
            fail(WORK_RESULT_POSITION, now);
        else if (offset >= -WORK_STOP_DX && offset <= WORK_STOP_DX &&
                 stable_frames >= 3 && (uint32_t)(now - stable_since) >= WORK_TARGET_STABLE_MS)
        {
            tree_since = now;
            change_state(VISION_WORK_WAIT_STOP, now);
        }
        else if (new_frame)
        {
            if (!Vision_Region && offset * WORK_LEAVE_DIRECTION >= WORK_LEAVE_DX)
            {
                /* 无目标的树连续 3 帧越过离树位置，记录跳过并请求放行。 */
                if (++departed_frames >= 3)
                {
                    Vision_WorkResult = WORK_RESULT_SKIPPED;
                    change_state(VISION_WORK_RELEASE, now);
                }
            }
            else departed_frames = 0;
        }
        break;
    case VISION_WORK_WAIT_STOP:
        /* 最多等底盘 2 秒。收到当前任务 STOPPED 才开始图像复检。 */
        if (reply == WORK_STOPPED) begin_check(now, frame_count, 0);
        else if ((uint32_t)(now - state_since) >= WORK_STOP_TIMEOUT_MS)
            fail(WORK_RESULT_STOP_TIMEOUT, now);
        break;
    case VISION_WORK_CHECK:
        /* 所有肘击启动条件集中在 process_check()。 */
        process_check(now, frame_count, new_frame, tree_valid, offset);
        break;
    case VISION_WORK_ACTING:
        /* 电机定时动作与换向由 vision_motor.c 推进，这里等整轮结束。 */
        if (!Vision_MotorPhase) begin_check(now, frame_count, 1);
        break;
    case VISION_WORK_RELEASE:
        /* 收到 GO_ACK 表示底盘已解除本次作业限制，再进入离树阶段。 */
        if (reply == WORK_GO_ACK)
        {
            moving_timing = missing_timing = saw_movement = 0;
            change_state(VISION_WORK_LEAVING, now);
        }
        break;
    case VISION_WORK_LEAVING:
        process_departure(now, new_frame, tree_valid, offset);
        break;
    default:
        /* 故障锁存。已经启动的动作仍完成受控定时返回，复位后才重新握手。 */
        break;
    }
    publish(); /* 每轮同步最新请求，具体发送节奏由 can_test.c 管理。 */
}

/* 定点作业使用底盘确认的位置，像素偏差保留作观察数据。 */
static void process_station(uint32_t now, uint32_t frame_count)
{
    WorkCanPacket command;
    uint8_t response;
    uint8_t new_frame = frame_count != last_frame;
    uint8_t have_command = Work_CAN_StationCommand(&command);
    uint8_t approaching;
    int32_t offset = Vision_TreeDX - WORK_DX_CENTER;
    last_frame = frame_count;
    if (paired && !have_command && Vision_WorkState != VISION_WORK_FAULT)
        fail(WORK_RESULT_LINK, now);
    if (have_command && command.state == STATION_HOME && !command.task && !command.region)
    {
        if (!Vision_MotorPhase && (!Vision_WorkTask || Vision_WorkState == VISION_WORK_LEAVING ||
            Vision_WorkState == VISION_WORK_FAULT) &&
            Vision_Motor_ConfirmHome())
        {
            Work_Control_Init();
            Work_CAN_SetRequest(STATION_HOME_READY, 0, 0, WORK_RESULT_OK);
            return;
        }
    }
    if (have_command && command.state == STATION_SYNC)
    {
        if (Vision_MotorPhase || Vision_WorkState == VISION_WORK_CHECK ||
            Vision_WorkState == VISION_WORK_ACTING || Vision_WorkState == VISION_WORK_RELEASE)
            fail(WORK_RESULT_RESET, now);
        else if (Vision_Online && Vision_HomeConfirmed)
        {
            paired = 1;
            Vision_WorkTask = 0;
            station_number = 0;
            Vision_WorkResult = WORK_RESULT_OK;
            change_state(VISION_WORK_IDLE, now);
        }
    }
    else if (have_command && command.state == STATION_APPROACH && paired)
    {
        if (command.task > Vision_WorkTask &&
            (Vision_WorkState == VISION_WORK_IDLE || Vision_WorkState == VISION_WORK_LEAVING) &&
            !Vision_MotorPhase && Vision_Online)
        {
            Vision_WorkTask = command.task;
            station_number = command.region;
            Vision_WorkResult = WORK_RESULT_OK;
            memset(attempts, 0, sizeof(attempts));
            stable_region = stable_frames = 0;
            alignment_centered = 0;
            alignment_reply = STATION_TRACKING;
            change_state(VISION_WORK_IDLE, now);
        }
        else if (command.task != Vision_WorkTask || command.region != station_number)
            fail(WORK_RESULT_PROTOCOL, now);
    }
    else if (have_command && command.state == STATION_ALIGN && paired &&
             command.task == Vision_WorkTask && command.region == station_number &&
             (Vision_WorkState == VISION_WORK_IDLE || Vision_WorkState == VISION_WORK_APPROACH ||
              Vision_WorkState == VISION_WORK_WAIT_STOP))
    {
        stable_frames = 0;
        alignment_centered = 0;
        alignment_reply = STATION_TRACKING;
        change_state(VISION_WORK_ALIGN, now);
    }
    else if (have_command && command.state == STATION_START && paired &&
             command.task == Vision_WorkTask && command.region == station_number &&
             Vision_WorkState == VISION_WORK_ALIGN)
    {
        if (!alignment_centered || alignment_reply != STATION_CENTERED || !Vision_Online ||
            Vision_TreeDX == 9999 || offset < -STATION_VISION_RESTART_PX || offset > STATION_VISION_RESTART_PX)
            fail(WORK_RESULT_POSITION, now);
        else
        {
            tree_since = now;
            begin_check(now, frame_count, 0);
        }
    }
    else if (have_command && command.state == STATION_ABORT &&
             command.task == Vision_WorkTask && command.region == station_number)
        fail(command.result ? command.result : WORK_RESULT_PROTOCOL, now);
    else if (have_command && command.state == STATION_RELEASE &&
             command.task == Vision_WorkTask && command.region == station_number &&
             Vision_WorkState == VISION_WORK_RELEASE && !Vision_MotorPhase)
        change_state(VISION_WORK_LEAVING, now);

    approaching = station_number && (Vision_WorkState == VISION_WORK_IDLE ||
        Vision_WorkState == VISION_WORK_APPROACH || Vision_WorkState == VISION_WORK_WAIT_STOP);
    if (approaching)
    {
        if (Vision_MotorResult) fail(Vision_MotorResult, now);
        else if (!Vision_Online) fail(WORK_RESULT_VISION, now);
        else if (new_frame && Vision_TreeDX != 9999)
        {
            if (Vision_WorkState == VISION_WORK_IDLE &&
                offset >= -WORK_SLOW_DX && offset <= WORK_SLOW_DX)
                change_state(VISION_WORK_APPROACH, now);
            if (offset >= -WORK_STOP_DX && offset <= WORK_STOP_DX)
            {
                if (!stable_frames) stable_since = now;
                if (stable_frames < 255) stable_frames++;
                if (stable_frames >= 3 && (uint32_t)(now - stable_since) >= WORK_TARGET_STABLE_MS)
                    change_state(VISION_WORK_WAIT_STOP, now);
            }
            else stable_frames = 0;
        }
        else if (new_frame) stable_frames = 0;
    }
    if (Vision_WorkState == VISION_WORK_CHECK || Vision_WorkState == VISION_WORK_ACTING)
    {
        if (Vision_MotorResult) fail(Vision_MotorResult, now);
        else if (!Vision_Online) fail(WORK_RESULT_VISION, now);
        else if ((uint32_t)(now - tree_since) >= WORK_TREE_TIMEOUT_MS)
            fail(WORK_RESULT_TREE_TIMEOUT, now);
        else if (Vision_WorkState == VISION_WORK_ACTING)
        {
            if (!Vision_MotorPhase) begin_check(now, frame_count, 1);
        }
        else
        {
            uint8_t tree_valid = Vision_Online && Vision_TreeDX != 9999;
            update_target(now, new_frame, tree_valid, offset);
            process_check(now, frame_count, new_frame, tree_valid, offset);
        }
    }
    if (Vision_WorkState == VISION_WORK_ALIGN)
    {
        if (!Vision_Online) fail(WORK_RESULT_VISION, now);
        else if (new_frame)
        {
            if (Vision_TreeDX == 9999)
            {
                alignment_centered = stable_frames = 0;
                alignment_reply = STATION_TRACKING;
            }
            else
            {
                int32_t error = offset < 0 ? -offset : offset;
                if (error <= STATION_VISION_STOP_PX) alignment_centered = 1;
                else if (error > STATION_VISION_RESTART_PX) alignment_centered = 0;
                if (alignment_centered)
                {
                    if (!stable_frames) stable_since = now;
                    if (stable_frames < 255) stable_frames++;
                    alignment_reply = stable_frames >= 3 &&
                        (uint32_t)(now - stable_since) >= STATION_VISION_STABLE_MS ?
                        STATION_CENTERED : STATION_TRACKING;
                }
                else
                {
                    stable_frames = 0;
                    alignment_reply = offset > 0 ? STATION_ALIGN_EAST : STATION_ALIGN_WEST;
                }
            }
        }
    }
    switch (Vision_WorkState)
    {
    case VISION_WORK_IDLE:
        response = station_number ? STATION_TRACKING :
            Vision_Online && Vision_HomeConfirmed ? STATION_READY : STATION_HELLO;
        break;
    case VISION_WORK_APPROACH: response = STATION_SLOW; break;
    case VISION_WORK_WAIT_STOP: response = STATION_WAIT; break;
    case VISION_WORK_ALIGN: response = alignment_reply; break;
    case VISION_WORK_CHECK:
    case VISION_WORK_ACTING: response = STATION_WORKING; break;
    case VISION_WORK_RELEASE: response = STATION_DONE; break;
    case VISION_WORK_LEAVING: response = Vision_Online ? STATION_RELEASED : STATION_HELLO; break;
    case VISION_WORK_FAULT:
        response = Vision_MotorPhase || Vision_MotorResult ? STATION_ABORTING : STATION_FAULT;
        break;
    default: response = STATION_HELLO; break;
    }
    if (response == STATION_HELLO)
        Work_CAN_SetRequest(response, 0, 0, WORK_RESULT_OK);
    else
        Work_CAN_SetRequest(response, Vision_WorkTask, station_number,
            station_can_is_alignment(response) ? (uint8_t)frame_count : Vision_WorkResult);
}

void Work_Control_Task(uint32_t now, uint32_t frame_count)
{
    if (ELBOW_TEST_MODE == 2)
        process_station(now, frame_count);
    else if (ELBOW_TEST_MODE == 1)
        process_test(now, frame_count);
    else if (ELBOW_TEST_MODE == 0)
        process_link(now, frame_count);
}
