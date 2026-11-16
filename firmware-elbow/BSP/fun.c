/**
 * @file fun.c
 * @brief 初始化、编码器测速、OLED 显示和 USART3 诊断收发。
 * @note 肘击条件看 work_control.c，动作与方向看 vision_motor.c，参数看 vision_motor.h。
 *       OLED 每 100 ms 显示一次，USART3 每 500 ms 上报一次，均在主循环执行。
 *       TIM5 中断每 10 ms 更新转速，数组下标 0～3 分别为 A、B、C、D。
 */
#include "headfile.h"
#include "work_can.h"

char str[32];                  /* OLED 单行文本缓冲，先格式化文字，再写入屏幕显存。 */
char uart_rx[32];              /* 串口接收缓冲，解析时使用实际长度，不依赖字符串结束符。 */
/* 中断更新、主循环读取的测速数据。转速单位为 RPM，负号表示编码方向。 */
volatile int32_t Encoder_Count[4] = {0};
volatile int32_t Encoder_Position[4] = {0}; /* A～D 累计计数，供行程测量和回位控制。 */
volatile int32_t Speed[4] = {0};
static uint32_t Encoder_Previous[4] = {0};
/* 每圈编码器四倍频计数，当前每路为 384，需按编码器与减速比标定。 */
static const uint32_t Encoder_CountsPerRev[4] = {384, 384, 384, 384};
volatile uint8_t uart_flag;    /* 最近发生过串口接收事件时置 1。 */
volatile uint16_t uart_rx_size; /* 最近一次串口接收的有效字节数。 */
static char uart_tx[384];      /* 中断发送结束前必须保持有效，发送忙时不改写。 */
volatile uint32_t Vision_UART_ErrorCount = 0;
volatile uint32_t Vision_UART_LastError = 0;
volatile uint32_t Vision_UART_ArmErrorCount = 0;

/* 开启串口空闲中断接收。失败时累计错误，主循环后续可以再次尝试。 */
static void Vision_UART_Arm(void)
{
    if (HAL_UARTEx_ReceiveToIdle_IT(&huart3, (uint8_t *)uart_rx, sizeof(uart_rx)) != HAL_OK)
        Vision_UART_ArmErrorCount++;
}

volatile uint32_t UART_TxCount = 0; /* 成功提交到 HAL 的发送次数。 */
volatile uint32_t UART_TxErrorCount = 0; /* 格式化失败或 HAL 拒绝发送的次数。 */

/** @brief 旧的固定电机方向设置接口。
 *  @param mode 电机编号，1=A、2=B、3=C、4=D。
 *  @note 当前主循环使用 vision_motor.c 的独立区域方向函数。
 */
void Motor_Statu(uint8_t mode)
{
	if(mode == 1)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	}
	else if(mode == 2)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	}
	else if(mode == 3)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
	}	
	else if(mode == 4)
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
	}
}

/** @brief 按指定间隔翻转运行指示灯。
 *  @param time 每次翻转间隔，单位 ms，完整亮灭周期为两倍间隔。
 */
void LED_Blink(uint32_t time)
{
    static uint32_t blink_time = 0;

    if ((uint32_t)(uwTick - blink_time) >= time)
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
        blink_time = uwTick;
    }
}

/* GPIO 初始化后立即显示启动信息，等待视觉、CAN 和激光数据到达。 */
void OLED_PowerOn(void)
{
    OLED_Init();
    if (!OLED_Ready) return;
    OLED_Clear();
    OLED_ShowString(0, 0, ELBOW_TEST_MODE ? "MODE:VISION TEST" : "CAR:WAIT CAN:WAIT", OLED_6X8);
    OLED_ShowString(0, 8, "V:OFF R:0 DX:--", OLED_6X8);
    OLED_ShowString(0, 16, "M:0/- STOP PWM:0%", OLED_6X8);
    OLED_ShowString(0, 24, "W:BOOT E:0", OLED_6X8);
    OLED_ShowString(0, 32, "LEFT:WAIT", OLED_6X8);
    OLED_ShowString(0, 40, "RIGHT:WAIT", OLED_6X8);
    OLED_ShowString(0, 48, "REAR:WAIT", OLED_6X8);
    OLED_ShowString(0, 56, "FRONT:WAIT", OLED_6X8);
    OLED_Update();
    if (OLED_I2C_LastError != 0) OLED_Ready = 0;
}

/* 128×64 OLED，使用 6×8 字体，共 8 行，每 100 ms 刷新。
 * 第 1 行视觉状态/区域/dx，第 2 行动作电机/阶段/倒计时/PWM。
 * 第 3 行 CAR 车辆状态和 CAN 在线状态，第 4 行 W 作业阶段/E 结果/C 通信诊断。
 * 第 5～8 行 LEFT/RIGHT/REAR/FRONT 激光毫米值，转速保留在串口输出。
 */
void OLED_Display(void)
{
    static uint32_t last_display = 0;
    static uint8_t first_display = 1;
    static uint8_t transfer_page = 0;
    uint32_t now = HAL_GetTick();
    uint8_t i, phase, motor, pwm;
    uint32_t remaining_tenths;
    const char *direction, *car_state, *job_state;
    uint8_t reply, online, result;
    static const char *laser_names[4] = {"LEFT", "RIGHT", "REAR", "FRONT"};
    uint16_t distance_mm;
    uint8_t laser_status;
    /* 屏幕初始化或刷新失败后，每秒重试一次，避免每轮反复初始化。 */
    if (!OLED_Ready)
    {
        static uint32_t last_retry = 0;
        /* Reinitialization clears the whole panel. Defer it during an action
         * or its image check so display recovery cannot extend motor timing. */
        if (Vision_MotorPhase || Vision_WorkState == VISION_WORK_CHECK ||
            Vision_WorkState == VISION_WORK_ACTING) return;
        if ((uint32_t)(now - last_retry) < 1000U) return;
        last_retry = now;
        OLED_Init();
        if (!OLED_Ready) return;
        first_display = 1;
        transfer_page = 0;
    }
    if (!first_display && (uint32_t)(now - last_display) < 100U) goto transfer;
    first_display = 0;
    last_display = now;
    phase = Vision_MotorPhase;
    motor = Vision_MotorRegion;
    pwm = (phase == 1 || phase == 3) ? VISION_MOTOR_PWM : 0;
    direction = phase == 1 ? "HIT" : phase == 2 ? "PAUSE" :
                phase == 3 ? "BACK" : phase == 4 ? "COOL" : "STOP";
    /* 先判断应答是否新鲜，再显示本次任务状态，离线时隐藏旧状态。 */
    online = Work_CAN_Online();
    reply = Work_CAN_GetReply(Vision_WorkTask);
    switch (reply)
    {
    case WORK_SYNC: car_state = "SYNC"; break;
    case WORK_READY: car_state = Work_CAN_Moving() ? "MOVING" : "READY"; break;
    case WORK_SLOW_ACK: car_state = "SLOW"; break;
    case WORK_STOPPING: car_state = "STOPPING"; break;
    case WORK_STOPPED: car_state = "STOPPED"; break;
    case WORK_GO_ACK: car_state = Work_CAN_Moving() ? "MOVING" : "RELEASED"; break;
    case WORK_FAULT: car_state = "FAULT"; break;
    default: car_state = online ? "WAIT" : "OFFLINE"; break;
    }
    if (ELBOW_TEST_MODE == 2)
    {
        switch (reply)
        {
        case STATION_SYNC: car_state = Vision_WorkState == VISION_WORK_IDLE && Vision_Online ? "READY" : "SYNC"; break;
        case STATION_APPROACH: car_state = "APPROACH"; break;
        case STATION_ALIGN: car_state = "ALIGN"; break;
        case STATION_START: car_state = "STOPPED"; break;
        case STATION_RELEASE: car_state = "RELEASED"; break;
        case STATION_ABORT: car_state = "ABORT"; break;
        default: car_state = online ? "WAIT" : "OFFLINE"; break;
        }
    }
    /* W 对应肘击作业阶段，CAR 对应底盘应答，两者含义分别显示。 */
    switch (Vision_WorkState)
    {
    case VISION_WORK_TEST: job_state = "TEST_WAIT"; break;
    case VISION_WORK_BOOT: job_state = "BOOT"; break;
    case VISION_WORK_IDLE: job_state = "IDLE"; break;
    case VISION_WORK_APPROACH: job_state = "APPROACH"; break;
    case VISION_WORK_WAIT_STOP: job_state = "WAIT_STOP"; break;
    case VISION_WORK_ALIGN: job_state = "ALIGN"; break;
    case VISION_WORK_CHECK: job_state = "CHECK"; break;
    case VISION_WORK_ACTING: job_state = "ACTING"; break;
    case VISION_WORK_RELEASE: job_state = "RELEASE"; break;
    case VISION_WORK_LEAVING: job_state = "LEAVING"; break;
    default: job_state = "FAULT"; break;
    }
    if (ELBOW_TEST_MODE == 2 && !Vision_Online && !Vision_MotorPhase &&
        (Vision_WorkState == VISION_WORK_IDLE || Vision_WorkState == VISION_WORK_BOOT))
        job_state = "WAIT_CAM";
    /* E 优先显示底盘故障原因，其余情况显示肘击端当前结果码。 */
    result = ELBOW_TEST_MODE != 2 && reply == WORK_FAULT ? Work_CAN_Result() : Vision_WorkResult;
    remaining_tenths = (Vision_Motor_RemainingMs() + 99U) / 100U;
    OLED_Clear();
    if (Vision_TreeDX == 9999)
        sprintf(str, "V:%s R:%u DX:--", Vision_Online ? "ON" : "OFF", Vision_Region);
    else
        sprintf(str, "V:%s R:%u DX:%ld", Vision_Online ? "ON" : "OFF",
                Vision_Region, (long)Vision_TreeDX);
    OLED_ShowString(0, 0, str, OLED_6X8);
    snprintf(str, sizeof(str), "%c %s %lu.%lus %u%%",
            motor ? 'A' + motor - 1 : '-', direction,
            (unsigned long)(remaining_tenths / 10U),
            (unsigned long)(remaining_tenths % 10U), pwm);
    OLED_ShowString(0, 8, str, OLED_6X8);
    if (ELBOW_TEST_MODE == 3)
        snprintf(str, sizeof(str), "MODE:ENCODER");
    else if (ELBOW_TEST_MODE == 1)
        snprintf(str, sizeof(str), "MODE:VISION TEST");
    else
        snprintf(str, sizeof(str), "CAR:%s CAN:%s", car_state, online ? "ON" : "OFF");
    OLED_ShowString(0, 16, str, OLED_6X8);
    snprintf(str, sizeof(str), "W:%s E:%u C:%u", job_state, result, CAN_Test_State);
    OLED_ShowString(0, 24, str, OLED_6X8);
    for (i = 0; i < 4; i++)
    {
        /* WAIT 未收帧，INVALID 超范围，OFFLINE 超过 300 ms 无新帧。 */
        laser_status = CAN_Laser_Read(i, &distance_mm);
        switch (laser_status)
        {
        case 1: snprintf(str, sizeof(str), "%s:%u mm", laser_names[i], distance_mm); break;
        case 2: snprintf(str, sizeof(str), "%s:INVALID", laser_names[i]); break;
        case 3: snprintf(str, sizeof(str), "%s:OFFLINE", laser_names[i]); break;
        default: snprintf(str, sizeof(str), "%s:WAIT", laser_names[i]); break;
        }
        OLED_ShowString(0, 32 + i * 8, str, OLED_6X8);
    }
transfer:
    /* One page costs about 19ms at the configured 5us I2C delay. A full
     * frame blocks CAN polling and motor phase updates for at least 151ms. */
    OLED_I2C_LastError = 0;
    OLED_UpdateArea(0, (int16_t)(transfer_page * 8U), 128, 8);
    if (OLED_I2C_LastError != 0) OLED_Ready = 0;
    else transfer_page = (uint8_t)((transfer_page + 1U) % 8U);
}


/** @brief USART3 每 500 ms 输出四路转速、作业、CAN 和 OLED 诊断。
 *  @note PB10=TX、PB11=RX，115200 bit/s，8N1，使用中断发送。
 *        UART 忙时跳过本轮发送，接收与作业流程继续运行。
 */
void UART_SendStatus(void)
{
    static uint32_t last_uart = 0;
    uint32_t now = HAL_GetTick();
    uint32_t primask;
    int32_t rpm[4];
    uint8_t i;
    int length;
    if ((uint32_t)(now - last_uart) < 500U ||
        huart3.gState != HAL_UART_STATE_READY) return;
    /* 短暂保护四路测速数据的读取，随后恢复原中断状态。 */
    primask = __get_PRIMASK();
    __disable_irq();
    for (i = 0; i < 4; i++) rpm[i] = Speed[i];
    __set_PRIMASK(primask);
    /* 静态缓冲在中断发送完成前始终有效，发送忙时不会重写。 */
    if ((uint32_t)(now - last_uart) >= 500U && huart3.gState == HAL_UART_STATE_READY)
    {
        last_uart = now;
        length = snprintf(uart_tx, sizeof(uart_tx),
            "A:%ld RPM B:%ld RPM C:%ld RPM D:%ld RPM | WORK:%u TASK:%lu RESULT:%u VIS:%u REGION:%u DX:%ld | CAN2 WORK %s TX:%lu RX:%lu ERR:%lu S:%u | OLED:%u OE:%lu NACK:%lu\r\n",
            (long)rpm[0], (long)rpm[1], (long)rpm[2], (long)rpm[3],
            Vision_WorkState, (unsigned long)Vision_WorkTask, Vision_WorkResult,
            Vision_Online, Vision_Region, (long)Vision_TreeDX,
            CAN_Test_State == 1 ? "OK" : (CAN_Test_State == 0 ? "WAIT" : "FAIL"),
            (unsigned long)CAN_Test_TxCount, (unsigned long)CAN_Test_RxCount,
            (unsigned long)CAN_Test_ErrorCount, CAN_Test_State,
            OLED_Ready, (unsigned long)OLED_I2C_LastError,
            (unsigned long)OLED_I2C_NackCount);
        if (length < 0 || length >= (int)sizeof(uart_tx))
        {
            UART_TxErrorCount++;
            return;
        }
        if (HAL_UART_Transmit_IT(&huart3, (uint8_t *)uart_tx, (uint16_t)length) == HAL_OK)
            UART_TxCount++;
        else UART_TxErrorCount++;
    }
}

/** @brief 配置外设完成后，启动串口、PWM、编码器和 10 ms 测速中断。
 *  @note OLED 已在 main.c 的 GPIO 初始化后通过 OLED_PowerOn() 提前启动。
 *        Vision_Motor_Init() 初始化作业流程并关闭全部电机输出。
 */
void Init(void)
{
  Vision_Motor_Init();

    /* 使用串口空闲中断接收一批字节。 */
	Vision_UART_Arm();

	// 启动四路 PWM 定时器通道，实际占空比初值为 0。
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);	
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);

	// 编码器由定时器硬件计数，TIM5 每 10 ms 读取一次计数变化。
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);	
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);	
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
	


	// 保存各路初始计数、清除旧标志，然后开启 TIM5 周期中断。
	Encoder_Previous[0] = __HAL_TIM_GET_COUNTER(&htim8);
  Encoder_Previous[1] = __HAL_TIM_GET_COUNTER(&htim3);
  Encoder_Previous[2] = __HAL_TIM_GET_COUNTER(&htim4);
  Encoder_Previous[3] = __HAL_TIM_GET_COUNTER(&htim2);
  __HAL_TIM_SET_COUNTER(&htim5, 0);
  __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
  HAL_TIM_Base_Start_IT(&htim5);
	
	
	/* ARR=99 时，一个 PWM 周期有 100 个计数。上电四路 CCR 都设为 0。 */
	TIM1->ARR = 99;

	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = 0;	
}

/** @brief TIM5 每 10 ms 计算四路编码器增量和转速。
 *  @param htim 产生此次周期中断的定时器。
 *  @note 根据硬件计数器位宽处理回绕，单个采样周期增量需小于计数范围一半。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    TIM_HandleTypeDef *encoders[4] = {&htim8, &htim3, &htim4, &htim2};
    uint8_t i;
    uint32_t current, difference;
    int64_t delta;
    if (htim->Instance != TIM5) return;
    for (i = 0; i < 4; i++)
    {
        current = __HAL_TIM_GET_COUNTER(encoders[i]);
        difference = current - Encoder_Previous[i];
        Encoder_Previous[i] = current;
        if (i == 3) /* TIM2 是 32 位计数器，处理计数回绕和负方向。 */
            delta = (difference & 0x80000000U) ?
                    (int64_t)difference - 4294967296LL : difference;
        else /* TIM3、TIM4、TIM8 是 16 位计数器，只保留低 16 位再换算有符号增量。 */
        {
            difference &= 0xFFFFU;
            delta = (int64_t)difference;
            if (difference & 0x8000U) delta -= 65536LL;
        }
        Encoder_Count[i] = (int32_t)delta;
        Encoder_Position[i] = (int32_t)((uint32_t)Encoder_Position[i] + (uint32_t)(int32_t)delta);
        /* RPM = 10 ms 内计数增量 / 每圈计数 × 100 × 60。 */
        Speed[i] = (int32_t)(delta * 6000 / Encoder_CountsPerRev[i]);
    }
    Vision_Motor_EncoderTick();
}


/* 先把本次字节送给视觉解析器，再重新开启接收，避免缓冲被提前覆盖。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance==USART3)
	{
		uart_flag=1;
    uart_rx_size = Size;
    Vision_Motor_Receive((const uint8_t *)uart_rx, Size);
    /* 本批数据处理完成，继续等待下一批串口数据。 */
	Vision_UART_Arm();
	}
}

/* 出现串口帧错误/溢出时丢弃未完成帧，再恢复接收。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) return;
    Vision_UART_LastError = HAL_UART_GetError(huart);
    Vision_UART_ErrorCount++;
    Vision_Motor_RxError();
    (void)HAL_UART_AbortReceive(huart);
    Vision_UART_Arm();
}

/* 主循环入口，必要时恢复接收，再更新视觉状态、作业流程和电机计时。 */
void Vision_UART_Task(void)
{
    if (huart3.RxState == HAL_UART_STATE_READY)
        Vision_UART_Arm();
    Vision_Motor_Task();
}
