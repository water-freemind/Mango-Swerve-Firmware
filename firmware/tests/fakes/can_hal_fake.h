#ifndef MANGO_SWERVE_CAN_HAL_FAKE_H
#define MANGO_SWERVE_CAN_HAL_FAKE_H

/* Host-only HAL boundary; the production build uses the STM32 vendor headers. */
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef struct { uint32_t TXBRP, PSR, ECR, TXBTO; } FDCAN_GlobalTypeDef;
typedef struct {
    uint32_t StdFiltersNbr, TxBuffersNbr, TxFifoQueueElmtsNbr;
} FDCAN_InitTypeDef;
typedef struct {
    FDCAN_GlobalTypeDef* Instance;
    FDCAN_InitTypeDef Init;
    uint32_t State, ErrorCode;
} FDCAN_HandleTypeDef;
typedef struct {
    uint32_t Identifier, IdType, RxFrameType, DataLength, FDFormat;
} FDCAN_RxHeaderTypeDef;
typedef struct {
    uint32_t Identifier, IdType, TxFrameType, DataLength, ErrorStateIndicator;
    uint32_t BitRateSwitch, FDFormat, TxEventFifoControl, MessageMarker;
} FDCAN_TxHeaderTypeDef;
typedef struct {
    uint32_t IdType, FilterIndex, FilterType, FilterConfig, FilterID1, FilterID2;
} FDCAN_FilterTypeDef;
typedef struct { uint32_t BusOff, ErrorPassive; } FDCAN_ProtocolStatusTypeDef;

extern FDCAN_GlobalTypeDef g_fake_can_regs[3];
#define FDCAN1 (&g_fake_can_regs[0])
#define FDCAN2 (&g_fake_can_regs[1])
#define FDCAN3 (&g_fake_can_regs[2])
#define HAL_FDCAN_STATE_BUSY 2u
#define FDCAN_STANDARD_ID 0u
#define FDCAN_EXTENDED_ID 0x40000000u
#define FDCAN_DATA_FRAME 0u
#define FDCAN_REMOTE_FRAME 0x20000000u
#define FDCAN_CLASSIC_CAN 0u
#define FDCAN_FD_CAN 0x00200000u
#define FDCAN_DLC_BYTES_0 0u
#define FDCAN_DLC_BYTES_1 1u
#define FDCAN_DLC_BYTES_2 2u
#define FDCAN_DLC_BYTES_3 3u
#define FDCAN_DLC_BYTES_4 4u
#define FDCAN_DLC_BYTES_5 5u
#define FDCAN_DLC_BYTES_6 6u
#define FDCAN_DLC_BYTES_7 7u
#define FDCAN_DLC_BYTES_8 8u
#define FDCAN_ESI_ACTIVE 0u
#define FDCAN_BRS_OFF 0u
#define FDCAN_NO_TX_EVENTS 0u
#define FDCAN_FILTER_MASK 2u
#define FDCAN_FILTER_TO_RXFIFO0 1u
#define FDCAN_ACCEPT_IN_RX_FIFO0 0u
#define FDCAN_REJECT 2u
#define FDCAN_REJECT_REMOTE 1u
#define FDCAN_RX_FIFO0 0x40u
#define FDCAN_INTERRUPT_LINE0 1u
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE (1u << 0)
#define FDCAN_IT_RX_FIFO0_FULL (1u << 2)
#define FDCAN_IT_RX_FIFO0_MESSAGE_LOST (1u << 3)
#define FDCAN_IT_TX_COMPLETE (1u << 9)
#define FDCAN_IT_ERROR_PASSIVE (1u << 23)
#define FDCAN_IT_BUS_OFF (1u << 25)
#define FDCAN_IT_ARB_PROTOCOL_ERROR (1u << 27)
#define FDCAN_IT_DATA_PROTOCOL_ERROR (1u << 28)
#define CAN1_EN_GPIO_Port 1u
#define CAN2_EN_GPIO_Port 2u
#define CAN1_EN_Pin 1u
#define CAN2_EN_Pin 2u
#define GPIO_PIN_SET 1u

HAL_StatusTypeDef HAL_FDCAN_ConfigInterruptLines(FDCAN_HandleTypeDef*, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef*, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef*, const FDCAN_FilterTypeDef*);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef*, uint32_t, uint32_t, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef*, FDCAN_ProtocolStatusTypeDef*);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef*, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef*, FDCAN_TxHeaderTypeDef*, uint8_t*);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef*, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef*, uint32_t, FDCAN_RxHeaderTypeDef*, uint8_t*);
void HAL_GPIO_WritePin(uint32_t, uint32_t, uint32_t);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef*, uint32_t);

#endif
