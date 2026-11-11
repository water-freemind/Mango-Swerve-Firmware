
#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
#define HAL_GPIO_WritePin H7_Fake_HAL_GPIO_WritePin
#include "can_hal_fake.h"
#undef HAL_GPIO_WritePin
#undef GPIO_PIN_SET
typedef struct { unsigned id; } GPIO_TypeDef;
extern GPIO_TypeDef portc, portd;
#define GPIOC (&portc)
#define GPIOD (&portd)
#define GPIO_PIN_0 1U
#define GPIO_PIN_1 2U
#define GPIO_PIN_2 4U
#define GPIO_PIN_3 8U
#define GPIO_PIN_4 16U
#define GPIO_PIN_5 32U
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, int state);
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);
#define USART3 ((void *)3)
#define HAL_UART_STATE_READY 1U
#define HAL_OK 0
#define HAL_BUSY 2
#define HAL_ERROR 1
#define BUSY_RX 2U
typedef struct { void *Instance; unsigned RxState, ErrorCode; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart3;
extern char uart_rx[32];
extern volatile uint8_t uart_flag;
extern volatile uint16_t uart_rx_size;
extern volatile uint32_t Vision_UART_ErrorCount, Vision_UART_LastError;
extern volatile uint32_t Vision_UART_ArmErrorCount;
int HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
int HAL_UART_AbortReceive(UART_HandleTypeDef *);
uint32_t HAL_UART_GetError(UART_HandleTypeDef *);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *, uint16_t);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
void Vision_UART_Task(void);
typedef struct { int unused; } CAN_HandleTypeDef;
extern CAN_HandleTypeDef hcan2;
typedef struct { uint32_t StdId, IDE, RTR, DLC; } CAN_RxHeaderTypeDef;
typedef struct { uint32_t StdId, IDE, RTR, DLC, TransmitGlobalTime; } CAN_TxHeaderTypeDef;
typedef struct {
 uint32_t FilterBank, SlaveStartFilterBank, FilterMode, FilterScale;
 uint32_t FilterFIFOAssignment, FilterActivation, FilterIdHigh, FilterIdLow;
 uint32_t FilterMaskIdHigh, FilterMaskIdLow;
} CAN_FilterTypeDef;
#define CAN_FILTERMODE_IDMASK 0u
#define CAN_FILTERSCALE_32BIT 0u
#define CAN_RX_FIFO0 0u
#define CAN_ID_STD 0u
#define CAN_RTR_DATA 0u
#define ENABLE 1u
#define DISABLE 0u
int HAL_CAN_ConfigFilter(CAN_HandleTypeDef*, CAN_FilterTypeDef*);
int HAL_CAN_Start(CAN_HandleTypeDef*);
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef*);
int HAL_CAN_AbortTxRequest(CAN_HandleTypeDef*, uint32_t);
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef*, uint32_t);
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef*, uint32_t);
int HAL_CAN_GetRxMessage(CAN_HandleTypeDef*, uint32_t, CAN_RxHeaderTypeDef*, uint8_t*);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef*);
int HAL_CAN_AddTxMessage(CAN_HandleTypeDef*, CAN_TxHeaderTypeDef*, uint8_t*, uint32_t*);

#endif
