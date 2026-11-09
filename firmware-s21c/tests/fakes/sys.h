#ifndef S21_TEST_SYS_H
#define S21_TEST_SYS_H
#include <stdint.h>
typedef uint16_t u16;
extern uint32_t test_irq_mask;
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
typedef struct { uint32_t StdId,ExtId; uint8_t IDE,RTR,DLC,Data[8]; } CanTxMsg;
typedef struct { volatile uint32_t SR,DR; } USART_TypeDef;
extern USART_TypeDef test_usart1;
#define USART1 (&test_usart1)
typedef struct {volatile uint32_t TSR;} CAN_TypeDef;
extern CAN_TypeDef test_can1;
#define CAN1 (&test_can1)
#define CAN_TSR_TME0 (1u<<26)
#define USART_SR_TXE (1u<<7)
#define USART_SR_TC (1u<<6)
#define CAN_Id_Standard 0u
#define CAN_RTR_Data 0u
#define CAN_TxStatus_Failed 0u
#define CAN_TxStatus_Ok 1u
#define CAN_TxStatus_Pending 2u
#define CAN_TxStatus_NoMailBox 4u
uint8_t CAN_Transmit(void* can,CanTxMsg* msg);
uint8_t CAN_TransmitStatus(void* can,uint8_t box);
void CAN_CancelTransmit(void* can,uint8_t box);
#endif
