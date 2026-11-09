#include "s21_transport.h"
#include "s21_board.h"
#include "sys.h"
#include <string.h>
volatile uint32_t s21_usb_tx_failures;
volatile uint32_t s21_can_abort_fault;
bool s21_send_can(uint32_t id,const uint8_t bytes[8]) {
    if(id>0x7ffu || s21_can_abort_fault) return false;
    CanTxMsg message={0};
    message.StdId=id;
    message.IDE=CAN_Id_Standard;
    message.RTR=CAN_RTR_Data;
    message.DLC=8;
    memcpy(message.Data,bytes,8);
    const uint8_t box=CAN_Transmit(CAN1,&message);
    if(box==CAN_TxStatus_NoMailBox) return false;
    const uint32_t start=s21_ms;
    unsigned budget=200000u;
    uint8_t status;
    do {
        status=CAN_TransmitStatus(CAN1,box);
        if(status==CAN_TxStatus_Ok) return true;
        if(status==CAN_TxStatus_Failed) return false;
    } while((uint32_t)(s21_ms-start)<2u && --budget);
    /* Expired samples must not remain queued for later transmission. */
    CAN_CancelTransmit(CAN1,box);
    const uint32_t abort_start=s21_ms;
    budget=200000u;
    /* Confirm the mailbox is actually empty; a generic failed-status code is
     * not by itself proof that a cancellation request has completed. */
    while(!(CAN1->TSR & (CAN_TSR_TME0<<box))) {
        if((uint32_t)(s21_ms-abort_start)>=2u || !--budget) {
            s21_can_abort_fault=1; /* Require reset if hardware cannot acknowledge abort. */
            break;
        }
    }
    return false;
}
bool s21_send_usb(const uint8_t bytes[11]) {
    const uint32_t start=s21_ms;
    unsigned budget=200000u;
    for(unsigned i=0;i<11u;++i) {
        while(!(USART1->SR & USART_SR_TXE)) {
            if((uint32_t)(s21_ms-start)>=10u || !--budget) goto failed;
        }
        USART1->DR=bytes[i];
    }
    while(!(USART1->SR & USART_SR_TC)) {
        if((uint32_t)(s21_ms-start)>=10u || !--budget) goto failed;
    }
    return true;
failed:
    ++s21_usb_tx_failures;
    return false;
}
