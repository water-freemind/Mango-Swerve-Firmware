#include "sys.h"
#include "s21_transport.h"
#include "s21_board.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);exit(1);} } while(0)
volatile uint32_t s21_ms;
USART_TypeDef test_usart1;
CAN_TypeDef test_can1;
static CanTxMsg sent;
static uint8_t box=2,status=CAN_TxStatus_Ok;
static unsigned submits,polls,aborts;
static bool abort_ack=true,tick=true;
uint8_t CAN_Transmit(void* can,CanTxMsg* msg) {
    CHECK(can==CAN1);++submits;sent=*msg;
    test_can1.TSR=0;
    return box;
}
uint8_t CAN_TransmitStatus(void* can,uint8_t mailbox) {
    CHECK(can==CAN1 && mailbox==2);++polls;if(tick)++s21_ms;
    if(status!=CAN_TxStatus_Pending)test_can1.TSR=CAN_TSR_TME0<<mailbox;
    return status;
}
void CAN_CancelTransmit(void* can,uint8_t mailbox) {
    CHECK(can==CAN1 && mailbox==2);++aborts;
    if(abort_ack) {status=CAN_TxStatus_Failed;test_can1.TSR=CAN_TSR_TME0<<mailbox;}
}
int main(void) {
    const uint8_t data[8]={0x12,0x34,0,50,0xff,0xff,0x1d,0x4c};
    CHECK(s21_send_can(0x601,data));
    CHECK(sent.StdId==0x601 && sent.IDE==0 && sent.RTR==0 && sent.DLC==8);
    CHECK(!memcmp(sent.Data,data,8));
    CHECK(!s21_send_can(0x800,data) && submits==1);
    box=CAN_TxStatus_NoMailBox;unsigned before=polls;
    CHECK(!s21_send_can(0x601,data) && polls==before && aborts==0);
    box=2;status=CAN_TxStatus_Failed;CHECK(!s21_send_can(0x601,data));
    status=CAN_TxStatus_Pending;s21_ms=UINT32_MAX;
    CHECK(!s21_send_can(0x601,data) && aborts==1 && !s21_can_abort_fault);
    status=CAN_TxStatus_Ok;CHECK(s21_send_can(0x601,data));
    status=CAN_TxStatus_Pending;tick=false; /* Frozen timer is still bounded by poll budget. */
    CHECK(!s21_send_can(0x601,data) && aborts==2 && !s21_can_abort_fault);
    tick=true;status=CAN_TxStatus_Pending;abort_ack=false;
    CHECK(!s21_send_can(0x601,data) && s21_can_abort_fault==1);
    before=submits;status=CAN_TxStatus_Ok;
    CHECK(!s21_send_can(0x601,data) && submits==before); /* No later frame behind an unconfirmed old request. */
    uint8_t usb[11]={0x7b,0x12,0x34,0,50,0xff,0xff,0x1d,0x4c,0,0x7d};
    test_usart1.SR=USART_SR_TXE|USART_SR_TC;
    CHECK(s21_send_usb(usb) && test_usart1.DR==0x7d && !s21_usb_tx_failures);
    test_usart1.SR=0;
    CHECK(!s21_send_usb(usb) && s21_usb_tx_failures==1);
    test_usart1.SR=USART_SR_TXE;
    CHECK(!s21_send_usb(usb) && s21_usb_tx_failures==2);
    test_usart1.SR=USART_SR_TXE|USART_SR_TC;
    CHECK(s21_send_usb(usb));
    s21_can_abort_fault=0; /* Simulate a fresh healthy controller for independent-sink check. */
    test_usart1.SR=0;
    CHECK(!s21_send_usb(usb));
    CHECK(s21_send_can(0x601,data)); /* USB failure does not gate CAN. */
    puts("S21C_TRANSPORT_OK actual TX code: bytes, ID/DLC, no mailbox, failure, abort/recovery/latch, wrap, frozen clock and USB stalls");
}
