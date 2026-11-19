#ifndef CAR_BUTTON_PROTOCOL_H
#define CAR_BUTTON_PROTOCOL_H
#include <stdint.h>
/* 视觉按键单独使用 0x612/0x613，不改变原有作业协议。 */
#define CAR_REQUEST_ID 0x612u
#define CAR_REPLY_ID 0x613u
enum { CAR_NAV=1, CAR_WORK, CAR_ORIGIN, CAR_ARM, CAR_GO, CAR_STOP, CAR_STATUS,
    CAR_START_NAV, CAR_START_WORK };
static inline uint8_t car_checksum(const uint8_t *p) {
    uint8_t x=0x5a; for(unsigned i=0;i<7;i++) x^=p[i]; return x;
}
static inline int car_valid(const uint8_t *p) {
    return p[0]==0xc1 && p[1]>=CAR_NAV && p[1]<=CAR_START_WORK &&
        (p[2] || p[3]) && p[7]==car_checksum(p);
}
/* F407 串口中断只发布请求，CAN 和串口发送均由主循环执行。 */
extern volatile uint16_t Car_ButtonSeq;
extern volatile uint8_t Car_ButtonOp;
extern volatile uint16_t Car_AckSeq;
extern volatile uint8_t Car_AckOp, Car_AckResult, Car_AckMode, Car_AckPhase;
#endif
