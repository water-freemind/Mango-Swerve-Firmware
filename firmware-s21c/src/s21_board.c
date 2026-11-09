#include "s21_board.h"
#include "sys.h"
StpParser s21_channels[4];
volatile uint32_t s21_ms;
volatile uint32_t s21_can_tx_failures;
extern u16 distance_1,distance_2,distance_3,distance_4;
void s21_init(void) {
    for(unsigned i=0;i<4u;++i) stp_init(&s21_channels[i]);
    distance_1=distance_2=distance_3=distance_4=STP_INVALID_MM;
}
void s21_receive(unsigned channel,uint32_t status,uint8_t byte) {
    StpParser* p=&s21_channels[channel];
    /* Reading SR then DR in the UART adapter clears ORE/FE/NE/PE on F103. */
    if(status & 0x0fu) {stp_uart_error(p);return;}
    if(status & (1u<<5)) (void)stp_feed(p,byte,s21_ms);
}
void s21_poll(void) {
    uint16_t mm[4];
    const uint32_t key=__get_PRIMASK();
    __disable_irq();
    const uint32_t now=s21_ms;
    for(unsigned i=0;i<4u;++i) {
        /* Latch expiry while polling, so a disconnected sample cannot reappear
         * after a complete 32-bit millisecond clock wrap. */
        if(s21_channels[i].valid && (uint32_t)(now-s21_channels[i].last_valid_ms)>STP_TIMEOUT_MS)
            s21_channels[i].valid=false;
        mm[i]=stp_distance(&s21_channels[i],now);
    }
    __set_PRIMASK(key);
    distance_1=mm[0];distance_2=mm[1];distance_3=mm[2];distance_4=mm[3];
}
