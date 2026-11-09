#include "sys.h"
#include "s21_board.h"
#include <stdio.h>
#include <stdlib.h>
uint32_t test_irq_mask;
u16 distance_1,distance_2,distance_3,distance_4;
uint32_t __get_PRIMASK(void) {return test_irq_mask;}
void __disable_irq(void) {test_irq_mask=1;}
void __set_PRIMASK(uint32_t mask) {test_irq_mask=mask;}
#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL board %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static void measurement(unsigned ch,uint16_t mm,uint32_t stamp) {
    uint8_t b[195]={0xaa,0xaa,0xaa,0xaa,0,2,0,0,184,0};
    for(unsigned i=0;i<12u;++i){b[10u+15u*i]=(uint8_t)mm;b[11u+15u*i]=(uint8_t)(mm>>8);}
    for(unsigned i=0;i<4u;++i)b[190u+i]=(uint8_t)(stamp>>(8u*i));
    uint8_t sum=0;for(unsigned i=4;i<194u;++i)sum=(uint8_t)(sum+b[i]);b[194]=sum;
    for(unsigned i=0;i<195u;++i)s21_receive(ch,1u<<5,b[i]);
}
int main(void) {
    s21_init();s21_poll();CHECK(distance_1==65535 && distance_4==65535 && !test_irq_mask);
    s21_ms=100;
    for(unsigned i=0;i<4u;++i)measurement(i,(uint16_t)(100+i*100),1);
    test_irq_mask=1;s21_poll();CHECK(test_irq_mask==1);test_irq_mask=0;
    CHECK(distance_1==100 && distance_2==200 && distance_3==300 && distance_4==400);
    s21_ms=401;
    for(unsigned i=1;i<4u;++i)measurement(i,(uint16_t)(100+i*100),2);
    s21_poll();CHECK(distance_1==65535 && distance_2==200 && !s21_channels[0].valid);
    /* An expired measurement stays invalid even if the 32-bit timer returns to its old value. */
    s21_ms=100;s21_poll();CHECK(distance_1==65535);
    s21_ms=402;measurement(0,50,2);s21_poll();CHECK(distance_1==50);
    unsigned old=s21_channels[0].used;
    s21_receive(0,0,0xaa);CHECK(s21_channels[0].used==old);
    s21_receive(0,(1u<<5)|8u,0xaa);CHECK(s21_channels[0].used==0 && s21_channels[0].uart_errors==1);
    puts("S21C_BOARD_OK actual RX adapter: connector order, IRQ restore, errors, independent timeout latch and recovery");
}
