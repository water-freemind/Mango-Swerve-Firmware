#include "s21_board.h"
#include "time.h"
void time5_init(void) {
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    const uint32_t timer_hz=clocks.PCLK1_Frequency * (clocks.PCLK1_Frequency==clocks.HCLK_Frequency ? 1u : 2u);
    TIM_TimeBaseInitTypeDef timer;
    TIM_TimeBaseStructInit(&timer);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
    timer.TIM_Period=999;
    timer.TIM_Prescaler=(uint16_t)(timer_hz/1000000u-1u);
    timer.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5,&timer);
    NVIC_InitTypeDef irq={0};
    irq.NVIC_IRQChannel=TIM5_IRQn;
    irq.NVIC_IRQChannelCmd=ENABLE;
    irq.NVIC_IRQChannelPreemptionPriority=0;
    irq.NVIC_IRQChannelSubPriority=0;
    TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
    NVIC_Init(&irq);
    TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM5,ENABLE);
}
void TIM5_IRQHandler(void) {
    static unsigned ticks;
    if(TIM_GetITStatus(TIM5,TIM_IT_Update)!=RESET) {
        TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
        ++s21_ms;
        LED_Flash(1000);
        if(++ticks==100u) {ticks=0;Send_Flag=1;}
    }
}
