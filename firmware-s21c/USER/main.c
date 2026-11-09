#include "sys.h"
#include "s21_board.h"

void oled_show(void);
u16 distance_1,distance_2,distance_3,distance_4;
volatile u8 Send_Flag;
u8 Usart_Send_Data[11],CAN_Send_Data[8];
volatile uint32_t s21_can_init_status;
int main(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    delay_init();
    JTAG_Set(SWD_ENABLE);
    s21_init();
    LED_Init();
    OLED_Init();
    s21_can_init_status=CAN1_Mode_Init(1,3,2,6,0);
    uart_init(115200);
    uart2_init(230400);
    uart3_init(230400);
    uart4_init(230400);
    uart5_init(230400);
    time5_init();
    for(;;) {
        const uint32_t key=__get_PRIMASK();
        __disable_irq();
        const u8 send=Send_Flag;
        Send_Flag=0;
        __set_PRIMASK(key);
        if(!send) continue;
        s21_poll();
        data_trastion();
        /* Bus control data has priority over the diagnostic USB/OLED sinks. */
        if(s21_can_init_status || CAN1_Send_Num(0x601,CAN_Send_Data)) ++s21_can_tx_failures;
        USART1_SEND();
        oled_show();
    }
}
