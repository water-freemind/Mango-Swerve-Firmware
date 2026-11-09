#ifndef S21_BOARD_H
#define S21_BOARD_H
#include <stdint.h>
#include "stp23l.h"
extern StpParser s21_channels[4]; /* IRQ writers; inspect diagnostics only while IRQs masked. */
extern volatile uint32_t s21_ms;
extern volatile uint32_t s21_can_tx_failures;
void s21_init(void);
void s21_poll(void);
void s21_receive(unsigned channel,uint32_t status,uint8_t byte);
#endif
