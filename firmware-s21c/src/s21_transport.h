#ifndef S21_TRANSPORT_H
#define S21_TRANSPORT_H
#include <stdbool.h>
#include <stdint.h>
extern volatile uint32_t s21_usb_tx_failures;
extern volatile uint32_t s21_can_abort_fault;
bool s21_send_can(uint32_t id, const uint8_t bytes[8]);
bool s21_send_usb(const uint8_t bytes[11]);
#endif
