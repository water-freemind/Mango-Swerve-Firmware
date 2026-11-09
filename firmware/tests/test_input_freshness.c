/* Inject RX immediately after reading HAL time in the actual input drivers. */
typedef struct { unsigned unused; } UART_HandleTypeDef;
#include "../src/device/fs_ia10b.c"
#include "laser.h"
#include <assert.h>
#include <stdio.h>

uint32_t g_test_tick_ms, g_test_irq_mask;
UART_HandleTypeDef huart1, huart5;
static bool inject_rx;
static const uint8_t distances[8] = {1, 0, 2, 0, 3, 0, 4, 0};

uint32_t test_rx_tick(void) {
    const uint32_t now = g_test_tick_ms;
    if(inject_rx && !g_test_irq_mask) {
        inject_rx = false;
        ++g_test_tick_ms;
        s_data.last_update_ms = g_test_tick_ms;
        laser_accept_frame(distances, g_test_tick_ms);
    }
    return now;
}
LogStatus log_info(const char* format, ...) { (void)format; return LOG_STATUS_OK; }
bool uart_receive_it(UART_HandleTypeDef* h, uint8_t* p, uint16_t n) {
    (void)h; (void)p; (void)n; return true;
}
bool uart_abort_receive_it(UART_HandleTypeDef* h) { (void)h; return true; }
void uart_register_rx_complete_callback(UART_HandleTypeDef* h, void (*cb)(void)) { (void)h; (void)cb; }
void uart_register_error_callback(UART_HandleTypeDef* h, void (*cb)(void)) { (void)h; (void)cb; }

int main(void) {
    LaserSnapshot snapshot;
    g_test_tick_ms = 1000u;
    s_data.valid = true;
    s_data.last_update_ms = 1000u;
    inject_rx = true;
    assert(ibus_is_online(100u));
    assert(!inject_rx && g_test_irq_mask == 0u);
    laser_accept_frame(distances, g_test_tick_ms);
    inject_rx = true;
    assert(laser_get_snapshot(&snapshot) && snapshot.frame_fresh);
    assert(g_test_irq_mask == 0u);
    inject_rx = false;
    g_test_irq_mask = 1u;
    assert(laser_get_snapshot(&snapshot) && snapshot.frame_fresh);
    assert(ibus_is_online(100u) && g_test_irq_mask == 1u);
    g_test_irq_mask = 0u;
    g_test_tick_ms += 101u;
    assert(!ibus_is_online(100u));
    g_test_tick_ms += 200u;
    assert(laser_get_snapshot(&snapshot) && !snapshot.frame_fresh);
    s_data.last_update_ms = UINT32_MAX - 10u;
    laser_accept_frame(distances, UINT32_MAX - 10u);
    g_test_tick_ms = 20u;
    assert(ibus_is_online(100u));
    assert(laser_get_snapshot(&snapshot) && snapshot.frame_fresh);
    puts("INPUT_FRESHNESS_TEST_OK RX preemption, IRQ restoration, real timeouts and tick wrap");
    return 0;
}
