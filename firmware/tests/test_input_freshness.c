/* Inject RX immediately after reading HAL time in the actual input drivers. */
typedef struct { unsigned unused; } UART_HandleTypeDef;
#include "../src/device/fs_ia10b.c"
#include "laser.h"
#include <assert.h>
#include <stdio.h>

uint32_t g_test_tick_ms, g_test_irq_mask;
UART_HandleTypeDef huart1, huart5;
static bool inject_rx;
static bool receive_ok = true;
static unsigned receive_calls, abort_calls;
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
    (void)h; (void)p; (void)n; ++receive_calls; return receive_ok;
}
bool uart_abort_receive_it(UART_HandleTypeDef* h) { (void)h; ++abort_calls; return true; }
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
    inject_rx = false; g_test_irq_mask = 0u; g_test_tick_ms = 0u;
    ibus_init(); ibus_maintain();
    assert(s_rx_active && receive_calls == 1u && abort_calls == 0u);
    s_frame_index = 15u; /* An outstanding HAL read can silently stop completing. */
    g_test_tick_ms = 199u; ibus_maintain();
    assert(receive_calls == 1u && s_frame_index == 15u);
    g_test_tick_ms = 200u; ibus_maintain();
    assert(s_rx_active && receive_calls == 2u && abort_calls == 1u && s_frame_index == 0u);
    for(unsigned i = 0u; i < 20u; ++i) {
        g_test_tick_ms += 10u; s_rx_byte = 0u;
        ibus_rx_complete_callback(); ibus_maintain();
    }
    assert(abort_calls == 1u); /* Active byte reception must not be aborted. */
    ibus_error_callback(); ibus_maintain();
    assert(!s_rx_active && s_ibus_restart_pending);
    g_test_tick_ms += 10u; ibus_maintain();
    assert(s_rx_active && !s_ibus_restart_pending && abort_calls == 2u);
    receive_ok = false;
    ++g_test_tick_ms; s_rx_byte = 0u; ibus_rx_complete_callback();
    const unsigned before_retry = receive_calls;
    g_test_tick_ms += 9u; ibus_maintain();
    assert(receive_calls == before_retry);
    ++g_test_tick_ms; ibus_maintain();
    assert(receive_calls == before_retry + 1u && s_ibus_restart_pending);
    receive_ok = true; g_test_tick_ms += 10u; g_test_irq_mask = 1u;
    ibus_maintain();
    assert(s_rx_active && !s_ibus_restart_pending && g_test_irq_mask == 1u);
    g_test_irq_mask = 0u;
    uint8_t frame[FS_IA10B_IBUS_FRAME_LEN] = {0x20u, 0x40u};
    for(unsigned i = 0u; i < FS_IA10B_CHANNEL_COUNT; ++i) {
        frame[2u + i * 2u] = (uint8_t)1500u;
        frame[3u + i * 2u] = (uint8_t)(1500u >> 8u);
    }
    uint16_t sum = 0xffffu;
    for(unsigned i = 0u; i < 30u; ++i) sum = (uint16_t)(sum - frame[i]);
    frame[30] = (uint8_t)sum; frame[31] = (uint8_t)(sum >> 8u);
    for(unsigned i = 0u; i < sizeof(frame); ++i) {
        ++g_test_tick_ms; s_rx_byte = frame[i]; ibus_rx_complete_callback();
    }
    assert(ibus_is_online(100u) && s_data.frame_count == 1u);
    g_test_tick_ms = UINT32_MAX - 100u; ibus_init(); ibus_maintain();
    const unsigned before_wrap = receive_calls;
    g_test_tick_ms += 199u; ibus_maintain();
    assert(receive_calls == before_wrap);
    ++g_test_tick_ms; ibus_maintain();
    assert(receive_calls == before_wrap + 1u && s_rx_active);
    puts("INPUT_FRESHNESS_TEST_OK RX preemption, IRQ restoration, real timeouts and tick wrap");
    puts("IBUS_RECOVERY_TEST_OK silent stalled read, active stream, UART error, bounded retry and tick wrap");
    return 0;
}
