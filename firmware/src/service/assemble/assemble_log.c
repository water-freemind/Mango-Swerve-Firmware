#include "assemble.h"

#include "log.h"
#include "stm32_hal_uart.h"
#include "app_config.h"

#if APP_RUN_MODE == APP_MODE_STRAIGHT_TEST
#include "delay.h"
#include <stddef.h>

/* The wireless serial path has exhibited 120-byte truncated bursts. Keep
 * each write smaller and leave an idle gap, including between log records. */
static bool straight_log_write(const char* data, uint32_t len) {
    if(data == NULL || len == 0u) return false;
    while(len != 0u) {
        const uint32_t chunk = len > 64u ? 64u : len;
        if(!uart1_write_blocking(data, chunk)) return false;
        data += chunk;
        len -= chunk;
        delay_us(1000u);
    }
    return true;
}
#endif

// ! ========================= 变 量 声 明 ========================= ! //

static const LogPortOps log_ops = {
#if APP_RUN_MODE == APP_MODE_STRAIGHT_TEST
    .write = straight_log_write,
#else
    .write = uart1_write_blocking,
#endif
};

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_log(void) {
    LogConfig log_config = {
        .ops = &log_ops,
        .level = LOG_LEVEL_INFO,
        .enable_color = true,
        .async_write = false,
    };

    if(log_init(&log_config) != LOG_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }

    return SYSTEM_STATUS_OK;
}
