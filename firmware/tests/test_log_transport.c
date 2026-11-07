/* Exercise the production log port against a receiver limited to 120-byte
 * bursts, matching observed truncation. No UART hardware is opened. */
typedef struct { unsigned unused; } UART_HandleTypeDef;
#include "delay.h"
#include "../src/service/assemble/assemble_log.c"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static LogConfig captured;
static char received[1024];
static uint32_t received_len, burst_len, write_count, max_write, gap_count, fail_at;

LogStatus log_init(const LogConfig* config) { captured = *config; return LOG_STATUS_OK; }
bool uart1_write_blocking(const char* data, uint32_t len) {
    ++write_count;
    if(fail_at == write_count) return false;
    if(len > max_write) max_write = len;
    uint32_t accepted = burst_len < 120u ? 120u - burst_len : 0u;
    if(accepted > len) accepted = len;
    memcpy(received + received_len, data, accepted);
    received_len += accepted;
    burst_len += len;
    return true;
}
void delay_us(us_t us) {
    assert(us == 1000u);
    burst_len = 0u;
    ++gap_count;
}

int main(void) {
    assert(assemble_log() == SYSTEM_STATUS_OK);
    assert(!captured.async_write && captured.enable_color);
#if APP_RUN_MODE == APP_MODE_STRAIGHT_TEST
    char message[255];
    for(unsigned i = 0; i < sizeof(message); ++i) message[i] = (char)('A' + i % 26u);
    assert(!captured.ops->write(NULL, 1u));
    assert(!captured.ops->write(message, 0u));
    assert(captured.ops->write(message, sizeof(message)));
    assert(captured.ops->write(message, sizeof(message)));
    assert(received_len == 2u * sizeof(message));
    assert(!memcmp(received, message, sizeof(message)));
    assert(!memcmp(received + sizeof(message), message, sizeof(message)));
    assert(max_write <= 64u && gap_count == write_count);
    fail_at = write_count + 2u;
    assert(!captured.ops->write(message, sizeof(message)));
    puts("LOG_TRANSPORT_TEST_OK (mode 6, 120-byte burst receiver, complete records, idle gaps, UART failure)");
#else
    assert(captured.ops->write == uart1_write_blocking);
    assert(write_count == 0u && gap_count == 0u);
    puts("LOG_TRANSPORT_LEGACY_TEST_OK (normal mode keeps original UART writer)");
#endif
    return 0;
}
