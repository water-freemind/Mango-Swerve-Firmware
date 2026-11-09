#ifndef S21_STP23L_H
#define S21_STP23L_H
#include <stdint.h>
#include <stdbool.h>
#define STP_FRAME_BYTES 195u
#define STP_TIMEOUT_MS 300u
#define STP_INVALID_MM UINT16_MAX
#define STP_MAX_MM 15000u /* Transport screen matches DM; not a physical range guarantee. */
typedef struct {
    uint8_t bytes[STP_FRAME_BYTES];
    uint16_t used, distance_mm;
    uint32_t last_byte_ms, last_valid_ms, sensor_timestamp;
    uint32_t good_frames, checksum_errors, header_errors, empty_frames, duplicate_frames, uart_errors;
    bool valid, seen_timestamp;
} StpParser;
void stp_init(StpParser* p);
bool stp_feed(StpParser* p, uint8_t byte, uint32_t now_ms);
void stp_uart_error(StpParser* p);
uint16_t stp_distance(const StpParser* p, uint32_t now_ms);
/* Snapshot values already selected by stp_distance; original CAN big-endian order. */
void stp_can_payload(const uint16_t mm[4], uint8_t out[8]);
void stp_usb_payload(const uint8_t can[8], uint8_t out[11]);
#endif
