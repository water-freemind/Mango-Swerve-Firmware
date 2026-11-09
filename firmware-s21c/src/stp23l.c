#include "stp23l.h"
#include <string.h>
static void discard(StpParser* p) {
    --p->used;
    memmove(p->bytes,p->bytes+1,p->used);
}
void stp_init(StpParser* p) {
    memset(p,0,sizeof(*p));
    p->distance_mm=STP_INVALID_MM;
}
void stp_uart_error(StpParser* p) {
    p->used=0;
    ++p->uart_errors;
}
bool stp_feed(StpParser* p,uint8_t byte,uint32_t now) {
    if(p->used && (uint32_t)(now-p->last_byte_ms)>20u) p->used=0;
    p->last_byte_ms=now;
    p->bytes[p->used++]=byte;
    /* Sliding resynchronization retains partial headers and embedded next frames.
     * Bounded to 195 bytes; no heap and no native packed-struct casts. */
    while(p->used>=4u) {
        if(p->bytes[0]!=0xaa || p->bytes[1]!=0xaa || p->bytes[2]!=0xaa || p->bytes[3]!=0xaa) {
            discard(p); continue;
        }
        if(p->used<10u) return false;
        if(p->bytes[4]!=0 || p->bytes[5]!=2 || p->bytes[6]!=0 || p->bytes[7]!=0 ||
           p->bytes[8]!=184u || p->bytes[9]!=0) {
            ++p->header_errors; discard(p); continue;
        }
        if(p->used<STP_FRAME_BYTES) return false;
        uint8_t sum=0;
        for(unsigned i=4;i<194u;++i) sum=(uint8_t)(sum+p->bytes[i]);
        if(sum!=p->bytes[194]) {
            ++p->checksum_errors; discard(p); continue;
        }
        uint32_t stamp=0;
        for(unsigned i=0;i<4;++i) stamp|=(uint32_t)p->bytes[190u+i]<<(8u*i);
        ++p->good_frames;
        /* Even a checksum-valid candidate can be a shifted frame with an 8-bit
         * sum collision. Retain trailing AA as possible next-header bytes. */
        p->used=0;
        while(p->used<3u && p->bytes[194u-p->used]==0xaau) ++p->used;
        memset(p->bytes,0xaa,p->used);
        if(p->seen_timestamp && stamp==p->sensor_timestamp) {
            ++p->duplicate_frames;
            return false; /* A replay must not renew measurement freshness. */
        }
        p->seen_timestamp=true;
        p->sensor_timestamp=stamp;
        uint32_t total=0, count=0;
        for(unsigned i=0;i<12u;++i) {
            const unsigned at=10u+15u*i;
            const uint16_t raw=(uint16_t)(p->bytes[at]|((uint16_t)p->bytes[at+1u]<<8));
            /* int16_t protocol: exclude zero, negative/error encodings and transport outliers. */
            if(raw>0 && raw<=STP_MAX_MM) {total+=raw;++count;}
        }
        if(!count) {
            p->valid=false;
            ++p->empty_frames;
            return false;
        }
        p->distance_mm=(uint16_t)(total/count);
        p->last_valid_ms=now;
        p->valid=true;
        return true;
    }
    return false;
}
uint16_t stp_distance(const StpParser* p,uint32_t now) {
    return p->valid && (uint32_t)(now-p->last_valid_ms)<=STP_TIMEOUT_MS ? p->distance_mm : STP_INVALID_MM;
}
void stp_can_payload(const uint16_t mm[4],uint8_t out[8]) {
    for(unsigned i=0;i<4u;++i) {out[2u*i]=(uint8_t)(mm[i]>>8);out[2u*i+1u]=(uint8_t)mm[i];}
}
void stp_usb_payload(const uint8_t can[8],uint8_t out[11]) {
    out[0]=0x7b;
    memcpy(out+1,can,8);
    uint8_t sum=out[0];
    for(unsigned i=1;i<9u;++i) sum^=out[i];
    out[9]=sum;
    out[10]=0x7d;
}
