#include "stp23l.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef S21_TEST_DM
#include "laser.h"
uint32_t g_test_tick_ms;
#endif
#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static void seal(uint8_t b[195]) {
    uint8_t sum=0;for(unsigned i=4;i<194;++i)sum=(uint8_t)(sum+b[i]);b[194]=sum;
}
static void frame(uint8_t b[195],uint16_t mm,uint32_t stamp) {
    memset(b,0,195);memset(b,0xaa,4);b[5]=2;b[8]=184;
    for(unsigned i=0;i<12;++i){b[10+15*i]=(uint8_t)mm;b[11+15*i]=(uint8_t)(mm>>8);}
    for(unsigned i=0;i<4;++i)b[190+i]=(uint8_t)(stamp>>(8*i));
    seal(b);
}
static unsigned feed(StpParser* p,const uint8_t* b,unsigned n,uint32_t now) {
    unsigned accepted=0;for(unsigned i=0;i<n;++i)accepted+=stp_feed(p,b[i],now);return accepted;
}
int main(void) {
    StpParser p;uint8_t b[195];stp_init(&p);
    CHECK(stp_distance(&p,0)==65535);
    frame(b,7500,0x44332211);CHECK(feed(&p,b,194,10)==0);CHECK(!p.valid);
    CHECK(stp_feed(&p,b[194],10));CHECK(p.sensor_timestamp==0x44332211 && p.distance_mm==7500);
    CHECK(stp_distance(&p,310)==7500 && stp_distance(&p,311)==65535);
    frame(b,109,2);b[194]^=1;CHECK(feed(&p,b,195,20)==0);
    CHECK(p.distance_mm==7500 && p.last_valid_ms==10 && p.checksum_errors==1);
    frame(b,1000,3);b[10]=100;b[11]=0;seal(b);
    CHECK(feed(&p,b,195,30)==1 && p.distance_mm==925); /* mean, not first 100 */
    CHECK(feed(&p,b,195,200)==0 && p.last_valid_ms==30 && p.duplicate_frames==1);
    CHECK(stp_distance(&p,331)==65535);
    frame(b,0xffff,4);CHECK(feed(&p,b,195,340)==0 && !p.valid && p.empty_frames==1);
    frame(b,0,5);CHECK(feed(&p,b,195,350)==0 && !p.valid);
    frame(b,15000,6);CHECK(feed(&p,b,195,360)==1 && p.distance_mm==15000);
    for(unsigned i=0;i<500;++i){frame(b,3000,7+i);CHECK(feed(&p,b,195,370+i)==1 && p.distance_mm==3000);}
    frame(b,1200,1000);b[8]=183;seal(b);CHECK(feed(&p,b,195,900)==0);
    frame(b,1300,1001);CHECK(feed(&p,b,195,910)==1 && p.distance_mm==1300);
    CHECK(feed(&p,b,70,920)==0);frame(b,1400,1002);
    CHECK(feed(&p,b,195,920)==1 && p.distance_mm==1400); /* no idle gap, embedded next header */
    frame(b,1500,1003);CHECK(feed(&p,b,50,930)==0);stp_uart_error(&p);
    CHECK(p.used==0 && p.uart_errors==1);
    CHECK(feed(&p,b,195,940)==1 && p.distance_mm==1500);
    frame(b,1600,1004);CHECK(feed(&p,b,30,950)==0);
    CHECK(feed(&p,b+30,165,980)==0);CHECK(feed(&p,b,195,990)==1);
    frame(b,1700,1005);CHECK(feed(&p,b,195,UINT32_MAX-50u)==1);
    CHECK(stp_distance(&p,249)==1700 && stp_distance(&p,250)==65535);
    StpParser ports[4];uint16_t mm[4];uint8_t can[8];
    for(unsigned i=0;i<4;++i){stp_init(&ports[i]);frame(b,(uint16_t)(100+i*100),1);CHECK(feed(&ports[i],b,195,100)==1);}
    for(unsigned i=1;i<4;++i){frame(b,(uint16_t)(100+i*100),2);CHECK(feed(&ports[i],b,195,401)==1);}
    for(unsigned i=0;i<4;++i)mm[i]=stp_distance(&ports[i],401);
    stp_can_payload(mm,can);CHECK(can[0]==255 && can[1]==255 && can[2]==0 && can[3]==200 && can[6]==1 && can[7]==144);
    uint8_t usb[11];stp_usb_payload(can,usb);
    CHECK(usb[0]==0x7b && usb[10]==0x7d && !memcmp(usb+1,can,8));
    uint8_t xor_sum=0;for(unsigned i=0;i<10u;++i)xor_sum^=usb[i];
    CHECK(xor_sum==0);
    /* All eight CAN bytes, including header/tail-looking values, are binary data. */
    const uint16_t edges[4]={0x007b,0x7d00,0xffff,0x1234};
    const uint8_t expected[8]={0,0x7b,0x7d,0,0xff,0xff,0x12,0x34};
    stp_can_payload(edges,can);CHECK(!memcmp(can,expected,8));
    stp_usb_payload(can,usb);CHECK(!memcmp(usb+1,expected,8) && usb[9]==(uint8_t)(0x7b^0x7b^0x7d^0x12^0x34));
    /* Single-bit corruption at every position must not publish a measurement. */
    for(unsigned at=0;at<195u*8u;++at) {
        StpParser damaged;stp_init(&damaged);frame(b,640,0x12345678);
        b[at/8u]^=(uint8_t)(1u<<(at%8u));CHECK(feed(&damaged,b,195,50)==0 && !damaged.valid);
        frame(b,650,0x12345679);CHECK(feed(&damaged,b,195,60)==1 && damaged.distance_mm==650);
    }
    unsigned drop_collisions=0;
    for(unsigned at=0;at<195u;++at) {
        StpParser dropped;stp_init(&dropped);frame(b,660,0x12345678);
        for(unsigned j=0;j<195u;++j)if(j!=at)(void)stp_feed(&dropped,b[j],50);
        frame(b,670,0x12345679);
        const unsigned recovered=feed(&dropped,b,195,60);
        /* Deleted-byte corruption may collide in the protocol's weak sum.
         * Verify recovery without pretending every corruption is detectable. */
        CHECK(recovered>=1 && dropped.distance_mm==670);
        drop_collisions+=recovered>1u;
    }
    printf("S21C_PROTOCOL_LIMIT deletion fixtures with checksum collision=%u (not a measured probability)\n",drop_collisions);
    for(unsigned at=0;at<=195u;++at) {
        StpParser inserted;stp_init(&inserted);frame(b,660,0x12345678);
        for(unsigned j=0;j<=195u;++j) {
            if(j==at)(void)stp_feed(&inserted,0x55,50);
            if(j<195u)(void)stp_feed(&inserted,b[j],50);
        }
        frame(b,670,0x12345679);
        CHECK(feed(&inserted,b,195,60)>=1 && inserted.distance_mm==670);
    }
    /* Mixed samples: exclude negative/zero/out-of-screen values before averaging. */
    stp_init(&p);frame(b,0xffff,1);b[10]=100;b[11]=0;b[25]=200;b[26]=0;
    b[40]=0;b[41]=0;b[55]=0x99;b[56]=0x3a;seal(b);
    CHECK(feed(&p,b,195,100)==1 && p.distance_mm==150);
    uint32_t fuzz=0x12345678;
    stp_init(&p);
    for(unsigned i=0;i<100000u;++i) {
        fuzz=fuzz*1664525u+1013904223u;
        (void)stp_feed(&p,(uint8_t)(fuzz>>24),i/23u);
        CHECK(p.used<STP_FRAME_BYTES);
    }
    frame(b,680,2);CHECK(feed(&p,b,195,5000)==1 && p.distance_mm==680);
    stp_init(&p);frame(b,690,0x00aaaaaa);
    b[18]=(uint8_t)(b[18]+0xaau-b[194]);seal(b);CHECK(b[194]==0xaa);
    CHECK(feed(&p,b,195,5010)==1 && p.distance_mm==690);
    frame(b,700,0x00aaaaab);CHECK(feed(&p,b,195,5020)==1 && p.distance_mm==700);
    /* Restore the earlier four-port timeout packet for the DM integration below. */
    stp_can_payload(mm,can);
    frame(b,50,2);CHECK(feed(&ports[0],b,195,402)==1 && stp_distance(&ports[0],402)==50);
#ifdef S21_TEST_DM
    /* Exercise the actual current DM receiver with unchanged 0x601 bytes. */
    LaserSnapshot snapshot;
    uint16_t initial[4]={100,200,300,400};
    stp_can_payload(initial,can);laser_accept_frame(can,100);
    stp_can_payload(mm,can);laser_accept_frame(can,401);
    CHECK(laser_get_snapshot_at(&snapshot,401));
    CHECK(snapshot.frame_fresh && snapshot.valid_mask==14 && snapshot.distance_mm[0]==65535);
    CHECK(snapshot.accepted_mm[1]==200 && snapshot.accepted_mm[3]==400);
    mm[0]=50;stp_can_payload(mm,can);laser_accept_frame(can,402);
    CHECK(laser_get_snapshot_at(&snapshot,402) && snapshot.valid_mask==15 && snapshot.accepted_mm[0]==50);
    puts("S21C_DM_COMPAT_OK startup/single-channel loss/reconnect using actual DM laser.c");
#endif
    puts("S21C_PARSER_TEST_OK checksum, timestamp, mean, signed input, bounds, reset, replay, timeout, resync, four ports and CAN");
    return 0;
}
