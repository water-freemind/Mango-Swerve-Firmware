#include "chassis_trace.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(e) do{if(!(e)){fprintf(stderr,"FAIL %s:%d %s\n",__FILE__,__LINE__,#e);exit(1);}}while(0)
static unsigned columns(const char* s){unsigned n=1;for(;*s;++s)if(*s==',')++n;return n;}
int main(void) {
    ChassisTrace t;ChassisTraceSample s={0},got;
    chassis_trace_init(&t);
    CHECK(!chassis_trace_get(&t,1,&got));
    s.t_ms=100;s.mode=1;s.permitted=1;s.drive_valid_mask=s.steer_valid_mask=15;
    CHECK(chassis_trace_push(&t,&s));
    s.t_ms=119;CHECK(!chassis_trace_push(&t,&s));
    s.t_ms=120;CHECK(chassis_trace_push(&t,&s));
    s.t_ms=121;s.mode=2;CHECK(chassis_trace_push(&t,&s));
    s.fault=7;CHECK(chassis_trace_push(&t,&s)); /* same-time safety event retained */
    s.t_ms=122;s.permitted=0;CHECK(chassis_trace_push(&t,&s));
    s.t_ms=123;s.drive_valid_mask=7;CHECK(chassis_trace_push(&t,&s));
    CHECK(t.count==6 && t.overwritten==0);
    CHECK(chassis_trace_get(&t,3,&got) && got.mode==2 && got.fault==0);
    CHECK(chassis_trace_get(&t,4,&got) && got.fault==7);
    for(unsigned i=0;i<CHASSIS_TRACE_CAPACITY+72u;++i){s.t_ms+=20;CHECK(chassis_trace_push(&t,&s));}
    CHECK(t.count==CHASSIS_TRACE_CAPACITY && t.overwritten==78);
    CHECK(!chassis_trace_get(&t,1,&got));
    CHECK(!chassis_trace_get(&t,t.next_sequence,&got));
    CHECK(chassis_trace_get(&t,chassis_trace_oldest(&t),&got));
    CHECK(got.sequence==79);
    /* Modulo sequence and uint32 millisecond rollover. */
    chassis_trace_init(&t);t.next_sequence=UINT32_MAX;
    s.t_ms=UINT32_MAX-10;CHECK(chassis_trace_push(&t,&s));
    s.t_ms=5;CHECK(!chassis_trace_push(&t,&s));
    s.t_ms=10;CHECK(chassis_trace_push(&t,&s));
    CHECK(t.next_sequence==1 && chassis_trace_oldest(&t)==UINT32_MAX);
    CHECK(chassis_trace_get(&t,0,&got) && got.t_ms==10);
    CHECK(chassis_trace_get(&t,UINT32_MAX,&got));
    char row[CHASSIS_TRACE_CSV_CAPACITY];
    SteerWheelModel m={.length=.26572986916f,.width=.26572986916f,.wheel_radius=.051f,.max_wheel_linear_speed=1.5f};
    CHECK(chassis_trace_csv_row(&s,&m,row,sizeof(row)));
    CHECK(columns(chassis_trace_csv_header())==38 && columns(row)==38);
    CHECK(!chassis_trace_csv_row(&s,&m,row,2) && row[0]=='\0');
    CHECK(!chassis_trace_csv_row(NULL,&m,row,sizeof(row)) && row[0]=='\0');
    CHECK(!chassis_trace_push(NULL,&s));
    printf("CHASSIS_TRACE_OK bytes=%u sample_bytes=%u\n",(unsigned)sizeof(t),(unsigned)sizeof(s));return 0;
}
