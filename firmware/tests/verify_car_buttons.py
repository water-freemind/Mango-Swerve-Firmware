"""Run the production camera dispatcher and elbow UART status with fake hardware."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / "src/app/straight_test_app.c").read_text(encoding="utf-8")
receive = source[source.index("static bool car_receive("):source.index("static ChassisTrace trace;")]
dispatch = source[source.index("static bool map_drive_selected("):source.index("static void commands(")]
stub = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "car_button_protocol.h"
enum { ST_ARMED=1,ST_RUNNING,ST_STOPPING,DRIVE_SOURCE_HOST=4,
       MAP_IDLE=0,MAP_READY,MAP_ARMED,MAP_RUNNING,STATION_LINK_APPROACH=10,
       STATION_LINK_ALIGN,STATION_LINK_WORKING,STATION_LINK_RELEASE,CAN3_BUS_OK=0 };
#define WORK_STATION_ENABLED 1
typedef struct { bool stationary,healthy; } StraightInput;
typedef struct { bool work_enabled; } FullMapInput;
typedef struct { uint8_t len,data[8]; uint32_t rx_ms; } Can3Frame;
typedef struct { int unused; } DriveVelocity;
static struct { int phase; } test;
static struct { int mode; bool manual_armed; } control_source;
static struct { bool hold; int phase; } g_station_work;
static struct { int phase; uint32_t token; const char* reason; } full_map;
static bool map_work_enabled,car_pending,car_reply_pending,reject_arm,reject_origin,reject_go;
static bool work_ready=true, hardware_ready=true, started_work, camera_map_owner;
static uint8_t car_inbox[8],car_reply[8];
static uint32_t car_rx_ms,car_arm_token;
static uint16_t car_last_seq;
static DriveVelocity manual_velocity;
static unsigned origin_calls,arm_calls,go_calls,stop_calls,output_stops;
static unsigned __get_PRIMASK(void) { return 0; }
static void __disable_irq(void) {}
static void __set_PRIMASK(unsigned mask) { (void)mask; }
static void full_map_stop(void* m,const char* why,bool lose) {
    (void)m;(void)lose;full_map.phase=MAP_IDLE;full_map.reason=why;stop_calls++;
}
static void drive_source_stop_manual(void* p) { (void)p; }
static void chassis_yaw_hold_disable(void) {}
static void stop_outputs(void) { output_stops++; }
static bool straight_command(void* t,const char* c,uint32_t n,StraightInput i) {
    (void)t;(void)c;(void)n;(void)i;return true;
}
static bool full_map_active(void* m) { (void)m;return full_map.phase==MAP_RUNNING || full_map.phase==MAP_ARMED; }
static bool station_work_reset(bool stopped) { return stopped; }
static bool station_work_ready(void) { return work_ready; }
static bool hardware_healthy(void) { return hardware_ready; }
static FullMapInput map_input(uint32_t now,StraightInput i) {
    (void)now;(void)i;return (FullMapInput){map_work_enabled};
}
static bool full_map_command(void* m,const char* c,FullMapInput i) {
    (void)m;
    if(!strcmp(c,"MAP ORIGIN")) {
        origin_calls++;
        if(reject_origin) { full_map.reason="ORIGIN_FRONT_NOT_READY";return false; }
        full_map.phase=MAP_READY;return true;
    }
    if(!strcmp(c,"MAP ARM ALL")) {
        arm_calls++;if(reject_arm) { full_map.reason="WORK_PEER_NOT_READY";return false; }
        full_map.token=42;full_map.phase=MAP_ARMED;return true;
    }
    if(!strncmp(c,"MAP GO ",7)) {
        assert(!strcmp(c,"MAP GO 42") && full_map.phase==MAP_ARMED);
        go_calls++;
        if(reject_go) return false;
        full_map.phase=MAP_RUNNING;started_work=i.work_enabled;return true;
    }
    return false;
}
static int can3_bus_send(unsigned id,const uint8_t* p,unsigned len) {
    assert(id==CAR_REPLY_ID && len==8 && car_valid(p));return CAN3_BUS_OK;
}
#define log_info(...) ((void)0)
'''
with tempfile.TemporaryDirectory(prefix="car_buttons_") as directory:
    path = Path(directory)
    unit = path / "test.c"
    unit.write_text(stub + receive + dispatch + (root / "tests/test_car_buttons.c").read_text(), encoding="utf-8")
    exe = path / "test.exe"
    subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I" + str(root / "src/service"), str(unit), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)

    elbow = (root.parent / "firmware-elbow/BSP/fun.c").read_text(encoding="utf-8")
    uart_start = elbow.index("void UART_SendStatus(void)")
    uart_end = elbow.index("\n}", uart_start) + 2
    uart_status = elbow[uart_start:uart_end]
    uart_stub = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
enum { HAL_UART_STATE_READY=0, HAL_BUSY=1, HAL_OK=0 };
static struct { int gState; } huart3;
static char uart_tx[384], transmitted[384];
static uint32_t now, UART_TxCount, UART_TxErrorCount;
static int32_t Speed[4];
static uint16_t Car_ButtonSeq, Car_AckSeq;
static uint8_t Car_ButtonOp, Car_AckOp, Car_AckResult, Car_AckMode, Car_AckPhase;
static uint8_t Vision_WorkState, Vision_WorkResult, Vision_Online, Vision_Region;
static uint32_t Vision_WorkTask, Vision_StrokeLastMs;
static int32_t Vision_TreeDX, Vision_StrokeLastCounts;
static uint8_t CAN_Test_State, OLED_Ready;
static uint32_t CAN_Test_TxCount, CAN_Test_RxCount, CAN_Test_ErrorCount;
static uint32_t OLED_I2C_LastError, OLED_I2C_NackCount;
static uint32_t HAL_GetTick(void) { return now; }
static unsigned __get_PRIMASK(void) { return 0; }
static void __disable_irq(void) {}
static void __set_PRIMASK(unsigned mask) { (void)mask; }
static int HAL_UART_Transmit_IT(void* uart, uint8_t* data, uint16_t length) {
    assert(uart==&huart3 && length<sizeof(transmitted));
    memcpy(transmitted,data,length);
    transmitted[length]=0;
    return HAL_OK;
}
'''
    uart_test = r'''
int main(void) {
    now=500u;UART_SendStatus();
    assert(UART_TxCount==1u && strstr(transmitted,"$CARLINK,0,0\r\n"));
    Car_ButtonSeq=42u;Car_ButtonOp=7u;
    now=1000u;UART_SendStatus();
    assert(UART_TxCount==2u && strstr(transmitted,"$CARLINK,42,7\r\n"));
    assert(strstr(transmitted,"$CARACK,0,0,0,0,0\r\n"));
    Car_AckSeq=42u;Car_AckOp=7u;Car_AckMode=1u;
    now=1025u;huart3.gState=HAL_BUSY;UART_SendStatus();
    assert(UART_TxCount==2u);
    huart3.gState=HAL_UART_STATE_READY;UART_SendStatus();
    assert(UART_TxCount==3u && strstr(transmitted,"$CARACK,42,7,0,1,0\r\n"));
    assert(!strstr(transmitted," RPM"));
    now=1100u;UART_SendStatus();assert(UART_TxCount==3u);
    now=1500u;UART_SendStatus();
    assert(UART_TxCount==4u && strstr(transmitted,"$CARLINK,42,7\r\n") && strstr(transmitted," RPM"));
    puts("F407_UART_LINK_HEARTBEAT_AND_PRIORITY_CHASSIS_ACK_OK");
    return 0;
}
'''
    unit.write_text(uart_stub + uart_status + "\n" + uart_test, encoding="utf-8")
    subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", str(unit), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
