"""Exercise the production OLED display with simulated chassis replies."""
from pathlib import Path
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[1]
chassis = root.parent / "firmware"
if not (chassis / "src/service").is_dir():
    chassis = root.parents[2] / "Mango-Swerve-Firmware/firmware"
source = (root / "BSP/fun.c").read_bytes().decode("latin1")
start = source.index("void OLED_Display(void)")
brace = source.index("{", start)
end, depth = brace + 1, 1
while depth:
    depth += (source[end] == "{") - (source[end] == "}")
    end += 1
display = source[start:end]

harness = r'''
#include "vision_motor.h"
#include "work_can.h"
#include "can_test.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#define OLED_6X8 6
char str[32], rows[8][32];
volatile int32_t Speed[4];
volatile uint8_t Vision_Online, Vision_Region, Vision_MotorPhase, Vision_MotorRegion;
volatile uint8_t Vision_WorkState, Vision_WorkResult, CAN_Test_State;
volatile uint32_t Vision_WorkTask, OLED_I2C_LastError;
volatile int32_t Vision_TreeDX = 9999;
volatile uint8_t OLED_Ready = 1;
static uint32_t tick;
static uint8_t online, reply, moving, result, fail_update;
static unsigned page_transfers, page_mask, init_calls;
static uint8_t laser_status[4];
uint8_t CAN_Laser_Read(uint8_t channel, uint16_t *mm) {
    *mm = 15000;
    return laser_status[channel];
}
uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return 0; }
void __disable_irq(void) {}
void __set_PRIMASK(uint32_t value) { (void)value; }
uint8_t Work_CAN_Online(void) { return online; }
uint8_t Work_CAN_GetReply(uint32_t task) { (void)task; return online ? reply : 0; }
uint8_t Work_CAN_Moving(void) { return online && moving; }
uint8_t Work_CAN_Result(void) { return result; }
uint32_t Vision_Motor_RemainingMs(void) { return 600; }
void OLED_Init(void) { ++init_calls; OLED_Ready = 1; }
void OLED_Clear(void) { memset(rows, 0, sizeof(rows)); }
void OLED_UpdateArea(int16_t x, int16_t y, uint8_t width, uint8_t height) {
    assert(x == 0 && y >= 0 && y <= 56 && y % 8 == 0 && width == 128 && height == 8);
    ++page_transfers;
    page_mask |= 1u << (y / 8);
    if (fail_update) OLED_I2C_LastError = 1;
}
void OLED_ShowString(int x, int y, char *text, int font) {
    assert(x == 0 && y >= 0 && y < 64 && font == 6);
    assert(strlen(text) <= 21);
    strcpy(rows[y / 8], text);
}
'''

checks = r'''
static void check_car(uint8_t state, const char *expected) {
    reply = state;
    tick += 100;
    OLED_Display();
    if (ELBOW_TEST_MODE == 1) {
        assert(strcmp(rows[2], "MODE:VISION TEST") == 0);
    } else {
        assert(strstr(rows[2], expected));
    }
}
int main(void) {
    unsigned before, i;
    tick = 1000;
    OLED_Display();
    assert(strcmp(rows[2], ELBOW_TEST_MODE == 1 ? "MODE:VISION TEST" : "CAR:OFFLINE CAN:OFF") == 0);
    before = page_transfers;
    for (i = 0; i < 8; ++i) { ++tick; OLED_Display(); }
    assert(page_transfers == before + 8 && page_mask == 255u);
    fail_update = 1;
    ++tick;
    OLED_Display();
    assert(!OLED_Ready);
    fail_update = 0;
    before = init_calls;
    Vision_MotorPhase = 1;
    tick += 1000;
    OLED_Display();
    assert(!OLED_Ready && init_calls == before);
    Vision_MotorPhase = 0;
    Vision_WorkState = VISION_WORK_CHECK;
    tick += 1000;
    OLED_Display();
    assert(!OLED_Ready && init_calls == before);
    Vision_WorkState = VISION_WORK_BOOT;
    tick += 1000;
    OLED_Display();
    assert(OLED_Ready && init_calls == before + 1);
    puts("OLED_PAGED_PROGRESS_AND_ACTION_RECOVERY_GATE_OK");
    Vision_WorkState = VISION_WORK_TEST;
    tick += 100;
    OLED_Display();
    assert(strstr(rows[3], "W:TEST_WAIT"));
    online = 1;
    CAN_Test_State = 1;
    if (ELBOW_TEST_MODE == 2) {
        check_car(STATION_SYNC, "CAR:SYNC");
        check_car(STATION_APPROACH, "CAR:APPROACH");
        check_car(STATION_START, "CAR:STOPPED");
        check_car(STATION_RELEASE, "CAR:RELEASED");
        check_car(STATION_ABORT, "CAR:ABORT");
        Vision_Online=0;
        Vision_WorkState=VISION_WORK_IDLE;
        check_car(STATION_SYNC, "CAR:SYNC");
        assert(strstr(rows[3], "W:WAIT_CAM"));
        laser_status[0]=1;
        laser_status[1]=2;
        laser_status[2]=3;
        laser_status[3]=1;
        tick+=100;
        OLED_Display();
        assert(strcmp(rows[4], "LEFT:15000 mm")==0);
        assert(strcmp(rows[5], "RIGHT:INVALID")==0);
        assert(strcmp(rows[6], "REAR:OFFLINE")==0);
        assert(strcmp(rows[7], "FRONT:15000 mm")==0);
        puts("OLED_STATION_CAMERA_OFF_LASER_ROWS_OK");
        puts("OLED_STATION_PROTOCOL_LABELS_OK");
        return 0;
    }
    check_car(WORK_SYNC, "CAR:SYNC");
    check_car(WORK_READY, "CAR:READY");
    moving = 1;
    check_car(WORK_READY, "CAR:MOVING");
    check_car(WORK_SLOW_ACK, "CAR:SLOW");
    check_car(WORK_STOPPING, "CAR:STOPPING");
    check_car(WORK_STOPPED, "CAR:STOPPED");
    moving = 0;
    check_car(WORK_GO_ACK, "CAR:RELEASED");
    moving = 1;
    check_car(WORK_GO_ACK, "CAR:MOVING");
    result = WORK_RESULT_FEEDBACK;
    check_car(WORK_FAULT, "CAR:FAULT");
    assert(strstr(rows[3], "E:7"));
    for (Vision_WorkState = VISION_WORK_BOOT;
         Vision_WorkState <= VISION_WORK_FAULT; Vision_WorkState++) {
        Vision_WorkResult = WORK_RESULT_SKIPPED;
        check_car(WORK_READY, "CAR:MOVING");
        assert(strstr(rows[3], "E:10"));
    }
    online = 0;
    check_car(WORK_STOPPED, "CAR:OFFLINE CAN:OFF");
    assert(strcmp(rows[4], "LEFT:WAIT") == 0);
    assert(strcmp(rows[7], "FRONT:WAIT") == 0);
    laser_status[0] = 1;
    laser_status[1] = 2;
    laser_status[2] = 3;
    laser_status[3] = 1;
    tick += 100;
    OLED_Display();
    assert(strcmp(rows[4], "LEFT:15000 mm") == 0);
    assert(strcmp(rows[5], "RIGHT:INVALID") == 0);
    assert(strcmp(rows[6], "REAR:OFFLINE") == 0);
    assert(strcmp(rows[7], "FRONT:15000 mm") == 0);
    fail_update = 1;
    tick += 100;
    OLED_Display();
    assert(!OLED_Ready);
    fail_update = 0;
    tick += 1000;
    OLED_Display();
    assert(OLED_Ready);
    puts("OLED_CHASSIS_STATES_OFFLINE_WIDTH_RECOVERY_OK");
}
'''

with tempfile.TemporaryDirectory(prefix="oled_", dir=root / "tests") as directory:
    work = Path(directory)
    test_source = work / "oled_test.c"
    test_source.write_text(harness + display + checks, encoding="utf-8")
    executable = work / "verify.exe"
    for mode in (0, 1, 2):
        subprocess.run([
            "gcc", "-DELBOW_TEST_MODE=" + str(mode), "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-I" + str(root / "tests/work_fakes"), "-I" + str(root / "BSP"),
            "-I" + str(chassis / "tests/fakes"), str(test_source),
            "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True)

subprocess.run([sys.executable, str(root / "tests/verify_oled_budget.py")], check=True)
