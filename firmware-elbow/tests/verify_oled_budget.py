"""Count the real I2C driver delays with a successful simulated OLED bus."""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / "BSP/OLED/OLED.c").read_bytes().decode("latin1")
delay_us = int(re.search(r"#define OLED_I2C_DELAY_US\s+(\d+)", source)[1])


def function(signature):
    start = source.index(signature)
    brace = source.index("{", start)
    end, depth = brace + 1, 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


prefix = r'''
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
typedef int GPIO_PinState;
#define OLED_SCL_PORT 0
#define OLED_SDA_PORT 0
#define OLED_SCL_PIN 1
#define OLED_SDA_PIN 2
static uint32_t elapsed_us, bytes;
static int scl=1,sda=1,active,clocks;
uint8_t OLED_DisplayBuf[8][128];
volatile uint32_t OLED_I2C_NackCount,OLED_I2C_LastError;
static uint8_t OLED_TransactionFailed;
static void OLED_I2C_Delay(void) { elapsed_us+=DELAY_US; }
void HAL_GPIO_WritePin(int port,int pin,int value) {
    (void)port;
    if(pin==1) {
        if(!scl && value && active) {
            if(++clocks==9) ++bytes;
            else if(clocks>9) clocks=1;
        }
        scl=value;
    } else {
        if(scl && sda!=value) {
            active=!value;
            if(active) clocks=0;
        }
        sda=value;
    }
}
int HAL_GPIO_ReadPin(int port,int pin) {
    (void)port;
    return pin==1?scl:(active && clocks==9?0:sda);
}
'''.replace("DELAY_US", str(delay_us))

signatures = ["void OLED_W_SCL(", "void OLED_W_SDA(", "void OLED_I2C_Start(",
              "void OLED_I2C_Stop(", "void OLED_I2C_SendByte(",
              "static void OLED_WriteCommands(", "void OLED_WriteCommand(",
              "void OLED_WriteData(", "void OLED_SetCursor(",
              "void OLED_Update(", "void OLED_UpdateArea("]
body = "\n".join(function(name) for name in signatures)
checks = r'''
int main(void) {
    uint32_t max_page=0;
    for(unsigned page=0;page<8;page++) {
        elapsed_us=0;
        OLED_UpdateArea(0,(int16_t)(page*8),128,8);
        assert(!OLED_I2C_LastError && elapsed_us<=20000u);
        if(elapsed_us>max_page) max_page=elapsed_us;
    }
    elapsed_us=0;
    OLED_Update();
    assert(!OLED_I2C_LastError);
    printf("OLED_I2C_DELAY_BUDGET_OK page_us=%u full_us=%u GPIO_CPU_overhead_excluded=1\n",
        max_page,elapsed_us);
}
'''
with tempfile.TemporaryDirectory(prefix="oled_budget_", dir=root / "tests") as directory:
    c = Path(directory) / "budget.c"
    exe = Path(directory) / "budget.exe"
    c.write_text(prefix + body + checks, encoding="utf-8")
    subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                    str(c), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
