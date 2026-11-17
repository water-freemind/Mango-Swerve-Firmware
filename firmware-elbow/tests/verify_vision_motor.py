"""Verify production UART callbacks, parser, GPIO directions and action timing."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
chassis = root.parent / "firmware"
if not (chassis / "src/service").is_dir():
    chassis = root.parents[2] / "Mango-Swerve-Firmware/firmware"

def production_function(name):
    source = (root / "BSP/fun.c").read_bytes().decode("latin1")
    start = source.index("void " + name + "(")
    if source[max(0, start-7):start] == "static ":
        start -= 7
    brace = source.index("{", start)
    depth, end = 1, brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]

callbacks = "\n".join(production_function(name) for name in (
    "Vision_UART_Arm", "HAL_UARTEx_RxEventCallback",
    "HAL_UART_ErrorCallback", "Vision_UART_Task"))

# A plain temp directory keeps host builds independent of the checkout ACLs.
work = Path(tempfile.gettempdir()) / ("vision_motor_" + str(os.getpid()))
work.mkdir(parents=True, exist_ok=True)
try:
    callback_file = work / "uart_callbacks.c"
    callback_file.write_text('#include "vision_motor.h"\n' + callbacks, encoding="utf-8")
    executable = work / "verify.exe"
    for mode in (0, 2, 3):
        subprocess.run([
            "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-DELBOW_TEST_MODE=" + str(mode),
            "-I" + str(root / "tests/work_fakes"), "-I" + str(root / "BSP"),
            "-I" + str(chassis / "tests/fakes"),
            str(root / "BSP/vision_motor.c"), str(root / "tests/test_vision_motor.c"),
            str(callback_file), "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True)
    # 可选行程结束检测（默认 0 = 纯定时）单独编一次，保证这段代码仍有覆盖。
    subprocess.run([
        "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-DELBOW_TEST_MODE=2",
        "-DELBOW_STROKE_END_MS=200",
        "-I" + str(root / "tests/work_fakes"), "-I" + str(root / "BSP"),
        "-I" + str(chassis / "tests/fakes"),
        str(root / "BSP/vision_motor.c"), str(root / "tests/test_vision_motor.c"),
        str(callback_file), "-o", str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True)
finally:
    shutil.rmtree(work, ignore_errors=True)