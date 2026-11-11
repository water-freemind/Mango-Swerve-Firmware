"""Verify production UART callbacks, parser, GPIO directions and action timing."""
from pathlib import Path
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

with tempfile.TemporaryDirectory(prefix="vision_motor_", dir=root / "tests") as directory:
    work = Path(directory)
    callback_file = work / "uart_callbacks.c"
    callback_file.write_text('#include "vision_motor.h"\n' + callbacks, encoding="utf-8")
    executable = work / "verify.exe"
    subprocess.run([
        "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-I" + str(root / "tests/work_fakes"), "-I" + str(root / "BSP"),
        "-I" + str(chassis / "tests/fakes"),
        str(root / "BSP/vision_motor.c"), str(root / "tests/test_vision_motor.c"),
        str(callback_file), "-o", str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True)
