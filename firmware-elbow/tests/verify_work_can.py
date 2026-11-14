"""Compile and exercise both production CAN endpoints and the action controller."""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
chassis = root.parent / "firmware"
if not (chassis / "src/service").is_dir():
    chassis = root.parents[2] / "Mango-Swerve-Firmware/firmware"
compiler = shutil.which("gcc")
if compiler is None:
    raise RuntimeError("MinGW gcc is required")
assert (root / "BSP/work_can_protocol.h").read_text() == (
    chassis / "src/service/work_can_protocol.h"
).read_text(), "The two protocol headers differ"

can_source = (root / "Core/Src/can.c").read_text()
ioc = (root / "RED_Demo.ioc").read_text()
for expected in ("CAN_MODE_NORMAL", "CAN_BS1_5TQ", "CAN_BS2_2TQ"):
    assert expected in can_source and expected in ioc
assert "CAN2.CalculateBaudRate=1000000" in ioc

with tempfile.TemporaryDirectory(prefix="work_can_", dir=root / "tests") as directory:
    executable = Path(directory) / "test_work_can.exe"
    includes = [
        root / "tests/work_fakes", root / "BSP", chassis / "tests/fakes", chassis / "tests",
        chassis / "src/service", chassis / "src/platform", chassis / "src/app",
    ]
    sources = [
        root / "tests/test_work_can.c", root / "BSP/vision_motor.c",
        root / "BSP/work_control.c", root / "BSP/can_test.c",
        chassis / "src/service/work_link.c", chassis / "src/service/station_work.c",
    ]
    for mode in (0, 1, 2, 3):
        subprocess.run([
            compiler, "-DELBOW_TEST_MODE=" + str(mode),
            *(["-DAPP_RUN_MODE=6", "-DWORK_STATION_ENABLED=1"] if mode == 2 else []), "-std=c11", "-Wall", "-Wextra", "-Werror", "-DTEST_CAN_HAL",
            *["-I" + str(path) for path in includes],
            *[str(path) for path in sources], "-lm", "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True)

    route_executable = Path(directory) / "test_station_route.exe"
    route_sources = [root / "tests/test_station_route.c", *sources[1:]] + [
        chassis / "src/service" / name for name in (
            "full_map_test.c", "field_pose.c", "auto_route.c", "auto_motion.c",
            "auto_geometry.c", "laser_prediction.c")]
    subprocess.run([
        compiler, "-DELBOW_TEST_MODE=2", "-DAPP_RUN_MODE=6", "-DWORK_STATION_ENABLED=1",
        "-std=c11", "-Wall", "-Wextra", "-Werror", "-DTEST_CAN_HAL",
        *["-I" + str(path) for path in includes],
        *[str(path) for path in route_sources], "-lm", "-o", str(route_executable)
    ], check=True)
    subprocess.run([str(route_executable)], check=True)
