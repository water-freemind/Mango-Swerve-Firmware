"""Check the linked F407 image, interrupt table and GNU runtime entry points."""
import argparse
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--mode", default="FullMapWork",
                    choices=("FullMapWork", "VisionTest", "LegacyWork", "ManualEncoder"))
parser.add_argument("--toolchain", default=os.environ.get("ARM_GCC_ROOT"))
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
candidates = [Path(args.toolchain)] if args.toolchain else [
    Path("E:/EmbeddedTools/xpack-arm-none-eabi-gcc-15.2.1-1.1"),
    Path.home() / ".eide/tools/gcc_arm",
]
nm = next((p / "bin/arm-none-eabi-nm.exe" for p in candidates
           if (p / "bin/arm-none-eabi-nm.exe").is_file()), None)
if nm is None:
    found = shutil.which("arm-none-eabi-nm")
    if found is None:
        raise RuntimeError("Pass --toolchain or set ARM_GCC_ROOT")
    nm = Path(found)
suffix = nm.suffix
readelf = nm.with_name("arm-none-eabi-readelf" + suffix)
objdump = nm.with_name("arm-none-eabi-objdump" + suffix)
folder = root / "build" / args.mode
elf = folder / "elbow.elf"
binary = (folder / "elbow.bin").read_bytes()


def run(tool, *options):
    return subprocess.check_output([str(tool), *options, str(elf.relative_to(root))],
                                   cwd=root, text=True, encoding="utf-8",
                                   env={**os.environ, "LC_ALL": "C", "LANG": "C"})


symbols = {}
types = {}
for line in run(nm, "--defined-only").splitlines():
    match = re.fullmatch(r"([0-9a-fA-F]+)\s+(\w)\s+(\S+)", line)
    if match:
        address, kind, name = match.groups()
        symbols[name] = int(address, 16)
        types[name] = kind

assert len(binary) < 512 * 1024
assert symbols["_estack"] == 0x20020000
assert 0x20000000 <= symbols["_sdata"] <= symbols["_edata"] <= symbols["_sbss"]
assert symbols["_sbss"] <= symbols["_ebss"] < symbols["_estack"] - 0x600
assert 0x08000000 <= symbols["_sidata"] < 0x08080000
assert symbols["g_pfnVectors"] == 0x08000000
assert all(symbols[name] % 4 == 0 for name in ("_sdata", "_edata", "_sbss", "_ebss"))

vendor = (root / "MDK-ARM/startup_stm32f407xx.s").read_text()
vectors = re.findall(r"^\s*(?:__Vectors\s+)?DCD\s+(\w+)", vendor, re.MULTILINE)
assert len(vectors) == 98
values = struct.unpack_from("<98I", binary)
assert values[0] == symbols["_estack"]
for index, name in enumerate(vectors[1:], 1):
    assert values[index] == (0 if name == "0" else symbols[name] | 1), (index, name)

for name in ("Reset_Handler", "SysTick_Handler", "TIM5_IRQHandler",
             "USART3_IRQHandler", "TIM2_IRQHandler", "TIM3_IRQHandler", "TIM4_IRQHandler"):
    assert types[name] == "T", (name, types[name])
assert not re.search(r"\bU\s+", run(nm, "--undefined-only"))
attributes = run(readelf, "-A")
assert "Tag_ABI_VFP_args: VFP registers" in attributes
segments = run(readelf, "-lW")
assert not re.search(r"\bRWE\b", segments)
reset = run(objdump, "-d", "--disassemble=Reset_Handler")
calls = re.findall(r"\bbl\s+[0-9a-f]+\s+<([^>]+)>", reset)
assert calls == ["SystemInit", "__libc_init_array", "main"], calls

upper = 0
eof = False
for line in (folder / "elbow.hex").read_text().splitlines():
    record = bytes.fromhex(line.removeprefix(":"))
    assert sum(record) % 256 == 0
    size, address, kind = record[0], int.from_bytes(record[1:3], "big"), record[3]
    payload = record[4:4 + size]
    if kind == 4:
        upper = int.from_bytes(payload, "big") << 16
    elif kind == 0:
        offset = upper + address - 0x08000000
        assert offset >= 0 and binary[offset:offset + size] == payload
    elif kind == 1:
        eof = True
assert eof
print(f"GCC_IMAGE_OK mode={args.mode} vectors=98 hard_float=1 flash_bytes={len(binary)}")
