# 疏果 F407 配套工程

当前候选首次作业前无目标复检2秒，始终无目标返回SKIPPED并由底盘记录未完成。两板须配套更新；见[修复与部署状态](../docs/work/full-map/20261117-ridge-end-and-no-target.md)。

本目录用于与同仓库的 H723 全图底盘一起调试和交接。来源为 `肘击装置/RED_Demo/RED_Demo`，原仓库基线 `c33ab16`。包含 Core、BSP、所需 STM32F4 HAL/CMSIS、CubeMX 配置、GCC/EIDE构建配置、历史Keil工程和测试。

## 工程与模式

使用与底盘相同的 ARM GCC、PowerShell 和 EIDE 构建方式。打开 `elbow.code-workspace` 可在 EIDE 中选择模式，命令行使用 `build-local.ps1`。控制器为 STM32F407VETx，Flash512KB、SRAM128KB，Cortex-M4F硬浮点。原Keil工程保留用于历史对照。

`BSP/vision_motor.h` 的 ELBOW_TEST_MODE 默认2，与 H723 的 FullMapWork 镜像协作。模式1用于独立视觉测试，模式0使用原版本1的视觉主导协同，模式3用于PWM为零的手动编码器读数。修改后重新构建和烧录。

2026-11-16按 `yjjy25/elbow-controller` 已验证的分阶段计时改为纯定时：100% 占空比打击 650 ms → 停 10 ms 换向 → 返回 650 ms → 等待 50 ms，参数和四条方向函数与该仓库逐项对齐（TIM1 PSC=79/ARR=99、CCR=100 即满占空比）。不再使用编码器位置闭环、软限位、无进展检测和回位核验。`VISION_MOTOR_PWM`、`VISION_CW_MS`、`VISION_CCW_MS`、`VISION_REVERSE_PAUSE_MS`、`VISION_RETRIGGER_DELAY_MS` 都在 `BSP/vision_motor.h`。上电不再需要手动收回并登记零位，`Vision_HomeConfirmed` 直接为 1，底盘 HOME 握手仍可应答。编码器只保留测速和手动换行程读数；若现场出现顶死堵转，把 `ELBOW_STROKE_END_MS` 从 0 改成 200 可启用“编码器停走即收尾”。视觉目标dx=-22，ALIGN与START均要求±10像素，作业复检超过±30像素时请求返回后重新对齐。连续2秒无合法帧才判离线。`elbow_stroke_config.h` 已停用，只留作手推量程记录。串口状态行新增 `STROKE:<ms> CNT:<counts>`，可直接读到上一段打击/返回的真实用时和编码器变化量。

模式2当前源码采用版本5协议。底盘沿垄巡航，同时识别到 Tree 和 Gmg 才提出 SLOW/WAIT，停车位置由视觉触发。dx=-22 对中、连续停稳200ms后发送 START。定时返回结束后通过 DONE/RELEASE/RELEASED 离树，不再做编码器回收核验。同一区域最多4次，且一棵树累计最多4次；次数耗尽记录未打落，通过 DONE/RELEASE 交给底盘前往下一个工作区。离树确认避免重复触发同一目标。两板须配套更新版本5。操作、状态和异常处理见[联合调试说明](../docs/CAN_WORK_DEBUG.md)。

相机离线时显示 `W:WAIT_CAM`，作业等待视觉就绪，激光接收与显示继续刷新。周期显示每次只传一页，规定I2C延时约18.9ms，8页轮转；动作与复检期间推迟耗时的屏幕重初始化。相机断线就绪修复及本次逐页显示修复均需下次上电更新，镜像历史与当前候选区别见[离线复审记录](../docs/CAN_WORK_DEBUG.md#offline-review-pr16)。

## 当前击打控制与上电准备

FullMapWork模式2、VisionTest模式1和LegacyWork模式0均按毫秒计时：100%占空比打击650ms、停止10ms换向、返回650ms，再等待50ms。模式3保持PWM为零，用于手推编码器读数。当前不是编码器位置闭环；可选停走检测默认关闭。

1. 保持底盘停车、SWD=1000，上电前人工将四路收回，确认机构无遮挡。
2. 两板采用配套版本5固件；不再需要登记编码器零位，HOME只兼容旧握手，不会自动找零。
3. 核查视觉与CAN，提供遥控许可后再准备、开始联合任务。
4. 定时结束不证明机构已收回；首次动作需现场确认机械回收净空和方向。

每棵树累计最多4次打击，达到上限后记录未完成目标并握手放行。STROKE/CNT记录上一段实际用时和编码器变化。时间参数位于BSP/vision_motor.h，旧elbow_stroke_config.h仅保留历史量程。

## 源码入口

| 文件 | 用途 |
| --- | --- |
| Core/Src/main.c | 初始化及主循环调用 |
| BSP/work_control.c | 握手、接近请求、复检、作业、结束和故障状态机 |
| BSP/can_test.c | CAN2过滤、收发、100ms重发和500ms超时 |
| BSP/work_can_protocol.h | 与H723一致的旧版本1与当前版本5协议 |
| BSP/vision_motor.c | 视觉解析和四个区域的电机动作 |
| BSP/vision_motor.h | 模式、阈值、动作计时和重试参数 |
| BSP/fun.c | OLED及周期串口诊断 |

视觉串口帧为 `$VIS,区域,dx` 加 CRLF，区域0表示当前无待处理目标，dx9999表示无有效树。dx用于阈值请求、OLED及串口观察。WORK按视觉触发作业，固定树位用于NAV测试。肘击按固定时间正反转，实机需要核验回位和净空。

## 构建与仿真

从仓库根目录运行，工具链搜索顺序与底盘一致。可通过 `ARM_GCC_ROOT` 环境变量或 `-Toolchain` 指定GCC安装根目录。

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File firmware-elbow/build-local.ps1 -Mode FullMapWork
python firmware-elbow/tests/verify_gcc_build.py --mode FullMapWork
python firmware-elbow/tests/verify_work_can.py
python firmware-elbow/tests/verify_vision_motor.py
python firmware-elbow/tests/verify_oled.py
```

| 构建模式 | ELBOW_TEST_MODE | 用途 | HEX输出 |
| --- | --- | --- | --- |
| FullMapWork | 2 | 与H723联合调试，默认模式 | build/FullMapWork/elbow.hex |
| VisionTest | 1 | 独立视觉测试 | build/VisionTest/elbow.hex |
| LegacyWork | 0 | 原视觉主导协同 | build/LegacyWork/elbow.hex |
| ManualEncoder | 3 | PWM为零的手动编码器读数 | build/ManualEncoder/elbow.hex |

各模式分别生成 `elbow.elf`、`elbow.hex`、`elbow.bin`、`elbow.map` 和 `obj/`，构建失败会撤销该次构建的旧ELF/HEX/BIN入口。GCC使用 `Core/Startup/startup_stm32f407xx.s` 和 `STM32F407VETX_FLASH.ld`，98项中断向量与原F407启动表逐项核对。Flash从0x08000000开始，栈顶为0x20020000，最小堆512字节、栈1024字节。GNU运行库使用Newlib Nano，硬件串口继续经现有HAL接口收发。

VS Code仓库工作区提供 `Elbow Firmware Build` 任务。EIDE的四个目标与PowerShell使用同一模式宏、HAL源文件、链接脚本和硬浮点配置。GCC构建独立于Keil安装。

测试需要 Python 与64位MinGW-w64 GCC。联合测试使用两板实际状态机，覆盖25种速度组合及无绿果、第三棵树等场景。`verify_gcc_build.py` 检查已生成镜像的98项向量、强中断入口、C运行时初始化调用、RAM/Flash边界、硬浮点ABI及HEX/BIN一致性。GCC迁移完成四种模式的编译和镜像核验，实际板上运行待验证。

本次迁移沿用现有控制逻辑和机械参数。定时方案下 F407 复位后不再需要手动收回四路和登记零位，上电即按已收回状态应答底盘。两端协议头通过联合测试逐字核对，后续修改应同时更新两份定义。
