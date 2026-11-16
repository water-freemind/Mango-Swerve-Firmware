# 疏果 F407 配套工程

本目录用于与同仓库的 H723 全图底盘一起调试和交接。来源为 `肘击装置/RED_Demo/RED_Demo`，原仓库基线 `c33ab16`。包含 Core、BSP、所需 STM32F4 HAL/CMSIS、CubeMX 配置、GCC/EIDE构建配置、历史Keil工程和测试。

## 工程与模式

使用与底盘相同的 ARM GCC、PowerShell 和 EIDE 构建方式。打开 `elbow.code-workspace` 可在 EIDE 中选择模式，命令行使用 `build-local.ps1`。控制器为 STM32F407VETx，Flash512KB、SRAM128KB，Cortex-M4F硬浮点。原Keil工程保留用于历史对照。

`BSP/vision_motor.h` 的 ELBOW_TEST_MODE 默认2，与 H723 的 FullMapWork 镜像协作。模式1用于独立视觉测试，模式0使用原版本1的视觉主导协同，模式3用于PWM为零的手动编码器读数。修改后重新构建和烧录。

2026-11-14使用手推量程进行编码器闭环作业。A/B/C/D的行程为3822/3866/3866/3858计数，C、D的伸出编码器方向为负。伸出目标85%、回收目标2%；伸出前段100%、回收40%，接近目标时35%（A路伸出45%），回收最后5%行程使用10% PWM。TIM5每10ms采样后立即执行到点断PWM，反馈异常撤销输出并保持底盘停车。每次上电先手动完全收回四路，在VOFA登记肘击器零位。视觉目标dx=0，对中接受范围±30、重新调整阈值±45像素。连续2秒无合法帧才判离线。此前提速版已校验烧录。当前新增编码器诊断候选按用户要求通过PR交付，待实车验证。方向异常、软限位、300ms无进展、6秒超时分别使用结果码12～15，回收未就绪使用结果码16，并向H723发送电机、动作阶段、进度和目标快照。

模式2当前源码采用版本5协议。底盘沿垄巡航，同时识别到 Tree 和 Gmg 才提出 SLOW/WAIT，停车位置由视觉触发。dx=0 对中、连续停稳200ms后发送 START。四路编码器回收位置核验通过后，通过 DONE/RELEASE/RELEASED 离树。相同区域最多4次，次数耗尽记录未打落并继续行驶。离树确认避免重复触发同一目标。两板须配套更新版本5。操作、状态和异常处理见[联合调试说明](../docs/CAN_WORK_DEBUG.md)。

相机离线时显示 `W:WAIT_CAM`，作业等待视觉就绪，激光接收与显示继续刷新。周期显示每次只传一页，规定I2C延时约18.9ms，8页轮转；动作与复检期间推迟耗时的屏幕重初始化。相机断线就绪修复及本次逐页显示修复均需下次上电更新，镜像历史与当前候选区别见[离线复审记录](../docs/CAN_WORK_DEBUG.md#offline-review-pr16)。

## 当前击打控制与上电准备

| 模式 | 动作结束依据 | 使用范围 |
| --- | --- | --- |
| FullMapWork / 模式2 | 编码器位置到点停止，分段PWM减力；不是位置PID | 当前两板联合调试 |
| VisionTest / 模式1 | 定时击打600ms、回收600ms | 独立视觉动作测试，不具备模式2的位置判停 |
| LegacyWork / 模式0 | 定时击打600ms、回收600ms | 历史协同模式 |
| ManualEncoder / 模式3 | 四路PWM保持0 | 手推测量编码器行程 |

1. 先保持底盘停车、SWD=1000，人工将四路完全收回，检查机构无遮挡。
2. 采用配套版本5两板固件；每次F407复位后用VOFA命令 `UI MAP ELBOW HOME` 登记当前编码器零位，核对回执。该命令只登记当前位置，不会自动找机械零位。
3. 核查视觉、四路回收位置和CAN握手；具备遥控许可后再按联合调试流程准备、开始任务。
4. 发生方向、限位或超时故障后先排查机构和接线，核对零位是否失效，再按故障恢复流程操作；不得仅重复发送开始命令。

模式2参数以 `BSP/elbow_stroke_config.h` 为准：A/B/C/D总行程为3822/3866/-3866/-3858计数，负号表示伸出编码器方向。伸出到85%，回收到2%；回收放行上限为2.5%，零位允许负向1%偏移。区域1/2/3/4对应B/D/C/A。TIM5每10ms检查位置，到点关闭PWM。

正常动作300ms无有效进展或单阶段超过6秒触发保护；部分可信堵转/越限允许一次受限恢复，并非所有异常都自动重试。恢复短推受80ms及位置目标共同限制，恢复回收无进展阈值80ms。换向停顿、冷却、视觉稳定和通信超时仍使用计时，因此不能把所有时间宏理解为击打时长。

2026-11-17已构建当前FullMapWork并通过镜像检查，未烧录。先前部署记录只证明对应历史镜像；当前物理回收净空、动作效果和恢复动作仍需现场验收。

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

视觉串口帧为 `$VIS,区域,dx` 加 CRLF，区域0表示当前无待处理目标，dx9999表示无有效树。dx用于阈值请求、OLED及串口观察。WORK按视觉触发停车作业，固定树位用于NAV测试。模式2伸出和回收均按编码器位置判停，实机仍需核验零位、行程和回收净空。

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

本次迁移沿用现有控制逻辑和机械参数。每次F407复位后，仍需操作者手动完全收回四路，再登记编码器零位。两端协议头通过联合测试逐字核对，后续修改应同时更新两份定义。
