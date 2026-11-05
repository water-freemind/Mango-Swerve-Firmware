# Mango Swerve Firmware

芒果疏果机器人四舵轮底盘的独立 STM32H723 裸机工程，转向使用达妙 DM6220，驱动使用大疆 M3508/C620。范围包括底盘电机、遥控、BMI088、轮式/IMU 里程计，以及 S21C 的 CAN3 四路测距接收。疏果装置仅预留共享总线接入边界，具体协议尚未确定。

**技术文档统一维护在 [项目指南](docs/PROJECT_GUIDE.md)。**

## 当前进度（2026-11-05）

默认 `APP_RUN_MODE=0` 为静态通信诊断：三路 CAN 接收、USART1 日志和激光快照，不初始化电机、IMU、遥控，不发送应用层 CAN 命令。CAN 外设使用正常模式，会提供 ACK。

| 项目 | 已有证据与下一步 |
| --- | --- |
| CAN1/2 | PR #1 的现场记录确认 CAN1=四个 DM6220，回复 ID `0x11–0x14`；CAN2=四个 C620，反馈 ID `0x201–0x204`。默认 `CHASSIS_CAN_BUSES_SWAPPED=0`。各 ID 对应的物理轮位仍需逐轮核对。 |
| CAN3 激光 | 已记录约 10 Hz 的标准帧 `0x601` 和四路解包；遮挡测试确认 S21C 接口 1/2/3/4=左/右/后/前。原始 `distance_mm[0..3]` 维持接口顺序，上层前/后/左/右索引为 3/2/0/1。位置坐标、逐路健康与导航融合待完成。 |
| 四轮零位 | 已采集 ID 1–4 的正前方读数，独立 `ZeroTest` 完成一次逐轮低速回位，用户确认轮向对齐。零位补偿尚未接入正常底盘运动学；重复性与转向/驱动符号仍待核验。 |
| 直行准备 | 当前保留旧车几何、半径及经验偏航补偿；`chassis_yaw_hold.*` 尚无运行入口调用。先完成轮位、零位、符号与低速入口，再验证 IMU 航向保持和侧向测距纠偏。 |
| 验证边界 | 合并维护阶段完成主机测试及 Diagnostics、Manual、Steering、Can2Query、ZeroTest 五种完整 ARM 构建（各 80 个源文件）。本次未烧录、点动或重新验证上述现场结果。 |

明日按 [2026-11-06 调车与长距离直行计划](docs/PROJECT_GUIDE.md#明日调车与长距离直行计划2026-11-06) 执行。历史现场结果、日志索引和待办均在指南中保留；完整手动固件尚未具备新车首轮落地标定条件。

## 目录与代码入口

```text
README.md                  项目入口、当前状态、构建命令
docs/PROJECT_GUIDE.md       唯一持续维护的技术指南与现场证据
.vscode/tasks.json         仓库根目录的 VS Code 任务
firmware/
  robot.ioc                CubeMX 配置
  .eide/                   EIDE 工程配置
  Core/、Drivers/          CubeMX 生成代码及依赖
  src/app/                 模式、裸机入口和调度
  src/service/             底盘、装配、诊断、测距、里程计
  src/domain/              四舵轮运动学、PID、融合算法
  src/device/              电机、IMU、遥控等驱动
  src/platform/            HAL 适配及 CAN1/2 映射
  tests/                   主机逻辑及 CubeMX 一致性检查
  build-local.ps1          离线构建
  flash-local.ps1          SWD 探测、备份及烧录
  monitor-local.ps1        串口日志采集
  build/                   本机产物、日志及备份，Git 忽略
```

建议阅读顺序：`Core/Src/main.c → src/app/entry.h → src/service/assemble/assemble_chassis.c → src/service/chassis.c → src/domain/steer_wheel_kine.c → src/device/bus_motor/ → src/service/odom.c`。

CAN3 从 `src/service/assemble/assemble_laser.c → can3_bus.* → can3_router.* → laser.*` 阅读；通信观察变量为 `g_commissioning`。构建和代码入口路径均相对于 `firmware/`。

## 构建与模式

在仓库根目录运行；主机测试需要 MinGW GCC，固件需要 ARM GNU GCC。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\firmware\tests\run_host_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File .\firmware\build-local.ps1
```

构建自动查找 `ARM_GCC_ROOT`、原开发机的 `E:\EmbeddedTools`、当前用户 EIDE 工具目录及 PATH。也可显式指定安装根目录：`-Toolchain 'C:\Users\Zhanglongsheng\.eide\tools\gcc_arm'`。输出位于下面各目录中的 `robot.elf/.hex/.bin`，失败构建会先移除该模式的旧固件产物。

| `-Mode` | 宏值 | 输出目录 | 上电行为 |
| --- | --- | --- | --- |
| Diagnostics（默认） | 0 | `firmware/build/Debug/` | 只接收和记录，无应用 CAN 发送 |
| Manual | 1 | `firmware/build/Manual/` | 完整底盘初始化，电机使能、驻车及遥控动作 |
| Steering | 2 | `firmware/build/Steering/` | 转向初始化，清错、使能并切换模式 |
| Can2Query | 3 | `firmware/build/Can2Query/` | 发送只读 ESC_ID 查询，使用实际转向总线映射 |
| ZeroTest | 4 | `firmware/build/ZeroTest/` | 转向初始化后等待 5 秒，按 ID 1–4 逐轮回位并释放 |

编译不执行硬件动作；烧录后行为取决于模式。VS Code 打开仓库根目录可用 `Ctrl+Shift+B` 构建诊断模式；EIDE 也可直接打开 `firmware/`。每次 CubeMX 重新生成后重新运行主机测试和完整构建。

## 烧录、日志与维护

`flash-local.ps1` 默认只选择 Diagnostics，写入前重新构建并备份、校验原 1 MB Flash。OpenOCD 自动查找 `OPENOCD_ROOT`、原 E 盘工具、EIDE 及 PATH，也支持 `-OpenOcdRoot`。具体硬件操作见 [开发命令](docs/PROJECT_GUIDE.md#本机开发命令2026-11-05)。

日志使用 USART1（PA9=TX、PA10=RX），921600 bit/s、8N1；板侧插座以实物丝印为准。VS Code 的 `Firmware Monitor` 任务会询问串口，默认示例 COM12，采集 30 秒。主机测试不访问串口或调试器。

维护时通过分支和 PR 更新 `main`，保留编译、测试及实车证据的区别。`build/` 中的二进制、串口日志与原板备份仅保存在本机，不提交 Git。修改固件时同步更新本页与项目指南，不另建零散进度文档。

来源为 [AgroTech-SCAU/Steering-Wheel-Chassis](https://github.com/AgroTech-SCAU/Steering-Wheel-Chassis) 的 `Hephaestus/chassis_control_code`，基线 `65088f94be15c69ccfa647779d7f847b06b95735`；原仓库未修改。原文档虽混用 Atlas 标题，其底盘驱动仍可参考，新车参数以本项目记录为准。
