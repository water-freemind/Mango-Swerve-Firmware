# Mango Swerve Firmware

芒果疏果机器人舵轮底盘的独立 STM32H723 电控工程。当前范围为四舵轮电机、遥控手动控制、BMI088、轮式里程计、VOFA+ 日志和 S21C 的 FDCAN3 测距接收解包。底盘启动与状态检查已按这些设备独立组织。

**从这里阅读：[底盘电控项目总览与目录](docs/PROJECT_GUIDE.md)。** 赛题、硬件连接、源代码阅读路线、机械尺寸、测距协议、CAN3 接入、导航方案、联调顺序及待机械队友确认的问题都集中在这一份文档维护。

## 当前进度（2026-11-05）

**当前默认是通信诊断模式。** `firmware/src/app/app_config.h` 的 `APP_RUN_MODE` 默认为 `APP_MODE_CAN_DIAGNOSTICS`：三路 CAN 接收、USART1 日志和激光快照独立运行，不初始化底盘电机、IMU或遥控，也不发送应用层 CAN 命令。CAN 外设使用正常模式，会对正确帧提供 ACK。下午首轮使用 `firmware/build/Debug/robot.hex`，按[调车准备与现场检查](docs/PROJECT_GUIDE.md#调车准备与现场检查2026-11-05)先核对接线和真实收帧。

日志串口为 USART1（MCU PA9=TX、PA10=RX），921600 bit/s、8N1；外部接线按 DM 板实际串口插座丝印核对。每秒输出三路物理 CAN 的状态、轮询显示的报文 ID/数据和四路激光毫米值；SWD 可观察 `g_commissioning`。完整手动模式输出 `CHASSIS_BOOT` 分步启动日志和 `CHASSIS_CHECK` 底盘状态。

| 状态 | 内容 |
| --- | --- |
| 已有源码 | STM32H723 底盘固件与 EIDE/CubeMX 工程；CAN3 为经典 CAN 1 Mbps，具有 8 个标准 ID 端点槽位、16 帧 RX FIFO 和 8 帧 TX FIFO，接收标准 ID `0x601` 并解包四路毫米值。尚未按新车几何与电机配置完成实车联调。 |
| 已验证的总线 | 接线修复后，CAN1上四个达妙转向电机均回应且初始化返回 `state=1`，用户确认指示灯转绿；CAN2持续收到四个C620反馈；CAN3持续收到约10Hz的 `0x601` 并完成四路解包。四只激光均已接入，独立遮挡测试确认接口1=左、2=右、3=后、4=前。 |
| 已确认接口约定 | 原厂例程发送经典 CAN 标准数据帧 `0x601`、1 Mbps、8 字节；四路距离按 S21C 接口 1–4 顺序、每路大端 `uint16_t` 毫米。详见[CAN3 接收与解包](docs/PROJECT_GUIDE.md#h723-fdcan3-接收与解包)。 |
| 当前待验证 | 四轮零位、新车几何与有效轮径；激光安装坐标、方位角与距离精度。 |

**后续调试安排**见项目指南中的调零与参数标定计划。CAN1/2设备归属和四路激光方向已完成静态验证，四轮零位和新车参数仍待标定。加入疏果器节点时检查波特率、ID冲突、终端和错误计数。手动模式仍包含旧车几何和较高速度档，动作测试前需配置受限标定入口。

2026-11-05 纯底盘裁剪后，79个源文件以Arm GNU 15.2.1通过静态诊断、手动、转向初始化与只读查询四种构建。CubeMX配置与生成代码同步清理了UART7、UART8、USART10及原吸盘输出。主机测试覆盖CAN诊断、转向初始化、只读查询、遥控许可和裁剪范围。实现依据与启动依赖详见[底盘裁剪分析](docs/PROJECT_GUIDE.md#纯底盘裁剪与自检边界2026-11-05)。

在安装 MinGW GCC 的 Windows PowerShell 中，可重复执行 CAN3 逻辑测试和 CubeMX/生成代码一致性检查：

```powershell
cd firmware
powershell -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1
```

成功时应输出 `CAN3_LOGIC_TEST_OK`、`COMMISSIONING_TEST_OK`、`COMMISSIONING_CAN1_FAILURE_TEST_OK` 和 `CAN3_CONFIG_TEST_OK`；两种 CAN1/2 映射各运行一遍诊断测试。该测试不访问硬件。

激光接入的离线准备已补充初始状态、四接口顺序、全零/重复载荷、300ms边界和计时回绕测试，输出 `LASER_OFFLINE_PREPARATION_TEST_OK`。装车后按[激光接入准备与验收](docs/PROJECT_GUIDE.md#激光接入准备与验收2026-11-05)逐路验证。

装车后的[调零与参数标定计划](docs/PROJECT_GUIDE.md#装车后调零与参数标定计划离线准备)已列出四轮身份核对、软件零位偏置、方向与复定位记录表、受限标定入口要求，以及几何/轮径/里程计的调整顺序。实测值待装车后采集。

## 工程入口

| 内容 | 位置 |
| --- | --- |
| EIDE 工程与 CubeMX 配置 | `firmware/`、`firmware/robot.ioc`；VS Code EIDE 选择 Debug/Release，目标 STM32H723VGT6、ARM GNU GCC |
| 启动与调度 | `firmware/Core/Src/main.c`、`firmware/src/app/entry.h` |
| 电机装配与底盘执行 | `firmware/src/service/assemble/assemble_chassis.c`、`firmware/src/service/chassis.c` |
| 四舵轮运动学 | `firmware/src/domain/steer_wheel_kine.c` |
| 电机协议与里程计 | `firmware/src/device/bus_motor/`、`firmware/src/service/odom.c` |
| S21C CAN3 接收与数据快照 | `firmware/src/service/assemble/assemble_laser.c`、`firmware/src/service/laser.h` |
| CAN3 共享总线与诊断 | `firmware/src/service/can3_bus.*`、`firmware/src/service/can3_router.*`、`firmware/src/platform/stm32_hal_can.*` |
| 调车诊断入口与观察变量 | `firmware/src/app/app_config.h`、`firmware/src/service/commissioning.*`、`g_commissioning` |
| 转向/驱动的总线映射 | `firmware/src/platform/chassis_can_map.h`；当前默认 `CHASSIS_CAN_BUSES_SWAPPED=0`，转向CAN1、驱动CAN2。四个转向回复ID为 `0x11–0x14`，四个驱动反馈ID为 `0x201–0x204`，均已收帧验证。开关同时交换收发绑定 |

**测距接口维护边界**为按 S21C 接口1–4保存四路原始毫米值，`distance_mm[0]` 对应接口1，依此类推。2026-11-05静态遮挡测试确认数组顺序为左、右、后、前。上层按前、后、左、右读取时索引依次为3、2、0、1。方向映射在上层使用，当前测距数据尚待接入导航或底盘控制。接口定义、验证状态与后续接入步骤以[项目总览的 CAN3 小节](docs/PROJECT_GUIDE.md#h723-fdcan3-接收与解包)为准。

本工程取自 [AgroTech-SCAU/Steering-Wheel-Chassis](https://github.com/AgroTech-SCAU/Steering-Wheel-Chassis) 的 `Hephaestus/chassis_control_code`，基线提交 `65088f94be15c69ccfa647779d7f847b06b95735`；原仓库未修改。原 Hephaestus 文档可作为驱动代码参考，车体几何和赛题方案以本仓库的[项目总览](docs/PROJECT_GUIDE.md)为准。
