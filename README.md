# Mango Swerve Firmware

芒果疏果机器人舵轮底盘的独立 STM32H723 电控工程。当前固件包含四舵轮驱动、遥控手动控制、BMI088、轮式里程计和 S21C 的 FDCAN3 测距接收解包；FDCAN3 已整理为供 DM 主控、S21C 测距板和独立疏果装置共享的总线基础设施，实物 CAN 联调仍待完成。

**从这里阅读：[底盘电控项目总览与目录](docs/PROJECT_GUIDE.md)。** 赛题、硬件连接、源代码阅读路线、机械尺寸、测距协议、CAN3 接入、导航方案、联调顺序及待机械队友确认的问题都集中在这一份文档维护。

## 当前进度（2026-11-05）

**当前默认是通信诊断模式。** `firmware/src/app/app_config.h` 的 `APP_RUN_MODE` 默认为 `APP_MODE_CAN_DIAGNOSTICS`：三路 CAN 接收、USART1 日志和激光快照独立运行，不初始化底盘电机、IMU或遥控，也不发送应用层 CAN 命令。CAN 外设使用正常模式，会对正确帧提供 ACK。下午首轮使用 `firmware/build/Debug/robot.hex`，按[调车准备与现场检查](docs/PROJECT_GUIDE.md#调车准备与现场检查2026-11-05)先核对接线和真实收帧。

日志串口为 USART1（MCU PA9=TX、PA10=RX），921600 bit/s、8N1；外部接线按 DM 板实际串口插座丝印核对。每秒输出三路物理 CAN 的状态、轮询显示的报文 ID/数据和四路激光毫米值；SWD 可观察 `g_commissioning`。疏果器协议尚为 `TBD`，当前只能观察其主动上报，不能控制作业动作。

| 状态 | 内容 |
| --- | --- |
| 已有源码 | STM32H723 底盘固件与 EIDE/CubeMX 工程；CAN3 为经典 CAN 1 Mbps，具有 8 个标准 ID 端点槽位、16 帧 RX FIFO 和 8 帧 TX FIFO，接收标准 ID `0x601` 并解包四路毫米值。尚未按新车几何与电机配置完成实车联调。 |
| 用户已反馈 | S21C 板上的 STP23L 例程可以正常运行；实测在 0–7 cm 范围内仍能得到距离读数，但相对误差较大；CAN 线上是否实际收到数据仍待抓包。 |
| 已确认接口约定 | 原厂例程发送经典 CAN 标准数据帧 `0x601`、1 Mbps、8 字节；四路距离按 S21C 接口 1–4 顺序、每路大端 `uint16_t` 毫米。详见[CAN3 接收与解包](docs/PROJECT_GUIDE.md#h723-fdcan3-接收与解包)。 |
| 尚未实现/验证 | 疏果装置 CAN ID、载荷和周期仍为 `TBD`，未发送占位协议；逐路测距有效性处理和自主遍历尚未实现；CAN3 三节点接线、终端和实物收发仍未验证。 |

**下一步：**先用诊断固件确定 CAN1/2 实际接了哪组电机，再核对 S21C `0x601` 快照与超时；加入疏果器节点时检查波特率、ID 冲突、终端和错误计数。只有接线、四轮零位和几何参数确认后，才切换 `APP_RUN_MODE=1` 开始架空逐轮测试；该手动模式仍包含旧车几何和较高速度档，不适合直接落地试车。

FDCAN3 配置已同时写入 `firmware/robot.ioc` 和生成文件，无需再手动在 CubeMX 配置。2026-11-05 本机以 EIDE 构建器与 Arm GNU 15.2 对当前80个源文件清理编译和链接，诊断模式、手动模式与 CAN1/2 交换映射的手动模式均通过；供首轮烧录的诊断产物是 `firmware/build/Debug/robot.elf` 和 `robot.hex`。主机测试同时覆盖不发电机命令、CAN1故障不阻断CAN3、FIFO积压继续处理与共享队列满不撤销其他报文；实物收发仍待现场验证。

在安装 MinGW GCC 的 Windows PowerShell 中，可重复执行 CAN3 逻辑测试和 CubeMX/生成代码一致性检查：

```powershell
cd firmware
powershell -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1
```

成功时应输出 `CAN3_LOGIC_TEST_OK`、`COMMISSIONING_TEST_OK`、`COMMISSIONING_CAN1_FAILURE_TEST_OK` 和 `CAN3_CONFIG_TEST_OK`；两种 CAN1/2 映射各运行一遍诊断测试。该测试不访问硬件。

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
| 协议无关的疏果作业状态 | `firmware/src/service/thinning.*`；实际 CAN ID/字段仍为 `TBD` |
| 调车诊断入口与观察变量 | `firmware/src/app/app_config.h`、`firmware/src/service/commissioning.*`、`g_commissioning` |
| 转向/驱动的总线映射 | `firmware/src/platform/chassis_can_map.h`；`CHASSIS_CAN_BUSES_SWAPPED=0` 为转向CAN1/驱动CAN2，`=1` 同时交换收发绑定；收发器使能仍跟随物理外设 |

**测距接口维护边界：**当前仅按 S21C 接口 1–4 保存四路原始毫米值；`distance_mm[0]` 对应接口 1，依此类推。尚未确定各接口对应的车体方向，待用户提供实物接线/安装对应关系后再单独增加方向映射。不要在 CAN 解包层把接口号直接写成前、后、左、右；当前测距数据也未接入导航或底盘控制。接口定义、验证状态与后续接入步骤以[项目总览的 CAN3 小节](docs/PROJECT_GUIDE.md#h723-fdcan3-接收与解包)为准。

本工程取自 [AgroTech-SCAU/Steering-Wheel-Chassis](https://github.com/AgroTech-SCAU/Steering-Wheel-Chassis) 的 `Hephaestus/chassis_control_code`，基线提交 `65088f94be15c69ccfa647779d7f847b06b95735`；原仓库未修改。原 Hephaestus 文档可作为驱动代码参考，车体几何和赛题方案以本仓库的[项目总览](docs/PROJECT_GUIDE.md)为准。
