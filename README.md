# Mango Swerve Firmware

芒果疏果机器人舵轮底盘的独立 STM32H723 电控工程。当前固件包含四舵轮驱动、遥控手动控制、BMI088、轮式里程计和 S21C 的 FDCAN3 测距接收解包；S21C 与 H723 的实物 CAN 联调仍待完成。

**从这里阅读：[底盘电控项目总览与目录](docs/PROJECT_GUIDE.md)。** 赛题、硬件连接、源代码阅读路线、机械尺寸、测距协议、CAN3 接入、导航方案、联调顺序及待机械队友确认的问题都集中在这一份文档维护。

## 当前进度（2026-10-12）

| 状态 | 内容 |
| --- | --- |
| 已有源码 | STM32H723 底盘固件与 EIDE/CubeMX 工程；CAN3 已配置为经典 CAN 1 Mbps，接收标准 ID `0x601` 并解包四路毫米值。尚未按新车几何与电机配置完成实车联调。 |
| 用户已反馈 | S21C 板上的 STP23L 例程可以正常运行；实测在 0–7 cm 范围内仍能得到距离读数，但相对误差较大；CAN 线上是否实际收到数据仍待抓包。 |
| 已确认接口约定 | 原厂例程发送经典 CAN 标准数据帧 `0x601`、1 Mbps、8 字节；四路距离按 S21C 接口 1–4 顺序、每路大端 `uint16_t` 毫米。详见[CAN3 接收与解包](docs/PROJECT_GUIDE.md#h723-fdcan3-接收与解包)。 |
| 尚未实现 | 逐路测距有效性处理、自主遍历和作业机构握手；CAN3 实物接线与收帧尚未验证。 |

**下一步：**核对 DM 板 CAN3 接线、供电和终端，用 S21C 实物 CAN 帧验证 H723 接收值、帧计数和超时；做逐路断线/无目标测试。同时在底盘联调前处理当前驱动 ID 映射不一致，并向机械队友确认转向零位与受载轮径。具体顺序和验收依据见[实施与验收顺序](docs/PROJECT_GUIDE.md#8-实施与验收顺序)。

FDCAN3 已写入 `firmware/robot.ioc` 和固件源文件，无需再手动在 CubeMX 配置。完整固件已用 Arm GNU 13.3 编译链接通过；本机未运行 CubeMX 代码重新生成，实物收帧仍待验证。

## 工程入口

| 内容 | 位置 |
| --- | --- |
| EIDE 工程与 CubeMX 配置 | `firmware/`、`firmware/robot.ioc`；VS Code EIDE 选择 Debug/Release，目标 STM32H723VGT6、ARM GNU GCC |
| 启动与调度 | `firmware/Core/Src/main.c`、`firmware/src/app/entry.h` |
| 电机装配与底盘执行 | `firmware/src/service/assemble/assemble_chassis.c`、`firmware/src/service/chassis.c` |
| 四舵轮运动学 | `firmware/src/domain/steer_wheel_kine.c` |
| 电机协议与里程计 | `firmware/src/device/bus_motor/`、`firmware/src/service/odom.c` |
| S21C CAN3 接收与数据快照 | `firmware/src/service/assemble/assemble_laser.c`、`firmware/src/service/laser.h` |

本工程取自 [AgroTech-SCAU/Steering-Wheel-Chassis](https://github.com/AgroTech-SCAU/Steering-Wheel-Chassis) 的 `Hephaestus/chassis_control_code`，基线提交 `65088f94be15c69ccfa647779d7f847b06b95735`；原仓库未修改。原 Hephaestus 文档可作为驱动代码参考，车体几何和赛题方案以本仓库的[项目总览](docs/PROJECT_GUIDE.md)为准。
