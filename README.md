# Mango Swerve Firmware

芒果疏果机器人舵轮底盘的独立 STM32H723 电控工程。当前只包含底盘运动、遥控手动控制、BMI088 IMU 与轮式里程计，不包含机械臂、ROS、Pi 通信或语音任务代码。

## 来源与版本

本工程从 AgroTech-SCAU 的 `Steering-Wheel-Chassis` 仓库中 `Hephaestus/chassis_control_code` 提取，基线提交为 `65088f94be15c69ccfa647779d7f847b06b95735`。原仓库未被修改。`Core` 和 `Drivers` 是在原工程中由 STM32CubeMX 6.18.1、STM32Cube FW_H7 V1.13.0 生成的依赖，随本仓库保存以便独立编译；ST 许可文本保留在各 `Drivers` 子目录中。

原 Hephaestus 的 MCU 与电机驱动文档是本项目理解和维护底盘的参考。当前车体已有变化，驱动硬件仍沿用原方案；[底盘驱动基线与本车差异](docs/hephaestus-chassis-baseline.md)列出可继承的代码、原文档与实际型号配置的区别，以及需要重新测量的几何参数。

机械队友已提供左右轮中心距 220 mm、名义轮径 102 mm、最大通行外廓 352 mm，以及 STP23L 暂定安装尺寸；[当前车体尺寸与电控影响](docs/current-chassis-dimensions.md)记录数据、算术疑点和固件待标定项。其余待确认尺寸见[车体参数确认清单](docs/mechanical-chassis-parameters-request.md)。

## 实车与软件配置

| 部分 | 当前信息 |
| --- | --- |
| 主控 | STM32H723VGT6 |
| 转向 | 4 × 达妙 6220，FDCAN1，ID 1–4；现有驱动使用 DM 位置速度模式 |
| 驱动 | 4 × DJI M3508/C620，FDCAN2，ID 1–4；`src/service/chassis.c` 配置 `DJI_MOTOR_MODEL_M3508` |
| 遥控 | FS-iA10B i.BUS，UART5，115200 bit/s，RX 电平反相 |
| 姿态 | BMI088，SPI2 |
| 里程计 | 现有 `odom.c` 融合轮速与 IMU；不是外置独立编码器模块 |

四轮顺序为 FL、FR、RR、RL。两个 CAN 都是 1 Mbps。`robot.ioc` 沿用原控制板引脚配置；其中 UART7、UART8、USART10 和 PE9/PE13 是原项目遗留资源，本版启动流程不使用它们。后续变更 `.ioc` 并重新生成代码时，需检查这些外设初始化是否被重新写入 `Core/Src/main.c`。

## 手动控制

仅当遥控器在线、SWD 处于手动位置、SWA 选底盘且电机反馈齐全时接收速度命令。右杆 Y/X 对应前后/横移，左杆 X 对应旋转；SWB 选择三档速度，SWC 高位或 VRA 低位请求驻车。遥控离线、退出手动或底盘未就绪时请求驻车。原有底盘驱动在 500 Hz 运行，里程计在 250 Hz 更新，遥控在 100 Hz 更新。

## 编译与目录

在 VS Code 中用 EIDE 打开 `firmware` 目录，选择 `Debug` 或 `Release` 构建。工程使用 ARM GNU GCC、STM32H723 和 `STM32H723VGTX_FLASH_USER.ld`。`firmware/robot.ioc` 可由 STM32CubeMX 打开；重新生成代码后先核对 `Core/Src/main.c` 的自定义入口和未使用的外设初始化。

`firmware/src/app/entry.h` 是精简后的启动与遥控控制入口；`src/service/chassis.c` 是轮位映射和底盘执行层；`src/domain/steer_wheel_kine.c` 是舵轮运动学；`src/device/bus_motor/` 是达妙和大疆电机协议实现；`src/service/odom.c` 是当前里程计。

## 后续接入

拟采用 WHEELTEC S21C 采集板配 STP23L 单点测距模块，数量和安装朝向待定。若实物是 DM-MC-Board02，建议 S21C 使用其独立 CAN3；[DM 板 CAN3 接入方案](docs/dm-board02-s21c-can3-plan.md)列出针脚、供电、终端与固件改动。资料和赛题约束的分析见[赛题与 S21C/STP23L 技术分析](docs/competition-s21c-stp23l-analysis.md)；传感器规格、原始协议和 S21C 数据有效性风险见[STP-23L 资料详审](docs/stp23l-sensor-review.md)。本版尚未实现 CAN3、S21C 接收、自主地图遍历或比赛起跑/出场状态机；比赛过程中不得依赖现有遥控手动控制。任何上车下载前，先核对 6220 的 CAN ID/控制模式、3508/C620 的 ID 与轮向，以及控制板实际接线。
