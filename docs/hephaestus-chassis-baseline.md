# Hephaestus 底盘驱动基线与本车差异

> 2026-10-08 核对。来源为原仓库 `AgroTech-SCAU/Steering-Wheel-Chassis` 的 `Hephaestus/chassis_control_code/README.md`、`src/device/bus_motor/README.md`、`src/domain/README.md` 及同目录底盘源码；原仓库基线提交 `65088f94be15c69ccfa647779d7f847b06b95735`。原仓库只读，本文维护在本项目。用户确认本车车体有变化，底盘驱动硬件没有变化。

## 如何使用原文档

原 `Hephaestus/README.md` 虽以 Atlas 为标题，两车的底盘驱动基本相同，因此继续将其 MCU 底盘部分作为本项目的参考。`chassis_control_code/README.md` 的分层、底盘调度和手动控制说明，以及 `src/device/bus_motor/README.md` 的电机接口说明，是熟悉本车代码的主要入口；再用本仓库源码确认**当前实际启用**的配置。原文档对 Pi、ROS2、机械臂和语音任务的说明可帮助理解原系统，但这些模块尚未纳入本仓库的精简固件，不能按原文档的操作流程直接使用。

| 原文档主题 | 本仓库对应位置 | 当前状态 |
| --- | --- | --- |
| MCU 启动与 500 Hz 底盘控制 | `firmware/Core/Src/main.c`、`firmware/src/app/entry.h` | 保留底盘 500 Hz、里程计 250 Hz、遥控 100 Hz；启动装配精简为 delay、log、RGB、IMU、odom、chassis、remote。 |
| 电机协议与四舵轮运动学 | `firmware/src/device/bus_motor/`、`firmware/src/service/chassis.c`、`firmware/src/domain/steer_wheel_kine.c` | 保留原驱动和运动学；实际电机仍为 6220 转向与 M3508/C620 驱动。 |
| 手动遥控 | `firmware/src/app/entry.h`、`firmware/src/service/remote.c` | 保留底盘手动控制，入口已简化；以当前代码的开关条件为准。 |
| AutoPi、ROS2、机械臂和语音任务 | 当前固件没有对应实现 | 留作原系统参考；本赛题的自主模式与作业装置将按本车需求设计。 |

## 保留的驱动基线

| 项目 | 当前固件依据 | 本车使用方式 |
| --- | --- | --- |
| 转向 | `assemble_chassis.c` 把达妙电机接 FDCAN1，`chassis_prepare_steer_motor()` 切至 `DM_MOTOR_MODE_POS_VEL` | 四个达妙 6220，ID 1～4；保留原控制与反馈协议。 |
| 驱动 | `assemble_chassis.c` 把大疆电机接 FDCAN2，`chassis.c` 显式指定 `DJI_MOTOR_MODEL_M3508` | 四个 M3508/C620，ID 1～4；保留原大疆控制与反馈协议。 |
| 四轮顺序 | `chassis.h` 的 `CHASSIS_MODULE_TABLE` | FL、FR、RR、RL；驱动方向符号按当前 `chassis.c` 映射为 `+、−、−、+`，实车轮向仍以标定为准。 |
| 运动学 | `steer_wheel_kine.c/.h` | 保留四舵轮正/逆运动学，输入线速度 m/s、角速度 rad/s；几何参数由新车实测值提供。 |
| 里程计 | `odom.c` 与 BMI088、轮速反馈 | 保留原融合链路；车体变化后重新核对轴向、偏置和实际位移误差。 |
| 调度 | 精简后的 `entry.h` | 底盘处理 500 Hz、里程计 250 Hz、遥控 100 Hz；原工程其他任务频率不直接套用。 |

本次以文件 SHA-256 比对原检出目录与本仓库：`chassis.h`、`assemble_chassis.c`、`steer_wheel_kine.c/.h`、`dm_motor.c/.h`、`dji_motor.c/.h`、`odom.c` 均相同。`chassis.c` 唯一差异是一处说明性注释：原检出文件写 M2006/C610，本仓库注释写 M3508/C620；两边**实际编译的型号配置都为 `DJI_MOTOR_MODEL_M3508`**。因此保留的是底盘驱动代码和行为基线，不意味着新车的几何、载荷或导航性能已经标定。

原 `bus_motor/README.md` 提到 `driver_config == NULL` 时大疆驱动默认 M2006；这是驱动设备层的兼容默认值。当前 `chassis.c` 显式传入 M3508 配置，不能把该默认值理解成本车使用 M2006。

## 车体变化后需重新测量的参数

当前 `assemble_chassis.c` 仍沿用原参数：前后轮中心距 `length = 0.26572986916 m`、左右轮中心距 `width = 0.26572986916 m`、轮半径 `0.035276 m`、`wheel_drive_ratio = 1.0`，以及 yaw 补偿 `k_vx = -0.049`、`k_vy = 0.055`。其中 `length/width` 是**轮中心几何距离**，并非车体外廓。车架改变时先量新车四轮转向轴中心；若轮组位置改变，更新轮距。若轮胎、减速传动未变，轮半径和传动比可作为起点，但仍用实测行驶距离核验。载荷、重心、轮胎接地与地毯条件改变后，应重新评估 yaw 补偿、加减速和停车距离。比赛 38 cm 可能净空要单独量**整车及作业装置最大外宽**，不能由轮中心距推断。

## 后续改动边界

S21C/STP23L 经 DM 板 CAN3 接入时，新增 FDCAN3 初始化和测距接收模块；保留 FDCAN1 转向、FDCAN2 驱动以及现有电机协议。地图遍历使用当前舵轮速度接口与里程计作为底层，不复制原 Atlas 的 ROS2/AutoPi 任务流。实现自主模式前，仍须验证电机反馈新鲜度、CAN 发送失败处理和实车制动效果；原有 `chassis.is_ready()` 表示曾收到反馈，不能单独证明当前电机在线。

接线方案见 [S21C 与 DM 板 CAN3 接入](dm-board02-s21c-can3-plan.md)，场地导航约束见 [赛题技术分析](competition-s21c-stp23l-analysis.md)。
