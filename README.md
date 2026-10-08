# Mango Swerve Firmware

芒果疏果机器人舵轮底盘的独立 STM32H723 电控工程。当前固件包括四舵轮驱动、遥控手动控制、BMI088 和轮式里程计；S21C/STP-23L 测距、CAN3、自主地图遍历和作业机构接口尚待实现。

**从这里阅读：[底盘电控项目总览与目录](docs/PROJECT_GUIDE.md)。** 赛题、硬件连接、源代码阅读路线、机械尺寸、测距协议、CAN3 接入、导航方案、联调顺序及待机械队友确认的问题都集中在这一份文档维护。

## 工程入口

| 内容 | 位置 |
| --- | --- |
| EIDE 工程与 CubeMX 配置 | `firmware/`、`firmware/robot.ioc`；VS Code EIDE 选择 Debug/Release，目标 STM32H723VGT6、ARM GNU GCC |
| 启动与调度 | `firmware/Core/Src/main.c`、`firmware/src/app/entry.h` |
| 电机装配与底盘执行 | `firmware/src/service/assemble/assemble_chassis.c`、`firmware/src/service/chassis.c` |
| 四舵轮运动学 | `firmware/src/domain/steer_wheel_kine.c` |
| 电机协议与里程计 | `firmware/src/device/bus_motor/`、`firmware/src/service/odom.c` |

本工程取自 [AgroTech-SCAU/Steering-Wheel-Chassis](https://github.com/AgroTech-SCAU/Steering-Wheel-Chassis) 的 `Hephaestus/chassis_control_code`，基线提交 `65088f94be15c69ccfa647779d7f847b06b95735`；原仓库未修改。原 Hephaestus 文档可作为驱动代码参考，车体几何和赛题方案以本仓库的[项目总览](docs/PROJECT_GUIDE.md)为准。
