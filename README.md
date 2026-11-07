# Mango Swerve Firmware

芒果疏果机器人四舵轮底盘的独立 STM32H723 裸机工程。转向为达妙 DM6220，驱动为大疆 M3508/C620，保留遥控、BMI088、轮式/IMU里程计、S21C CAN3测距和独立疏果器作业握手。

**详细操作、硬件连接、协议、标定与验证记录统一维护在 [项目指南](docs/PROJECT_GUIDE.md)。**

## 当前状态

截至2026-11-07，车端已部署 `StraightTest` 模式6、协议2，本次新增SWC双来源互斥与无需心跳的车端运行。已备份并校验原1MB Flash，写入后 `Verified OK`。默认构建仍是 `Diagnostics`，构建默认值不代表车上正在运行的固件。

| 项目 | 已有结果与边界 |
| --- | --- |
| 底盘基础 | 电机通信、轮位、架空前后左右方向、独立零位回位及低速航向保持已有现场记录 |
| 原单次1米 | 两轮正常到点；用户地面近似实测约103.2cm，未据此修改轮径，重复性待测 |
| 新测试 | 1/2/4米普通往返、右墙定距往返、前方靠近停车已实现、编译并部署，完整运动验收待做 |
| 测试速度 | 0.05 / 0.08 / 0.10 / 0.20 / 0.30 / 0.50m/s，默认0.10；靠近停车最高0.05 |
| 无线状态传输 | 64字节分块、1ms间隔后，最终组合固件约20秒采集206/206条状态完整；远距/隔墙与运动期间仍待对照 |
| VOFA面板 | 10组74按钮，UDP桥接处理一次性编号；GO后无需心跳或状态持续回传，17类日志可分别开关 |
| 控制切换 | 组合固件SWC：1000停车、1500上位机、2000手动；SWD统一1000停车/2000许可，VRB可回中 |
| 隔离边界 | 切换取消旧任务；手动不受上位机心跳/串口错误影响；旧独立手动模式BIN哈希保持一致 |

## 从哪里开始

| 目的 | 入口 |
| --- | --- |
| 按步骤调车 | [详细调试流程：上电、检查、行走与恢复](docs/PROJECT_GUIDE.md#quick-start) |
| 排查无线距离/隔墙影响 | [静态链路对照](docs/PROJECT_GUIDE.md#debug-link) |
| 查看yaw波形与零漂 | [yaw核验步骤](docs/PROJECT_GUIDE.md#debug-yaw) |
| 做1米、往返、右墙和靠近停车 | [现场测试步骤](docs/PROJECT_GUIDE.md#debug-one-meter)与[参数及保护](docs/PROJECT_GUIDE.md#test-programs-v2) |
| 看电机、激光、疏果器怎么接 | [硬件连接与通信协议](docs/PROJECT_GUIDE.md#hardware) |
| 更换尺寸、轮组或重新对零 | [机械参数与标定](docs/PROJECT_GUIDE.md#calibration) |
| 确认“实现了”和“实测过”的区别 | [验证结果及部署标识](docs/PROJECT_GUIDE.md#verification) |

已安装的运动入口为 `D:\Vofa +\x64\Mango-VOFA-Test-OPEN-AREA.cmd`，静态入口为 `Mango-VOFA-Static.cmd`。VOFA选择 **FireWater + UDP**，本地1347、远端127.0.0.1:1348；后台桥接独占COM19、921600/8N1。组03单次1米不使用激光限位，组04–06使用对应测距条件与保护。

## 恢复手动和切回上位机

先SWD=1000停车、摇杆回中；SWA=1000、VRA>1200，手动先用SWB=1000低档。将SWC切2000并保持SWD关闭，等IMU标定和电机就绪后，再把SWD切2000；看到 `TEST_CONTROL source=MANUAL manual_armed=1` 后使用摇杆。STOP、失联或切换后需重新关闭SWD、回中停稳、再打开。

上位机测试则先停车回中，把SWC切1500、SWD切2000，重新准备并开始。两种来源互斥，手动不执行旧ARM/GO；08组测试速度不改变SWB手动档。**GO后车端独立执行，关闭VOFA或断开无线不会自动停车；停车使用遥控SWD或明确STOP。** 完整检查和各测试步骤见[详细调试流程](docs/PROJECT_GUIDE.md#quick-start)。

## 目录与维护边界

```text
README.md                    项目入口与当前状态
docs/PROJECT_GUIDE.md        唯一持续维护的技术指南
.vscode/tasks.json           仓库根目录的VS Code任务
firmware/
  robot.ioc                  CubeMX配置
  .eide/、robot.code-workspace EIDE配置和工作区入口
  Core/、Drivers/            生成代码、HAL和CMSIS依赖
  src/app/                   模式选择、入口、测试硬件适配
  src/service/               底盘、CAN3路由、测距、测试状态机、作业握手
  src/domain/、device/       运动学/控制算法、电机/IMU/遥控驱动
  src/platform/、infra/      HAL适配和基础设施
  tests/                     主机回归与CubeMX一致性检查
  vofa/                      命令源、安装和联合启动脚本
  build-local.ps1            按当前源码完整构建
  flash-local.ps1            构建、原板备份/校验、烧录/校验
  straight-console.ps1       串口与UDP桥接、会话和日志控制
  monitor-local.ps1          独立串口采集；不能与桥接同时占用串口
  build/                     本地产物、原始日志、备份和部署归档；Git忽略
```

本机仓库位于 `C:\Users\Zhanglongsheng\Projects\GDGCS\Mango-Swerve-Firmware`，主线为 `main`，本次按要求在临时功能分支提交PR。源码、权威配置、脚本、测试与命令源入Git；`build`现场证据保留在本机。详细目录约定及证据位置见[仓库维护](docs/PROJECT_GUIDE.md#repository-layout)。

## 构建和调试入口

以下命令在仓库根目录执行，使用Windows PowerShell 5.1；主机回归依赖MinGW GCC，固件依赖Arm GNU GCC。

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\firmware\tests\run_host_tests.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\firmware\build-local.ps1 -Mode StraightTest
```

只安装/更新VOFA配置时，先完全退出VOFA，再运行：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\firmware\vofa\install-vofa-panel.ps1 -ConfigureContext
```

烧录命令、七种模式的上电行为及CubeMX重新生成后的检查见[开发与烧录说明](docs/PROJECT_GUIDE.md#development)。`Ctrl+Shift+B`默认构建Diagnostics；切换固件必须显式选择Mode。编译本身不执行硬件动作；实际部署与现场验证以指南第8节为准。

代码提取自[原仓库Hephaestus工程](https://github.com/AgroTech-SCAU/Steering-Wheel-Chassis/tree/main/Hephaestus)，提取基线 `65088f94be15c69ccfa647779d7f847b06b95735`。原说明混用Atlas标题，底盘部分可参考；原仓库保持只读。本项目不包含ROS、原机械臂、吸盘或语音业务，S21C已烧录的采集程序不在本仓库修改。
