# Mango Swerve Firmware

芒果疏果机器人四舵轮底盘的独立 STM32H723 裸机工程。转向为达妙 DM6220，驱动为大疆 M3508/C620，保留遥控、BMI088、轮式/IMU里程计、S21C CAN3测距和独立疏果器作业握手。

**详细操作、硬件连接、协议、标定与验证记录统一维护在 [项目指南](docs/PROJECT_GUIDE.md)。**

## 当前状态

截至2026-11-07，车端已部署PR #4合并提交 `8555a15` 的 `StraightTest` 模式6、协议2组合固件；原1MB Flash备份及新镜像校验通过。包含**仅手动模式要求VRB≤1250**、手动自由运动不自动修正yaw，以及往返等待段保留航向参考。启动日志确认IMU标定完成，恢复桥接后约87秒收到872/872条完整状态；运动效果仍待验收。默认构建仍是 `Diagnostics`，构建默认值不代表车上正在运行的固件。

| 项目 | 已有结果与边界 |
| --- | --- |
| 自主路线 | 按用户要求先沿左侧直达远端，再G7至G1下降S形、底沟回门；完整状态机已完成设计，离线工具可复现几何/时间检查，尚未接入STM32 |
| 底盘基础 | 电机通信、轮位、架空前后左右方向、独立零位回位及低速航向保持已有现场记录 |
| 原单次1米 | 两轮正常到点；用户地面近似实测约103.2cm，未据此修改轮径，重复性待测 |
| 新测试 | 1/2/4米普通往返、右墙定距往返、前方靠近停车已实现、编译并部署，完整运动验收待做 |
| 测试速度 | 0.05 / 0.08 / 0.10 / 0.20 / 0.30 / 0.50m/s，默认0.10；靠近停车最高0.05 |
| 无线状态传输 | 64字节分块、1ms间隔后，已部署f5bc3da组合固件约20秒采集206/206条状态完整；远距/隔墙与运动期间仍待对照 |
| VOFA面板 | 10组74按钮，UDP桥接处理一次性编号；GO后无需心跳或状态持续回传，17类日志可分别开关 |
| 控制切换 | 组合固件SWC：1000停车、1500上位机、2000手动；SWD统一1000停车/2000许可；手动还须VRB≤1250，上位机不要求VRB |
| 本轮手动起停候选 | MANUAL加入向量加减速、反向停稳和调轮期间不积累目标；完整主机回归通过，StraightTest和ManualStandalone各85源构建成功，尚未烧录 |
| 隔离边界 | 切换取消旧任务；手动不受上位机心跳/串口错误影响；旧独立手动模式BIN哈希保持一致 |
| 往返审查修复 | 等待段保留原航向参考，避免误报航向偏差及返程失去角度环；主循环回归通过且已部署，实车往返待验收 |

## 从哪里开始

| 目的 | 入口 |
| --- | --- |
| 理解赛题与从出发到结束的逻辑 | [完整自主路线设计](docs/PROJECT_GUIDE.md#roadmap)，含逐段坐标、作业互锁、时间预算与待测参数 |
| 按步骤调车 | [详细调试流程：上电、检查、行走与恢复](docs/PROJECT_GUIDE.md#quick-start) |
| 处理三档满杆起步易翻车 | [手动缓加减速候选及验收](docs/PROJECT_GUIDE.md#manual-ramp)，未烧录，不代表防翻已验证 |
| 排查无线距离/隔墙影响 | [静态链路对照](docs/PROJECT_GUIDE.md#debug-link) |
| 查看yaw波形与零漂 | [yaw核验步骤](docs/PROJECT_GUIDE.md#debug-yaw) |
| 做1米、往返、右墙和靠近停车 | [现场测试步骤](docs/PROJECT_GUIDE.md#debug-one-meter)与[参数及保护](docs/PROJECT_GUIDE.md#test-programs-v2) |
| 看电机、激光、疏果器怎么接 | [硬件连接与通信协议](docs/PROJECT_GUIDE.md#hardware) |
| 更换尺寸、轮组或重新对零 | [机械参数与标定](docs/PROJECT_GUIDE.md#calibration) |
| 确认“实现了”和“实测过”的区别 | [验证结果及部署标识](docs/PROJECT_GUIDE.md#verification) |

已安装的运动入口为 `D:\Vofa +\x64\Mango-VOFA-Test-OPEN-AREA.cmd`，静态入口为 `Mango-VOFA-Static.cmd`。VOFA选择 **FireWater + UDP**，本地1347、远端127.0.0.1:1348；后台桥接独占COM19、921600/8N1。组03单次1米不使用激光限位，组04–06使用对应测距条件与保护。

## 恢复手动和切回上位机

以下为新增VRB保护版本的操作，须烧录该版本后使用：先SWD=1000停车、摇杆回中；SWA=1000、VRA>1200，手动先用SWB=1000低档。将SWC切2000、**保持VRB≤1250**并保持SWD关闭，等IMU标定和电机就绪后，再把SWD切2000；看到 `TEST_CONTROL source=MANUAL manual_armed=1 vrb_required=1 vrb_ok=1` 后使用摇杆。手动期间VRB>1250即撤销许可并停止输出；若VRB松手回到1500，也会进入保护。恢复时须重新关闭SWD、回中停稳、保持VRB≤1250，再打开SWD，不能仅把VRB重新压低就续跑。

手动为自由遥控，前后/横移/旋转速度均由摇杆给定，保留电机闭环和SWB限速，**不自动修正yaw**。上位机直行、往返和沿墙测试仍使用航向保持。

本轮源码候选为MANUAL增加命令斜率：平移加速/正常减速0.20/0.30m/s²，旋转0.40/0.60rad/s²；反向先停稳，调轮期间不积累大目标。SWD、VRB撤销及故障仍直接进入原停车路径。初值未实测，普通松杆停车会延长距离，例如已达1.5m/s时命令曲线约需5秒/3.75m减至零，不能当急停。当前车上8555a15尚无此功能。

上位机测试则先停车回中，把SWC切1500、SWD切2000，重新准备并开始；此模式VRB可回中，不参与许可判断。两种来源互斥，手动不执行旧ARM/GO；08组测试速度不改变SWB手动档。**GO后车端独立执行，关闭VOFA或断开无线不会自动停车；停车使用遥控SWD或明确STOP。** 完整检查和各测试步骤见[详细调试流程](docs/PROJECT_GUIDE.md#quick-start)。

## 目录与维护边界

```text
README.md                    项目入口与当前状态
docs/PROJECT_GUIDE.md        唯一持续维护的技术指南
docs/assets/                规则原图与可复现的路线图
tools/competition_route.py  独立离线路线/扫掠/时间核查，不控制硬件
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

本机仓库位于 `C:\Users\Zhanglongsheng\Projects\GDGCS\Mango-Swerve-Firmware`，主线为 `main`；PR #4已合并，临时功能分支已清理。源码、权威配置、脚本、测试与命令源入Git；`build`现场证据保留在本机。详细目录约定及证据位置见[仓库维护](docs/PROJECT_GUIDE.md#repository-layout)。

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

## 自主路线离线推演

第9章按用户提供的2026-09-10版原件和确定的顺序设计“先沿左侧直达远端，再从G7开始S形往回走到G1、底沟回门、全车退出后停车”。固定工作朝向为待机械确认的建议。完整中心路线示例30.6m；首次直行经过沟口不计沟覆盖，七沟通过与实际目标果的可见/可达覆盖分别管理。单侧机构尚不能保证够到全部目标，现有模式6的遥控许可也不能直接作为正式比赛启动方式。

最新CAD按用户更正分别记录纵向水平间隙65mm、横向55mm，两张原图已存入仓库。它们不等于激光安装偏移；净空必须按方向、姿态扣除测距基准到碰撞边缘的距离。旧共用110mm仅有左侧接触依据，不能替代四向标定，详见[包络与激光偏移](docs/PROJECT_GUIDE.md#competition-envelope)。

```powershell
py -3 -B -m unittest discover -s tools -p test_competition_route.py -v
py -3 -B tools/competition_route.py --output firmware/build/competition-analysis/route-report.json --svg docs/assets/competition-route.svg
```

Python 3.10及以上、仅标准库；24项离线回归通过。本工具不连接车辆。默认270mm方形整车包络为未确认假设，输出始终`deployment_ready=false`；默认0.30m/s、加减速度0.10m/s²、12树各8秒等假设下预算351.6秒，未满足300秒。更快加减速仅作敏感性分析，不由预算结果自动修改固件参数。详细条件、原件留存路径和队友交接见[项目指南第9章](docs/PROJECT_GUIDE.md#roadmap)。
