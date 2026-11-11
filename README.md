# Mango Swerve Firmware

芒果疏果机器人四舵轮底盘的独立 STM32H723 裸机工程。转向为达妙 DM6220，驱动为大疆 M3508/C620，保留遥控、BMI088、轮式/IMU里程计、S21C CAN3测距和独立疏果器作业握手。

**详细操作、硬件连接、协议、标定与验证记录统一维护在 [项目指南](docs/PROJECT_GUIDE.md)。**

## 当前状态

仓库同时包含 H723 底盘工程 `firmware/`、疏果 F407 配套工程 [firmware-elbow](firmware-elbow/README.md) 和 S21C 工程 `firmware-s21c/`。联合调试操作与协议见[CAN_WORK_DEBUG.md](docs/CAN_WORK_DEBUG.md)，详细硬件和历史测试见[项目指南](docs/PROJECT_GUIDE.md)。

底盘使用 FullMapWork 镜像，疏果端使用 ELBOW_TEST_MODE=2。底盘分配当前树位，疏果端提出 SLOW/WAIT 请求，底盘到点停稳后许可动作，疏果端定时返回、视觉复检完成并确认释放后进入下一树位。像素偏差用于阈值请求、OLED和串口观察。

VOFA命令组为7组、31条，包含全图启停、两组速度、疏果握手和必要诊断。行驶五档为0.30/0.40/0.50/0.60/0.70m/s，默认0.30。接近五档为0.05/0.10/0.15/0.20/0.25m/s，默认0.10。

本次H723与F407均已完成原固件备份、烧录与读回校验。断电后离线补测修复了相机断开后空闲状态保留READY的问题，最终F407候选需要下次上电更新。当前源码显示WAIT_CAM，激光接收与显示独立刷新。已烧录和候选镜像哈希、备份及后续验收见[部署进度](docs/CAN_WORK_DEBUG.md#2026-11-11部署进度)。

完整主机回归、49项Python测试、25种速度组合的联合闭环仿真通过，共300次树位作业。F407交接工程构建为0错误、0警告。现场激光刷新原因、完整实车联合运行及机械回位、疏果效果待上电验证。

运行完整离线检查。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File firmware/tests/run_host_tests.ps1
```

## 从哪里开始

| 目的 | 入口 |
| --- | --- |
| 理解赛题与从出发到结束的逻辑 | [完整自主路线设计](docs/PROJECT_GUIDE.md#roadmap)，含逐段坐标、作业互锁、时间预算与待测参数 |
| 按步骤调车 | [详细调试流程：上电、检查、行走与恢复](docs/PROJECT_GUIDE.md#quick-start) |
| 处理三档满杆起步易翻车 | [手动缓加减速候选及验收](docs/PROJECT_GUIDE.md#manual-ramp)，未烧录，不代表防翻已验证 |
| 复现底盘候选、查看延期项 | [底盘候选交付与测试教程](docs/PROJECT_GUIDE.md#chassis-offline-backlog)，含时间基准/PID、参数、S曲线、TRACE和停止距离边界 |
| 复现自主仿真、给队友交接 | [轨迹与日志报告](docs/PROJECT_GUIDE.md#autonomy-offline-replay)、[交付证据与现场记录表](docs/PROJECT_GUIDE.md#autonomy-handoff) |
| 排查无线距离/隔墙影响 | [静态链路对照](docs/PROJECT_GUIDE.md#debug-link) |
| 查看yaw波形与零漂 | [yaw核验步骤](docs/PROJECT_GUIDE.md#debug-yaw) |
| 做1米、往返、右墙和靠近停车 | [现场测试步骤](docs/PROJECT_GUIDE.md#debug-one-meter)与[参数及保护](docs/PROJECT_GUIDE.md#test-programs-v2) |
| 看电机、激光、疏果器怎么接 | [硬件连接与通信协议](docs/PROJECT_GUIDE.md#hardware) |
| 更换尺寸、轮组或重新对零 | [机械参数与标定](docs/PROJECT_GUIDE.md#calibration) |
| 确认“实现了”和“实测过”的区别 | [验证结果及部署标识](docs/PROJECT_GUIDE.md#verification) |

已安装的运动入口为 `D:\Vofa +\x64\Mango-VOFA-Test-OPEN-AREA.cmd`，静态入口为 `Mango-VOFA-Static.cmd`。VOFA选择 **FireWater + UDP**，本地1347、远端127.0.0.1:1348；后台桥接独占COM19、921600/8N1。组03单次1米不使用激光限位，组04–06使用对应测距条件与保护。

## 恢复手动和切回上位机

以下操作适用于已部署`8555a15`及保留同一开关策略的组合候选：先SWD=1000停车、摇杆回中；SWA=1000、VRA>1200，手动先用SWB=1000低档。将SWC切2000、**保持VRB≤1250**并保持SWD关闭，等IMU标定和电机就绪后，再把SWD切2000；看到 `TEST_CONTROL source=MANUAL manual_armed=1 vrb_required=1 vrb_ok=1` 后使用摇杆。手动期间VRB>1250即撤销许可并停止输出；若VRB松手回到1500，也会进入保护。恢复时须重新关闭SWD、回中停稳、保持VRB≤1250，再打开SWD，不能仅把VRB重新压低就续跑。

手动为自由遥控，前后/横移/旋转速度均由摇杆给定，保留电机闭环和SWB限速，**不自动修正yaw**。上位机直行、往返和沿墙测试仍使用航向保持。

本轮源码候选为MANUAL加入S曲线：平移加速/正常减速0.20/0.30m/s²、jerk 0.80m/s³，旋转0.40/0.60rad/s²、jerk 1.60rad/s³；反向先停稳，调轮期间不积累大目标。平移与旋转还进行目标耦合限幅，初值未实测。SWD、VRB撤销及故障直接进入保护停车路径，不经过普通斜坡。**普通松杆停车距离变长**：1.5m/s命令回放约5.38秒、4.03m减至零，原线性坡道约5秒、3.75m；这些是命令曲线积分，不是实际制动距离。当前车上8555a15尚无此功能。

上位机测试则先停车回中，把SWC切1500、SWD切2000，重新准备并开始；此模式VRB可回中，不参与许可判断。两种来源互斥，手动不执行旧ARM/GO；08组测试速度不改变SWB手动档。**GO后车端独立执行，关闭VOFA或断开无线不会自动停车；停车使用遥控SWD或明确STOP。** 完整检查和各测试步骤见[详细调试流程](docs/PROJECT_GUIDE.md#quick-start)。

2026-11-09已烧录正常SWC切换修复及当前手动S曲线候选，原1MB Flash备份校验、新镜像Verified OK及复位完成，实车运动验收待做；归档位于`firmware/build/deployed/20261109-mode-switch/`，本次部署接续上方8555a15历史记录。首次启用仍须观察到SWD=1000再切2000。此后正常切换HOST/MANUAL/STOP保留这次确认，切换先清零旧目标、结束旧任务；切入手动后等摇杆回中、车轮停稳、硬件正常且VRB≤1250即可接管，无需重复拨动SWD。VRB在手动中撤销、VRA许可撤销、SWA退出底盘、遥控失联、硬件故障或明确STOP仍清除确认，恢复须重新关闭再打开SWD。HOST任务仍须重新ARM/GO。

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

自主核心源码为`firmware/src/service/auto_*.c`，仅计算输入快照和输出建议，不访问设备。`tools/autonomy_report.py`读取共享C测试产生的CSV，输出路线图、状态转移及首个故障，也可分析已有手动缓加减速日志；复现命令和数据边界见[离线报告教程](docs/PROJECT_GUIDE.md#autonomy-offline-replay)。

本机仓库位于 `C:\Users\Zhanglongsheng\Projects\GDGCS\Mango-Swerve-Firmware`，主线为 `main`。本轮以 `1a4345b` 为基线在 `feat/chassis-offline-optimization` 整理候选；PR及合并状态以GitHub和 `git status` 为准。源码、权威配置、脚本、测试与命令源入Git；`build`现场证据保留在本机。详细目录约定及证据位置见[仓库维护](docs/PROJECT_GUIDE.md#repository-layout)。

## 构建和调试入口

以下命令在仓库根目录执行，使用Windows PowerShell 5.1；主机回归依赖MinGW GCC及Python 3.10以上（`py -3`），固件依赖Arm GNU GCC。主机回归包含共享C自主测试和报告生成，不连接串口或下载器。

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\firmware\tests\run_host_tests.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\firmware\build-local.ps1 -Mode Diagnostics
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\firmware\build-local.ps1 -Mode ManualStandalone
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\firmware\build-local.ps1 -Mode StraightTest
```

底盘专项为 `firmware/tests/run_chassis_tests.ps1`；完整回归会生成S曲线、四向净空与TRACE报告。详细输出路径、TRACE采集/导出及报告复现见[第9.18节](docs/PROJECT_GUIDE.md#chassis-offline-backlog)。当前整理只交付已有成果，机械实测、PID实车调参、全局定位及自主运行接入列为延期项。

只安装/更新VOFA配置时，先完全退出VOFA，再运行：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\firmware\vofa\install-vofa-panel.ps1 -ConfigureContext
```

烧录命令、七种模式的上电行为及CubeMX重新生成后的检查见[开发与烧录说明](docs/PROJECT_GUIDE.md#development)。`Ctrl+Shift+B`默认构建Diagnostics；切换固件必须显式选择Mode。编译本身不执行硬件动作；实际部署与现场验证以指南第8节为准。

代码提取自[原仓库Hephaestus工程](https://github.com/AgroTech-SCAU/Steering-Wheel-Chassis/tree/main/Hephaestus)，提取基线 `65088f94be15c69ccfa647779d7f847b06b95735`。原说明混用Atlas标题，底盘部分可参考；原仓库保持只读。本项目不包含ROS、原机械臂、吸盘或语音业务，S21C已烧录镜像保持原状，其独立维护副本现位于`firmware-s21c/`。

## 自主路线离线推演

第9章按用户提供的2026-09-10版原件和确定的顺序设计“先沿左侧直达远端，再从G7开始S形往回走到G1、底沟回门、全车退出后停车”。固定工作朝向为待机械确认的建议。完整中心路线示例30.6m；首次直行经过沟口不计沟覆盖，七沟通过与实际目标果的可见/可达覆盖分别管理。单侧机构尚不能保证够到全部目标，现有模式6的遥控许可也不能直接作为正式比赛启动方式。

最新CAD按用户更正分别记录纵向水平间隙65mm、横向55mm，两张原图已存入仓库。它们不等于激光安装偏移；净空必须按方向、姿态扣除测距基准到碰撞边缘的距离。旧共用110mm仅有左侧接触依据，不能替代四向标定，详见[包络与激光偏移](docs/PROJECT_GUIDE.md#competition-envelope)。

```powershell
py -3 -B -m unittest discover -s tools -p test_competition_route.py -v
py -3 -B tools/competition_route.py --output firmware/build/competition-analysis/route-report.json --svg docs/assets/competition-route.svg
```

Python 3.10及以上、仅标准库；24项离线回归通过。本工具不连接车辆。默认270mm方形整车包络为未确认假设，输出始终`deployment_ready=false`；默认0.30m/s、加减速度0.10m/s²、12树各8秒等假设下预算351.6秒，未满足300秒。更快加减速仅作敏感性分析，不由预算结果自动修改固件参数。详细条件、原件留存路径和队友交接见[项目指南第9章](docs/PROJECT_GUIDE.md#roadmap)。

## 此前部署记录

<details>
<summary>展开历史进度</summary>

2026-11-10全图状态：基于`d76ca21`的本地起点修复已构建并烧录为`StraightTest`模式6，OpenOCD `Verified OK`；上电后`healthy=1`、`stopped=1`、IMU标定`2000/2000`，全图仍为`NEED_ORIGIN`。尚未运行全图运动验收。此前段0失败分别由前向绝对地图参考偏差和左侧近距保护触发，日志与修复记录见[项目指南2.13](docs/PROJECT_GUIDE.md#full-map-debug)及[8.1.4](docs/PROJECT_GUIDE.md#latest-full-map-log)。

全图起点修复已在停稳登记时用新鲜前向围栏测距推算入口纵坐标，并同步入口段起点；地图内路径、未知障碍保护和地图包络没有放宽。当前镜像已烧录，但登记起点后才可开始测试。**本轮没有发送运动命令；下一轮先低速逐段验证段0，再决定是否继续。** 13通道中原始距离波形不做滤波以便诊断，控制使用的接纳距离另有三点中值；图区分说明及保护停车条件见[项目指南最新全图日志](docs/PROJECT_GUIDE.md#latest-full-map-log)。

2026-11-10参数整合：队友PR #9的两套运动学长/宽统一为0.160m，模式0–5半径为0.0501m；模式6仍为0.051m。原S21C激光板独立候选仍未烧录；底盘此轮不要求重烧S21C。

S21C激光采集板现单独放在 [firmware-s21c](firmware-s21c/README.md)，含厂家源码来源记录、本地解析修复、独立构建及测试。F103候选已通过离线测试和完整GCC编译，未烧录；H723工程仍位于`firmware/`。


遥控链路修复位于`fix/remote-link-recovery`，覆盖UART5接收卡死后的自动重启、SWA/SWD端点容差和失联后的回中停稳恢复。手动继续要求VRB≤1250，首次启用、硬件故障、明确STOP及SWA/VRA许可撤销后须重新关闭再打开SWD。手动周期日志已控制发送批次，避免日志阻塞触发50ms超时。恢复步骤和验证边界见[项目指南2.1节](docs/PROJECT_GUIDE.md#21-同一固件的手动与上位机开关)。

**G7末端修复候选（2026-11-10）：** 实车在段2因`MAP_ENVELOPE`停止。G7改为前激光约束北围栏，排除跨R6垄端的后向定位观测及G7成对居中；后向近障保护保留。新增闭环回归和全图回放通过，96源固件编译通过。保留12个实测树位及每树停2秒；详见[指南8.1.21](docs/PROJECT_GUIDE.md)。合并后再烧录，未宣称实车已跑通。

**最新已烧录版本（Verified OK）：前后激光纠偏＋12个实测树位，每树停稳等待2秒；G7只通行，不启动疏果器。VOFA和新版UDP桥接已恢复。** 坐标配置在firmware/src/service/full_map_tree_positions.h，操作/记录见指南8.1.19；同时新增底盘控制失败的具体错误快照，见8.1.20。最终部署状态以该节最新烧录记录为准。

**前后纠偏（已并入最新树位部署）。** 三帧确认可靠的成对垄边观测后修正地图Y并前后居中；前近向后、后近向前，纠偏期间降低横移速度，保留100mm保护。沟口、短R4缺口、异常/冻结读数退出成对修正。新增TEST_MAP_AISLE日志，需配套新桥接；详见指南8.1.18。实车效果待验收。

**最新已烧录：段末横向验收也放宽至±20mm（Verified OK）**，与沿程±20mm、航向±3°配套，解决G7约10mm侧偏仍被旧8mm窗口卡住的问题；只放宽到点验收，行进修正及包络保护保留。车端已包含此项，沿程±20mm、横向±20mm、段末±3°；见指南8.1.16。

**最新已部署：连续全图、五档0.20/0.30/0.40/0.50/0.60m/s（默认0.20）、P直达G7、入口不单独限低速、段末±3°、终点容差20mm、100mm激光保护。OpenOCD Verified OK，VOFA与新版桥接已启动，95命令已安装。** 最新操作见项目指南8.1.14，下方旧部署/逐段记录作为历史保留。

**历史候选（现已并入最新部署）：P直达G7，取消入口中途停车及入口专用低速；首段采用设定全图速度并正常加减速。段末航向验收放宽至±3°，行进航向修正保留原精度。** 几何记录保留，运行段号从0直接进入2；详见项目指南8.1.13。车端当前仍为100mm保护、旧停车点及旧±1.15°验收版本。

**最新部署：全图近障门限为激光接纳读数≤100mm，动态制动预算以100mm起算，不叠加车边偏置；已烧录并经OpenOCD Verified OK。归档firmware/build/deployed/20261110-full-map-range100/。** 此前110mm记录属于上一部署；带垄前后居中功能仍待实现。

最新全图110mm版本已烧录并Verified OK，镜像归档firmware/build/deployed/20261110-full-map-range110/。全图静态门限为激光接纳读数≤110mm，不叠加车边偏置；动态制动预算以110mm为基线。复位后停稳、IMU完成标定，但遥控RC_OFFLINE且后距约100–107mm，测试前需恢复遥控并清开后方。此前224mm门限及530mm入口要求来自含义误解，已撤销。右激光仅确认最远端G7中心再横移，沟口识别尚未实现。

**2026-11-10最新部署：** 全图起点修复版`StraightTest`模式6已在14:49烧录并经OpenOCD `Verified OK`。启动记录为`healthy=1`、`stopped=1`、IMU标定`2000/2000`，地图等待`NEED_ORIGIN`；尚未用新版本行走验收。保存镜像、原Flash备份及保护停车说明见[项目指南8.1.4](docs/PROJECT_GUIDE.md#latest-full-map-log)。VOFA原始激光通道会保留65000量级尖峰；控制用通道是对应的接纳/中值通道，见[项目指南8.1.5](docs/PROJECT_GUIDE.md#latest-full-map-log)。

2026-11-09本机已拉取PR #8的`51fced7`，并复核D盘原厂S21C/STP23L四路源码：确认时间戳重复消费、校验外提交、汇总覆盖/累加及逐路旧值持续发送风险。DM侧滤波与时效专项通过，但不能识别采集板反复发送的范围内旧值；队友电脑的S21C修复工程尚待取得；本机另建候选见上方，未烧录。证据、修复边界与台架步骤见项目指南5.2.1。

2026-11-09最新底盘为`firmware/build/deployed/20261109-paused-guard/`，已Verified OK。修复激光等待中单帧近距误锁终止，保持零输出确认及恢复判稳。完整回归与构建通过，部署后4轮4m贴墙往返均完成，返程最大右墙距偏离41–85mm，方向标定和精度仍待完善。COM12/UDP和波形持续接收，S21C候选未烧录，原始测距异常仍在排查。以下为较早版本。

2026-11-09当前部署`firmware/build/deployed/20261109-laser-median/`，三点中值、近距暂停恢复、测距前视限速及低幅反向反馈100ms判稳已烧录Verified OK。完整回归和构建通过，UDP及13通道波形持续接收，新版运动效果待复测。后方原始测距异常仍需查采集端原因。以下为同日较早版本。

2026-11-09最新为`firmware/build/deployed/20261109-laser-recovery/`。已烧录并Verified OK，包含按帧间隔判别突变、独立样本确认、近距直通保护，以及关键激光短暂过期后停车判稳续行。新增13通道激光波形及暂停事件，COM12/UDP1348至VOFA1347已恢复。完整回归和构建通过，运动效果待实车复测。参数及恢复条件见项目指南最新记录。下方为同日较早版本。

2026-11-09最新部署为`StraightTest`，见`firmware/build/deployed/20261109-guard-stop/`。速度五档0.50/1.00/1.50/1.80/2.00m/s，默认0.50；加速度可在VOFA停车时选择0.10/0.20/0.30/0.50m/s²，默认0.20。增强贴墙及航向修正，保护暂停等待测距有效、连续停稳200ms后续行，正常方向保护保留。完整回归、93源构建及烧录Verified OK通过，实车新参数待复测。UDP1348持续桥接COM12/921600至VOFA1347，支持VOFA重连。手动恢复已由用户确认，VRB持续许可保留。详细参数和历史记录见项目指南。

截至2026-11-07，车端已部署PR #4合并提交 `8555a15` 的 `StraightTest` 模式6、协议2组合固件；原1MB Flash备份及新镜像校验通过。包含**仅手动模式要求VRB≤1250**、手动自由运动不自动修正yaw，以及往返等待段保留航向参考。启动日志确认IMU标定完成，恢复桥接后约87秒收到872/872条完整状态；运动效果仍待验收。默认构建仍是 `Diagnostics`，构建默认值不代表车上正在运行的固件。

| 项目 | 已有结果与边界 |
| --- | --- |
| 自主路线 | 模式6已接入17段空场全图路线与里程/激光定位；起点修复已部署，段0仍待低速实车验收，尚未完成全程 |
| 自主基础代码（离线交付基线） | 四向净空、观测校验、线段控制、17段任务状态机及独立作业事务；`37752cf`阶段9个共享C场景、32项Python回归及三模式各91源构建通过。尚未接入/烧录/实测；历史基线见指南9.15–9.17，本轮结果见9.18 |
| 底盘基础 | 电机通信、轮位、架空前后左右方向、独立零位回位及低速航向保持已有现场记录 |
| 原单次1米 | 两轮正常到点；用户地面近似实测约103.2cm，未据此修改轮径，重复性待测 |
| 新测试 | 1/2/4米普通往返、右墙定距往返、前方靠近停车已实现、编译并部署，完整运动验收待做 |
| 测试速度 | 0.05 / 0.08 / 0.10 / 0.20 / 0.30 / 0.50m/s，默认0.10；靠近停车最高0.05 |
| 无线状态传输 | 64字节分块、1ms间隔后，已部署f5bc3da组合固件约20秒采集206/206条状态完整；远距/隔墙与运动期间仍待对照 |
| VOFA面板 | 现场运行96条命令（含全图第12组）；UDP桥接处理一次性编号，GO后无需心跳或状态持续回传，17类日志可分别开关。激光原始/接纳波形尚需使用清晰名称区分 |
| 控制切换 | 组合固件SWC：1000停车、1500上位机、2000手动；SWD统一1000停车/2000许可；手动还须VRB≤1250，上位机不要求VRB |
| 本轮底盘候选 | 已实现时间基准/PID参考周期归一化、数值保护、参数档案、MANUAL S曲线/复合运动限幅、舵轮低速与对齐滞回、512帧TRACE及回放。完整C/PowerShell回归、43项Python测试与三模式各93源构建通过，独立审查未发现阻塞合入问题；尚未烧录，见指南9.18 |
| 隔离边界 | 切换取消旧任务；手动不受上位机心跳/串口错误影响。模式0/5也使用修改后的共享底盘/电机服务，不能再宣称其BIN或底层行为完全不变；仅模式6接入新S曲线和TRACE |
| 往返审查修复 | 等待段保留原航向参考，避免误报航向偏差及返程失去角度环；主循环回归通过且已部署，实车往返待验收 |

</details>
