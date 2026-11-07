# Mango Swerve Firmware

芒果疏果机器人四舵轮底盘的独立 STM32H723 裸机工程，转向使用达妙 DM6220，驱动使用大疆 M3508/C620。范围包括底盘电机、遥控、BMI088、轮式/IMU 里程计，以及 S21C 的 CAN3 四路测距接收。疏果装置通过已定义的 CAN3 作业握手协同，其电机控制由独立工程负责。

**技术文档统一维护在 [项目指南](docs/PROJECT_GUIDE.md)。**

**新测试指令扩展（2026-11-07，已烧录，待新动作验收）。** 面板现有10组、74个按钮：新增单程1/2/4米的普通往返与右墙定距往返、前方靠近停车、贴边记录，以及0.05/0.08/0.10/0.20/0.30/0.50m/s速度档。默认0.10m/s，靠近停车不超过0.05m/s。仅模式6扩展；正常手动模式独立保留。左侧贴边实测108–109mm，按用户要求以110mm作为四向共用初始碰撞基准；新测试的基础保护线130mm，闭合方向再加测距延迟与制动预算，前方停车目标默认210mm原始距离。原单次1米组仍不启用激光限位。本次已完成1MB备份校验、写入和Verified OK，COM19确认proto=2及IDLE/零速待命；新动作运动效果尚未实机验证，不会自动启动。操作、协议与验证见[扩展测试与UDP日志控制](docs/PROJECT_GUIDE.md#test-programs-v2)。

**日志与波形开关。** 09组可按17类分别开启/关闭，也提供安静、默认简洁、全部开启三个预设。默认状态/流程最多1Hz，波形关闭；要看yaw曲线，在09组开启FireWater波形。开关只控制UDP发往VOFA的数据，后台仍保存完整串口原文，ACK和主动查询回复始终可见。通用入口为`D:\Vofa +\x64\Mango-VOFA-Test-OPEN-AREA.cmd`，原1m入口作为兼容别名保留。

模式6日志增加64字节分块与1ms间隔，处理无线串口120字节断尾现象；已重新烧录校验，20秒静态采集200条状态全部完整，运动中链路仍待复测。正常手动BIN哈希保持一致。桥接修正了重试ARM时旧完成/故障状态抢先清除等待的问题，超时提示分别给出状态更新或GO确认年龄，原超时停车保护保留。检查结果与现场验证见项目指南第4、8节。

**当前场地：先做单次1米（2026-11-07，已烧录并首次行走）。** 目标1.00m，最高0.10m/s，使用轮速里程积分判断距离、IMU航向角环修正方向，并保留缓起步/近终点减速。激光仅记录，不参与限位、避障或停车。主机回归及83源文件ARM构建通过，烧录Verified OK；遥控许可曾确认`healthy=1、blocked=NONE`。用户首轮通过SWD中止；后续token=2/3均返回`TARGET_REACHED`，最终里程估计分别0.991m/0.992m。用户实测约103.2cm，较100cm目标多约3.2%，反馈“挺直”。该次近似测量尚未绑定具体编号，保留现有参数，后续重复测量再校准。运动入口为`D:\Vofa +\x64\Mango-VOFA-1m-OPEN-AREA.cmd`，桥接确认车端`target_m=1.00`才允许准备/开始。

**VOFA中文命令维护。** 命令源、UDP桥接、安装与联合启动器均在`firmware/vofa/`和`firmware/straight-console.ps1`内维护。入口会备份并设置默认FireWater/UDP和命令文件；VOFA点“连接”后即可发中文命令。Static拒绝ARM/GO，可观察、控制日志显示并在停稳时设置测试参数。实际保存目录为`AppData/Local/vofa+/100/context`，只替换D盘JSON不会自动载入；须退出VOFA后由联合入口安装。完全退出VOFA会关闭桥接并尝试STOP。此前13按钮面板的实际双向查询已验证，新面板在此基础上扩展。详见[维护与操作说明](docs/PROJECT_GUIDE.md#test-programs-v2)。

**1米测试遥控许可更新。** 仅模式6改用SWD=2000许可、SWD=1000停止，VRB可以松手回中；SWA=1000、SWC不为1000、VRA>1200、摇杆回中及其他反馈/链路检查保留。新固件已烧录，实测VRB=1500时许可通过，首轮SWD撤销也确实触发停车。正常手动模式的BIN与修改前SHA256完全相同。上电及“准备”阶段保持当前轮向；“开始”后计算直行轮向，并由现有转向到位检查放行驱动，不自动续跑。

**静态漂移排查记录（2026-11-07）。** 测试模式新增遥控、四轮、三路CAN、激光、IMU及5秒漂移统计，4通道波形保持10Hz。仅StraightTest改用本机启动测得零偏，并修正新帧消费标志及反馈计时误判。增强版已编译、备份烧录并静态复测：连续上电约15～16分钟后，经用户确认静止，599点/59.8秒的yaw净变-0.028126°，全段范围0.109995°；IMU就绪、无更新错误。此前截图估算约0.21°/s的持续漂移明显改善，小角度旋转比例、冷启动重复性及更长时间漂移仍待验证。该次静态采集时遥控离线，未执行运动；最新行走状态见上文。详见[日志字段与实测证据](docs/PROJECT_GUIDE.md#状态日志与静态漂移排查2026-11-07)。

## 单次1米测试固件（2026-11-07，已烧录待测试）

新增`StraightTest`（模式6）：上电待命，串口`ARM`后用一次性编号`GO <token>`启动，锁定起步航向，最高0.10m/s、缓起步/近终点减速，停稳后结束，不自动往返。`STOP`中止；运行中须每200ms左右发送`HB <token>`，超过750ms无有效心跳则故障并请求停车。保留遥控许可与电机/IMU新鲜度检查。右侧靠墙，日志记录接口2测距，不自动横移。

构建：`powershell -NoProfile -ExecutionPolicy Bypass -File .\firmware\build-local.ps1 -Mode StraightTest`。产物为`firmware/build/StraightTest/robot.hex`及对应ELF。用户授权后已通过无线Horco DAP完成原1MB备份、备份校验、写入、Verified OK和复位；随后用户通过面板启动首轮并用SWD停止。默认构建仍为Diagnostics，实物当前为模式6。

距离以102mm名义轮径估算，须用地面标尺核验真实1m；名义半径仅在测试模式使用。无线SWD备份/写入/校验、COM19双向指令及许可撤销停车已有实测，已完成两轮到点，用户单次近似测量约103.2cm，重复性仍待验证。协议、Windows PowerShell 5.1交互工具及停机条件见[测试操作说明](docs/PROJECT_GUIDE.md#指令启动的单次1米测试固件2026-11-07)。先完成单次1m，再决定往复测试。

测试模式新增10Hz FireWater帧`FW: yaw_deg,ref_deg,error_deg,wz_rad_s`（实际发送数值）。前3路为度，第4路为rad/s。VOFA+选择FireWater并绑定第0路为yaw；命名控件不等于解析文本日志。执行测试同时看波形时，使用`straight-console.ps1 -Port COM19 -WavePort 1347`独占串口、发送心跳并将合法4通道帧转发到本机UDP；VOFA+改用UDP监听端口1347，避免抢占COM19。

波形若为0，先检查通道“增益与偏移”：增益应为1、偏移为0。本机VOFA保存的yaw配置曾为scale=0，与图上全0现象一致；另需Auto适配实际约-30°的范围。状态日志不删除，UDP工具同时转发TEST_STATUS/TEST_ACK/TEST_READY等普通文本，原始串口全文仍保存到日志文件。此项仅修改主机工具，无需重烧固件。

**本轮拉取与今天安排（2026-11-07）：**已同步PR #3合并提交`2035c58`，并核对正常零位补偿、独立遥控和航向保持已接入。今天按[单侧走廊长距离直行计划](docs/PROJECT_GUIDE.md#今天的单侧走廊直行计划2026-11-07)执行：车身对墙平行 → 低速2m量化复测 → 分原因调参 → 延长距离。单墙测距闭环尚未实现，先将墙作为外部验收基准。

本轮离线检查修复不存在E盘时的工具发现、入口stddef.h依赖及两处MinGW浮点断言兼容性；主机回归及ManualStandalone/Manual/Diagnostics各81源文件构建通过。本轮未烧录或执行硬件动作。下文的实机结论为仓库合入的既有现场记录。

## 早期遥控联调记录（2026-11-07，当前固件见上文）

该阶段完成底盘 CAN 通信检查、遥控恢复和三档速度调整，使用 `ManualStandalone` 独立遥控固件。

### CAN 通信检查与联调

| 总线 | 接入设备 | 今天检查的内容 |
| --- | --- | --- |
| CAN1 | 四个达妙转向电机 | 核对电机 ID 1～4、反馈 ID `0x11`～`0x14`，完成初始化、逐轮回位与状态检查，四个电机均能转向 |
| CAN2 | 四个 C620/M3508 驱动电机 | 核对反馈 ID `0x201`～`0x204`，读取转速和电流，完成四轮驱动及停车检查 |
| CAN3 | S21C 测距与肘击作业协议 | 检查 S21C 的 `0x601` 四路测距报文，以及肘击请求 `0x610`、底盘回复 `0x611` 的握手与作业停车逻辑 |

- 定位遥控无动作的原因，实机原先运行 `Diagnostics`，该模式关闭应用层电机发送。切换手动模式后，继续检查肘击握手对行驶许可的影响。
- 增加 `ManualStandalone` 模式，用于底盘单独遥控。`Manual` 模式继续用于肘击联动，接入握手、减速、停车、放行及故障停车逻辑。
- 完成 CAN 配置、收帧解包和肘击协议互操作的主机回归。肘击装置与底盘的完整实车联动仍待验证。
- 完善构建、备份、烧录与校验入口。下载器使用 HID 后端，并启用 CMSIS-DAP 兼容处理，完成固件更新。

### 遥控恢复与三档速度调整

- 恢复遥控输入与行驶许可，现场确认前、后、左、右方向正确，四轮均能驱动并正常停车。
- 确认轮位为 ID1左前、ID2右前、ID3右后、ID4左后，将机械零位补偿接入正常控制，并重新标定左前轮零位。
- 接入 IMU 航向保持，低速直行左偏较此前明显改善。当前采用 `Kp=0.8`、`Kd=0.1`，纠偏限幅为 `±0.03 rad/s`。
- 修复三档速度被统一调试上限压成同一速度的问题，恢复 SWB 三档调节，并加入开关端点容差。
- 根据现场速度偏快的反馈，将最终平移三档设为 **0.4 / 0.8 / 1.5 m/s**，旋转三档设为 **1.5 / 3.0 / 6.0 rad/s**。SWB 原始值依次为 1000、1500、2000。
- 完成三档速度、斜向合速度限制、停车及遥控失联后重新使能的回归验证。实机已确认三个 SWB 档位均能识别。

### 今日验证结果与后续工作

`ManualStandalone` 和 `Manual` 构建通过，主机回归通过。最终降速版本已烧录并校验，日志确认遥控在线、停车命令为零、四轮转速为零。上电或遥控失联后，先关闭再开启 VRB 才能恢复行驶许可。

后续继续验证降速后的实际速度手感、中高速航向保持、车体尺寸与有效轮径、里程计精度，以及肘击装置 CAN 联动。当前直行改善来自低速实车反馈，残余偏角尚未量化。

今日主要记录在 `firmware/build/reduced-speed-tests.log`、`firmware/build/reduced-speed-flash.log`、`firmware/build/reduced-speed-final-status.txt` 和 `firmware/build/three-speed-switch-retest.txt`。构建脚本默认选择 `Diagnostics`，烧录独立遥控固件使用 `./firmware/flash-local.ps1 -Mode ManualStandalone`。

## 独立遥控调试（2026-11-07）

`firmware/flash-local.ps1 -Mode ManualStandalone` 备份原固件后构建、烧录并校验底盘独立遥控固件。该模式不注册肘击作业端点，遥控行驶不依赖肘击握手。`-Mode Manual` 用于肘击联动，保持握手与作业停车要求。下载器默认使用已实测可用的 HID 后端，可通过 `-DapBackend` 选择。

初轮调试采用平移0.10 m/s、旋转0.30 rad/s的限制，完成方向核对后恢复原三档速度。用户随后反馈速度偏快，当前将三档降低约20%～25%。SWB按下表选择平移合速度和旋转上限，全局上限在 `firmware/src/app/app_config.h`。行驶许可要求遥控在线、四轮电机就绪、SWA=1000、SWD=2000、SWC不为1000、VRA大于1200、VRB不大于1200。VRB关闭时保持零速度。串口每秒输出 `CHASSIS_CHECK`、`WORK_CHECK` 和在线遥控的 `REMOTE_CHECK`，用于区分接收、许可与联动停车状态。

| SWB 原始值 | 档位 | 平移上限 m/s | 旋转上限 rad/s |
| --- | --- | --- | --- |
| 1000 | 低速 | 0.4 | 1.5 |
| 1500 | 中速 | 0.8 | 3.0 |
| 2000 | 高速 | 1.5 | 6.0 |

SWB端点采用250的容差，避免开关原始值轻微偏离1000或2000时误入中档。主机测试通过实际 `app_apply_remote` 入口验证三档速度、端点容差、斜向合速度限制及停车/失联重启许可。航向保持参数沿用此前0.10 m/s实车验证的低速基线，恢复三档后的中高速表现待实车验证。

三档版本已备份、烧录并通过校验，实机拨档采集到 SWB=1000、1500、2000，摇杆回中且VRB关闭时命令和四轮速度均为零。开关验证记录为 `firmware/build/three-speed-switch-retest.txt`，三档输出回归记录为 `firmware/build/three-speed-host-tests.log`。

降速后的0.4/0.8/1.5 m/s版本也已完成回归、烧录校验和停车状态检查，记录分别为 `firmware/build/reduced-speed-tests.log`、`firmware/build/reduced-speed-flash.log` 和 `firmware/build/reduced-speed-final-status.txt`。

上电、遥控断联或行驶许可撤销后，需先关闭 VRB，再开启 VRB 才能输出速度。右摇杆映射按现场复测采用 `vx=norm(Y)`、`vy=-norm(X)`，横向输入已经过底盘坐标转换。`MOTOR_CHECK` 逐个输出 ID 1–4 的转向状态、原始角度、驱动速度和电流，用于核对实际轮位。

四轮正前方零位统一保存在 `firmware/src/service/steering_calibration.h`，正常控制和 ZeroTest 共用。正常控制将原始反馈减去零位后送入运动学，驱动与驻车目标加回零位后选择电机最近等效角。主机测试覆盖正前方、横移、驻车、多圈等效角和默认零偏置兼容行为。

现场逐轮核对轮位为 ID1左前、ID2右前、ID3右后、ID4左后，四个转向均有动作。左前手动精确对齐后，新反馈 `11 3f 37 7f f8 00 20 1c` 的位置码为 `0x3f37`，原始角度约 -6.326581216 rad，归一化机械零位更新为 -0.043395909 rad，比原记录修正约 -2.563°。证据日志为 `firmware/build/logs/uart-20261107-024241-401.log`。

最终烧录 `ManualStandalone` 并通过 Flash 校验，用户在架空复测中确认前、后、左、右方向正确，四个驱动轮均有动作且能够正常停车。该结果覆盖独立遥控恢复和架空方向核对，地面行驶、新车几何、有效轮径及里程计精度仍待标定。

### 直行偏航调校

地面前进约1米，用户报告车头向左偏转约5～6°。已关闭旧车 `yaw_bias` 经验补偿，复测仍有左偏。主控板已确认固定在车体上，转向测试验证外部旋转指令与 IMU z 轴角速度同号。遥控平移且旋转摇杆回中时，现接入 `chassis_yaw_hold` 锁定起步航向。主动旋转、停车、撤销行驶许可、遥控失联或 IMU 无效时解除航向保持。初始参数在 `app_config.h`，Kp=0.8、Kd=0.1，纠偏限幅为±0.03 rad/s。串口 `YAW_CHECK` 输出参考航向、误差和纠偏量，供固定速度、固定距离复测调参。

烧录后用户实车复测反馈“现在好多了”，低速直行左偏明显改善，当前保留上述增益和限幅作为低速标定基线。残余偏角和摆动幅度尚未量化。

## 底盘与肘击装置协同（2026-11-06）

Manual 模式接入 CAN3 作业状态端点 0x610，并通过 0x611 回复。肘击端使用 CAN2 正常模式、经典 CAN 1 Mbit/s。启动握手后，SLOW 将平移速度限制为 0.10 m/s，WAIT 保持停车，四路 C620 实际反馈新鲜且低于阈值后确认 STOPPED，GO 解除任务限制。失联、反馈异常或单端复位进入故障停车。

底盘参数见 `firmware/src/app/work_config.h`，状态和计数可观察 `g_work_link`。接近速度、C620 停稳转速及反馈时效为待标定初值。默认 Diagnostics 用于静态总线诊断，业务联调构建使用 `build-local.ps1 -Mode Manual`。目前完整自主行驶、机械回位及视觉多树关联仍需后续验证。

## 历史进度（2026-11-05）

默认 `APP_RUN_MODE=0` 为静态通信诊断：三路 CAN 接收、USART1 日志和激光快照，不初始化电机、IMU、遥控，不发送应用层 CAN 命令。CAN 外设使用正常模式，会提供 ACK。

| 项目 | 已有证据与下一步 |
| --- | --- |
| CAN1/2 | PR #1 的现场记录确认 CAN1=四个 DM6220，回复 ID `0x11–0x14`；CAN2=四个 C620，反馈 ID `0x201–0x204`。默认 `CHASSIS_CAN_BUSES_SWAPPED=0`。各 ID 对应的物理轮位仍需逐轮核对。 |
| CAN3 激光 | 已记录约 10 Hz 的标准帧 `0x601` 和四路解包；遮挡测试确认 S21C 接口 1/2/3/4=左/右/后/前。原始 `distance_mm[0..3]` 维持接口顺序，上层前/后/左/右索引为 3/2/0/1。位置坐标、逐路健康与导航融合待完成。 |
| 四轮零位 | 已采集 ID 1–4 的正前方读数，独立 `ZeroTest` 完成一次逐轮低速回位，用户确认轮向对齐。2026-11-07 已将零位补偿接入正常底盘运动学并重新标定左前轮，完整模式轮向仍待实车复核。 |
| 直行准备 | 旧车几何、半径及经验偏航补偿仍在，`chassis_yaw_hold.*` 已于 2026-11-07 接入遥控平移（旋转摇杆回中时锁定起步航向）。新车几何、有效轮径与低速入口仍待标定，之后再验证中高速航向保持和侧向测距纠偏。 |
| 验证边界 | 合并维护阶段完成主机测试及 Diagnostics、Manual、Steering、Can2Query、ZeroTest 五种完整 ARM 构建（各 80 个源文件）。本次未烧录、点动或重新验证上述现场结果。 |

明日按 [2026-11-06 调车与长距离直行计划](docs/PROJECT_GUIDE.md#明日调车与长距离直行计划2026-11-06) 执行。历史现场结果、日志索引和待办均在指南中保留；完整手动固件尚未具备新车首轮落地标定条件。

计划附有逐轮记录表、偏差排查表及通行误差预算。单轮入口须适配DJI驱动四轮整组发送，区分零速度与零电流；先实现并验证该入口，再做有源方向标定。

2026-11-05 已采集四轮正前方角度，独立 `ZeroTest` 模式完成四轮低速逐轮回位，用户确认机械轮向对齐，实测结果见[四轮零位回位验证](docs/PROJECT_GUIDE.md#四轮零位回位验证2026-11-05)。测试每次复位后等待 5 秒执行，完成后释放转向。2026-11-07 已将零位补偿接入正常运动学路径，完整模式轮向仍需实车复核。默认构建保持静态诊断模式。

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
| ManualStandalone | 5 | `firmware/build/ManualStandalone/` | 底盘独立遥控：完整初始化与遥控行驶，不注册肘击作业端点，行驶不依赖握手 |
| Steering | 2 | `firmware/build/Steering/` | 转向初始化，清错、使能并切换模式 |
| Can2Query | 3 | `firmware/build/Can2Query/` | 发送只读 ESC_ID 查询，使用实际转向总线映射 |
| ZeroTest | 4 | `firmware/build/ZeroTest/` | 转向初始化后等待 5 秒，按 ID 1–4 逐轮回位并释放 |

编译不执行硬件动作；烧录后行为取决于模式。VS Code 打开仓库根目录可用 `Ctrl+Shift+B` 构建诊断模式；EIDE 也可直接打开 `firmware/`。每次 CubeMX 重新生成后重新运行主机测试和完整构建。

## 烧录、日志与维护

`flash-local.ps1` 默认只选择 Diagnostics，写入前重新构建并备份、校验原 1 MB Flash。OpenOCD 自动查找 `OPENOCD_ROOT`、原 E 盘工具、EIDE 及 PATH，也支持 `-OpenOcdRoot`。具体硬件操作见 [开发命令](docs/PROJECT_GUIDE.md#本机开发命令2026-11-05)。

日志使用 USART1（PA9=TX、PA10=RX），921600 bit/s、8N1；板侧插座以实物丝印为准。VS Code 的 `Firmware Monitor` 任务会询问串口，默认示例 COM12，采集 30 秒。主机测试不访问串口或调试器。

维护时通过分支和 PR 更新 `main`，保留编译、测试及实车证据的区别。`build/` 中的二进制、串口日志与原板备份仅保存在本机，不提交 Git。修改固件时同步更新本页与项目指南，不另建零散进度文档。

来源为 [AgroTech-SCAU/Steering-Wheel-Chassis](https://github.com/AgroTech-SCAU/Steering-Wheel-Chassis) 的 `Hephaestus/chassis_control_code`，基线 `65088f94be15c69ccfa647779d7f847b06b95735`；原仓库未修改。原文档虽混用 Atlas 标题，其底盘驱动仍可参考，新车参数以本项目记录为准。
