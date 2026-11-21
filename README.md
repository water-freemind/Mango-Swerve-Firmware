# Mango Swerve Firmware

芒果疏果机器人多板固件仓库：H723四舵轮底盘、F407肘击器及独立S21C/F103四路激光采集板。

## 当前版本与阅读入口

2026-11-21：主线整合现场稳定版、优化版与比赛最终版，保留 G7 前激光定位和近障保护。VOFA 面板有 7 组 33 条命令，保留五档行驶速度。各版本的源码及实车记录见[版本与测试索引](docs/VERSION_MATRIX.md)。

| 目的 | 入口 |
| --- | --- |
| 核对各版本、模式、命令组及测试结论 | [版本与测试索引](docs/VERSION_MATRIX.md) |
| 核对已烧录与候选镜像 SHA256 | [11月18日版本清单](docs/releases/20261118-field-navigation.json) |
| 看当前33条按钮和实际UI命令 | [命令目录](docs/COMMAND_CATALOG.md) |
| 上电、遥控、硬件、标定与详细调试 | [项目指南](docs/PROJECT_GUIDE.md) |
| NAV/WORK操作、两板CAN协议 | [联合调试说明](docs/CAN_WORK_DEBUG.md) |
| 肘击动作、模式与上电准备 | [F407说明](firmware-elbow/README.md) |
| 激光采集板独立修复候选 | [S21C说明](firmware-s21c/README.md) |
| 旧部署、回退和实车基线 | [历史记录](docs/PROJECT_HISTORY.md) |

## 当前控制逻辑

底盘 FullMapWork 上电默认 WORK。沿垄扫描时，连续识别到绿色芒果才触发视觉减速。树位用于确认停车和对中，停稳后发送 START，完成 DONE/RELEASE/RELEASED 后继续行驶。两板配套协议 v5。NAV 在固定树点停留 2 秒，切换模式后重新登记起点。

F407 模式 2 以 100% 占空比伸出 700 ms，直接换向全速回收 700 ms。两次动作间隔 50 ms，每棵树累计最多 6 次。上电前人工收回四路。视觉对中目标为 `dx=-12`，原始读数 −26 至 +2 像素通过对中确认，距中心超过 30 像素时重新对中。底盘找正按像素偏差连续限速，临近死区时降至 0.025 m/s，远处上限为 0.12 m/s。机械回收净空及视觉找正效果待实测。

2026-11-20 实车日志：G1–G6 作业段成对前/后激光稳定在 118–199 mm（中位 150 mm），恰好卡在原 150 mm 横向保护阈值上反复钳速，作业段 40–67% 时间近乎静止。现将该阈值缩短到 110 mm、作业段制动余量由 95 mm 缩到 60 mm，95 mm 硬近停与地图包络保护保留；作业巡垄上限统一提到 0.60 m/s，成对观测确认后段速上限由 0.20–0.35 提到 0.26–0.44 m/s，观测丢失期仍保持原低速以防绕过入口保护。换垄方面把修正档下限由 0.12 提到 0.20 m/s 并收窄横差/航向降速区间，减少垄间走走停停；基础巡航档 0.28/0.30、0.70 对中档门控和 G4 外墙保护均未放宽，底盘 plant 仿真门控重闭仍为 1 次。

手动/上位机由SWC区分：1000停车、1500上位机、2000手动；SWD=1000停车、2000许可。手动还需VRB≤1250及原SWA/VRA许可，摇杆回中停稳后接管；手动为自由运动，不自动保持yaw。上位机模式VRB不参与许可，准备/开始由面板发起。详细恢复条件见项目指南。

比赛时可直接在视觉屏点击「单独跑图」或「联合作业」，启动与运行均支持遥控器离线。遥控在线时，SWD=1000 会阻止视觉屏启动并停止运行。车端在停稳后核验硬件和起点条件，联合作业还检查 F407 和视觉就绪。视觉屏「停止」和上位机 `STOP` 均可停车。

**GO后车端独立执行，关闭VOFA或无线断开不会自动停车；停车使用视觉屏停止、上位机 STOP 或在线遥控的 SWD 安全开关。** 不自动发送登记起点、ARM或GO。

## 构建入口

在仓库根目录使用Windows PowerShell 5.1、ARM GCC；主机回归另需Python与现代MinGW-w64 GCC。

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File firmware/build-local.ps1 -Mode FullMapWork
powershell.exe -NoProfile -ExecutionPolicy Bypass -File firmware-elbow/build-local.ps1 -Mode FullMapWork -Toolchain "$env:USERPROFILE\.eide\tools\gcc_arm"
py -3 firmware-elbow/tests/verify_gcc_build.py --mode FullMapWork
```

产物分别为firmware/build/FullMapWork/robot.hex与firmware-elbow/build/FullMapWork/elbow.hex。H723不传Mode默认Diagnostics，F407默认FullMapWork；不可省略模式后猜测产物。所有模式及测试复现命令见版本索引。S21C独立构建，镜像不能混烧。

## UDP与命令组

完全退出VOFA后，用当前脚本安装并启动（软件路径、串口改为本机实际值）：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File firmware/vofa/start-vofa-panel.ps1 -VofaDir 'D:\Vofa +\x64' -Port COM19 -Mode Motion
```

VOFA选择FireWater/UDP，本地1347、远端127.0.0.1:1348；桥接独占COM19/921600。Motion允许用户准备/开始，Static仅观察；两者共用当前33条命令。旧直线/往返面板请配套历史源码，不覆盖当前默认。安装源为firmware/vofa/vofa+.cmds.json，安装器同步软件旁和LocalAppData中的实际加载文件并保留备份。

## 目录与维护边界

| 路径 | 用途 |
| --- | --- |
| firmware/ | H723 CubeMX/EIDE工程、控制源码、构建/烧录脚本、主机测试、VOFA桥接 |
| firmware-elbow/ | F407工程、定时动作与视觉/CAN作业状态机、独立构建和测试 |
| firmware-s21c/ | F103激光采集修复候选；尚未烧录/台架验收 |
| docs/PROJECT_GUIDE.md | 详细硬件和操作指南；带日期的旧章节按历史阅读 |
| docs/VERSION_MATRIX.md、COMMAND_CATALOG.md | 当前版本/测试矩阵及命令目录 |
| docs/releases/ | 可入Git的部署清单与哈希，不包含固件二进制 |
| docs/work/、PROJECT_HISTORY.md | 现场记录、审查与回退追溯 |
| tools/ | 离线路线、包络和报告工具，不控制硬件 |
| 各板build/ | 本机生成镜像、原始日志与备份；Git忽略 |

各板独立构建。源码、配置、命令源和验证摘要纳入 Git。生成文件与原始日志保留在本机。各版本的编译、烧录校验、静态通信和实车结果分别记录。
