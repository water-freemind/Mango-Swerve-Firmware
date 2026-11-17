# Mango Swerve Firmware

芒果疏果机器人多板固件仓库：H723四舵轮底盘、F407肘击器及独立S21C/F103四路激光采集板。

## 当前版本与阅读入口

2026-11-17：PR #22定时肘击版已合并，H723与F407的FullMapWork均烧录校验通过；PR #23移除旧零位登记按钮。当前VOFA为7组33条。静态通信已恢复；随后首轮WORK在右侧换入G4后触发MAP_ENVELOPE，详见[现场记录](docs/work/full-map/20261117-right-shift-stop.md)。肘击实际行程及整图效果未验收。

| 目的 | 入口 |
| --- | --- |
| 核对各版本、模式、命令组及测试结论 | [版本与测试索引](docs/VERSION_MATRIX.md) |
| 核对已烧录镜像/备份SHA256 | [本次部署清单](docs/releases/20261117-timed-elbow.json) |
| 看当前33条按钮和实际UI命令 | [命令目录](docs/COMMAND_CATALOG.md) |
| 上电、遥控、硬件、标定与详细调试 | [项目指南](docs/PROJECT_GUIDE.md) |
| NAV/WORK操作、两板CAN协议 | [联合调试说明](docs/CAN_WORK_DEBUG.md) |
| 肘击动作、模式与上电准备 | [F407说明](firmware-elbow/README.md) |
| 激光采集板独立修复候选 | [S21C说明](firmware-s21c/README.md) |
| 旧部署、回退和实车基线 | [历史记录](docs/PROJECT_HISTORY.md) |

## 当前控制逻辑

底盘FullMapWork上电默认WORK：沿垄扫描，视觉识别Tree与Gmg后减速、对中停稳，发送START许可肘击，完成DONE/RELEASE/RELEASED后继续。两板配套协议v5。NAV为同一固件的单独跑图选项，使用固定树点停2秒；切换模式后重新登记起点。

F407模式2使用100%占空比打击650ms、换向停10ms、返回650ms、等待50ms；每树累计最多4次。当前没有编码器位置闭环或回位核验。上电前人工收回四路，无需登记编码器零位。对中接受±10像素，超过±30重新对中；机械回收净空及视觉找正效果待实测。

手动/上位机由SWC区分：1000停车、1500上位机、2000手动；SWD=1000停车、2000许可。手动还需VRB≤1250及原SWA/VRA许可，摇杆回中停稳后接管；手动为自由运动，不自动保持yaw。上位机模式VRB不参与许可，准备/开始由面板发起。详细恢复条件见项目指南。

**GO后车端独立执行，关闭VOFA或无线断开不会自动停车；停车使用SWD或STOP。** 不自动发送登记起点、ARM或GO。

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

保持各板独立构建；源码、权威配置、命令源和证据摘要入Git，生成文件及个人IDE状态不提交。当前测试与历史测试分别记录，编译成功、烧录校验、静态通信、实车验收不互相替代。本轮仅整理文档，不改变控制参数或重新烧录。
