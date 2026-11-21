# 版本、命令组与测试索引

## 11 月 21 日比赛版整合

| 现场源码 | 回溯标识 | 作业和导航差异 |
| --- | --- | --- |
| 稳定版 [PR #28](https://github.com/water-freemind/Mango-Swerve-Firmware/pull/28) | `6d58fda` · `stable-field-20261120` | 视觉屏启停、换垄退让、G6 西端预减速、无果复检 |
| 优化版 [PR #29](https://github.com/water-freemind/Mango-Swerve-Firmware/pull/29) | `4bbd5b9` · `optimized-20261120` | 四段定时肘击、默认巡航 0.90 m/s、作业横向保护带 110 mm |
| 比赛最终版 [PR #30](https://github.com/water-freemind/Mango-Swerve-Firmware/pull/30) | `3002545` · `final-20261121` | 单次伸出和回收各 700 ms，`dx=-12±14 px`，G7 中心线移至 Y=4.38 m |
| 主线整合版 | 基于 `3002545` 与主线 `6eb9b9f` | 延续比赛版动作与路线，合入 G7 前激光定位及提前减速。遥控在线且 SWD=1000 时，视觉屏无法开始或继续发起运动 |

三个标签位于队友的 fork，原始提交随整合进入主线。原 PR 中的镜像校验与实车记录对应各自的现场快照。主线整合版以本仓库源码重新构建，整车烧录和现场复测待进行。面板使用同提交的 [VOFA 7 组 33 条命令](COMMAND_CATALOG.md)。H723 与 F407 使用配套 FullMapWork，S21C 独立维护。

G7 横移使用真实前激光校正 Y，130–155 mm 区间平滑调整横移上限，前激光失效暂停。入口转入 G7 前需确认前距至少 165 mm。近障 95 mm 和地图包络继续独立判断。联合作业和单独跑图分别验收。

主线整合版通过完整主机回归（含 49 项 Python 测试）、视觉屏与 SWD 门控专项、33 条 VOFA 命令检查及双板 ARM GCC FullMapWork 编译。对应本次构建的 HEX SHA256 如下。

| 板 | SHA256 |
| --- | --- |
| H723 | `8BDC33C24C08D3ECCBDDF245E0F019082C6A2562B14F4E8EB598BC8245E7D245` |
| F407 | `F634A7D21D928B3C54DF091ACFA1FE751FCD4607EAA91FA7E4A96FA7D0B39C1A` |

11 月 18 日的现场和候选镜像见[历史清单](releases/20261118-field-navigation.json)与[现场记录](work/full-map/20261118-field-navigation.md)。

## 11月17日交付记录

11月17日后续调整：F407当前打击/返回均为700ms，换向10ms、结束等待50ms；仅F407重烧，动作与CAN回归、镜像检查和Verified OK通过。500/500ms表格及旧部署JSON保留历史，当前记录见[700ms部署](work/full-map/20261117-elbow-700ms.md)。

| 版本 | 内容 | 部署与测试结论 |
| --- | --- | --- |
| PR #22，合并fb7f91a | F407纯定时500/10/500ms、满占空比、等待50ms；每树最多4次；视觉对中±10、重新对中±30像素，两板协议v5 | H723/F407 FullMapWork均构建、备份、烧录校验并复位；静态日志恢复。随后首轮WORK运行至G4入口发生MAP_ENVELOPE；整图及机构效果未验收 |
| PR #23，合并8ac4227 | 移除旧零位按钮，7组33条面板；仅面板/文档/测试修改 | 面板回归通过、安装文件哈希一致；固件沿用PR #22镜像，无需因本次文档整理重烧 |
| 2b05f4d及PR #21文档 | 编码器位置判停、85%伸出/2%回收、上电登记零位 | 历史实现，已被定时版替代；11月17日早期构建日志不属于新定时版 |
| C9FAA、058B8D5A、411A0C6B等 | 历次底盘候选、回退与速度实测对照 | 见[历史记录](PROJECT_HISTORY.md)，旧版0.50m/s实测不能证明当前联合版本已验收 |
| S21C独立修复候选 | F103四路激光解析/输出修复 | 历史GCC及主机测试通过；未烧录、未台架验收。本轮未修改激光板 |

11 月 17 日两板镜像、备份和命令源哈希见[部署清单](releases/20261117-timed-elbow.json)，操作记录见[部署说明](work/full-map/20261117-timed-elbow-deployment.md)。清单中的build路径为本机证据，不随Git上传；队友需在对应源码重新构建。

## 固件模式与命令匹配

| 板 | 构建模式 | 配套用途/面板 |
| --- | --- | --- |
| H723 | FullMapWork | 当前组合版本；运行时选NAV单独跑图或WORK联合疏果；使用当前33条面板。上电默认WORK |
| H723 | StraightTest | 模式6、未启用WORK_STATION_ENABLED；独立行走测试。不可据同为模式6认定已启用联合疏果；非本轮部署 |
| H723 | Diagnostics（默认构建） | 模式0，CAN诊断；不能把省略Mode构建的镜像当作FullMapWork |
| H723 | Manual / ManualStandalone | 模式1/5手动专项；按对应源码说明测试，当前全图面板不作为完整适配保证 |
| H723 | Steering / Can2Query / ZeroTest | 模式2/3/4专项诊断；分别构建，不用于全图验收 |
| F407 | FullMapWork（默认构建） | ELBOW_TEST_MODE=2，配套当前H723 v5，等START才动作；不需要登记零位 |
| F407 | VisionTest | 模式1，独立视觉动作；视觉可触发电机，不经底盘START，不用于联合运行 |
| F407 | LegacyWork | 模式0，旧v1协同，不能替代当前v5联合模式 |
| F407 | ManualEncoder | 模式3，PWM保持0，只用于手推读数 |
| F103 | S21C工程 | 独立采集板；不使用H723/F407镜像，详见[采集板说明](../firmware-s21c/README.md) |

NAV与WORK是同一H723 FullMapWork的运行选项，不是两个镜像。切换后需重新登记起点。手动由SWC选择，与NAV/WORK选择不同层级。

## 命令组与启动器

唯一当前命令源为[命令目录](COMMAND_CATALOG.md)中的7组33条。两种启动器加载相同JSON，区别是桥接是否允许运动：Static只观察，Motion允许用户准备/开始。没有为每个固件模式维护一份重复面板，旧7组以外的直线/往返面板不应覆盖当前文件。

先完全退出VOFA，再在仓库根目录运行（改为本机实际软件目录及串口）：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File firmware/vofa/start-vofa-panel.ps1 -VofaDir 'D:\Vofa +\x64' -Port COM19 -Mode Motion
```

VOFA：FireWater，UDP本地1347、远端127.0.0.1:1348。桥接独占COM19/921600，串口号按设备管理器核对；本轮COM21属于另一烧录器，不能直接替换。桥接不自动登记/准备/开始。安装器备份并同时更新软件旁及LocalAppData的命令文件；软件退出会保存配置，所以更新前先正常退出。

## 11 月 17 日验证证据

| 检查层级 | 本轮证据 | 能证明什么 |
| --- | --- | --- |
| 主机动作回归 | pr22-motor.log | 定时动作、方向互斥、解析、故障返回、毫秒回绕、可选停走保护测试通过 |
| CAN/路线模拟 | pr22-can.log | 两端协议一致、四次上限与放行、视觉触发及模拟路线通过；不能代表实车 |
| OLED | pr22-oled.log | 显示及分页时延预算通过 |
| 面板 | test_vofa_panel.ps1，33条通过 | UI命令与桥接门禁回归通过；本轮整理后再次检查 |
| ARM GCC | pr22-h723.log / pr22-f407.log | 两板完整链接及HEX/BIN导出成功；F407镜像98向量、硬浮点检查通过 |
| 烧录 | pr22-*-flash.log | 原板备份校验、新镜像Verified OK及复位 |
| 静态实机 | COM19原始日志 | 停稳、零输出、激光有效位0xf、CAN3持续更新；不是运动验收 |
| 实车运动/打击 | [首轮右侧换垄停车](work/full-map/20261117-right-shift-stop.md) | WORK到G4入口触发MAP_ENVELOPE；日志有视觉作业事务，实际回收净空和整圈连续运行未验收 |

以上pr22日志和备份均在本机firmware/build/verify-20261117。完整全仓库回归本轮没有重新运行，不将历史49项/300事务报告直接标成当前版本结果。

复现最小检查：

```powershell
py -3 firmware-elbow/tests/verify_vision_motor.py
py -3 firmware-elbow/tests/verify_work_can.py
py -3 firmware-elbow/tests/verify_oled.py
powershell.exe -NoProfile -ExecutionPolicy Bypass -File firmware/tests/test_vofa_panel.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File firmware/build-local.ps1 -Mode FullMapWork
powershell.exe -NoProfile -ExecutionPolicy Bypass -File firmware-elbow/build-local.ps1 -Mode FullMapWork -Toolchain "$env:USERPROFILE\.eide\tools\gcc_arm"
py -3 firmware-elbow/tests/verify_gcc_build.py --mode FullMapWork
```

主机测试需要现代MinGW-w64 GCC在PATH；ARM GCC另行选择。F407自动查找在本机遇到不存在E盘的路径错误，本轮用显式Toolchain成功构建，不能拿失败后残留镜像作本轮成果。

## 下一次现场验收与留档

1. SWD=1000、车辆放稳；上电前人工收回四路，确认动作空间。定时版没有编码器回位核验，HOME不代替机械检查。
2. 启动面板，查状态、遥控、激光、CAN；确认IMU完成标定、停止与零输出。模式和速度以实时回执为准。
3. 先NAV单独跑图验证底盘，再在受控区域验证单树定时动作及四次上限，最后WORK全图。WORK视觉、对端必须就绪；不混用VisionTest与联合模式。
4. 记录每次提交号、两板镜像哈希、模式、命令组哈希、速度、SWC/SWD状态、开始/结束时间、结束原因和原始日志。区分操作者停止、保护停车和正常完成。
5. 重点检查定时500ms能否完整返回、满占空比的限位负载、±10像素是否造成反复找正、失败四次后能否正常离树。未经实测不标“可比赛”。

紧急停车使用SWD=1000或STOP。GO后车端独立运行，无上位机心跳停车，不能靠关闭VOFA停车。

## 维护规则

只维护一个当前命令源；旧版本通过提交号和历史文档追溯，不复制生成目录冒充版本。源码/配置/命令/验证摘要入Git，HEX/BIN、串口原始日志、备份留build本机归档。历史文件不批量删除；本轮原有.gitignore个人修改不纳入整理提交。后续部署新增清单，不覆盖旧清单，且没有重新验证就不更新测试结论。

2026-11-17后续：当前工作区改为打击/返回各650ms，并修复视觉对齐短暂丢识别导致确认饥饿；见[本次记录](work/full-map/20261117-elbow-650ms-vision-dropout.md)。上面的700ms记录保留部署历史。
