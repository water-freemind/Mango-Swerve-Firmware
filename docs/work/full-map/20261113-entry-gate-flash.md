# 411A0C6B入口与调轮修复烧录

2026-11-13用户明确要求完成修复并验证后烧录，随后确认车辆已停稳、SWD关闭。烧录前新鲜日志亦确认SWD=1000、stopped=1、指令为零。仅操作H723底盘，未自动发ORIGIN、ARM、GO。

- 新FullMapWork SHA256：`411a0c6b35ad2dd8dc68423f26acbb2ca49e58661a8116ad5687df126730e6be`。
- 部署目录：`firmware/build/deployed/entry-gate-20261113-221844/`，候选目录 `firmware/build/candidates/entry-gate-20261113-214005/`。源码、既有修改备份、相对058B8D5A差异、构建产物和测试日志均保留。
- 下载器USB Bulk CMSIS-DAP、序列号332107140234，200kHz；H723身份检查通过。
- 旧058B8D5A完整1MB备份，32块逐块读回并verify通过；与旧HEX163192有效字节一致，0不符。备份 `before-flash-058B8D5A.bin`，SHA256 `b72c0bdf26fd5682eb03aa482a1037ebc8b5e55888471ebf74bacffe71c49543`。
- 新固件164216有效字节写入成功，6段全部校验，`CANDIDATE_IMAGE_ALL_VERIFIED` / `ENTRY_GATE_FLASH_OK`确认，下载与备份进程均退出0，已复位。
- 复位后串口日志持续更新，IMU ready=1、2000/2000，uart_fault=tx_fault=0，初始保持停止，SWD=1000；上电WORK / 0.50 / 0.15、origin=0。AISLE新增 `se=0.0`字段已在实机收到。
- 新监听目录 `firmware/build/live-monitor/listen-entry-gate-20261113-222502/`、PID40968；原始串口源 `straight-console-20261113-210722.log`。桥接PID624、VOFA PID14496，UDP1348/1347端口均存在。自动UDP探针未获得新PING回执，随后已观察到用户自行操作的GO成功ACK及NAV/0.50任务运行，下行命令当时可以到达车端；不能将自动探针未获ACK当成下行故障。助手没有启动任务或中断用户测试。

本版按危险方向限制速度，修复调轮准备后逐分量裁剪造成的起步方向突变，收紧换垄交接位置/航向；95mm、硬包络及其它安全门禁保留。验证详情和模型限制见[修订记录](20261113-entry-gate-fix.md)。G7扰动模型重闭从2降0，G4/G2模型没有完整复现现场长等待，**实车改善仍待下一轮测量**。C9FAA/0.50优先实车基线不变。
