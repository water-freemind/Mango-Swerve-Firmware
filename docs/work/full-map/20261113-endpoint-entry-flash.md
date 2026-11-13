# 058B8D5A G7入口与树位微调修订烧录

2026-11-13，用户确认车辆已停稳、SWD关闭后执行。目标仅H723底盘，未烧录F407或测距板，未发ORIGIN、ARM、GO。

- 新固件：FullMapWork，SHA256 `058b8d5a31173e09f6fd5cd07c0980e177144d652b1d19aa6ef385acdff18d47`。
- 归档：`firmware/build/deployed/endpoint-entry-20261113-195149/`。包含原固件、源码快照、修改前后差异、OpenOCD日志和manifest。
- 下载器：CMSIS-DAP USB Bulk，序列号332107140234；目标身份 `CHASSIS_ID=0x10016483 CPU=0x411fc272`。
- 烧录前完整1MB读取，32块32KB逐块校验通过；与旧3E0030 HEX全部160536有效字节匹配，0字节不符。备份 `before-flash-3E0030.bin`，SHA256 `6d7194aef759a84c3c63f0d3f281988a6a5c9cf9fa12d5108ca0786cec2137da`。
- 新镜像163192有效字节写入，5段 `verify_image` 全部通过，复位成功。备份和烧录进程均退出0。
- 复位后COM12/921600桥接恢复，UDP1348命令与VOFA1347连接存在，PING获ACK；新POSE显示 `g`/`st` 字段。IMU ready=1、cal=2000/2000，uart_fault=0、tx_fault=0，stopped=1且指令为零。
- 上电默认WORK、行驶0.50m/s、接近0.15m/s、origin=0；烧录前故障任务为NAV/0.60/0.25，复位后没有恢复任务或自动发车。遥控SWC=1000、SWD=1000，当前RC_SWITCH/HOST_NOT_SELECTED是未使能状态。
- 原UDP桥接和VOFA保持运行，新版监听另起独立目录，由 `active-live-monitor.txt` 指向；完整原始串口日志继续写入 `straight-console-20261113-164938.log`。

本版提高正常减速度至0.90m/s²，分开入口连续规划与紧急预算，加入端点平滑观测与15/20mm滞回、冻结参考立即撤销、地图边缘横向限速。原95mm近距、400ms判稳及完整2s驻留保留。详见[修改和离线验证](20261113-endpoint-entry-optimization.md)。

**已烧录与校验，不代表实车改善已确认。** 新制动能力、各速度G7暂停与树位调整时间仍需下一轮实测；C9FAA/0.50用户优先基线及其镜像保持不变。
