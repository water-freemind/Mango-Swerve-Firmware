# 1E12C880 G7观测接管修订烧录与监听

2026-11-13，用户要求烧录后开启监听，并确认车辆已停稳、SWD关闭。仅烧录H723底盘，未对F407或测距板操作，未发ORIGIN、ARM、GO。

- 新版FullMapWork SHA256：`1e12c880d748d57218f58b615b50eb1e66c1f9c3c6c29b6b4a5460672df690ba`。
- 部署归档：`firmware/build/deployed/g7-handover-20261113-203154/`，含源码快照、精确差异、镜像、备份和全部下载日志。
- 下载器CMSIS-DAP USB Bulk，序列号332107140234；H723身份检查通过。
- 旧版058B8D5A完整1MB备份，32块各32KB读取并逐块校验；与旧HEX全部163192有效字节一致、0字节不符。备份 `before-flash-058B8D5A.bin`，SHA256 `b72c0bdf26fd5682eb03aa482a1037ebc8b5e55888471ebf74bacffe71c49543`。
- 首次1000kHz写入失败，错误为 `Failed to write memory at 0x20002304`，保留 `flash-attempt-1000khz.log`。未将失败尝试记为成功部署。
- 随后改用200kHz重试，164536有效字节写入成功，6个HEX分段全部校验，`CANDIDATE_IMAGE_ALL_VERIFIED`、`G7_HANDOVER_FLASH_OK`和进程退出0均确认，已复位。
- 复位后COM12/921600、UDP命令1348及VOFA1347恢复；IMU ready=1、2000/2000，uart_fault=tx_fault=0。新监听目录 `firmware/build/live-monitor/listen-g7-20261113-203647/`，PID39952；桥接37668、VOFA14496保持运行，原始串口源继续为 `straight-console-20261113-164938.log`。
- 当前保持停止、未登记起点。上电默认WORK / 0.50 / 0.15；此前故障任务的NAV / 0.80 / 0.35没有恢复。当前SWC=SWD=1000对应未使能状态，RC_SWITCH/HOST_NOT_SELECTED不是通信错误。单独跑图需选择NAV并重新登记起点。

本次针对G7右距重新确认时提前平滑限速、可信左墙与北墙共同支持的有限时距接管，最长1.2秒/0.50米。四向95mm、真实短距、定位与包络保护保留。完整主机回归和五档底盘扰动模型通过，详见[修改与离线验证](20261113-G7-handover-optimization.md)。

**固件已烧录并校验，监听已开启；实车G7卡顿改善尚待测试。** C9FAA/0.50用户优先实车基线保持不变。
