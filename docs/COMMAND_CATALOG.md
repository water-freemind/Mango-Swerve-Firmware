# 当前命令组清单

2026-11-21 更新。面板源为 [vofa+.cmds.json](../firmware/vofa/vofa+.cmds.json)，共 7 组 33 条。安装时同步软件目录和 LocalAppData 实际加载目录。

比赛版默认巡航 0.90 m/s。面板配合 H723 FullMapWork 和版本 5 定时 F407；NAV/WORK共用面板，由运行时模式选择。旧直线/往返、编码器零位登记等面板不再作为当前默认，历史命令请从对应Git版本提取并配套同版桥接与固件。

| 分组 | 按钮 | 发送给桥接的文本 |
| --- | --- | --- |
| 00 停车与连通 | 停止／取消本次测试 | `UI STOP` |
| 00 停车与连通 | 连通检查（不运动） | `UI PING` |
| 01 全图作业 | 查看全图状态（不运动） | `UI MAP STATUS` |
| 01 全图作业 | 选择单独跑图模式 | `UI MAP MODE NAV` |
| 01 全图作业 | 选择联合疏果模式 | `UI MAP MODE WORK` |
| 01 全图作业 | 登记起点：x=0.20、车头朝场内（前向激光标定y） | `UI MAP ORIGIN` |
| 01 全图作业 | 准备全图（不运动） | `UI MAP ARM ALL` |
| 01 全图作业 | 开始已准备的全图 | `UI MAP GO` |
| 01 全图作业 | 暂停全图 | `UI MAP PAUSE` |
| 01 全图作业 | 继续全图（用户暂停后） | `UI MAP RESUME` |
| 02 行驶速度 | 档位1 0.50 m/s（当前实测最稳) | `UI MAP SPEED 0.50` |
| 02 行驶速度 | 档位2 0.60 m/s（试验档，待优化) | `UI MAP SPEED 0.60` |
| 02 行驶速度 | 档位3 0.70 m/s（试验档，待优化) | `UI MAP SPEED 0.70` |
| 02 行驶速度 | 档位4 0.80 m/s（试验档，待优化) | `UI MAP SPEED 0.80` |
| 02 行驶速度 | 档位5 0.90 m/s（试验档，待优化) | `UI MAP SPEED 0.90` |
| 03 接近速度 | 接近档位1 0.15 m/s | `UI MAP APPROACH 0.15` |
| 03 接近速度 | 接近档位2 0.20 m/s | `UI MAP APPROACH 0.20` |
| 03 接近速度 | 接近档位3 0.25 m/s | `UI MAP APPROACH 0.25` |
| 03 接近速度 | 接近档位4 0.30 m/s | `UI MAP APPROACH 0.30` |
| 03 接近速度 | 接近档位5 0.35 m/s | `UI MAP APPROACH 0.35` |
| 04 疏果联调 | 查看疏果任务与握手 | `UI MAP WORK STATUS` |
| 04 疏果联调 | 停稳后重新握手 | `UI MAP WORK RESET` |
| 05 状态检查 | 整车状态与未就绪原因 | `UI SNAPSHOT STATE` |
| 05 状态检查 | 遥控器开关与摇杆 | `UI SNAPSHOT REMOTE` |
| 05 状态检查 | 四路激光距离 | `UI SNAPSHOT LASER` |
| 05 状态检查 | 四轮电机反馈 | `UI SNAPSHOT MOTORS` |
| 05 状态检查 | CAN1／CAN2／CAN3诊断 | `UI SNAPSHOT CAN` |
| 06 波形与日志 | 开启：FireWater波形数据（10Hz） | `UI LOG WAVE ON` |
| 06 波形与日志 | 关闭：FireWater波形数据（10Hz） | `UI LOG WAVE OFF` |
| 06 波形与日志 | 打开全图定位和进度日志 | `UI LOG MAP ON` |
| 06 波形与日志 | 关闭全图周期日志（保留回执和事件） | `UI LOG MAP OFF` |
| 06 波形与日志 | 只看命令回复和主动查询（安静） | `UI LOG QUIET` |
| 06 波形与日志 | 恢复简洁状态（默认） | `UI LOG DEFAULT` |

这些UI命令由桥接转换，不应把UI文本直接发往固件串口。Static启动器拒绝运动准备/开始；Motion允许操作，但不会自动开始。普通关闭VOFA或断开UDP不替代SWD/STOP。

维护用横移命令及现场测试条件见[定距测试说明](work/full-map/20261118-field-navigation.md#现场复测)。这些命令由 Motion 桥接处理。

安装、启动和测试顺序见[版本与测试索引](VERSION_MATRIX.md)。
