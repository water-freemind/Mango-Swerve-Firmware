# 疏果 F407 配套工程

本目录用于与同仓库的 H723 全图底盘一起调试和交接。来源为 `肘击装置/RED_Demo/RED_Demo`，原仓库基线 `c33ab16`。包含 Core、BSP、所需 STM32F4 HAL/CMSIS、CubeMX 配置、Keil 工程和测试。

## 工程与模式

打开 `MDK-ARM/RED_Demo.uvprojx`，目标 RED_Demo，使用 ARMCC 5.06 update 7。控制器为 STM32F407VETx，Flash512KB。

`BSP/vision_motor.h` 的 ELBOW_TEST_MODE 默认2，与 H723 的 FullMapWork 镜像协作。模式1用于独立视觉测试，模式0使用原版本1的视觉主导协同。修改后重新构建和烧录。

模式2采用版本2协议，底盘分配任务号和树位号，疏果端提出 SLOW/WAIT 请求，底盘到点停稳后发送 START，动作返回及复检完成后通过 DONE/RELEASE/RELEASED 结束事务。操作、状态和异常处理见[联合调试说明](../docs/CAN_WORK_DEBUG.md)。

相机离线时显示 `W:WAIT_CAM`，作业等待视觉就绪，激光接收与显示继续刷新。周期显示每次只传一页，规定I2C延时约18.9ms，8页轮转；动作与复检期间推迟耗时的屏幕重初始化。相机断线就绪修复及本次逐页显示修复均需下次上电更新，镜像历史与当前候选区别见[离线复审记录](../docs/CAN_WORK_DEBUG.md#offline-review-pr16)。

## 源码入口

| 文件 | 用途 |
| --- | --- |
| Core/Src/main.c | 初始化及主循环调用 |
| BSP/work_control.c | 握手、接近请求、复检、作业、结束和故障状态机 |
| BSP/can_test.c | CAN2过滤、收发、100ms重发和500ms超时 |
| BSP/work_can_protocol.h | 与H723一致的版本1、版本2协议 |
| BSP/vision_motor.c | 视觉解析和四个区域的电机动作 |
| BSP/vision_motor.h | 模式、阈值、动作计时和重试参数 |
| BSP/fun.c | OLED及周期串口诊断 |

视觉串口帧为 `$VIS,区域,dx` 加 CRLF，区域0表示当前无待处理目标，dx9999表示无有效树。dx用于阈值请求、OLED及串口观察。车辆按全图树位定位。电机回位采用定时反转，实机需要核验回位和净空。

## 构建与仿真

从仓库根目录运行。

```powershell
& 'E:\Keil5\Core\UV4\UV4.exe' -r firmware-elbow/MDK-ARM/RED_Demo.uvprojx -t RED_Demo -j0 -o firmware/build/elbow-build.log
python firmware-elbow/tests/verify_work_can.py
python firmware-elbow/tests/verify_vision_motor.py
python firmware-elbow/tests/verify_oled.py
```

测试需要 Python 与64位MinGW-w64 GCC。联合测试编译本目录和 `firmware/` 中的实际状态机，并运行25种速度组合的全图闭环仿真，覆盖300次树位作业。另覆盖在途READY、单树四区域25ms调度和生产I2C驱动逐页延时预算。本机追加修复完成ARM GCC的31个C文件编译及可重定位链接，尚未在本机重新完成ARMCC5最终链接。硬件连接、机械回位和地面定位精度需要实机验证。

产物为 `MDK-ARM/RED_Demo/RED_Demo.hex`，构建产物及个人调试配置由忽略规则管理。两端协议头通过联合测试逐字核对，后续修改应同时更新两份定义。
