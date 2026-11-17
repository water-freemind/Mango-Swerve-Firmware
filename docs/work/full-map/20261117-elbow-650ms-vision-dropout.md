# 650ms肘击与视觉间断识别修复

2026-11-17，接续700ms部署。打击/返回均改为650ms，换向10ms、结束等待50ms，100%PWM；纯定时仍不能保证精确机械回位。

## 本轮停车证据

日志：`firmware/build/logs/straight-console-20261117-103813.log`。本轮token=3，segment=10（G3），目标station=5（一号树）。首次故障为t_ms=1449342的TREE_VISION_TIMEOUT，而非后续CONTROL_MODE_CHANGED。前后测距约149/147mm，遥控在线、blockers=0，CAN3无错误。视觉在线，dx=-5到-7与9999交替，未完成居中确认。

## 修改范围

F407对齐阶段原来每个9999立即清零，要求持续识别300ms。现允许有效树帧之间最多300ms的间断，累计至少3个有效帧且时间跨度至少300ms。缺树帧仍只回复TRACKING，不能START；超过300ms或偏移超过30像素清零。±10像素候选、±30像素退出滞回以及H723的12秒超时保持。仅改对齐阶段，不改变实际击打复检和四次总尝试上限。

这修复间断识别导致确认饥饿的问题；持续无法识别仍需排查相机，不能绕过视觉强行击打。此前右侧换垄MAP_ENVELOPE记录仍独立待处理。

## 验证

主机回归新增交替100ms有效/100ms缺树，验证可完成确认；缺树START拒绝，长丢失和31像素偏移清零。两板通信与完整路线回归、定时动作回归通过。构建/烧录记录见本机firmware/build/verify-20261117/650ms-*.log；实际烧录结果在完成后补记。无助手触发行走或击打验收。

## 烧录完成

F407序列号134244206173，FullMapWork，HID/SWD 400kHz，650ms-f407-flash.log记录Programming Finished、Verified OK、Resetting Target。H723和S21C未改写。烧录后日志显示视觉帧重新计数、online=1、state=1，车静止，RC_SWITCH保护开启。VOFA面板时间说明和UDP启动配置同步更新。

HEX SHA256：`e995c33424d979f91b8a7fbbf874d2755e1beb3fb274269f760b67a250b66a43`。

## 后续ALIGN/START门槛一致性修复

进一步复盘发现dx=11时居中滞回曾导致WORKING后7ms退回ALIGN。现ALIGN确认与START均要求±10px，超出立即给出调整方向，实际作业过程的±30px重对齐规则保留。上述原有ALIGN滞回说明属于首次修复记录，已由本节取代。追加回归650ms-align-can.log通过，650ms-align-build.log构建通过，650ms-align-flash.log完成Verified OK并重启。最终HEX SHA256：`a8dae5e847b6a0150ffe3a1ef391e9abc0cd49a62fda92b7c83c92226c4acdbd`。详见[跑图复盘](20261117-last-route-review.md)。
