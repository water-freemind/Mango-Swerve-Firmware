/***********************************************
公司：东莞市微宏智能科技有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：
修改时间：2021-04-29

Company: WeiHong Co.Ltd
Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version:
Update：2021-04-29

All rights reserved
***********************************************/
#ifndef __USRAT5_H
#define __USRAT5_H
#include "sys.h"
typedef struct {
	int16_t distance5;  						/* 距离数据：测量目标距离单位 mm */
	uint16_t noise5;		 						/* 环境噪声：当前测量环境下的外部环境噪声，越大说明噪声越大 */
	uint32_t peak5;								/* 接收强度信息：测量目标反射回的光强度 */
	uint8_t confidence5;						/* 置信度：由环境噪声和接收强度信息融合后的测量点的可信度 */
	uint32_t intg5;     						/* 积分次数：当前传感器测量的积分次数 */
	int16_t reftof5;   						/* 温度表征值：测量芯片内部温度变化表征值，只是一个温度变化量无法与真实温度对应 */
}LidarPointTypedef5;

struct AckResultData5{
	uint8_t ack_cmd_id5;						/* 答复的命令 id */
	uint8_t result5; 							/* 1表示成功,0表示失败 */
};

struct LiManuConfig5
{
	uint32_t version5; 						/* 软件版本号 */
	uint32_t hardware_version5; 		/* 硬件版本号 */
	uint32_t manufacture_date5; 		/* 生产日期 */
	uint32_t manufacture_time5; 		/* 生产时间 */
	uint32_t id15; 								/* 设备 id1 */
	uint32_t id25; 								/* 设备 id2 */
	uint32_t id35; 								/* 设备 id3 */
	uint8_t sn4[8]; 								/* sn */
	uint16_t pitch_angle5[4]; 			/* 角度信息 */
	uint16_t blind_area5[2]; 			/* 盲区信息 */
	uint32_t frequence5; 					/* 数据点频 */
};




void uart5_init(u32 bound);
void UART5_IRQHandler(void);
void data_process5(void);

#endif
