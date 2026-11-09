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
#ifndef __USRAT4_H
#define __USRAT4_H
#include "sys.h"

typedef struct {
	int16_t distance4;  						/* 距离数据：测量目标距离单位 mm */
	uint16_t noise4;		 						/* 环境噪声：当前测量环境下的外部环境噪声，越大说明噪声越大 */
	uint32_t peak4;								/* 接收强度信息：测量目标反射回的光强度 */
	uint8_t confidence4;						/* 置信度：由环境噪声和接收强度信息融合后的测量点的可信度 */
	uint32_t intg4;     						/* 积分次数：当前传感器测量的积分次数 */
	int16_t reftof4;   						/* 温度表征值：测量芯片内部温度变化表征值，只是一个温度变化量无法与真实温度对应 */
}LidarPointTypedef4;

struct AckResultData4{
	uint8_t ack_cmd_id4;						/* 答复的命令 id */
	uint8_t result4; 							/* 1表示成功,0表示失败 */
};

struct LiManuConfig4
{
	uint32_t version4; 						/* 软件版本号 */
	uint32_t hardware_version4; 		/* 硬件版本号 */
	uint32_t manufacture_date4; 		/* 生产日期 */
	uint32_t manufacture_time4; 		/* 生产时间 */
	uint32_t id14; 								/* 设备 id1 */
	uint32_t id24; 								/* 设备 id2 */
	uint32_t id34; 								/* 设备 id3 */
	uint8_t sn4[8]; 								/* sn */
	uint16_t pitch_angle4[4]; 			/* 角度信息 */
	uint16_t blind_area4[2]; 			/* 盲区信息 */
	uint32_t frequence4; 					/* 数据点频 */
};



void uart4_init(u32 bound);
void UART4_IRQHandler(void);
void data_process4(void);

#endif
