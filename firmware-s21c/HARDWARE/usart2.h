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
#ifndef __USRAT2_H
#define __USRAT2_H
#include "sys.h"

//#define HEADER 0xAA							/* 起始符 */
//#define device_address 0x00     /* 设备地址 */
//#define chunk_offset 0x00       /* 偏移地址命令 */
//#define PACK_GET_DISTANCE 0x02 	/* 获取测量数据命令 */
//#define PACK_RESET_SYSTEM 0x0D 	/* 复位命令 */
//#define PACK_STOP 0x0F 				  /* 停止测量数据传输命令 */
//#define PACK_ACK 0x10           /* 应答码命令 */
//#define PACK_VERSION 0x14       /* 获取传感器信息命令 */

typedef struct {
	int16_t distance2;  						/* 距离数据：测量目标距离单位 mm */
	uint16_t noise2;		 						/* 环境噪声：当前测量环境下的外部环境噪声，越大说明噪声越大 */
	uint32_t peak2;								/* 接收强度信息：测量目标反射回的光强度 */
	uint8_t confidence2;						/* 置信度：由环境噪声和接收强度信息融合后的测量点的可信度 */
	uint32_t intg2;     						/* 积分次数：当前传感器测量的积分次数 */
	int16_t reftof2;   						/* 温度表征值：测量芯片内部温度变化表征值，只是一个温度变化量无法与真实温度对应 */
}LidarPointTypedef2;

struct AckResultData2{
	uint8_t ack_cmd_id2;						/* 答复的命令 id */
	uint8_t result2; 							/* 1表示成功,0表示失败 */
};

struct LiManuConfig2
{
	uint32_t version2; 						/* 软件版本号 */
	uint32_t hardware_version2; 		/* 硬件版本号 */
	uint32_t manufacture_date2; 		/* 生产日期 */
	uint32_t manufacture_time2; 		/* 生产时间 */
	uint32_t id12; 								/* 设备 id1 */
	uint32_t id22; 								/* 设备 id2 */
	uint32_t id32; 								/* 设备 id3 */
	uint8_t sn2[8]; 								/* sn */
	uint16_t pitch_angle2[4]; 			/* 角度信息 */
	uint16_t blind_area2[2]; 			/* 盲区信息 */
	uint32_t frequence2; 					/* 数据点频 */
};



void uart2_init(u32 bound);
void USART2_IRQHandler(void);
void data_process2(void);

#endif
