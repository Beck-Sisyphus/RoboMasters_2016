#ifndef __CAN2_H__
#define __CAN2_H__

#include <stm32f4xx.h>
void CAN2_Configuration(void);

#define RATE_BUF_SIZE 6
typedef struct{
	int32_t raw_value;									// Raw reading from encoder without processing  //编码器不经处理的原始值
	int32_t last_raw_value;							// Last reading from encoder without processing //上一次的编码器原始值
	int32_t ecd_value;									// Continous reading from encoder after processing //经过处理后连续的编码器值
	int32_t diff;												// The difference between two encoder reading   //两次编码器之间的差值
	int32_t temp_count;									// count 																				//计数用
	uint8_t buf_count;									// filter buffer updates 												//滤波更新buf用
	int32_t ecd_bias;										// Initial encoder value												//初始编码器值
	int32_t ecd_raw_rate;								// Valocity raw value calculated from encoder		//通过编码器计算得到的速度原始值
	int32_t rate_buf[RATE_BUF_SIZE];		// buffer，for filter
	int32_t round_cnt;									// round count																	//圈数
	int32_t filter_rate;								// velocity																			//速度
	float ecd_angle;										// angle																				//角度
}Encoder;

#endif
