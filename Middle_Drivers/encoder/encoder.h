#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>
#include "main.h"   /* main.h 里已包含 hal 头文件 */

/* 轮子编号 -> 定时器映射（顺序故意打乱：轮3 = TIM2）
 * ENC_WHEEL1 -> TIM1 (PE9/PE11)    ENC_WHEEL3 -> TIM2 (PA0/PA1)
 * ENC_WHEEL2 -> TIM3 (PA6/PA7)     ENC_WHEEL4 -> TIM4 (PB6/PB7)   */
#define ENC_WHEEL1    0U
#define ENC_WHEEL2    1U
#define ENC_WHEEL3    2U
#define ENC_WHEEL4    3U
#define ENC_WHEEL_NUM 4U

void    Encoder_Init(void);                 /* 启动四路编码器计数 */
void    Encoder_Update(void);               /* 在 5ms 中断里调用：读四路增量 */
int32_t Encoder_GetDelta(uint8_t wheel);    /* 返回本次 5ms 采样的脉冲增量 */
int32_t Encoder_GetTotal(uint8_t wheel);    /* 返回累计脉冲（里程计备用） */

#endif
