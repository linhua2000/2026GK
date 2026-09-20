#include "encoder.h"
#include "tim.h"    /* htim1..htim4 的外部声明 */

/* 轮子编号 -> 定时器句柄，对应关系见 encoder.h 注释 */
static TIM_HandleTypeDef *const s_tim[ENC_WHEEL_NUM] = {
    &htim1,   /* ENC_WHEEL1  PE9/PE11 */
    &htim3,   /* ENC_WHEEL2  PA6/PA7  */
    &htim2,   /* ENC_WHEEL3  PA0/PA1  */
    &htim4,   /* ENC_WHEEL4  PB6/PB7  */
};

/* 中断里写、主循环里读，故必须 volatile */
static volatile int32_t s_delta[ENC_WHEEL_NUM];
static volatile int32_t s_total[ENC_WHEEL_NUM];

void Encoder_Init(void)
{
    uint8_t i;

    for (i = 0; i < ENC_WHEEL_NUM; i++)
    {
        HAL_TIM_Encoder_Start(s_tim[i], TIM_CHANNEL_ALL);
        __HAL_TIM_SET_COUNTER(s_tim[i], 0);
        s_delta[i] = 0;
        s_total[i] = 0;
    }
}

void Encoder_Update(void)
{
    uint8_t i;

    for (i = 0; i < ENC_WHEEL_NUM; i++)
    {
        /* 先读后清零。(int16_t) 截断即模 65536 的有符号增量，
         * 四路 ARR 都是 65535，正好在 65536 回绕（5ms 内 |增量| < 32768） */
        int16_t delta = (int16_t)__HAL_TIM_GET_COUNTER(s_tim[i]);
        __HAL_TIM_SET_COUNTER(s_tim[i], 0);

        s_delta[i]  = (int32_t)delta;   /* TIM6 周期 5ms 内的脉冲增量 */
        s_total[i] += (int32_t)delta;
    }
}

int32_t Encoder_GetDelta(uint8_t wheel)
{
    return (wheel < ENC_WHEEL_NUM) ? s_delta[wheel] : 0;
}

int32_t Encoder_GetTotal(uint8_t wheel)
{
    return (wheel < ENC_WHEEL_NUM) ? s_total[wheel] : 0;
}
