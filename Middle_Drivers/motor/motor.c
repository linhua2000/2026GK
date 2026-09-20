#include "motor.h"
#include "tim.h"      /* htim10 / htim11 / htim12 */
#include "uart1.h"    /* 仅自检打印用 */
#include <stdio.h>

/* 轮号 -> 驱动通道，顺序与 encoder.c 的 s_tim[] 一一对应
 * 轮1 H1-A  PWM PB14 TIM12_CH1  DIR PD10/PD11
 * 轮2 H1-B  PWM PB15 TIM12_CH2  DIR PD14/PD15
 * 轮3 H2-A  PWM PB8  TIM10_CH1  DIR PD3 /PD4
 * 轮4 H2-B  PWM PB9  TIM11_CH1  DIR PD5 /PD7  */

static int moto_abs(int p)
{
    return (p > 0) ? p : (-p);
}

static void Limit(int *m1, int *m2, int *m3, int *m4)
{
    if (*m1 > PWM_MAX) *m1 = PWM_MAX;
    if (*m1 < PWM_MIN) *m1 = PWM_MIN;

    if (*m2 > PWM_MAX) *m2 = PWM_MAX;
    if (*m2 < PWM_MIN) *m2 = PWM_MIN;

    if (*m3 > PWM_MAX) *m3 = PWM_MAX;
    if (*m3 < PWM_MIN) *m3 = PWM_MIN;

    if (*m4 > PWM_MAX) *m4 = PWM_MAX;
    if (*m4 < PWM_MIN) *m4 = PWM_MIN;
}

/* TB6612 真值表：IN1=L/IN2=H 或 H/L 是两个转向；
 * d==0 时两脚同电平，两路输出都拉到低，等效滑行停车 */
static void Dir(GPIO_TypeDef *port, uint16_t p1, uint16_t p2, int d)
{
    HAL_GPIO_WritePin(port, p1, (d > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(port, p2, (d > 0) ? GPIO_PIN_SET   : GPIO_PIN_RESET);
}

void Motor_Init(void)
{
    /* 四路 PWM 统一 20kHz。TIM10/TIM11 挂在 APB2(168MHz)，TIM12 挂在 APB1(84MHz)，
     * 所以预分频不同。CubeMX 里 TIM10 的 ARR 还是 65535(320Hz)，这里一并纠正。 */
    htim10.Init.Prescaler = 7;  htim10.Init.Period = 1049;  HAL_TIM_Base_Init(&htim10);
    htim11.Init.Prescaler = 7;  htim11.Init.Period = 1049;  HAL_TIM_Base_Init(&htim11);
    htim12.Init.Prescaler = 3;  htim12.Init.Period = 1049;  HAL_TIM_Base_Init(&htim12);

    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

    Motor_Load(0, 0, 0, 0);
}

void Motor_Load(int m1, int m2, int m3, int m4)
{
    Limit(&m1, &m2, &m3, &m4);

    Dir(GPIOD, GPIO_PIN_10, GPIO_PIN_11, m1);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, moto_abs(m1));

    Dir(GPIOD, GPIO_PIN_14, GPIO_PIN_15, m2);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, moto_abs(m2));

    Dir(GPIOD, GPIO_PIN_3, GPIO_PIN_4, m3);
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, moto_abs(m3));

    Dir(GPIOD, GPIO_PIN_5, GPIO_PIN_7, m4);
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, moto_abs(m4));
}


// void Motor_TestRun(void)
// {
//     static const int s_seq[4][4] = {      /* 单轮相位：依次只转一个轮 */
//         { 315,   0,   0,   0 },
//         {   0, 315,   0,   0 },
//         {   0,   0, 315,   0 },
//         {   0,   0,   0, 315 }
//     };
//     uint8_t i;

//     Motor_Init();

//     report("TEST: 3s to put the car on blocks");
//     HAL_Delay(3000);

//     report("P1 all FWD 2s");
//     Motor_Load(315, 315, 315, 315);
//     HAL_Delay(2000);

//     report("P2 all REV 2s");
//     Motor_Load(-315, -315, -315, -315);
//     HAL_Delay(2000);

//     for (i = 0; i < 4; i++)
//     {
//         report("P3 single wheel FWD 1s");
//         Motor_Load(s_seq[i][0], s_seq[i][1], s_seq[i][2], s_seq[i][3]);
//         HAL_Delay(1000);
//     }

//     Motor_Load(0, 0, 0, 0);
//     report("TEST DONE, all stopped");
// }
