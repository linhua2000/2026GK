#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"      /* main.h 里已包含 hal 头文件 */
#include "encoder.h"   /* 轮子编号只有一处定义：ENC_WHEEL1..4 */

/* 占空比单位：比较值(计数)，与 ARR 同量纲。
 * 1050 = ARR(1049)+1 —— PWM1 模式下 CCR>ARR 才输出恒高，即真正的 100%。
 * 故意不加 U 后缀：与 int 比较时若带 U 会被提升为无符号，负数会全部判成超限。 */
#define PWM_MAX    1050
#define PWM_MIN   (-1050)

void Motor_Init(void);                             /* 启动四路 PWM 并统一到 20kHz */
void Motor_Load(int m1, int m2, int m3, int m4);   /* 有符号：符号=方向，绝对值=比较值 */


#endif
