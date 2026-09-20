#ifndef __MAILUNCONTROL_H
#define __MAILUNCONTROL_H

/* 轮号 -> 实际轮位（接线时确认的，代码里推不出来 —— 调增益时对着这张表看）：
 *   Wheel1 = 左后 B轮   TIM1 编码器   PB14 PWM
 *   Wheel2 = 左前 A轮   TIM3 编码器   PB15 PWM
 *   Wheel3 = 右前 B轮   TIM2 编码器   PB8  PWM
 *   Wheel4 = 右后 A轮   TIM4 编码器   PB9  PWM
 * 对角同型号（左前=右后=A，左后=右前=B），标准麦轮 X 布局。
 * A/B 不影响速度环，但麦轮运动学解算时要用到这张表。 */

/* 四路速度环 PI 增益，每路一对、彼此独立 —— 用来分别微调四个轮子的
 * 摩擦、齿轮箱阻力、电机一致性差异，不是用来兜符号问题的
 * （符号若不对外，四路应当一起取负）。
 *
 * 每个都必须带 f 后缀：不带 f 就是 double 字面量，F407 只有单精度
 * FPU(FPv4-SP)，Velocity_Kp1 * EnC_Err_Lowout 会被提升成 double
 * 走软件模拟，而这段代码将来要跑在 5ms 中断里，慢几十倍。
 *
 * 当前都是 0.0f —— 控制器无输出，上电电机绝不动。标定完再逐路填。 */
#define Velocity_Kp1   2.6f
#define Velocity_Ki1   0.08f
#define Velocity_Kp2   2.6f
#define Velocity_Ki2   0.08f
#define Velocity_Kp3   2.6f
#define Velocity_Ki3   0.08f
#define Velocity_Kp4   2.6f
#define Velocity_Ki4   0.08f

/* 四路速度环。Target 与 encoder 同单位（脉冲/5ms）。
 * 状态各自独立、增益也各自独立，四路可任意顺序调用。返回带符号 PWM 比较值，
 * 未做限幅，由 Motor_Load 内部 Limit() 夹到 ±1050。 */
int Velocity_Wheel1(float Target, int encoder);
int Velocity_Wheel2(float Target, int encoder);
int Velocity_Wheel3(float Target, int encoder);
int Velocity_Wheel4(float Target, int encoder);

#endif
