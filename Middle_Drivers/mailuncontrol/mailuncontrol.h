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

/* ============ 车体位置环（外环）============
 * 反馈取 odometry.x / .y / .theta，返回【速度指令】，喂 Set_Vel(vx, vy, ω)。
 * 后面接的是现成的 Exp_Speed_Cal -> 轮速环 -> Motor_Load，本模块不碰电机。
 *
 * 单位：Pos_X / Pos_Y  误差 mm  -> 返回 mm/s
 *       Pos_Yaw       误差 deg -> 返回 rad/s（函数内已乘 DEG2RAD）
 * 三个环的 Kp 单位统一是 1/s。全部带 f 后缀：F407 只有单精度 FPU，
 * 不带 f 会被提升成 double 走软件模拟（同本文件上面 Velocity_Kp 那条注释）。
 *
 * 调参顺序：先只给 Kp（Ki 清 0），加到响应够快、略有超调，再退回 60~80%；
 * 然后加 Ki，从 Kp 的 1/20 ~ 1/50 起。三个一起调会定位不出问题源。
 *
 * 注意：调用周期直接决定 Ki 和积分限幅的量级，下面是按「5ms 调一次」估的。 */
#define Pos_Kp_X      2.0f      /* 1/s：100mm 误差 -> 100mm/s */
#define Pos_Ki_X      0.02f
#define Pos_Kp_Y      2.0f
#define Pos_Ki_Y      0.02f
#define Pos_Kp_Yaw    0.07f      /* 1/s：10deg 误差 -> 0.87rad/s ≈ 50deg/s */
#define Pos_Ki_Yaw    0.0f

/* 积分累计量限幅（单位：mm·拍 / rad·拍）。yaw 单独给 —— 误差换成 rad 后数值比 mm
 * 小一个量级，共用同一个值会让积分项顶到输出上限。 */
#define Pos_I_Limit_XY   200.0f
#define Pos_I_Limit_Yaw  5.0f

/* 输出限幅：x/y 是 mm/s，yaw 是 rad/s。只管把指令关进合理范围 ——
 * 后面 Exp_Speed_Cal() 还会按 MAX_RPM 做一次整体比例缩放（kinematics.c:83），
 * 所以这不是最后一道防线，只是别让指令一开始就离谱。 */
#define Pos_V_Max     300.0f
#define Pos_W_Max     1.0f

/* 误差一阶低通系数（0 = 不滤，越大越钝）。x/y 用 0.8 是你原来的值；
 * yaw 原来写 0.0，即直通不过滤，也原样保留。嫌位置环超调就先把 XY 调到 0.0 试。 */
#define Pos_A_XY      0.8f
#define Pos_A_Yaw     0.0f

float Pos_X(float Target, float pos_x);                   /* mm  -> mm/s */
float Pos_Y(float Target, float pos_y);                   /* mm  -> mm/s */
float Pos_Yaw(float Target, float pos_theta, float Yaw);  /* deg -> rad/s */

#endif
