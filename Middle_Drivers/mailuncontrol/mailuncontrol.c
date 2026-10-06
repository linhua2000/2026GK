/* ============================================================
 * 速度环 PI 控制 —— 四路麦轮
 * 采样在 TIM6 的 5ms 中断里触发，encoder 参数单位 = 脉冲/5ms。
 * 四路增益各自独立，在 mailuncontrol.h（当前 Kp=2.6f / Ki=0.08f）。
 *
 * 本文件后半还有【车体位置环】Pos_X / Pos_Y / Pos_Yaw：反馈取 odometry 的
 * x/y/θ，输出速度指令喂 Set_Vel，是轮速环的外一层。增益也在 mailuncontrol.h。
 * ============================================================ */
#include "mailuncontrol.h"
#include "kinematics.h"     /* Get_MiMx()：位置环的输出限幅和积分限幅都用它 */

int Velocity_Wheel1(float Target, int encoder)
{
	static int PWM_out;
	static float Encoder_Err;
	static float EnC_Err_Lowout;
	static float EnC_Err_Lowout_last;
	static float Encoder_S;
	float a = 0.8;

	Encoder_Err = Target - encoder;
	EnC_Err_Lowout = (1-a)*Encoder_Err + a*EnC_Err_Lowout_last;
	EnC_Err_Lowout_last = EnC_Err_Lowout;
	Encoder_S += EnC_Err_Lowout;
	Encoder_S = Encoder_S>10000 ? 10000 : (Encoder_S<(-10000) ? (-10000) : Encoder_S);

	if(Target==0) Encoder_S = 0;

	PWM_out = Velocity_Kp1 * EnC_Err_Lowout + Velocity_Ki1 * Encoder_S;
	return PWM_out;
}

/* ------------------------------------------------------------ */

int Velocity_Wheel2(float Target, int encoder)
{
	static int PWM_out;
	static float Encoder_Err;
	static float EnC_Err_Lowout;
	static float EnC_Err_Lowout_last;
	static float Encoder_S;
	float a = 0.8;

	Encoder_Err = Target - encoder;
	EnC_Err_Lowout = (1-a)*Encoder_Err + a*EnC_Err_Lowout_last;
	EnC_Err_Lowout_last = EnC_Err_Lowout;
	Encoder_S += EnC_Err_Lowout;
	Encoder_S = Encoder_S>10000 ? 10000 : (Encoder_S<(-10000) ? (-10000) : Encoder_S);

	if(Target==0) Encoder_S = 0;

	PWM_out = Velocity_Kp2 * EnC_Err_Lowout + Velocity_Ki2 * Encoder_S;
	return PWM_out;
}

/* ------------------------------------------------------------ */

int Velocity_Wheel3(float Target, int encoder)
{
	static int PWM_out;
	static float Encoder_Err;
	static float EnC_Err_Lowout;
	static float EnC_Err_Lowout_last;
	static float Encoder_S;
	float a = 0.8;

	Encoder_Err = Target - encoder;
	EnC_Err_Lowout = (1-a)*Encoder_Err + a*EnC_Err_Lowout_last;
	EnC_Err_Lowout_last = EnC_Err_Lowout;
	Encoder_S += EnC_Err_Lowout;
	Encoder_S = Encoder_S>10000 ? 10000 : (Encoder_S<(-10000) ? (-10000) : Encoder_S);

	if(Target==0) Encoder_S = 0;

	PWM_out = Velocity_Kp3 * EnC_Err_Lowout + Velocity_Ki3 * Encoder_S;
	return PWM_out;
}

/* ------------------------------------------------------------ */

int Velocity_Wheel4(float Target, int encoder)
{
	static int PWM_out;
	static float Encoder_Err;
	static float EnC_Err_Lowout;
	static float EnC_Err_Lowout_last;
	static float Encoder_S;
	float a = 0.8;

	Encoder_Err = Target - encoder;
	EnC_Err_Lowout = (1-a)*Encoder_Err + a*EnC_Err_Lowout_last;
	EnC_Err_Lowout_last = EnC_Err_Lowout;
	Encoder_S += EnC_Err_Lowout;
	Encoder_S = Encoder_S>10000 ? 10000 : (Encoder_S<(-10000) ? (-10000) : Encoder_S);

	if(Target==0) Encoder_S = 0;

	PWM_out = Velocity_Kp4 * EnC_Err_Lowout + Velocity_Ki4 * Encoder_S;
	return PWM_out;
}

/* ============ 车体位置环（外环）============
 * 反馈取 odometry.x / .y / .theta，返回【速度指令】，喂 Set_Vel(vx, vy, ω)。
 * 后面接的是现成的 Exp_Speed_Cal -> 轮速环 -> Motor_Load，本模块不碰电机。
 * 单位见 mailuncontrol.h 里那几个宏的注释。 */

/* 三路各自独立的滤波状态 / 积分累计量。函数各是单例，所以这块只有一份。
 * 三路积分量全程不清零、跨段延续：停机走的是调用方的 Set_Vel(0,0,0)（根本不调
 * Pos_*），积分就冻在那儿。Ki 很小（≤0.02）、限幅 ±200，残留偏置最多 4mm/s，
 * 且误差一变号积分自己就退回去了，可接受。 */
static float s_err_lp_x,   s_err_sum_x;
static float s_err_lp_y,   s_err_sum_y;
static float s_err_lp_yaw, s_err_sum_yaw;

/* 定义在文件末尾；此处前置声明，避免隐式声明（C99 起是错误） */
static float Angle_ShortestError(float target, float current);

float Pos_X(float Target, float pos_x)
{
	float err, err_lp;

	err    = Target - pos_x;
	err_lp = (1.0f - Pos_A_XY) * err + Pos_A_XY * s_err_lp_x;
	s_err_lp_x = err_lp;

	/* 积分项：累加滤波后的误差，再单独限幅（抗积分饱和）。 */
	s_err_sum_x += err_lp;
	s_err_sum_x = Get_MiMx(s_err_sum_x, -Pos_I_Limit_XY, Pos_I_Limit_XY);

	return Get_MiMx(Pos_Kp_X * err_lp + Pos_Ki_X * s_err_sum_x,
	                -Pos_V_Max, Pos_V_Max);
}

float Pos_Y(float Target, float pos_y)
{
	float err, err_lp;

	err    = Target - pos_y;
	err_lp = (1.0f - Pos_A_XY) * err + Pos_A_XY * s_err_lp_y;
	s_err_lp_y = err_lp;

	s_err_sum_y += err_lp;
	s_err_sum_y = Get_MiMx(s_err_sum_y, -Pos_I_Limit_XY, Pos_I_Limit_XY);

	return Get_MiMx(Pos_Kp_Y * err_lp + Pos_Ki_Y * s_err_sum_y,
	                -Pos_V_Max, Pos_V_Max);
}

float Pos_Yaw(float Target, float pos_theta, float Yaw)
{
	float err, err_lp;

	(void)Yaw;      /* 保留形参：以后做「麦轮正解角 + 陀螺仪角」融合用（决定 D） */

	/* 误差是 deg，输出直接就是 rad/s，所以 Kp 的单位是 rad/(s·deg) ——
	 * 和 x/y 的 1/s 不是同一量纲，数值别横向比。 */
	err    = Angle_ShortestError(Target, pos_theta);
	err_lp = (1.0f - Pos_A_Yaw) * err + Pos_A_Yaw * s_err_lp_yaw;
	s_err_lp_yaw = err_lp;

	s_err_sum_yaw += err_lp;
	s_err_sum_yaw = Get_MiMx(s_err_sum_yaw, -Pos_I_Limit_Yaw, Pos_I_Limit_Yaw);

	return Get_MiMx(Pos_Kp_Yaw * err_lp + Pos_Ki_Yaw * s_err_sum_yaw,
	                -Pos_W_Max, Pos_W_Max);
}

/**********************************************************************
 * @brief  将角度误差归一化到 [-180°, +180°], 确保走最短路径
 * @param  target  目标角度 (度, 任意值)
 * @param  current 当前角度 (度, 任意值)
 * @return float   最短路径误差: + = 需要顺时针转, - = 需要逆时针转
 * @note   例: target=90, current=-170 → 误差 = -100° (逆时针转100° 而非顺转260°)
 **********************************************************************/
static float Angle_ShortestError(float target, float current)
{
    float err = target - current;
    /* 映射到 (-180, +180] */
    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}
