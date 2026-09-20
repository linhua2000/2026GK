/* ============================================================
 * 速度环 PI 控制 —— 四路麦轮
 * 采样在 TIM6 的 5ms 中断里触发，encoder 参数单位 = 脉冲/5ms。
 * 四路增益各自独立，在 mailuncontrol.h；当前都是 0.0f（未标定）—— 输出恒 0。
 * ============================================================ */
#include "mailuncontrol.h"

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
