/* ============================================================
 * 逆运动学 —— vx / vy / ω  ->  四路目标转速 (RPM)
 *
 * 轮序与 ENC_WHEEL1..4、Motor_Load(m1..m4) 完全一致：
 *   motor_1 = 左后 BL     motor_2 = 左前 FL
 *   motor_3 = 右前 FR     motor_4 = 右后 BR
 * 所以 kinematics 的输出可以按同序直接喂给 Velocity_WheelN / Motor_Load，
 * 中间不需要任何换序。
 *
 * 约定：+vx = 车头方向，+vy = 左侧，+ω = 俯视逆时针
 * 单位：vx/vy = mm/s，ω = rad/s，输出 RPM。
 * 符号宏在 kinematics.h，实车对不上就翻那里的 WHEEL_VY_SIGN_x / WHEEL_WZ_SIGN_x。
 * ============================================================ */
#include "kinematics.h"

/* 头文件里是 extern，全工程必须有且仅有这一处定义 */
Kinematics_t kinematics;

void Kinematics_Init(void)
{
    kinematics.wheel_circumference_ = WHEEL_CIRCUMFERENCE;
    kinematics.wheels_x_distance_   = WHEELS_X_DISTANCE;
    kinematics.wheels_y_distance_   = WHEELS_Y_DISTANCE;
    kinematics.max_rpm_             = MAX_RPM;

    /* 显式逐字段清零。不用 memset：结构体全是 float，memset 成 0 在 IEEE754
     * 下确实等于 +0.0f，但没必要依赖这个巧合。 */
    kinematics.exp_vel.linear_x  = 0.0f;//目标车体速度
    kinematics.exp_vel.linear_y  = 0.0f;
    kinematics.exp_vel.angular_z = 0.0f;

    kinematics.exp_wheel_rpm.motor_1 = 0.0f;//目标轮速
    kinematics.exp_wheel_rpm.motor_2 = 0.0f;
    kinematics.exp_wheel_rpm.motor_3 = 0.0f;
    kinematics.exp_wheel_rpm.motor_4 = 0.0f;

    kinematics.fb_wheel_rpm.motor_1 = 0.0f;//反馈轮速
    kinematics.fb_wheel_rpm.motor_2 = 0.0f;
    kinematics.fb_wheel_rpm.motor_3 = 0.0f;
    kinematics.fb_wheel_rpm.motor_4 = 0.0f;
}

/* 写目标车体速度。只存不算 —— 解算由 Exp_Speed_Cal() 做 */
void Set_Vel(float linear_x, float linear_y, float angular_z)
{
    kinematics.exp_vel.linear_x  = linear_x;
    kinematics.exp_vel.linear_y  = linear_y;
    kinematics.exp_vel.angular_z = angular_z;
}

/* 标量钳位：返回夹在 [min, max] 之间的 value */
float Get_MiMx(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

/* 逆运动学解算：exp_vel -> exp_wheel_rpm。
 * 三个中间量统一到 mm/s，再按轮子周长换算成 RPM，最后四路一起比例缩放。 */
void Exp_Speed_Cal(void)
{
    float vx = kinematics.exp_vel.linear_x;
    float vy = kinematics.exp_vel.linear_y;
    /* rad/s × mm = mm/s。四路共用的差速项，半径 = a + b（见 kinematics.h） */
    float wz = kinematics.exp_vel.angular_z * WHEELS_WZ_RADIUS;

    /* 各轮线速度 (mm/s) */
    float v1 = vx + WHEEL_VY_SIGN_1 * vy + WHEEL_WZ_SIGN_1 * wz;   /* 左后 */
    float v2 = vx + WHEEL_VY_SIGN_2 * vy + WHEEL_WZ_SIGN_2 * wz;   /* 左前 */
    float v3 = vx + WHEEL_VY_SIGN_3 * vy + WHEEL_WZ_SIGN_3 * wz;   /* 右前 */
    float v4 = vx + WHEEL_VY_SIGN_4 * vy + WHEEL_WZ_SIGN_4 * wz;   /* 右后 */

    /* mm/s -> RPM：v / 周长 = 转/秒，×60 = 转/分 */
    float k       = 60.0f / kinematics.wheel_circumference_;
    float max_rpm = kinematics.max_rpm_;

    float r1 = v1 * k;
    float r2 = v2 * k;
    float r3 = v3 * k;
    float r4 = v4 * k;

    /* 比例缩放：找出绝对值最大的一路，四路乘同一个 scale。
     * 不用逐路钳位 —— 那会把四轮之间的比例压平：例如 vx=1000、ω=1 时四路本应
     * {820, 820, 1180, 1180} mm/s，逐路夹完变成四路全 200 RPM（四路相等 =
     * 旋转分量整个丢失，车会走偏）；缩放后是 {139, 139, 200, 200}，比例原样保住。
     * 代价是超速时整体慢下来，而不是某一路掉队。 */
    float a1 = (r1 < 0.0f) ? -r1 : r1;   /* 手写取绝对值，不引 <math.h> */
    float a2 = (r2 < 0.0f) ? -r2 : r2;
    float a3 = (r3 < 0.0f) ? -r3 : r3;
    float a4 = (r4 < 0.0f) ? -r4 : r4;

    float max_abs = a1;
    if (a2 > max_abs) max_abs = a2;
    if (a3 > max_abs) max_abs = a3;
    if (a4 > max_abs) max_abs = a4;

    float scale = (max_abs > max_rpm) ? (max_rpm / max_abs) : 1.0f;

    kinematics.exp_wheel_rpm.motor_1 = r1 * scale;
    kinematics.exp_wheel_rpm.motor_2 = r2 * scale;
    kinematics.exp_wheel_rpm.motor_3 = r3 * scale;
    kinematics.exp_wheel_rpm.motor_4 = r4 * scale;
}

/* RPM = d * 12000 / ENCODER_PPR，推导见 kinematics.h */
float Kinematics_Pulse_To_RPM(float pulse_5ms)
{
    return pulse_5ms * 12000.0f / ENCODER_PPR;
}

/* d = RPM * ENCODER_PPR / 12000，推导见 kinematics.h */
float Kinematics_RPM_To_Pulse(float rpm)
{
    return rpm * ENCODER_PPR / 12000.0f;
}
