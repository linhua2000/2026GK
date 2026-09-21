#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#include "main.h"

// ================== 机械参数（单位：毫米 mm） ==================
#define WHEEL_DIAMETER          60.0f       // 轮子直径 (mm)
#define WHEEL_CIRCUMFERENCE     (3.1415926f * WHEEL_DIAMETER)  // 轮子周长 (mm)
#define WHEELS_X_DISTANCE       160.0f      // 前后轮中心距 (mm)，即 2a
#define WHEELS_Y_DISTANCE       200.0f      // 左右轮中心距 (mm)，即 2b

// ω 项（差速项）用的等效半径 = a + b。
// 上面两个是「中心距」，即 2a / 2b，所以这里取一半。若误用 160+200=360，
// 角速度对轮速的贡献会翻倍，转起来比指令快一倍。
#define WHEELS_WZ_RADIUS        ((WHEELS_X_DISTANCE + WHEELS_Y_DISTANCE) * 0.5f)  // = 180.0f mm

// 编码器参数：轮子转一圈产生的总脉冲数
// MG513-P30 GMR：500ppr
// 轮子一圈 = `500 × 4 × 30 = 60000`
#define ENCODER_PPR             60000.0f

// 最大轮速 (RPM)
#define MAX_RPM                 200.0f

// ================== 逆运动学符号（标准麦轮 X 布局） ==================
// 约定：+vx = 车头方向，+vy = 左侧，+ω = 俯视逆时针。
// 实车对不上时按下面三条改这些宏，不要去动 Exp_Speed_Cal 里的公式：
//   1) 命令 vy = +100（左移）车却右移     -> WHEEL_VY_SIGN_1..4 四个一起取反
//   2) 命令 ω 为正值（逆时针）车却顺时针  -> WHEEL_WZ_SIGN_1..4 四个一起取反
//   3) 命令 vx = +100（前进）车却后退     -> 那是电机/编码器整体极性问题，
//      去 motor / encoder 层改，别在这里动
#define WHEEL_VY_SIGN_1         (+1.0f)     // 左后
#define WHEEL_VY_SIGN_2         (-1.0f)     // 左前
#define WHEEL_VY_SIGN_3         (+1.0f)     // 右前
#define WHEEL_VY_SIGN_4         (-1.0f)     // 右后
#define WHEEL_WZ_SIGN_1         (-1.0f)     // 左后
#define WHEEL_WZ_SIGN_2         (-1.0f)     // 左前
#define WHEEL_WZ_SIGN_3         (+1.0f)     // 右前
#define WHEEL_WZ_SIGN_4         (+1.0f)     // 右后

// ================== 数据结构 ==================
typedef struct {
    float linear_x;         // 目标 X 方向线速度 (mm/s)
    float linear_y;         // 目标 Y 方向线速度 (mm/s)
    float angular_z;        // 目标 Z 轴角速度 (rad/s)
} Exp_Vel_t;

typedef struct {
    float motor_1;          // 左后 BL (RPM)  = ENC_WHEEL1 = Motor_Load 第 1 个参数
    float motor_2;          // 左前 FL (RPM)  = ENC_WHEEL2
    float motor_3;          // 右前 FR (RPM)  = ENC_WHEEL3
    float motor_4;          // 右后 BR (RPM)  = ENC_WHEEL4
} Wheel_RPM_t;

typedef struct {
    Exp_Vel_t    exp_vel;
    Wheel_RPM_t  exp_wheel_rpm;
    Wheel_RPM_t  fb_wheel_rpm;

    float wheel_circumference_; // 轮子周长 (mm)
    float wheels_x_distance_;   // 前后轮中心距 (mm)
    float wheels_y_distance_;   // 左右轮中心距 (mm)
    float max_rpm_;
} Kinematics_t;

extern Kinematics_t kinematics;

void Set_Vel(float linear_x, float linear_y, float angular_z);
void Exp_Speed_Cal(void);
float Get_MiMx(float value, float min, float max);
void Kinematics_Init(void);

// 编码器增量 <-> 轮速。两个都是纯换算、无副作用，也刻意不依赖 encoder.h
// （本模块保持纯数学，读编码器的动作放在调用方）。
//
// 推导（TIM6 每 5ms 采一次，本次增量 d 个脉冲）：
//     d / ENCODER_PPR   = 这 5ms 里轮子转过的圈数
//     x 200             = 圈/秒      (1s / 5ms = 200)
//     x 60              = 圈/分
//   => RPM = d * 12000 / ENCODER_PPR
// 反方向同理：d = RPM * ENCODER_PPR / 12000
// 代入当前 ENCODER_PPR = 60000：RPM = d * 0.2，d = RPM * 5
//
// 注意 12000 这个常数是由「5ms 采样」推出来的（200 x 60）。若哪天改 TIM6 周期，
// 必须同步改这两个函数里的常数。
float Kinematics_Pulse_To_RPM(float pulse_5ms);
float Kinematics_RPM_To_Pulse(float rpm);

#endif
