#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include <stdint.h>

/* 正运动学 + 里程计：四路编码器增量 -> 车体 vx/vy/ω -> 积分 -> 世界坐标 x/y/θ。
 *
 * 坐标系（与 kinematics.h 的约定一致）：上电瞬间为准 —— 上电位置 = (0,0)，
 * 上电时车头方向 = +x；+y = 左侧，θ 逆时针为正。
 *
 * 正解就是逆解的转置：WHEEL_VY_SIGN_n / WHEEL_WZ_SIGN_n 两边共用，方向天然一致
 * （逆解方向调对了，正解就跟着对），所以本模块不需要任何按轮符号。
 * 唯一要标定的是航向 YAW_SIGN —— 在 odometry.c 里。 */

typedef struct {
    float x;        /* mm，上电位置为原点 */
    float y;        /* mm，左为正 */
    float theta;    /* deg，上电朝向为 0，逆时针为正 */

    float vx;       /* mm/s，车体前向 */
    float vy;       /* mm/s，车体左向 */
    float wz;       /* rad/s，逆时针为正；编码器解出，本轮只存不用 */

    float yaw0;     /* 抓到的航向零点 (deg) */
    uint8_t ready;  /* 0 = 陀螺仪还没出帧，坐标未开始累积 */
} Odometry_t;

extern Odometry_t odometry;

void Odometry_Init(void);

/* 必须在 TIM6 的 5ms 中断里、紧跟 Encoder_Update() 之后调用 ——
 * 它读的 Encoder_GetDelta() 返回的就是那一拍刚采到的增量。 */
void Odometry_Update(void);

/* 运行时重设航向零点: 把当前陀螺仪 Yaw 记为新的 yaw0, theta 清零。 */
void Odometry_ResetYaw0(void);

#endif
