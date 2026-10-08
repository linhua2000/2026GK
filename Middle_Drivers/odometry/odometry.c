/* ============================================================
 * 正运动学 + 里程计
 *
 * 逆解（kinematics.c）是  v_i = vx + sy_i·vy + sw_i·(R·ω)。
 * 那八个符号宏满足 Σsy = Σsw = Σsy·sw = 0 且 Σsy² = Σsw² = 4，
 * 四行两两正交，所以最小二乘解就是转置除以 4 —— 精确解，不是近似：
 *
 *     vx = (v1 + v2 + v3 + v4) / 4
 *     vy = (sy1·v1 + sy2·v2 + sy3·v3 + sy4·v4) / 4
 *     ω  = (sw1·v1 + sw2·v2 + sw3·v3 + sw4·v4) / 4 / WHEELS_WZ_RADIUS
 *
 * 输出 vx/vy 单位 mm/s、ω 单位 rad/s。
 *
 * 航向拿陀螺仪（Yaw）作主；编码器积出的 ω 只写进 odometry.wz 备用，不参与 x/y。
 * ============================================================ */
#include "odometry.h"
#include "encoder.h"
#include "kinematics.h"
#include "jy61p.h"
#include <math.h>

/* 实车对不上就翻这里：从上往下看逆时针转、theta 却减小 -> 改成 -1.0f。
 * 对应验收第 4 步。 */
#define YAW_SIGN    (+1.0f)

#define DEG2RAD     (3.1415926f / 180.0f)

/* TIM6 周期。改这里的话 kinematics.h 里那两个换算函数的 12000 也要一起改
 * （12000 = 200Hz × 60），否则脉冲<->RPM 会跟着错。 */
#define DT_S        (0.005f)

Odometry_t odometry;

/* 每脉冲毫米数 = 轮周长 / 每圈脉冲数。Init 里算一次存着，别每 5ms 除。 */
static float s_mm_per_pulse;

void Odometry_Init(void)
{
    odometry.x     = 0.0f;
    odometry.y     = 0.0f;
    odometry.theta = 0.0f;

    odometry.vx    = 0.0f;
    odometry.vy    = 0.0f;
    odometry.wz    = 0.0f;

    odometry.yaw0  = 0.0f;
    odometry.ready = 0;

    s_mm_per_pulse = WHEEL_CIRCUMFERENCE / ENCODER_PPR;
}

void Odometry_Update(void)
{
    float theta, th;
    float v1, v2, v3, v4;
    float vx, vy, wz;
    float dx_body, dy_body, c, s;

    /* ---- 第一段：等陀螺仪出帧，再记航向零点 ----
     * Yaw 上电初值是 0.0f，但模块第一帧要等几十毫秒才到，这时候抓零点会抓到错的
     * 0，所以等 jy61p 的 Yaw_Valid 置位。这一拍只记零点、不积分（也不补算）。 */
    if (!odometry.ready)
    {
        if (!Yaw_Valid) return;

        odometry.yaw0  = Yaw;
        odometry.theta = 0.0f;
        odometry.ready = 1;
        return;
    }

    /* ---- 第二段：航向 ---- */
    theta = (Yaw - odometry.yaw0) * YAW_SIGN;
    odometry.theta = theta;
    th = theta * DEG2RAD;

    /* ---- 第三段：正解 ----
     * 编码器增量 -> 每轮线速度 (mm/s)。四路同向为正（分配表.md「编码器极性备注」），
     * 不需要按轮翻号。 */
    v1 = (float)Encoder_GetDelta(ENC_WHEEL1) * s_mm_per_pulse / DT_S;
    v2 = (float)Encoder_GetDelta(ENC_WHEEL2) * s_mm_per_pulse / DT_S;
    v3 = (float)Encoder_GetDelta(ENC_WHEEL3) * s_mm_per_pulse / DT_S;
    v4 = (float)Encoder_GetDelta(ENC_WHEEL4) * s_mm_per_pulse / DT_S;

    /* 符号宏直接复用 kinematics.h 的那八个 —— 正解是逆解的转置，共用同一套 */
    vx = (v1 + v2 + v3 + v4) * 0.25f;
    vy = (WHEEL_VY_SIGN_1 * v1 + WHEEL_VY_SIGN_2 * v2 +
          WHEEL_VY_SIGN_3 * v3 + WHEEL_VY_SIGN_4 * v4) * 0.25f;
    wz = (WHEEL_WZ_SIGN_1 * v1 + WHEEL_WZ_SIGN_2 * v2 +
          WHEEL_WZ_SIGN_3 * v3 + WHEEL_WZ_SIGN_4 * v4) * 0.25f / WHEELS_WZ_RADIUS;

    odometry.vx = vx;
    odometry.vy = vy;
    odometry.wz = wz;

    /* ---- 积分：车体位移先转到世界系再累加 ----
     * +y 是左侧、θ 逆时针为正，与 kinematics.h 的约定一致。 */
    dx_body = vx * DT_S;
    dy_body = vy * DT_S;

    c = cosf(th);
    s = sinf(th);

    odometry.x += dx_body * c - dy_body * s;
    odometry.y += dx_body * s + dy_body * c;
}

/* 运行时重设航向零点: 把当前 Yaw 记为新的 yaw0, theta 归零。 */
void Odometry_ResetYaw0(void)
{
    odometry.yaw0  = Yaw;
    odometry.theta = 0.0f;
}
