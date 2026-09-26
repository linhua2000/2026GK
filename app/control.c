#include "control.h"
#include "main.h"
#include "encoder.h"
#include "motor.h"
#include "mailuncontrol.h"
#include "kinematics.h"
#include "uart1.h"
#include <stdio.h>

/* volatile 的理由见 control.h */
volatile int32_t Debug_Target[DEBUG_WHEEL_NUM] = {0, 0, 0, 0};
volatile int32_t Debug_Pwm[DEBUG_WHEEL_NUM]    = {0, 0, 0, 0};

/* ================= 指令接收 =================
 * 收满一行就地解析。刻意不用 sscanf —— 这段跑在 USART1 中断里，
 * scanf 家族不可重入，而主循环同时在用 sprintf，两边撞 stdio 内部状态风险太大。
 * 语法就这么点，手写反而更短也更快（几微秒）。
 */
#define CMD_LINE_MAX 48
static char    s_cmd[CMD_LINE_MAX];
static uint8_t s_cmd_len  = 0;
static uint8_t s_cmd_drop = 0;     /* 1 = 本行已超长，整行作废，丢到换行为止 */

/* "#N v" 或 "#a v1 v2 v3 v4"。返回 1 = 合法且已写入，0 = 作废 */
static uint8_t Cmd_Parse(const char *s)
{
    int32_t v[DEBUG_WHEEL_NUM];
    uint8_t i, n;
    int8_t  idx = -1;          /* -2 = 四路一起；0..3 = 单路 */

    while (*s == ' ' || *s == '\t') s++;
    if (*s != '#') return 0;
    s++;
    while (*s == ' ' || *s == '\t') s++;

    if (*s == 'a' || *s == 'A')      { idx = -2; s++; }
    else if (*s >= '1' && *s <= '4') { idx = (int8_t)(*s - '1'); s++; }
    else return 0;

    for (n = 0; n < DEBUG_WHEEL_NUM; n++)
    {
        int32_t val = 0;
        int32_t sign = 1;
        uint8_t digits = 0;

        while (*s == ' ' || *s == '\t') s++;
        if (*s == '\0') break;
        if (*s == '-')      { sign = -1; s++; }
        else if (*s == '+') { s++; }

        while (*s >= '0' && *s <= '9')
        {
            if (digits >= 7) return 0;     /* 挡掉 int32 溢出（UB） */
            val = val * 10 + (*s - '0');
            s++;
            digits++;
        }
        if (digits == 0) return 0;         /* 该有数字的位置是垃圾 -> 整行作废 */
        v[n] = sign * val;
    }

    while (*s == ' ' || *s == '\t') s++;
    if (*s != '\0') return 0;

    if (idx == -2)
    {
        if (n != DEBUG_WHEEL_NUM) return 0;   /* #a 必须正好四个 */
        for (i = 0; i < DEBUG_WHEEL_NUM; i++) Debug_Target[i] = v[i];
    }
    else
    {
        if (n != 1) return 0;                 /* 单路必须正好一个 */
        Debug_Target[idx] = v[0];
    }
    return 1;
}

void Debug_RxByte(uint8_t b)
{
    if (b == '\n' || b == '\r')
    {
        if (s_cmd_len > 0 && !s_cmd_drop)
        {
            s_cmd[s_cmd_len] = '\0';
            Cmd_Parse(s_cmd);          /* 不合法就静默丢弃 */
        }
        s_cmd_len  = 0;
        s_cmd_drop = 0;
        return;                        /* \r\n 的第二个字符：长度已是 0，空转一下 */
    }
    if (s_cmd_drop) return;            /* 本行已作废，剩下的字节直接扔 */
    if (s_cmd_len < CMD_LINE_MAX - 1) s_cmd[s_cmd_len++] = (char)b;
    else s_cmd_drop = 1;               /* 超长：整行作废（只丢尾巴会让长行的尾部被误解析） */
}

///* ================= 20ms 回传 ================= */
//void Debug_Poll(void)
//{
////    static uint32_t s_tick = 0;
//    uint8_t txbuf[160];
////    uint32_t now = HAL_GetTick();
//    int len;

////    if ((uint32_t)(now - s_tick) < 20U) return;
////    s_tick = now;

//    /* 一次 sprintf 组整行，再 UART1_Send_Buf 一次性发。
//     * 别用 UART1_Send_Str —— 它逐字节调 HAL_UART_Transmit，94 字节=94 次函数调用，
//     * 且每字节都带 100ms 超时。Send_Buf 是单次调用、单次超时。
//     * 全是 %d，不碰 %f（本工程 %f 从未被链接过，会多带几 KB 进来）。 */
//    len = sprintf((char *)txbuf,
//                  "T1:%d E1:%d P1:%d T2:%d E2:%d P2:%d "
//                  "T3:%d E3:%d P3:%d T4:%d E4:%d P4:%d\r\n",
//                  (int)Debug_Target[0], (int)Encoder_GetDelta(ENC_WHEEL1), (int)Debug_Pwm[0],
//                  (int)Debug_Target[1], (int)Encoder_GetDelta(ENC_WHEEL2), (int)Debug_Pwm[1],
//                  (int)Debug_Target[2], (int)Encoder_GetDelta(ENC_WHEEL3), (int)Debug_Pwm[2],
//                  (int)Debug_Target[3], (int)Encoder_GetDelta(ENC_WHEEL4), (int)Debug_Pwm[3]);
//    UART1_Send_Buf(txbuf, (uint16_t)len);
//}

/* ================= 5ms 闭环 ================= */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM6)
    {
        Encoder_Update();   /* 5ms：读取四路编码器增量（脉冲/5ms） */

        Set_Vel(0, -0, 0.60 );                      // mm/s, mm/s, rad/s —— 只存不算
        Exp_Speed_Cal();                         // 解算 -> exp_wheel_rpm (RPM)

        int p1 = 0, p2 = 0, p3 = 0, p4 = 0;
        p1 = Velocity_Wheel1(Kinematics_RPM_To_Pulse(kinematics.exp_wheel_rpm.motor_1),
                            (int)Encoder_GetDelta(ENC_WHEEL1));
        p2 = Velocity_Wheel2(Kinematics_RPM_To_Pulse(kinematics.exp_wheel_rpm.motor_2),
                            (int)Encoder_GetDelta(ENC_WHEEL2));
        p3 = Velocity_Wheel3(Kinematics_RPM_To_Pulse(kinematics.exp_wheel_rpm.motor_3),
                            (int)Encoder_GetDelta(ENC_WHEEL3));
        p4 = Velocity_Wheel4(Kinematics_RPM_To_Pulse(kinematics.exp_wheel_rpm.motor_4),
                            (int)Encoder_GetDelta(ENC_WHEEL4));
        /* 反馈轮速：只写 kinematics.fb_wheel_rpm 这个普通结构体，不驱动任何电机。
         * 注意下面那条 PID 通路并不用它 —— Velocity_WheelN 的 Target 和 encoder
         * 都是脉冲/5ms，两边单位一致，直接对着原始增量比。这里纯粹给遥测/观察用。
         * 四路同向为正（分配表.md「编码器极性备注」），故无需按轮符号。 */
        // kinematics.fb_wheel_rpm.motor_1 = Kinematics_Pulse_To_RPM((float)Encoder_GetDelta(ENC_WHEEL1));
        // kinematics.fb_wheel_rpm.motor_2 = Kinematics_Pulse_To_RPM((float)Encoder_GetDelta(ENC_WHEEL2));
        // kinematics.fb_wheel_rpm.motor_3 = Kinematics_Pulse_To_RPM((float)Encoder_GetDelta(ENC_WHEEL3));
        // kinematics.fb_wheel_rpm.motor_4 = Kinematics_Pulse_To_RPM((float)Encoder_GetDelta(ENC_WHEEL4));

        /* 四路速度环闭环。目标来自蓝牙 Debug_Target[]，单位脉冲/5ms。
         * 增益是 mailuncontrol.h 里的宏 —— 现在是 0.0f，所以 p1..p4 恒为 0、电机不动。
         * 恢复时把下面整块一起取消注释：声明也在块里。只放开 Motor_Load 那一行的话，
         * p1..p4 就是未初始化变量，中断会拿栈上残值直接驱动电机。 */
        // int p1 = 0, p2 = 0, p3 = 0, p4 = 0;
        // p1 = Velocity_Wheel1((float)Debug_Target[0], (int)Encoder_GetDelta(ENC_WHEEL1));
        // p2 = Velocity_Wheel2((float)Debug_Target[1], (int)Encoder_GetDelta(ENC_WHEEL2));
        // p3 = Velocity_Wheel3((float)Debug_Target[2], (int)Encoder_GetDelta(ENC_WHEEL3));
        // p4 = Velocity_Wheel4((float)Debug_Target[3], (int)Encoder_GetDelta(ENC_WHEEL4));



        Debug_Pwm[0] = p1;              /* 存起来给主循环回传 */
        Debug_Pwm[1] = p2;
        Debug_Pwm[2] = p3;
        Debug_Pwm[3] = p4;

        Motor_Load(p1, p2, p3, p4);     /* 内部 Limit() 夹到 ±1050，无需另加限幅 */
    }
}
