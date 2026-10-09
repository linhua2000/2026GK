#include "control.h"
#include "main.h"
#include "encoder.h"
#include "motor.h"
#include "mailuncontrol.h"
#include "kinematics.h"
#include "odometry.h"
#include "receive.h"
#include "jy61p.h"
#include "uart1.h"
#include "key.h"
#include "PID.h"
#include <stdio.h>
#include <math.h>

/* EasyLogger：LOG_TAG / LOG_LVL 必须先于 <elog.h> 定义 */
#define LOG_TAG    "CTRL"
#define LOG_LVL    ELOG_LVL_VERBOSE
#include <elog.h>

/* volatile 的理由见 control.h */
volatile int32_t Debug_Target[DEBUG_WHEEL_NUM] = {0, 0, 0, 0};
volatile int32_t Debug_Pwm[DEBUG_WHEEL_NUM]    = {0, 0, 0, 0};

/* 按键跑/停开关（定义见 control.h）。KEY_1 单击翻转：0 = 停，1 = 跑状态机。 */
volatile uint8_t KeyNum = 0;

/* 开机闸门：上电 1.5s（Timer 数到 300 拍 × 5ms）后 flag_Numdelay 置 1，之后不再回 0。
 * Timer 必须是 uint16_t —— uint8_t 最大 255、到 256 就回绕，`Timer > 300` 永远不成立。 */
static uint8_t  flag_Numdelay;
static uint16_t Timer;
/* 后轮打滑段开关(定义在此, 由 encoder.h extern 出去): =1 时后轮不驱动 + 编码器
 * 用同侧前轮代替后轮。默认 0 = 四轮正常。 */
uint8_t  No_rear_wheels = 0;
uint8_t  No_front_wheels = 0;
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

///* ================= 20ms 回传（已改为 elog 写法） ================= */
//void Debug_Poll(void)
//{
//    static uint32_t s_tick = 0;
//    uint32_t now = HAL_GetTick();
//
//    if ((uint32_t)(now - s_tick) < 20U) return;
//    s_tick = now;
//
//    /* 全是 %d，不碰 %f（本工程浮点 printf 从未链接过）。
//     * elog 自己组行并阻塞发到 USART1，不再需要 txbuf/sprintf/UART1_Send_Buf。 */
//    log_i("T1:%d E1:%d P1:%d T2:%d E2:%d P2:%d "
//          "T3:%d E3:%d P3:%d T4:%d E4:%d P4:%d",
//          (int)Debug_Target[0], (int)Encoder_GetDelta(ENC_WHEEL1), (int)Debug_Pwm[0],
//          (int)Debug_Target[1], (int)Encoder_GetDelta(ENC_WHEEL2), (int)Debug_Pwm[1],
//          (int)Debug_Target[2], (int)Encoder_GetDelta(ENC_WHEEL3), (int)Debug_Pwm[2],
//          (int)Debug_Target[3], (int)Encoder_GetDelta(ENC_WHEEL4), (int)Debug_Pwm[3]);
//}

/* ================= 里程计路径状态机 =================
 * 按 odometry.x/y 走一条 10 段阶梯：每段【沿程那根轴给定速、横向那根轴由位置环按住】，
 * 航向由 Pos_Yaw(0, odometry.theta, 0) 锁在 0°；走够距离就停、原地等三秒、再换下一段。
 * 只在 TIM6 的 5ms 中断里跑（在 Odometry_Update() 之后、Exp_Speed_Cal() 之前），
 * 所以它写的 Set_Vel 当拍就解算生效。
 *
 * 速度单位：vx/vy = mm/s，ω = rad/s。位置环只管「不跑偏」，沿程走多远仍由下面那句
 * odometry 阈值判定 —— 两者是独立的，改路径尺寸要同时改 Set_Vel 里的保持目标。
 * 刻意不加超时兜底：到位条件不成立就一直走 —— 上电前把车放好。
 *
 * sm_t 不再初始化：静态 0。活跃状态机 StateMachine_Update 的 SM_IDLE 现在是
 * 「等陀螺仪出帧 -> 让主循环做 part7 看直线位姿 -> 进 MOVE1」，不再依赖 sm_t 的三秒计时。
 */
typedef enum {
    SM_IDLE = 0,                        /* 上电先做 part7 看直线位姿, 完成后进 MOVE1 */
    SM_MOVE1, SM_HOLD1,
    SM_MOVE2, SM_MOVE2_ADJUST, SM_HOLD2,
    SM_MOVE3, SM_HOLD3,
    SM_MOVE4, SM_HOLD4,
    SM_MOVE5, SM_HOLD5,
    SM_MOVE6, SM_HOLD6,
    SM_MOVE7, SM_HOLD7,
    SM_MOVE8, SM_HOLD8,
    SM_MOVE9, SM_HOLD9,
    SM_MOVE9_ADJUST,                    /* HOLD9 后短距离后退(相对距离) */
    SM_HOLD9_POSE,                      /* 摆 HOLD8 看直线位姿(舵机3→2→1)+发 B6 06, 车停住 */
    SM_MOVE10,                          /* 第 10 段后面没有 HOLD，直接进 DONE */
    SM_DONE
} SM_State;

#define SM_HOLD_MS   3000U

/* MOVE3/HOLD3 的 x 落点修正量(mm)。打滑让 odometry.x 有系统偏差时,
 * 用 1630 + Slip_Offset 把目标挪一点。0.0f = 不修正(行为不变)。line_test() 与 StateMachine_Update() 都用。 */
#define Slip_Offset  (89.0f)

/* ==================== 行驶路线参数表 (唯一标定处) ====================
 * 每一段的位置判据 = 「进入本段时的里程计位置(sm_ex/sm_ey) + 本段增量」。
 * 所以改地图只需改本段增量, 不会牵连别的段。单位 mm; 正负 = 里程计坐标方向
 * (x 右为正 / y 左为正, 见 odometry.c 约定)。
 *
 * 命名: R_<段>_D    本段沿主轴要走的距离(出口判据)
 *       R_<段>_C    本段另一轴要保持在的偏移(相对进入点)
 *       R_<段>_CX/CY  HOLD 段两轴的保持偏移(相对进入点)
 *
 * ★ = 需现场标定。下列数值全部从旧绝对阈值反推, 是占位值 —— 上车逐段核对再改。
 * ◎ = 视觉段: 控制律走视觉(见段内注释), 表里只放里程计出口/保持量。
 *
 * 特殊段(不查本表): HOLD7/HOLD9_POSE(纯停住) 无里程计判据。
 * ==================================================================== */

/* ---- MOVE1: 起步沿 +X 直行, 到第一张二维码前的位姿 ★ ---- */
#define R_MOVE1_D     630.0f    /* 前进距离(旧: x>630, 起点 x≈0) */
#define R_MOVE1_C       0.0f    /* y 保持(旧: Pos_Y(0), 起点 y≈0) */

/* ---- HOLD1: 两阶段视觉矫正 ◎ x 由视觉 vx 驱动(不用 R_HOLD1_CX), 仅 y 保持 ---- */
#define R_HOLD1_DIST   43.0f    /* ★ 视觉 C7 的 X 目标距离(turn_y 的目标值); 现场标定 */
#define R_HOLD1_CY      0.0f    /* y 保持(旧: Pos_Y(0)) */

/* ---- MOVE2: 沿 +Y 左移, x 保持 ★ ---- */
#define R_MOVE2_D     560.0f    /* 左移距离(旧: y>580, 起点 y≈0) */
#define R_MOVE2_C       0.0f    /* x 保持(旧: Pos_X(650)) */

/* ---- MOVE2_ADJUST: 微调, x 前挪 100、y 拉到 90 ★ ---- */
#define R_MOVE2A_DX   100.0f    /* x 前进(旧: 出口 x>750; 起点 x≈650) */
#define R_MOVE2A_CY   110.0f    /* y 保持(旧: Pos_Y(670); 起点 y≈580) */

/* ---- HOLD2: 不停顿, 只摆目标, 立刻进 MOVE3（无用）---- */
#define R_HOLD2_CX    -10.0f    /* x 保持(旧: Pos_X(740); 起点 x≈750) */
#define R_HOLD2_CY      0.0f    /* y 保持(旧: Pos_Y(670)) */

/* ---- MOVE3: 沿 +X 长距离直行(后轮打滑段) ★ ---- */
#define R_MOVE3_D     938.0f    /* 前进距离(旧: x>1630+Slip=1719; 起点 x≈750) */
#define R_MOVE3_C       0.0f    /* y 保持(旧: Pos_Y(670)) */

/* ---- HOLD3: 视觉矫正 X距离+角度 ◎ x 由视觉 vx 驱动(不用 R_HOLD3_CX), 仅 y 保持 ---- */
#define R_HOLD3_DIST   43.0f    /* ★ 视觉 C7 的 X 目标距离(turn_y 的目标值); 现场标定 */
#define R_HOLD3_CY      0.0f    /* y 保持(旧: Pos_Y(670)) */

/* ---- MOVE4: 沿 +Y 左移 ★ ---- */
#define R_MOVE4_D     720.0f    /* 左移距离(旧: y>1390; 起点 y≈670) */
#define R_MOVE4_C       0.0f    /* x 保持(旧: Pos_X(1630+Slip)) */

/* ---- HOLD4: 停车保持(定时) ★ ---- */
#define R_HOLD4_CX    170.0f    /* x 前挪(旧: Pos_X(1800+Slip)=1889; 起点 x≈1719) */
#define R_HOLD4_CY    123.0f    /* y 保持(旧: Pos_Y(1513); 起点 y≈1390) */

/* ---- MOVE5: 沿 +X 直行 ★ ---- */
#define R_MOVE5_D     750.0f    /* 前进距离(旧: x>2550+Slip=2639; 起点 x≈1889) */
#define R_MOVE5_C      -5.0f    /* y 保持(旧: Pos_Y(1508); 起点 y≈1513) */

/* ---- HOLD5: 停车保持(定时) ★ ---- */
#define R_HOLD5_CX     88.0f    /* x 前挪(旧: Pos_X(2638+Slip)=2727; 起点 x≈2639) */
#define R_HOLD5_CY   -218.0f    /* y 收(旧: Pos_Y(1290); 起点 y≈1508) */

/* ---- MOVE6: 沿 -Y 右移, 只锁航向(不保持 x) ★ ---- */
#define R_MOVE6_D    -430.0f    /* 右移距离(旧: y<860; 起点 y≈1290) */

/* ---- HOLD6: 视觉追球保持 (part2) ◎ ---- */
#define R_HOLD6_CX     -8.0f    /* x 保持(旧: Pos_X(2630+Slip)=2719; 起点 x≈2727) */

/* ---- MOVE7: 视觉 C7 直线矫正, 沿 -Y 右移 ★◎ ---- */
#define R_MOVE7_D   -1700.0f    /* 右移距离(旧: y<-840; 起点 y≈860); vx/角度由视觉给 */

/* ---- MOVE8: 沿 -Y 右移, 只锁航向 ★ ---- */
#define R_MOVE8_D    -560.0f    /* 右移距离(旧: y<-1400; 起点 y≈-840) */

/* ---- HOLD8: 拉回位姿 + 视觉 Y/角度矫正 (part5) ★◎ ---- */
#define R_HOLD8_CX   -238.0f    /* x 拉回(旧: Pos_X(2400+Slip)=2489; 起点 x≈2727) */
#define R_HOLD8_CY    -65.0f    /* y 拉回(旧: Pos_Y(-1490); 起点 y≈-1400) */
#define R_HOLD8_POS_TOL   20.0f    /* HOLD8 到位容差 mm(原 PID.c HOLD8_POS_TOL) */

/* ---- MOVE9: 沿 -X 后退, 只锁航向 ★ ---- */
#define R_MOVE9_D    -750.0f    /* 后退距离(旧: x<1650+Slip=1739; 起点 x≈2489) */

/* ---- MOVE10: 沿 -X 退到终点, 视觉 Y 补偿 ★◎ ---- */
#define R_MOVE10_D  -1265.0f    /* 后退距离(旧: x<50+Slip=139; 起点 x≈1429) */

/* ---- HOLD1 视觉矫正(X距离+角度)完成判据 ---- */
#define HOLD1_ANGLE_TOL   2       /* 角度容差: |turn_x| ≤ 2 (即 0.2°), 与 MOVE7 的 ±2 一致 */
#define HOLD1_DIST_TOL    5       /* X距离容差: |turn_y - 61| ≤ 5 */
#define HOLD1_STABLE_CNT  3       /* 连续满足容差的帧数, 达到才认为矫正完成(去抖) */
#define HOLD1_TIMEOUT_MS  5000U   /* 兜底超时(ms): 2s 没达标也强制进扫码阶段, 避免卡死 */

/* ---- HOLD3 视觉矫正(X距离+角度)完成判据 ---- */
#define HOLD3_ANGLE_TOL   2       /* 角度容差: |turn_x| ≤ 2 (即 0.2°) */
#define HOLD3_DIST_TOL    5       /* X距离容差: |turn_y - R_HOLD3_DIST| ≤ 5 */
#define HOLD3_STABLE_CNT  3       /* 连续满足容差的帧数(去抖) */
#define HOLD3_TIMEOUT_MS  5000U   /* 兜底超时(ms): 2s 没达标也进 MOVE4 */

static SM_State  sm_phase = SM_IDLE;
static uint32_t  sm_t;                  /* 进入 IDLE / HOLD 的时刻 */
static float     sm_x0;                 /* SM_MOVE9_ADJUST 相对距离起点 */
static float     sm_ex;                 /* 进入当前状态时的 odometry.x —— 本段所有位置判据的基准 */
static float     sm_ey;                 /* 进入当前状态时的 odometry.y */
static SM_State  sm_last = SM_IDLE;     /* 检测状态切换, 用于抓进入点快照 */
static uint8_t   hold1_aligned = 0;     /* HOLD1: 1=视觉矫正完成, 进扫码阶段 */
static uint8_t   hold1_cnt     = 0;     /* HOLD1: 连续满足容差的帧数 */
static uint8_t   hold3_cnt     = 0;     /* HOLD3: 连续满足容差的帧数 */

//路线test
static void line_test(void)
{
    switch (sm_phase)
    {
    case SM_IDLE:
        Set_Vel(0, 0, 0);
        /* 等陀螺仪出帧：ready 之前 odometry.x/y 恒为 0，这时候起步第一段会多走一截 */
        if (!odometry.ready) return;
        if ((HAL_GetTick() - sm_t) >= SM_HOLD_MS) sm_phase = SM_MOVE1;
        break;

    case SM_MOVE1:      /* 前进到 x>650（y 按住 0） */
        //	Set_Vel(Pos_X(0,odometry.x),-100,Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
		Set_Vel(160,Pos_Y(0,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,y不变移动x
        if (odometry.x > 650.0f) { sm_t = HAL_GetTick(); sm_phase = SM_HOLD1; }
        break;
    case SM_HOLD1:
        Set_Vel(Pos_X(650.0,odometry.x), Pos_Y(0,odometry.y), Pos_Yaw(0,odometry.theta,0));
        if ((HAL_GetTick() - sm_t) >= SM_HOLD_MS) sm_phase = SM_MOVE2;
        break;

    case SM_MOVE2:      /* 左移到 y>650（x 按住 650） */
        Set_Vel(Pos_X(650.0,odometry.x),160,Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
		// Set_Vel(-100,Pos_Y(0,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,y不变移动x
        if (odometry.y > 580.0f) { sm_t = HAL_GetTick(); sm_phase = SM_MOVE2_ADJUST; }
        break;
		
    case SM_MOVE2_ADJUST:      /* 左移到 y>650（x 按住 650） */
        Set_Vel(Pos_X(750.0,odometry.x),Pos_Y(670,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
		// Set_Vel(-100,Pos_Y(0,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,y不变移动x
        if (odometry.x > 750.0f) { sm_t = HAL_GetTick(); sm_phase = SM_HOLD2; }
        break;
		
    case SM_HOLD2:
        Set_Vel(Pos_X(740.0,odometry.x),Pos_Y(670,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
//        if ((HAL_GetTick() - sm_t) >= SM_HOLD_MS) 
		{sm_phase = SM_MOVE3;}//
        break;

    case SM_MOVE3:      /* 前进到 x>1630（y 按住 650）; 本段后轮打滑 -> 只信前轮 */
//      No_rear_wheels = 1;
//		No_front_wheels = 1;
        //Set_Vel(Pos_X(650.0,odometry.x),100,Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
		Set_Vel(180,Pos_Y(670,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,y不变移动x
        if (odometry.x > 1630.0f+Slip_Offset) {
			No_rear_wheels = 0;
            No_front_wheels = 0;                        /* 出段: 恢复四轮 */
            sm_t = HAL_GetTick(); sm_phase = SM_HOLD3;
        }
        break;
    case SM_HOLD3:
        Set_Vel(Pos_X(1630.0f+Slip_Offset,odometry.x),Pos_Y(670,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
        if ((HAL_GetTick() - sm_t) >= SM_HOLD_MS) {sm_phase = SM_MOVE4;}//
        break;

    case SM_MOVE4:      /* 左移到 y>1500（x 按住 1630） */
        Set_Vel(Pos_X(1630.0f+Slip_Offset,odometry.x),110,Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
		//Set_Vel(100,Pos_Y(650,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,y不变移动x
        if (odometry.y > 1390.0f) { sm_t = HAL_GetTick(); sm_phase = SM_HOLD4; }
        break;
    case SM_HOLD4:
//		Set_Vel(0, 0, 0);
		Set_Vel(Pos_X(1800.0f+Slip_Offset,odometry.x),Pos_Y(1513,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
        if ((HAL_GetTick() - sm_t) >= SM_HOLD_MS) {sm_phase = SM_MOVE5;}//
        break;

    case SM_MOVE5:      /* 前进到 x>2600（y 按住 1500） */
        //Set_Vel(Pos_X(650.0,odometry.x),100,Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
		Set_Vel(150,Pos_Y(1508,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,y不变移动x
        if (odometry.x > 2550.0f+Slip_Offset) { sm_t = HAL_GetTick(); sm_phase = SM_HOLD5; }
        break;
    case SM_HOLD5:
//		Set_Vel(0, 0, 0);
        Set_Vel(Pos_X(2640.0f+Slip_Offset,odometry.x),Pos_Y(1290,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
        if ((HAL_GetTick() - sm_t) >= SM_HOLD_MS) {sm_phase = SM_MOVE6;}//
        break;

    case SM_MOVE6:      /* 右移到 y<850（x 按住 2600） */
		Set_Vel(0,-110,Pos_Yaw(0.075,odometry.theta,0)); //角度,x不变移动y
//      Set_Vel(Pos_X(2640.0f+Slip_Offset,odometry.x),-110,Pos_Yaw(-1.3,odometry.theta,-0.5)); //角度,x不变移动y
		//Set_Vel(100,Pos_Y(650,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,y不变移动x
        if (odometry.y <860.0f) { sm_t = HAL_GetTick(); sm_phase = SM_HOLD6; }
        break;
    case SM_HOLD6:
//		Set_Vel(0, 0, 0);
        Set_Vel(Pos_X(2640.0f+Slip_Offset,odometry.x-7),Pos_Y(860,odometry.y),Pos_Yaw(-0.75,odometry.theta,0)); //角度,x不变移动y
        if ((HAL_GetTick() - sm_t) >= SM_HOLD_MS){ sm_phase = SM_MOVE7;}//
        break;

    case SM_MOVE7:      /* 继续右移到 y<-840（x 按住 2600） */
		Set_Vel(0,-100,Pos_Yaw(0.075,odometry.theta,-0.75)); //角度,x不变移动y
//        Set_Vel(Pos_X(2639.0f+Slip_Offset,odometry.x),-100,Pos_Yaw(0.2,odometry.theta,0)); //角度,x不变移动y
		//Set_Vel(100,Pos_Y(650,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,y不变移动x
        if (odometry.y < -840.0f) { sm_t = HAL_GetTick(); sm_phase = SM_HOLD7; }//
        break;
    case SM_HOLD7:
//		Set_Vel(0, 0, 0);
        Set_Vel(Pos_X(2649.0f+Slip_Offset,odometry.x),Pos_Y(-850,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
        if ((HAL_GetTick() - sm_t) >= SM_HOLD_MS) {sm_phase = SM_MOVE8;}//
        break;

    case SM_MOVE8:      /* 继续右移到 y<-1450（x 按住 2600） */
		  Set_Vel(0,-100,Pos_Yaw(0.1,odometry.theta,0)); //角度,x不变移动y
//        Set_Vel(Pos_X(2649.0f+Slip_Offset,odometry.x),-100,Pos_Yaw(0.1,odometry.theta,0)); //角度,x不变移动y
		//Set_Vel(100,Pos_Y(650,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,y不变移动x
        if (odometry.y < -1400.0f) { sm_t = HAL_GetTick(); sm_phase = SM_HOLD8; }
        break;
    case SM_HOLD8:
//		Set_Vel(0, 0, 0);
		Set_Vel(Pos_X(2400.0f+Slip_Offset,odometry.x),Pos_Y( -1505,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
        if ((HAL_GetTick() - sm_t) >= SM_HOLD_MS) {sm_phase = SM_MOVE9;}//
        break;

    case SM_MOVE9:      /* 后退到 x<1520（y 按住 -1450） */
		
		Set_Vel(-100,0,Pos_Yaw(1.5,odometry.theta,0.0)); //角度,y不变移动x
		//Set_Vel(Pos_X(2500.0,odometry.x),-100,Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
//		Set_Vel(-100,Pos_Y(-1508,odometry.y),Pos_Yaw(4.8,odometry.theta,0.0)); //角度,y不变移动x
        if (odometry.x < 1650.0f+Slip_Offset) { sm_t = HAL_GetTick(); sm_phase = SM_HOLD9; }
        break;
    case SM_HOLD9:
//		Set_Vel(0, 0, 0);
		Set_Vel(Pos_X(1649.0f+Slip_Offset,odometry.x),Pos_Y( -1509,odometry.y),Pos_Yaw(2.25,odometry.theta,0)); //角度,x不变移动y
        if ((HAL_GetTick() - sm_t) >= SM_HOLD_MS) {sm_phase = SM_MOVE10;}//
        break;

    case SM_MOVE9_ADJUST:      /* line_test 不用此状态, 仅为补齐 switch */
        Set_Vel(-100, 0, Pos_Yaw(0, odometry.theta, 0.0));
        if (odometry.x < sm_x0 - 100.0f) sm_phase = SM_MOVE10;
        break;

    case SM_HOLD9_POSE:        /* line_test 不用此状态, 仅为补齐 switch */
        Set_Vel(0, 0, Pos_Yaw(0, odometry.theta, 0));
        if (hold_action_state == HOLD_ACTION_IDLE) { hold_action_id = 6; hold_action_state = HOLD_ACTION_RUN; }
        if (hold_action_state == HOLD_ACTION_DONE) { hold_action_state = HOLD_ACTION_IDLE; hold_action_id = 0; sm_phase = SM_MOVE10; }
        break;

    case SM_MOVE10:     /* 后退到 x<-50（y 按住 -1450） */
		Set_Vel(-100,0,Pos_Yaw(1.62,odometry.theta,0.0)); //角度,y不变移动x
        //Set_Vel(Pos_X(2500.0,odometry.x),-100,Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
//		Set_Vel(-100,Pos_Y(-1518,odometry.y),Pos_Yaw(8.2,odometry.theta,0)); //角度,y不变移动x
        if (odometry.x < 78.0f+Slip_Offset) sm_phase = SM_DONE;
        break;

    case SM_DONE:
        Set_Vel(0, 0, 0);               /* 停住，不再动 */
        break;
    }
}

static void StateMachine_Update(void)
{
    /* 换段就抓一次进入点快照: 本段的位置判据/保持目标全部相对它,
     * 这样改地图只需改本段增量 R_*, 不会波及上游下游任何一段。 */
    if (sm_phase != sm_last) { sm_last = sm_phase; sm_ex = odometry.x; sm_ey = odometry.y; }

    switch (sm_phase)
    {
    case SM_IDLE:
        Set_Vel(0, 0, 0);
        /* 等陀螺仪出帧：ready 之前 odometry.x/y 恒为 0，这时候起步第一段会多走一截 */
        if (!odometry.ready) return;
        if (hold_action_state == HOLD_ACTION_IDLE) { hold_action_id = 7; hold_action_state = HOLD_ACTION_RUN; }  /* 请求主循环做 part7 看直线位姿 */
        if (hold_action_state == HOLD_ACTION_DONE) { hold_action_state = HOLD_ACTION_IDLE; hold_action_id = 0; sm_phase = SM_MOVE1; }
        break;

    case SM_MOVE1:      /* 前进 R_MOVE1_D（y 按住进入点） */
        //	Set_Vel(Pos_X(0,odometry.x),-100,Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
		Set_Vel(160,Pos_Y(sm_ey + R_MOVE1_C,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,y不变移动x
        if (odometry.x > sm_ex + R_MOVE1_D) { sm_t = HAL_GetTick(); hold1_aligned = 0; hold1_cnt = 0; sm_phase = SM_HOLD1; }
        break;
    case SM_HOLD1:      /* 阶段A: 视觉矫正 X距离(turn_y->61)+角度(turn_x->0); 稳/超时后 阶段B: 停住锁航向, part1 扫码 */
        if (!hold1_aligned) {
            float vx = vision_data.turn_flag ? (LINE_KP * ((float)vision_data.turn_y - R_HOLD1_DIST)) : 0.0f;
            Set_Vel(vx, Pos_Y(sm_ey + R_HOLD1_CY, odometry.y), 0.0f);   /* 只锁航向, 不锁角度(由视觉给) */  

            if (vision_data.turn_flag &&
                vision_data.turn_x >= -HOLD1_ANGLE_TOL && vision_data.turn_x <= HOLD1_ANGLE_TOL &&
                (float)vision_data.turn_y >= R_HOLD1_DIST - HOLD1_DIST_TOL &&
                (float)vision_data.turn_y <= R_HOLD1_DIST + HOLD1_DIST_TOL) {
                if (++hold1_cnt >= HOLD1_STABLE_CNT) {
                    //Odometry_ResetYaw0();   /* 视觉角度≈0 -> 重设航向零点 */
                    Pos_Yaw_Reset();
                    hold1_aligned = 1;
                }
            } else {
                hold1_cnt = 0;
            }

            if ((HAL_GetTick() - sm_t) >= HOLD1_TIMEOUT_MS) hold1_aligned = 1;  /* 超时兜底 */
        } else {
            Set_Vel(0, 0, Pos_Yaw(0, odometry.theta, 0));   /* 停住 + 锁航向, 让 part1 扫码 */
            if (hold_action_state == HOLD_ACTION_IDLE) { hold_action_id = 1; hold_action_state = HOLD_ACTION_RUN; }  /* 请求主循环做 part1 扫码+舵机复位 */
            if (hold_action_state == HOLD_ACTION_DONE) { hold_action_state = HOLD_ACTION_IDLE; hold_action_id = 0; sm_phase = SM_MOVE2; }
        }
        break;

    case SM_MOVE2:      /* 左移 R_MOVE2_D（x 按住进入点） */
        Set_Vel(Pos_X(sm_ex + R_MOVE2_C,odometry.x),160,Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
		// Set_Vel(-100,Pos_Y(0,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,y不变移动x
        if (odometry.y > sm_ey + R_MOVE2_D) { sm_t = HAL_GetTick(); sm_phase = SM_MOVE2_ADJUST; }
        break;

    case SM_MOVE2_ADJUST:      /* 微调: x 前挪 R_MOVE2A_DX, y 保持 R_MOVE2A_CY */
        Set_Vel(Pos_X(sm_ex + R_MOVE2A_DX,odometry.x),Pos_Y(sm_ey + R_MOVE2A_CY,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
		// Set_Vel(-100,Pos_Y(0,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,y不变移动x
        if (odometry.x > sm_ex + R_MOVE2A_DX) { sm_t = HAL_GetTick(); sm_phase = SM_HOLD2; }
        break;

    case SM_HOLD2:
// Set_Vel(Pos_X(sm_ex + R_HOLD2_CX,odometry.x),Pos_Y(sm_ey + R_HOLD2_CY,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
//      if ((HAL_GetTick() - sm_t) >= SM_HOLD_MS)
		{sm_phase = SM_MOVE3;}//
        break;

    case SM_MOVE3:      /* 前进 R_MOVE3_D（y 按住进入点, 后轮打滑） */
        //Set_Vel(Pos_X(650.0,odometry.x),100,Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
		Set_Vel(180,Pos_Y(sm_ey + R_MOVE3_C,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,y不变移动x
        if (odometry.x > sm_ex + R_MOVE3_D) {
			No_rear_wheels = 0;
            No_front_wheels = 0;                        /* 出段: 恢复四轮 */
            Vision_Send_B6(0x06);                       /* 切直线模式+清turn滤波, 给HOLD3 X矫正 */
            vision_data.turn_flag = 0;
            sm_t = HAL_GetTick(); hold3_cnt = 0; sm_phase = SM_HOLD3;
        }
        break;

    case SM_HOLD3:      /* 视觉矫正 X距离(turn_y->R_HOLD3_DIST)+角度(turn_x->0); 连续3帧达标或2s超时 -> MOVE4 */
        {
            float vx = vision_data.turn_flag ? (LINE_KP * ((float)vision_data.turn_y - R_HOLD3_DIST)) : 0.0f;
            Set_Vel(vx, Pos_Y(sm_ey + R_HOLD3_CY, odometry.y), 0.0f);   /* 只锁航向, 不锁角度(由视觉给) */  

            if (vision_data.turn_flag &&
                vision_data.turn_x >= -HOLD3_ANGLE_TOL && vision_data.turn_x <= HOLD3_ANGLE_TOL &&
                (float)vision_data.turn_y >= R_HOLD3_DIST - HOLD3_DIST_TOL &&
                (float)vision_data.turn_y <= R_HOLD3_DIST + HOLD3_DIST_TOL) {
                if (++hold3_cnt >= HOLD3_STABLE_CNT) {
                    //Odometry_ResetYaw0();   /* 视觉角度≈0 -> 重设航向零点 */
                    Pos_Yaw_Reset();
                    sm_t = HAL_GetTick(); //sm_phase = SM_MOVE4;
                }
            } else {
                hold3_cnt = 0;
            }

            if ((HAL_GetTick() - sm_t) >= HOLD3_TIMEOUT_MS) { sm_t = HAL_GetTick();	sm_phase = SM_MOVE4; }  /* 超时兜底 *///
        }
        break;

    case SM_MOVE4:      /* 左移 R_MOVE4_D（x 按住进入点） */
        Set_Vel(Pos_X(sm_ex + R_MOVE4_C,odometry.x),110,Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
		//Set_Vel(100,Pos_Y(650,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,y不变移动x
        if (odometry.y > sm_ey + R_MOVE4_D) { sm_t = HAL_GetTick(); sm_phase = SM_HOLD4; }
        break;
    case SM_HOLD4:
//		Set_Vel(0, 0, 0);
		Set_Vel(Pos_X(sm_ex + R_HOLD4_CX,odometry.x),Pos_Y(sm_ey + R_HOLD4_CY,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
        if ((HAL_GetTick() - sm_t) >= SM_HOLD_MS) {sm_phase = SM_MOVE5;}//
        break;

    case SM_MOVE5:      /* 前进 R_MOVE5_D（y 按住进入点） */
        //Set_Vel(Pos_X(650.0,odometry.x),100,Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
		Set_Vel(150,Pos_Y(sm_ey + R_MOVE5_C,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,y不变移动x
        if (odometry.x > sm_ex + R_MOVE5_D) { sm_t = HAL_GetTick(); sm_phase = SM_HOLD5; }
        break;
    case SM_HOLD5:
//		Set_Vel(0, 0, 0);
        Set_Vel(Pos_X(sm_ex + R_HOLD5_CX,odometry.x),Pos_Y(sm_ey + R_HOLD5_CY,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
        if ((HAL_GetTick() - sm_t) >= SM_HOLD_MS) {sm_phase = SM_MOVE6;}//
        break;

    case SM_MOVE6:      /* 右移 R_MOVE6_D（只锁航向, 不保持 x） */
        Set_Vel(0,-110,Pos_Yaw(0.055,odometry.theta,0)); //角度,x不变移动y
		//Set_Vel(100,Pos_Y(650,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,y不变移动x
        if (odometry.y < sm_ey + R_MOVE6_D) { sm_t = HAL_GetTick(); sm_phase = SM_HOLD6; }
        break;
    case SM_HOLD6:
        {
            float vx, vy;
            if (vision_car_vx_track_enable) vx = vision_car_vx;                 /* y补偿: 前后由视觉接管 */
            else                            vx = Pos_X(sm_ex + R_HOLD6_CX, odometry.x-7);
            if (vision_car_track_enable)    vy = vision_car_vy;                 /* x对准: 左右由视觉接管 */
            else                            vy = Pos_Y(860,  odometry.y);
            Set_Vel(vx, vy, 0.0f);   /* 只锁航向, 不锁角度(由视觉给) */
        }
        if (hold_action_state == HOLD_ACTION_IDLE) { hold_action_id = 2; hold_action_state = HOLD_ACTION_RUN; }  /* 请求主循环做 part2 追球抓球+追桶放桶 */
        if (hold_action_state == HOLD_ACTION_DONE) {  hold_action_id = 0; hold_action_state = HOLD_ACTION_IDLE;sm_phase = SM_MOVE7;} //
        break;

    case SM_MOVE7:      /* 右移 R_MOVE7_D; 视觉 C7 距离矫正 x + 角度矫正(目标0) */
        {
            if (!vision_data.turn_flag) {          /* 等第一帧(树莓派启动延迟): 原地不动 */
                Set_Vel(0, 0, 0);
            } else {
                float vx = LINE_KP * ((float)vision_data.turn_y - LINE_DIST_TARGET);
                float w  = Pos_Yaw(0.0, vision_data.turn_x * 0.099f, -0.0);
                Set_Vel(vx, -100, w);
                if (vision_data.turn_x >= -2 && vision_data.turn_x <= 2) {
                    Odometry_ResetYaw0();   /* 视觉角度≈0 -> 重设航向零点 */
                    Pos_Yaw_Reset();
                }
            }
        }
		//Set_Vel(100,Pos_Y(650,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,y不变移动x
        if (vision_data.turn_flag && odometry.y < sm_ey + R_MOVE7_D) { sm_t = HAL_GetTick(); sm_phase = SM_HOLD7; }//
        break;

    case SM_HOLD7:
        Set_Vel(0, 0, Pos_Yaw(0, odometry.theta, 0)); //停住不回拉(保留视觉矫正后的位置), 只锁航向
        if (hold_action_state == HOLD_ACTION_IDLE) { hold_action_id = 3; hold_action_state = HOLD_ACTION_RUN; }  /* 请求主循环做 part3 追靶+激光+舵机 */	//
        if (hold_action_state == HOLD_ACTION_DONE) { hold_action_state = HOLD_ACTION_IDLE; hold_action_id = 0; sm_phase = SM_MOVE8; }//
        break;

    case SM_MOVE8:      /* 右移 R_MOVE8_D（只锁航向） */
        Set_Vel(0,-100,Pos_Yaw(0.1,odometry.theta,0)); //角度,x不变移动y
		//Set_Vel(100,Pos_Y(650,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,y不变移动x
        if (odometry.y < sm_ey + R_MOVE8_D) { sm_t = HAL_GetTick(); sm_phase = SM_HOLD8; }
        break;
     case SM_HOLD8:
//		Set_Vel(0, 0, 0);
		{
			float vx = Pos_X(sm_ex + R_HOLD8_CX, odometry.x);
			float vy, w;
			if (hold8_phase == 0) {                       /* 到位: 拉回位姿 */
				vy = Pos_Y(sm_ey + R_HOLD8_CY, odometry.y);
				w  = Pos_Yaw(0, odometry.theta, 0);
			} else if (hold8_phase == 1) {                /* Y 补偿(85) */
				vy = LINE_KP_Y * ((float)vision_data.turn_y - LINE_DIST_TARGET_Y);
				w  = Pos_Yaw(0, odometry.theta, 0);
			} else if (hold8_phase == 2) {                /* 角度矫正(目标3.6° = turn_x 36) */
				vy = 0.0f;
				w  = Pos_Yaw(LINE_ANGLE_TARGET * 0.1f, vision_data.turn_x * 0.1f, -0.0);
			}
			else if (hold8_phase == 3)
			{vx = 0.0f;
			 vy = 0.0f;
			 w  =0 ;
			}
			else {                                      /* phase 3: 直接走, 不拉回 */
				vx = 0.0f;
				vy = 0.0f;
				w  = Pos_Yaw(0, odometry.theta, 0);
			}
			Set_Vel(vx, vy, w);
		}
        if (hold_action_state == HOLD_ACTION_IDLE &&
            fabsf(odometry.x - (sm_ex + R_HOLD8_CX)) < R_HOLD8_POS_TOL &&
            fabsf(odometry.y - (sm_ey + R_HOLD8_CY)) < R_HOLD8_POS_TOL) {
            hold_action_id = 5; hold_action_state = HOLD_ACTION_RUN;
        }
        if (hold_action_state == HOLD_ACTION_DONE) { hold_action_state = HOLD_ACTION_IDLE; hold_action_id = 0; sm_phase = SM_MOVE9; }//
        break;

    case SM_MOVE9:      /* 后退 R_MOVE9_D（只锁航向） */
        //Set_Vel(Pos_X(2500.0,odometry.x),-100,Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
		Set_Vel(-100,0,Pos_Yaw(0,odometry.theta,0.0)); //角度,y不变移动x
        if (odometry.x < sm_ex + R_MOVE9_D) { sm_t = HAL_GetTick(); sm_phase = SM_HOLD9; sm_x0=odometry.x; }//
        break;
    case SM_HOLD9:
        {
            float vx, vy;
            if (vision_car_vx_track_enable) vx = vision_car_vx;                 /* x对准: 前后由视觉接管 */
            else                            vx = 0;//Pos_X(1649.0f+Slip_Offset, odometry.x);
            if (vision_car_track_enable)    vy = vision_car_vy;                 /* y补偿: 左右由视觉接管 */
            else                            vy = 0;//Pos_Y(-1515, odometry.y);
            Set_Vel(vx, vy, Pos_Yaw(0, odometry.theta, 0));
        }
        if (hold_action_state == HOLD_ACTION_IDLE) { hold_action_id = 4; hold_action_state = HOLD_ACTION_RUN; }  /* 请求主循环做 part4 抓人质 */
        if (hold_action_state == HOLD_ACTION_DONE) { hold_action_state = HOLD_ACTION_IDLE; hold_action_id = 0; sm_x0 = odometry.x; sm_phase = SM_MOVE9_ADJUST; }
        break;

    case SM_MOVE9_ADJUST:      /* 短距离后退一小段(x 负向), 相对距离 */
        Set_Vel(-100, 0, 0);
        // Set_Vel(-100, 0, Pos_Yaw(0, odometry.theta, 0.0));
//        if (odometry.x <600) {sm_phase = SM_HOLD9_POSE;}
        if (odometry.x < sm_x0 - 310.0f) sm_phase = SM_HOLD9_POSE;   /* 100mm 是占位符, 你调 */
        break;

    case SM_HOLD9_POSE:        /* 车停住, 摆 HOLD8 看直线位姿 + 发 B6 06 (part6) */
        Set_Vel(0, 0, Pos_Yaw(0, odometry.theta, 0));
        if (hold_action_state == HOLD_ACTION_IDLE) { hold_action_id = 6; hold_action_state = HOLD_ACTION_RUN; }//
        if (hold_action_state == HOLD_ACTION_DONE) { hold_action_state = HOLD_ACTION_IDLE; hold_action_id = 0; sm_phase = SM_MOVE10; }//
        break;

    case SM_MOVE10:     /* 后退 R_MOVE10_D 到终点, y 由视觉距离补偿(目标85) */
     {
            if (!vision_data.turn_flag) {          /* 等第一帧(树莓派启动延迟): 原地不动 */
                Set_Vel(0, 0, 0);
            } else {
                float vy = LINE_KP_Y * ((float)vision_data.turn_y - 85.0f);
                float w  = Pos_Yaw(LINE_ANGLE_TARGET * 0.1f, vision_data.turn_x * 0.1f, -0.0);
                Set_Vel(-100, vy, w);
            }
    }

//         {
//             float vy = vision_data.turn_flag
//                      ? (LINE_KP_Y * ((float)vision_data.turn_y - 85.0f))
//                      : 0.0f;
// 			Set_Vel(-100, vy, 0);
// //			Set_Vel(-100, vy, Pos_Yaw(0, odometry.theta, -0.0));
// //          Set_Vel(-100, vy, Pos_Yaw(LINE_ANGLE_TARGET * 0.1f, vision_data.turn_x * 0.1f, -0.0));
//         }
        if (vision_data.turn_flag && odometry.x < sm_ex + R_MOVE10_D) sm_phase = SM_DONE;
        break;

    case SM_DONE:
        Set_Vel(0, 0, 0);               /* 停住，不再动 */
        break;
    }
}

/* OLED 显示用：返回状态机当前状态名（数组下标与 SM_State 枚举值一一对应） */
const char * Control_GetStateName(void)
{
    static const char *names[] = {
        "IDLE","MOVE1","HOLD1","MOVE2","M2ADJ","HOLD2",
        "MOVE3","HOLD3","MOVE4","HOLD4","MOVE5","HOLD5",
        "MOVE6","HOLD6","MOVE7","HOLD7","MOVE8","HOLD8",
        "MOVE9","HOLD9","M9ADJ","H9POS","MOVE10","DONE"
    };
    return names[sm_phase];
}

/* ================= 识别直线小车补偿测试(KEY_4) =================
 * 中断侧: 读 C7 turn_y, 在 X 轴做比例补偿, 把到直线距离保持在 LINE_DIST_TARGET。
 * 只在 KeyNum==0(状态机没跑) 且 test_line_run==1 时由中断调用。 */

void Line_Compensate_Update(void)
{
    if (!line_test_ready) {                        /* 还没发 B6: 不动 */
        Set_Vel(0, 0, 0);
        return;
    }

    if (test_line_run == 1) {                      /* X 补偿(放桶->激光, 目标 61) */
        if (!vision_data.turn_flag) { Set_Vel(0, 0, 0); return; }
        float vx = LINE_KP * ((float)vision_data.turn_y - LINE_DIST_TARGET);
        Set_Vel(vx, 0.0f, Pos_Yaw(0, odometry.theta, 0));
        return;
    }

    if (test_line_run == 2) {                      /* Y -> 角度 -> 清零 序列 */
        if (line_test_phase == 1) {                /* Y 补偿(目标 85) */
            float vy = LINE_KP_Y * ((float)vision_data.turn_y - LINE_DIST_TARGET_Y);
            Set_Vel(0.0f, vy, Pos_Yaw(0, odometry.theta, 0));
        } else if (line_test_phase == 2) {         /* 角度稳定(目标 20=2°) */
            float ang = (float)(vision_data.turn_x - LINE_ANGLE_TARGET);   /* ×10 度 */
            float yaw_target = odometry.theta - LINE_ANGLE_KP * ang * 0.1f;
            Set_Vel(0.0f, 0.0f, Pos_Yaw(yaw_target, odometry.theta, 0));
        } else {                                   /* phase 0/3: 锁航向 */
            Set_Vel(0.0f, 0.0f, Pos_Yaw(0, odometry.theta, 0));
        }
        return;
    }

    Set_Vel(0, 0, 0);
}

/* ================= 5ms 闭环 ================= */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM6)
    {
        Key_Tick();             /* 5ms：按键状态机（Key.c，10ms 采样一次） */
        Encoder_Update();       /* 5ms：读取四路编码器增量（脉冲/5ms） */
        Odometry_Update();      /* 5ms：正解 + 积分 -> x/y/θ */

        /* 开机闸门：上电 1.5s（300 拍 × 5ms）后才允许按 KEY_1 起步。
         * 到点后 Timer 不再累加，避免 uint16 白绕。 */
        if (!flag_Numdelay)
        {
            Timer++;
            if (Timer > 300U) flag_Numdelay = 1;
        }

        /* KeyNum==1 才跑状态机；否则每拍显式写 0。
         * 光是不调 StateMachine_Update() 停不住车 —— Set_Vel 只是把指令存进
         * kinematics.exp_vel，Exp_Speed_Cal() 每拍都会拿上一拍的值解算。
         * 这段必须在 Exp_Speed_Cal() 之前，写的 Set_Vel 才当拍生效。 */
        if (flag_Numdelay && KeyNum == 1)
        {
			
           //单路线调试
           //line_test();
           //(加机械臂全层调试)
			StateMachine_Update();

//			Set_Vel(Pos_X(0,odometry.x),-100,Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
		// Set_Vel(-100,Pos_Y(0,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,y不变移动x
//			Set_Vel(Pos_X(0,odometry.x),100,0);     
//			Set_Vel(100,Pos_Y(0,odometry.y),0);
//			Set_Vel(0, 0, Pos_Yaw(0,odometry.theta,0));//角度不变移动x，y
//			Set_Vel(0, 0, 0.6);

        }
        else if (test_line_run)
        {
            Line_Compensate_Update();   /* 识别直线小车补偿测试(KEY_4) */
        }
        else
        {
            Set_Vel(0, 0, 0);
        }

        Exp_Speed_Cal();        /* 解算 -> exp_wheel_rpm (RPM) */

        int p1 = 0, p2 = 0, p3 = 0, p4 = 0;
		
        if(!No_rear_wheels)
        { 
        p1 = Velocity_Wheel1(Kinematics_RPM_To_Pulse(kinematics.exp_wheel_rpm.motor_1),
                            (int)Encoder_GetDelta(ENC_WHEEL1));
		p4 = Velocity_Wheel4(Kinematics_RPM_To_Pulse(kinematics.exp_wheel_rpm.motor_4),
                            (int)Encoder_GetDelta(ENC_WHEEL4));
        }
        if(!No_front_wheels)
        { 
        p2 = Velocity_Wheel2(Kinematics_RPM_To_Pulse(kinematics.exp_wheel_rpm.motor_2),
                            (int)Encoder_GetDelta(ENC_WHEEL2));
        p3 = Velocity_Wheel3(Kinematics_RPM_To_Pulse(kinematics.exp_wheel_rpm.motor_3),
                            (int)Encoder_GetDelta(ENC_WHEEL3));
        }
        /* 反馈轮速：只写 kinematics.fb_wheel_rpm 这个普通结构体，不驱动任何电机。
         * 注意下面那条 PID 通路并不用它 —— Velocity_WheelN 的 Target 和 encoder
         * 都是脉冲/5ms，两边单位一致，直接对着原始增量比。这里纯粹给遥测/观察用。
         * 四路同向为正（分配表.md「编码器极性备注」），故无需按轮符号。 */
        // kinematics.fb_wheel_rpm.motor_1 = Kinematics_Pulse_To_RPM((float)Encoder_GetDelta(ENC_WHEEL1));
        // kinematics.fb_wheel_rpm.motor_2 = Kinematics_Pulse_To_RPM((float)Encoder_GetDelta(ENC_WHEEL2));
        // kinematics.fb_wheel_rpm.motor_3 = Kinematics_Pulse_To_RPM((float)Encoder_GetDelta(ENC_WHEEL3));
        // kinematics.fb_wheel_rpm.motor_4 = Kinematics_Pulse_To_RPM((float)Encoder_GetDelta(ENC_WHEEL4));

        /* 四路速度环闭环。目标来自 kinematics.exp_wheel_rpm —— 即状态机 Set_Vel 进去、
         * Exp_Speed_Cal 解算出来的 RPM，经 Kinematics_RPM_To_Pulse 换成脉冲/5ms。
         * 增益是 mailuncontrol.h 里的 Velocity_Kp1..4 / Ki1..4。
         * 下面注释掉的是早先的蓝牙开环调试通路（目标取自 Debug_Target[]）。想切回去就
         * 整块一起放开：声明也在块里，只放开 Motor_Load 那一行的话 p1..p4 就是未初始化
         * 变量，中断会拿栈上残值直接驱动电机。 */
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

//实际路径（10 段，每段之间原地等 3 秒；沿程轴 vx/vy 单位 mm/s）：
//  MOVE1  前进 vx=+100，y 环压住 0      -> x>650   停
//  MOVE2  左移 vy=+100，x 环压住 650    -> y>650   停
//  MOVE3  前进 vx=+100，y 环压住 650    -> x>1630  停
//  MOVE4  左移 vy=+100，x 环压住 1630   -> y>1500  停
//  MOVE5  前进 vx=+100，y 环压住 1500   -> x>2600  停
//  MOVE6  右移 vy=-100，x 环压住 2600   -> y<850   停
//  MOVE7  右移 vy=-100，x 环压住 2600   -> y<-840  停
//  MOVE8  右移 vy=-100，x 环压住 2600   -> y<-1450 停
//  MOVE9  后退 vx=-100，y 环压住 -1450  -> x<1520  停
//  MOVE10 后退 vx=-100，y 环压住 -1450  -> x<-50   停
//终止：SM_DONE，Set_Vel(0,0,0)。SM_DONE 没有出口，重跑要复位。

//line_test() 是单路线调试，StateMachine_Update() 是全层调试（含机械臂抓球放桶）。
//line_test_debug() 是单路线调试debug 

//把状态机状态 + 收发状态 + 里程计坐标 + 偏航角刷新到LOG
/* 本工程不链接浮点 printf(见文件头注释), 浮点一律定点化后按 %d 打:
 * X/Y 单位 0.1mm, θ/Yaw 单位 0.01°。 */
void line_test_debug(void)
{
    log_i("ST:%s TX:%02X RX:%d X:%d Y:%d T:%d Yaw:%d",
          Control_GetStateName(),           /* 状态机状态名, 如 "MOVE2" */
          (unsigned)vision_data.last_tx_cmd,/* 最近发出的命令字节 */
          (int)vision_data.rx_status,       /* 0=未收到A5 1=收到 */
          (int)(odometry.x * 10.0f),        /* 里程计 X, 0.1mm */
          (int)(odometry.y * 10.0f),        /* 里程计 Y, 0.1mm */
          (int)(odometry.theta * 100.0f),   /* 里程计航向 θ, 0.01° */
          (int)(Yaw * 100.0f));             /* 陀螺仪偏航角, 0.01° */
}