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
 * sm_t 不再初始化：静态 0，而 HAL_GetTick 也是从 HAL_Init 起算的，
 * 于是 SM_IDLE 那三秒就是「上电后约三秒」，跟 main 里那段 HAL_Delay(3000) 的开机灯重叠。
 */
typedef enum {
    SM_IDLE = 0,                        /* 上电原地等三秒（同时等陀螺仪出帧） */
    SM_MOVE1, SM_HOLD1,
    SM_MOVE2, SM_MOVE2_ADJUST, SM_HOLD2,
    SM_MOVE3, SM_HOLD3,
    SM_MOVE4, SM_HOLD4,
    SM_MOVE5, SM_HOLD5,
    SM_MOVE6, SM_HOLD6,
    SM_MOVE7, SM_HOLD7,
    SM_MOVE8, SM_HOLD8,
    SM_MOVE9, SM_HOLD9,
    SM_MOVE10,                          /* 第 10 段后面没有 HOLD，直接进 DONE */
    SM_DONE
} SM_State;

#define SM_HOLD_MS   3000U

/* MOVE3/HOLD3 的 x 落点修正量(mm)。打滑让 odometry.x 有系统偏差时,
 * 用 1630 + Slip_Offset 把目标挪一点。0.0f = 不修正(行为不变)。line_test() 与 StateMachine_Update() 都用。 */
#define Slip_Offset  (89.0f)

static SM_State  sm_phase = SM_IDLE;
static uint32_t  sm_t;                  /* 进入 IDLE / HOLD 的时刻 */

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
		
		Set_Vel(-100,0,Pos_Yaw(1.675,odometry.theta,0.0)); //角度,y不变移动x
		//Set_Vel(Pos_X(2500.0,odometry.x),-100,Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
//		Set_Vel(-100,Pos_Y(-1508,odometry.y),Pos_Yaw(4.8,odometry.theta,0.0)); //角度,y不变移动x
        if (odometry.x < 1650.0f+Slip_Offset) { sm_t = HAL_GetTick(); sm_phase = SM_HOLD9; }
        break;
    case SM_HOLD9:
//		Set_Vel(0, 0, 0);
		Set_Vel(Pos_X(1649.0f+Slip_Offset,odometry.x),Pos_Y( -1514,odometry.y),Pos_Yaw(2.25,odometry.theta,0)); //角度,x不变移动y
        if ((HAL_GetTick() - sm_t) >= SM_HOLD_MS) {sm_phase = SM_MOVE10;}//
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
        if (hold_action_state == HOLD_ACTION_IDLE) { hold_action_id = 1; hold_action_state = HOLD_ACTION_RUN; }  /* 请求主循环做 part1 握手+舵机复位 */
        if (hold_action_state == HOLD_ACTION_DONE) { hold_action_state = HOLD_ACTION_IDLE; hold_action_id = 0; sm_phase = SM_MOVE2; }
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

    case SM_MOVE3:      /* 前进到 x>1630（y 按住 650） */
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
		//Set_Vel(100,Pos_Y(650,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,y不变移动x
        if (odometry.y <860.0f) { sm_t = HAL_GetTick(); sm_phase = SM_HOLD6; }
        break;
    case SM_HOLD6:
        {
            float vx, vy;
            if (vision_car_vx_track_enable) vx = vision_car_vx;                 /* y补偿: 前后由视觉接管 */
            else                            vx = Pos_X(2640.0f+Slip_Offset, odometry.x-7);
            if (vision_car_track_enable)    vy = vision_car_vy;                 /* x对准: 左右由视觉接管 */
            else                            vy = Pos_Y(860,  odometry.y);
            Set_Vel(vx, vy, Pos_Yaw(-0.75, odometry.theta, 0));
        }
        if (hold_action_state == HOLD_ACTION_IDLE) { hold_action_id = 2; hold_action_state = HOLD_ACTION_RUN; }  /* 请求主循环做 part2 追球抓球+追桶放桶 */
        if (hold_action_state == HOLD_ACTION_DONE) {  hold_action_id = 0;hold_action_state = HOLD_ACTION_IDLE;sm_phase = SM_MOVE7; } //
        break;

    case SM_MOVE7:      /* 继续右移到 y<-840（x 按住 2600） */
        Set_Vel(0,-100,Pos_Yaw(0.075,odometry.theta,-0.75)); //角度,x不变移动y
		//Set_Vel(100,Pos_Y(650,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,y不变移动x
        if (odometry.y < -840.0f) { sm_t = HAL_GetTick(); sm_phase = SM_HOLD7; }//
        break;

    case SM_HOLD7:
        Set_Vel(Pos_X(2649.0f+Slip_Offset,odometry.x),Pos_Y(-850,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
        if (hold_action_state == HOLD_ACTION_IDLE) { hold_action_id = 3; hold_action_state = HOLD_ACTION_RUN; }  /* 请求主循环做 part3 追靶+激光+舵机 */
        if (hold_action_state == HOLD_ACTION_DONE) { hold_action_state = HOLD_ACTION_IDLE; hold_action_id = 0; sm_phase = SM_MOVE8; }
        break;

    case SM_MOVE8:      /* 继续右移到 y<-1450（x 按住 2600） */
        Set_Vel(0,-100,Pos_Yaw(0.1,odometry.theta,0)); //角度,x不变移动y
		//Set_Vel(100,Pos_Y(650,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,y不变移动x
        if (odometry.y < -1400.0f) { sm_t = HAL_GetTick(); sm_phase = SM_HOLD8; }
        break;
     case SM_HOLD8:
//		Set_Vel(0, 0, 0);
        Set_Vel(Pos_X(2400.0f+Slip_Offset,odometry.x),Pos_Y( -1505,odometry.y),Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
        if ((HAL_GetTick() - sm_t) >= SM_HOLD_MS) {sm_phase = SM_MOVE9;}//
        break;

    case SM_MOVE9:      /* 后退到 x<1520（y 按住 -1450） */
        //Set_Vel(Pos_X(2500.0,odometry.x),-100,Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
		Set_Vel(-100,0,Pos_Yaw(1.675,odometry.theta,0.0)); //角度,y不变移动x
        if (odometry.x < 1650.0f+Slip_Offset) { sm_t = HAL_GetTick(); sm_phase = SM_HOLD9; }
        break;
    case SM_HOLD9:
        {
            float vx, vy;
            if (vision_car_vx_track_enable) vx = vision_car_vx;                 /* x对准: 前后由视觉接管 */
            else                            vx = Pos_X(1649.0f+Slip_Offset, odometry.x);
            if (vision_car_track_enable)    vy = vision_car_vy;                 /* y补偿: 左右由视觉接管 */
            else                            vy = Pos_Y(-1514, odometry.y);
            Set_Vel(vx, vy, Pos_Yaw(2.25, odometry.theta, 0));
        }
        if (hold_action_state == HOLD_ACTION_IDLE) { hold_action_id = 4; hold_action_state = HOLD_ACTION_RUN; }  /* 请求主循环做 part4 抓人质 */
        if (hold_action_state == HOLD_ACTION_DONE) { hold_action_state = HOLD_ACTION_IDLE; hold_action_id = 0; sm_phase = SM_MOVE10; }
        break;

    case SM_MOVE10:     /* 后退到 x<-50（y 按住 -1450） */
        //Set_Vel(Pos_X(2500.0,odometry.x),-100,Pos_Yaw(0,odometry.theta,0)); //角度,x不变移动y
		Set_Vel(-100,0,Pos_Yaw(1.62,odometry.theta,0.0)); //角度,y不变移动x
        if (odometry.x < 78.0f+Slip_Offset) sm_phase = SM_DONE;
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
        "MOVE9","HOLD9","MOVE10","DONE"
    };
    return names[sm_phase];
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