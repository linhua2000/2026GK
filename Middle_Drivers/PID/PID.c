#include "PID.h"
#include "SCServo.h"
#include "receive.h"
#include <math.h>

/* ============ 舵机 ID 与阈值 ============ */
#define SERVO_X_ID   3      /* X轴(水平/左右) */
#define SERVO_Y_ID   1      /* Y轴(上下/俯仰) */

#define SERVO_Y_MIN  1180   /* Y轴最上 */
#define SERVO_Y_MAX  2680   /* Y轴最下 */

/* X轴暂时没有阈值, 先用飞特全范围兜底; 现场测出范围后回填 */
#define SERVO_X_MIN  0
#define SERVO_X_MAX  4095

#define SERVO_X_INIT 700    /* X轴开机位置(现测值, 可调) */
#define SERVO_Y_INIT 1950   /* Y轴中心 (1180+2680)/2, 可调 */

#define SERVO_SPEED_X  10      /* 沿用现有速度, 可调 */
#define SERVO_SPEED_Y  25
#define SERVO_ACC    0

/* ============ 舵机4: 目标稳定后 2332 -> 1641 ============ */
#define SERVO4_ID     4
#define SERVO4_INIT   2332   /* 初始位置 */
#define SERVO4_TARGET 1641   /* 目标稳定后切换到的位置 */
#define SERVO4_SPEED  50     /* 可调 */

/* 目标(grab_x/grab_y 误差)稳定在中心范围内的阈值, 现场调 */
#define GRAB_STABLE_X   20    /* X误差阈值(像素), 可调 */
#define GRAB_STABLE_Y   20    /* Y误差阈值(像素), 可调 */
#define GRAB_STABLE_CNT 5     /* 连续稳定多少帧触发 */

/* 转身180°的绝对位置: 待测(暂按初始 700 + 半圈2048 = 2748) */
#define SERVO_X_TURN_POS   2748
#define SERVO_X_TURN_SPEED 10    /* 转身速度, 可调 */
#define TURN_WAIT_MS       1500   /* 等转身到位时间, 可调; 也可改用 ReadPos(3) 判断 */
#define GRAB_WAIT_MS       500    /* 抓球后等夹爪闭合的时间, 可调 */

/* ============ 通用 PID(模仿旧工程) ============ */
void Control_PID_Init(PID_Controller_t *pid, float kp, float ki, float kd,
                      float min_output, float max_output)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->error = 0.0f;
    pid->error_last = 0.0f;
    pid->intergral = 0.0f;
    pid->output = 0.0f;
    pid->output_min = min_output;
    pid->output_max = max_output;
    pid->intergral_max = max_output * 0.2f;  /* 积分只贡献输出的20% */
    pid->intergral_min = min_output * 0.2f;
    pid->deadzone_min = 5.0f;
}

float PID_Compute(PID_Controller_t *pid, float y_error)
{
    pid->error = y_error;

    if (fabsf(pid->error) > pid->deadzone_min) {
        pid->intergral += pid->error;
        if (pid->intergral > pid->intergral_max) pid->intergral = pid->intergral_max;
        else if (pid->intergral < pid->intergral_min) pid->intergral = pid->intergral_min;
    } else {
        pid->intergral *= 0.95f;   /* 死区内积分缓慢衰减, 防饱和 */
    }

    float differential = pid->error - pid->error_last;

    pid->output = pid->Kp * pid->error
                + pid->Ki * pid->intergral
                + pid->Kd * differential;

    if (pid->output > pid->output_max) pid->output = pid->output_max;
    else if (pid->output < pid->output_min) pid->output = pid->output_min;

    pid->error_last = pid->error;
    return pid->output;
}

/* ============ XY 舵机控制(模仿 PID_Control_Outer_Only) ============ */
static PID_Controller_t pid_control_x;
static PID_Controller_t pid_control_y;

static float servo_pos_x = SERVO_X_INIT;   /* 累加的舵机绝对位置 */
static float servo_pos_y = SERVO_Y_INIT;

static uint8_t stable_cnt = 0;    /* 连续稳定帧计数 */

static uint8_t  task_step = 0;        /* 0追球 1抓球等待 2转身 3追桶 4复位 5结束 */
static uint32_t wait_start_tick = 0;  /* 等待计时起点 */

void Servo_PID_Init(void)
{
    /* 使能所有舵机扭矩(扭矩被关的话, 舵机收了位置指令也不会动) */
    EnableTorque(1, 1);
    EnableTorque(2, 1);
    EnableTorque(3, 1);
    EnableTorque(4, 1);

    /* Kp/Ki/Kd 现场调; 输出=每帧位置增量, 限幅 ±50 */
    Control_PID_Init(&pid_control_x, -0.3f, 0.0f, 0.0f, -5000.0f, 5000.0f);
    Control_PID_Init(&pid_control_y, -0.1f, 0.0f, 0.0f, -5000.0f, 5000.0f);

    servo_pos_x = SERVO_X_INIT;
    servo_pos_y = SERVO_Y_INIT;

    WritePosEx(SERVO_X_ID, (int16_t)servo_pos_x, SERVO_SPEED_X, SERVO_ACC);
    WritePosEx(SERVO_Y_ID, (int16_t)servo_pos_y, SERVO_SPEED_Y, SERVO_ACC);

    /* 舵机4 回初始位 2332 */
    WritePosEx(SERVO4_ID, (int16_t)SERVO4_INIT, SERVO4_SPEED, SERVO_ACC);
    stable_cnt = 0;
    task_step = 0;
}

/* 追球/追桶共用: 跑一帧 X/Y PID, 返回是否已稳定居中(连续 GRAB_STABLE_CNT 帧) */
static int track_and_check_stable(void)
{
    /* X轴: grab_x -> 舵机3 */
    PID_Compute(&pid_control_x, (float)vision_data.grab_x);
    servo_pos_x += pid_control_x.output;
    if (servo_pos_x < SERVO_X_MIN) servo_pos_x = SERVO_X_MIN;
    if (servo_pos_x > SERVO_X_MAX) servo_pos_x = SERVO_X_MAX;
    WritePosEx(SERVO_X_ID, (int16_t)servo_pos_x, SERVO_SPEED_X, SERVO_ACC);

    /* Y轴: grab_y -> 舵机1 */
    PID_Compute(&pid_control_y, (float)vision_data.grab_y);
    servo_pos_y += pid_control_y.output;
    if (servo_pos_y < SERVO_Y_MIN) servo_pos_y = SERVO_Y_MIN;
    if (servo_pos_y > SERVO_Y_MAX) servo_pos_y = SERVO_Y_MAX;
    WritePosEx(SERVO_Y_ID, (int16_t)servo_pos_y, SERVO_SPEED_Y, SERVO_ACC);

    /* 目标稳定在中心范围内连续 N 帧 */
    if (vision_data.grab_x >= -GRAB_STABLE_X && vision_data.grab_x <= GRAB_STABLE_X &&
        vision_data.grab_y >= -GRAB_STABLE_Y && vision_data.grab_y <= GRAB_STABLE_Y) {
        stable_cnt++;
        if (stable_cnt >= GRAB_STABLE_CNT) {
            return 1;
        }
    } else {
        stable_cnt = 0;
    }
    return 0;
}

void Servo_PID_Update(void)
{
    /* task_step: 0追球 1抓球等待 2转身 3追桶 4复位 5结束 */
    if (task_step == 0) {                    /* 追球 */
        if (!vision_data.grab_flag) return;
        vision_data.grab_flag = 0;
        if (track_and_check_stable()) {
            /* 抓球: 舵机4 -> 1641, 然后等夹爪闭合 */
            WritePosEx(SERVO4_ID, (int16_t)SERVO4_TARGET, SERVO4_SPEED, SERVO_ACC);
            wait_start_tick = HAL_GetTick();
            stable_cnt = 0;
            task_step = 1;
        }
    }
    else if (task_step == 1) {               /* 抓球等待 GRAB_WAIT_MS */
        vision_data.grab_flag = 0;
        if (HAL_GetTick() - wait_start_tick >= GRAB_WAIT_MS) {
            /* 转身180°: 舵机X 到绝对位置, 同步累加器避免追桶时跳回 */
            WritePosEx(SERVO_X_ID, (int16_t)SERVO_X_TURN_POS, SERVO_X_TURN_SPEED, SERVO_ACC);
            servo_pos_x = SERVO_X_TURN_POS;
            wait_start_tick = HAL_GetTick();
            task_step = 2;
        }
    }
    else if (task_step == 2) {               /* 转身, 等到位 */
        vision_data.grab_flag = 0;           /* 排空转身期间的球帧 */
        if (HAL_GetTick() - wait_start_tick >= TURN_WAIT_MS) {
            Vision_Send_Switch_Bucket();     /* D8 01 8D, 视觉切桶识别 */
            /* 复位 PID 历史, 避免追桶时微分/积分残留(现 Ki/Kd=0, 保险起见) */
            pid_control_x.error_last = 0.0f;
            pid_control_x.intergral = 0.0f;
            pid_control_y.error_last = 0.0f;
            pid_control_y.intergral = 0.0f;
            stable_cnt = 0;
            task_step = 3;
        }
    }
    else if (task_step == 3) {               /* 追桶 */
        if (!vision_data.grab_flag) return;
        vision_data.grab_flag = 0;
        if (track_and_check_stable()) {
            /* 放球: 舵机4 -> 2332 */
            WritePosEx(SERVO4_ID, (int16_t)SERVO4_INIT, SERVO4_SPEED, SERVO_ACC);
            Vision_Send_Release_Done();      /* D8 02 8D, 通知视觉完成 */
            task_step = 4;
        }
    }
    else if (task_step == 4) {               /* 复位: 所有舵机回初始位 */
        vision_data.grab_flag = 0;
        WritePosEx(SERVO_X_ID, (int16_t)SERVO_X_INIT, SERVO_SPEED_X, SERVO_ACC);
        WritePosEx(SERVO_Y_ID, (int16_t)SERVO_Y_INIT, SERVO_SPEED_Y, SERVO_ACC);
        WritePosEx(SERVO4_ID, (int16_t)SERVO4_INIT, SERVO4_SPEED, SERVO_ACC);
        servo_pos_x = SERVO_X_INIT;
        servo_pos_y = SERVO_Y_INIT;
        stable_cnt = 0;
        task_step = 5;
    }
    /* task_step == 5: 结束, 停住 */
}
