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

#define SERVO_SPEED  25      /* 沿用现有速度, 可调 */
#define SERVO_ACC    0

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

void Servo_PID_Init(void)
{
    /* 使能所有舵机扭矩(扭矩被关的话, 舵机收了位置指令也不会动) */
    EnableTorque(1, 1);
    EnableTorque(2, 1);
    EnableTorque(3, 1);

    /* Kp/Ki/Kd 现场调; 输出=每帧位置增量, 限幅 ±50 */
    Control_PID_Init(&pid_control_x, -0.3f, 0.0f, 0.0f, -5000.0f, 5000.0f);
    Control_PID_Init(&pid_control_y, -0.1f, 0.0f, 0.0f, -5000.0f, 5000.0f);

    servo_pos_x = SERVO_X_INIT;
    servo_pos_y = SERVO_Y_INIT;

    WritePosEx(SERVO_X_ID, (int16_t)servo_pos_x, SERVO_SPEED, SERVO_ACC);
    WritePosEx(SERVO_Y_ID, (int16_t)servo_pos_y, SERVO_SPEED, SERVO_ACC);
}

void Servo_PID_Update(void)
{
    if (!vision_data.grab_flag) return;
    vision_data.grab_flag = 0;

    /* X轴: grab_x -> 舵机3 */
    PID_Compute(&pid_control_x, (float)vision_data.grab_x);
    servo_pos_x += pid_control_x.output;  
    if (servo_pos_x < SERVO_X_MIN) servo_pos_x = SERVO_X_MIN;
    if (servo_pos_x > SERVO_X_MAX) servo_pos_x = SERVO_X_MAX;
    WritePosEx(SERVO_X_ID, (int16_t)servo_pos_x, SERVO_SPEED, SERVO_ACC);

    /* Y轴: grab_y -> 舵机1 */
    PID_Compute(&pid_control_y, (float)vision_data.grab_y);
    servo_pos_y += pid_control_y.output;   
    if (servo_pos_y < SERVO_Y_MIN) servo_pos_y = SERVO_Y_MIN;
    if (servo_pos_y > SERVO_Y_MAX) servo_pos_y = SERVO_Y_MAX;
    WritePosEx(SERVO_Y_ID, (int16_t)servo_pos_y, SERVO_SPEED, SERVO_ACC);
}
