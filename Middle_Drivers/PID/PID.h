#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct {
    float Kp, Ki, Kd;
    float error;          /* 当前误差 */
    float error_last;     /* 上一次误差 */
    float intergral;      /* 积分累加(沿用旧工程拼写) */
    float output;         /* PID 输出 */
    float intergral_max, intergral_min;  /* 积分限幅 */
    float output_min, output_max;        /* 输出限幅 */
    float deadzone_min;   /* 死区: |error|<=该值不积分 */
} PID_Controller_t;

void Control_PID_Init(PID_Controller_t *pid, float kp, float ki, float kd,
                      float min_output, float max_output);
float PID_Compute(PID_Controller_t *pid, float y_error);

/* XY 舵机 PID 控制 */
void Servo_PID_Init(void);   /* 初始化 X/Y PID + 舵机回初始位 */

/* HOLD 段动作(视觉握手 + 舵机序列), 主循环每圈调用 */
#define HOLD_ACTION_IDLE  0
#define HOLD_ACTION_RUN   1
#define HOLD_ACTION_DONE  2
extern volatile uint8_t hold_action_state;  /* 状态机置 RUN, 动作完成后置 DONE */
extern volatile uint8_t hold_action_id;     /* 0=无 1=part1(HOLD1) 2=part2(HOLD6) */
extern volatile uint8_t sim_ball_stable;    /* KEY_3 第一次: 球稳定 */
extern volatile uint8_t sim_bucket_stable;  /* KEY_3 第二次: 桶稳定 */
void Hold_Action_Update(void);

/* 激光打靶测试(KEY_4 触发, 与运动系统无关) */
extern volatile uint8_t test_laser_run;
void Laser_Track_Test(void);

#endif
