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

/* 视觉误差x -> 小车左右速度指令(part2 夹小球): Hold_Action_Update 写入, control.c SM_HOLD6 读取 */
extern volatile float   vision_car_vy;           /* mm/s, +左 -右 */
extern volatile uint8_t vision_car_track_enable; /* 1=视觉追踪中, 小车vy由视觉接管 */

void Hold_Action_Update(void);

/* 激光打靶测试(KEY_4 触发, 与运动系统无关) */
extern volatile uint8_t test_laser_run;
void Laser_Track_Test(void);

/* 主循环里调；把四个舵机实际位置打一行到 LOG（蓝牙口 USART1） */
void FT_test_debug(void);

#endif
