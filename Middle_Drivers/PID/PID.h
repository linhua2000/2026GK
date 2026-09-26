#ifndef __PID_H
#define __PID_H

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
void Servo_PID_Update(void); /* 每来一帧视觉算一次, 累加位置写舵机 */

#endif
