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

/* ============ 舵机2: 肘/伸缩轴 ============ */
#define SERVO2_ID      2
#define SERVO2_HOME    1410   /* 回缩/初始位 */
#define SERVO2_SPEED   30     /* 速度, 现场调 */

/* 肘位: 往前(伸出)=500, 正常位=1410(SERVO2_HOME), 往后(缩)=3566 */
#define SERVO_ELBOW_MIN  500   /* 往前/伸出 */
#define SERVO_ELBOW_MAX  3566  /* 往后硬限位(抓取不用) */

/* 各阶段等待时间/位移量(现场调) */
#define Y_UP_STEP             500   /* Y轴往上抬的量(往上=减) */
#define Y_DOWN_STEP           200   /* Y轴往下扫木桶: 每步加的量(往下=加) */
#define ELBOW_FORWARD_WAIT_MS 500   /* 舵机2往前到位时间 */
#define Y_UP_WAIT_MS          300   /* Y抬500到位时间 */
#define ELBOW_RETRACT_WAIT_MS 500   /* 舵机2回1410到位时间 */
#define Y_DOWN_WAIT_MS        300   /* Y往下到位时间 */

/* 目标(grab_x/grab_y 误差)稳定在中心范围内的阈值, 现场调 */
#define GRAB_STABLE_X   20    /* X误差阈值(像素), 可调 */
#define GRAB_STABLE_Y   20    /* Y误差阈值(像素), 可调 */
#define GRAB_STABLE_CNT 5     /* 连续稳定多少帧触发 */

/* 转身180°的绝对位置: 待测(暂按初始 700 + 半圈2048 = 2748) */
#define SERVO_X_TURN_POS   2748
#define SERVO_X_TURN_SPEED 10    /* 转身速度, 可调 */
#define TURN_WAIT_MS       1500   /* 等转身到位时间, 可调; 也可改用 ReadPos(3) 判断 */
#define GRAB_WAIT_MS       500    /* 抓球后等夹爪闭合的时间, 可调 */

/* ============ 距离 -> 肘位 标定表 ============
 * 现场标定: 球摆几个已知距离, 先套中 X/Y, 再手动调舵机2到夹爪触球,
 * 记下 (grab_dist, 舵机2计数值) 按 d 升序填进下表。d 单调即可。
 * 方向: 球越远越要往前伸, 往前=计数值减小, 所以 pos 随 d 增大而减小。
 * 未标定前先放一组占位 (0~500cm 线性到 1410~500), 现场替换。 */
static const struct { int16_t d; int16_t pos; } ELBOW_TABLE[] = {
    { 0,   SERVO2_HOME    },
    { 500, SERVO_ELBOW_MIN },
};
#define ELBOW_TABLE_N (sizeof(ELBOW_TABLE) / sizeof(ELBOW_TABLE[0]))

/* 线性插值查表: grab_dist -> 舵机2计数值 */
static int16_t elbow_map(int16_t d)
{
    uint8_t i;
    if (d <= ELBOW_TABLE[0].d) return ELBOW_TABLE[0].pos;
    for (i = 1; i < ELBOW_TABLE_N; i++) {
        if (d <= ELBOW_TABLE[i].d) {
            int16_t d0 = ELBOW_TABLE[i-1].d;
            int16_t d1 = ELBOW_TABLE[i].d;
            int16_t p0 = ELBOW_TABLE[i-1].pos;
            int16_t p1 = ELBOW_TABLE[i].pos;
            return (int16_t)(p0 + (int32_t)(p1 - p0) * (d - d0) / (d1 - d0));
        }
    }
    return ELBOW_TABLE[ELBOW_TABLE_N - 1].pos;
}

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
static uint8_t elbow_cmd_done = 0; /* 追桶阶段是否已发过肘指令 */

static uint8_t  task_step = 0;        /* 0追球 1肘往前 2Y抬500 3抓取 4肘回1410 5转身 6Y往下 7追桶 8复位 */
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
    /* 舵机2(肘) 回缩到 1410 */
    WritePosEx(SERVO2_ID, (int16_t)SERVO2_HOME, SERVO2_SPEED, SERVO_ACC);
    stable_cnt = 0;
    elbow_cmd_done = 0;
    task_step = 0;
}

/* 跑一帧 X/Y PID 并写舵机(不判稳定), 追球/追桶/肘往前共用 */
static void track_xy(void)
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
}

/* 追球/追桶: track_xy + 判稳定, 返回是否已稳定居中(连续 GRAB_STABLE_CNT 帧) */
static int track_and_check_stable(void)
{
    track_xy();

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
    /* task_step: 0追球 1肘往前 2Y抬500 3抓取 4肘回1410 5转身 6Y往下 7追桶 8复位 */
    if (task_step == 0) {                    /* 追球 */
        if (!vision_data.grab_flag) return;
        vision_data.grab_flag = 0;
        if (track_and_check_stable()) {
            /* 稳定: 舵机2(肘)按距离往前伸 */
            WritePosEx(SERVO2_ID, (int16_t)elbow_map(vision_data.grab_dist), SERVO2_SPEED, SERVO_ACC);
            wait_start_tick = HAL_GetTick();
            stable_cnt = 0;
            task_step = 1;
        }
    }
    else if (task_step == 1) {               /* 肘往前, X/Y 继续跟踪 */
        if (vision_data.grab_flag) {
            vision_data.grab_flag = 0;
            track_xy();
        }
        if (HAL_GetTick() - wait_start_tick >= ELBOW_FORWARD_WAIT_MS) {
            /* Y轴往上抬 Y_UP_STEP */
            servo_pos_y -= Y_UP_STEP;
            if (servo_pos_y < SERVO_Y_MIN) servo_pos_y = SERVO_Y_MIN;
            WritePosEx(SERVO_Y_ID, (int16_t)servo_pos_y, SERVO_SPEED_Y, SERVO_ACC);
            wait_start_tick = HAL_GetTick();
            task_step = 2;
        }
    }
    else if (task_step == 2) {               /* Y抬500, 等到位 */
        vision_data.grab_flag = 0;
        if (HAL_GetTick() - wait_start_tick >= Y_UP_WAIT_MS) {
            /* 抓球: 舵机4 -> 1641, 然后等夹爪闭合 */
            WritePosEx(SERVO4_ID, (int16_t)SERVO4_TARGET, SERVO4_SPEED, SERVO_ACC);
            wait_start_tick = HAL_GetTick();
            task_step = 3;
        }
    }
    else if (task_step == 3) {               /* 抓球等待 GRAB_WAIT_MS */
        vision_data.grab_flag = 0;
        if (HAL_GetTick() - wait_start_tick >= GRAB_WAIT_MS) {
            /* 舵机2(肘)回缩到 1410 */
            WritePosEx(SERVO2_ID, (int16_t)SERVO2_HOME, SERVO2_SPEED, SERVO_ACC);
            wait_start_tick = HAL_GetTick();
            task_step = 4;
        }
    }
    else if (task_step == 4) {               /* 肘回1410, 等到位 */
        vision_data.grab_flag = 0;
        if (HAL_GetTick() - wait_start_tick >= ELBOW_RETRACT_WAIT_MS) {
            /* 转身180°: 舵机X 到绝对位置, 同步累加器避免追桶时跳回 */
            WritePosEx(SERVO_X_ID, (int16_t)SERVO_X_TURN_POS, SERVO_X_TURN_SPEED, SERVO_ACC);
            servo_pos_x = SERVO_X_TURN_POS;
            wait_start_tick = HAL_GetTick();
            task_step = 5;
        }
    }
    else if (task_step == 5) {               /* 转身, 等到位 */
        vision_data.grab_flag = 0;           /* 排空转身期间的球帧 */
        if (HAL_GetTick() - wait_start_tick >= TURN_WAIT_MS) {
            Vision_Send_Switch_Bucket();     /* D8 01 8D, 视觉切桶识别 */
            /* 复位 PID 历史, 避免追桶时微分/积分残留(现 Ki/Kd=0, 保险起见) */
            pid_control_x.error_last = 0.0f;
            pid_control_x.intergral = 0.0f;
            pid_control_y.error_last = 0.0f;
            pid_control_y.intergral = 0.0f;
            stable_cnt = 0;
            wait_start_tick = HAL_GetTick();
            task_step = 6;
        }
    }
    else if (task_step == 6) {               /* Y往下扫, 直到视觉看到桶 */
        if (vision_data.grab_flag) {         /* 桶帧来了, 转追桶(帧留给7处理) */
            task_step = 7;
            return;
        }
        if (HAL_GetTick() - wait_start_tick >= Y_DOWN_WAIT_MS) {
            servo_pos_y += Y_DOWN_STEP;
            if (servo_pos_y > SERVO_Y_MAX) servo_pos_y = SERVO_Y_MAX;
            WritePosEx(SERVO_Y_ID, (int16_t)servo_pos_y, SERVO_SPEED_Y, SERVO_ACC);
            wait_start_tick = HAL_GetTick();
        }
    }
    else if (task_step == 7) {               /* 追桶 */
        if (!vision_data.grab_flag) return;
        vision_data.grab_flag = 0;
        if (!elbow_cmd_done) {               /* 首帧才发肘指令 */
            WritePosEx(SERVO2_ID, (int16_t)elbow_map(vision_data.grab_dist), SERVO2_SPEED, SERVO_ACC);
            wait_start_tick = HAL_GetTick();
            elbow_cmd_done = 1;
        }
        if (track_and_check_stable() &&
            HAL_GetTick() - wait_start_tick >= ELBOW_FORWARD_WAIT_MS) {
            /* 放球: 舵机4 -> 2332 */
            WritePosEx(SERVO4_ID, (int16_t)SERVO4_INIT, SERVO4_SPEED, SERVO_ACC);
            Vision_Send_Release_Done();      /* D8 02 8D, 通知视觉完成 */
            task_step = 8;
        }
    }
    else if (task_step == 8) {               /* 复位: 所有舵机回初始位 */
        vision_data.grab_flag = 0;
        WritePosEx(SERVO_X_ID, (int16_t)SERVO_X_INIT, SERVO_SPEED_X, SERVO_ACC);
        WritePosEx(SERVO_Y_ID, (int16_t)SERVO_Y_INIT, SERVO_SPEED_Y, SERVO_ACC);
        WritePosEx(SERVO2_ID, (int16_t)SERVO2_HOME, SERVO2_SPEED, SERVO_ACC);
        WritePosEx(SERVO4_ID, (int16_t)SERVO4_INIT, SERVO4_SPEED, SERVO_ACC);
        servo_pos_x = SERVO_X_INIT;
        servo_pos_y = SERVO_Y_INIT;
        stable_cnt = 0;
        elbow_cmd_done = 0;
        task_step = 9;
    }
    /* task_step == 9: 结束, 停住 */
}
