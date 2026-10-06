#include "PID.h"
#include "SCServo.h"
#include "receive.h"
#include "laser.h"
#include "mailuncontrol.h"
#include <math.h>

/* elog：LOG_TAG / LOG_LVL 必须先于 <elog.h> 定义（同 control.c 的约定） */
#define LOG_TAG    "SERVO"
#define LOG_LVL    ELOG_LVL_VERBOSE
#include <elog.h>

/* ============ 舵机ID定义 ============ */
#define SERVO_Y_ID     1      /* Y轴(上下/俯仰) */
#define SERVO2_ID      2      /* 肘/伸缩轴 */
#define SERVO_X_ID     3      /* X轴(水平/左右) */
#define SERVO4_ID      4      /* 夹爪 */

/* ============ 舵机位置范围 ============ */
#define SERVO_Y_MIN        1180   /* ID1: Y轴最上 */
#define SERVO_Y_MAX        3389   /* ID1: Y轴最下 */
#define SERVO_X_MIN        0      /* ID3: X轴无限制, 全范围兜底 */
#define SERVO_X_MAX        4095   /* ID3: X轴无限制, 全范围兜底 */

/* ============ 开机初始位置 ============ */
#define SERVO_Y_INIT       3389   /* ID1: Y轴开机位置 */
#define SERVO2_INIT        1974   /* ID2: 肘回缩/初始位 */
#define SERVO_X_INIT       950    /* ID3: X轴开机位置 */
#define SERVO4_INIT        2543   /* ID4: 夹爪初始位置 */

/* ============ 舵机速度 ============ */
#define SERVO_SPEED_Y      80     /* ID1: Y速度 */
#define SERVO2_SPEED       30     /* ID2: 肘速度 */
#define SERVO_SPEED_X      20     /* ID3: X速度 */
#define SERVO4_SPEED       50     /* ID4: 夹爪速度 */

/* ============ 舵机加速度 ============ */
#define SERVO_ACC          0

/* ============ HOLD 段动作(part1: HOLD1 握手+复位) ============ */
#define HOLD1_SERVO1_POS       3389   /* 舵机1 Y */
#define HOLD1_SERVO2_POS       1383   /* 舵机2 肘 */
#define HOLD1_SERVO3_POS       950    /* 舵机3 X */
#define HOLD1_SERVO4_POS       2543   /* 舵机4 夹爪 */
#define HOLD1_WAIT_MS          500    /* 相邻舵机到位延时(可调) */

/* ============ HOLD 段动作(part3: HOLD7 追靶+激光+舵机) ============ */
#define TARGET_SERVO1_POS      2753   /* 舵机1 Y */
#define TARGET_SERVO2_POS      1492   /* 舵机2 肘 */
#define TARGET_SERVO4_POS      2543   /* 舵机4 夹爪 */

/* ============ part3 识别完激光后的复位位姿 ============ */
#define LASER_END_S1_POS      3389   /* 舵机1 Y */
#define LASER_END_S2_POS      1346   /* 舵机2 肘 */
#define LASER_END_S3_POS      1477   /* 舵机3 X */
#define LASER_END_S4_POS      2543   /* 舵机4 夹爪 */
#define LASER_ON_MS            10000   /* 激光打开时长 */

/* ============ HOLD 段动作(part4: HOLD9 抓人质) ============ */
#define HOSTAGE_S1_INIT        3393   /* 舵机1 Y 初始 */
#define HOSTAGE_S2_INIT        1346   /* 舵机2 肘 初始 */
#define HOSTAGE_S4_INIT        2543   /* 舵机4 夹爪初始(同 SERVO4_INIT) */
#define HOSTAGE_X_SWEEP_END    2549   /* 舵机3 X 巡视终点(起点1477=part3遗留) */
#define HOSTAGE_X_SWEEP_SPEED  3      /* 舵机3 巡视速度(慢, 给视觉时间检测) */
#define HOSTAGE_S1_GRAB        3146   /* 舵机1 Y 抓取高度 */
#define HOSTAGE_GRIP_N1        1800   /* qr_z==1 舵机4 */
#define HOSTAGE_GRIP_N2        1730   /* qr_z==2 舵机4 */
#define HOSTAGE_GRIP_N3        1800   /* qr_z==3 舵机4 */
#define HOSTAGE_S2_LIFT        2224   /* 抓完后舵机2 抬起 */

/* ============ 到位判断 ============ */
#define SERVO_ARRIVE_TOL     20     /* 到位误差(计数), 可调 */
#define SERVO_MAX_WAIT_MS    5000   /* 兜底超时, 防止舵机卡住死等 */
#define RETURN_860_DELAY_MS  500    /* 抓完球/放完桶后, 车回 860 前的延时 */

/* ============ 视觉稳定判断阈值 ============ */
#define GRAB_STABLE_X     20     /* X误差阈值(像素), 可调 */
#define GRAB_STABLE_Y     20     /* Y误差阈值(像素), 可调 */
#define GRAB_STABLE_CNT   8      /* 连续稳定多少帧触发 */

/* ============ 视觉误差x -> 小车左右速度(part2 夹小球) ============ */
#define VISION_CAR_VY_KP    (0.8f)  /* px -> mm/s 增益, 现场调 */
#define VISION_CAR_VY_KP_PLACE   (-VISION_CAR_VY_KP)  /* 放桶: 机械臂往后看, 小车左右方向与球相反 */
#define VISION_CAR_VY_MAX   80.0f   /* 左右速度限幅 mm/s */

/* ============ part2 夹小球 固定位姿 ============ */
#define GRAB_S1_APPROACH    2951   /* 舵机1: 摆姿态(粗定位) */
#define GRAB_S2_APPROACH    670    /* 舵机2: 摆姿态 */
#define GRAB_S1_FINAL       2698   /* 舵机1: y校准稳定后的抓取高度 */
#define GRAB_S4_GRIP        1695   /* 舵机4: 夹爪抓 */
#define GRAB_S2_LIFT        1559   /* 舵机2: 抓完抬起 */

/* ============ part2 放桶 固定位姿 ============ */
#define PLACE_S3_APPROACH    3092   /* 舵机3 X: 转向桶位 */
#define PLACE_S1_APPROACH    3363   /* 舵机1 Y: 摆姿态 */
#define PLACE_S1_Y_TRACK     3026   /* 舵机1 Y: y校准前过渡位 */
#define PLACE_S2_APPROACH    985    /* 舵机2 肘: 放桶姿态 */
#define PLACE_S1_FINAL       2792   /* 舵机1 Y: 放桶高度 */
#define PLACE_S4_OPEN        2543   /* 舵机4 夹爪: 松开(同 SERVO4_INIT) */

/* ============ part2 放桶后复位(准备看激光) ============ */
#define LASER_PREP_S3_POS      950    /* 舵机3 X */
#define LASER_PREP_S1_POS      3389   /* 舵机1 Y */
#define LASER_PREP_S2_POS      1938   /* 舵机2 肘 */

/* ============ 通用 PID ============ */
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

/* ============ XY 舵机控制 ============ */
static PID_Controller_t pid_control_x;
static PID_Controller_t pid_control_y;

static float servo_pos_x = SERVO_X_INIT;   /* 累加的舵机绝对位置 */
static float servo_pos_y = SERVO_Y_INIT;

static uint8_t stable_cnt = 0;    /* 连续稳定帧计数 */

/* HOLD 动作握手 + 测试模拟标志 */
volatile uint8_t hold_action_state = HOLD_ACTION_IDLE;
volatile uint8_t hold_action_id    = 0;    /* 0=无 1=part1(HOLD1) 2=part2(HOLD6) 3=part3(HOLD7) 4=part4(HOLD9) */
volatile uint8_t sim_ball_stable   = 0;    /* KEY_3 第一次: 球稳定 */
volatile uint8_t sim_bucket_stable = 0;    /* KEY_3 第二次: 桶稳定 */

/* 视觉误差x -> 小车左右速度: track_xy_err_car 写入, control.c SM_HOLD6 读取 */
volatile float   vision_car_vy = 0.0f;
volatile uint8_t vision_car_track_enable = 0;

void Servo_PID_Init(void)
{
    /* 使能所有舵机扭矩(扭矩被关的话, 舵机收了位置指令也不会动) */
    EnableTorque(1, 1);
    EnableTorque(2, 1);
    EnableTorque(3, 1);
    EnableTorque(4, 1);

    /* Kp/Ki/Kd 现场调; 输出=每帧位置增量 */
    Control_PID_Init(&pid_control_x, -0.32f, 0.0f, 0.0f, -5000.0f, 5000.0f);
    Control_PID_Init(&pid_control_y, -0.25f, 0.0f, 0.0f, -5000.0f, 5000.0f);

    servo_pos_x = SERVO_X_INIT;
    servo_pos_y = SERVO_Y_INIT;

    WritePosEx(SERVO_X_ID, (int16_t)servo_pos_x, SERVO_SPEED_X, SERVO_ACC);
    WritePosEx(SERVO_Y_ID, (int16_t)servo_pos_y, SERVO_SPEED_Y, SERVO_ACC);
    WritePosEx(SERVO4_ID, (int16_t)SERVO4_INIT, SERVO4_SPEED, SERVO_ACC);
    WritePosEx(SERVO2_ID, (int16_t)SERVO2_INIT, SERVO2_SPEED, SERVO_ACC);

    stable_cnt = 0;
}

/* 跑一帧 X/Y PID 并写舵机(误差源参数化: 球用 grab_x/y, 桶用 track_x/y) */
static void track_xy_err(int16_t ex, int16_t ey)
{
    /* X轴 -> 舵机3 */
    PID_Compute(&pid_control_x, (float)ex);
    servo_pos_x += pid_control_x.output;
    if (servo_pos_x < SERVO_X_MIN) servo_pos_x = SERVO_X_MIN;
    if (servo_pos_x > SERVO_X_MAX) servo_pos_x = SERVO_X_MAX;
    WritePosEx(SERVO_X_ID, (int16_t)servo_pos_x, SERVO_SPEED_X, SERVO_ACC);

    /* Y轴 -> 舵机1 */
    PID_Compute(&pid_control_y, (float)ey);
    servo_pos_y += pid_control_y.output;
    if (servo_pos_y < SERVO_Y_MIN) servo_pos_y = SERVO_Y_MIN;
    if (servo_pos_y > SERVO_Y_MAX) servo_pos_y = SERVO_Y_MAX;
    WritePosEx(SERVO_Y_ID, (int16_t)servo_pos_y, SERVO_SPEED_Y, SERVO_ACC);
}

/* 停小车左右(退出视觉追踪) */
static void car_vy_stop(void)
{
    vision_car_vy = 0.0f;
    vision_car_track_enable = 0;
}

/* 小车x对准: 视觉x误差 -> 小车左右速度(vy); 舵机3固定950不动。kp=增益(球正/桶负) */
static void car_x_track(int16_t ex, float kp)
{
    vision_car_vy = kp * (float)ex;
    if (vision_car_vy >  VISION_CAR_VY_MAX) vision_car_vy =  VISION_CAR_VY_MAX;
    if (vision_car_vy < -VISION_CAR_VY_MAX) vision_car_vy = -VISION_CAR_VY_MAX;
    vision_car_track_enable = 1;
}

/* 舵机1 y校准: 视觉y误差 -> 舵机1 (PID微调) */
static void servo_y_track(int16_t ey)
{
    PID_Compute(&pid_control_y, (float)ey);
    servo_pos_y += pid_control_y.output;
    if (servo_pos_y < SERVO_Y_MIN) servo_pos_y = SERVO_Y_MIN;
    if (servo_pos_y > SERVO_Y_MAX) servo_pos_y = SERVO_Y_MAX;
    WritePosEx(SERVO_Y_ID, (int16_t)servo_pos_y, SERVO_SPEED_Y, SERVO_ACC);
}

/* 判稳定: 误差连续 GRAB_STABLE_CNT 帧在阈值内 */
static int check_stable(int16_t ex, int16_t ey)
{
    if (ex >= -GRAB_STABLE_X && ex <= GRAB_STABLE_X &&
        ey >= -GRAB_STABLE_Y && ey <= GRAB_STABLE_Y) {
        stable_cnt++;
        if (stable_cnt >= GRAB_STABLE_CNT) return 1;
    } else {
        stable_cnt = 0;
    }
    return 0;
}

/* 单轴稳定判定: 连续 GRAB_STABLE_CNT 帧在阈值内(x对准用x, y校准用y) */
static int check_stable_axis(int16_t e, int16_t thresh)
{
    if (e >= -thresh && e <= thresh) {
        stable_cnt++;
        if (stable_cnt >= GRAB_STABLE_CNT) return 1;
    } else {
        stable_cnt = 0;
    }
    return 0;
}

/* 按 qr_z(二维码第三位) 取出抓人质时舵机4 的夹爪位置 */
static int16_t hostage_grip_by_qr(void)
{
    switch (vision_data.qr_z) {
    case 1:  return HOSTAGE_GRIP_N1;
    case 2:  return HOSTAGE_GRIP_N2;
    default: return HOSTAGE_GRIP_N3;  /* qr_z==3 */
    }
}

/* 判断舵机是否到位: 读实际位置, 与目标误差在阈值内 */
static int servo_reached(uint8_t id, int16_t target)
{
    int pos = ReadPos(id);
    if (pos < 0) return 0;                 /* 读失败, 继续等 */
    int16_t diff = (int16_t)(pos - target);
    if (diff < 0) diff = -diff;
    return diff <= SERVO_ARRIVE_TOL;
}

/* ============ HOLD 段动作状态机(主循环每圈调用) ============
 * 状态机(中断)置 hold_action_id + hold_action_state=RUN, 本函数执行, 做完置 DONE。 */
void Hold_Action_Update(void)
{
    static uint8_t  step    = 0;
    static uint32_t t       = 0;
    static uint8_t  last_id = 0;

    if (hold_action_state != HOLD_ACTION_RUN) return;

    if (last_id != hold_action_id) {   /* 换动作了, 复位步进 */
        last_id = hold_action_id;
        step = 0;
        t = 0;
        stable_cnt = 0;
    }

    /* ---- part1: HOLD1 发 B6 01 6B -> 等 A5 -> 舵机 1→2→3→4 复位 ---- */
    if (hold_action_id == 1) {
        switch (step) {
        case 0:
            Vision_Send_B6(0x01);
            step = 1;
            break;
        case 1:                                 /* 等 A5 握手帧 */
            if (vision_data.rx_status) {
                WritePosEx(SERVO_Y_ID, (int16_t)HOLD1_SERVO1_POS, SERVO_SPEED_Y, SERVO_ACC);
                servo_pos_y = HOLD1_SERVO1_POS;
                t = HAL_GetTick();
                step = 2;
            }
            break;
        case 2:                                 /* 舵机1到位 -> 舵机2 */
            if (servo_reached(SERVO_Y_ID, HOLD1_SERVO1_POS) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO2_ID, (int16_t)HOLD1_SERVO2_POS, SERVO2_SPEED, SERVO_ACC);
                t = HAL_GetTick();
                step = 3;
            }
            break;
        case 3:                                 /* 舵机2到位 -> 舵机3 */
            if (servo_reached(SERVO2_ID, HOLD1_SERVO2_POS) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO_X_ID, (int16_t)HOLD1_SERVO3_POS, SERVO_SPEED_X, SERVO_ACC);
                servo_pos_x = HOLD1_SERVO3_POS;
                t = HAL_GetTick();
                step = 4;
            }
            break;
        case 4:                                 /* 舵机3到位 -> 舵机4, 发完即完成 */
            if (servo_reached(SERVO_X_ID, HOLD1_SERVO3_POS) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO4_ID, (int16_t)HOLD1_SERVO4_POS, SERVO4_SPEED, SERVO_ACC);
                hold_action_state = HOLD_ACTION_DONE;
                step = 0;
            }
            break;
        }
        return;
    }

    /* ---- part2(夹小球): 先x对准 -> 摆姿态 -> y校准 -> 抓 -> 抬 ----
     * 串行: 先只动车追x(舵机1不动), 再只动舵机1追y; 舵机3固定950; 固定位姿 */
    if (hold_action_id == 2) {
        switch (step) {
        case 0:                                  /* 发 B6 02 6B 要球 + 初始位 */
            Vision_Send_B6(0x02);
            vision_data.grab_flag = 0;
            vision_data.track_flag = 0;
            pid_control_y.error_last = 0.0f;
            pid_control_y.intergral = 0.0f;
            stable_cnt = 0;
            WritePosEx(SERVO_X_ID, (int16_t)SERVO_X_INIT, SERVO_SPEED_X, SERVO_ACC);  /* 舵机3固定950 */
            servo_pos_x = SERVO_X_INIT;
            WritePosEx(SERVO4_ID, (int16_t)SERVO4_INIT, SERVO4_SPEED, SERVO_ACC);     /* 夹爪开2543 */
            step = 1;
            break;
        case 1:                                  /* 小车x对准(只动车) */
            if (sim_ball_stable) {
                sim_ball_stable = 0;
                step = 2;                        /* 模拟x稳, 直接摆姿态 */
            } else if (vision_data.grab_flag) {
                vision_data.grab_flag = 0;
                car_x_track(vision_data.grab_x, VISION_CAR_VY_KP);
                if (check_stable_axis(vision_data.grab_x, GRAB_STABLE_X)) {
                    vision_car_vy=0.0f;
                    step = 2;
                }
            }
            break;
        case 2:                                  /* 摆姿态: 舵机1 -> 2345 */
            WritePosEx(SERVO_Y_ID, (int16_t)GRAB_S1_APPROACH, SERVO_SPEED_Y, SERVO_ACC);
            servo_pos_y = GRAB_S1_APPROACH;
            t = HAL_GetTick();
            step = 3;
            break;
        case 3:                                  /* 舵机2 -> 651, 并清y PID */
            if (servo_reached(SERVO_Y_ID, GRAB_S1_APPROACH) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO2_ID, (int16_t)GRAB_S2_APPROACH, SERVO2_SPEED, SERVO_ACC);
                pid_control_y.error_last = 0.0f;
                pid_control_y.intergral = 0.0f;
                stable_cnt = 0;
                t = HAL_GetTick();
                step = 4;
            }
            break;
        case 4:                                  /* y校准: 舵机1 用 vision y PID 微调 */
            if (vision_data.grab_flag) {
                vision_data.grab_flag = 0;
                servo_y_track(vision_data.grab_y);
                if (check_stable_axis(vision_data.grab_y, GRAB_STABLE_Y)) {
                    step = 5;                    /* y稳 -> 抓 */
                }
            }
            break;
        case 5:                                  /* 舵机1 -> 2048 (抓取高度) */
            WritePosEx(SERVO_Y_ID, (int16_t)GRAB_S1_FINAL, SERVO_SPEED_Y, SERVO_ACC);
            servo_pos_y = GRAB_S1_FINAL;
            t = HAL_GetTick();
            step = 6;
            break;
        case 6:                                  /* 夹爪4 -> 1695 抓 (舵机2保持651) */
            if (servo_reached(SERVO_Y_ID, GRAB_S1_FINAL) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO4_ID, (int16_t)GRAB_S4_GRIP, SERVO4_SPEED, SERVO_ACC);
                t = HAL_GetTick();
                step = 7;
            }
            break;
        case 7:                                  /* 舵机2 -> 1559 抬起(抓完球), 车暂不回860 */
            if (servo_reached(SERVO4_ID, GRAB_S4_GRIP) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO2_ID, (int16_t)GRAB_S2_LIFT, SERVO2_SPEED, SERVO_ACC);
                t = HAL_GetTick();               /* 开始计时 */
                step = 8;
            }
            break;
        case 8:                                  /* 延时 500ms 后再回 860 */
            if (HAL_GetTick() - t >= RETURN_860_DELAY_MS) {
                vision_car_track_enable = 0;     /* 退出追踪，SM_HOLD6 把车拉回 y=860 */
                step = 9;
            }
            break;
        case 9:                                  /* 发 B6 03 要桶 + 舵机3 -> 3092 */
            Vision_Send_B6(0x03);
            vision_data.track_flag = 0;
            pid_control_y.error_last = 0.0f;
            pid_control_y.intergral = 0.0f;
            stable_cnt = 0;
            WritePosEx(SERVO_X_ID, (int16_t)PLACE_S3_APPROACH, SERVO_SPEED_X, SERVO_ACC);
            servo_pos_x = PLACE_S3_APPROACH;
            t = HAL_GetTick();
            step = 10;
            break;
        case 10:                                 /* 舵机1 -> 3363 */
            if (servo_reached(SERVO_X_ID, PLACE_S3_APPROACH) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO_Y_ID, (int16_t)PLACE_S1_APPROACH, SERVO_SPEED_Y, SERVO_ACC);
                servo_pos_y = PLACE_S1_APPROACH;
                t = HAL_GetTick();
                step = 11;
            }
            break;
        case 11:                                 /* 小车 x 对准(只动车, 追桶 x; 机械臂往后看, 方向与球相反) */
            if (sim_bucket_stable) {
                sim_bucket_stable = 0;
                vision_car_vy = 0.0f;
                step = 12;                       /* 模拟 x 稳 */
            } else if (vision_data.track_flag) {
                vision_data.track_flag = 0;
                car_x_track(vision_data.grab_x, VISION_CAR_VY_KP_PLACE);
                if (check_stable_axis(vision_data.grab_x, GRAB_STABLE_X)) {
                    vision_car_vy = 0.0f;        /* x 稳 -> 车停 */
                    step = 12;
                }
            }
            break;
        case 12:                                 /* 舵机1 -> 3026 */
            WritePosEx(SERVO_Y_ID, (int16_t)PLACE_S1_Y_TRACK, SERVO_SPEED_Y, SERVO_ACC);
            servo_pos_y = PLACE_S1_Y_TRACK;
            t = HAL_GetTick();
            step = 13;
            break;
        case 13:                                 /* 舵机2 -> 985, 并清 y PID */
            if (servo_reached(SERVO_Y_ID, PLACE_S1_Y_TRACK) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO2_ID, (int16_t)PLACE_S2_APPROACH, SERVO2_SPEED, SERVO_ACC);
                pid_control_y.error_last = 0.0f;
                pid_control_y.intergral = 0.0f;
                stable_cnt = 0;
                t = HAL_GetTick();
                step = 14;
            }
            break;
        case 14:                                 /* 舵机 y 校准: 追桶 y (PID 微调) */
            if (vision_data.track_flag) {
                vision_data.track_flag = 0;
                servo_y_track(vision_data.grab_y);
                if (check_stable_axis(vision_data.grab_y, GRAB_STABLE_Y)) {
                    step = 15;                   /* y 稳 -> 放 */
                }
            }
            break;
        case 15:                                 /* 舵机1 -> 2792 (放桶高度) */
            WritePosEx(SERVO_Y_ID, (int16_t)PLACE_S1_FINAL, SERVO_SPEED_Y, SERVO_ACC);
            servo_pos_y = PLACE_S1_FINAL;
            t = HAL_GetTick();
            step = 16;
            break;
        case 16:                                 /* 夹爪4 -> 2543 松开, 车暂不回860 */
            if (servo_reached(SERVO_Y_ID, PLACE_S1_FINAL) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO4_ID, (int16_t)PLACE_S4_OPEN, SERVO4_SPEED, SERVO_ACC);
                t = HAL_GetTick();               /* 开始计时 */
                step = 17;
            }
            break;
        case 17:                                 /* 延时 500ms 后回 860 + 舵机3 -> 950(准备看激光) */
            if (HAL_GetTick() - t >= RETURN_860_DELAY_MS) {
                vision_car_track_enable = 0;     /* 退出追踪，SM_HOLD6 把车拉回 y=860 */
                WritePosEx(SERVO_X_ID, (int16_t)LASER_PREP_S3_POS, SERVO_SPEED_X, SERVO_ACC);
                servo_pos_x = LASER_PREP_S3_POS;
                t = HAL_GetTick();
                step = 18;
            }
            break;
        case 18:                                 /* 舵机1 -> 3391 */
            if (servo_reached(SERVO_X_ID, LASER_PREP_S3_POS) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO_Y_ID, (int16_t)LASER_PREP_S1_POS, SERVO_SPEED_Y, SERVO_ACC);
                servo_pos_y = LASER_PREP_S1_POS;
                t = HAL_GetTick();
                step = 19;
            }
            break;
        case 19:                                 /* 舵机2 -> 1938 */
            if (servo_reached(SERVO_Y_ID, LASER_PREP_S1_POS) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO2_ID, (int16_t)LASER_PREP_S2_POS, SERVO2_SPEED, SERVO_ACC);
                t = HAL_GetTick();
                step = 20;
            }
            break;
        case 20:                                 /* 舵机2 到位 -> 完成 */
            if (servo_reached(SERVO2_ID, LASER_PREP_S2_POS) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                hold_action_state = HOLD_ACTION_DONE;
                step = 0;
            }
            break;
        }
        return;
    }

    /* ---- part3: HOLD7 发 B6 04 -> 追靶心 -> 激光 -> 舵机复位 ---- */
    if (hold_action_id == 3) {
        switch (step) {
        case 0:                                  /* 发 B6 04 6B 要靶心 */
            Vision_Send_B6(0x04);
            vision_data.track_flag = 0;
            pid_control_x.error_last = 0.0f;
            pid_control_x.intergral = 0.0f;
            pid_control_y.error_last = 0.0f;
            pid_control_y.intergral = 0.0f;
            stable_cnt = 0;
            step = 1;
            break;
        case 1:                                  /* 等靶心 D8 (6字节) / 追靶 */
            if (vision_data.track_flag) {
                vision_data.track_flag = 0;
                track_xy_err(vision_data.grab_x, vision_data.grab_y);
                if (check_stable(vision_data.grab_x, vision_data.grab_y)) {
                    step = 2;                    /* 稳定 -> 开激光 */
                }
            }
            break;
        case 2:                                  /* 开激光 */
            laser_On();
            t = HAL_GetTick();
            step = 3;
            break;
        case 3:                                  /* 激光开期间继续追踪 */
            if (vision_data.track_flag) {
                vision_data.track_flag = 0;
                track_xy_err(vision_data.grab_x, vision_data.grab_y);
            }
            if (HAL_GetTick() - t >= LASER_ON_MS) {
                laser_Off();
                t = HAL_GetTick();
                step = 4;
            }
            break;
        case 4:                                  /* 舵机1 -> 3389 */
            WritePosEx(SERVO_Y_ID, (int16_t)LASER_END_S1_POS, SERVO_SPEED_Y, SERVO_ACC);
            servo_pos_y = LASER_END_S1_POS;
            t = HAL_GetTick();
            step = 5;
            break;
        case 5:                                  /* 舵机2 -> 1346 */
            if (servo_reached(SERVO_Y_ID, LASER_END_S1_POS) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO2_ID, (int16_t)LASER_END_S2_POS, SERVO2_SPEED, SERVO_ACC);
                t = HAL_GetTick();
                step = 6;
            }
            break;
        case 6:                                  /* 舵机3 -> 1477 */
            if (servo_reached(SERVO2_ID, LASER_END_S2_POS) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO_X_ID, (int16_t)LASER_END_S3_POS, SERVO_SPEED_X, SERVO_ACC);
                servo_pos_x = LASER_END_S3_POS;
                t = HAL_GetTick();
                step = 7;
            }
            break;
        case 7:                                  /* 舵机4 -> 2543, 完成 */
            if (servo_reached(SERVO_X_ID, LASER_END_S3_POS) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO4_ID, (int16_t)LASER_END_S4_POS, SERVO4_SPEED, SERVO_ACC);
                hold_action_state = HOLD_ACTION_DONE;
                step = 0;
            }
            break;
        }
        return;
    }

    /* ---- part4: 复位 -> X巡视2549 -> 视觉x→车vx, 视觉y→舵机1 -> 抓人质 ---- */
    if (hold_action_id == 4) {
        switch (step) {
        case 0:                                  /* 发 B6 05 + 舵机1 -> 3393 */
            Vision_Send_B6(0x05);
            WritePosEx(SERVO_Y_ID, (int16_t)HOSTAGE_S1_INIT, SERVO_SPEED_Y, SERVO_ACC);
            servo_pos_y = HOSTAGE_S1_INIT;
            t = HAL_GetTick();
            step = 1;
            break;
        case 1:                                  /* 舵机2 -> 1346 */
            if (servo_reached(SERVO_Y_ID, HOSTAGE_S1_INIT) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO2_ID, (int16_t)HOSTAGE_S2_INIT, SERVO2_SPEED, SERVO_ACC);
                t = HAL_GetTick();
                step = 2;
            }
            break;
        case 2:                                  /* 舵机4 -> 2543 */
            if (servo_reached(SERVO2_ID, HOSTAGE_S2_INIT) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO4_ID, (int16_t)HOSTAGE_S4_INIT, SERVO4_SPEED, SERVO_ACC);
                t = HAL_GetTick();
                step = 3;
            }
            break;
        case 3:                                  /* 舵机3 -> 2549 慢速巡视, 清视觉 */
            if (servo_reached(SERVO4_ID, HOSTAGE_S4_INIT) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO_X_ID, (int16_t)HOSTAGE_X_SWEEP_END, HOSTAGE_X_SWEEP_SPEED, SERVO_ACC);
                servo_pos_x = HOSTAGE_X_SWEEP_END;
                vision_data.grab_flag = 0;
                pid_control_y.error_last = 0.0f;
                pid_control_y.intergral = 0.0f;
                stable_cnt = 0;
                step = 4;
            }
            break;
        case 4:                                  /* 等到视觉 -> 停舵机3 巡视 */
            if (vision_data.grab_flag) {
                vision_data.grab_flag = 0;
                int16_t cur = (int16_t)ReadPos(SERVO_X_ID);
                if (cur > 0) {                   /* 写回当前角, 停住巡视 */
                    WritePosEx(SERVO_X_ID, cur, HOSTAGE_X_SWEEP_SPEED, SERVO_ACC);
                    servo_pos_x = cur;
                }
                step = 5;
            }
            break;
        case 5:                                  /* 视觉x -> 小车vx(mailun PID), 稳定 -> 停 */
            if (vision_data.grab_flag) {
                vision_data.grab_flag = 0;
                track_err_car_vx(vision_data.grab_x);
                if (check_stable_axis(vision_data.grab_x, GRAB_STABLE_X)) {
                    car_vx_stop();
                    stable_cnt = 0;              /* 复位, 供下一步 y 稳定判定 */
                    step = 6;
                }
            }
            break;
        case 6:                                  /* 视觉y -> 舵机1(PID) */
            if (vision_data.grab_flag) {
                vision_data.grab_flag = 0;
                servo_y_track(vision_data.grab_y);
                if (check_stable_axis(vision_data.grab_y, GRAB_STABLE_Y)) {
                    step = 7;
                }
            }
            break;
        case 7:                                  /* 舵机1 -> 3146 */
            WritePosEx(SERVO_Y_ID, (int16_t)HOSTAGE_S1_GRAB, SERVO_SPEED_Y, SERVO_ACC);
            servo_pos_y = HOSTAGE_S1_GRAB;
            t = HAL_GetTick();
            step = 8;
            break;
        case 8:                                  /* 舵机4 按 qr_z 夹 */
            if (servo_reached(SERVO_Y_ID, HOSTAGE_S1_GRAB) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO4_ID, (int16_t)hostage_grip_by_qr(), SERVO4_SPEED, SERVO_ACC);
                t = HAL_GetTick();
                step = 9;
            }
            break;
        case 9:                                  /* 舵机2 -> 2224 抬起, 完成 */
            if (servo_reached(SERVO4_ID, hostage_grip_by_qr()) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO2_ID, (int16_t)HOSTAGE_S2_LIFT, SERVO2_SPEED, SERVO_ACC);
                hold_action_state = HOLD_ACTION_DONE;
                step = 0;
            }
            break;
        }
        return;
    }
}

/* ============ 激光打靶测试(KEY_4 触发, 与运动系统无关) ============ */
volatile uint8_t test_laser_run = 0;

void Laser_Track_Test(void)
{
    static uint8_t step     = 0;
    static uint8_t last_run = 0;

    if (!test_laser_run) { last_run = 0; return; }

    if (!last_run) {                 /* 刚触发, 复位 */
        last_run = 1;
        step = 0;
        stable_cnt = 0;
    }

    switch (step) {
    case 0:                          /* 发 B6 04 6B 要靶心 */
        Vision_Send_B6(0x04);
        vision_data.track_flag = 0;
        pid_control_x.error_last = 0.0f;
        pid_control_x.intergral = 0.0f;
        pid_control_y.error_last = 0.0f;
        pid_control_y.intergral = 0.0f;
        stable_cnt = 0;
        step = 1;
        break;
    case 1:                          /* 纯PID追靶, 稳定 -> 开激光 */
        if (vision_data.track_flag) {
            vision_data.track_flag = 0;
            track_xy_err(vision_data.grab_x, vision_data.grab_y);
            if (check_stable(vision_data.grab_x, vision_data.grab_y)) {
                laser_On();
                step = 2;
            }
        }
        break;
    case 2:                          /* 继续追踪(激光开), 直到再按 KEY_4 关闭 */
        if (vision_data.track_flag) {
            vision_data.track_flag = 0;
            track_xy_err(vision_data.grab_x, vision_data.grab_y);
        }
        break;
    }
}

//打印每个舵机的一个位置
void FT_test_debug(void)
{
    log_i("S1:%d S2:%d S3:%d S4:%d",
          ReadPos(SERVO_Y_ID), ReadPos(SERVO2_ID), ReadPos(SERVO_X_ID), ReadPos(SERVO4_ID));
}
