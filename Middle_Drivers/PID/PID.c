#include "PID.h"
#include "SCServo.h"
#include "receive.h"
#include "laser.h"
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
#define SERVO_Y_MAX        2770   /* ID1: Y轴最下 */
#define SERVO_X_MIN        0      /* ID3: X轴无限制, 全范围兜底 */
#define SERVO_X_MAX        4095   /* ID3: X轴无限制, 全范围兜底 */

/* ============ 开机初始位置 ============ */
#define SERVO_Y_INIT       2600   /* ID1: Y轴开机位置 */
#define SERVO2_INIT        1871   /* ID2: 肘回缩/初始位 */
#define SERVO_X_INIT       950    /* ID3: X轴开机位置 */
#define SERVO4_INIT        2543   /* ID4: 夹爪初始位置 */

/* ============ 舵机速度 ============ */
#define SERVO_SPEED_Y      25     /* ID1: Y速度 */
#define SERVO2_SPEED       30     /* ID2: 肘速度 */
#define SERVO_SPEED_X      20     /* ID3: X速度 */
#define SERVO4_SPEED       50     /* ID4: 夹爪速度 */

/* ============ 舵机加速度 ============ */
#define SERVO_ACC          0

/* ============ HOLD 段动作(part1: HOLD1 握手+复位) ============ */
#define HOLD1_SERVO1_POS       2770   /* 舵机1 Y */
#define HOLD1_SERVO2_POS       1505   /* 舵机2 肘 */
#define HOLD1_SERVO3_POS       950    /* 舵机3 X */
#define HOLD1_SERVO4_POS       2543   /* 舵机4 夹爪 */
#define HOLD1_WAIT_MS          500    /* 相邻舵机到位延时(可调) */

/* ============ HOLD 段动作(part2: HOLD6 追球抓球+追桶放桶) ============ */
#define HOLD2_WAIT_MS          500    /* part2 每步到位延时(可调) */
#define X_TURN_WAIT_MS         9000   /* 舵机3(X轴)转桶位后的到位延时, 比 HOLD2_WAIT_MS 长一倍 */
/* 按 num1 的抓球位置 */
#define GRAB_S1_N1             1790   /* num1==1 舵机1 */
#define GRAB_S2_N1             651    /* num1==1 舵机2 */
#define GRAB_S1_N2             2080   /* num1==2 舵机1 */
#define GRAB_S2_N2             746    /* num1==2 舵机2 */
#define GRAB_S1_N3             1790   /* num1==3 舵机1 */
#define GRAB_S2_N3             640    /* num1==3 舵机2 */
/* 后续固定位置 */
#define GRAB4_POS              1790   /* 夹爪抓 */
#define GRAB4_SPEED            60     /* 夹爪抓取速度(其余舵机保持原速) */
#define ELBOW_MID              1925
#define X_BUCKET               3050
#define Y_TRACE_BUCKET         2080
#define Y_BUCKET               2087
#define ELBOW_PLACE            976
#define GRIP_OPEN              2543   /* 夹爪开 */
#define Y_FINAL1               2551
#define X_FINAL                976

/* ============ HOLD 段动作(part3: HOLD7 追靶+激光+舵机) ============ */
#define TARGET_SERVO1_POS      2753   /* 舵机1 Y */
#define TARGET_SERVO2_POS      1492   /* 舵机2 肘 */
#define TARGET_SERVO4_POS      2543   /* 舵机4 夹爪 */
#define LASER_ON_MS            10000   /* 激光打开时长 */

/* ============ HOLD 段动作(part4: HOLD9 抓人质) ============ */
#define HOSTAGE_X_POS          2834   /* 舵机3 X 巡视终点 */
#define HOSTAGE_S1_N1          2197   /* num==1 舵机1 */
#define HOSTAGE_S2_N1          1112   /* num==1 舵机2 */
#define HOSTAGE_S1_N2          2292   /* num==2 舵机1 */
#define HOSTAGE_S2_N2          1166   /* num==2 舵机2 */
#define HOSTAGE_S1_N3          2102   /* num==3 舵机1 */
#define HOSTAGE_S2_N3          1071   /* num==3 舵机2 */
#define HOSTAGE_GRIP_N1        2278   /* qr_z==1 舵机4 */
#define HOSTAGE_GRIP_N2        1866   /* qr_z==2 舵机4 */
#define HOSTAGE_GRIP_N3        2142   /* qr_z==3 舵机4 */
#define HOSTAGE_ELBOW_END      1844   /* 抓完后舵机2 */

/* ============ 到位判断 ============ */
#define SERVO_ARRIVE_TOL     20     /* 到位误差(计数), 可调 */
#define SERVO_MAX_WAIT_MS    5000   /* 兜底超时, 防止舵机卡住死等 */

/* ============ 视觉稳定判断阈值 ============ */
#define GRAB_STABLE_X     20     /* X误差阈值(像素), 可调 */
#define GRAB_STABLE_Y     20     /* Y误差阈值(像素), 可调 */
#define GRAB_STABLE_CNT   8      /* 连续稳定多少帧触发 */

/* ============ 视觉误差x -> 小车左右速度(part2 夹小球) ============ */
#define VISION_CAR_VY_KP    (0.8f)  /* px -> mm/s 增益(负, 与舵机3同符号), 现场调 */
#define VISION_CAR_VY_MAX   80.0f   /* 左右速度限幅 mm/s */

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
    Control_PID_Init(&pid_control_x, -0.25f, 0.0f, 0.0f, -5000.0f, 5000.0f);
    Control_PID_Init(&pid_control_y, -0.2f, 0.0f, 0.0f, -5000.0f, 5000.0f);

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

/* 夹小球追踪: X误差 -> 小车左右(vy), Y误差 -> 舵机1, 舵机3(X)固定950不动 */
static void track_xy_err_car(int16_t ex, int16_t ey)
{
    /* X轴 -> 小车左右(vy): 与舵机3同符号(负增益), 对应 control.c 注释 +vy左 -vy右 */
    vision_car_vy = VISION_CAR_VY_KP * (float)ex;
    if (vision_car_vy >  VISION_CAR_VY_MAX) vision_car_vy =  VISION_CAR_VY_MAX;
    if (vision_car_vy < -VISION_CAR_VY_MAX) vision_car_vy = -VISION_CAR_VY_MAX;
    vision_car_track_enable = 1;

    /* Y轴 -> 舵机1 (与原逻辑一致) */
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

/* 按 num1 取出抓球时舵机1/舵机2 的目标位置 */
static void grab_pos_by_num1(int16_t *s1, int16_t *s2)
{
    switch (vision_data.num1) {
    case 1:  *s1 = GRAB_S1_N1; *s2 = GRAB_S2_N1; break;
    case 2:  *s1 = GRAB_S1_N2; *s2 = GRAB_S2_N2; break;
    default: *s1 = GRAB_S1_N3; *s2 = GRAB_S2_N3; break;  /* num1==3 */
    }
}

/* 按 num1 取出抓人质时舵机1/舵机2 的目标位置 */
static void hostage_pos_by_num1(int16_t *s1, int16_t *s2)
{
    switch (vision_data.num1) {
    case 1:  *s1 = HOSTAGE_S1_N1; *s2 = HOSTAGE_S2_N1; break;
    case 2:  *s1 = HOSTAGE_S1_N2; *s2 = HOSTAGE_S2_N2; break;
    default: *s1 = HOSTAGE_S1_N3; *s2 = HOSTAGE_S2_N3; break;  /* num1==3 */
    }
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
    int16_t s1 = 0, s2 = 0;

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

    /* ---- part2(测试1: 夹小球): 发 B6 02 -> 追球(x->车, y->舵机1, 舵机3=950) -> 抓球 ----
     * 放小球(抬球+识别桶+放桶)是下一个测试, 原序列保留在下方的 step5~18 */
    if (hold_action_id == 2) {
        switch (step) {
        case 0:                                  /* 发 B6 02 6B 要球 */
            Vision_Send_B6(0x02);
            vision_data.grab_flag = 0;
            vision_data.track_flag = 0;
            pid_control_y.error_last = 0.0f;
            pid_control_y.intergral = 0.0f;
            stable_cnt = 0;
            // car_vy_stop();                       /* 确保小车左右不动 */
            WritePosEx(SERVO_X_ID, (int16_t)SERVO_X_INIT, SERVO_SPEED_X, SERVO_ACC);  /* 舵机3固定950 */
            servo_pos_x = SERVO_X_INIT;
            step = 1;
            break;
        case 1:                                  /* 等球/追球: x->车, y->舵机1 */
            if (sim_ball_stable) {
                sim_ball_stable = 0;
                // car_vy_stop();
                step = 2;                        /* 模拟稳定, 直接抓 */
            } else if (vision_data.grab_flag) {
                vision_data.grab_flag = 0;
                track_xy_err_car(vision_data.grab_x, vision_data.grab_y);
                if (check_stable(vision_data.grab_x, vision_data.grab_y)) {
                    // car_vy_stop();               /* 稳定 -> 小车停, 舵机1保持 */
                    step = 2;
                }
            }
            break;
        case 2:                                  /* 舵机1 按 num1 */
            grab_pos_by_num1(&s1, &s2);
            WritePosEx(SERVO_Y_ID, s1, SERVO_SPEED_Y, SERVO_ACC);
            servo_pos_y = s1;
            t = HAL_GetTick();
            step = 3;
            break;
        case 3:                                  /* 舵机2 按 num1 */
            grab_pos_by_num1(&s1, &s2);
            if (servo_reached(SERVO_Y_ID, s1) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO2_ID, s2, SERVO2_SPEED, SERVO_ACC);
                t = HAL_GetTick();
                step = 4;
            }
            break;
        case 4:                                  /* 舵机4 夹爪抓 -> 测试结束 */
            grab_pos_by_num1(&s1, &s2);
            if (servo_reached(SERVO2_ID, s2) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO4_ID, (int16_t)GRAB4_POS, GRAB4_SPEED, SERVO_ACC);
                hold_action_state = HOLD_ACTION_DONE;
                step = 0;
            }
            break;
        /* ---- 下方 step5~18 为原放小球序列(抬肘->转桶->B6 03->放桶), 下一个测试再用 ----
         * 夹小球测试在 step4 抓完即结束; 放小球测试时把上面 step4 的 DONE 改回 step=5 即可进入。 */
        case 5:                                  /* 2 -> 1925 抬肘 */
            if (servo_reached(SERVO4_ID, GRAB4_POS) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO2_ID, (int16_t)ELBOW_MID, SERVO2_SPEED, SERVO_ACC);
                t = HAL_GetTick();
                vision_car_track_enable=0;  /* 小车左右停 */
                step = 6;
            }
            break;
        case 6:                                  /* 3 -> 3050 */
            if (servo_reached(SERVO2_ID, ELBOW_MID) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO_X_ID, (int16_t)X_BUCKET, SERVO_SPEED_X, SERVO_ACC);
                servo_pos_x = X_BUCKET;
                t = HAL_GetTick();
                step = 7;
            }
            break;
        case 7:                                  /* 4 -> 2075 爪夹开一下 */
            if (servo_reached(SERVO_X_ID, X_BUCKET) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO4_ID, (int16_t)2075, SERVO4_SPEED, SERVO_ACC);
                t = HAL_GetTick();
                step = 8;
            }
            break;
        case 8:                                  /* 4 -> 1790 爪夹关一下 */
            if (servo_reached(SERVO4_ID, 2075) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO4_ID, (int16_t)GRAB4_POS, SERVO4_SPEED, SERVO_ACC);
                t = HAL_GetTick();
                step = 9;
            }
            break;
        case 9:                                  /* 1 -> Y_TRACE_BUCKET (先等X轴转到桶位) */
            if (servo_reached(SERVO4_ID, GRAB4_POS) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO_Y_ID, (int16_t)Y_TRACE_BUCKET, SERVO_SPEED_Y, SERVO_ACC);
                servo_pos_y = Y_TRACE_BUCKET;
                t = HAL_GetTick();
                step = 10;
            }
            break;
        case 10:                                 /* 发 B6 03 6B 要桶 */
            Vision_Send_B6(0x03);
            vision_data.track_flag = 0;
            pid_control_x.error_last = 0.0f;
            pid_control_x.intergral = 0.0f;
            pid_control_y.error_last = 0.0f;
            pid_control_y.intergral = 0.0f;
            stable_cnt = 0;
            step = 11;
            break;
        case 11:                                 /* 放桶 (暂时注释追桶PID, 写死直接进序列) */
            /* ---- 暂时注释: 等桶/追桶 PID ----
            if (sim_bucket_stable) {
                sim_bucket_stable = 0;
                step = 12;                       // 模拟稳定, 舵机3保持不变
            } else if (vision_data.track_flag) {
                vision_data.track_flag = 0;
                track_xy_err(vision_data.grab_x, vision_data.grab_y);
                if (check_stable(vision_data.grab_x, vision_data.grab_y)) {
                    step = 12;                   // 稳定后3不变(不写舵机3)
                }
            }
            */
            step = 12;                           /* 写死: 直接进放桶序列 */
            break;
        case 12:                                 /* 1 -> Y_BUCKET */
            WritePosEx(SERVO_Y_ID, (int16_t)Y_BUCKET, SERVO_SPEED_Y, SERVO_ACC);
            servo_pos_y = Y_BUCKET;
            t = HAL_GetTick();
            step = 13;
            break;
        case 13:                                 /* 2 -> ELBOW_PLACE */
            if (servo_reached(SERVO_Y_ID, Y_BUCKET) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO2_ID, (int16_t)ELBOW_PLACE, SERVO2_SPEED, SERVO_ACC);
                t = HAL_GetTick();
                step = 14;
            }
            break;
        case 14:                                 /* 4 -> GRIP_OPEN */
            if (servo_reached(SERVO2_ID, ELBOW_PLACE) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO4_ID, (int16_t)GRIP_OPEN, SERVO4_SPEED, SERVO_ACC);
                t = HAL_GetTick();
                step = 15;
            }
            break;
        case 15:                                 /* 2 -> 1790 */
            if (servo_reached(SERVO4_ID, GRIP_OPEN) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO2_ID, 1790, SERVO2_SPEED, SERVO_ACC);
                t = HAL_GetTick();
                step = 16;
            }
            break;
        case 16:                                 /* 1 -> Y_FINAL1 */
            if (servo_reached(SERVO2_ID, 1790) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO_Y_ID, (int16_t)Y_FINAL1, SERVO_SPEED_Y, SERVO_ACC);
                servo_pos_y = Y_FINAL1;
                t = HAL_GetTick();
                step = 17;
            }
            break;
        case 17:                                 /* 3 -> X_FINAL */
            if (servo_reached(SERVO_Y_ID, Y_FINAL1) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO_X_ID, (int16_t)X_FINAL, SERVO_SPEED_X, SERVO_ACC);
                servo_pos_x = X_FINAL;
                t = HAL_GetTick();
                step = 18;
            }
            break;
        case 18:                                 /* 4 -> GRIP_OPEN, 完成 */
            if (servo_reached(SERVO_X_ID, X_FINAL) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO4_ID, (int16_t)GRIP_OPEN, SERVO4_SPEED, SERVO_ACC);
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
        case 4:                                  /* 舵机2 -> 1492 */
            WritePosEx(SERVO2_ID, (int16_t)TARGET_SERVO2_POS, SERVO2_SPEED, SERVO_ACC);
            t = HAL_GetTick();
            step = 5;
            break;
        case 5:                                  /* 舵机1 -> 2753 */
            if (servo_reached(SERVO2_ID, TARGET_SERVO2_POS) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO_Y_ID, (int16_t)TARGET_SERVO1_POS, SERVO_SPEED_Y, SERVO_ACC);
                servo_pos_y = TARGET_SERVO1_POS;
                t = HAL_GetTick();
                step = 6;
            }
            break;
        case 6:                                  /* 舵机4 -> 2543, 完成 (X不变) */
            if (servo_reached(SERVO_Y_ID, TARGET_SERVO1_POS) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO4_ID, (int16_t)TARGET_SERVO4_POS, SERVO4_SPEED, SERVO_ACC);
                hold_action_state = HOLD_ACTION_DONE;
                step = 0;
            }
            break;
        }
        return;
    }

    /* ---- part4: HOLD9 复位姿态 -> X巡视2834 -> 追人质 -> 抓人质 ---- */
    if (hold_action_id == 4) {
        switch (step) {
        case 0:                                  /* 发 B6 05, 先复位舵机2 -> 1492 */
            Vision_Send_B6(0x05);
            WritePosEx(SERVO2_ID, (int16_t)TARGET_SERVO2_POS, SERVO2_SPEED, SERVO_ACC);
            t = HAL_GetTick();
            step = 1;
            break;
        case 1:                                  /* 舵机1 -> 2753 */
            if (servo_reached(SERVO2_ID, TARGET_SERVO2_POS) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO_Y_ID, (int16_t)TARGET_SERVO1_POS, SERVO_SPEED_Y, SERVO_ACC);
                servo_pos_y = TARGET_SERVO1_POS;
                t = HAL_GetTick();
                step = 2;
            }
            break;
        case 2:                                  /* 舵机4 -> 2543 */
            if (servo_reached(SERVO_Y_ID, TARGET_SERVO1_POS) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO4_ID, (int16_t)TARGET_SERVO4_POS, SERVO4_SPEED, SERVO_ACC);
                t = HAL_GetTick();
                step = 3;
            }
            break;
        case 3:                                  /* X -> 2834 巡视(慢速 ramp) */
            if (servo_reached(SERVO4_ID, TARGET_SERVO4_POS) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO_X_ID, (int16_t)HOSTAGE_X_POS, 3, SERVO_ACC);
                servo_pos_x = HOSTAGE_X_POS;
                vision_data.grab_flag = 0;
                pid_control_x.error_last = 0.0f;
                pid_control_x.intergral = 0.0f;
                pid_control_y.error_last = 0.0f;
                pid_control_y.intergral = 0.0f;
                stable_cnt = 0;
                step = 4;
            }
            break;
        case 4:                                  /* PID 识别, 稳定 -> 抓 */
            if (vision_data.grab_flag) {
                vision_data.grab_flag = 0;
                servo_pos_x = (float)ReadPos(SERVO_X_ID);  /* 读实际X角度, 打断巡视, 从当前开始追目标 */
                servo_pos_y = (float)ReadPos(SERVO_Y_ID);
                track_xy_err(vision_data.grab_x, vision_data.grab_y);
                if (check_stable(vision_data.grab_x, vision_data.grab_y)) {
                    step = 5;
                }
            }
            break;
        case 5:                                  /* 舵机1 按 num1 */
            hostage_pos_by_num1(&s1, &s2);
            WritePosEx(SERVO_Y_ID, s1, SERVO_SPEED_Y, SERVO_ACC);
            servo_pos_y = s1;
            t = HAL_GetTick();
            step = 6;
            break;
        case 6:                                  /* 舵机2 按 num1 */
            hostage_pos_by_num1(&s1, &s2);
            if (servo_reached(SERVO_Y_ID, s1) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO2_ID, s2, SERVO2_SPEED, SERVO_ACC);
                t = HAL_GetTick();
                step = 7;
            }
            break;
        case 7:                                  /* 舵机4 按 qr_z 夹 */
            if (servo_reached(SERVO2_ID, s2) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO4_ID, (int16_t)hostage_grip_by_qr(), SERVO4_SPEED, SERVO_ACC);
                t = HAL_GetTick();
                step = 8;
            }
            break;
        case 8:                                  /* 舵机2 -> 1844, 完成 */
            if (servo_reached(SERVO4_ID, hostage_grip_by_qr()) || HAL_GetTick() - t >= SERVO_MAX_WAIT_MS) {
                WritePosEx(SERVO2_ID, (int16_t)HOSTAGE_ELBOW_END, SERVO2_SPEED, SERVO_ACC);
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
