#include "oled_ui.h"
#include "OLED.h"
#include "receive.h"
#include "jy61p.h"
#include "odometry.h"
#include "control.h"

/* 显示带符号十进制数(无前导零), 如 +5 / -640 */
static void OLED_ShowSigned(int16_t X, int16_t Y, int32_t Number, uint8_t FontSize)
{
    uint8_t digits[6];
    uint8_t len = 0;
    uint8_t i;
    uint32_t abs = (Number < 0) ? (uint32_t)(-Number) : (uint32_t)Number;

    if (abs == 0) {
        digits[len++] = 0;
    }
    while (abs > 0) {
        digits[len++] = abs % 10;
        abs /= 10;
    }

    OLED_ShowChar(X, Y, (Number < 0) ? '-' : '+', FontSize);
    X += FontSize;
    for (i = len; i > 0; i--) {
        OLED_ShowChar(X, Y, digits[i - 1] + '0', FontSize);
        X += FontSize;
    }
}

/* 把视觉收到的数据刷新到 OLED 显示 */
void OLED_ShowVision(void)
{
    OLED_Clear(); 

    /* 抓取 D8 x(int16) y(int16) 8D, 坐标为相对画面中心的误差 */
    OLED_ShowString(0, 0, "GR:", OLED_6X8);
    OLED_ShowSigned(24, 0, vision_data.grab_x, OLED_6X8);
    OLED_ShowSigned(72, 0, vision_data.grab_y, OLED_6X8);

    /* 循迹 B6 x y 6B */
    OLED_ShowString(0, 8, "TR:", OLED_6X8);
    OLED_ShowHexNum(24, 8, (uint8_t)vision_data.track_x, 2, OLED_6X8);
    OLED_ShowHexNum(42, 8, (uint8_t)vision_data.track_y, 2, OLED_6X8);

    /* 二维码 A5 x y z 5A */
    OLED_ShowString(0, 16, "QR:", OLED_6X8);
    OLED_ShowHexNum(24, 16, (uint8_t)vision_data.qr_x, 2, OLED_6X8);
    OLED_ShowHexNum(42, 16, (uint8_t)vision_data.qr_y, 2, OLED_6X8);
    OLED_ShowHexNum(60, 16, (uint8_t)vision_data.qr_z, 2, OLED_6X8);

    /* 转弯 C7 0 7C */
    OLED_ShowString(0, 24, "TN:", OLED_6X8);
    OLED_ShowHexNum(24, 24, vision_data.turn_flag, 2, OLED_6X8);

    /* 距离 D8 ... d 8D */
    OLED_ShowString(0, 32, "DI:", OLED_6X8);
    OLED_ShowSigned(24, 32, vision_data.grab_dist, OLED_6X8);

    OLED_Update();
}

/* 显示带符号小数(无前导零, 固定小数位), 如 +12.34 / -5.67 / +180.00 */
static void OLED_ShowSignedFloat(int16_t X, int16_t Y, float Number, uint8_t FraDigits, uint8_t FontSize)
{
    uint32_t scale = 1, abs_scaled, int_part, fra_part;
    int32_t  scaled;
    uint8_t  i, len = 0, digits[8];

    for (i = 0; i < FraDigits; i++) scale *= 10;
    scaled = (Number >= 0) ? (int32_t)(Number * scale + 0.5f)
                           : (int32_t)(Number * scale - 0.5f);      /* 四舍五入 */

    OLED_ShowChar(X, Y, (scaled < 0) ? '-' : '+', FontSize);
    X += FontSize;

    abs_scaled = (scaled < 0) ? (uint32_t)(-scaled) : (uint32_t)scaled;
    int_part   = abs_scaled / scale;
    fra_part   = abs_scaled % scale;

    if (int_part == 0) digits[len++] = 0;                          /* 整数部分无前导零 */
    while (int_part > 0) { digits[len++] = int_part % 10; int_part /= 10; }
    for (i = len; i > 0; i--) { OLED_ShowChar(X, Y, digits[i - 1] + '0', FontSize); X += FontSize; }

    OLED_ShowChar(X, Y, '.', FontSize); X += FontSize;
    OLED_ShowNum(X, Y, fra_part, FraDigits, FontSize);             /* ShowNum 自动补前导零 */
}

/* 把陀螺仪姿态角 Roll/Pitch/Yaw 刷新到 OLED 显示 */
void OLED_ShowGyro(void)
{
    OLED_Clear();

    OLED_ShowString(0,  0, "GYRO", OLED_6X8);

    OLED_ShowString(0, 16, "Roll :", OLED_6X8);
    OLED_ShowSignedFloat(48, 16, Roll, 2, OLED_6X8);

    OLED_ShowString(0, 32, "Pitch:", OLED_6X8);
    OLED_ShowSignedFloat(48, 32, Pitch, 2, OLED_6X8);

    OLED_ShowString(0, 48, "Yaw  :", OLED_6X8);
    OLED_ShowSignedFloat(48, 48, Yaw, 2, OLED_6X8);

    OLED_Update();
}

/* 把里程计坐标 x/y/θ 刷新到 OLED 显示。
 * 和 OLED_ShowGyro 并存 —— 哪一页显示由调用方(main 循环)决定，这里只管画。
 * y=32 那一行故意留空；陀螺仪页还在显示 Yaw，要对照时再补上去。 */
void OLED_ShowOdom(void)
{
    OLED_Clear();

    OLED_ShowString(0,  0, "ODOM", OLED_6X8);

    OLED_ShowString(0,  8, "X :", OLED_6X8);
    OLED_ShowSignedFloat(24, 8, odometry.x, 1, OLED_6X8);
    OLED_ShowString(96, 8, "mm", OLED_6X8);

    OLED_ShowString(0, 16, "Y :", OLED_6X8);
    OLED_ShowSignedFloat(24, 16, odometry.y, 1, OLED_6X8);
    OLED_ShowString(96, 16, "mm", OLED_6X8);

    OLED_ShowString(0, 24, "Th:", OLED_6X8);
    OLED_ShowSignedFloat(24, 24, odometry.theta, 1, OLED_6X8);
    OLED_ShowString(96, 24, "dg", OLED_6X8);

    OLED_ShowString(0, 32, "Yaw  :", OLED_6X8);
    OLED_ShowSignedFloat(48, 32, Yaw, 2, OLED_6X8);

    OLED_Update();
}

/* 把状态机状态 + 收发状态 + 里程计坐标 + 偏航角刷新到 OLED */
void OLED_ShowStatus(void)
{
    OLED_Clear();

    /* 状态机当前状态 */
    OLED_ShowString(0,  0, "ST:", OLED_6X8);
    OLED_ShowString(18, 0, (char *)Control_GetStateName(), OLED_6X8);

    /* 发送命令(不显示帧头) */
    OLED_ShowString(0,  8, "TX:", OLED_6X8);
    OLED_ShowHexNum(18, 8, vision_data.last_tx_cmd, 2, OLED_6X8);

    /* 接收状态 0/1 */
    OLED_ShowString(0, 16, "RX:", OLED_6X8);
    OLED_ShowNum(18, 16, vision_data.rx_status, 1, OLED_6X8);

    /* 里程计坐标 */
    OLED_ShowString(0, 24, "X :", OLED_6X8);
    OLED_ShowSignedFloat(24, 24, odometry.x, 1, OLED_6X8);
    OLED_ShowString(0, 32, "Y :", OLED_6X8);
    OLED_ShowSignedFloat(24, 32, odometry.y, 1, OLED_6X8);

    /* 偏航角 */
    OLED_ShowString(0, 40, "Yaw:", OLED_6X8);
    OLED_ShowSignedFloat(36, 40, Yaw, 2, OLED_6X8);

    /* 视觉收到的球帧数据 D8 x y num1 */
    OLED_ShowString(0, 48, "V:", OLED_6X8);
    OLED_ShowSigned(18, 48, vision_data.grab_x, OLED_6X8);
    OLED_ShowSigned(66, 48, vision_data.grab_y, OLED_6X8);
    OLED_ShowNum(108, 48, vision_data.num1, 1, OLED_6X8);

    /* 视觉收到的二维码帧数据 A5 x y z */
    OLED_ShowString(0, 56, "Q:", OLED_6X8);
    OLED_ShowSigned(18, 56, vision_data.qr_x, OLED_6X8);
    OLED_ShowSigned(48, 56, vision_data.qr_y, OLED_6X8);
    OLED_ShowSigned(78, 56, vision_data.qr_z, OLED_6X8);

    OLED_Update();
}
