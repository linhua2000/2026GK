#include "oled_ui.h"
#include "OLED.h"
#include "receive.h"

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
    OLED_Clear();  /* 先清空显存, 避免残留上一帧或 "Waiting..." 的字符 */

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

    OLED_Update();
}
