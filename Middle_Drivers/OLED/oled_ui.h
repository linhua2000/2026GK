#ifndef __OLED_UI_H
#define __OLED_UI_H

/* 把视觉收到的数据刷新到 OLED 显示 */
void OLED_ShowVision(void);

/* 把陀螺仪姿态角 Roll/Pitch/Yaw 刷新到 OLED 显示 */
void OLED_ShowGyro(void);

#endif
