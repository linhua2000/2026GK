#ifndef __OLED_UI_H
#define __OLED_UI_H

/* 把视觉收到的数据刷新到 OLED 显示 */
void OLED_ShowVision(void);

/* 把陀螺仪姿态角 Roll/Pitch/Yaw 刷新到 OLED 显示 */
void OLED_ShowGyro(void);

/* 把里程计坐标 x/y/θ 刷新到 OLED 显示 */
void OLED_ShowOdom(void);

/* 把状态机状态 + 收发状态 + 坐标 + 偏航角刷新到 OLED */
void OLED_ShowStatus(void);

#endif
