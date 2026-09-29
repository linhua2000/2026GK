#ifndef __KEY_H          // 防止头文件重复包含
#define __KEY_H

#include <stdint.h>     // uint8_t

#define KEY_COUNT				4   // 按键总数，同时是 Key_Flag / 状态机数组的长度

// 按键编号，直接当 Key_Check / Key_GetState 的 n 参数用
#define KEY_1					0
#define KEY_2					1
#define KEY_3					2
#define KEY_4					3

/*---------------------------------- 按键事件标志位 ----------------------------------*/
// 这些标志位用于 Key_Flag[] 变量，表示检测到的按键事件
#define KEY_HOLD		0x01    // 按键保持按下状态（长按过程中持续有效）
#define KEY_DOWN		0x02    // 按键按下瞬间（下降沿）
#define KEY_UP			0x04    // 按键释放瞬间（上升沿）
#define KEY_SINGLE		0x08    // 单击事件（按下后快速释放，未触发双击）
#define KEY_DOUBLE		0x10    // 双击事件（在指定时间内连续按下两次）
#define KEY_LONG		0x20    // 长按事件（按下时间超过长按阈值）
#define KEY_REPEAT		0x40    // 长按重复触发事件（长按期间周期性触发，用于连续响应）

/*---------------------------------- 函数声明 ----------------------------------*/
/**
 * @brief   按键硬件初始化（把四个按键引脚配成上拉输入）
 * @param   无
 * @retval  无
 * @note    引脚分配：KEY_1=PC5  KEY_2=PE7  KEY_3=PE0  KEY_4=PE8，四个都是按下为低。
 */
void Key_Init(void);

/**
 * @brief   读取指定按键的当前物理状态（原始电平，未消抖）
 * @param   n: 按键编号 KEY_1 ~ KEY_4
 * @retval  1: 按下（低电平）；0: 释放
 */
uint8_t Key_GetState(uint8_t n);

/**
 * @brief   检查指定按键的某个事件是否发生，并清除相应标志（KEY_HOLD 除外）
 * @param   n: 按键编号 KEY_1 ~ KEY_4
 * @param   Flag: 需要检查的事件标志（如 KEY_SINGLE、KEY_DOUBLE 等）
 * @retval  1: 事件发生；0: 未发生
 * @note    该函数会自动清除除 KEY_HOLD 外的其他标志，避免重复响应。
 */
uint8_t Key_Check(uint8_t n, uint8_t Flag);

/**
 * @brief   按键状态机，需周期性调用（本工程每 5ms 调用一次，即 TIM6 中断）
 * @param   无
 * @retval  无
 * @note    此函数负责按键去抖动、检测按下/释放、识别单击/双击/长按/重复触发等，
 *          并将事件记录在内部标志中，供 Key_Check 查询。
 *          调用周期必须是 5ms —— Key.c 里的时间常数是按 5ms 折算的。
 */
void Key_Tick(void);

// 以下为旧版本接口（已注释，可忽略）
//void Key_Init(void);
//uint8_t Key_GetNum(void);
//void Key_Tick(void);

#endif /* __KEY_H */
