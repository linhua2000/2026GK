#ifndef _CONTROL_H
#define _CONTROL_H

#include <stdint.h>

/* ============ 蓝牙 PID 调试口（USART1 = PA9/PA10，115200） ============
 *
 * 下发（ASCII，\n 或 \r 结尾，空格个数不限，不合法就静默丢弃）：
 *   #1 100             第1路目标 = +100（脉冲/5ms）
 *   #3 -50             第3路目标 = -50
 *   #a 100 100 100 100 四路一起，顺序 1,2,3,4
 * 轮号：1=左后 2=左前 3=右前 4=右后（见 分配表.md）
 *
 * 回传（每 20ms 一行，50Hz，主循环里发）：
 *   T1:目标 E1:编码器 P1:PWM  (×4路)
 *   T/E 单位都是脉冲/5ms；P 是比较值，±1050 饱和。
 *
 * Kp/Ki 不在这里调 —— 那是 mailuncontrol.h 里的宏，改完重新烧录。
 * ==================================================================== */

#define DEBUG_WHEEL_NUM   4U

/* 目标值：USART1 中断里解析写入，主循环里回传读取 -> 必须 volatile
 * （-O3 下没有 volatile 会被优化成寄存器缓存，读到旧值） */
extern volatile int32_t Debug_Target[DEBUG_WHEEL_NUM];

/* 四路 PWM 实际输出：TIM6 中断里写，主循环里读 -> 必须 volatile */
extern volatile int32_t Debug_Pwm[DEBUG_WHEEL_NUM];

/* USART1 每收到一字节调一次。由 main.c 的 HAL_UART_RxCpltCallback 调用，
 * 接收的启动与重武装也都在 main.c —— 和 jy61p 一个约定。 */
void Debug_RxByte(uint8_t b);

/* 主循环里调；内部自己计时，够 20ms 就发一行回传。 */
void Debug_Poll(void);

#endif
