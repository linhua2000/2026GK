#ifndef __RECEIVE_H
#define __RECEIVE_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ===================== 视觉协议帧定义 =====================
 * 帧头帧尾固定, 数据字段按小端解析:
 *
 *   二维码识别 : A5  x  y  z      5A   (5 字节) -> qr_x / qr_y / qr_z (int8)
 *   循迹       : B6  x  y         6B   (4 字节) -> track_x / track_y (int8)
 *   转弯       : C7  0            7C   (3 字节) -> turn_flag
 *   抓取       : D8  xL xH yL yH  8D   (6 字节) -> grab_x / grab_y (int16, 小端)
 *
 * 注: 抓取帧坐标为 int16(2字节, 低字节在前); 其余字段为单字节 int8。
 */
#define FRAME_QR_HEAD     0xA5
#define FRAME_QR_TAIL     0x5A
#define FRAME_QR_LEN      5

#define FRAME_TRACK_HEAD  0xB6
#define FRAME_TRACK_TAIL  0x6B
#define FRAME_TRACK_LEN   4

#define FRAME_TURN_HEAD   0xC7
#define FRAME_TURN_TAIL   0x7C
#define FRAME_TURN_LEN    3

#define FRAME_GRAB_HEAD   0xD8
#define FRAME_GRAB_TAIL   0x8D
#define FRAME_GRAB_LEN    6   /* D8 + x(int16) + y(int16) + 8D */

/* 所有帧中最大的字节数, 用于接收缓冲 */
#define FRAME_MAX_LEN     6

/* 半包超时时间(ms), 超过则认为之前的半截包作废 */
#define VISION_RX_TIMEOUT 50

/* 视觉接收数据。
 * 收到对应帧后由状态机填入, 并把对应的 flag 置 1;
 * 任务读取完数据后自行把 flag 清零。 */
typedef struct {
    /* 二维码 A5 x y z 5A */
    int8_t qr_x;
    int8_t qr_y;
    int8_t qr_z;
    /* 循迹 B6 x y 6B */
    int8_t track_x;
    int8_t track_y;
    /* 抓取 D8 x(int16,小端) y(int16,小端) 8D */
    int16_t grab_x;
    int16_t grab_y;
    /* 新帧标志 */
    uint8_t qr_flag;      /* 收到二维码帧 */
    uint8_t track_flag;   /* 收到循迹帧 */
    uint8_t turn_flag;    /* 收到转弯帧 C7 0 7C, 需要转弯 */
    uint8_t grab_flag;    /* 收到抓取帧 */
} VisionData_t;

extern VisionData_t vision_data;

/* 启动视觉串口接收中断, 主程序初始化时调用一次 */
void Vision_UART_Init(void);

/* 收到一帧后回一个字节 0x01 给视觉, 用于测试确认 */
void Vision_Send_Ack(void);

/* 抓完球转身后, 通知视觉切桶识别: D8 01 8D */
void Vision_Send_Switch_Bucket(void);

/* 放完球, 通知视觉完成: D8 02 8D */
void Vision_Send_Release_Done(void);

#endif /* __RECEIVE_H */
