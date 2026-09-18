#ifndef __RECEIVE_H
#define __RECEIVE_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ===================== 视觉协议帧定义 (int8, 小端) =====================
 * 所有数据字段都是单字节 int8, 帧头帧尾固定:
 *
 *   二维码识别 : A5  x  y  z  5A   (5 字节) -> qr_x / qr_y / qr_z
 *   循迹       : B6  x  y      6B   (4 字节) -> track_x / track_y
 *   转弯       : C7  0          7C   (3 字节) -> turn_flag
 *   抓取       : D8  x  y      D8   (4 字节) -> grab_x / grab_y
 *
 * 注: 单字节数据不存在字节序问题; 若日后某字段扩展为多字节, 按小端解析。
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
#define FRAME_GRAB_LEN    4

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
    /* 抓取 D8 x y 8D */
    int8_t grab_x;
    int8_t grab_y;
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

#endif /* __RECEIVE_H */
