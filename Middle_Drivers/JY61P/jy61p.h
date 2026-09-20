#ifndef __JY61P_H
#define __JY61P_H

#include "main.h"

/* 逐字节喂给状态机。由 main.c 的 HAL_UART_RxCpltCallback 调用，
 * 串口接收的启动和重武装都在 main.c，本文件只做协议解析。 */
void jy61p_ReceiveData(uint8_t RxData);

extern volatile float Roll, Pitch, Yaw;         /* 角度 °   */
extern volatile float Ax, Ay, Az;               /* 加速度 g */
extern volatile float Gx, Gy, Gz;               /* 角速度 °/s */

#endif
