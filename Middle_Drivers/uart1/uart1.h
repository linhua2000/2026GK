#ifndef __UART1_H
#define __UART1_H

#include "stm32f4xx_hal.h"

// 串口句柄，外部可用
extern UART_HandleTypeDef huart1;

// 函数声明
void UART1_Init(uint32_t baudrate);       // USART1初始化，传入波特率
void UART1_Send_Byte(uint8_t dat);        // 发送单个字节
void UART1_Send_Str(uint8_t *str);        // 发送字符串（以'\0'结尾）
void UART1_Send_Buf(uint8_t *buf, uint16_t len); // 指定长度缓冲区发送

#endif
