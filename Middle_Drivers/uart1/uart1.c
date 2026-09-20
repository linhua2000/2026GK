#include "uart1.h"

void UART1_Init(uint32_t baudrate)
{
//   if (HAL_UART_Init(&huart1) != HAL_OK)
//   {
//     Error_Handler(); // HAL库错误处理，CubeMX自带
//   }
}

// 发送单个字节，阻塞
void UART1_Send_Byte(uint8_t dat)
{
  HAL_UART_Transmit(&huart1, &dat, 1, 100); // 超时100ms
}

// 发送字符串，自动直到'\0'停止
void UART1_Send_Str(uint8_t *str)
{
  while(*str != '\0')
  {
    UART1_Send_Byte(*str++);
  }
}

// 发送指定长度缓冲区
void UART1_Send_Buf(uint8_t *buf, uint16_t len)
{
  HAL_UART_Transmit(&huart1, buf, len, 100);
}
