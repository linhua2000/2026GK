/*
 * uart.c
 * UART接口
 * 日期: 2019.9.7
 * 作者: 
 */

#include <stdio.h>
#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_hal.h"
//UART 读数据缓冲区
__IO uint8_t uartBuf[128];
__IO int head = 0;
__IO int tail  = 0;

void Uart_Flush(void)
{
	head = tail = 0;
}

int16_t Uart_Read(void)
{
	if(head!=tail){
		uint8_t Data = uartBuf[head];
		head =  (head+1)%128;
		return Data;
	}else{
		return -1;
	}
}
/*---------------
舵机走 USART2 (PA2 TX / PA3 RX)
注意: UART4 (PC10/PC11) 是视觉串口, 不要搞混
------------------*/

extern UART_HandleTypeDef huart2;//舵机走 USART2

#define UART_TX_TIMEOUT_MS 20U

/**
  * @brief UART 接收中断服务程序
  * 说明：当USART2接收到数据时会触发此中断服务程序，程序会将接收到的数据存入uartBuf缓冲区，并更新tail指针。用户可以通过调用Uart_Read函数从缓冲区中读取数据。需要注意的是，缓冲区大小为128字节，如果接收数据过快可能会导致数据丢失，因此建议在使用时合理设置波特率和处理接收数据的速度。   
  */

void Uart_Send(uint8_t *buf , uint8_t len)
{
	if (buf == NULL || len == 0U)
	{
		return;
	}

	HAL_UART_Transmit(&huart2, buf, len, UART_TX_TIMEOUT_MS);
}


