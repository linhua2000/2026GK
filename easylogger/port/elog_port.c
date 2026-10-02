/*
 * This file is part of the EasyLogger Library.
 *
 * Copyright (c) 2015, Armink, <armink.ztl@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: Portable interface for STM32F407 + HAL, bare-metal, sync output on USART1.
 */

#include <elog.h>
#include "usart.h"   /* huart1（usart.h 内部已 include main.h，HAL_GetTick 可用） */
#include <stdio.h>   /* snprintf */

/* 256B@115200 ≈ 25.6ms，100ms 留足余量 */
#define ELOG_PORT_UART_TIMEOUT_MS   100U

/**
 * EasyLogger port initialize
 */
ElogErrCode elog_port_init(void) {
    /* USART1 已由 MX_USART1_UART_Init() 初始化，这里无需动作 */
    return ELOG_NO_ERR;
}

/**
 * EasyLogger port deinitialize
 */
void elog_port_deinit(void) {
}

/**
 * output log port interface
 */
void elog_port_output(const char *log, size_t size) {
    /* 同步阻塞发到蓝牙口。本工程没做 printf 重定向，所以直接调 HAL */
    HAL_UART_Transmit(&huart1, (uint8_t *)log, (uint16_t)size, ELOG_PORT_UART_TIMEOUT_MS);
}

/**
 * output lock
 */
void elog_port_output_lock(void) {
    /* 空实现。本工程日志只在主循环调用（单上下文），无并发。
     * 绝不能用 __disable_irq —— 它会被包住整个阻塞发送，导致 USART3/UART4/TIM6
     * 中断丢数据，甚至因 SysTick 冻结让 HAL_UART_Transmit 永久卡死。 */
}

/**
 * output unlock
 */
void elog_port_output_unlock(void) {
}

/**
 * get current time interface（返回静态缓冲，0 点起算的 时:分:秒.毫秒）
 */
const char *elog_port_get_time(void) {
    static char time_str[16];
    uint32_t ms = HAL_GetTick();
    uint32_t s  = ms / 1000U;
    (void)snprintf(time_str, sizeof(time_str), "%02lu:%02lu:%02lu.%03lu",
                   (unsigned long)((s / 3600U) % 100U),
                   (unsigned long)((s / 60U)   % 60U),
                   (unsigned long)(s % 60U),
                   (unsigned long)(ms % 1000U));
    return time_str;
}

/**
 * get current process name interface（裸机无进程概念）
 */
const char *elog_port_get_p_info(void) {
    return "";
}

/**
 * get current thread name interface（裸机无线程概念）
 */
const char *elog_port_get_t_info(void) {
    return "";
}
