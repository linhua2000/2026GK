#ifndef __LED_H
#define __LED_H

#include "stm32f4xx_hal.h"

// 定义LED引脚，按需修改
#define LED1_PIN    GPIO_PIN_2
#define LED1_PORT   GPIOB

#define LED2_PIN    GPIO_PIN_0
#define LED2_PORT   GPIOC

#define LED3_PIN    GPIO_PIN_1
#define LED3_PORT   GPIOE

#define LED4_PIN    GPIO_PIN_6
#define LED4_PORT   GPIOD


// 函数声明
void LED_Init(void);                // LED GPIO初始化
void LED_On(uint8_t led_num);       // 点亮LED
void LED_Off(uint8_t led_num);      // 熄灭LED
void LED_Toggle(uint8_t led_num);   // 翻转LED
uint8_t LED_Read(uint8_t led_num);  // 读取LED状态

#endif
