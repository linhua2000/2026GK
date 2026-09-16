#include "led.h"

void LED_Init(void)
{
}

void LED_On(uint8_t led_num)
{
    switch(led_num)
    {
        case 1:
            HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET);
            break;
        case 2:
            HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_SET);
            break;
        case 3:
            HAL_GPIO_WritePin(LED3_PORT, LED3_PIN, GPIO_PIN_SET);
            break;
        case 4:
            HAL_GPIO_WritePin(LED4_PORT, LED4_PIN, GPIO_PIN_SET);
            break;
        default:
            break;
    }
}

void LED_Off(uint8_t led_num)
{
    switch(led_num)
    {
        case 1:
            HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET);
            break;
        case 2:
            HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_RESET);
            break;
        case 3:
            HAL_GPIO_WritePin(LED3_PORT, LED3_PIN, GPIO_PIN_RESET);
            break;
        case 4:
            HAL_GPIO_WritePin(LED4_PORT, LED4_PIN, GPIO_PIN_RESET);
        default:
            break;
    }
}

void LED_Toggle(uint8_t led_num)
{
    switch(led_num)
    {
        case 1:
            HAL_GPIO_TogglePin(LED1_PORT, LED1_PIN);
            break;
        case 2:
            HAL_GPIO_TogglePin(LED2_PORT, LED2_PIN);
            break;
        case 3:
            HAL_GPIO_TogglePin(LED3_PORT, LED3_PIN);
            break;
        case 4:
            HAL_GPIO_TogglePin(LED4_PORT, LED4_PIN);
        default:
            break;
    }
}

uint8_t LED_Read(uint8_t led_num)
{
    GPIO_PinState state;
    switch(led_num)
    {
        case 1:
            state = HAL_GPIO_ReadPin(LED1_PORT, LED1_PIN);
            break;
        case 2:
            state = HAL_GPIO_ReadPin(LED2_PORT, LED2_PIN);
            break;
        case 3:
            state = HAL_GPIO_ReadPin(LED3_PORT, LED3_PIN);
            break;
        case 4:
            state = HAL_GPIO_ReadPin(LED4_PORT, LED4_PIN);
            break;
        default:
            return 0;
    }
    return (state == GPIO_PIN_SET) ? 1 : 0;
}
