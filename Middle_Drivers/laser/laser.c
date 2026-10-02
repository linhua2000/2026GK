#include "laser.h"
void laser_On(void)
{
    //pc4
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
    
}

void laser_Off(void)
{
    //pc4
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
    
}

void laser_Toggle(void)
{
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
}

