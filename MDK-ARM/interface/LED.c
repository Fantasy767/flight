#include "LED.h"

void LED__Start( LED_TypeDef *led)
{
    HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_RESET);
}

void LED__Stop( LED_TypeDef *led)
{
    HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_SET);
}
void LED__Toggle( LED_TypeDef *led)
{
    HAL_GPIO_TogglePin(led->GPIOx, led->GPIO_Pin);
}