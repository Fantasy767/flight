//防止重复定义
#ifndef __LED_H
#define __LED_H 

#include "main.h"
typedef struct
{
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
} LED_TypeDef;
void LED__Start( LED_TypeDef *led);
void LED__Stop( LED_TypeDef *led);
void LED__Toggle( LED_TypeDef *led);
#endif