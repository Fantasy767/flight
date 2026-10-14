//防止重复定义
#ifndef __MOTOR_H
#define __MOTOR_H
#include "tim.h"
typedef struct
{
    TIM_HandleTypeDef *tim;
    uint16_t channel;
    int16_t speed;
} motor_struct;
//传入的参数是比较值，max=1000，min=0（0 才能真正停电机）
void motor_speed_set(motor_struct *motor);
void motor_start(motor_struct *motor);
#endif