#include"motor.h"
void motor_speed_set(motor_struct *motor)
{
    if(motor->speed > 1000)
    {
        motor->speed = 1000;
    }
    else if(motor->speed < 0)
    {
        //下限必须到 0：失控/油门归零时要能真正停电机
        motor->speed = 0;
    }
    __HAL_TIM_SET_COMPARE(motor->tim, motor->channel, motor->speed);
}
void motor_start(motor_struct *motor)
{
    HAL_TIM_PWM_Start(motor->tim, motor->channel);
}