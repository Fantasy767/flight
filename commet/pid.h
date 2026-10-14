#ifndef __PID__
#define __PID__
#define PID_PERIOD 0.01
typedef struct
{
    float kp;
    float ki;//解决稳态误差 
    float kd;//解决过度调整
    float err;
    float target;
    float measure;//测量值
    float err_last;
    float err_sum;//积分累计
    float out;
}PID;
void pid_base(PID *pid);
void pid_chain(PID *out_pid,PID *in_pid);

#endif