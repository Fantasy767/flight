#include "pid.h"
void pid_base(PID *pid)
{
    pid->err=pid->measure-pid->target;
    pid->err_sum+=pid->err;
    if(pid->err_last==0)
    {
        pid->err_last=pid->err;
    }
    float err_diff=pid->err-pid->err_last;
    pid->out=pid->kp*pid->err+
    (pid->ki*pid->err_sum*PID_PERIOD)+
    (pid->kd*err_diff/PID_PERIOD);
    pid->err_last=pid->err;

}
void pid_chain(PID *out_pid,PID *in_pid)
{
    pid_base(out_pid);
    in_pid->target=out_pid->out;
    pid_base(in_pid);
}

