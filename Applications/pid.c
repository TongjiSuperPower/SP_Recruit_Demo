#include "pid.h"
#include "stdlib.h"
#include "MotorTask.h"
void LimitMax(fp32 *input, fp32 max)
{
	if(*input > max)
	{
		*input = max;
	}
	else if(*input < -max)
	{
		*input = -max;
	}
}

//pid参数初始化
void PID_init(pid_type_def *pid, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

//pid控制函数
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set,fp32 speed)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = rad_format(set - ref);
		pid->error_rad = pid->error[0];
		pid->error_angle = pid->error_rad * 180 / PI;
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dout = pid->Kd * speed;
        LimitMax(&pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(&pid->out, pid->max_out);
    return pid->out;
}
