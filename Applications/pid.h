#ifndef _PID_H
#define _PID_H

#include "struct_typedef.h"

//创建pid数据结构体

typedef struct
{
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;
		
		fp32 error_rad;
		fp32 error_angle;
    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
	
}pid_type_def;

void PID_init(pid_type_def *pid, const fp32 PID[3], fp32 max_out, fp32 max_iout);
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set,fp32 speed);


#endif
