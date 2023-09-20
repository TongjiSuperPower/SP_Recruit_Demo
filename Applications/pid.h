#ifndef _PID_H
#define _PID_H

#include "struct_typedef.h"

//����pid���ݽṹ��

typedef struct
{
    //PID ������
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;
		
		fp32 error_rad;
		fp32 error_angle;
    fp32 max_out;  //������
    fp32 max_iout; //���������

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    fp32 error[3]; //����� 0���� 1��һ�� 2���ϴ�

/***�������***/	
		fp32 dT;
		fp32 max_dout;
	
}pid_type_def;

void PID_init(pid_type_def *pid, const fp32 PID[3], fp32 max_out, fp32 max_iout);
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set,fp32 speed);


#endif
