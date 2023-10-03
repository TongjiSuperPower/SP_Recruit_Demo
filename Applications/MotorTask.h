#ifndef _MOTORTASK_H_
#define _MOTORTASK_H_

#include "CAN_receive.h"
#include "pid.h"
extern motor_measure_t motor1;
extern motor_measure_t motor2;

#define PI 3.14159265358979f

typedef struct
{
    int32_t round_num;
    fp32 relative_angle;
		fp32 relative_speed;
		uint16_t offecd_ecd;
		fp32 target_angle;
		pid_type_def motor_pid;
		fp32 given_current;
} motor_data_t;


extern motor_data_t motor_6020;
extern motor_data_t motor_3508;
extern bool_t motor_flag;
fp32 rad_format(fp32 rad);

#endif

