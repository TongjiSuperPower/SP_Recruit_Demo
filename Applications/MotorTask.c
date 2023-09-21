#include "MotorTask.h"
#include "main.h"
#include "cmsis_os.h"

motor_measure_t motor1;
motor_measure_t motor2;

motor_data_t motor_6020;
motor_data_t motor_3508;

void motor_init(void);
fp32 rad_format(fp32 rad);


int16_t current2motor = 500;

bool_t motor_flag = 0;
int key_delay = 1000;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == KEY_Pin)
	{
		if(key_delay == 0)
		{
		motor_flag = 1;
		key_delay = 1000;
		}
	}
}
	
void motor_task(void const * argument)
{
	vTaskDelay(1000);
	motor_init();
		while(1)
		{
//				CAN_cmd_motor3508(current2motor, 0, 0, 0);
//				HAL_Delay(1);
			if(key_delay > 0)
		key_delay --;
		motor_6020.relative_speed = (motor2.speed_rpm) * 2 * PI / 60;
			
		motor_6020.relative_angle = (float)((int)motor2.ecd - 4096) / 8192 * 2 * PI;

		
			
			
			if(motor_flag)
			{
				motor_6020.target_angle = rad_format(motor_6020.relative_angle + PI / 2);
				motor_3508.target_angle = rad_format(motor_3508.relative_angle + PI / 2);
				motor_flag = 0;
			}
				motor_6020.given_current = PID_calc(&motor_6020.motor_pid, motor_6020.relative_angle, motor_6020.target_angle,motor_6020.relative_speed);
				motor_3508.given_current = PID_calc(&motor_3508.motor_pid, motor_3508.relative_angle, motor_3508.target_angle,motor_3508.relative_speed);

			CAN_cmd_motor6020(motor_6020.given_current,0,0,0);
			CAN_cmd_motor3508(motor_3508.given_current,0,0,0);


					vTaskDelay(1);

		}
}

void motor_init(void)
{
	motor_6020.offecd_ecd = 0;
	motor_3508.offecd_ecd = 0;
	
	motor_3508.relative_speed = 0;
	motor_6020.relative_speed = 0;
	
	motor_3508.round_num = 0;
	motor_6020.round_num = 0;
	
	motor_3508.target_angle = motor_3508.relative_angle;
	motor_6020.target_angle = motor_6020.relative_angle = motor_6020.relative_angle = (float)((motor2.ecd - motor_6020.offecd_ecd) - 4096) / 8192 * 2 * PI;
;

	fp32 pos_pid_3508[3] = {2000,0,-25000};
	fp32 pos_pid_6020[3] = {8000,0,0};
	
	PID_init(&motor_3508.motor_pid,pos_pid_3508,5000,500);
	PID_init(&motor_6020.motor_pid,pos_pid_6020,5000,1000);

}

fp32 rad_format(fp32 rad)
{
	if(rad > PI)
	{
		while(rad > PI)
		{
			rad = rad - 2 * PI;
		}
	}
	else if(rad < -PI)
	{
		while(rad < -PI)
		{
			rad = rad + 2 * PI;
		}
	}
	return rad;
}
