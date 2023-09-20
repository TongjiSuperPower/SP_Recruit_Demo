#include "MotorTask.h"
#include "main.h"
#include "cmsis_os.h"

motor_measure_t motor1;

int16_t current2motor = 500;

void motor_task(void const * argument)
{
		while(1)
		{
				CAN_cmd_motor3508(current2motor, 0, 0, 0);
				HAL_Delay(1);
		}
}

