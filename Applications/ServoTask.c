#include "ServoTask.h"
#include "main.h"
#include "cmsis_os.h"
#include "tim.h"

int servo_flag = 0;

void servo_init()
{

	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,100);
	osDelay(1000);
}

void Servo_Task(void const * argument)
{
//	vTaskDelay(SERVO_TASK_INIT_TIME);
	servo_init();
	while(1)
	{	
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,75);
		osDelay (1000);
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,125);
		osDelay (1000);
//		osDelay(SERVO_CONTROL_TIME_MS);
	}
	
}





