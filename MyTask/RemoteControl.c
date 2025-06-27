#include "RemoteControl.h"
#include "ChassisControl.h"
#include "ODrive.h"
#include "tim.h"
#include "semphr.h"
#include "Action.h"
#include <string.h>
#include <math.h>

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;

extern QueueHandle_t launch_action_semphr;
extern QueueHandle_t move_action_semphr;
extern QueueHandle_t jump_action_semphr;

extern float velocity,direction,omega;

uint8_t chassis_ctrl_recv[sizeof(ChassisCtrl_t)];
ChassisCtrl_t chassis_ctrl,last_chassis_ctrl;


SemaphoreHandle_t upboard_semaphore;

//上下底盘锁定舵机
float unlock_angle1=195.0f;
float unlock_angle2=230.0f;
float lock_angle1=175.0f;
float lock_angle2=210.0f;

float debug1=0.0f;
float debug2=0.0f;

uint8_t state=1;
extern uint8_t forced_set_dir;


void UartRecvTask(void* parma)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart4,chassis_ctrl_recv,sizeof(chassis_ctrl_recv));
	upboard_semaphore=xSemaphoreCreateBinary();
	while(1)
	{
		if(xSemaphoreTake(upboard_semaphore,pdMS_TO_TICKS(100))!=pdTRUE)
		{
			velocity=0.0f;	//上位机掉线，底盘速度归0
			omega=0.0f;
		}
		velocity=sqrt(chassis_ctrl.v_x*chassis_ctrl.v_x+chassis_ctrl.v_y*chassis_ctrl.v_y);		//运动学部分
		if(velocity>0.00001f)
			direction=RAD2ANGLE(atan2f(-chassis_ctrl.v_y,-chassis_ctrl.v_x));
		omega=chassis_ctrl.omega;
		
		//命令执行
		if(chassis_ctrl.cmd==CMD_MODE_MOVE&&state==1)
		{
			xSemaphoreGive(move_action_semphr);
		}
		else if(chassis_ctrl.cmd==CMD_MODE_JUMP&&state==2)
		{
			xSemaphoreGive(jump_action_semphr);
		}
		else if(chassis_ctrl.cmd==CMD_MODE_LAUNCH&&forced_set_dir==0)
		{
			xSemaphoreGive(launch_action_semphr);
		}
		else if(chassis_ctrl.cmd==CMD_MODE_LOCK_CHASSIS)
		{
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,SetSteeringEngineRAD270(lock_angle1*3.14159265f/180.0f));
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,SetSteeringEngineRAD270(lock_angle2*3.14159265f/180.0f));
		}
		else if(chassis_ctrl.cmd==CMD_MODE_UNLOCK_CHASSIS)
		{
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,SetSteeringEngineRAD270(unlock_angle1*3.14159265f/180.0f));
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,SetSteeringEngineRAD270(unlock_angle2*3.14159265f/180.0f));
		}
		else if(chassis_ctrl.cmd==10)
		{
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,SetSteeringEngineRAD270(debug1*3.14159265f/180.0f));
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,SetSteeringEngineRAD270(debug2*3.14159265f/180.0f));
		}
		//TODO:填写并发送底盘反馈
		
		last_chassis_ctrl=chassis_ctrl;
	}
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance==UART4)	//来自上板的控制信息接收
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart4, chassis_ctrl_recv, sizeof(ChassisCtrl_t));
		if(chassis_ctrl_recv[0]!=0x5A)		//没有通过包头校验
			return;
		memcpy(&chassis_ctrl,chassis_ctrl_recv,sizeof(ChassisCtrl_t));
		BaseType_t temp;
		xSemaphoreGiveFromISR(upboard_semaphore,&temp);
		portYIELD_FROM_ISR(temp);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)	//收到错误帧处理
{
	if(huart->Instance==UART4)
	{
		HAL_UART_DMAStop(huart);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart4,chassis_ctrl_recv,sizeof(chassis_ctrl_recv));
	}
}
