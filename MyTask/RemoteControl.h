#ifndef __REMOTECONTROL_H__
#define __REMOTECONTROL_H__

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"


#define CMD_MODE_MOVE               1     //主桅杆上升，舵机操纵主桅杆锁定，允许自由移动
#define CMD_MODE_JUMP               2     //舵机操纵主桅杆取消锁定，主桅杆下降，轮子离地，准备跳跃
#define CMD_MODE_LOCK_CHASSIS       3     //锁定上下小底盘
#define CMD_MODE_UNLOCK_CHASSIS     4     //解除锁定上下小底盘
#define CMD_MODE_LAUNCH             5     ///舵机操纵主桅杆取消锁定，主桅杆和轮子同时触地，同时轮子内八锁定防止车辆发射时旋转


#pragma pack(1)

typedef struct
{
	uint8_t head;
  float v_x;
  float v_y;
  float omega;
  float acc;

  uint32_t cmd;    //附加的命令/参数
} ChassisCtrl_t;
  
typedef struct
{
  uint8_t head;
  float v_x;
  float v_y;
  float omega;

  uint32_t state;
} ChassisState_t;   //底盘状态反馈


#pragma pack()


extern SemaphoreHandle_t upboard_semaphore;

void UartRecvTask(void* parma);

#endif
