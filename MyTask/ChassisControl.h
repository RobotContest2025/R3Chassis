#ifndef __CHASSISCONTROL_H__
#define __CHASSISCONTROL_H__

#include "stm32f4xx_hal.h"

#include "motor.h"
#include "PID.h"
#include "FreeRTOS.h"
#include "Task.h"
#include "CANDrive.h"
#include "SteeringWheel.h"
#include "WatchDog2.h"

//正方形边长385 mm
//减速比10：3
//轮子直径96mm

#define  L		0.385f
#define VEL_TRANSFORM (10.0f/(3.0f*2.0f*3.14159265f)/0.096f) //将m/s速度转换为电机r/s

void ChassisControl(void* param);

#endif
