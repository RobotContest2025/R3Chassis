#ifndef __STEERINGWHEEL_H__
#define __STEERINGWHEEL_H__

#include <math.h>

#include "FreeRTOS.h"
#include "Task.h"

#include "PID.h"
#include "motor.h"
#include "ODrive.h"

#define PI 3.1415926536f
#define ANGLE2RAD(x) (x) * PI / 180.0f
#define RAD2ANGLE(x) (x) * 180.0f / PI
#define RADIUS 130.0f

typedef struct
{
  ODrive DriveMotor;           // ODrive~5055电机
  M2006_TypeDef SteeringMotor; // M2006电机
  PID_TypeDef Steering_Dir_PID; // 转向电机PID角度环控制器
  PID_TypeDef Steering_Vel_PID; // 转向电机PID速度环控制器

  float currentDirection;
  float expectDirection;
  float expextVelocity;
  float putoutDirection;
  float putoutVelocity;
  float last_expDirection;

  float postureAngle;     // 安装位置与重心点连线与车辆正方向的夹角(0~360)
  float postureRadius;    // 舵轮到中心的距离
  float offset;           // 2006电机起始误差
  float addoffsetangle;   // 2006电机安装误差
  float maxRotateAngle;   // 最大正反转度数
  float floatRotateAngle; // 柔性区间度数（防止因目标值在机械角度限制附近振动导致输出震荡，应当略小于maxRotateAngle大约10~20度）

  GPIO_TypeDef *Key_GPIO_Port; // 光电门引脚端口
  uint16_t Key_GPIO_Pin;       // 光电门引脚号
  uint8_t ready_edge_flag;     // 舵轮处于奇点附近，舵轮已经复位（0b G000 HIJK）
} SteeringWheel;

void SteeringWheel_Deal(SteeringWheel *pSteWhe, float vel, float direction, float omega);

void LimitAngle(float *angle);
float AngleDiffer(float angle1, float angle2);
void CalcWheelSpeed1(SteeringWheel *pSteWhe, float vel, float direction, float omega);
void CalcWheelSpeed2(SteeringWheel *pSteWhe, float x, float y, float omega);
void MinorArcDeal2(SteeringWheel *motor);
uint8_t SteeringWheelReady(SteeringWheel *StrWhe);

#endif
