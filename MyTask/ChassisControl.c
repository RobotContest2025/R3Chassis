#include "ChassisControl.h"
#include "RemoteControl.h"
#include "pid_old.h"
#include "usart.h"

SteeringWheel steering1, steering2, steering3, steering4;
int16_t motorCurrentBuf[4] = {0};
int16_t motorCurrentBuf2[4] = {0};
float velocity = 0.0f, direction = 0.0f, omega = 0.0f;

float dir1,dir2,dir3,dir4,vel;
uint8_t forced_set_dir=0;

M2006_TypeDef lift_motor1;
M2006_TypeDef lift_motor2;
PID2 lift_motor1_vel_pid;
PID2 lift_motor2_vel_pid;
float lift_motor1_target_vel=0.0f;
float lift_motor2_target_vel=0.0f;

ChassisState_t chassis_state = {.head=0x1B};

static float v[4];        // 轮子线速度
static float theta[4];    // 角度转为弧度
static float pos[4][2];   //四个轮子的坐标

void ChassisControl(void *param)
{
	lift_motor1_vel_pid.Kp = 8.0f;
	lift_motor1_vel_pid.Ki = 0.3f;
	lift_motor1_vel_pid.Kd = 0.0f;
	lift_motor1_vel_pid.limit = 10000.0f;
	lift_motor1_vel_pid.output_limit = 10000.0f;
	lift_motor2_vel_pid=lift_motor1_vel_pid;
	
  // 舵轮通用参数
  steering1.Steering_Vel_PID.Kp = 10.0f; // 设置速度环PID
  steering1.Steering_Vel_PID.Ki = 0.3f;
  steering1.Steering_Vel_PID.Kd = 0.0f;
  steering1.Steering_Vel_PID.DeadBand = 0.0f;
  steering1.Steering_Vel_PID.MaxOut = 10000.0f;
  steering1.Steering_Vel_PID.IntegralLimit = 10000.0f;

  steering1.Steering_Dir_PID.Kp = 200.0f; // 设置角度环PID
  steering1.Steering_Dir_PID.Ki = 0.0f;
  steering1.Steering_Dir_PID.Kd = 3.3f;
  steering1.Steering_Dir_PID.DeadBand = 0.0f;
  steering1.Steering_Dir_PID.MaxOut = 10000.0f;
  steering1.Steering_Dir_PID.IntegralLimit = 10.0f;

  steering1.offset = 0.0f; // 稍后由复位程序重新设置
  steering1.postureRadius=0.385*1.414f; //轮子到几何中心的距离
  steering1.maxRotateAngle = 170.0f;    //机械限位角度
  steering1.floatRotateAngle = 160.0f; // 柔性区间大小为10度
  steering1.ready_edge_flag = 0;
  steering1.DriveMotor.hcan = &hcan1;   //使用CAN1总线

  steering4 = steering3 = steering2 = steering1;  //参数拷贝

  // 舵轮个性化参数
  steering1.addoffsetangle = 0.0f;
  steering1.postureAngle = 135.0f;
  steering1.Key_GPIO_Port = GPIOA;
  steering1.Key_GPIO_Pin = GPIO_PIN_4;
  steering1.DriveMotor.motorID = 0x00;

  steering2.addoffsetangle = 0.0f;
  steering2.postureAngle = 45.0f;
  steering2.Key_GPIO_Port = GPIOA;
  steering2.Key_GPIO_Pin = GPIO_PIN_5;
  steering2.DriveMotor.motorID = 0x10;

  steering3.addoffsetangle = 0.0f;
  steering3.postureAngle = -135.0f;
  steering3.Key_GPIO_Port = GPIOA;
  steering3.Key_GPIO_Pin = GPIO_PIN_6;
  steering3.DriveMotor.motorID = 0x20;

  steering4.addoffsetangle = 0.0f;
  steering4.postureAngle = -45.0f;
  steering4.Key_GPIO_Port = GPIOC;
  steering4.Key_GPIO_Pin = GPIO_PIN_4;
  steering4.DriveMotor.motorID = 0x30;

  //填写轮子坐标
  pos[0][0]=-L/2;
  pos[0][1]=L/2;
  pos[1][0]=L/2;
  pos[1][1]=L/2;
  pos[2][0]=-L/2;
  pos[2][1]=-L/2;
  pos[3][0]=L/2;
  pos[3][1]=-L/2;

	vTaskDelay(pdMS_TO_TICKS(5000));
  TickType_t last_wake_time = xTaskGetTickCount();
	
	//steering1.ready_edge_flag=0x80;
	//steering2.ready_edge_flag=0x80;
	//steering3.ready_edge_flag=0x80;
	//steering4.ready_edge_flag=0x80;
	
  while (1)
  {
		if(!forced_set_dir)
		{
			dir1=dir2=dir3=dir4=direction;
			vel=velocity;
		}
		
    SteeringWheel_Deal(&steering1, vel, dir1, omega);
    SteeringWheel_Deal(&steering2, vel, dir2, omega);
    SteeringWheel_Deal(&steering3, vel, dir3, omega);
    SteeringWheel_Deal(&steering4, vel, dir4, omega);
		
    motorCurrentBuf[0] = steering1.Steering_Vel_PID.Output;
    motorCurrentBuf[1] = steering2.Steering_Vel_PID.Output;
    motorCurrentBuf[2] = steering3.Steering_Vel_PID.Output;
    motorCurrentBuf[3] = steering4.Steering_Vel_PID.Output;
    MotorSend(&hcan2, 0x200, motorCurrentBuf); // C610电调控制-进行转向控制
		
		PID_Control2(lift_motor1.Speed,lift_motor1_target_vel,&lift_motor1_vel_pid);
		PID_Control2(lift_motor2.Speed,lift_motor2_target_vel,&lift_motor2_vel_pid);
		motorCurrentBuf2[0]=lift_motor1_vel_pid.pid_out;
		motorCurrentBuf2[1]=lift_motor2_vel_pid.pid_out;
		MotorSend(&hcan2, 0x1FF, motorCurrentBuf2);	//C610电调控制-桅杆抬升电机
		
    ODriveSetVelocity(&steering1.DriveMotor, steering1.putoutVelocity*VEL_TRANSFORM, 0.0f);
    ODriveSetVelocity(&steering2.DriveMotor, steering2.putoutVelocity*VEL_TRANSFORM, 0.0f);
    vTaskDelay(1);
    ODriveSetVelocity(&steering3.DriveMotor, steering3.putoutVelocity*VEL_TRANSFORM, 0.0f);
    ODriveSetVelocity(&steering4.DriveMotor, -steering4.putoutVelocity*VEL_TRANSFORM, 0.0f);
    ODriveGetEncoderEstimate(&steering1.DriveMotor);
    vTaskDelay(1);
    ODriveGetEncoderEstimate(&steering2.DriveMotor);
    ODriveGetEncoderEstimate(&steering3.DriveMotor);
    ODriveGetEncoderEstimate(&steering4.DriveMotor);
		vTaskDelay(1);

    
    v[0]=-steering1.DriveMotor.posVelEstimateGet.velocity/VEL_TRANSFORM; // 转速转为线速度
    v[1]=steering2.DriveMotor.posVelEstimateGet.velocity/VEL_TRANSFORM; // 转速转为线速度
    v[2]=steering3.DriveMotor.posVelEstimateGet.velocity/VEL_TRANSFORM; // 转速转为线速度
    v[3]=-steering4.DriveMotor.posVelEstimateGet.velocity/VEL_TRANSFORM; // 转速转为线速度

    theta[0]=steering1.currentDirection*PI/180.0f; //获得轮子的角度（弧度）
    theta[1]=steering2.currentDirection*PI/180.0f;
    theta[2]=steering3.currentDirection*PI/180.0f;
    theta[3]=steering4.currentDirection*PI/180.0f;

    //TODO:运动学反解
    float  A[4][3] = {0};
    float b[4] = {0};
    float ATA[3][3] = {0};

    for (int i = 0; i < 4; i++) {     //构建矩阵A和向量b
        float cos_t = cos(theta[i]);
        float sin_t = sin(theta[i]);
        A[i][0] = cos_t;
        A[i][1] = sin_t;
        A[i][2] = -sin_t * pos[i][1] + cos_t * pos[i][0];
        b[i] = v[i];
    }

    for (int i = 0; i < 3; i++) {     // 行
        for (int j = 0; j < 3; j++) { // 列
            for (int k = 0; k < 4; k++) {
                ATA[i][j] += A[k][i] * A[k][j];
            }
        }
    }

    // 计算 A^T * b （3x1 向量）
    float ATb[3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int k = 0; k < 4; k++) {
            ATb[i] += A[k][i] * b[k];
        }
    }

    float detA =
        ATA[0][0]*(ATA[1][1]*ATA[2][2] - ATA[1][2]*ATA[2][1]) -
        ATA[0][1]*(ATA[1][0]*ATA[2][2] - ATA[1][2]*ATA[2][0]) +
        ATA[0][2]*(ATA[1][0]*ATA[2][1] - ATA[1][1]*ATA[2][0]);

        chassis_state.v_x=
        ATb[0]*(ATA[1][1]*ATA[2][2] - ATA[1][2]*ATA[2][1]) -
        ATA[0][1]*(ATb[1]*ATA[2][2] - ATA[1][2]*ATb[2]) +
        ATA[0][2]*(ATb[1]*ATA[2][1] - ATA[1][1]*ATb[2]);

        chassis_state.v_y =
        ATA[0][0]*(ATb[1]*ATA[2][2] - ATA[1][2]*ATb[2]) -
        ATb[0]*(ATA[1][0]*ATA[2][2] - ATA[1][2]*ATA[2][0]) +
        ATA[0][2]*(ATA[1][0]*ATb[2] - ATb[1]*ATA[2][0]);

        chassis_state.omega =
        ATA[0][0]*(ATA[1][1]*ATb[2] - ATb[1]*ATA[2][1]) -
        ATA[0][1]*(ATA[1][0]*ATb[2] - ATb[1]*ATA[2][0]) +
        ATb[0]*(ATA[1][0]*ATA[2][1] - ATA[1][1]*ATA[2][0]);


		HAL_UART_Transmit_DMA(&huart4,(uint8_t *)&chassis_state,sizeof(chassis_state));
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5));
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) // 接收oDrive的反馈
{
  if (hcan->Instance == CAN1)
  {
    uint8_t Recv[8] = {0};
    uint32_t ID = CAN_Receive_DataFrame(hcan, Recv);
		ODriveRecvServe(&steering1.DriveMotor, ID, Recv);
    ODriveRecvServe(&steering2.DriveMotor, ID, Recv);
    ODriveRecvServe(&steering3.DriveMotor, ID, Recv);
    ODriveRecvServe(&steering4.DriveMotor, ID, Recv);
  }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) // 接收2006的反馈
{
  if (hcan->Instance == CAN2)
  {
    uint8_t Recv[8] = {0};
    uint32_t ID = CAN_Receive_DataFrame(hcan, Recv);
    if (ID == 0x201) // 左上，象限2
    {
      M2006_Receive(&steering1.SteeringMotor, Recv);
    }
    else if (ID == 0x202) // 右上(象限1)
    {
      M2006_Receive(&steering2.SteeringMotor, Recv);
    }
    else if (ID == 0x203) // 左下(象限3)
    {
      M2006_Receive(&steering3.SteeringMotor, Recv);
    }
    else if (ID == 0x204) // 右下(象限4)
    {
      M2006_Receive(&steering4.SteeringMotor, Recv);
    }
		else if(ID==0x205)		//桅杆抬升电机
		{
			M2006_Receive(&lift_motor1, Recv);
		}
		else if (ID == 0x206)
    {
      M2006_Receive(&lift_motor2, Recv);
    } 
  }
}
