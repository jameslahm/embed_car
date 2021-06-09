/**********************************************************************
版权所有：	  喵呜实验室MiaowLabs，2017.
官		网：	http://www.miaowlabs.com
淘		宝：	https://miaowlabs.taobao.com/
文 件 名: 	  control.c
作    者:   喵呜实验室MiaowLabs
版		本:   3.00
完成日期:   2017.03.01
概		要:


***********************************************************************/
#include "control.h"
#include "MPU6050.H"
#include "bsp.h"
#include "communicate.h"
#include "debug.H"
#include "infrare.h"
#include "math.h"
#include "stdio.h"
#include "ultrasonic.h"
#include "usart.h"

unsigned char g_u8MainEventCount;
unsigned char g_u8SpeedControlCount;
unsigned char g_u8SpeedControlPeriod;
unsigned char g_u8DirectionControlPeriod;
unsigned char g_u8DirectionControlCount;

unsigned char g_cMotorDisable = 0; //值等于0时电机正常转动，否则停止转动

int g_iGravity_Offset = 0;

/******电机控制参数******/
float g_fSpeedControlOut;
float g_fSpeedControlOutOld;
float g_fSpeedControlOutNew;
float g_fAngleControlOut;
float g_fLeftMotorOut;
float g_fRightMotorOut;
float g_fxyAngleControlOut;
/******速度控制参数******/

short g_s16LeftMotorPulse;
short g_s16RightMotorPulse;

int g_s32LeftMotorPulseOld;
int g_s32RightMotorPulseOld;
int g_s32LeftMotorPulseSigma;
int g_s32RightMotorPulseSigma;

float g_fCarSpeed;
float g_iCarSpeedSet;
float g_fCarSpeedOld;
float g_fCarPosition;

/*-----角度环和速度环PID控制参数-----*/
PID_t g_tCarAnglePID = {17.0, 0, 23.0};  //*5 /10
PID_t g_tCarSpeedPID = {15.25, 1.08, 0}; // i/10
PID_t g_txyAnglePID = {0, 0, 30.0};
/******蓝牙控制参数******/
float g_fBluetoothSpeed;
float g_fBluetoothDirection;
float g_fBluetoothDirectionOld;
float g_fBluetoothDirectionNew;
float g_fBluetoothDirectionOut;

float g_fCarAngle;       //
float g_fGyroAngleSpeed; //
float g_fGravityAngle;   //
float g_fxyAngle;
float g_fYawAngle;
float g_fGyroAngleSpeed_z;

int g_iLeftTurnRoundCnt = 0;
int g_iRightTurnRoundCnt = 0;

static int AbnormalSpinFlag = 0;

int g_fSpeedDelta = 0;
int g_fForwardStatus = 1;
float speed_old_2 = 0;

/***************************************************************
** 函数名称: CarUpstandInit
** 功能描述: 全局变量初始化函数
** 输　入:
** 输　出:
** 全局变量:
** 作　者:   喵呜实验室MiaowLabs
** 淘  宝：  https://miaowlabs.taobao.com/
** 日　期:   2014年08月01日
***************************************************************/
void CarUpstandInit(void)
{
  // g_iAccelInputVoltage_X_Axis = g_iGyroInputVoltage_Y_Axis = 0;
  g_s16LeftMotorPulse = g_s16RightMotorPulse = 0;
  g_s32LeftMotorPulseOld = g_s32RightMotorPulseOld = 0;
  g_s32LeftMotorPulseSigma = g_s32RightMotorPulseSigma = 0;

  g_fCarSpeed = g_fCarSpeedOld = 0;
  g_fCarPosition = 0;
  g_fCarAngle = 0;
  g_fGyroAngleSpeed = 0;
  g_fGravityAngle = 0;

  g_fAngleControlOut = g_fSpeedControlOut = g_fBluetoothDirectionOut = 0;
  g_fLeftMotorOut = g_fRightMotorOut = 0;
  g_fBluetoothSpeed = g_fBluetoothDirection = 0;
  g_fBluetoothDirectionNew = g_fBluetoothDirectionOld = 0;

  g_u8MainEventCount = 0;
  g_u8SpeedControlCount = 0;
  g_u8SpeedControlPeriod = 0;

  g_fxyAngle = 0;
  g_fYawAngle = 0;
  g_fGyroAngleSpeed_z = 0;
}

/***************************************************************
** 函数名称: AbnormalSpinDetect
** 功能描述: 电机转速异常检测
** 输　入:
** 输　出:
** 全局变量:
** 作　者:   喵呜实验室MiaowLabs
** 日　期:   2017年4月26日
***************************************************************/

void AbnormalSpinDetect(short leftSpeed, short rightSpeed)
{
  static unsigned short count = 0;

  //速度设置为0时检测，否则不检测
  if (g_iCarSpeedSet == 0)
  {
    if (((leftSpeed > 30) && (rightSpeed > 30) && (g_fCarAngle > -30) &&
         (g_fCarAngle < 30)) ||
        ((leftSpeed < -30) && (rightSpeed < -30)) && (g_fCarAngle > -30) &&
            (g_fCarAngle <
             30))
    { // 左右电机转速大于30、方向相同、持续时间超过250ms，且车身角度不超过30度，则判断为悬空空转
      count++;
      if (count > 50)
      {
        count = 0;
        AbnormalSpinFlag = 1;
      }
    }
    else
    {
      count = 0;
    }
  }
  else
  {
    count = 0;
  }
}

/***************************************************************
** 函数名称: LandingDetect
** 功能描述: 小车着地检测
** 输　入:
** 输　出:
** 全局变量:
** 作　者:   喵呜实验室MiaowLabs
** 日　期:   2017年4月26日
***************************************************************/
void LandingDetect(void)
{
  static float lastCarAngle = 0;
  static unsigned short count = 0, count1 = 0;

  if (AbnormalSpinFlag == 0)
    return;

  // 小车角度5°~-5°启动检测
  if ((g_fCarAngle > -5) && (g_fCarAngle < 5))
  {
    count1++;
    if (count1 >=
        50)
    { //每隔250ms判断一次小车角度变化量，变化量小于0.8°或大于-0.8°判断为小车静止
      count1 = 0;
      if (((g_fCarAngle - lastCarAngle) < 0.8) &&
          ((g_fCarAngle - lastCarAngle) > -0.8))
      {
        count++;
        if (count >= 4)
        {
          count = 0;
          count1 = 0;
          g_fCarPosition = 0;
          AbnormalSpinFlag = 0;
        }
      }
      else
      {
        count = 0;
      }
      lastCarAngle = g_fCarAngle;
    }
  }
  else
  {
    count1 = 0;
    count = 0;
  }
}

/***************************************************************
** 函数名称: MotorManage
** 功能描述: 电机使能/失能控制
** 输　入:
** 输　出:
** 全局变量:
** 作　者:   喵呜实验室MiaowLabs
** 日　期:   2017年4月26日
***************************************************************/
void MotorManage(void)
{

  AbnormalSpinDetect(g_s16LeftMotorPulse, g_s16RightMotorPulse);

  LandingDetect();

  if (AbnormalSpinFlag)
  {
    g_cMotorDisable |= (0x01 << 1);
  }
  else
  {
    g_cMotorDisable &= ~(0x01 << 1);
  }

  if (g_fCarAngle > 30 || g_fCarAngle < (-30))
  {
    g_cMotorDisable |= (0x01 << 2);
  }
  else
  {
    g_cMotorDisable &= ~(0x01 << 2);
  }
}

/***************************************************************
** 函数名称: SetMotorVoltageAndDirection
** 功能描述: 电机转速及方向控制函数
** 输　入:
** 输　出:
** 全局变量:
** 作　者:   喵呜实验室MiaowLabs
** 淘  宝：  https://miaowlabs.taobao.com/
** 日　期:   2018年08月27日
***************************************************************/
void SetMotorVoltageAndDirection(int i16LeftVoltage, int i16RightVoltage)
{
  if (i16LeftVoltage < 0)
  {
    GPIO_SetBits(GPIOA, GPIO_Pin_3);
    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    i16LeftVoltage = (-i16LeftVoltage);
  }
  else
  {
    GPIO_SetBits(GPIOA, GPIO_Pin_4);
    GPIO_ResetBits(GPIOA, GPIO_Pin_3);
  }

  if (i16RightVoltage < 0)
  {
    GPIO_SetBits(GPIOB, GPIO_Pin_0);
    GPIO_ResetBits(GPIOB, GPIO_Pin_1);
    i16RightVoltage = (-i16RightVoltage);
  }
  else
  {
    GPIO_SetBits(GPIOB, GPIO_Pin_1);
    GPIO_ResetBits(GPIOB, GPIO_Pin_0);
  }

  if (i16RightVoltage > MOTOR_OUT_MAX)
  {
    i16RightVoltage = MOTOR_OUT_MAX;
  }
  if (i16LeftVoltage > MOTOR_OUT_MAX)
  {
    i16LeftVoltage = MOTOR_OUT_MAX;
  }

  if (g_cMotorDisable)
  {
    TIM_SetCompare1(TIM3, 0);
    TIM_SetCompare2(TIM3, 0);
  }
  else
  {
    TIM_SetCompare1(TIM3, i16RightVoltage);
    TIM_SetCompare2(TIM3, i16LeftVoltage);
  }
}

/***************************************************************
** 函数名称: MotorOutput
** 功能描述: 电机输出函数
             将直立控制、速度控制、方向控制的输出量进行叠加,并加
                         入死区常量，对输出饱和作出处理。
** 输　入:
** 输　出:
** 全局变量:
** 作　者:   喵呜实验室MiaowLabs
** 淘  宝：  https://miaowlabs.taobao.com/
** 日　期:   2014年08月01日
***************************************************************/
void MotorOutput(void)
{
  // char buffer[50];
  g_fLeftMotorOut =
      g_fAngleControlOut - g_fSpeedControlOut -
      g_fBluetoothDirection; //这里的电机输出等于角度环控制量 +
                             //速度环外环,这里的 - g_fSpeedControlOut
                             //是因为速度环的极性跟角度环不一样，角度环是负反馈，速度环是正反馈
  g_fRightMotorOut =
      g_fAngleControlOut - g_fSpeedControlOut + g_fBluetoothDirection;
  ;
  // if (g_fForwardStatus == 1)
  //   g_fLeftMotorOut -= g_fSpeedDelta;
  if (g_fForwardStatus == 1)
  {
    g_fLeftMotorOut += g_fxyAngleControlOut;
    g_fRightMotorOut -= g_fxyAngleControlOut;
  }
  /*增加死区常数*/
  if ((int)g_fLeftMotorOut > 0)
    g_fLeftMotorOut += MOTOR_OUT_DEAD_VAL;
  else if ((int)g_fLeftMotorOut < 0)
    g_fLeftMotorOut -= MOTOR_OUT_DEAD_VAL;
  if ((int)g_fRightMotorOut > 0)
    g_fRightMotorOut += MOTOR_OUT_DEAD_VAL;
  else if ((int)g_fRightMotorOut < 0)
    g_fRightMotorOut -= MOTOR_OUT_DEAD_VAL;

  /*输出饱和处理，防止超出PWM范围*/
  if ((int)g_fLeftMotorOut > MOTOR_OUT_MAX)
    g_fLeftMotorOut = MOTOR_OUT_MAX;
  if ((int)g_fLeftMotorOut < MOTOR_OUT_MIN)
    g_fLeftMotorOut = MOTOR_OUT_MIN;
  if ((int)g_fRightMotorOut > MOTOR_OUT_MAX)
    g_fRightMotorOut = MOTOR_OUT_MAX;
  if ((int)g_fRightMotorOut < MOTOR_OUT_MIN)
    g_fRightMotorOut = MOTOR_OUT_MIN;

  SetMotorVoltageAndDirection((int)g_fLeftMotorOut, (int)g_fRightMotorOut);
}

void GetMotorPulse(void) //采集电机速度脉冲
{
  g_s16LeftMotorPulse = TIM_GetCounter(TIM2);
  g_s16RightMotorPulse = -TIM_GetCounter(TIM4);
  TIM2->CNT = 0;
  TIM4->CNT = 0; //清零

  g_s32LeftMotorPulseSigma += g_s16LeftMotorPulse;
  g_s32RightMotorPulseSigma += g_s16RightMotorPulse;

  g_iLeftTurnRoundCnt -= g_s16LeftMotorPulse;
  g_iRightTurnRoundCnt -= g_s16RightMotorPulse;
}

/***************************************************************
** 作　  者: MiaowLabs Team
** 官    网：http://www.miaowlabs.com
** 淘    宝：https://miaowlabs.taobao.com/
** 日　  期: 2015年11月29日
** 函数名称: AngleCalculate
** 功能描述: 角度环计算函数
** 输　  入:
** 输　  出:
** 备    注:
********************喵呜实验室MiaowLabs版权所有**************************
***************************************************************/
void AngleCalculate(void)
{
  //-------加速度--------------------------
  //量程为±2g时，灵敏度：16384 LSB/g
  g_fGravityAngle =
      atan2(g_fAccel_y / 16384.0, g_fAccel_z / 16384.0) * 180.0 / 3.14;
  g_fGravityAngle = g_fGravityAngle - g_iGravity_Offset;

  //-------角速度-------------------------
  //范围为2000deg/s时，换算关系：16.4 LSB/(deg/s)
  g_fGyro_x = g_fGyro_x / 16.4; //计算角速度值
  g_fGyroAngleSpeed = g_fGyro_x;

  //-------互补滤波---------------
  g_fCarAngle =
      0.98 * (g_fCarAngle + g_fGyroAngleSpeed * 0.005) + 0.02 * g_fGravityAngle;

  g_fYawAngle = atan2(g_fAccel_x / 16384.0, g_fAccel_y / 16384.0) * 180.0 / 3.14;

  g_fGyro_z = g_fGyro_z / 16.4;
  g_fGyroAngleSpeed_z = g_fGyro_z;
  // g_fxyAngle = 0.98 * (g_fxyAngle + g_fGyroAngleSpeed_z * 0.005) - 0.02 * g_fYawAngle;
  g_fxyAngle = g_fxyAngle + g_fGyroAngleSpeed_z * 0.005;
  //g_fxyAngle=g_fxyAngle+5;
}

/***************************************************************
** 作　  者: 喵呜实验室MiaowLabs
** 官    网：http://www.miaowlabs.com
** 淘    宝：https://miaowlabs.taobao.com/
** 日　  期: 2018年08月27日
** 函数名称: AngleControl
** 功能描述: 角度环控制函数
** 输　  入:
** 输　  出:
** 备    注:
********************喵呜实验室MiaowLabs版权所有**************************
***************************************************************/
void AngleControl(void)
{
  g_fAngleControlOut =
      (CAR_ANGLE_SET - g_fCarAngle) * g_tCarAnglePID.P * 5 +
      (CAR_ANGLE_SPEED_SET - g_fGyroAngleSpeed) * (g_tCarAnglePID.D / 10);
  // 角速度向右为负值 角度向右为正值
  g_fxyAngleControlOut = (CAR_XY_ANGLE_SPEED_SET - g_fGyroAngleSpeed_z) * (g_txyAnglePID.D / 10);
}

/***************************************************************
** 函数名称: SpeedControl
** 功能描述: 速度环控制函数
** 输　入:
** 输　出:
** 全局变量:
** 作　者:   喵呜实验室MiaowLabs
** 淘  宝：  https://miaowlabs.taobao.com/
** 日　期:   2014年08月01日
***************************************************************/

void SpeedControl(void)
{
  float fP, fI;
  float fDelta;
  float fk = 0.02;
  speed_old_2 = g_fCarSpeed;
  g_fCarSpeed = (g_s32LeftMotorPulseSigma + g_s32RightMotorPulseSigma) * 0.5;
  g_fSpeedDelta = fk * (g_s32LeftMotorPulseSigma - g_s32RightMotorPulseSigma);
  

  g_s32LeftMotorPulseSigma = g_s32RightMotorPulseSigma =
      0; //全局变量 注意及时清零

  g_fCarSpeed =
      0.7 * g_fCarSpeedOld + 0.3 * g_fCarSpeed; //低通滤波，使速度更平滑
  g_fCarSpeedOld = g_fCarSpeed;

  fDelta = CAR_SPEED_SET;
  fDelta -= g_fCarSpeed;

  fP = fDelta * (g_tCarSpeedPID.P);
  fI = fDelta * (g_tCarSpeedPID.I / 10.0);

  g_fCarPosition += fI;
  g_fCarPosition += g_fBluetoothSpeed;

  //积分上限设限
  if ((s16)g_fCarPosition > CAR_POSITION_MAX)
    g_fCarPosition = CAR_POSITION_MAX;
  if ((s16)g_fCarPosition < CAR_POSITION_MIN)
    g_fCarPosition = CAR_POSITION_MIN;

  g_fSpeedControlOutOld = g_fSpeedControlOutNew;
  g_fSpeedControlOutNew = fP + g_fCarPosition;
}
/***************************************************************
** 函数名称: SpeedControlOutput
** 功能描述:
*速度环控制输出函数-分多步逐次逼近最终输出，尽可能将对直立环的干扰降低。
** 输　入:
** 输　出:
** 全局变量:
** 作　者:   喵呜实验室MiaowLabs
** 淘  宝：  https://miaowlabs.taobao.com/
** 日　期:   2014年08月01日
***************************************************************/
void SpeedControlOutput(void)
{
  float fValue;
  fValue = g_fSpeedControlOutNew - g_fSpeedControlOutOld;
  g_fSpeedControlOut =
      fValue * (g_u8SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD +
      g_fSpeedControlOutOld;
}

/***************************************************************
** 函数名称: Scale
** 功能描述: 量程归一化处理
** 输　入:
** 输　出:
** 全局变量:
** 作　者:   喵呜实验室MiaowLabs
** 淘  宝：  https://miaowlabs.taobao.com/
** 日　期:   2014年08月01日
***************************************************************/
float Scale(float input, float inputMin, float inputMax, float outputMin,
            float outputMax)
{
  float output;
  if (inputMin < inputMax)
    output =
        (input - inputMin) / ((inputMax - inputMin) / (outputMax - outputMin));
  else
    output =
        (inputMin - input) / ((inputMin - inputMax) / (outputMax - outputMin));
  if (output > outputMax)
    output = outputMax;
  else if (output < outputMin)
    output = outputMin;
  return output;
}

/***************************************************************
** 函数名称: Steer
** 功能描述: 遥控速度及方向处理函数
** 输　入:
** 输　出:
** 全局变量:
** 作　者:   喵呜实验室MiaowLabs
** 淘  宝：  https://miaowlabs.taobao.com/
** 日　期:   2014年08月01日
***************************************************************/
void Steer(float direct, float speed)
{
  if (direct == 0)
  {
    g_fForwardStatus = 1;
    g_fBluetoothDirection = Scale(direct, 0, 10, 0, 400);
  }
  else if (direct > 0)
  {
    g_fForwardStatus = 0;
    g_fBluetoothDirection = Scale(direct, 0, 10, 0, 400);
  }
  else
  {
    g_fForwardStatus = 0;
    g_fBluetoothDirection = -Scale(direct, 0, -10, 0, 400);
  }
  if (speed > 0)
    g_iCarSpeedSet = Scale(speed, 0, 10, 0, 70);
  else
    g_iCarSpeedSet = -Scale(speed, 0, -10, 0, 70);
}

/***************************************************************
** 作　  者: Songyibiao
** 官    网：http://www.miaowlabs.com
** 淘    宝：https://miaowlabs.taobao.com/
** 日　  期: 20160415
** 函数名称: UltraControl
** 功能描述: 超声波跟随/避障
** 输　  入:
** 输　  出:
** 备    注:
********************喵呜实验室MiaowLabs版权所有**************************/
void UltraControl(int mode)
{
  if (mode == 0)
  {
    if ((Distance >= 0) && (Distance <= 12))
    { //距离小于12cm则后退
      Steer(0, -4);
    }
    else if ((Distance > 18) &&
             (Distance <= 30))
    { //距离大于18cm且小于30则前进
      Steer(0, 4);
    }
    else
      Steer(0, 0);
  }
  else if (mode == 1)
  {
    if ((Distance >= 0) &&
        (Distance <= 20))
    { //右转750个脉冲计数，转弯角度约为90度
      Steer(5, 0);
      g_iLeftTurnRoundCnt = 750;
      g_iRightTurnRoundCnt = -750;
    }
    if ((g_iLeftTurnRoundCnt < 0) && (g_iRightTurnRoundCnt > 0))
    {
      Steer(0, 4);
    }
  }
}

/***************************************************************
** 作　  者: MiaowLabs Team
** 官    网：http://www.miaowlabs.com
** 淘    宝：https://miaowlabs.taobao.com/
** 日　  期: 20160415
** 函数名称: TailingControl
** 功能描述: 红外寻迹
** 输　  入:
** 输　  出:
** 备    注:
********************喵呜实验室MiaowLabs版权所有**************************
***************************************************************/
void TailingControl(void)
{
#if INFRARE_DEBUG_EN > 0
  char buff[32];
#endif
  char result;
  float direct = 0;
  float speed = 0;

  result = InfraredDetect();

  speed = 3;

  if ((result & infrared_channel_La) && (result & infrared_channel_Ra) && (result & infrared_channel_Lb) && (result & infrared_channel_Rb))
  {
    speed = 0;
  }
  //if (result & infrared_channel_Lc)
  // direct = -10;
  // if (result & infrared_channel_Lb)
  // direct = 0;
  else if (result & infrared_channel_La)
    direct = -4;
  //else if (result & infrared_channel_Rc)
  // direct = 10;
  // else if (result & infrared_channel_Rb)
  // direct = 6;
  else if (result & infrared_channel_Ra)
    direct = 4;
  else
    direct = 0.0;

  Steer(direct, speed);

#if INFRARE_DEBUG_EN > 0
  sprintf(buff, "Steer:%d, Speed:%d\r\n", (int)direct, (int)speed);
  // DebugOutStr(buff);
#endif
}

#define FORWARD 0
#define FORWARD_STOP 1
#define BACK 2
#define BACK_STOP 3
#define LEFT_TURN 4
#define LEFT_TURN_STOP 5
#define RIGHT_TURN 6
#define RIGHT_TURN_STOP 7

// 初始化状态为前进1m
int report_mode_one_status = FORWARD;
// 计时 主要用于停止计时
int report_mode_one_timer = 0;

void ResetReportOneTimer() { report_mode_one_timer = 0; }

void IncReportOneTimer() { report_mode_one_timer += 20; }

float g_Distance = 0;
float g_speed_old = 0;
int print_counter = 0;
int left_turn_start = 1;
int right_turn_start = 1;
// 每20ms执行一次
void ReportModeOneControl(void)
{
  float current_speed;
  // char buffer[60];
  // sprintf(buffer, "state:%d distance:%f m speed:%f cm/s direction:%f left pulse:%d right pulse:%d\r\n",
  //         report_mode_one_status, g_Distance, g_fCarSpeed,
  //         g_fBluetoothDirection,g_s32LeftMotorPulseSigma,g_s32RightMotorPulseSigma);
  // Uart1SendStr(buffer);
  // print_counter = 0;
  switch (report_mode_one_status)
  {
  case FORWARD:
  {
    if (g_Distance > 2)
    {
      report_mode_one_status = FORWARD_STOP;
      g_Distance = 0;
      g_speed_old = 0;
      ResetReportOneTimer();
      Steer(0, 0);
    }
    else
    {
      Steer(0, 4);
      current_speed = (g_speed_old + g_fCarSpeed) * 0.5;
      g_speed_old = g_fCarSpeed;
      g_Distance += (current_speed * 0.01) * 0.02;
    }
    break;
  }
  case BACK:
  {
    if (g_Distance < -1.3)
    {
      report_mode_one_status = BACK_STOP;
      g_Distance = 0;
      g_speed_old = 0;
      ResetReportOneTimer();
      Steer(0, 0);
    }
    else
    {
      Steer(0, -3);
      current_speed = (g_speed_old + g_fCarSpeed) * 0.5;
      g_speed_old = g_fCarSpeed;
      g_Distance += (current_speed * 0.01) * 0.02;
    }
    break;
  }
  case LEFT_TURN:
  {
    if (left_turn_start == 1)
    {
      Steer(-1, 4);
      // g_iLeftTurnRoundCnt = -750;
      g_iRightTurnRoundCnt = 5000;
      left_turn_start = 0;
    }

    if (g_iRightTurnRoundCnt < 0)
    {
      report_mode_one_status = LEFT_TURN_STOP;
      ResetReportOneTimer();
      Steer(0, 0);
    }
    break;
  }
  case RIGHT_TURN:
  {
    if (right_turn_start == 1)
    {
      Steer(1, 4);
      g_iLeftTurnRoundCnt = 5000;
      // g_iRightTurnRoundCnt = -750;
      right_turn_start = 0;
    }
    if (g_iLeftTurnRoundCnt < 0)
    {
      report_mode_one_status = RIGHT_TURN_STOP;
      ResetReportOneTimer();
      Steer(0, 0);
    }
    break;
  }
  case FORWARD_STOP:
  case BACK_STOP:
  case LEFT_TURN_STOP:
  case RIGHT_TURN_STOP:
  {
    if (report_mode_one_timer > 1000)
    {
      if (report_mode_one_status == FORWARD_STOP)
      {
        report_mode_one_status = BACK;
      }
      else if (report_mode_one_status == BACK_STOP)
      {
        report_mode_one_status = LEFT_TURN;
      }
      else if (report_mode_one_status == LEFT_TURN_STOP)
      {
        report_mode_one_status = RIGHT_TURN;
      }
      else if (report_mode_one_status == RIGHT_TURN_STOP)
      {
      }
      ResetReportOneTimer();
    }
    else
    {
      IncReportOneTimer();
    }
    break;
  }
  }
}

#define TRACE 0
#define AVOID 1
#define STOP 2
#define AVOID_BARRIAR 3

#define AVOID_FORWARD 0
#define AVOID_RIGHT_ONE 1
#define AVOID_FORWARD_AFTER_RIGHT_ONE 2
#define AVOID_LEFT_ONE 3
#define AVOID_FORWARD_AFTER_LEFT_ONE 4
#define AVOID_LEFT_TWO 5
#define AVOID_FORWARD_AFTER_LEFT_TWO 6
#define AVOID_RIGHT_TWO 7

#define DISTANCE_FORWARD 0
#define DISTANCE_BACKWARD 1
#define DISTANCE_TURN 2
#define FORCED_BACKWARD 3
#define FORCED_TURN 4

float abs(float a){
  return a>0 ? a:-a;
}

int avoid_status = AVOID_FORWARD;
int report_mode_two_status = AVOID;
int right_one_start = 1;
int left_one_start = 1;
int right_two_start = 1;
int left_two_start = 1;
int current_direction = 1;
int rotate_speed = 4;
int delta_distance = 0;
int distance_old = -1;
int fixed_distance = 0;
// int leq_0 = 1;
// int geq_180 = 0;
int distance_status = DISTANCE_FORWARD;
int distance_status_old = DISTANCE_FORWARD;
int backward_counter = 0;
int turn_counter = 0;
int stop_counter = 0;
int finetune_counter = 0;
float delta_speed = 0;
int left_barriar_counter = 80;

void ReportModeTwoControl()
{
  // 首先寻迹进入赛道区
  char result;
  char buffer[40];

  result = InfraredDetect();
  if ((result & infrared_channel_La) &&
   (result & infrared_channel_Ra) && 
   (result & infrared_channel_Lb) && 
   (result & infrared_channel_Rb) && 
   (result & infrared_channel_Lc) && 
   (result & infrared_channel_Rc))
  {
    if (report_mode_two_status == TRACE)
    {
      report_mode_two_status = AVOID;
    }
    else if (report_mode_two_status == AVOID)
    {
      // if(left_barriar_counter<=0)
      //   {report_mode_two_status = STOP;}
    }
  }

  if (report_mode_two_status == TRACE)
  {
    float direct = 0;
    float speed = 0;
    speed = 2;

    if (result & infrared_channel_Lc)
    direct = -10;
    // if (result & infrared_channel_Lb)
    // direct = 0;
    if (result & infrared_channel_La)
      direct = -4;
    else if (result & infrared_channel_Rc)
    direct = 10;
    // else if (result & infrared_channel_Rb)
    // direct = 6;
    else if (result & infrared_channel_Ra)
      direct = 4;
    else
      direct = 0.0;

    Steer(direct, speed);
  }

  // if (report_mode_two_status == AVOID_BARRIAR)
  // {
  //   if (left_barriar_counter >= 0)
  //   {
  //     Steer(0, 4);
  //     left_barriar_counter--;
  //   }
  //   else
  //   {
  //     report_mode_two_status = AVOID;
  //   }
  // }

  // 进入避障区
  if (report_mode_two_status == AVOID)
  {
    float txy_Angle=g_fxyAngle;
    if (left_barriar_counter >= 0)
    {
      left_barriar_counter--;
      Steer(0, 4);
      return;
    }

    if (distance_old == -1)
    {
      fixed_distance = Distance;
      distance_old = Distance;
    }
    else if (Distance <= 400)
    {
      if (distance_status == DISTANCE_TURN || distance_status == FORCED_TURN)
      {
        distance_old = fixed_distance;
        fixed_distance = Distance;
      }
      else
      {
        distance_old = fixed_distance;
        fixed_distance = 0.8 * distance_old + 0.2 * Distance;
      }
    }
    
    sprintf(buffer, "status:%d distance:%d speed:%.3f ", distance_status, fixed_distance, g_fCarSpeed);
    Uart1SendStr(buffer);
    delta_distance = fixed_distance - distance_old;
    distance_old = fixed_distance;
    delta_speed = g_fCarSpeed - speed_old_2;

    if (delta_speed <= -13 && distance_status == DISTANCE_FORWARD)
    {
      distance_status = FORCED_BACKWARD;
    }

    if (distance_status == FORCED_BACKWARD)
    {
      if (backward_counter <= 50)
      {
        Steer(0, -4);
        backward_counter++;
      }
      else
      {
        backward_counter = 0;
        distance_status_old = distance_status;
        distance_status = FORCED_TURN;
      }
      return;
    }
  else if (distance_status == FORCED_TURN)
    {
      if (turn_counter <= 50)
      {
        if (90 + txy_Angle <= 0 && current_direction == 1)
        {
          current_direction = -current_direction;
        }
        if (90 + txy_Angle >= 180 && current_direction == -1)
        {
          current_direction = -current_direction;
        }
        Steer(current_direction * rotate_speed, 0);
        turn_counter++;
      }
      else
      {
        turn_counter = 0;
        distance_status_old = distance_status;
        distance_status = DISTANCE_FORWARD;
      }
      return;
    }

    //移到上面
  else if (distance_status == DISTANCE_BACKWARD)
    {
      Steer(0, -4);
      distance_status_old = distance_status;
    }
  else if (distance_status == DISTANCE_TURN)
    {
      float angle = 90 + g_fxyAngle+16;
      if (angle <= 0 && current_direction == 1)
      {
        current_direction = -current_direction;
      }
      if (angle >= 180 && current_direction == -1)
      {
        current_direction = -current_direction;
      }
      Steer(current_direction * rotate_speed, 0);
      distance_status_old = distance_status;
    }
    else
    {
      // if (distance_status_old == DISTANCE_TURN)
      // {
      //   if (finetune_counter < 0)
      //   {
      //     float angle = 90 + txy_Angle;
      //     if (angle <= 0 && current_direction == 1)
      //     {
      //       current_direction = -current_direction;
      //     }
      //     if (angle >= 180 && current_direction == -1)
      //     {
      //       current_direction = -current_direction;
      //     }
      //     Steer(current_direction * rotate_speed, 0);
      //     finetune_counter++;
      //   }
      //   else
      //   {
      //     finetune_counter = 0;
      //     distance_status_old = distance_status;
      //   }
      //}
      
        Steer(0, 6);
    }



    if (delta_distance > 25)
    {

    }
    else
    {
      int is_direct=1;
      
      //预处理
      if(txy_Angle>0 && txy_Angle <6) 
        txy_Angle=txy_Angle-5;

      txy_Angle=txy_Angle+12;

      is_direct= (abs(txy_Angle + 90) < 5 || abs(txy_Angle - 90) < 5 || abs(txy_Angle) < 6);
      // TODO: test 500
      //bug:起初一段时间fixed_distance=0；
      if (fixed_distance!= 0 &&fixed_distance <= 5 || fixed_distance > 500)
        distance_status = DISTANCE_BACKWARD;
      else if ( (fixed_distance >= 5 && fixed_distance < 15)|| (!is_direct))
      //( (fixed_distance >= 5 && fixed_distance < 15)|| (!is_direct))
        distance_status = DISTANCE_TURN;
      else if ( is_direct && fixed_distance >= 15)
      {
        if (g_fCarSpeed <= 0)
          stop_counter++;
        else
          stop_counter = 0;
        if (stop_counter >= 100)
        {
          distance_status = FORCED_BACKWARD;
          stop_counter = 0;
        }
        else
          distance_status = DISTANCE_FORWARD;
      }
    }

  }


  if (report_mode_two_status == STOP)
  {
    Steer(0, 0);
  }
  
  
}
