/**********************************************************************
��Ȩ���У�	  ����ʵ����MiaowLabs��2017.
��		����	http://www.miaowlabs.com
��		����	https://miaowlabs.taobao.com/
�� �� ��: 	  control.c
��    ��:   ����ʵ����MiaowLabs
��		��:   3.00
�������:   2017.03.01
��		Ҫ:


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

unsigned char g_cMotorDisable = 0; //ֵ����0ʱ�������ת��������ֹͣת��

int g_iGravity_Offset = 0;

/******������Ʋ���******/
float g_fSpeedControlOut;
float g_fSpeedControlOutOld;
float g_fSpeedControlOutNew;
float g_fAngleControlOut;
float g_fLeftMotorOut;
float g_fRightMotorOut;
float g_fxyAngleControlOut;
/******�ٶȿ��Ʋ���******/

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

/*-----�ǶȻ����ٶȻ�PID���Ʋ���-----*/
PID_t g_tCarAnglePID = {17.0, 0, 23.0};  //*5 /10
PID_t g_tCarSpeedPID = {15.25, 1.08, 0}; // i/10
PID_t g_txyAnglePID = {0, 0, 23.0};
/******�������Ʋ���******/
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
** ��������: CarUpstandInit
** ��������: ȫ�ֱ�����ʼ������
** �䡡��:
** �䡡��:
** ȫ�ֱ���:
** ������:   ����ʵ����MiaowLabs
** ��  ����  https://miaowlabs.taobao.com/
** �ա���:   2014��08��01��
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
** ��������: AbnormalSpinDetect
** ��������: ���ת���쳣���
** �䡡��:
** �䡡��:
** ȫ�ֱ���:
** ������:   ����ʵ����MiaowLabs
** �ա���:   2017��4��26��
***************************************************************/

void AbnormalSpinDetect(short leftSpeed, short rightSpeed)
{
  static unsigned short count = 0;

  //�ٶ�����Ϊ0ʱ��⣬���򲻼��
  if (g_iCarSpeedSet == 0)
  {
    if (((leftSpeed > 30) && (rightSpeed > 30) && (g_fCarAngle > -30) &&
         (g_fCarAngle < 30)) ||
        ((leftSpeed < -30) && (rightSpeed < -30)) && (g_fCarAngle > -30) &&
            (g_fCarAngle <
             30))
    { // ���ҵ��ת�ٴ���30��������ͬ������ʱ�䳬��250ms���ҳ����ǶȲ�����30�ȣ����ж�Ϊ���տ�ת
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
** ��������: LandingDetect
** ��������: С���ŵؼ��
** �䡡��:
** �䡡��:
** ȫ�ֱ���:
** ������:   ����ʵ����MiaowLabs
** �ա���:   2017��4��26��
***************************************************************/
void LandingDetect(void)
{
  static float lastCarAngle = 0;
  static unsigned short count = 0, count1 = 0;

  if (AbnormalSpinFlag == 0)
    return;

  // С���Ƕ�5��~-5���������
  if ((g_fCarAngle > -5) && (g_fCarAngle < 5))
  {
    count1++;
    if (count1 >=
        50)
    { //ÿ��250ms�ж�һ��С���Ƕȱ仯�����仯��С��0.8������-0.8���ж�ΪС����ֹ
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
** ��������: MotorManage
** ��������: ���ʹ��/ʧ�ܿ���
** �䡡��:
** �䡡��:
** ȫ�ֱ���:
** ������:   ����ʵ����MiaowLabs
** �ա���:   2017��4��26��
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
** ��������: SetMotorVoltageAndDirection
** ��������: ���ת�ټ�������ƺ���
** �䡡��:
** �䡡��:
** ȫ�ֱ���:
** ������:   ����ʵ����MiaowLabs
** ��  ����  https://miaowlabs.taobao.com/
** �ա���:   2018��08��27��
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
** ��������: MotorOutput
** ��������: ����������
             ��ֱ�����ơ��ٶȿ��ơ�������Ƶ���������е���,����
                         �������������������������������
** �䡡��:
** �䡡��:
** ȫ�ֱ���:
** ������:   ����ʵ����MiaowLabs
** ��  ����  https://miaowlabs.taobao.com/
** �ա���:   2014��08��01��
***************************************************************/
void MotorOutput(void)
{
  // char buffer[50];
  g_fLeftMotorOut =
      g_fAngleControlOut - g_fSpeedControlOut -
      g_fBluetoothDirection; //����ĵ��������ڽǶȻ������� +
                             //�ٶȻ��⻷,����� - g_fSpeedControlOut
                             //����Ϊ�ٶȻ��ļ��Ը��ǶȻ���һ�����ǶȻ��Ǹ��������ٶȻ���������
  g_fRightMotorOut =
      g_fAngleControlOut - g_fSpeedControlOut + g_fBluetoothDirection;
  ;
  // if (g_fForwardStatus == 1)
  //   g_fLeftMotorOut -= g_fSpeedDelta;
  // if (g_fForwardStatus == 1)
  // {
  //   g_fLeftMotorOut += g_fxyAngleControlOut;
  //   g_fRightMotorOut -= g_fxyAngleControlOut;
  // }
  /*������������*/
  if ((int)g_fLeftMotorOut > 0)
    g_fLeftMotorOut += MOTOR_OUT_DEAD_VAL;
  else if ((int)g_fLeftMotorOut < 0)
    g_fLeftMotorOut -= MOTOR_OUT_DEAD_VAL;
  if ((int)g_fRightMotorOut > 0)
    g_fRightMotorOut += MOTOR_OUT_DEAD_VAL;
  else if ((int)g_fRightMotorOut < 0)
    g_fRightMotorOut -= MOTOR_OUT_DEAD_VAL;

  /*������ʹ�������ֹ����PWM��Χ*/
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

void GetMotorPulse(void) //�ɼ�����ٶ�����
{
  g_s16LeftMotorPulse = TIM_GetCounter(TIM2);
  g_s16RightMotorPulse = -TIM_GetCounter(TIM4);
  TIM2->CNT = 0;
  TIM4->CNT = 0; //����

  g_s32LeftMotorPulseSigma += g_s16LeftMotorPulse;
  g_s32RightMotorPulseSigma += g_s16RightMotorPulse;

  g_iLeftTurnRoundCnt -= g_s16LeftMotorPulse;
  g_iRightTurnRoundCnt -= g_s16RightMotorPulse;
}

/***************************************************************
** ����  ��: MiaowLabs Team
** ��    ����http://www.miaowlabs.com
** ��    ����https://miaowlabs.taobao.com/
** �ա�  ��: 2015��11��29��
** ��������: AngleCalculate
** ��������: �ǶȻ����㺯��
** �䡡  ��:
** �䡡  ��:
** ��    ע:
********************����ʵ����MiaowLabs��Ȩ����**************************
***************************************************************/
void AngleCalculate(void)
{
  //-------���ٶ�--------------------------
  //����Ϊ��2gʱ�������ȣ�16384 LSB/g
  g_fGravityAngle =
      atan2(g_fAccel_y / 16384.0, g_fAccel_z / 16384.0) * 180.0 / 3.14;
  g_fGravityAngle = g_fGravityAngle - g_iGravity_Offset;

  //-------���ٶ�-------------------------
  //��ΧΪ2000deg/sʱ�������ϵ��16.4 LSB/(deg/s)
  g_fGyro_x = g_fGyro_x / 16.4; //������ٶ�ֵ
  g_fGyroAngleSpeed = g_fGyro_x;

  //-------�����˲�---------------
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
** ����  ��: ����ʵ����MiaowLabs
** ��    ����http://www.miaowlabs.com
** ��    ����https://miaowlabs.taobao.com/
** �ա�  ��: 2018��08��27��
** ��������: AngleControl
** ��������: �ǶȻ����ƺ���
** �䡡  ��:
** �䡡  ��:
** ��    ע:
********************����ʵ����MiaowLabs��Ȩ����**************************
***************************************************************/
void AngleControl(void)
{
  g_fAngleControlOut =
      (CAR_ANGLE_SET - g_fCarAngle) * g_tCarAnglePID.P * 5 +
      (CAR_ANGLE_SPEED_SET - g_fGyroAngleSpeed) * (g_tCarAnglePID.D / 10);
  // ���ٶ�����Ϊ��ֵ �Ƕ�����Ϊ��ֵ
  g_fxyAngleControlOut = (CAR_XY_ANGLE_SPEED_SET - g_fGyroAngleSpeed_z) * (g_txyAnglePID.D / 10);
}

/***************************************************************
** ��������: SpeedControl
** ��������: �ٶȻ����ƺ���
** �䡡��:
** �䡡��:
** ȫ�ֱ���:
** ������:   ����ʵ����MiaowLabs
** ��  ����  https://miaowlabs.taobao.com/
** �ա���:   2014��08��01��
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
      0; //ȫ�ֱ��� ע�⼰ʱ����

  g_fCarSpeed =
      0.7 * g_fCarSpeedOld + 0.3 * g_fCarSpeed; //��ͨ�˲���ʹ�ٶȸ�ƽ��
  g_fCarSpeedOld = g_fCarSpeed;

  fDelta = CAR_SPEED_SET;
  fDelta -= g_fCarSpeed;

  fP = fDelta * (g_tCarSpeedPID.P);
  fI = fDelta * (g_tCarSpeedPID.I / 10.0);

  g_fCarPosition += fI;
  g_fCarPosition += g_fBluetoothSpeed;

  //������������
  if ((s16)g_fCarPosition > CAR_POSITION_MAX)
    g_fCarPosition = CAR_POSITION_MAX;
  if ((s16)g_fCarPosition < CAR_POSITION_MIN)
    g_fCarPosition = CAR_POSITION_MIN;

  g_fSpeedControlOutOld = g_fSpeedControlOutNew;
  g_fSpeedControlOutNew = fP + g_fCarPosition;
}
/***************************************************************
** ��������: SpeedControlOutput
** ��������:
*�ٶȻ������������-�ֶಽ��αƽ���������������ܽ���ֱ�����ĸ��Ž��͡�
** �䡡��:
** �䡡��:
** ȫ�ֱ���:
** ������:   ����ʵ����MiaowLabs
** ��  ����  https://miaowlabs.taobao.com/
** �ա���:   2014��08��01��
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
** ��������: Scale
** ��������: ���̹�һ������
** �䡡��:
** �䡡��:
** ȫ�ֱ���:
** ������:   ����ʵ����MiaowLabs
** ��  ����  https://miaowlabs.taobao.com/
** �ա���:   2014��08��01��
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
** ��������: Steer
** ��������: ң���ٶȼ�����������
** �䡡��:
** �䡡��:
** ȫ�ֱ���:
** ������:   ����ʵ����MiaowLabs
** ��  ����  https://miaowlabs.taobao.com/
** �ա���:   2014��08��01��
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
** ����  ��: Songyibiao
** ��    ����http://www.miaowlabs.com
** ��    ����https://miaowlabs.taobao.com/
** �ա�  ��: 20160415
** ��������: UltraControl
** ��������: ����������/����
** �䡡  ��:
** �䡡  ��:
** ��    ע:
********************����ʵ����MiaowLabs��Ȩ����**************************/
void UltraControl(int mode)
{
  if (mode == 0)
  {
    if ((Distance >= 0) && (Distance <= 12))
    { //����С��12cm�����
      Steer(0, -4);
    }
    else if ((Distance > 18) &&
             (Distance <= 30))
    { //�������18cm��С��30��ǰ��
      Steer(0, 4);
    }
    else
      Steer(0, 0);
  }
  else if (mode == 1)
  {
    if ((Distance >= 0) &&
        (Distance <= 20))
    { //��ת750�����������ת��Ƕ�ԼΪ90��
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
** ����  ��: MiaowLabs Team
** ��    ����http://www.miaowlabs.com
** ��    ����https://miaowlabs.taobao.com/
** �ա�  ��: 20160415
** ��������: TailingControl
** ��������: ����Ѱ��
** �䡡  ��:
** �䡡  ��:
** ��    ע:
********************����ʵ����MiaowLabs��Ȩ����**************************
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

// ��ʼ��״̬Ϊǰ��1m
int report_mode_one_status = FORWARD;
// ��ʱ ��Ҫ����ֹͣ��ʱ
int report_mode_one_timer = 0;

void ResetReportOneTimer() { report_mode_one_timer = 0; }

void IncReportOneTimer() { report_mode_one_timer += 20; }

float g_Distance = 0;
float g_speed_old = 0;
int print_counter = 0;
int left_turn_start = 1;
int right_turn_start = 1;
// ÿ20msִ��һ��
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
      Steer(0, -4);
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
int report_mode_two_status = TRACE;
int right_one_start = 1;
int left_one_start = 1;
int right_two_start = 1;
int left_two_start = 1;
int current_direction = 1;
int rotate_speed = 3;
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
  // ����Ѱ������������
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
      report_mode_two_status = AVOID_BARRIAR;
    }
    else if (report_mode_two_status == AVOID)
    {
      // report_mode_two_status = STOP;
    }
  }

  if (report_mode_two_status == TRACE)
  {
    float direct = 0;
    float speed = 0;
    speed = 2;

    //if (result & infrared_channel_Lc)
    // direct = -10;
    // if (result & infrared_channel_Lb)
    // direct = 0;
    if (result & infrared_channel_La)
      direct = -4;
    // else if (result & infrared_channel_Rc)
    // direct = 10;
    // else if (result & infrared_channel_Rb)
    // direct = 6;
    else if (result & infrared_channel_Ra)
      direct = 4;
    else
      direct = 0.0;

    Steer(direct, speed);
  }

  if (report_mode_two_status == AVOID_BARRIAR)
  {
    if (left_barriar_counter >= 0)
    {
      Steer(0, 4);
      left_barriar_counter--;
    }
    else
    {
      report_mode_two_status = AVOID;
    }
  }

  // ���������
  if (report_mode_two_status == AVOID)
  {

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
        if (90 + g_fxyAngle <= 0 && current_direction == 1)
        {
          current_direction = -current_direction;
        }
        if (90 + g_fxyAngle >= 180 && current_direction == -1)
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

    //�Ƶ�����
    if (distance_status == DISTANCE_BACKWARD)
    {
      Steer(0, -4);
      distance_status_old = distance_status;
    }
    else if (distance_status == DISTANCE_TURN)
    {
      float angle = 90 + g_fxyAngle;
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
      if (distance_status_old == DISTANCE_TURN)
      {
        if (finetune_counter < 0)
        {
          float angle = 90 + g_fxyAngle;
          if (angle <= 0 && current_direction == 1)
          {
            current_direction = -current_direction;
          }
          if (angle >= 180 && current_direction == -1)
          {
            current_direction = -current_direction;
          }
          Steer(current_direction * rotate_speed, 0);
          finetune_counter++;
        }
        else
        {
          finetune_counter = 0;
          distance_status_old = distance_status;
        }
      }
      else
      {
        Steer(0, 4);
      }
    }

    if (delta_distance > 25)
    {

    }
    else
    {
      int is_direct=1;
      float txy_Angle=g_fxyAngle;
      //Ԥ����
      if(txy_Angle>0 && txy_Angle <5) 
        txy_Angle=txy_Angle-3.5;

      txy_Angle=txy_Angle+3.5;

      is_direct= (abs(txy_Angle + 90) < 5 || abs(txy_Angle - 90) < 5 || abs(txy_Angle) < 6);
      // TODO: test 500
      //bug:���һ��ʱ��fixed_distance=0��
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

    
  
  // // ���������
  // if (report_mode_two_status == AVOID) {
  //   char buffer[40];
  //   sprintf(buffer,"status:%d left pulse:%d right pulse:%d",avoid_status,g_s16LeftMotorPulse,g_s16RightMotorPulse);
  //   Uart1SendStr(buffer);
  //   switch (avoid_status) {
  //   case AVOID_FORWARD: {
  //     left_one_start = 1;
  //     left_two_start = 1;
  //     right_one_start = 1;
  //     right_two_start = 1;
  //     if (Distance > 0 && Distance <= 20) {
  //       avoid_status = AVOID_RIGHT_ONE;
  //     } else {
  //       Steer(0, 4);
  //     }
  //     break;
  //   }
  //   case AVOID_RIGHT_ONE: {
  //     if (right_one_start == 1) {
  //       Steer(5, 0);
  //       g_iLeftTurnRoundCnt = 1200;
  //       right_one_start = 0;
  //     }
  //     if ((g_iLeftTurnRoundCnt < 0)) {
  //       Steer(0, 4);
  //       avoid_status = AVOID_FORWARD_AFTER_RIGHT_ONE;
  //     }
  //     break;
  //   }
  //   case AVOID_FORWARD_AFTER_RIGHT_ONE: {
  //     if (Distance >= 0 && Distance <= 20) {
  //       avoid_status = AVOID_LEFT_ONE;
  //     } else {
  //       Steer(0, 4);
  //     }
  //     break;
  //   }
  //   case AVOID_LEFT_ONE: {
  //     if (left_one_start == 1) {
  //       Steer(-5, 0);
  //       g_iRightTurnRoundCnt = 750;
  //       left_one_start = 0;
  //     }
  //     if ((g_iRightTurnRoundCnt < 0)) {
  //       Steer(0, 4);
  //       avoid_status = AVOID_FORWARD_AFTER_LEFT_ONE;
  //     }
  //     break;
  //   }
  //   case AVOID_FORWARD_AFTER_LEFT_ONE: {
  //     if ((Distance >= 0) && (Distance <= 20)) {
  //       avoid_status = AVOID_LEFT_TWO;
  //     } else {
  //       avoid_status = AVOID_FORWARD;
  //     }
  //     break;
  //   }
  //   case AVOID_LEFT_TWO: {
  //     if (left_two_start ==1) {
  //       Steer(-5, 0);
  //       g_iRightTurnRoundCnt = 750;
  //       left_two_start = 0;
  //     }
  //     if ((g_iRightTurnRoundCnt < 0)) {
  //       Steer(0, 4);
  //       avoid_status = AVOID_FORWARD_AFTER_LEFT_TWO;
  //     }
  //     break;
  //   }
  //   case AVOID_FORWARD_AFTER_LEFT_TWO: {
  //     if (Distance >= 0 && Distance <= 20) {
  //       avoid_status = AVOID_RIGHT_TWO;
  //     } else {
  //       Steer(0, 4);
  //     }
  //     break;
  //   }
  //   case AVOID_RIGHT_TWO: {
  //     if (right_two_start == 1) {
  //       Steer(5, 0);
  //       g_iLeftTurnRoundCnt = 1200;
  //       right_two_start = 0;
  //     }
  //     if ((g_iLeftTurnRoundCnt < 0)) {
  //       Steer(0, 4);
  //       avoid_status = AVOID_FORWARD;
  //     }
  //     break;
  //   }
  //   }
  // }
  // //
  }


  if (report_mode_two_status == STOP)
  {
    Steer(0, 0);
  }
  
  
}
