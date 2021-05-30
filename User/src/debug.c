/**********************************************************************
��Ȩ���У�	���ش��¿Ƽ���2017.
��		����	http://www.miaowlabs.com
��		����	https://miaowlabs.taobao.com/
�� �� ��: 	debug.c
��    ��:   ����ʵ����
��		��:   3.00
�������:   2017.03.01
��		Ҫ: 	ϵͳ���Կ��ƣ�ͨ��dbg.h�ļ����õ������ģʽ


***********************************************************************/
#include "common.h"
#include "mpu6050.h"
#include "control.h"
#include "debug.h"


static float IMUData[4] = {0, 0, 0, 0};


void IMUToScope(void)
{
  int temp[4] = {0};
  unsigned int temp1[4] = {0};
  unsigned char databuf[10] = {0};
  unsigned char i;
  unsigned short CRC16 = 0;

  for(i = 0; i < 4; i++){
      	temp[i]  = (int)IMUData[i];
   	    temp1[i] = (unsigned int)temp[i];
    }
   
  for(i = 0; i < 4; i++){
    databuf[i*2]   = (unsigned char)(temp1[i]%256);
    databuf[i*2+1] = (unsigned char)(temp1[i]/256);
  }
  
  CRC16 = CRC16Calculate(databuf,8);
  databuf[8] = CRC16%256;
  databuf[9] = CRC16/256;

  DebugOutBuff((char*)databuf, 10);

}


void IMUToFourAxisMonitor(void)
{
	short temp[3];
	unsigned char cTemp[32], Index = 0;
	char i = 0;
	
	temp[0] = IMUData[0];
	temp[1] = IMUData[1];
	temp[2] = IMUData[2];

	cTemp[Index++] = 0xaa;//֡ͷ
	cTemp[Index++] = 0x55;//֡ͷ
	cTemp[Index++] = 0x01;//������
	cTemp[Index++] = 0x06;//���ݳ��ȣ��ֽڣ�
	
	for(i = 0; i < 3;  i ++){
		cTemp[Index++] = (temp[i] >> 8);
		cTemp[Index++] = (temp[i]);
	}

	cTemp[Index++] = Sum_Get((char*)cTemp, Index);

	DebugOutBuff((char*)cTemp, Index);
}



void DebugService(void)
{
	IMUData[0] = g_fGravityAngle;
	IMUData[1] = g_fGyroAngleSpeed;
	IMUData[2] = g_fCarAngle;

	#if IMU_SCOPE_EN
		IMUToScope();
	#endif

	#if IMU_FOURAXISMONITOR_EN
		IMUToFourAxisMonitor();
	#endif
}

