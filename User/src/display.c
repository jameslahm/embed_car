/**********************************************************************
��Ȩ���У�	���ش��¿Ƽ���2017.
��		����	http://www.miaowlabs.com
��		����	https://miaowlabs.taobao.com/
�� �� ��: 	display.c
��    ��:   ����ʵ����
��		��:   3.00
�������?:   2017.03.01
��		Ҫ: 	


***********************************************************************/

#include "stdio.h"
#include "oled.h"
#include "bmp.h"
#include "control.h"
#include "ultrasonic.h"
#include "mpu6050.h"
#include "common.h"
#include "manage.h"
#include "bsp.h"

extern unsigned short BatVol;
extern int report_mode_two_status;
extern int distance_status;
extern float g_fxyAngle;


/*
	��ʾlogo
*/
void ShowHomePageInit(void)
{
	OLED_DrawBMP(0,0,128,8,LOGO);  //ͼƬ��ʾ
	delay_ms(1000);
	OLED_Clear();
}


/*
	oled��ҳˢ�º���
	������ˢ�£�����һ��ˢ��ʱ�����?
*/

void ShowHomePage(void)
{
	unsigned char buff[48]={0};
	static char step = 0;

	step++;
	if(step >= 6)step = 0;

	//�ֲ�ִ�У����̵���ˢ��ʱ��
	if(step == 0){
		OLED_ShowString(0, 0, "Mode: Complementary  ");
	}

	if(step == 1){

		snprintf((char*)buff, 21, "xy: %0.1f      ", g_fxyAngle);
		// if(IsUltraOK()){
		// 	snprintf((char*)buff, 21,  "D: %d,%d", Distance,fixed_distance);
		// }
		// else
		// 	snprintf((char*)buff, 21,  "Distance:  %s(cm)       ", "xx");
		
		OLED_ShowString(0, 1, buff);
	}

	if(step == 2){
		snprintf((char*)buff, 21,  "EncoLeft:  %d         ",g_s16LeftMotorPulse);
		OLED_ShowString(0, 2, buff);
	}
	if(step == 3){
		snprintf((char*)buff, 21, "EncoRight: %d         ",g_s16RightMotorPulse);
		OLED_ShowString(0, 3, buff);
	}
	
	if(step == 4){
		snprintf((char*)buff, 21, "Angle:     %0.1f      ", g_fCarAngle);
		OLED_ShowString(0, 4, buff);
	}
	if(step == 5){
		// snprintf((char*)buff, 21, "Battery:   %0.1f(V)      ", g_BatVolt/100.0);
		snprintf((char*)buff, 21, "Status:   %d      ", distance_status);
		// snprintf((char*)buff, 21, "Positio:   %0.1f(m)      ", g_fCarPosition/100.0);
		OLED_ShowString(0, 5, buff);	
		}
}


