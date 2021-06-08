/**********************************************************************
��Ȩ���У�	���ش��¿Ƽ���2017.
��		����	http://www.miaowlabs.com
��		����	https://shop275516297.taobao.com/
�� �� ��: 	infrare.c
��    ��:   ���ش���
��		��:   3.00
�������:   2017.03.01
��		Ҫ: 	������ģ��


***********************************************************************/

#include "stm32f10x_gpio.h"
#include "infrare.h"


#define La GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)

#define Lc GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3)
#define Ra GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15)

#define Rc GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)

#define Lb GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4)
#define Rb GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12)

void InfraredIOInit(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;

 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //ʹ��GPIOBʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //ʹ��GPIOBʱ��

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //PB0 ??

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8 | GPIO_Pin_15;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3 | GPIO_Pin_5;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*
	��ͨ������ģ��
	result�Ӹ�λ����λ�ֱ��ʶͨ��Lb��La��Ra��Rb
	��⵽���ߵ�ͨ����Ӧ��λ��1
*/
char InfraredDetect(void)
{
	char resut = 0;
	if(Lc)
		resut |= infrared_channel_Lc;
	if(Lb)
	 	resut |= infrared_channel_Lb;
	if(La)
		resut |= infrared_channel_La;
	if(Ra)
		resut |= infrared_channel_Ra;
	if(Rb)
	 	resut |= infrared_channel_Rb;
	if(Rc)
		resut |= infrared_channel_Rc;

	return resut;
}

static char InfrareError = 0;//0:Ok; 1:error
void InfrareSelfCheck(void)
{
		char cnt = 0;
		if(La==1)cnt++;
//		if(Lb==1)cnt++;
		if(Lc==1)cnt++;
		if(Ra==1)cnt++;
//		if(Rb==1)cnt++;
		if(Rc==1)cnt++;

		if(cnt == 4)// ���ÿ��ͨ�����Ǹߵ�ƽ�����ж�Ϊ����ģ��û�н���
			InfrareError = 1;
}

int IsInfrareOK(void)
{
	return !InfrareError;
}
