/**********************************************************************
��Ȩ���У�	���ش��¿Ƽ���2017.
��		����	http://www.miaowlabs.com
��		����	https://shop275516297.taobao.com/
�� �� ��: 	ultrasonic.c
��    ��:   ����ʵ����
��		��:   3.00
�������:   2017.03.01
��		Ҫ: 	


***********************************************************************/
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "common.h"
#include "ultrasonic.h"



unsigned int TIM1CH4_CAPTURE_STA;
//bit7:������ɱ�־ 
//bit6�����񵽸ߵ�ƽ��־
//bit5~0�����񵽸ߵ�ƽ��ʱ������Ĵ���
unsigned int TIM1CH4_CAPTURE_VAL;

// �����������룬��λcm
int Distance = 0;
// �������Լ��ʶ��0--ģ��û���ϣ�1--ģ������
int UltraError = 0;


/*
	����ģʽ��ʼ��(�����������������)
*/
void TIM1_Cap_Init(void)	
{	 
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	//ʹ��TIM1ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //ʹ��GPIOAʱ��
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 				//PA11 ����  
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2;     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     	//PA2��� 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     	//2M
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//��ʼ����ʱ��1 TIM1	 
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF; 						//�趨�������Զ���װֵ 
	TIM_TimeBaseStructure.TIM_Prescaler =72-1; 						//Ԥ��Ƶ��   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 			//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
  
	//��ʼ��TIM1���벶�����
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; 			//CC1S=03 	ѡ������� IC3ӳ�䵽TI1��
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;//���������Ƶ,����Ƶ 
 	TIM_ICInitStructure.TIM_ICFilter = 0x00;							//���������˲��� ���˲�
  TIM_ICInit(TIM1, &TIM_ICInitStructure);
	

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn|TIM1_UP_IRQn;  //TIM1�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 	
	TIM_ITConfig(TIM1,TIM_IT_Update|TIM_IT_CC4,ENABLE);//���������ж� ,����CC3IE�����ж�	
 	TIM_Cmd(TIM1,ENABLE ); 	//ʹ�ܶ�ʱ��1
}

/*
	����һ�γ�������࣬����ȡ�ϴβ������
*/
void Read_Distane(void)
{   
	 GPIO_SetBits(GPIOA, GPIO_Pin_2);
	 delay_us(20);  
	 GPIO_ResetBits(GPIOA, GPIO_Pin_2);
	if(TIM1CH4_CAPTURE_STA&0X80)//�ɹ�������һ�θߵ�ƽ
	{
		Distance = TIM1CH4_CAPTURE_STA&0X3F;
		Distance *= 65536;					 //���ʱ���ܺ�
		Distance += TIM1CH4_CAPTURE_VAL;		//�õ��ܵĸߵ�ƽʱ��
		Distance = Distance*170/10000;//����������ǲ�����������ȥ�ͷ��������ʱ��������΢������λ��
//�����е�������ÿ��340�ס�һ΢��ʱ����������ľ�����340��0.000001=0.00034��=0.034���ף�����2�����������:0.017=1.7/100�������1.7��������
		TIM1CH4_CAPTURE_STA=0;				//������һ�β���
	}

}

/*
	�����жϴ�������
*/
void TIM1_CC_IRQHandler(void)
{ 		    		  			    
	if((TIM1CH4_CAPTURE_STA&0X80)==0)//��δ���һ�����岶��	
	{	  
		if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
		{	    
			if(TIM1CH4_CAPTURE_STA&0X40)
			{
				if((TIM1CH4_CAPTURE_STA&0X3F)==0X3F)
				{
					TIM1CH4_CAPTURE_STA|=0X80;
					TIM1CH4_CAPTURE_VAL=0XFFFF;
				}
				else TIM1CH4_CAPTURE_STA++;
			}	 
		}
		if (TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET)//ͨ��4�����¼�
		{	
			if(TIM1CH4_CAPTURE_STA&0X40){	  			
				TIM1CH4_CAPTURE_STA|=0X80;		//����Ѿ������½���
				TIM1CH4_CAPTURE_VAL=TIM_GetCapture4(TIM1);
				TIM_OC4PolarityConfig(TIM1,TIM_ICPolarity_Rising); //����Ϊ�����ز���
			}
			else{
				TIM1CH4_CAPTURE_STA=0;			
				TIM1CH4_CAPTURE_VAL=0;
				TIM_SetCounter(TIM1,0);
				TIM1CH4_CAPTURE_STA|=0X40;													//����Ѿ�����������
				TIM_OC4PolarityConfig(TIM1,TIM_ICPolarity_Falling);	// ����Ϊ�½��ز���
			}		    
		}			     	    					   
  }
  TIM_ClearITPendingBit(TIM1, TIM_IT_CC4|TIM_IT_Update); 
}

/*
	ģ���Լ죬�����ϵ�ʱ��ⳬ����ģ���Ƿ����
*/
void UltraSelfCheck(void)
{
	delay_ms(1000);//�°泬����ģ���ϵ��ڲ���ʼ��Ҫ��1�룬����ʱ1��
	if(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11)){
		delay_ms(50);
		if(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11))
			UltraError = 1;
	}
}

int IsUltraOK(void)
{
	return UltraError;
}


