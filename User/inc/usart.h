/**********************************************************************
��Ȩ���У�	���ش��¿Ƽ���2017.
��		����	http://www.miaowlabs.com
��		����	https://shop275516297.taobao.com/
�� �� ��: 	.h
��    ��:   ����ʵ����
��		��:   3.00
�������:   2017.03.01
��		Ҫ: 	




***********************************************************************/

#ifndef __USART_H
#define __USART_H



#define Uart3BufferSize 128

extern char Uart3Buffer[];			//����ͨ�Ŵ��ڽ��ջ�����
extern unsigned char Uart3Index;

void USART1Init(void);
void USART3Init(char baudrate);


void Uart1SendByte(char byte) ;
void Uart1SendBuff(char *buf, unsigned short len);
void Uart1SendStr(char * str);
	
void Uart3SendByte(char byte)  ;
void Uart3SendBuff(char *buf, unsigned short len);
void Uart3SendStr(char *str);

#define BluetoothOut(str) Uart3SendStr(str)



#endif /* __USART1_H */

