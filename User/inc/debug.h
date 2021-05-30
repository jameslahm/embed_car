/**********************************************************************
��Ȩ���У�	���ش��¿Ƽ���2017.
��		����	http://www.miaowlabs.com
��		����	https://shop275516297.taobao.com/
�� �� ��: 	debug.h
��    ��:   ����ʵ����
��		��:   3.00
�������:   2017.03.01
��		Ҫ: 	




***********************************************************************/
#ifndef _DEBUG_H
#define _DEBUG_H

#include "usart.h"

#define DEBUG_EN				1


#define IMU_SCOPE_EN						0	// ʹ�����ģ��ʾ����������Ϣ
#define IMU_FOURAXISMONITOR_EN	1	// ʹ���������ģ����������Ϣ

#define COMMUNICATE_DEBUG_EN		0	// ʹ���������ͨ���ŵ�����Ϣ
#define INFRARE_DEBUG_EN				0	// ʹ���������Ѱ��������Ϣ


#if DEBUG_EN
#define DebugOutByte(byte)	Uart1SendByte(byte)
#define DebugOutBuff(buff, len)	Uart1SendBuff(buff, len)
#define DebugOutStr(str)		Uart1SendStr(str)
#else
#define DebugOutByte(byte)
#define DebugOutBuff(buff,len)
#define DebugOutStr(str)
#endif


void DebugService(void);




#endif 
