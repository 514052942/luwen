#ifndef __ADC_H
#define __ADC_H
#include "stm32f4xx.h"


void MY_ADC_Init(void); 				//ADCͨ����ʼ��
uint16_t  Get_Adc(); 		        //���ĳ��ͨ��ֵ 
uint16_t Get_Adc_Average(uint8_t times);//�õ�ĳ��ͨ����������������ƽ��ֵ
#endif 
