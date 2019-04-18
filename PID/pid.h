#ifndef _PID_H
#define _PID_H


void Pid_Init(void);
float pidCal(float set);

typedef struct
{
	double target;			//�û��趨ֵ
  double actual;			//ʵ��ֵ
	
	double P;//����ϵ��
	double I;//���û���ϵ��
	double D;//����΢��ϵ��
	
	double deviation;//����ƫ��
	double deviation_1;//�ϴ�ƫ��
	double deviation_2;//���ϴ�ƫ��
  
	double out;

}PID;



#endif