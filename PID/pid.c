#include "pid.h"


PID pid;


void Pid_Init(void)
{

	pid.target=0;
	pid.actual=0;

	pid.P=0;
	pid.I=0;
	pid.D=0;

	
	pid.deviation=0;
	pid.deviation_1=0;
	pid.deviation_2=0;
	pid.out=0;
	
	
}


float pidCal(float set)
{
	float out;
	
	pid.target=set;
	pid.actual=(double)(DS18B20_Get_Temp()/10);   //读取传感器值
	pid.deviation=pid.target-pid.actual;

	double increment=(double)(pid.P*(pid.deviation-pid.deviation_1)+pid.I*pid.deviation+pid.D*(pid.deviation-2*pid.deviation_1+pid.deviation_2));

	
	if(increment>50)
	{increment=50;}
	
	if(increment<-50)
	{increment=-50;}
		
	
	pid.out+=increment;
	out=pid.out;
	
	pid.deviation_2=pid.deviation_1;
	pid.deviation_1=pid.deviation;	
	
	return out;
}



