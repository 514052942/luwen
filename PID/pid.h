#ifndef _PID_H
#define _PID_H


void Pid_Init(void);
float pidCal(float set);

typedef struct
{
	double target;			//用户设定值
  double actual;			//实际值
	
	double P;//比例系数
	double I;//设置积分系数
	double D;//设置微分系数
	
	double deviation;//本次偏差
	double deviation_1;//上次偏差
	double deviation_2;//上上次偏差
  
	double out;

}PID;



#endif