/*************************************************************************
ע��;
1.�豸����ָ�������Ϊ��0XFF 0XFF 0XFF�������ֽ�
�ڱ����������Ӧ�ڡ���HMISendEND();��������������Ҫ�е�
2.����ָ�����Լ�����ȫ��ʹ��ASCII �ַ�����ʽ���Ƕ��������ݣ������Ķ��͵��ԡ�
3.����ָ����ʹ��Сд��ĸ(�˴�����ָ����ָ������ΪСд�������ô�д��ʱ����Ҫ��д)��	
 **************************************************************************/	

#include "hmi.h"
#include "delay.h"
#include "usart.h"

u8 dataseq[100]={0};
u8 FinishCode[3] = {0xFF,0xFF,0xFF};

void HMISends(u8 number,char *buf1)		  //�ַ������ͺ���
{
	sprintf((char*)&dataseq[0],"t%d.txt=\"%s\"",number,buf1);
	printf("%s",dataseq);
	printf("%s",FinishCode);	
}

void HMISendData(u8 number,u32 k)
{
	sprintf((char*)&dataseq[0],"t%d.txt=\"%d\"",number,k);
	printf("%s",dataseq);
	printf("%s",FinishCode);
}

void HMISendstart(void)    		  //HMI��ʼ���ͺ���
{
	printf("%s",FinishCode);
	delay_ms(200);
}

void HMIPage(u8 k)
{
	if(k==0)
	{
		u8 page0[]={"page 0"};
		printf("%s",page0);
		printf("%s",FinishCode);
	}
	else if(k==1)
	{
		u8 page1[]={"page 1"};
		printf("%s",page1);
		printf("%s",FinishCode);
	}
		else if(k==2)
	{
		u8 page2[]={"page 2"};
		printf("%s",page2);
		printf("%s",FinishCode);
	}
		else if(k==3)
	{
		u8 page3[]={"page 3"};
		printf("%s",page3);
		printf("%s",FinishCode);
	}
		else if(k==4)
	{
		u8 page4[]={"page 4"};
		printf("%s",page4);
		printf("%s",FinishCode);
	}
		else if(k==5)
	{
		u8 page5[]={"page 5"};
		printf("%s",page5);
		printf("%s",FinishCode);
	}
		else if(k==6)
	{
		u8 page6[]={"page 6"};
		printf("%s",page6);
		printf("%s",FinishCode);
	}
		else if(k==7)
	{
		u8 page7[]={"page 7"};
		printf("%s",page7);
		printf("%s",FinishCode);
	}
	else
	{
		printf("%s","t0.txt=\"��������\"");
		printf("%s",FinishCode);
	}
}

