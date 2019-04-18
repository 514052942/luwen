#ifndef __HMI_H
#define __HMI_H
#include "sys.h"

void HMISends(u8 number,char *buf1);
void HMISendData(u8 number,u32 k);
void HMISendstart(void);
void HMIPage(u8 k);    		  
#endif
