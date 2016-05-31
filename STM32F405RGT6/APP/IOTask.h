#ifndef _IO_TASK_H_
#define _IO_TASK_H_
#include "main.h"

//上传数据的类型
// The type of uploaded data
typedef enum
{
	REIMU = 1,
	REMOV = 2,
	REHMC = 3,
	REOFFSET = 4,
	REVERSION = 5,
	REERROR =6,
	REPID =7,
}UploadParamType_e;

void UploadParameter(void);

#endif
