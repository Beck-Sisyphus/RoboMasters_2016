#ifndef ACOM_H
#define ACOM_H


#include "windows.h"
#include <process.h>
#include "string.h"
/***************
串口功能函数:
打开opencomm
读取ReadData
写入WriteData
关闭CloseComm
***************/



int OpenComm(char* port);

int ReadData(HANDLE hCom,unsigned char* pBuffer, unsigned char ulen);

int WriteData(HANDLE hCom, unsigned char *pBuffer, unsigned char uLen);	

void CloseComm(HANDLE hCom);

/****校验和计算函数********/
unsigned char  CheckSum(unsigned char *uBuff, unsigned char uBuffLen); 


/********操作类封装********/

class Operate
{
public:
	Operate()
	{
	}
	~Operate()
	{}
	//总初始化函数
	bool Initial(char* port);


	//联机请求
	bool SendLocation(HANDLE hCom,unsigned char* data);
	bool SendTankNumber(HANDLE hCom,unsigned char* data);
	bool SendVelocity(HANDLE hCom,unsigned char* data);
public:
	int i;

};	





HANDLE A_hCom;//全局句柄
unsigned char str[100];//接受数据保存地址


#endif