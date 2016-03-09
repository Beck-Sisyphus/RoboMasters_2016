#ifndef MYCOMCLASS_H
#define MYCOMCLASS_H

#include "windows.h"
#include "MyConstant.h"
//openmp支持
#include <omp.h>
/********操作类封装********/

class PortOperate
{
public:
	PortOperate()
	{
	}
	~PortOperate()
	{}
	//总初始化函数
	bool Initial(char* port);
	/***************
	串口功能函数:
	打开opencomm
	读取ReadData
	写入WriteData
	关闭CloseComm
	***************/
	int OpenComm(char* port);

	int ReadData(unsigned char *pBuffer, unsigned char ulen);

	int WriteData(unsigned char *pBuffer, unsigned char uLen);	

	void CloseComm();

	/****校验和计算函数********/
	unsigned char  CheckSum(unsigned char *uBuff, unsigned char uBuffLen); 

	//功能函数
	bool SendAction(char direction,char speed = 0);
	bool SendShoot();
	bool SendLocation(unsigned short angle, char speed);
	bool SendAngle(unsigned short angle, unsigned short nVangle);
	bool SendAngleLocation(short int nHangle, short int  nVangle, short int  nPixelX, short int  nPixelY);
	bool SendOK();

public:
	HANDLE A_hCom;
	unsigned char str[100];
	unsigned char tmpchar[12];
};	


#endif