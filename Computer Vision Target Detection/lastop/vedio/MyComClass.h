#ifndef MYCOMCLASS_H
#define MYCOMCLASS_H

#include "windows.h"
#include "MyConstant.h"
//openmp֧��
#include <omp.h>
/********�������װ********/

class PortOperate
{
public:
	PortOperate()
	{
	}
	~PortOperate()
	{}
	//�ܳ�ʼ������
	bool Initial(char* port);
	/***************
	���ڹ��ܺ���:
	��opencomm
	��ȡReadData
	д��WriteData
	�ر�CloseComm
	***************/
	int OpenComm(char* port);

	int ReadData(unsigned char *pBuffer, unsigned char ulen);

	int WriteData(unsigned char *pBuffer, unsigned char uLen);	

	void CloseComm();

	/****У��ͼ��㺯��********/
	unsigned char  CheckSum(unsigned char *uBuff, unsigned char uBuffLen); 

	//���ܺ���
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