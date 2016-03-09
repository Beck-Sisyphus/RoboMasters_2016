#ifndef ACOM_H
#define ACOM_H


#include "windows.h"
#include <process.h>
#include "string.h"
/***************
���ڹ��ܺ���:
��opencomm
��ȡReadData
д��WriteData
�ر�CloseComm
***************/



int OpenComm(char* port);

int ReadData(HANDLE hCom,unsigned char* pBuffer, unsigned char ulen);

int WriteData(HANDLE hCom, unsigned char *pBuffer, unsigned char uLen);	

void CloseComm(HANDLE hCom);

/****У��ͼ��㺯��********/
unsigned char  CheckSum(unsigned char *uBuff, unsigned char uBuffLen); 


/********�������װ********/

class Operate
{
public:
	Operate()
	{
	}
	~Operate()
	{}
	//�ܳ�ʼ������
	bool Initial(char* port);


	//��������
	bool SendLocation(HANDLE hCom,unsigned char* data);
	bool SendTankNumber(HANDLE hCom,unsigned char* data);
	bool SendVelocity(HANDLE hCom,unsigned char* data);
public:
	int i;

};	





HANDLE A_hCom;//ȫ�־��
unsigned char str[100];//�������ݱ����ַ


#endif