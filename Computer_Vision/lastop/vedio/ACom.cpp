#include "stdafx.h"
#include "MyHead.h"

extern HANDLE A_hCom;
extern unsigned char str[100];
int OpenComm(char* port)
{
	A_hCom = CreateFile(port,
		GENERIC_READ|GENERIC_WRITE, //�������д
		0, //��ռ��ʽ
		NULL,
		OPEN_EXISTING, //�򿪶����Ǵ���
		0, //ͬ����ʽ
		NULL);
	if(A_hCom == INVALID_HANDLE_VALUE)
	{
		DWORD dwErr = GetLastError();
		printf("opencomm failed %d\n",dwErr);
		return FALSE;
	}
	else
	{
		// �򿪳ɹ�����ʼ������
		DCB wdcb = {0};
		GetCommState(A_hCom, &wdcb); //��ȡ��ǰ���ڲ���
		wdcb.BaudRate = CBR_57600;         // ������
		wdcb.ByteSize = 8;                  // ����λ8
		wdcb.fBinary  = TRUE;				// �����Ʒ�ʽ
		wdcb.fParity  = FALSE;
		wdcb.Parity   = NOPARITY;			// ����żУ��
		wdcb.StopBits = ONESTOPBIT;        //1ֹͣλ
		//	wdcb.fRtsControl = false;
		//	wdcb.fDtrControl = false;
		//	wdcb.fOutxCtsFlow = false;
		//	wdcb.fOutxDsrFlow = false;
		wdcb.XonLim = 2048;
		wdcb.XoffLim = 512;
		wdcb.EofChar = 0;
		// ���ô��ڲ���
		SetCommState(A_hCom, &wdcb);

		// ���ô��ڳ�ʱ����
		COMMTIMEOUTS to =                   // ���ڳ�ʱ���Ʋ���
		{
			MAXDWORD,                       // ���ַ������ʱʱ��
			10,                              // ������ʱÿ�ַ���ʱ��
			10,                              // �����ģ�����ģ�����ʱʱ��
			MAXDWORD,                       // д����ʱÿ�ַ���ʱ��
			10                               // �����ģ�����ģ�д��ʱʱ��
		};
		SetCommTimeouts(A_hCom, &to);
		// ���ô��ڻ������
		SetupComm(A_hCom, 1024, 1024);
		// ��ղ��������ڵ�ǰ����
		PurgeComm(A_hCom, PURGE_TXCLEAR|PURGE_RXCLEAR);
		printf("open com done\n");
		//m_hCom = hCom;						// ������
	}
	return 0;
}
int ReadData(HANDLE hCom,unsigned char* pBuffer, unsigned char ulen)
{	
	// �Ӵ��ڶ�ȡ����
	DWORD dwRead;
	if (!ReadFile(hCom, pBuffer, ulen, &dwRead, NULL))
	{
		DWORD dwErr = GetLastError();
		//printf("%ld\n",dwErr);
		printf("read error %ld\n",dwErr);
		return dwErr;
	}
	//	printf("read done %d\n",dwRead);
	//	PurgeComm(A_hCom, PURGE_TXCLEAR|PURGE_RXCLEAR);
	return 0;
}
int WriteData(HANDLE hCom, unsigned char *pBuffer, unsigned char uLen)
{	
	// д�����ݵ�����
	DWORD dwWritten;
	if (uLen > 0)
	{
		dwWritten = 0;
		if (!WriteFile(hCom, pBuffer, uLen, &dwWritten, NULL))
		{
			DWORD dwErr = GetLastError();
			printf("wirte error %ld\n",dwErr);
			return dwErr;
		}
		else{
			//			printf("write done %d\n",dwWritten);
		}
	}
	//	PurgeComm(A_hCom, PURGE_TXCLEAR|PURGE_RXCLEAR);
	return 0;
}
void CloseComm(HANDLE hCom)
{		
	CloseHandle(hCom);
	hCom = NULL;
}

/****У��ͼ��㺯��********/
unsigned char  CheckSum(unsigned char *uBuff, unsigned char uBuffLen) 
{
	unsigned char i,uSum=0;
	for(i=3;i<uBuffLen;i++)
	{
		uSum = uSum + uBuff[i];
	}
	uSum = (~uSum) + 1;
	if(uSum > 0xf0)
		uSum -= 16;
	//	printf("%X\n",uSum);
	return uSum;
}

/********************
�����ӿں���
�����
1.01 ��λ����
2.02 �ٶȣ�λ��
3.04 ͼƬ

ָ���ʽ��
��λ��->��λ��
0xFE 55 AA ���� ����1 ����2 ����3 ����4 У���
��λ��->��λ��
0xAA ������ȷ
0x66 ���մ���

*********************/


bool Operate::Initial(char* port)
	{
		OpenComm(port);
		//	printf("%d",1);
		return 0;
	}

bool Operate::SendLocation(HANDLE hCom,short X,short Y)
{
		int i;
		unsigned char tmpchar[9];
		memset(tmpchar, 0x00, sizeof(tmpchar));
		tmpchar[0] = 0xFE;
		tmpchar[1] = 0x55;
		tmpchar[2] = 0xAA;
		tmpchar[3] = 0x01;
		tmpchar[4] = X >> 8;
		tmpchar[5] = X & 0x00ff;
		tmpchar[6] = Y >> 8;
		tmpchar[7] = Y & 0x00ff;
		//tmpchar[7] = 
		tmpchar[8] = CheckSum(tmpchar,9);
		WriteData(hCom,tmpchar, 9);
		ReadData(hCom,str,100);
		for (i=0;i<20;i++)
		{
			printf("%X ",str[i]);
		}	
		printf("\n");
		return 0;
}
bool Operate::SendVelocity(HANDLE hCom,unsigned char Vel,short Locate)
{
	int i;
	unsigned char tmpchar[9];
	memset(tmpchar, 0x00, sizeof(tmpchar));
	tmpchar[0] = 0xFE;
	tmpchar[1] = 0x55;
	tmpchar[2] = 0xAA;
	tmpchar[3] = 0x02;
	tmpchar[4] = Vel;
	tmpchar[5] = Locate >> 8;
	tmpchar[6] = Locate & 0x00ff;
	//tmpchar[7] = 
	tmpchar[8] = CheckSum(tmpchar,9);
	WriteData(hCom,tmpchar, 9);
	ReadData(hCom,str,100);
	for (i=0;i<20;i++)
	{
		printf("%X ",str[i]);
	}	
	printf("\n");
	return 0;
}
bool Operate::SendTankNumber(HANDLE hCom,unsigned char* data)
{
	int i;
	unsigned char tmpchar[9];
	memset(tmpchar, 0x00, sizeof(tmpchar));
	tmpchar[0] = 0xFE;
	tmpchar[1] = 0x55;
	tmpchar[2] = 0xAA;
	tmpchar[3] = 0x04;
	tmpchar[4] = data[0];
	//tmpchar[5] = 0xC3;
	//tmpchar[6] = 0x12;
	//tmpchar[7] = 
	tmpchar[8] = CheckSum(tmpchar,9);
	WriteData(hCom,tmpchar, 9);
	ReadData(hCom,str,100);
	for (i=0;i<20;i++)
	{
		printf("%X ",str[i]);
	}	
	printf("\n");
	return 0;
}

