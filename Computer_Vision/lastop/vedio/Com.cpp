#include "stdafx.h"
#include "MyComClass.h"


int PortOperate::OpenComm(char* port)
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
		printf("opencom failed %d\n",dwErr);
		return FALSE;
	}
	else
	{
		// �򿪳ɹ�����ʼ������
		DCB wdcb = {0};
		GetCommState(A_hCom, &wdcb); //��ȡ��ǰ���ڲ���
		wdcb.BaudRate = CBR_115200;         // ������
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
			0,                       // ���ַ������ʱʱ��
			100,                              // ������ʱÿ�ַ���ʱ��
			0,                              // �����ģ�����ģ�����ʱʱ��
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
int PortOperate::ReadData(unsigned char* pBuffer, unsigned char ulen)
{	
	// �Ӵ��ڶ�ȡ����
	DWORD dwRead;
	memset(str,0x00,sizeof(str));
	if (!ReadFile(A_hCom, pBuffer, ulen, &dwRead, NULL))
	{
		DWORD dwErr = GetLastError();
		//printf("%ld\n",dwErr);
		printf("read error %ld\n",dwErr);
		return dwErr;
	}
	//printf("read done %d\n",dwRead);
	//	PurgeComm(A_hCom, PURGE_TXCLEAR|PURGE_RXCLEAR);
	return 0;
}
int PortOperate::WriteData(unsigned char *pBuffer, unsigned char uLen)
{	
	// д�����ݵ�����
	DWORD dwWritten;
	if (uLen > 0)
	{
		dwWritten = 0; 
		if (!WriteFile(A_hCom, pBuffer, uLen, &dwWritten, NULL))
		{
			DWORD dwErr = GetLastError();
			printf("wirte error %ld\n",dwErr);
			return dwErr;
		}
		else{
		//	printf("write done %d\n",dwWritten);
		}
	}
	//	PurgeComm(A_hCom, PURGE_TXCLEAR|PURGE_RXCLEAR);
	return 0;
}
void PortOperate::CloseComm()
{		
	CloseHandle(A_hCom);
	A_hCom = NULL;
}

/****У��ͼ��㺯��********/
unsigned char PortOperate::CheckSum(unsigned char *uBuff, unsigned char uBuffLen) 
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
1.01 ����
2.02 �Ƕȣ�Ħ����
3.03 ����
4.04 ���� �Ƕ�

ָ���ʽ��
��λ��->��λ��
0xFE 55 AA ���� ����1 ����2 ����3 ����4 У���
��λ��->��λ��
0xAA ������ȷ
0x66 ���մ���

*********************/
bool PortOperate::Initial(char* port)
{
	OpenComm(port);
	return 0;
}
bool PortOperate::SendAction(char direction, char speed)
{
	memset(tmpchar, 0x00, sizeof(tmpchar));
	tmpchar[0] = 0xFE;
	tmpchar[1] = 0x55;
	tmpchar[2] = 0xAA;
	tmpchar[3] = 0x01;
	tmpchar[4] = speed << 7 | direction;
	tmpchar[8] = CheckSum(tmpchar,9);
	WriteData(tmpchar, 9);
	return 0;
}

bool PortOperate::SendShoot()
{
	memset(tmpchar, 0x00, sizeof(tmpchar));
	tmpchar[0] = 0xFE;
	tmpchar[1] = 0x55;
	tmpchar[2] = 0xAA;
	tmpchar[3] = 0x03;
	tmpchar[8] = CheckSum(tmpchar, 9);
	WriteData(tmpchar, 9);
	return 0;
}
bool PortOperate::SendLocation(unsigned short angle, char speed)
{
	memset(tmpchar, 0x00, sizeof(tmpchar));
	tmpchar[0] = 0xFE;
	tmpchar[1] = 0x55;
	tmpchar[2] = 0xAA;
	tmpchar[3] = 0x02;
	tmpchar[4] = angle >> 8;
	tmpchar[5] = angle & 0x00ff;
	tmpchar[6] = speed;
	tmpchar[8] = CheckSum(tmpchar, 9);
	WriteData(tmpchar, 9);
	return 0;
}

bool PortOperate::SendAngle(unsigned short nHangle, unsigned short nVangle)
{
	memset(tmpchar, 0x00, sizeof(tmpchar));
	tmpchar[0] = 0xAA;
	tmpchar[1] = 0x55;
	tmpchar[2] = nHangle & 0x00ff;
	tmpchar[3] = nHangle >> 8;
	tmpchar[4] = nVangle & 0x00ff;
	tmpchar[5] = nVangle >> 8;
	//tmpchar[5] = speed;
	//tmpchar[6] = speed;
	tmpchar[6] = 0xBB;
	WriteData(tmpchar, 7);
	return 0;
}

bool PortOperate::SendAngleLocation(short int nHangle, short int  nVangle, short int  nPixelX, short int  nPixelY)
{
	memset(tmpchar, 0x00, sizeof(tmpchar));
	tmpchar[0] = 0xAA;
	tmpchar[1] = 0x55;
	tmpchar[2] = nHangle & 0x00ff;
	tmpchar[3] = nHangle >> 8;
	tmpchar[4] = nVangle & 0x00ff;
	tmpchar[5] = nVangle >> 8;
	tmpchar[6] = nPixelX & 0x00ff;
	tmpchar[7] = nPixelX >> 8;
	tmpchar[8] = nPixelY & 0x00ff;
	tmpchar[9] = nPixelY >> 8;
	//tmpchar[5] = speed;
	//tmpchar[6] = speed;
	tmpchar[10] = 0xBB;
	WriteData(tmpchar, 11);
	return 0;
}

bool PortOperate::SendOK()
{
	memset(tmpchar, 0x00, sizeof(tmpchar));
	tmpchar[0] = 0xAA;
	tmpchar[1] = 0x55;
	tmpchar[2] = 0x01;
	tmpchar[3] = 0xBB;
	//tmpchar[4] = nVangle & 0x00ff;
	//tmpchar[5] = nVangle >> 8;
	//tmpchar[5] = speed;
	//tmpchar[6] = speed;
	//tmpchar[4] = 0xBB;
	WriteData(tmpchar, 4);
	return 0;
}