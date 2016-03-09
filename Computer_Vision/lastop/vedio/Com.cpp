#include "stdafx.h"
#include "MyComClass.h"


int PortOperate::OpenComm(char* port)
{
	A_hCom = CreateFile(port,
		GENERIC_READ|GENERIC_WRITE, //允许读和写
		0, //独占方式
		NULL,
		OPEN_EXISTING, //打开而不是创建
		0, //同步方式
		NULL);
	if(A_hCom == INVALID_HANDLE_VALUE)
	{
		DWORD dwErr = GetLastError();
		printf("opencom failed %d\n",dwErr);
		return FALSE;
	}
	else
	{
		// 打开成功，初始化串口
		DCB wdcb = {0};
		GetCommState(A_hCom, &wdcb); //读取当前串口参数
		wdcb.BaudRate = CBR_115200;         // 波特率
		wdcb.ByteSize = 8;                  // 数据位8
		wdcb.fBinary  = TRUE;				// 二进制方式
		wdcb.fParity  = FALSE;
		wdcb.Parity   = NOPARITY;			// 无奇偶校验
		wdcb.StopBits = ONESTOPBIT;        //1停止位
		//	wdcb.fRtsControl = false;
		//	wdcb.fDtrControl = false;
		//	wdcb.fOutxCtsFlow = false;
		//	wdcb.fOutxDsrFlow = false;
		wdcb.XonLim = 2048;
		wdcb.XoffLim = 512;
		wdcb.EofChar = 0;
		// 设置串口参数
		SetCommState(A_hCom, &wdcb);

		// 设置串口超时参数
		COMMTIMEOUTS to =                   // 串口超时控制参数
		{
			0,                       // 读字符间隔超时时间
			100,                              // 读操作时每字符的时间
			0,                              // 基本的（额外的）读超时时间
			MAXDWORD,                       // 写操作时每字符的时间
			10                               // 基本的（额外的）写超时时间
		};
		SetCommTimeouts(A_hCom, &to);
		// 设置串口缓冲队列
		SetupComm(A_hCom, 1024, 1024);
		// 清空并结束串口当前动作
		PurgeComm(A_hCom, PURGE_TXCLEAR|PURGE_RXCLEAR);
		printf("open com done\n");
		//m_hCom = hCom;						// 保存句柄
	}
	return 0;
}
int PortOperate::ReadData(unsigned char* pBuffer, unsigned char ulen)
{	
	// 从串口读取数据
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
	// 写入数据到串口
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

/****校验和计算函数********/
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
操作接口函数
命令集：
1.01 底盘
2.02 角度，摩擦轮
3.03 发射
4.04 俯仰 角度

指令格式：
上位机->下位机
0xFE 55 AA 命令 数据1 数据2 数据3 数据4 校验和
下位机->上位机
0xAA 接收正确
0x66 接收错误

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