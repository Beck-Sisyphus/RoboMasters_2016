//////////////////////////////////////////////////////////////////////
// Video Capture using DirectShow
// Author: Shiqi Yu (shiqi.yu@gmail.com)
// Thanks to:
//		HardyAI@OpenCV China
//		flymanbox@OpenCV China (for his contribution to function CameraName, and frame width/height setting)
// Last modification: April 9, 2009
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// ʹ��˵����
//   1. ��CameraDS.h CameraDS.cpp�Լ�Ŀ¼DirectShow���Ƶ������Ŀ��
//   2. �˵� Project->Settings->Settings for:(All configurations)->C/C++->Category(Preprocessor)->Additional include directories
//      ����Ϊ DirectShow/Include
//   3. �˵� Project->Settings->Settings for:(All configurations)->Link->Category(Input)->Additional library directories
//      ����Ϊ DirectShow/Lib
//////////////////////////////////////////////////////////////////////

#ifndef CCAMERA_H
#define CCAMERA_H

#define WIN32_LEAN_AND_MEAN

#include <atlbase.h>
#include "DirectShow/Include/qedit.h"
#include "DirectShow/Include/dshow.h"
#include <windows.h>
#include <cxcore.h>

#define MYFREEMEDIATYPE(mt)	{if ((mt).cbFormat != 0)		\
					{CoTaskMemFree((PVOID)(mt).pbFormat);	\
					(mt).cbFormat = 0;						\
					(mt).pbFormat = NULL;					\
				}											\
				if ((mt).pUnk != NULL)						\
				{											\
					(mt).pUnk->Release();					\
					(mt).pUnk = NULL;						\
				}}									


class CCameraDS  
{
private:
	IplImage * m_pFrame;
	bool m_bConnected;
	int m_nWidth;
	int m_nHeight;
	bool m_bLock;
	bool m_bChanged;
	long m_nBufferSize;

	CComPtr<IGraphBuilder> m_pGraph;
	CComPtr<IBaseFilter> m_pDeviceFilter;
	CComPtr<IMediaControl> m_pMediaControl;
	CComPtr<IBaseFilter> m_pSampleGrabberFilter;
	CComPtr<ISampleGrabber> m_pSampleGrabber;
	CComPtr<IPin> m_pGrabberInput;
	CComPtr<IPin> m_pGrabberOutput;
	CComPtr<IPin> m_pCameraOutput;
	CComPtr<IMediaEvent> m_pMediaEvent;
	CComPtr<IBaseFilter> m_pNullFilter;
	CComPtr<IPin> m_pNullInputPin;

private:
	bool BindFilter(int nCamIDX, IBaseFilter **pFilter);
	void SetCrossBar();

public:
	CCameraDS();
	virtual ~CCameraDS();

	//������ͷ��nCamIDָ�����ĸ�����ͷ��ȡֵ����Ϊ0,1,2,...
	//bDisplayPropertiesָʾ�Ƿ��Զ���������ͷ����ҳ
	//nWidth��nHeight���õ�����ͷ�Ŀ�͸ߣ��������ͷ��֧�����趨�Ŀ�Ⱥ͸߶ȣ��򷵻�false
	bool CCameraDS::OpenCamera(int nCamID, bool bDisplayProperties=true, int nWidth=320, int nHeight=240);

	//�ر�����ͷ�������������Զ������������
	void CloseCamera();

	//��������ͷ����Ŀ
	//���Բ��ô���CCameraDSʵ��������int c=CCameraDS::CameraCount();�õ������
	static int CameraCount(); 

	//��������ͷ�ı�ŷ�������ͷ������
	//nCamID: ����ͷ���
	//sName: ���ڴ������ͷ���ֵ�����
	//nBufferSize: sName�Ĵ�С
	//���Բ��ô���CCameraDSʵ��������CCameraDS::CameraName();�õ������
	static int CCameraDS::CameraName(int nCamID, char* sName, int nBufferSize);

	//����ͼ����
	int GetWidth(){return m_nWidth;} 

	//����ͼ��߶�
	int GetHeight(){return m_nHeight;}

	//ץȡһ֡�����ص�IplImage�����ֶ��ͷţ�
	//����ͼ�����ݵ�ΪRGBģʽ��Top-down(��һ���ֽ�Ϊ���Ͻ�����)����IplImage::origin=0(IPL_ORIGIN_TL)
	IplImage * QueryFrame();
};

#endif 
