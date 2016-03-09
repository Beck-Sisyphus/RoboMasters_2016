// --------------------------------------------------------------------------
// All or portions of this software are copyrighted by D-IMAGE.
// Copyright 2010-2011 D-IMAGE Corporation.  
// Company proprietary.
// --------------------------------------------------------------------------
//******************************************************************************
/**
 *  \file           DICAMAPI.h
 *  \brief          Defines the API for the D-IMAGE USB2.0 CAMERA DLL application
 *  \author         Jiafu 
 *  \version        \$ Revision: 0.1 \$         
 *  \arg            first implemetation    
 *  \date           2010/07/26 10:52:00  
 Revision | Submission Date | Description of Change
 0x20         2011/5/20       add CameraGetSDKRevision()；debug CameraSetMessage, CameraInit 函数参数位置，使之更合理。
 0x21         2011/6/30       add CameraContinuousCaptureFile()。
 0x22         2011/9/05       ADD SetBMPPelsPerMeter();  
 0x23         2012/01/10      add CameraGrabFrame(),CameraImageProcess(),CameraDisplayRGB24()...
 0X24         2012/02/13      ADD CameraGetMTF();
 0x25         2012/07/23      ADD AWB_ALWAYS_ON MODE
 */
#ifndef _DICAMAPI_H_
#define _DICAMAPI_H_
#include "stdafx.h"
#include "DIDefine.h"

#include<cstring>
// DECLDIR will perform an export for us

#ifdef DLL_EXPORT
	#define DI_API extern "C" __declspec(dllexport) 
#else
	#define DI_API extern "C" __declspec(dllimport) 
#endif
/*==============================================================
Name:	CameraGetSDKRevision
Desc:   .
Param: *pRevision Revision number pointer

Return: Call returns a STATUS_OK on success,otherwise returns an error code
  Note:   
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraGetSDKRevision(IN BYTE *pRevision);

/*==============================================================
Name:	CameraGetCameraNum
Desc:  Get the number of MultiCamera that connected to the PC.

Param:	*CamAllNum      Camera number connected to the PC
        *pResolution    the resolution of the previous work 
		*pCapResolution  the resolution of capture
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note:   
  --------------------------------------------------------------*/

DI_API DI_CAMERA_STATUS _stdcall CameraGetMultiCameraNumber(BYTE *CamAllNum, BYTE *pResolution, BYTE *pCapResolution);

/*==============================================================
Name:	CameraSetMessage
Desc:   .
Param:  MsHWND  control handle which get message
        MessageID Message ID

Return: Call returns a STATUS_OK on success,otherwise returns an error code
  Note:   
  --------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraSetMessage(HWND MsHWND,UINT MessageID);

/*==============================================================
Name:	CameraInit
Desc:   Initialize video equipment

Param:   CamNum        One in MultiCamera	
    	uiResolution  Resolution index
        hWndDisplay	  Video display control handle(Set it to NULL,when don't need to display an image )
		pCallbackFunction  Callback function pointer,called by SDK,users can add the image analysis in callback function.	
		lpContext Callback function context

Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note:   
  --------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall  CameraInit(BYTE CamNum,
											   DI_RESOLUTION uiResolution, 
											   HWND hWndDisplay, 
											   DI_SNAP_PROC pCallbackFunction,
											   LPVOID lpContext
								   );
/*==============================================================
Name:	CameraUnInit
Desc:	Anti-initialization equipment
Param:   
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note:   It must call when exit the program for releasing the memory allocation space.
  --------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall  CameraUnInit(void);

/*==============================================================
Name:	CameraPlay
Desc:	Open the video stream
Param: 
Return: Call returns a STATUS_OK on success,otherwise returns an error code 
Note:   
  --------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall  CameraPlay();

/*==============================================================
Name:	CameraPlay
Desc:	Stop the video stream
Param:   
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note:   
  --------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall  CameraStop(void);


/*==============================================================
Name:	CameraSaveCurrentImage
Desc:	Save current image.
Param:   strFileName  file name(include directory path)
        FileType     File type,specific definition refer FILE_TYPE 
Return: Call returns a STATUS_OK on success,otherwise returns an error code    
Note:   The last image will be saved when the video stream stops , .
  --------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall  CameraSaveCurrentImage(LPCTSTR strFileName, BYTE FileType,IN BYTE Quality);

/*==============================================================
Name:	CameraCaptureFile
Desc:	Capture an image to a file, the file format will change according to FileType
Param:   strFileName  file name(include directory path)
        FileType     File type,specific definition refer FILE_TYPE
		Quality      JEPG compression parameters，range：1 - 100，value larger，image quality better，file memory bigger.
		uiCapResolution resolution， refer DI_RESOLUTION definition
Return: Call returns a STATUS_OK on success,otherwise returns an error code  
Note:  
  --------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall  CameraCaptureFile(IN LPCTSTR strFileName, IN BYTE FileType, IN BYTE Quality,
										  IN DI_RESOLUTION uiCapResolution);

/*==============================================================
Name:	CameraContinuousCaptureFile
Desc:   Continuous Capture an image to a file, the file format will change according to FileType
Param:   strFileName  file name(include directory path)
        FileType     File type,specific definition refer FILE_TYPE
		Number how many frames want capture
Return: Call returns a STATUS_OK on success,otherwise returns an error code  
Note:  
  --------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraContinuousCaptureFile(IN LPCTSTR strFileName,IN  BYTE FileType,IN  BYTE Number);
/*==============================================================
Name:	CameraCaptureToBuf
Desc:	Capture image data to buffer
Param:   

Return: Call returns a STATUS_OK on success,otherwise returns an error code  
Note:  
  --------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraCaptureToBuf(BYTE* pBuffer, DI_DATA_TYPE Type, IN DI_RESOLUTION uiCapResolution);

/*==============================================================
Name:	CameraGetImageSize
Desc:	Read current image size
Param:   *pWidth image width pointer
		*pHeight image height pointer
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note:   
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall  CameraGetImageSize(int *pWidth, int *pHeight);
/*==============================================================
Name:	CameraSetAeState
Desc:	Set AE mode
Param:   bState - TRUE automatic exposure
               - FALSE manual exposure
		
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note:   
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall  CameraSetAeState(IN BOOL bState);
DI_API DI_CAMERA_STATUS _stdcall  CameraGetAeState(BOOL *pAeState);

/*==============================================================
Name:	CameraSetAeTarget
Desc:   Set AE target
Param:   uiAeTarget -AE target
		
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note:   
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraSetAeTarget(IN BYTE uiAeTarget);
DI_API DI_CAMERA_STATUS _stdcall CameraGetAeTarget(IN OUT BYTE *pAeTarget);
/*==============================================================
Name:	CameraSetExposureTime
Desc:   Set exposure time
Param:   uiExposureTime -exposure time
		
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note:   
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraSetExposureTime(IN int uiExposureTime);
DI_API DI_CAMERA_STATUS _stdcall CameraGetExposureTime(IN int *pExposureTime);
/*==============================================================
Name:	CameraGetMaxExposureTime
Desc:   Read max exposure time in line
Param:   *pExposureTime -save exposure time pointer
		
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note:   
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraGetMaxExposureTime(IN USHORT *pMaxExposureTime);

/*==============================================================
Name:	CameraGetRowTime
Desc:   Read the line cycle at current
Param:   
		
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note: Get the current setting line time,units is microseconds(us);
      Can be used to count exposure time.
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraGetRowTime(UINT *pRowTime);
/*==============================================================
Name:	CameraSetAnalogGain
Desc:   Set gain
Param:   usAnalogGain gain
		
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note:   
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraSetAnalogGain(IN USHORT usAnalogGain);
DI_API DI_CAMERA_STATUS _stdcall CameraGetAnalogGain(IN USHORT *pAnalogGain);
/*==============================================================
Name:	CameraSetGamma
Desc:   set GAMMA
Param:   uiGamma 
		
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note:   
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraSetGamma(IN BYTE uiGamma);
DI_API DI_CAMERA_STATUS _stdcall CameraGetGamma(IN BYTE *pGamma);
/*==============================================================
Name:	CameraSetContrast
Desc:   Set contrast
Param:   uiContrast 
		
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note:   
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraSetContrast(IN BYTE uiContrast);
DI_API DI_CAMERA_STATUS _stdcall CameraGetContrast(IN BYTE *pContrast);

/*==============================================================
Name:	CameraSetWBWindow
Desc:   Set white balance window
Param:   HOff Line offset
        VOff Offset
		Width  area width
		Height area height
		
Return:Call returns a STATUS_OK on success,otherwise returns an error code
Note: Default regional is center,a quarter of full-screen 
	m_WBWHOff = m_width>>2;
	m_WBWVOff = m_height>>2;
	m_WBWWidth = m_width>>1;
	m_WBWHeight = m_height>>1;
Notice：
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraSetWBWindow(USHORT HOff, USHORT VOff, USHORT Width, USHORT Height);
/*==============================================================
Name:	CameraSetAWBState
Desc:   Set white balance mode
Prame:   AWBState - AWB_ALWAYS_ON automatic white balance always on
				  - AWB_ONE_TIME one time automatic white balance
                  - AWB_MAUNAL manual white balance
		
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note:   
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraSetAWBState(BYTE AWBState);
DI_API DI_CAMERA_STATUS _stdcall CameraGetAWBState(BYTE *pAWBState);
/*==============================================================
Name:	CameraSetGain
Desc:   Set each color channel gain
Param:   RGain red channel gain
		GGain green channel gain
		BGain blue channel gain
		
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note:   
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraSetGain(IN INT RGain, INT GGain, INT BGain);
DI_API DI_CAMERA_STATUS _stdcall CameraGetGain(IN INT *pRGain, INT *pGGain, INT *pBGain);

/*==============================================================
Name:	CameraSetColorEnhancement
Desc:   Color enhancement
Param:   bEnable - TRUE enable 
                - FALSE 
		
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note:   
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraSetColorEnhancement(IN BOOL bEnable);
DI_API DI_CAMERA_STATUS _stdcall CameraGetColorEnhancement(IN BOOL *pEnable);
/*==============================================================
Name:	CameraSetSaturation
Desc:   Set saturation
Param:   uiSaturation 
		
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note:   
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraSetSaturation(IN BYTE uiSaturation);
DI_API DI_CAMERA_STATUS _stdcall CameraGetSaturation(IN BYTE *pSaturation);

/*==============================================================
Name:	CameraSetMonochrome
Desc:   Set monochrome
Param:   bEnable - TRUE monochrome
                - FALSE cancel monochrome
		
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note:   
--------------------------------------------------------------*/

DI_API DI_CAMERA_STATUS _stdcall CameraSetMonochrome(IN BOOL bEnable);
DI_API DI_CAMERA_STATUS _stdcall CameraGetMonochrome(IN BOOL *pEnable);
/*==============================================================
Name:	CameraSetB2RGBMode
Desc:   Set the transfer mode from BAYER to RGB24.
Param:   Mode --B2RGB_MODE_LAROCHE  base on  Gradient and color interpolation
			--B2RGB_MODE_HAMILTON  Adaptive interpolation
			--B2RGB_MODE_LINE  Linear interpolation
  Return: Call returns a STATUS_OK on success,otherwise returns an error code
  Note:   
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraSetB2RGBMode(DI_B2RGB_MODE Mode);
DI_API DI_CAMERA_STATUS _stdcall CameraGetB2RGBMode(byte *pMode);

/*==============================================================
Name:	CameraSetMirror
Desc:   Set image mirror
Parem:   uiDir direction specified,refer DI_MIRROR_DIRECTION definition,
        bEnable - TRUE mirror
		        - FALSE cancel mirror
		
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note:   
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraSetMirror(IN DI_MIRROR_DIRECTION uiDir, IN BOOL bEnable);
DI_API DI_CAMERA_STATUS _stdcall CameraGetMirror(IN DI_MIRROR_DIRECTION uiDir, IN BOOL *bEnable);
/*==============================================================
Name:	CameraSetFrameSpeed
Desc:   Set frame speed
Param:   FrameSpeed index,Specific definitions refer DI_FRAME_SPEED
		
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note:   
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraSetFrameSpeed(IN DI_FRAME_SPEED FrameSpeed);
DI_API DI_CAMERA_STATUS _stdcall CameraGetFrameSpeed(IN BYTE *pFrameSpeed);

/*==============================================================
Name:	CameraSetLightFrquency
Desc:   Set light frquency
Param:   
		
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note:   It is effective in automatic expoture.
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraSetLightFrquency(IN DI_LIGHT_FREQUENCY LightFrequency );
DI_API DI_CAMERA_STATUS _stdcall CameraGetLightFrquency(IN BYTE *pLightFrequency );
/*==============================================================
Name:	CameraSaveParameter
Desc:   Save parameters to team ?
Param:   Team -team
		
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note:   
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraSaveParameter(IN DI_PARAMETER_TEAM Team);
DI_API DI_CAMERA_STATUS _stdcall CameraReadParameter(IN DI_PARAMETER_TEAM Team);
/*==============================================================
Name:	CameraGetCurrentParameterTeam
Desc:   Read current parameter team
Param:   
		
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note:   
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraGetCurrentParameterTeam(IN BYTE *pTeam);
/*==============================================================
Name:	CameraSetGpio
Desc:   Configure the input and output of GPIO
Param:   bValue －－bit0 to IO3
               －－bit1 to IO2
		
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note:   input is 0，output is 1；
        such as：CameraSetGpio(0x01)--configure IO3 as output，IO2 as input.
        
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraSetGpio(IN BYTE Value);
/*==============================================================
Name:	CameraSetGpio
Desc:   Read GPIO
Param:   bValue －－bit0 to IO3
               －－bit1 to IO2
		
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note:   Configure the appropriate IO port as output before read.
        
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraReadGpio(IN BYTE *pbValue);
/*==============================================================
Name:	CameraSetGpio
Desc:   Write GPIO
Param:   bValue －－bit0 to IO3
               －－bit1 to IO2
		
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note:   Configure the appropriate IO port as output before read.
        
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraWriteGpio(IN BYTE Value);
/*==============================================================
Name:	CameraSetROI
Desc:   Set ROI area
Param:   HOff Line offset
        VOff Offset
		Width area width
		Height area height
		
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note: 
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraSetROI(USHORT HOff, USHORT VOff, USHORT Width, USHORT Height);
DI_API DI_CAMERA_STATUS _stdcall CameraGetROI(USHORT *pHOff, USHORT *pVOff, USHORT *pWidth, USHORT *pHeight);

/*==============================================================
Name:	CameraSetDisplayWindow
Desc:   Set display window
Param:   HOff Line offset
        VOff Offset
		Width area width
		Height area height
		
Return: Call returns a STATUS_OK on success,otherwise returns an error code
Note: full resolution；
Notice：The function is effective when call CameraInit（）.
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraSetDisplayWindow(USHORT HOff, USHORT VOff, USHORT Width, USHORT Height);

/*==============================================================
Name:	CameraEnableDeadPixelCorrection
Desc:   initializatize dead pixel correction.
Param:   no

  Return: Call returns a STATUS_OK on success,otherwise returns an error code
  Note:   When need to dead pixel correction,first cover the lens with shade cap,turn off auto-exposure ,set the exposure time and gain to appropriate value.
  And then call the fuction to initialize,the program willautomatically find and record the position of dead pixel.
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraEnableDeadPixelCorrection(void);

/*==============================================================
Name:	CameraWriteSN
Desc:   Write product number.
Param:   

  Return: Call returns a STATUS_OK on success,otherwise returns an error code
  Note: make sure eeprom is writeable
        the SNCnt max value is 32  
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraWriteSN(char *pSN, BYTE SNCnt);
/*==============================================================
Name:	CameraReadSN
Desc:   read product number
Param:   

Return: Call returns a STATUS_OK on success,otherwise returns an error code
  Note:   
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraReadSN(char *pSN, BYTE SNCnt);

/*==============================================================
Name:	CameraLoadDefault
Desc:   load default camera setting
Param:   

Return: Call returns a STATUS_OK on success,otherwise returns an error code
  Note:   
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraLoadDefault(void);

/*==============================================================
Name:	CameraGetFPS
Desc:   Get Camera frame speed
Param:   

Return: Call returns a STATUS_OK on success,otherwise returns an error code
  Note:   
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraGetFPS(float* fps);
/*==============================================================
Name: Save***File
Desc: save image data to file  
Param: sfilename -- file name exp: d://test//test.jpg  
       pBuffer -- 数据指针
       width -- 宽度
	   height -- 高度
       qualty -- JPG图像质量 范围：0-100 值越大，压缩比越小，文件越大，图像质量越高。 
Return: Call returns a STATUS_OK on success,otherwise returns an error code
  Note:   
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraSaveBMPFile(LPCSTR sfilename, BYTE *pBuffer, UINT width, UINT height);
DI_API DI_CAMERA_STATUS _stdcall CameraSaveJPEG(LPCSTR sfilename, BYTE *pBuffer, UINT width, UINT height, INT qualty);
DI_API DI_CAMERA_STATUS _stdcall CameraSavePNG(LPCSTR sfilename, BYTE *pBuffer, UINT width, UINT height);
DI_API DI_CAMERA_STATUS _stdcall CameraSaveRawFile(LPCSTR sfilename, BYTE *pRaw, int width, int height);

/*==============================================================
Name:	RecordAviStart
Desc:   AVI Record start
Param:   

Return: Call returns a STATUS_OK on success,otherwise returns an error code
  Note:   
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall RecordAviStart(LPCSTR  csFileName);
DI_API DI_CAMERA_STATUS _stdcall RecordAviStop(void);

/*==============================================================
Name:	SetBMPPelsPerMeter
Desc:   set BMP DPI 
Param:   

Return: Call returns a STATUS_OK on success,otherwise returns an error code
  Note:  this is pixel per meter.  3780--96dpi
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall SetBMPPelsPerMeter(LONG iXPelsPerMeter, LONG iYPelsPerMeter);

/*==============================================================
函数:	CameraGrabFrame
功能:  抓取图像数据
参数:  pImageBuffer 用户分配数据空间指针 

  返回值: 调用成功返回STATUS_OK 否则返回错误代码
  说明:
 
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraGrabFrame(BYTE *pImageBuffer);
/*==============================================================
函数:	CameraImageProcess
功能:  图像数据处理
参数: pRawBuffer 原始图像数据
      pImageRGB24 处理后图像数据空间指针  

  返回值: 调用成功返回STATUS_OK 否则返回错误代码
  说明:
 
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraImageProcess(BYTE *pRawBuffer, BYTE *pImageRGB24);
/*==============================================================
函数:	CameraDisplayRGB24
功能:  显示RGB24数据到显示窗口
参数:  pImageRGB24 图像数据指针 

  返回值: 调用成功返回STATUS_OK 否则返回错误代码
  说明:
 
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraDisplayRGB24( BYTE *pImageRGB24 );
/*==============================================================
函数:	CameraGetMTF
功能:  读取图像清晰度评估参数
参数: pMTF 数据指针 

  返回值: 调用成功返回STATUS_OK 否则返回错误代码
  说明:
 
--------------------------------------------------------------*/
DI_API DI_CAMERA_STATUS _stdcall CameraGetMTF(BYTE *pMTF);

#endif