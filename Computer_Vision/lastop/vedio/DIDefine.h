// --------------------------------------------------------------------------
// All or portions of this software are copyrighted by D-IMAGE.
// Copyright 1996-2008 D-IMAGE Corporation.  
// Company proprietary.
// --------------------------------------------------------------------------
//******************************************************************************
/**
*  \file           DICAMAPI.h
*  \brief          Defines for the D-IMAGE USB2.0 Camera application
*  \author         Mike 
*  \version        \$ Revision: 1.1 \$         
*  \arg            first implemetation    
*  \date           2011/2/08 10:52:00     
*/
#ifndef _DIDEFINE_H_
#define _DIDEFINE_H_
#include <Windows.h>
//#define D1400C
//#define D1000C
//#define D900C
//#define D800C
//#define D500C
#define D300C
//#define D210C
//#define D200CR
//#define D130C
//#define D036C
//parameter

#ifdef D1400C
	#define FULL_WIDTH  4384
	#define FULL_HEIGHT 3288
	typedef enum tagDI_RESOLUTION
	{
		R4384_3288,
			R2048_1536,
			R1280_960,
			R1024_768,
			R640_480,
			R_ROI,
	}DI_RESOLUTION;
#endif

#ifdef D1000C
	#define FULL_WIDTH  3664
	#define FULL_HEIGHT 2748
	typedef enum tagDI_RESOLUTION
	{
		R3664_2748,
			R1600_1200,
			R1280_960,
			R800_600,
			R640_480,
			R_ROI,			
	}DI_RESOLUTION;
#endif

#ifdef D900C
	#define FULL_WIDTH  3488
	#define FULL_HEIGHT 2616

	typedef enum tagDI_RESOLUTION
	{
		R3488_2616,
			R1600_1200,
			R1280_960,
			R800_600,
			R640_480,
			R_ROI,
	}DI_RESOLUTION;
#endif

#ifdef D800C
	#define FULL_WIDTH  3264
	#define FULL_HEIGHT 2448

	typedef enum tagDI_RESOLUTION
	{
		R3264_2448,
			R1600_1200,
			R1280_960,
			R800_600,
			R640_480,
			R_ROI,
	}DI_RESOLUTION;
#endif

#ifdef D500C
	#define FULL_WIDTH  2592
	#define FULL_HEIGHT 1944

	typedef enum tagDI_RESOLUTION
	{
		R2592_1944=0,
			R1280_960,
			R1024_768,
			R640_480,
			R_ROI,
	}DI_RESOLUTION;
#endif

#ifdef D300C
	#define FULL_WIDTH  2048
	#define FULL_HEIGHT 1536

	typedef enum tagDI_RESOLUTION
	{
		R2048_1536=0,
			R1024_768,
			R640_480,
			R_ROI,
			R320_240,
	}DI_RESOLUTION;
#endif

#ifdef D210C
	#define FULL_WIDTH 1600
	#define FULL_HEIGHT 1200
	typedef enum tagDI_RESOLUTION
	{
		R1600_1200=0,
			R800_600,
			R640_480,
			R_ROI,
	}DI_RESOLUTION;
#endif

#ifdef D200CR
	#define FULL_WIDTH 1600
	#define FULL_HEIGHT 1200
	typedef enum tagDI_RESOLUTION
	{
		R1600_1200=0,
			R800_600,
			R_ROI,
	}DI_RESOLUTION;
#endif
#ifdef D130C
	#define FULL_WIDTH 1280
	#define FULL_HEIGHT 1024

	typedef enum tagDI_RESOLUTION
	{
		R1280_1024 = 0,
			R640_480,
			R320_240,
			R_ROI
	}DI_RESOLUTION;
#endif

#ifdef D036C
	#define FULL_WIDTH 752
	#define FULL_HEIGHT 480
	
	typedef enum tagDI_RESOLUTION
	{
		R752_480 = 0,
			R640_480,
			R320_240,
			R_ROI
	}DI_RESOLUTION;
#endif
//parameter
typedef enum tagDI_RUNMODE
{
		RUNMODE_PLAY=0,
		RUNMODE_PAUSE,
		RUNMODE_STOP,
}DI_RUNMODE;

typedef enum tagDI_CAMERA_STATUS
{
		STATUS_OK = 1,                         //动作成功
		STATUS_INTERNAL_ERROR = 0,             //内部错误
		STATUS_NO_DEVICE_FIND = -1,            //没有发现相机
		STATUS_NOT_ENOUGH_SYSTEM_MEMORY = -2,  //没有足够系统内存  
		STATUS_HW_IO_ERROR = -3,               //硬件IO错误
		STATUS_PARAMETER_INVALID = -4,         //参数无效
		STATUS_PARAMETER_OUT_OF_BOUND = -5,    //参数越界
		STATUS_FILE_CREATE_ERROR = -6,         //创建文件失败
		STATUS_FILE_INVALID = -7,              //文件格式无效
		
}DI_CAMERA_STATUS;

typedef enum tagDI_MIRROR_DIRECTION
{
		MIRROR_DIRECTION_HORIZONTAL = 0,
		MIRROR_DIRECTION_VERTICAL = 1,
}DI_MIRROR_DIRECTION;

typedef enum tagDI_FRAME_SPEED
{
		FRAME_SPEED_NORMAL = 0,
		FRAME_SPEED_HIGH = 1,
		FRAME_SPEED_SUPER = 2,
		FRAME_SPEED_SUPER2,
}DI_FRAME_SPEED;

typedef enum tagDI_FILE_TYPE
{
		FILE_JPG = 1,
		FILE_BMP = 2,
		FILE_RAW = 4,
		FILE_PNG = 8,
}DI_FILE_TYPE;

typedef enum tagDI_DATA_TYPE
{
		DATA_TYPE_RAW = 0,
		DATA_TYPE_RGB24 = 1,
}DI_DATA_TYPE;

typedef enum tagDI_SNAP_MODE { 
		SNAP_MODE_CONTINUATION	= 0,
		SNAP_MODE_SOFT_TRIGGER	= 1,
		SNAP_MODE_EXTERNAL_TRIGGER	= 2,//external
} DI_SNAP_MODE;

typedef enum tagDI_LIGHT_FREQUENCY{
		LIGHT_FREQUENCY_50HZ = 0,
		LIGHT_FREQUENCY_60HZ = 1,
}DI_LIGHT_FREQUENCY;

typedef enum tagDI_SUBSAMPLE_MODE{
		SUBSAMPLE_MODE_BIN = 0,
		SUBSAMPLE_MODE_SKIP = 1,
}DI_SUBSAMPLE_MODE;

typedef enum tagDI_COLOR_CH{
		COLOR_CH_G1 = 0,
		COLOR_CH_G2 = 1,
		COLOR_CH_R = 2,
		COLOR_CH_B = 3,
}DI_COLOR_CH;

typedef enum tagDI_PARAMETER_TEAM{
		PARAMETER_TEAM_A = 0,
		PARAMETER_TEAM_B = 1,
		PARAMETER_TEAM_C = 2,
		PARAMETER_TEAM_D = 3,
}DI_PARAMETER_TEAM;

typedef enum tagDI_STROBE_MODE{
	STROBE_MODE_AUTO = 0,
	STROBE_MODE_OFF  = 1,
}DI_STROBE_MODE;

typedef enum tagDI_B2RGB_MODE{
	B2RGB_MODE_LINE	 = 0,
		B2RGB_MODE_HAMILTON = 1,
		B2RGB_MODE_LAROCHE= 2,
		B2RGB_MODE_DIRECTLY = 3,
}DI_B2RGB_MODE;

typedef enum tagDI_CAMERA_MSG{
	CAMERA_MSG_VIDEO = 0,
			CAMERA_MSG_AVI = 1,
}DI_CAMERA_MSG;

typedef enum tagDI_RECORDMODE
{
	RECORDMODE_START=0,
		RECORDMODE_STOP,
}tagDI_RECORDMODE;

typedef enum tagDI_AWB_MODE
{
	AWB_MAUNAL=0,
		AWB_ONE_TIME= 1,
		AWB_ALWAYS_ON = 2,
}tagDI_AWB_MODE;

typedef int (CALLBACK* DI_SNAP_PROC)(BYTE *pImageBuffer, DI_DATA_TYPE TYPE, LPVOID lpContext);

#endif