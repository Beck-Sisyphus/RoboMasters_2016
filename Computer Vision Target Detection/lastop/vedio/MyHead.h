#ifndef MYHEAD_H
#define MYHEAD_H

#include "cv.h"
#include "highgui.h"
#include "windows.h"
#include "stdlib.h"
#include <fstream>
#include <iostream>
#include <opencv2/legacy/legacy.hpp>
#include <stack>
#include<queue>

#pragma comment(lib, "opencv_legacy249.lib")

//#pragma comment (lib,"D:\\opencv\\opencv\\DirectShow\\Lib\\strmiids.lib")
using namespace std;

#define Fx 817.74458
#define Fy 818.04921	
#define Cx 314.85848
#define Cy 227.90983
#define K1 0.02061
#define K2 -0.29551
#define P1 0.00385
#define P2 0.00028


struct PixelNode
{
	int NeighboorNum;
	int Neighboor[12];
	struct PixelNode()
	{
		NeighboorNum = 0;
		for (int i = 0; i<12; i++)
		{
			Neighboor[i] = 0;
		}
	}
};
struct SDisjointSetNode
{
	int Parent;
	int Rank;
	struct SDisjointSetNode()
	{
		Parent = 0;
		Rank = 0;
	}
};
struct SLabelNode
{
	int LabelVal;
	int LabelCounter;
};

struct SObstacle
{
	CvPoint FrPt;
	CvPoint NrPt;
	CvPoint CnPt;
	int nWidth;
	int nHeight;
	float fDist;
	float fHAngle;
	float fVAngle;
	float fVx;
	float fVy;
};

#endif