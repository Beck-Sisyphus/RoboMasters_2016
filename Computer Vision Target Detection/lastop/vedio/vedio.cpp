//根据颜色检测识别敌方车辆


#include "stdafx.h"
#include "MyHead.h"
#include "omp.h"
#include "MyComClass.h"

//函数输出模式
#define PRINT_PARAM 1
#define NOT_PRINT 0

#define defaultNbSamples 20		//每个像素点的样本个数
#define defaultReqMatches 2		//#min指数
#define defaultRadius 20		//Sqthere半径
#define defaultSubsamplingFactor 16	//子采样概率
#define background 0		//背景像素
#define foreground 255		//前景像素
#define T_ROW_SEGMENT_THRESHOLD 50		//聚类阈值
#define T_COL_SEGMENT_THRESHOLD 50		//聚类阈值
#define T_VOTE_STHRESHOLD 0.2		//判断是否目标矩形框的阈值
#define T_SUM_STHRESHOLD 19
#define T_COLOUR_THRESHOLD 1      //颜色分类阈值
#define T_IMAGE_WIDTH 640      //图像宽度
#define T_IMAGE_HEIGHT 480     //图像高度
#define T_RECT_MERGE   50      //两个目标矩形可以合并的距离阈值
#define T_RED_A     5
#define T_RED_B     170
#define T_RED_C     0
#define T_RED_D     175
#define T_RED_E     180
#define T_BLUE_LOW     100
#define T_BLUE_HIGH     110
#define max_corners 50
#define T_RED_COLOUR FALSE     //
//#define T_IMAGE_CENTER_X     180
//#define T_IMAGE_CENTER_Y     180


using namespace cv;

IplImage* RgbImage = cvCreateImage(cvSize(T_IMAGE_WIDTH, T_IMAGE_HEIGHT), IPL_DEPTH_8U, 3);
IplImage* GrayImage = cvCreateImage(cvSize(T_IMAGE_WIDTH, T_IMAGE_HEIGHT), IPL_DEPTH_8U, 1);
IplImage* LastGrayImage = cvCreateImage(cvSize(T_IMAGE_WIDTH, T_IMAGE_HEIGHT), IPL_DEPTH_8U, 1);
IplImage* DiffImage = cvCreateImage(cvSize(T_IMAGE_WIDTH, T_IMAGE_HEIGHT), IPL_DEPTH_8U, 1);
IplImage* ThresholdImage = cvCreateImage(cvSize(T_IMAGE_WIDTH, T_IMAGE_HEIGHT), IPL_DEPTH_8U, 1);
IplImage* VibeImage = cvCreateImage(cvSize(T_IMAGE_WIDTH, T_IMAGE_HEIGHT), IPL_DEPTH_8U, 1);
IplImage* TickImage = cvCreateImage(cvSize(T_IMAGE_WIDTH, T_IMAGE_HEIGHT), IPL_DEPTH_8U, 1);
IplImage* TmpImage = cvCreateImage(cvSize(T_IMAGE_WIDTH, T_IMAGE_HEIGHT), IPL_DEPTH_8U, 1);
IplImage* EDImage = cvCreateImage(cvSize(T_IMAGE_WIDTH, T_IMAGE_HEIGHT), IPL_DEPTH_8U, 1);
IplImage* pImgTst = cvCreateImage(cvSize(T_IMAGE_WIDTH, T_IMAGE_HEIGHT), IPL_DEPTH_8U, 3);
IplImage* pImgHsv = cvCreateImage(cvSize(T_IMAGE_WIDTH, T_IMAGE_HEIGHT), IPL_DEPTH_8U, 3);
CvMat* GrayMat = cvCreateMat(T_IMAGE_HEIGHT, T_IMAGE_WIDTH, CV_32FC1);
CvMat* SegMat = cvCreateMat(T_IMAGE_HEIGHT, T_IMAGE_WIDTH, CV_32FC1);
int Thre;
RNG rng;
int nTolHeight, nTolWidth;
int nQueueCount;
static int c_xoff[9] = { -1, 0, 1, -1, 1, -1, 0, 1, 0 };//x的邻居点
static int c_yoff[9] = { -1, 0, 1, -1, 1, -1, 0, 1, 0 };//y的邻居点
float samples[1024][1024][defaultNbSamples + 1];//保存每个像素点的样本值
queue <SObstacle> qObs;     //存储过去N帧的跟踪结果
vector<SObstacle> vNewObs, vOldObs;
vector<int> vNewCount, vOldCount;

BOOL IntsectRect(SObstacle sObs1, SObstacle sObs2)
{
	SObstacle TmpObs;
	TmpObs = sObs2;
	if ((sObs1.FrPt.y >= sObs2.NrPt.y && sObs2.FrPt.y >= sObs1.NrPt.y) && (sObs1.FrPt.x >= sObs2.NrPt.x && sObs2.FrPt.x >= sObs1.NrPt.x))
	{
		return TRUE;
	}
	else
		return FALSE;
}

float RectDist(SObstacle & sObs1, SObstacle sObs2)
{
	float fDis,dX,dY;
	if (sObs1.FrPt.x >= sObs2.NrPt.x && sObs2.FrPt.x >= sObs1.NrPt.x)
	{
		if (sObs1.FrPt.y >= sObs2.NrPt.y && sObs2.FrPt.y >= sObs1.NrPt.y)
		{
			fDis = 0;
		}
		else
		{
			fDis = (sObs1.FrPt.y < sObs2.NrPt.y) ? (sObs2.NrPt.y - sObs1.FrPt.y) : (sObs1.NrPt.y - sObs2.FrPt.y);
		}
	}
	else
	{
		if (sObs1.FrPt.y >= sObs2.NrPt.y && sObs2.FrPt.y >= sObs1.NrPt.y)
		{
			fDis = (sObs1.FrPt.x < sObs2.NrPt.x) ? (sObs2.NrPt.x - sObs1.FrPt.x) : (sObs1.NrPt.x - sObs2.FrPt.x);
		}
		else
		{
			dX = (sObs1.FrPt.x < sObs2.NrPt.x) ? (sObs2.NrPt.x - sObs1.FrPt.x) : (sObs1.NrPt.x - sObs2.FrPt.x);
			dY = (sObs1.FrPt.y < sObs2.NrPt.y) ? (sObs2.NrPt.y - sObs1.FrPt.y) : (sObs1.NrPt.y - sObs2.FrPt.y);
			fDis = sqrt(dX * dX + dY * dY);
		}
	}
	if (fDis < T_RECT_MERGE)
	{
		sObs1.FrPt.x = max(sObs1.FrPt.x, sObs2.FrPt.x);
		sObs1.FrPt.y = max(sObs1.FrPt.y, sObs2.FrPt.y);
		sObs1.NrPt.x = min(sObs1.NrPt.x, sObs2.NrPt.x);
		sObs1.NrPt.y = min(sObs1.NrPt.y, sObs2.NrPt.y);
		sObs1.CnPt.x = (sObs1.NrPt.x + sObs1.FrPt.x) / 2;
		sObs1.CnPt.y = (sObs1.NrPt.y + sObs1.FrPt.y) / 2;
		sObs1.nWidth = sObs1.FrPt.x - sObs1.NrPt.x;
		sObs1.nHeight = sObs1.FrPt.y - sObs1.NrPt.y;
		sObs1.fHAngle = 35 * (2 * (float)sObs1.CnPt.x / T_IMAGE_WIDTH - 1);
		sObs1.fVAngle = 0.1 * (T_IMAGE_HEIGHT - sObs1.CnPt.y);
	}
	
	return fDis;
}

void MergeObs(vector<SObstacle> &vSrcObsList, vector<SObstacle>& vSRltObsList)
{
	//SObstacleInGridMap STmpGridObs;
	//vector<SObstacleInfo> vTmpObsList;
	int nNum;
	nNum = vSrcObsList.size();
	BOOL bKeepWork = TRUE;
	//BOOL bFlag[200];
	vector<BOOL> bFlag;
	vSRltObsList.clear();

	if (nNum < 2)
	{
		vSRltObsList.push_back(vSrcObsList[0]);
	}
	else
	{
		for (unsigned int nI = 0; nI < vSrcObsList.size(); nI++)
		{
			bFlag.push_back(TRUE);
		}
		while (bKeepWork)
		{
			bKeepWork = FALSE;
			for (unsigned int nI = 0; nI < vSrcObsList.size(); nI++)
			{
				if (bFlag[nI])
				{
					for (int nJ = nI + 1; nJ < nNum; nJ++)
					{
						if (bFlag[nJ])
						{
							if (RectDist(vSrcObsList[nI], vSrcObsList[nJ]) < T_RECT_MERGE)
							{
								bFlag[nJ] = FALSE;
								bKeepWork = TRUE;
							}
						}
					}
				}
			}
		}

		for (int nI = 0; nI < nNum; nI++)
		{
			if (bFlag[nI])
			{
				vSRltObsList.push_back(vSrcObsList.at(nI));
			}
		}
		bFlag.clear();
	}

}

int  Find(int i, vector <SDisjointSetNode> & DisjointSet)
{
	int  Ix = 0, result = 0, Root = 0;
	stack <int> S;
	if (DisjointSet[i].Parent == i)
	{
		return i;
	}
	S.push(i);
	while (!S.empty())
	{
		Ix = S.top();
		if (DisjointSet[Ix].Parent == Ix)
		{
			break;
		}
		S.push(DisjointSet[Ix].Parent);
	}
	Root = Ix;
	while (!S.empty())      //Path Compression
	{
		Ix = S.top();
		S.pop();
		DisjointSet[Ix].Parent = Root;
	}
	return Root;
}
int  Union(int i, int j, vector <SDisjointSetNode> & DisjointSet)
{
	int Root1 = Find(i, DisjointSet);
	int Root2 = Find(j, DisjointSet);
	if (Root1 == Root2)      //两个Set是联通的
	{
		return 1;
	}
	if (DisjointSet[Root1].Rank<DisjointSet[Root2].Rank)    //Union by Rank
	{
		DisjointSet[Root1].Parent = Root2;
	}
	else if (DisjointSet[Root1].Rank>DisjointSet[Root2].Rank)
	{
		DisjointSet[Root2].Parent = Root1;
	}
	else
	{
		DisjointSet[Root2].Parent = Root1;
		DisjointSet[Root1].Rank++;
	}
	return 0;
}

IplImage * FindConnectedComp2(IplImage * binaryImg, vector <int> & Label, vector<vector<CvPoint>> & BlobLabel, int MinPtsNum)
{
	BlobLabel.clear();
	//int dx[2] = {-1,  0};    //四邻域
	//int dy[2] = { 0, -1};
	int dx[4] = { -1, -1, 0, 1 };   //八邻域
	int dy[4] = { 0, -1, -1, -1 };
	//int dx[12] = { -2, -1, 0, 1, 2, -2, -1, 0, 1, 2, -2, -1};   //24邻域
	//int dy[12] = { -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, 0, 0 };

	int CurrentX = 0, CurrentY = 0, tempVal1 = 0, tempVal2 = 0, step = 0, label_Count = 0, NeighboorIx = 0;
	PixelNode *     tempPixelNode = (PixelNode *)malloc(sizeof(PixelNode));
	SDisjointSetNode * tempDSNode = (SDisjointSetNode *)malloc(sizeof(SDisjointSetNode));
	vector <SDisjointSetNode> DisjointSet;

	IplImage * pLabelImg = cvCreateImage(cvGetSize(binaryImg), IPL_DEPTH_16U, binaryImg->nChannels); //Label可能超过255,所以选择IPL_DEPTH_16U的图像进行存储
	cvConvert(binaryImg, pLabelImg);

	const int N = sizeof(dx) / sizeof(int);

	ushort * ImgData = (ushort *)pLabelImg->imageData;
	step = pLabelImg->widthStep / sizeof(ushort);

	tempDSNode->Parent = 0;         //0是 Singleton Set,其他的Label Set 进行union-find操作
	tempDSNode->Rank = 0;
	DisjointSet.push_back(*tempDSNode);

	omp_set_num_threads(8);
#pragma omp parallel for
	for (int y = 0; y<pLabelImg->height; y++)     //第一趟标记
	{
		for (int x = 0; x<pLabelImg->width; x++)
		{
			if (!ImgData[y * step + x])    //碰到背景时跳过
			{
				continue;
			}
			memset(tempPixelNode, 0, sizeof(PixelNode));
			int i = 0;
			while (i < N)    //邻域判断
			{
				CurrentX = x + dx[i];
				CurrentY = y + dy[i];
				if (CurrentX < pLabelImg->width && CurrentX >= 0 && CurrentY < pLabelImg->height && CurrentY >= 0)  //在图像范围内
				{
					if (ImgData[CurrentY * step + CurrentX])
					{
						NeighboorIx = tempPixelNode->NeighboorNum;
						tempPixelNode->Neighboor[NeighboorIx] = int(ImgData[CurrentY*step + CurrentX]);
						tempPixelNode->NeighboorNum++;
					}
				}
				i++; 
			}
			if (!tempPixelNode->NeighboorNum)
			{
				label_Count++;
				ImgData[y * step + x] = label_Count;

				tempDSNode->Parent = label_Count;
				tempDSNode->Rank = 0;
				DisjointSet.push_back(*tempDSNode);
			}
			else
			{
				if (tempPixelNode->NeighboorNum == 1)
				{
					ImgData[y * step + x] = tempPixelNode->Neighboor[0];
				}
				else if (tempPixelNode->NeighboorNum == 2)
				{
					ImgData[y * step + x] = tempPixelNode->Neighboor[0] <= tempPixelNode->Neighboor[1] ? tempPixelNode->Neighboor[0] : tempPixelNode->Neighboor[1];
				}
				else if (tempPixelNode->NeighboorNum == 3)
				{
					tempVal1 = tempPixelNode->Neighboor[0] <= tempPixelNode->Neighboor[1] ? tempPixelNode->Neighboor[0] : tempPixelNode->Neighboor[1];
					tempVal2 = tempVal1 <= tempPixelNode->Neighboor[2] ? tempVal1 : tempPixelNode->Neighboor[2];
					ImgData[y * step + x] = tempVal2;
				}
				else
				{
					tempVal1 = tempPixelNode->Neighboor[0] <= tempPixelNode->Neighboor[1] ? tempPixelNode->Neighboor[0] : tempPixelNode->Neighboor[1];
					tempVal2 = tempPixelNode->Neighboor[2] <= tempPixelNode->Neighboor[3] ? tempPixelNode->Neighboor[2] : tempPixelNode->Neighboor[3];
					ImgData[y * step + x] = tempVal2;
				}
				for (int j = 1; j<tempPixelNode->NeighboorNum; j++)    //合并Equivence Label
				{
					Union(tempPixelNode->Neighboor[j], tempPixelNode->Neighboor[0], DisjointSet);
				}
			}
		}
	}
	free(tempPixelNode);
	free(tempDSNode);

	int tempLabelVal = 0, Diff = 0;
	vector <int>::iterator iter;

	vector <int> LabelCounter;
	Label.clear();
	LabelCounter.clear();
	for (int y = 0; y<pLabelImg->height; y++)     //第二趟标记,替换Equivelence labels
	{
		for (int x = 0; x<pLabelImg->width; x++)
		{
			if (!ImgData[y * step + x])
			{
				continue;
			}
			tempLabelVal = Find(ImgData[y*step + x], DisjointSet);
			ImgData[y * step + x] = tempLabelVal;
			iter = find(Label.begin(), Label.end(), tempLabelVal);
			if (iter == Label.end())    //碰到新的Label
			{
				Label.push_back(tempLabelVal);
				LabelCounter.push_back(1);
			}
			else
			{
				Diff = iter - Label.begin();
				LabelCounter[Diff]++;
			}
		}
	}

	for (vector<int>::size_type i = 0; i != Label.size(); i++)    //剔除小于阈值的Label
	{
		if (LabelCounter[i] < MinPtsNum)
		{
			Label[i] = 0;
			LabelCounter[i] = 0;
		}
	}
	vector<int>::iterator rmLabel_iter = remove(Label.begin(), Label.end(), 0);
	vector<int>::iterator rmCount_iter = remove(LabelCounter.begin(), LabelCounter.end(), 0);
	Label.erase(rmLabel_iter, Label.end());
	LabelCounter.erase(rmCount_iter, LabelCounter.end());

	for (int y = 0; y<pLabelImg->height; y++)    //着色并重新按次序整定label Image
	{
		for (int x = 0; x<pLabelImg->width; x++)
		{
			if (!ImgData[y * step + x])
			{
				continue;
			}
			iter = find(Label.begin(), Label.end(), ImgData[y * step + x]);
			if (iter == Label.end())
			{
				ImgData[y * step + x] = 0;
				continue;
			}
			Diff = iter - Label.begin();
			if (BlobLabel.size() <= Diff)
			{
				vector<CvPoint> vecTemp;
				vecTemp.push_back(cvPoint(x, y));
				BlobLabel.push_back(vecTemp);
			}
			else
			{
				BlobLabel[Diff].push_back(cvPoint(x, y));
			}
			ImgData[y * step + x] = Diff + 1;
		}
	}
	return pLabelImg;
}

IplImage * FindConnectedComp(IplImage * binaryImg, vector <int> & Label, vector<vector<CvPoint>> & BlobLabel, int MinPtsNum, vector<SObstacle>& vSObs)
{
	BlobLabel.clear();
	//int dx[2] = {-1,  0};    //四邻域
	//int dy[2] = { 0, -1};
	int dx[4] = { -1, -1, 0, 1 };   //八邻域
	int dy[4] = { 0, -1, -1, -1 };
	//int dx[12] = { -2, -1, 0, 1, 2, -2, -1, 0, 1, 2, -2, -1};   //24邻域
	//int dy[12] = { -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, 0, 0 };

	int CurrentX = 0, CurrentY = 0, tempVal1 = 0, tempVal2 = 0, step = 0, label_Count = 0, NeighboorIx = 0;
	PixelNode *     tempPixelNode = (PixelNode *)malloc(sizeof(PixelNode));
	SDisjointSetNode * tempDSNode = (SDisjointSetNode *)malloc(sizeof(SDisjointSetNode));
	vector <SDisjointSetNode> DisjointSet;

	IplImage * pLabelImg = cvCreateImage(cvGetSize(binaryImg), IPL_DEPTH_16U, binaryImg->nChannels); //Label可能超过255,所以选择IPL_DEPTH_16U的图像进行存储
	cvConvert(binaryImg, pLabelImg);

	const int N = sizeof(dx) / sizeof(int);

	ushort * ImgData = (ushort *)pLabelImg->imageData;
	step = pLabelImg->widthStep / sizeof(ushort);

	tempDSNode->Parent = 0;         //0是 Singleton Set,其他的Label Set 进行union-find操作
	tempDSNode->Rank = 0;
	DisjointSet.push_back(*tempDSNode);

	omp_set_num_threads(8);
#pragma omp parallel for
	for (int y = 0; y<pLabelImg->height; y++)     //第一趟标记
	{
		for (int x = 0; x<pLabelImg->width; x++)
		{
			if (!ImgData[y * step + x])    //碰到背景时跳过
			{
				continue;
			}
			memset(tempPixelNode, 0, sizeof(PixelNode));
			int i = 0;
			while (i < N)    //邻域判断
			{
				CurrentX = x + dx[i];
				CurrentY = y + dy[i];
				if (CurrentX < pLabelImg->width && CurrentX >= 0 && CurrentY < pLabelImg->height && CurrentY >= 0)  //在图像范围内
				{
					if (ImgData[CurrentY * step + CurrentX])
					{
						NeighboorIx = tempPixelNode->NeighboorNum;
						tempPixelNode->Neighboor[NeighboorIx] = int(ImgData[CurrentY*step + CurrentX]);
						tempPixelNode->NeighboorNum++;
					}
				}
				i++;
			}
			if (!tempPixelNode->NeighboorNum)
			{
				label_Count++;
				ImgData[y * step + x] = label_Count;

				tempDSNode->Parent = label_Count;
				tempDSNode->Rank = 0;
				DisjointSet.push_back(*tempDSNode);
			}
			else
			{
				if (tempPixelNode->NeighboorNum == 1)
				{
					ImgData[y * step + x] = tempPixelNode->Neighboor[0];
				}
				else if (tempPixelNode->NeighboorNum == 2)
				{
					ImgData[y * step + x] = tempPixelNode->Neighboor[0] <= tempPixelNode->Neighboor[1] ? tempPixelNode->Neighboor[0] : tempPixelNode->Neighboor[1];
				}
				else if (tempPixelNode->NeighboorNum == 3)
				{
					tempVal1 = tempPixelNode->Neighboor[0] <= tempPixelNode->Neighboor[1] ? tempPixelNode->Neighboor[0] : tempPixelNode->Neighboor[1];
					tempVal2 = tempVal1 <= tempPixelNode->Neighboor[2] ? tempVal1 : tempPixelNode->Neighboor[2];
					ImgData[y * step + x] = tempVal2;
				}
				else
				{
					tempVal1 = tempPixelNode->Neighboor[0] <= tempPixelNode->Neighboor[1] ? tempPixelNode->Neighboor[0] : tempPixelNode->Neighboor[1];
					tempVal2 = tempPixelNode->Neighboor[2] <= tempPixelNode->Neighboor[3] ? tempPixelNode->Neighboor[2] : tempPixelNode->Neighboor[3];
					ImgData[y * step + x] = tempVal2;
				}
				for (int j = 1; j<tempPixelNode->NeighboorNum; j++)    //合并Equivence Label
				{
					Union(tempPixelNode->Neighboor[j], tempPixelNode->Neighboor[0], DisjointSet);
				}
			}
		}
	}
	free(tempPixelNode);
	free(tempDSNode);

	int tempLabelVal = 0, Diff = 0;
	vector <int>::iterator iter;

	vector <int> LabelCounter;
	Label.clear();
	LabelCounter.clear();
	for (int y = 0; y<pLabelImg->height; y++)     //第二趟标记,替换Equivelence labels
	{
		for (int x = 0; x<pLabelImg->width; x++)
		{
			if (!ImgData[y * step + x])
			{
				continue;
			}
			tempLabelVal = Find(ImgData[y*step + x], DisjointSet);
			ImgData[y * step + x] = tempLabelVal;
			iter = find(Label.begin(), Label.end(), tempLabelVal);
			if (iter == Label.end())    //碰到新的Label
			{
				Label.push_back(tempLabelVal);
				LabelCounter.push_back(1);
			}
			else
			{
				Diff = iter - Label.begin();
				LabelCounter[Diff]++;
			}
		}
	}

	for (vector<int>::size_type i = 0; i != Label.size(); i++)    //剔除小于阈值的Label
	{
		if (LabelCounter[i] < MinPtsNum)
		{
			Label[i] = 0;
			LabelCounter[i] = 0;
		}
	}
	vector<int>::iterator rmLabel_iter = remove(Label.begin(), Label.end(), 0);
	vector<int>::iterator rmCount_iter = remove(LabelCounter.begin(), LabelCounter.end(), 0);
	Label.erase(rmLabel_iter, Label.end());
	LabelCounter.erase(rmCount_iter, LabelCounter.end());
	int nMaxX = 0, nMinX = pLabelImg->width, nMaxY = 0, nMinY = pLabelImg->height;
	SObstacle sTmpObs;
	memset(&sTmpObs, 0, sizeof(sTmpObs));
	sTmpObs.FrPt.x = 0;
	sTmpObs.FrPt.y = 0;
	sTmpObs.NrPt.x = pLabelImg->width;
	sTmpObs.NrPt.y = pLabelImg->height;
	//vector<int> vMaxX, vMinX, vMaxY, vMinY;
	vSObs.clear();
	for (unsigned int nI = 0; nI < Label.size(); nI++)
	{
		vSObs.push_back(sTmpObs);
		//vMinX.push_back(nMinX);
		//vMaxY.push_back(nMaxY);
		//vMinY.push_back(nMinY);
	}

	for (int y = 0; y<pLabelImg->height; y++)    //着色并重新按次序整定label Image
	{
		for (int x = 0; x<pLabelImg->width; x++)
		{
			if (!ImgData[y * step + x])
			{
				continue;
			}
			iter = find(Label.begin(), Label.end(), ImgData[y * step + x]);
			if (iter == Label.end())
			{
				ImgData[y * step + x] = 0;
				continue;
			}
			Diff = iter - Label.begin();
			if (BlobLabel.size() <= Diff)
			{
				vector<CvPoint> vecTemp;
				vecTemp.push_back(cvPoint(x, y));
				BlobLabel.push_back(vecTemp);
				vSObs[Diff].FrPt.x = x;
				vSObs[Diff].FrPt.y = y;
				vSObs[Diff].NrPt.x = x;
				vSObs[Diff].NrPt.y = y;
			}
			else
			{
				BlobLabel[Diff].push_back(cvPoint(x, y));
				if (vSObs[Diff].FrPt.x < x)
					vSObs[Diff].FrPt.x = x;
				if (vSObs[Diff].NrPt.x > x)
					vSObs[Diff].NrPt.x = x;
				if (vSObs[Diff].FrPt.y < y)
					vSObs[Diff].FrPt.y = y;
				if (vSObs[Diff].NrPt.y > y)
					vSObs[Diff].NrPt.y = y;
			}
			ImgData[y * step + x] = Diff + 1;
		}
	}
	for (unsigned int nI = 0; nI < vSObs.size(); nI++)
	{
		vSObs[nI].nWidth = vSObs[nI].FrPt.x - vSObs[nI].NrPt.x;
		vSObs[nI].nHeight = vSObs[nI].FrPt.y - vSObs[nI].NrPt.y;
		vSObs[nI].CnPt.x = (vSObs[nI].FrPt.x + vSObs[nI].NrPt.x) / 2;
		vSObs[nI].CnPt.y = (vSObs[nI].FrPt.y + vSObs[nI].NrPt.y) / 2;
		vSObs[nI].fHAngle = (2 * vSObs[nI].CnPt.x / T_IMAGE_WIDTH - 1) * 37.5;
	}
	return pLabelImg;
}

void  doPyrSegmentation(IplImage * src, IplImage * dst)
{
	assert(src->width % 2 == 0 && src->height % 2 == 0);

	CvMemStorage * stoage = cvCreateMemStorage(0);
	CvSeq* comp = NULL;

	int level = 2;   //进行n层采样  
	double threshold1 = 150;
	double threshold2 = 30;
	cvPyrSegmentation(src, dst, stoage, &comp, level, threshold1, threshold2);
};

int FindMax(int * pN)
{
	int nMax = *pN;
	pN++;
	while (pN)
	{
		nMax = max(nMax, *pN);
	}
	return nMax;
}

int ColourRecognitionHSV(IplImage* pSrcImg, int nRowB, int nRowE, int nColB, int nColE)
{
	int nR[256] = { 0 }, nG[256] = { 0 }, nB[256] = { 0 };
	int nCountR = 0;
	int nCountB = 0;
	CvScalar sColour;
	cvCvtColor(pSrcImg, pImgHsv, CV_BGR2HSV);
	for (int nI = nRowB; nI < nRowE + 1; nI ++)
	{
		for (int nJ = nColB; nJ < nColE + 1; nJ++)
		{
			sColour = cvGet2D(pImgHsv, nI, nJ);
			int nH = (int)sColour.val[0];
			int nS = (int)sColour.val[1];
			int nV = (int)sColour.val[2];
			if (nV > 46 && nS > 43)
			{
				if (nH < 10 || nH > 156)
				{
					cvCircle(pImgTst, cvPoint(nJ, nI), 1, CV_RGB(255, 0, 0), 1, 8, 3);
					nCountR++;
				}
				else if (nH > 80 && nH < 124)
				{
					cvCircle(pImgTst, cvPoint(nJ, nI), 1, CV_RGB(0, 0, 255), 1, 8, 3);
					nCountB++;
				}
			}
		}
	}
	float fPerct = (float)nCountB / nCountR;
	if (fPerct < T_COLOUR_THRESHOLD)
		return 1;
	else
		return 0;
	//int nMaxR, nIndexR, nMaxB, nIndexB;
	//nMaxR = nR[0];
	//nMaxB = nB[0];
	//nIndexR = 0;
	//nIndexB = 0;
	//for (int nI = 1; nI < 256; nI++)
	//{
	//	if (nMaxR < nR[nI])
	//	{
	//		nMaxR = nR[nI];
	//		nIndexR = nI;
	//	}
	//	if (nMaxB < nB[nI])
	//	{
	//		nMaxB = nB[nI];
	//		nIndexB = nI;
	//	}
	//}
	////for (int nI = 0; nI < 256)
	//if (nIndexR > nIndexB)
	//	return 1;
	//else
	//	return 0;
}

int ColourRecognition(IplImage* pSrcImg, int nRowB, int nRowE, int nColB, int nColE)
{
	//int nR[256] = { 0 }, nG[256] = { 0 }, nB[256] = { 0 };
	int nCountR = 0;
	int nCountB = 0;
	for (int nI = nRowB; nI < nRowE + 1; nI++)
	{
		for (int nJ = nColB; nJ < nColE + 1; nJ++)
		{
			CvScalar sColour = cvGet2D(pSrcImg, nI, nJ);
			if (sColour.val[0] > sColour.val[2])
			{
				cvCircle(pImgTst, cvPoint(nJ, nI), 1, CV_RGB(255, 0, 0), 1, 8, 3);
				nCountR++;
			}
			else if (sColour.val[0] < sColour.val[2])
			{
				cvCircle(pImgTst, cvPoint(nJ, nI), 1, CV_RGB(0, 0, 255), 1, 8, 3);
				nCountB++;
			}
		}
	}
	float fPerct = (float)nCountB / nCountR;
	if (fPerct < T_COLOUR_THRESHOLD)
		return 1;
	else
		return 0;
	//int nMaxR, nIndexR, nMaxB, nIndexB;
	//nMaxR = nR[0];
	//nMaxB = nB[0];
	//nIndexR = 0;
	//nIndexB = 0;
	//for (int nI = 1; nI < 256; nI++)
	//{
	//	if (nMaxR < nR[nI])
	//	{
	//		nMaxR = nR[nI];
	//		nIndexR = nI;
	//	}
	//	if (nMaxB < nB[nI])
	//	{
	//		nMaxB = nB[nI];
	//		nIndexB = nI;
	//	}
	//}
	////for (int nI = 0; nI < 256)
	//if (nIndexR > nIndexB)
	//	return 1;
	//else
	//	return 0;
}

int SumImg(IplImage* src)
{
	int i, j;
	unsigned char* SrcData = (unsigned char*)src->imageData;
	int step = src->widthStep / sizeof(unsigned char);
	char sumRow[480] = { 0 };
	char sumCol[640] = { 0 };

	omp_set_num_threads(8);
#pragma omp parallel for
	//行,列求和
	for (i = 0; i < src->height; i++)
	{
		for (j = 0; j < src->width; j++)
		{
			if (SrcData[i*step + j] == 255)
			{
				sumRow[i]++;
				sumCol[j]++;
			}
		}
	}

	i = 0;
	j = 0;
	vector<int> vRow, vCol,vTmp;
	vector<vector<int>> vRowSeg, vColSeg;
	vector<int> vRowBeg, vRowEnd, vColBeg, vColEnd;

	while (i<src->height)
	{
		if (sumRow[i] >= T_SUM_STHRESHOLD)
		{
			vRow.push_back(i);
		}
		i++;
	}

	if (vRow.size() > 1)
	{
		int nTmp = vRow[0];
		vTmp.push_back(nTmp);
		for (unsigned int nI = 1; nI < vRow.size(); nI++)
		{
			if (vRow[nI] - nTmp > T_ROW_SEGMENT_THRESHOLD)
			{
				if (vTmp.size() > 1)
					vRowSeg.push_back(vTmp);
				vTmp.clear();
				nTmp = vRow[nI];
				vTmp.push_back(nTmp);
			}
			else
			{
				nTmp = vRow[nI];
				vTmp.push_back(nTmp);
			}
		}
		if (vTmp.size() > 0)
		{
			vRowSeg.push_back(vTmp);
		}
		vTmp.clear();
		for (unsigned int nI = 0; nI < vRowSeg.size(); nI++)
		{
			vRowBeg.push_back(vRowSeg[nI][0]);
			vRowEnd.push_back(vRowSeg[nI][vRowSeg[nI].size() - 1]);
			//cvLine(RgbImage, cvPoint(0, vRowSeg[nI][0]), cvPoint(640, vRowSeg[nI][0]), cvScalar(255, 0, 0), 1);
			//cvLine(RgbImage, cvPoint(0, vRowSeg[nI][vRowSeg[nI].size() - 1]), cvPoint(640, vRowSeg[nI][vRowSeg[nI].size() - 1]), cvScalar(255, 0, 0), 1);
		}
		//cvLine(RgbImage, cvPoint(0, vRow[0]), cvPoint(640, vRow[0]), cvScalar(0, 255, 0), 1);
		//cvLine(RgbImage, cvPoint(0, vRow[vRow.size()-1]), cvPoint(640, vRow[vRow.size()-1]), cvScalar(0, 255, 0), 1);
	}

	while (j<src->width)
	{
		if (sumCol[j] >= T_SUM_STHRESHOLD)
		{
			vCol.push_back(j);
		}
		j++;
	}
	if (vCol.size() > 1)
	{
		int nTmp = vCol[0];
		vTmp.push_back(nTmp);
		for (unsigned int nI = 1; nI < vCol.size(); nI++)
		{
			if (vCol[nI] - nTmp > T_COL_SEGMENT_THRESHOLD)
			{
				if (vTmp.size() > 1)
					vColSeg.push_back(vTmp);
				vTmp.clear();
				nTmp = vCol[nI];
				vTmp.push_back(nTmp);
			}
			else
			{
				nTmp = vCol[nI];
				vTmp.push_back(nTmp);
			}
		}
		if (vTmp.size() > 0)
		{
			vColSeg.push_back(vTmp);
		}
		vTmp.clear();
		for (unsigned int nI = 0; nI < vColSeg.size(); nI++)
		{
			vColBeg.push_back(vColSeg[nI][0]);
			vColEnd.push_back(vColSeg[nI][vColSeg[nI].size() - 1]);
		}

	}

	float fPerct;
	for (unsigned int nI = 0; nI < vRowBeg.size(); nI++)
	{
		for (unsigned int nJ = 0; nJ < vColBeg.size(); nJ++)
		{
			int nCount = 0;
			for (int nK = vRowBeg[nI]; nK < vRowEnd[nI]; nK++)
			{
				for (int nL = vColBeg[nJ]; nL < vColEnd[nJ]; nL++)
				{
					if (SrcData[nK*step + nL] == 255)
					{
						nCount++;
					}
				}
			}
			int nColNum = vColEnd[nJ] - vColBeg[nJ] + 1;
			int nRowNum = vRowEnd[nI] - vRowBeg[nI] + 1;
			fPerct = (float)nCount / (nRowNum * nColNum);
			if (fPerct > T_VOTE_STHRESHOLD)
			{
				char * pColour;
				CvFont font;
				cvInitFont(&font, CV_FONT_HERSHEY_DUPLEX, 1.0f, 1.0f, 0, 1, CV_AA);
				cvRectangle(RgbImage, cvPoint(vColBeg[nJ], vRowBeg[nI]), cvPoint(vColEnd[nJ], vRowEnd[nI]), cvScalar(0, 0, 255));
				if (ColourRecognitionHSV(RgbImage, vRowBeg[nI], vRowEnd[nI], vColBeg[nJ], vColEnd[nJ]) > 0)
				{
					pColour = "RED";
					//cvPutText(RgbImage,);
				}
				else
					pColour = "BLUE";
				cvPutText(RgbImage, pColour, cvPoint(vColBeg[nJ], vRowBeg[nI]), &font,CV_RGB(0,255,0));
			}
		}
	}

	vRow.clear();
	vCol.clear();
	vRowBeg.clear();
	vRowEnd.clear();
	vColBeg.clear();
	vRowEnd.clear();
	vRowSeg.clear();
	vColSeg.clear();
	return TRUE;
}

//int SumImg(IplImage* src)
//{
//	int i, j;
//	unsigned char* SrcData = (unsigned char*)src->imageData;
//	int step = src->widthStep / sizeof(unsigned char);
//	char sumRow[480] = { 0 };
//	char sumCol[640] = { 0 };
//
//	omp_set_num_threads(8);
//	#pragma omp parallel for
//	//行,列求和
//	for (i = 0; i < src->height; i++)
//	{
//		for (j = 0; j < src->width; j++)
//		{
//			if (SrcData[i*step + j] == 255)
//			{
//				sumRow[i]++;
//				sumCol[j]++;
//			}
//		}
//	}
//	i=0;
//	j = 0;
//	while (i<src->height)
//	{
//		if (sumRow[i] >= 20)
//		{
//			cvLine(RgbImage, cvPoint(0, i), cvPoint(640, i), cvScalar(0, 255, 0), 1);
//		}
//		i++;
//	}
//	return TRUE;
//}

int CheckLine(IplImage* src)
{
	int i, j;
	unsigned char* SrcData = (unsigned char*)src->imageData;
	int step = src->widthStep / sizeof(unsigned char);
	

	for (i = 0; i < src->height; i++)
	{
		for (j = 0; j < src->width; j++)
		{
			if (SrcData[i*step + j] == 255 && SrcData[i*step + j + 1] == 255 && SrcData[i*step + j+2] == 255 &&
				(SrcData[(i + 4)*step + j] == 255 || SrcData[(i + 4)*step + j + 1] == 255 || SrcData[(i + 4)*step + j] == 255))
			{
			//	已确定上部摄像头位置，(j,i)
				cvCircle(RgbImage, cvPoint(j, i), 8, cvScalar(0, 0, 255), 3);
				for (i = i; i < src->height; i++)
				{
					if (SrcData[i*step + j] == 255 && SrcData[(i + 3)*step + j] == 0 && SrcData[(i + 5)*step + j] == 0)
					{
						cvCircle(RgbImage, cvPoint(j, i), 8, cvScalar(255, 0, 0), 3);
						return i;
					}
				}
			}
		}
	}
	
	return 0;
	//cvShowImage("RGB", RgbImage);

}

int SumWhite(IplImage* src)
{
	int i,j;
	unsigned char* SrcData = (unsigned char*)src->imageData;
	int step = src->widthStep / sizeof(unsigned char);
	omp_set_num_threads(8);	
	#pragma omp parallel for
	int num=0;
	for (i = 0; i < src->height; i++)
	{
		for (j = 0; j < src->width; j++)
		{
			if (SrcData[i*step + j] == 255)
			{
				num++;
			}
		}
	}
	return num;
}

int TickWhite(IplImage* src, bool OutPutMode)
{
	int i, j;
	cvCopy(src, TickImage);
	unsigned char* SrcData = (unsigned char*)src->imageData;
	unsigned char* dstData = (unsigned char*)TickImage->imageData;
	int step = src->widthStep / sizeof(unsigned char);
	for (i = 0; i < src->height; i++)
	{
		for (j = 0; j < src->width; j++)
		{
			if (SrcData[i*step + j] == 255)
			{
				if (SrcData[(i - 1)*step + j] == 0 &&
					(SrcData[i*step + j - 1] == 0 && SrcData[i*step + j + 1] == 0 )&&             //噪声点去除
					 SrcData[(i + 1)*step + j] == 0)
				{
					dstData[i*step + j] = 0;
				}
			}
		}
	}
	if (OutPutMode == PRINT_PARAM)
	{
		cvShowImage("TickWhite", TickImage);
	}
//	free(SrcData);
	//delete[] dstData;
	return TRUE;
}
int cvThresholdOtsu(IplImage* src)
{
	int height = src->height;
	int width = src->width;

	//histogram  
	float histogram[256] = { 0 };
	for (int i = 0; i<height; i++) {
		unsigned char* p = (unsigned char*)src->imageData + src->widthStep*i;
		for (int j = 0; j<width; j++) {
			histogram[*p++]++;
		}
	}

	//normalize histogram  
	int size = height*width;
	for (int i = 0; i<256; i++) {
		histogram[i] = histogram[i] / size;
	}

	//average pixel value  
	float avgValue = 0;
	for (int i = 0; i<256; i++) {
		avgValue += i*histogram[i];
	}

	int threshold;
	float maxVariance = 0;
	float w = 0, u = 0;
	for (int i = 0; i<256; i++) {
		w += histogram[i];
		u += i*histogram[i];

		float t = avgValue*w - u;
		float variance = t*t / (w*(1 - w));
		if (variance>maxVariance) {
			maxVariance = variance;
			threshold = i;
		}
	}

	return threshold;
}

int GetThreshold(IplImage* src, IplImage* dst, int thre, int FixValue = 255)
{
	int i, j, num;
	unsigned char* SrcData = (unsigned char*)src->imageData;
	unsigned char* DstData = (unsigned char*)dst->imageData;
	int step = src->widthStep / sizeof(unsigned char);
	//cvSet(dst,Black);
	num = 0;

	omp_set_num_threads(8);
#pragma omp parallel for

	for (i = 0; i<src->height; i++)
	{
		for (j = 0; j<src->width; j++)
		{
			if (SrcData[i*step + j] > thre)
			{
				DstData[i*step + j] = FixValue;
				num++;
			}
			else
			{
				DstData[i*step + j] = 0;
			}
		}
	}

	return num;
}

int GetDiffThreImage(bool OutPutMode)
{
	//int tmp;
	cvAbsDiff(GrayImage, LastGrayImage, DiffImage);
	cvShowImage("Diff", DiffImage);
	cvCopy(GrayImage, LastGrayImage);
	//cvDilate(DiffImage,DilateImage);
	//cvErode(DilateImage,ErodeImage);
	Thre = cvThresholdOtsu(DiffImage);
	//printf("%d\n",Thre);
	if (Thre<=10)
		cvThreshold(DiffImage,ThresholdImage,50,255,CV_THRESH_BINARY);
	else
		GetThreshold(DiffImage, ThresholdImage, Thre);

	//cvThreshold(DiffImage,ThresholdImage,Thre,255,CV_THRESH_BINARY);
	//
	//printf("%d\n",tmp);
	if (OutPutMode == PRINT_PARAM)
	{
		cvShowImage("DiffThre", ThresholdImage);
	}
	return TRUE;
}

void Vibe_Init(CvMat* pFrameMat)
{
	rng(0xFFFFFFFF);
	//记录随机生成的 行(r) 和 列(c)
	int rand, r, c;
	omp_set_num_threads(8);
#pragma omp parallel for

	//对每个像素样本进行初始化
	for (int y = 0; y<pFrameMat->rows; y++){//Height
		for (int x = 0; x<pFrameMat->cols; x++){//Width
			for (int k = 0; k<defaultNbSamples; k++){
				//随机获取像素样本值
				rand = rng.uniform(0, 9);
				r = y + c_yoff[rand]; if (r<0) r = 0; if (r >= pFrameMat->rows) r = pFrameMat->rows - 1;	//行
				c = x + c_xoff[rand]; if (c<0) c = 0; if (c >= pFrameMat->cols) c = pFrameMat->cols - 1;	//列
				//存储像素样本值
				samples[y][x][k] = CV_MAT_ELEM(*pFrameMat, float, r, c);
			}
			samples[y][x][defaultNbSamples] = 0;
		}
	}
}
void Vibe_Update(CvMat* pFrameMat, CvMat* segMat)
{
	omp_set_num_threads(8);
#pragma omp parallel for
	for (int y = 0; y<pFrameMat->rows; y++){	//Height
		for (int x = 0; x<pFrameMat->cols; x++){	//Width

			//用于判断一个点是否是背景点,index记录已比较的样本个数，count表示匹配的样本个数
			int count = 0, index = 0; float dist = 0;
			//
			while ((count<defaultReqMatches) && (index<defaultNbSamples)){
				dist = CV_MAT_ELEM(*pFrameMat, float, y, x) - samples[y][x][index];
				if (dist<0) dist = -dist;
				if (dist<defaultRadius) count++;
				index++;
			}
			if (count >= defaultReqMatches){

				//判断为背景像素,只有背景点才能被用来传播和更新存储样本值
				samples[y][x][defaultNbSamples] = 0;

				*((float *)CV_MAT_ELEM_PTR(*segMat, y, x)) = background;

				int rand = rng.uniform(0, defaultSubsamplingFactor);
				if (rand == 0){
					rand = rng.uniform(0, defaultNbSamples);
					samples[y][x][rand] = CV_MAT_ELEM(*pFrameMat, float, y, x);
				}
				rand = rng.uniform(0, defaultSubsamplingFactor);
				if (rand == 0){
					int xN, yN;
					rand = rng.uniform(0, 9); yN = y + c_yoff[rand]; if (yN<0) yN = 0; if (yN >= pFrameMat->rows) yN = pFrameMat->rows - 1;
					rand = rng.uniform(0, 9); xN = x + c_xoff[rand]; if (xN<0) xN = 0; if (xN >= pFrameMat->cols) xN = pFrameMat->cols - 1;
					rand = rng.uniform(0, defaultNbSamples);
					samples[yN][xN][rand] = CV_MAT_ELEM(*pFrameMat, float, y, x);
				}
			}
			else {
				//判断为前景像素
				*((float *)CV_MAT_ELEM_PTR(*segMat, y, x)) = foreground;

				samples[y][x][defaultNbSamples]++;
				if (samples[y][x][defaultNbSamples]>30){
					int rand = rng.uniform(0, defaultNbSamples);
					if (rand == 0){
						rand = rng.uniform(0, defaultNbSamples);
						samples[y][x][rand] = CV_MAT_ELEM(*pFrameMat, float, y, x);
					}
				}
			}

		}
	}
}
void Vibe(bool OutPutMode)
{
	static int nFrmNum = 0;//记录帧数

	nFrmNum++;

	if (nFrmNum == 1){
		cvConvert(GrayImage, GrayMat);
		Vibe_Init(GrayMat);
	}
	else{
		cvConvert(GrayImage, GrayMat);
		Vibe_Update(GrayMat, SegMat);
		cvConvert(SegMat, VibeImage);
	}
	if (OutPutMode == PRINT_PARAM)
	{
		cvShowImage("Vibe", VibeImage);
	}
}

//vector<SObstacle> Tracking(vector<SObstacle> vPreObs, vector<SObstacle> vCurObs, int nDeltaT)
//{
//	SObstacle TmpObs,RltObs;
//	vector<SObstacle> vRltObs;
//	BOOL bPreState = TRUE;
//	BOOL bOldState = TRUE;
//	BOOL bNewState = TRUE;
//
//	for (unsigned int nI = 0; nI < vCurObs.size(); nI ++)
//	{
//		for (unsigned int nJ = 0; nJ < vPreObs.size(); nJ ++)
//		{
//			TmpObs = vPreObs[nJ];
//			if (TmpObs.fVx > 0)
//				TmpObs.FrPt.x += TmpObs.fVx * nDeltaT;
//			else
//				TmpObs.NrPt.x += TmpObs.fVx * nDeltaT;
//			if (TmpObs.fVy > 0)
//				TmpObs.FrPt.y += TmpObs.fVy * nDeltaT;
//			else
//				TmpObs.NrPt.y += TmpObs.fVy * nDeltaT;
//			if (IntsectRect(vCurObs[nI], TmpObs))
//			{
//				RltObs.CnPt = vCurObs[nI].CnPt;
//				//RltObs.nHeight = (nTolHeight - qObs.front().nHeight + vCurObs[nI].nHeight) / nQueueCount;
//				//RltObs.nWidth = (nTolWidth - qObs.front().nWidth + vCurObs[nI].nWidth) / nQueueCount;
//				RltObs.nHeight = (vPreObs[nJ].nHeight + vCurObs[nI].nHeight) / 2;
//				RltObs.nWidth = (vPreObs[nJ].nWidth + vCurObs[nI].nWidth) / 2;
//				RltObs.FrPt.x = RltObs.CnPt.x + RltObs.nWidth / 2;
//				RltObs.FrPt.y = RltObs.CnPt.y + RltObs.nHeight / 2;
//				RltObs.NrPt.x = RltObs.CnPt.x - RltObs.nWidth / 2;
//				RltObs.NrPt.y = RltObs.CnPt.y - RltObs.nHeight / 2;
//				RltObs.fVx = (float)(RltObs.CnPt.x - vPreObs[nJ].CnPt.x) / nDeltaT;
//				RltObs.fVy = (float)(RltObs.CnPt.y - vPreObs[nJ].CnPt.y) / nDeltaT;
//				vRltObs.push_back(RltObs);
//				bPreState = FALSE;
//			}
//		}
//		if (bPreState)
//		{
//			for (unsigned int nJ = 0; nJ < vOldObs.size(); nJ++)
//			{
//				if (IntsectRect(vCurObs[nI], vOldObs[nJ]))
//				{
//					RltObs.CnPt = vCurObs[nI].CnPt;
//					//RltObs.nHeight = (nTolHeight - qObs.front().nHeight + vCurObs[nI].nHeight) / nQueueCount;
//					//RltObs.nWidth = (nTolWidth - qObs.front().nWidth + vCurObs[nI].nWidth) / nQueueCount;
//					RltObs.nHeight = (vOldObs[nJ].nHeight + vCurObs[nI].nHeight) / 2;
//					RltObs.nWidth = (vOldObs[nJ].nWidth + vCurObs[nI].nWidth) / 2;
//					RltObs.FrPt.x = RltObs.CnPt.x + RltObs.nWidth / 2;
//					RltObs.FrPt.y = RltObs.CnPt.y + RltObs.nHeight / 2;
//					RltObs.NrPt.x = RltObs.CnPt.x - RltObs.nWidth / 2;
//					RltObs.NrPt.y = RltObs.CnPt.y - RltObs.nHeight / 2;
//					RltObs.fVx = (float)(RltObs.CnPt.x - vOldObs[nJ].CnPt.x) / nDeltaT;
//					RltObs.fVy = (float)(RltObs.CnPt.y - vOldObs[nJ].CnPt.y) / nDeltaT;
//					vRltObs.push_back(RltObs);
//					vOldObs.erase(vOldObs.begin() + nJ);
//					vOldCount.erase(vOldCount.begin() + nJ);
//					bOldState = FALSE;
//				}
//			}
//			if (bOldState)
//			{
//				for (unsigned int nJ = 0; nJ < vNewObs.size(); nJ++)
//				{
//					if (IntsectRect(vCurObs[nI], vOldObs[nJ]))
//					{
//						RltObs.CnPt = vCurObs[nI].CnPt;
//						//RltObs.nHeight = (nTolHeight - qObs.front().nHeight + vCurObs[nI].nHeight) / nQueueCount;
//						//RltObs.nWidth = (nTolWidth - qObs.front().nWidth + vCurObs[nI].nWidth) / nQueueCount;
//						RltObs.nHeight = (vNewObs[nJ].nHeight + vCurObs[nI].nHeight) / 2;
//						RltObs.nWidth = (vNewObs[nJ].nWidth + vCurObs[nI].nWidth) / 2;
//						RltObs.FrPt.x = RltObs.CnPt.x + RltObs.nWidth / 2;
//						RltObs.FrPt.y = RltObs.CnPt.y + RltObs.nHeight / 2;
//						RltObs.NrPt.x = RltObs.CnPt.x - RltObs.nWidth / 2;
//						RltObs.NrPt.y = RltObs.CnPt.y - RltObs.nHeight / 2;
//						RltObs.fVx = (float)(RltObs.CnPt.x - vNewObs[nJ].CnPt.x) / nDeltaT;
//						RltObs.fVy = (float)(RltObs.CnPt.y - vNewObs[nJ].CnPt.y) / nDeltaT;
//						vRltObs.push_back(RltObs);
//						//vOldObs.erase(vOldObs.begin() + nJ);
//						//vOldCount.erase(vOldCount.begin() + nJ);
//						bOldState = FALSE;
//					}
//				}
//			}
//		}
//		else
//		{
//			vNewObs.push_back(vCurObs[nI]);
//			vNewCount.push_back(1);
//		}
//
//
//	}
//	vPreObs.clear();
//	vPreObs = vRltObs;
//	return vRltObs;
//}

void Tracking(vector<SObstacle> vPreObs, vector<SObstacle> & vCurObs, int nDeltaT)
{
	SObstacle TmpObs;
	//vector<SObstacle> vRltObs;
	//BOOL bPreState = TRUE;
	//BOOL bOldState = TRUE;
	//BOOL bNewState = TRUE;

	for (unsigned int nI = 0; nI < vCurObs.size(); nI++)
	{
		for (unsigned int nJ = 0; nJ < vPreObs.size(); nJ++)
		{
			TmpObs = vPreObs[nJ];
			if (TmpObs.fVx > 0)
				TmpObs.FrPt.x += (TmpObs.fVx * nDeltaT / 1000);
			else
				TmpObs.NrPt.x += (TmpObs.fVx * nDeltaT / 1000);
			if (TmpObs.fVy > 0)
				TmpObs.FrPt.y += (TmpObs.fVy * nDeltaT / 1000);
			else
				TmpObs.NrPt.y += (TmpObs.fVy * nDeltaT / 1000);
			if (IntsectRect(vCurObs[nI], TmpObs))
			{
				//RltObs.nHeight = (nTolHeight - qObs.front().nHeight + vCurObs[nI].nHeight) / nQueueCount;
				//RltObs.nWidth = (nTolWidth - qObs.front().nWidth + vCurObs[nI].nWidth) / nQueueCount;
				vCurObs[nI].nHeight = (vPreObs[nJ].nHeight + vCurObs[nI].nHeight) / 2;
				vCurObs[nI].nWidth = (vPreObs[nJ].nWidth + vCurObs[nI].nWidth) / 2;
				vCurObs[nI].FrPt.x = vCurObs[nI].CnPt.x + vCurObs[nI].nWidth / 2;
				vCurObs[nI].FrPt.y = vCurObs[nI].CnPt.y + vCurObs[nI].nHeight / 2;
				vCurObs[nI].NrPt.x = vCurObs[nI].CnPt.x - vCurObs[nI].nWidth / 2;
				vCurObs[nI].NrPt.y = vCurObs[nI].CnPt.y - vCurObs[nI].nHeight / 2;
				vCurObs[nI].fVx = (float)(vCurObs[nI].CnPt.x - vPreObs[nJ].CnPt.x) * 1000 / nDeltaT;
				vCurObs[nI].fVy = (float)(vCurObs[nI].CnPt.y - vPreObs[nJ].CnPt.y) * 1000 / nDeltaT;
				//vRltObs.push_back(RltObs);
				//bPreState = FALSE;
			}
		}
		//if (bPreState)
		//{
		//	for (unsigned int nJ = 0; nJ < vOldObs.size(); nJ++)
		//	{
		//		if (IntsectRect(vCurObs[nI], vOldObs[nJ]))
		//		{
		//			RltObs.CnPt = vCurObs[nI].CnPt;
		//			//RltObs.nHeight = (nTolHeight - qObs.front().nHeight + vCurObs[nI].nHeight) / nQueueCount;
		//			//RltObs.nWidth = (nTolWidth - qObs.front().nWidth + vCurObs[nI].nWidth) / nQueueCount;
		//			RltObs.nHeight = (vOldObs[nJ].nHeight + vCurObs[nI].nHeight) / 2;
		//			RltObs.nWidth = (vOldObs[nJ].nWidth + vCurObs[nI].nWidth) / 2;
		//			RltObs.FrPt.x = RltObs.CnPt.x + RltObs.nWidth / 2;
		//			RltObs.FrPt.y = RltObs.CnPt.y + RltObs.nHeight / 2;
		//			RltObs.NrPt.x = RltObs.CnPt.x - RltObs.nWidth / 2;
		//			RltObs.NrPt.y = RltObs.CnPt.y - RltObs.nHeight / 2;
		//			RltObs.fVx = (float)(RltObs.CnPt.x - vOldObs[nJ].CnPt.x) / nDeltaT;
		//			RltObs.fVy = (float)(RltObs.CnPt.y - vOldObs[nJ].CnPt.y) / nDeltaT;
		//			vRltObs.push_back(RltObs);
		//			vOldObs.erase(vOldObs.begin() + nJ);
		//			vOldCount.erase(vOldCount.begin() + nJ);
		//			bOldState = FALSE;
		//		}
		//	}
		//	if (bOldState)
		//	{
		//		for (unsigned int nJ = 0; nJ < vNewObs.size(); nJ++)
		//		{
		//			if (IntsectRect(vCurObs[nI], vOldObs[nJ]))
		//			{
		//				RltObs.CnPt = vCurObs[nI].CnPt;
		//				//RltObs.nHeight = (nTolHeight - qObs.front().nHeight + vCurObs[nI].nHeight) / nQueueCount;
		//				//RltObs.nWidth = (nTolWidth - qObs.front().nWidth + vCurObs[nI].nWidth) / nQueueCount;
		//				RltObs.nHeight = (vNewObs[nJ].nHeight + vCurObs[nI].nHeight) / 2;
		//				RltObs.nWidth = (vNewObs[nJ].nWidth + vCurObs[nI].nWidth) / 2;
		//				RltObs.FrPt.x = RltObs.CnPt.x + RltObs.nWidth / 2;
		//				RltObs.FrPt.y = RltObs.CnPt.y + RltObs.nHeight / 2;
		//				RltObs.NrPt.x = RltObs.CnPt.x - RltObs.nWidth / 2;
		//				RltObs.NrPt.y = RltObs.CnPt.y - RltObs.nHeight / 2;
		//				RltObs.fVx = (float)(RltObs.CnPt.x - vNewObs[nJ].CnPt.x) / nDeltaT;
		//				RltObs.fVy = (float)(RltObs.CnPt.y - vNewObs[nJ].CnPt.y) / nDeltaT;
		//				vRltObs.push_back(RltObs);
		//				//vOldObs.erase(vOldObs.begin() + nJ);
		//				//vOldCount.erase(vOldCount.begin() + nJ);
		//				bOldState = FALSE;
		//			}
		//		}
		//	}
		//}
		//else
		//{
		//	vNewObs.push_back(vCurObs[nI]);
		//	vNewCount.push_back(1);
		//}


	}
	//vPreObs.clear();
	//vPreObs = vRltObs;
	//return vRltObs;
}

void drawcross(CvArr* img, CvPoint2D32f pt)
{
	const int radius = 3;
	int ptx = cvRound(pt.x);
	int pty = cvRound(pt.y);
	int ls = ptx - radius;
	int re = ptx + radius;
	int us = pty - radius;
	int de = pty + radius;
	cvLine(img, cvPoint(ls, pty), cvPoint(re, pty), CV_RGB(0, 0, 255), 1, 0);
	cvLine(img, cvPoint(ptx, us), cvPoint(ptx, de), CV_RGB(0, 0, 255), 1, 0);
}

int main(){ 

	CvCapture* pCapture0 = cvCreateFileCapture("TestVideo_1.avi");
	//CvCapture* pCapture0 = cvCreateFileCapture("TestVideo_2.avi");
	//CvCapture* pCapture0 = cvCreateCameraCapture(0);
	IplImage* pFrame0 = NULL;
	IplImage * pLabelImg = NULL;
	vector<SObstacle> vSObs,vRltObs, vTmpObs,vPreObs;
	//time_t c_start, c_end;
	CvScalar sColour;
	int nH, nS, nV, nStep;
	unsigned char* DstData;
	//char * pStr = "D:\\DJI\\testfile\\avi\\Result\\";
	char ImgName[100];
	long tPre = 0, tCur;
	int nDeltaT;

	IplImage *pOutlineImage = cvCreateImage(cvGetSize(TmpImage), IPL_DEPTH_8U, 3);
	CvMemStorage *pcvMStorage = cvCreateMemStorage();
	CvSeq *pcvSeq = NULL;
	//CvRect tRect;
	SYSTEMTIME sTm,sPreTm;
	int nMilSecond;
	SObstacle sTmpObs, sPreSendObs;
	int nMinIndex;
	PortOperate mPort;
	int nHangle = 620;
	short int nPixelX, nPixelY;
	float fLW,fPixDis, fMinDis;
	int nIndex = 1;

	GetLocalTime(&sPreTm);
	//nMilSecond = sTm.wMilliseconds;

	mPort.Initial("\\\\.\\COM7");

	
	int count=0;
	vTmpObs.clear();
	vPreObs.clear();

	//int i=0;
	//创建窗口
	//cvNamedWindow("RGB", 1);
//	cvNamedWindow("G");
	//char saveAdd[100];
	cvNamedWindow("RGB", CV_WINDOW_AUTOSIZE);
	RgbImage->origin = IPL_ORIGIN_TL;
	pImgTst->origin = IPL_ORIGIN_TL;
	pFrame0 = cvQueryFrame(pCapture0);
	while (pFrame0)
	{
		cvCopy(pFrame0, RgbImage); 
		cvCopy(pFrame0, pImgTst);

		GetLocalTime(&sTm);
		cout << sTm.wHour << "; " << sTm.wMinute << ": " << sTm.wSecond << ": " << sTm.wMilliseconds << endl;

		nDeltaT = sTm.wMilliseconds - sPreTm.wMilliseconds;
		if (nDeltaT < 0)
			nDeltaT += 1000;
		//cvCvtColor(RgbImage, GrayImage, CV_BGR2GRAY);
		cvRectangle(TmpImage, cvPoint(0, 0), cvPoint(T_IMAGE_WIDTH, T_IMAGE_HEIGHT), cvScalar(0, 0, 0), CV_FILLED,8,0);
		cvCvtColor(RgbImage, pImgHsv, CV_BGR2HSV);
		DstData = (unsigned char*)TmpImage->imageData;
		nStep = TmpImage->widthStep / sizeof(unsigned char);

		for (int nI = 0; nI < T_IMAGE_HEIGHT; nI++)
		{
			for (int nJ = 0; nJ < T_IMAGE_WIDTH; nJ++)
			{
				sColour = cvGet2D(pImgHsv, nI, nJ);
				nH = (int)sColour.val[0];
				nS = (int)sColour.val[1];
				nV = (int)sColour.val[2];
				if (nV > 46 && nS > 43)
				{
					//if (nH < T_RED_C || (nH > T_RED_D && nH < T_RED_E)) //
					if (T_RED_COLOUR)
					{
						if (nH < T_RED_A || nH > T_RED_B)
						{
							DstData[nI*nStep + nJ] = 255;
						}
					}
					else
					{
						if (nH > T_BLUE_LOW && nH < T_BLUE_HIGH)
						{
							DstData[nI*nStep + nJ] = 255;
						}
					}
				}
			}
		}
		//sprintf(ImgName, "D:\\DJI\\testfile\\avi\\Blue\\%d.jpg", nIndex);
		//cvSaveImage(ImgName, TmpImage);

		cvErode(TmpImage, EDImage, NULL, 3);
		cvDilate(EDImage, TmpImage, NULL, 1);

		cvDilate(TmpImage, EDImage, NULL, 1);
		cvErode(EDImage, TickImage, NULL, 1);


		cvFindContours(TickImage, pcvMStorage, &pcvSeq, sizeof(CvContour), CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));
		//cvReleaseImage(&TickImage);
		//cvRectangle(pOutlineImage, cvPoint(0, 0), cvPoint(pOutlineImage->width, pOutlineImage->height), CV_RGB(0, 0, 0), CV_FILLED);
		vSObs.clear();
		vRltObs.clear();
		for (; pcvSeq != NULL; pcvSeq = pcvSeq->h_next)
		{
			CvRect rect = cvBoundingRect(pcvSeq, 0);
			memset(&sTmpObs, 0, sizeof(sTmpObs));
			sTmpObs.NrPt.x = rect.x;
			sTmpObs.NrPt.y = rect.y;
			sTmpObs.FrPt.x = rect.x + rect.width;
			sTmpObs.FrPt.y = rect.y + rect.height;
			sTmpObs.CnPt.x = (sTmpObs.NrPt.x + sTmpObs.FrPt.x) / 2;
			sTmpObs.CnPt.y = (sTmpObs.NrPt.y + sTmpObs.FrPt.y) / 2;
			sTmpObs.nHeight = rect.height;
			sTmpObs.nWidth = rect.width;
			sTmpObs.fHAngle = 35 * (2 * (float)sTmpObs.CnPt.x / T_IMAGE_WIDTH - 1);
			sTmpObs.fVAngle = 0.1 * (T_IMAGE_HEIGHT - sTmpObs.CnPt.y);
			vSObs.push_back(sTmpObs);
			//cvRectangle(RgbImage, cvPoint(rect.x, rect.y), cvPoint(rect.x + rect.width, rect.y + rect.height), cvScalar(255, 255, 255), 0);
		}

		if (vSObs.size() > 0)
		{
			MergeObs(vSObs, vRltObs);
			vTmpObs.clear();
			for (int nI = 0; nI < vRltObs.size(); nI++)
			{
				if (vRltObs[nI].nHeight * vRltObs[nI].nWidth > 10)
				{
					vTmpObs.push_back(vRltObs[nI]);
					//vSObs.erase(vSObs.begin() + nI);
				}
			}
			vRltObs.clear();
			for (int nI = 0; nI < vTmpObs.size(); nI++)
			{
				fLW = ((float)(vTmpObs[nI].nHeight)) / vTmpObs[nI].nWidth;
				//cout << "长宽比： " << fLW << endl;
				//fLW = ((float)(vRltObs[nI].nHeight) * vRltObs[nI].nWidth) / (vRltObs[nI].nHeight + vRltObs[nI].nWidth);
				if (fLW < 0.7 || fLW > 1.34)
				{
					vRltObs.push_back(vTmpObs[nI]);
				}
				//vRltObs.push_back(vTmpObs[nI]);
			}

			Tracking(vPreObs, vRltObs, nDeltaT);

			if (vRltObs.size() > 0)
			{
				if (vRltObs.size() > 1)
				{
					for (unsigned int nI = 0; nI < vRltObs.size(); nI++)
					{
						fPixDis = (vRltObs[nI].CnPt.x - sPreSendObs.CnPt.x) * (vRltObs[nI].CnPt.x - sPreSendObs.CnPt.x) + (vRltObs[nI].CnPt.y - sPreSendObs.CnPt.y) * (vRltObs[nI].CnPt.y - sPreSendObs.CnPt.y);
						if (nI == 0)
						{
							//fPixDis = (vRltObs[nI].CnPt.x - sPreSendObs.CnPt.x) * (vRltObs[nI].CnPt.x - sPreSendObs.CnPt.x) + (vRltObs[nI].CnPt.y - sPreSendObs.CnPt.y) * (vRltObs[nI].CnPt.y - sPreSendObs.CnPt.y);
							fMinDis = fPixDis;
							nMinIndex = nI;
						}
						else
						{
							if (fPixDis < fMinDis)
							{
								fMinDis = fPixDis;
								nMinIndex = nI;
							}
						}
					}
					nHangle = (short int)(vRltObs[nMinIndex].fHAngle * 100 + 14 * (vRltObs[nMinIndex].fVx * nDeltaT) / T_IMAGE_WIDTH);
					nPixelX = vRltObs[nMinIndex].CnPt.x + vRltObs[nMinIndex].fVx * nDeltaT / 1000;
					nPixelY = vRltObs[nMinIndex].CnPt.y + vRltObs[nMinIndex].fVy * nDeltaT / 1000;
					//mPort.SendAngle(nHangle, 800);
					mPort.SendAngleLocation(nHangle, 800, nPixelX, nPixelY);
					sPreSendObs = vRltObs[nMinIndex];
					cout << nPixelX << ",  " << nPixelY << endl;
				}
				else
				{
					nHangle = (short int)(vRltObs[0].fHAngle * 100 + 14 * (vRltObs[0].fVx * nDeltaT) / T_IMAGE_WIDTH);
					nPixelX = vRltObs[0].CnPt.x + vRltObs[0].fVx * nDeltaT / 1000;
					nPixelY = vRltObs[0].CnPt.y + vRltObs[0].fVy * nDeltaT / 1000;
					//mPort.SendAngle(nHangle, 800);
					mPort.SendAngleLocation(nHangle, 800, nPixelX, nPixelY);
					sPreSendObs = vRltObs[0];
					cout << nPixelX << ",  " << nPixelY << endl;
				}
			}
			else
			{
				//mPort.SendAngle(2000, 2000);
				mPort.SendAngleLocation(20000, 20000, 20000, 20000);
			}
			for (unsigned int nI = 0; nI < vRltObs.size(); nI++)
			{
				//cout << vRltObs[nI].fVx << ", " << vRltObs[nI].fVy << endl;
				cvRectangle(RgbImage, vRltObs[nI].FrPt, vRltObs[nI].NrPt, cvScalar(255, 0, 0));
			}

		}
		else
		{
			//mPort.SendAngle(2000, 2000);
			mPort.SendAngleLocation(20000, 20000, 20000, 20000);
		}

		vPreObs.clear();
		vPreObs = vRltObs;
		//sprintf(ImgName, "D:\\DJI\\testfile\\avi\\Blue\\%d.jpg", nIndex);
		cvShowImage("TickWhite", TmpImage);
		cvShowImage("RGB", RgbImage);
		//cvSaveImage(ImgName, TmpImage);
		cvWaitKey(1);
		sPreTm = sTm;
		nIndex++;
		DstData = NULL;
		pcvSeq = NULL;
		vTmpObs.clear();
		vRltObs.clear();
		vSObs.clear();
		cvReleaseImage(&pOutlineImage);
		pFrame0 = cvQueryFrame(pCapture0);
	}
	cvReleaseCapture(&pCapture0);
	cvDestroyAllWindows();

	return 0 ;
}