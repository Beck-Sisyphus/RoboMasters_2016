// receiveing Grid Map and Checking Big Box
// chunjia.zhang  3012.08.24

#include "stdafx.h"
#include "GridMap.h"
#include "Show.h"

extern CLog   g_Log;
extern CShow  g_Show;


UINT T_GridMap(LPVOID lpParameter)
{
	g_Log.RcdLog(_T("T_GridMap Is Started"));

	CGridMap * pCGridMap = (CGridMap *)lpParameter;

	SOCKET SocketGridMap;
	struct sockaddr_in SockAddrGridMapFrom;
	int nLenSockAddrGridMapFrom = sizeof(sockaddr_in);
	struct sockaddr_in SockAddrGridMap;
	SockAddrGridMap.sin_family = AF_INET;
	SockAddrGridMap.sin_port = htons(SOCKET_GRID_MAP_RECV_PORT);       //slave blade port
	SockAddrGridMap.sin_addr.s_addr = inet_addr(SOCKET_GRID_MAP_RECV_IP);
	SocketGridMap = socket(AF_INET, SOCK_DGRAM, 0);
	if (-1 == bind(SocketGridMap, (sockaddr*)&SockAddrGridMap, sizeof(SockAddrGridMap)))
	{
		g_Log.RcdLog(_T("Failed to Bind IP and Port of Grid Map Receive"));

		return -1;
	}

	g_Log.RcdLog(_T("Succeed to Bind IP and Port for Grid Map Receive"));

	WaitForSingleObject(pCGridMap->m_hMutexWork, INFINITE);
	BOOL bWork = pCGridMap->m_bWork;
	ReleaseMutex(pCGridMap->m_hMutexWork);

	int nRecv = 0;
	//char sBuf[sizeof(SGridMapTrans)];
	SGridMapTrans GridMapRecv;

	while (bWork)
	{
		memset(&GridMapRecv, 0, sizeof(SGridMapTrans));
		nRecv = recvfrom(SocketGridMap,
			(char *)& GridMapRecv,
			sizeof(SGridMapTrans),
			0,
			(struct sockaddr*)&SockAddrGridMapFrom,
			&nLenSockAddrGridMapFrom);

		if (nRecv < 0)
		{
			g_Log.RcdLog(_T("Failed to Receive Grid Map Data"));

			continue;
		}

		// Extract the Grid Map form the Binary bite to bool
		pCGridMap->ExtractGridMap(GridMapRecv);

		// Update the Buffer for Data Transport
		WaitForSingleObject(pCGridMap->m_hMutexGridMapCurr, INFINITE);
		pCGridMap->m_GridMapCurr = pCGridMap->m_GridMap;
		ReleaseMutex(pCGridMap->m_hMutexGridMapCurr);

		// Record the Grid Map
		pCGridMap->RcdGridMap();

		// Update the Flag of Work or Not
		WaitForSingleObject(pCGridMap->m_hMutexWork, INFINITE);
		BOOL bWork = pCGridMap->m_bWork;
		ReleaseMutex(pCGridMap->m_hMutexWork);
	}

	//
	closesocket(SocketGridMap);
	//
	pCGridMap->ReleseResourse();

	//
	return 0;
}


bool Cvt2SGridMap(const char * GridMapDir, SGridMap & GridMap)
{
	ifstream ifGridMap;
	ifGridMap.open(GridMapDir);
	if (!ifGridMap)
	{
		return false;
	}

	int RowCounter = 0, ColCounter = 0;
	string tempStr;
	while (!ifGridMap.eof())
	{
		getline(ifGridMap, tempStr);
		if (!tempStr.size())
		{
			continue;
		}
		ColCounter = 0;
		while (ColCounter < T_WIDTH_GRID_MAP)
		{
			if (tempStr[2 * ColCounter] == '1')
			{
				GridMap.bGridMap[RowCounter][ColCounter] = true;
			}
			else
			{
				GridMap.bGridMap[RowCounter][ColCounter] = false;
			}
			ColCounter++;
		}
		RowCounter++;
	}
	ifGridMap.close();
	return true;
}

bool ReadGridMap(SGridMap & GridMap, IplImage * pImg)
{
	cvSetZero(pImg);
	uchar * ImgData = (uchar *)pImg->imageData;
	int step = pImg->width / sizeof(uchar);
	for (int y = 0; y < T_HIGH_GRID_MAP; y++)
	{
		for (int x = 125; x < T_WIDTH_GRID_MAP - 125; x++)
		{
			ImgData[y * step + x] = int(GridMap.bGridMap[y][x]) * 255;
		}
	}
	return true;
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
IplImage * FindConnectedComp(IplImage * binaryImg, vector <int> & Label, vector<vector<CvPoint>> & BlobLabel, int MinPtsNum)
{
	BlobLabel.clear();
	//int dx[2] = {-1,  0};    //四邻域
	//int dy[2] = { 0, -1};
	int dx[4] = { -1, -1, 0, 1 };   //八邻域
	int dy[4] = { 0, -1, -1, -1 };

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
int FindLargestBox(vector<SObstacleInfo> & vecBox)
{
	int MaxId = 0;
	float EdgeLength1 = 0, EdgeLength2 = 0, Area = 0, MaxArea = 0;
	for (vector<SObstacleInfo>::size_type ix = 0; ix != vecBox.size(); ix++)
	{
		EdgeLength1 = DistPointXY(vecBox[ix].LeftBack, vecBox[ix].LeftFront);
		EdgeLength2 = DistPointXY(vecBox[ix].LeftBack, vecBox[ix].RightBack);
		Area = EdgeLength1 * EdgeLength2;
		if (Area > MaxArea)
		{
			MaxArea = Area;
			MaxId = ix;
		}
	}
	return MaxId;
}
void MergeBox(vector <SObstacleInfo> & vecBox, SObstacleInfo & tempMergeBox)
{
	int PtsNum = vecBox.size() * 4;
	CvPoint2D32f * PointsArr = new CvPoint2D32f[PtsNum];
	for (vector<SObstacleInfo>::size_type i = 0; i != vecBox.size(); i++)
	{
		PointsArr[4 * i] = cvPoint2D32f(vecBox[i].LeftBack.x, vecBox[i].LeftBack.y);
		PointsArr[4 * i + 1] = cvPoint2D32f(vecBox[i].LeftFront.x, vecBox[i].LeftFront.y);
		PointsArr[4 * i + 2] = cvPoint2D32f(vecBox[i].RightBack.x, vecBox[i].RightBack.y);
		PointsArr[4 * i + 3] = cvPoint2D32f(vecBox[i].RightFront.x, vecBox[i].RightFront.y);
	}
	CvMat pointMat = cvMat(1, PtsNum, CV_32FC2, PointsArr);
	CvBox2D MLeanBox = cvMinAreaRect2(&pointMat, 0);

	int MaxId = FindLargestBox(vecBox);
	tempMergeBox.a = vecBox[MaxId].a;
	tempMergeBox.v = vecBox[MaxId].v;
	tempMergeBox.Heading = vecBox[MaxId].Heading;
	tempMergeBox.ObsLaneNo = vecBox[MaxId].ObsLaneNo;
	tempMergeBox.ObstacleID = vecBox[MaxId].ObstacleID;
	CvPoint2D32f pt[4];
	cvBoxPoints(MLeanBox, pt);
	tempMergeBox.Center.x = (pt[0].x + pt[1].x + pt[2].x + pt[3].x) / 4;
	tempMergeBox.Center.y = (pt[0].y + pt[1].y + pt[2].y + pt[3].y) / 4;
	tempMergeBox.LeftFront.x = pt[0].x;
	tempMergeBox.LeftFront.y = pt[0].y;

	tempMergeBox.LeftBack.x = pt[1].x;
	tempMergeBox.LeftBack.y = pt[1].y;

	tempMergeBox.RightBack.x = pt[2].x;
	tempMergeBox.RightBack.y = pt[2].y;

	tempMergeBox.RightFront.x = pt[3].x;
	tempMergeBox.RightFront.y = pt[3].y;
	free(PointsArr);
}
void GetBoxRange(SObstacleInfo & LuxBox, vector<float> & BoxRange)
{
	float XCoodinate[4] = {}, YCoodinate[4] = {};
	XCoodinate[0] = LuxBox.LeftBack.x;
	XCoodinate[1] = LuxBox.LeftFront.x;
	XCoodinate[2] = LuxBox.RightBack.x;
	XCoodinate[3] = LuxBox.RightFront.x;

	YCoodinate[0] = LuxBox.LeftBack.y;
	YCoodinate[1] = LuxBox.LeftFront.y;
	YCoodinate[2] = LuxBox.RightBack.y;
	YCoodinate[3] = LuxBox.RightFront.y;

	float fMinX = *(min_element(XCoodinate, XCoodinate + 4));
	float fMinY = *(min_element(YCoodinate, YCoodinate + 4));
	float fMaxX = *(max_element(XCoodinate, XCoodinate + 4));
	float fMaxY = *(max_element(YCoodinate, YCoodinate + 4));
	BoxRange.push_back(fMinX);
	BoxRange.push_back(fMaxX);
	BoxRange.push_back(fMinY);
	BoxRange.push_back(fMaxY);
}
bool CvtBoxV2I(SObstacleInfo & LuxBoxV, SObstacleInfo & LuxBoxI)
{
	bool flag[5] = {};
	flag[0] = CvtV2I(LuxBoxV.LeftFront, LuxBoxI.LeftFront);
	flag[1] = CvtV2I(LuxBoxV.LeftBack, LuxBoxI.LeftBack);
	flag[2] = CvtV2I(LuxBoxV.RightFront, LuxBoxI.RightFront);
	flag[3] = CvtV2I(LuxBoxV.RightBack, LuxBoxI.RightBack);
	flag[4] = (flag[0] && flag[1] && flag[2] && flag[3]);
	return flag[4];
}
float CalLineVal(SPointXY & Pt1, SPointXY & Pt2, float x)   //根据Pt1，Pt2求解直线方程y = kx + b, 将x带入,求解y值
{
	if (Pt1.x == Pt2.x)
	{
		return 0;
	}
	else
	{
		float k = (Pt1.y - Pt2.y) / (Pt1.x - Pt2.x);
		float b = Pt1.y - k * Pt2.x;
		return k * x + b;
	}
}
bool IsWithinBox(CvPoint Pt, SObstacleInfo & LuxBox, vector <float> & BoxRange, float eps)
{
	if (Pt.x < BoxRange[0] || Pt.x > BoxRange[1] || Pt.y < BoxRange[2] || Pt.y > BoxRange[3])   //点在最大外接矩形之外
	{
		return false;
	}
	else
	{
		float fVal[4] = {};
		if (LuxBox.LeftBack.x == LuxBox.LeftFront.x)   //直线退化为x = b
		{
			fVal[0] = Pt.x - LuxBox.LeftBack.x;
		}
		else                    //直线方程是y = kx + b
		{
			fVal[0] = CalLineVal(LuxBox.LeftBack, LuxBox.LeftFront, Pt.x) - Pt.y;
		}
		if (LuxBox.LeftBack.x == LuxBox.RightBack.x)
		{
			fVal[1] = Pt.x - LuxBox.LeftBack.x;
		}
		else
		{
			fVal[1] = CalLineVal(LuxBox.LeftBack, LuxBox.RightBack, Pt.x) - Pt.y;
		}

		if (LuxBox.RightBack.x == LuxBox.RightFront.x)
		{
			fVal[2] = Pt.x - LuxBox.RightBack.x;
		}
		else
		{
			fVal[2] = CalLineVal(LuxBox.RightBack, LuxBox.RightFront, Pt.x) - Pt.y;
		}
		if (LuxBox.LeftFront.x == LuxBox.RightFront.x)
		{
			fVal[2] = Pt.x - LuxBox.LeftFront.x;
		}
		else
		{
			fVal[3] = CalLineVal(LuxBox.LeftFront, LuxBox.RightFront, Pt.x) - Pt.y;
		}
		if (fVal[0] * fVal[2] <= 0 && fVal[1] * fVal[3] <= 0)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
}

void GetBlobLuxLabel(SGridMap & GridMap, SObstacle & Obs, vector<vector<int>> & BlobLuxID, IplImage * pLabelImg)
{
	CvPoint BoxCorner[4] = {};
	float XCoodinate[4] = {}, YCoodinate[4] = {};
	float k[4] = {};   //直线方程:y = kx + b
	float b[4] = {};
	float tempfVal[4] = {};
	ushort * ImgData = (ushort *)pLabelImg->imageData;
	int step = pLabelImg->widthStep / sizeof(short);
	SObstacleInfo tempLuxBox, tempExdBox;
	for (int ix = 0; ix != Obs.ObstacleNum; ix++)
	{
		if (FALSE//Obs.Obs[ix].ObstacleID == e_Static
			|| !ObsIn3Lane(Obs.Obs[ix], GridMap))
		{
			continue;
		}
		ExtendBox(Obs.Obs[ix], tempExdBox, 1.5f);
		bool isLuxInImg = CvtBoxV2I(tempExdBox, tempLuxBox);
		//bool isLuxInImg = CvtBoxV2I(Obs.Obs[ix], tempLuxBox);
		if (!isLuxInImg)
		{
			continue;
		}
		vector <float> BoxRange;   // 计算最大外接矩形
		GetBoxRange(tempLuxBox, BoxRange);
		int tempLabel = 0;
		bool isInBox = false;
		for (int y = int(BoxRange[2]); y <= int(BoxRange[3]); y++)   //遍历LuxBox内的图像元素
		{
			for (int x = int(BoxRange[0]); x <= int(BoxRange[1]); x++)
			{
				isInBox = IsWithinBox(cvPoint(x, y), tempLuxBox, BoxRange, 1);
				if (isInBox)
				{
					tempLabel = ImgData[y * step + x];
					if (tempLabel != 0)
					{
						vector<int>::iterator iter = find(BlobLuxID[tempLabel - 1].begin(), BlobLuxID[tempLabel - 1].end(), ix);
						if (iter == BlobLuxID[tempLabel - 1].end())
						{
							BlobLuxID[tempLabel - 1].push_back(ix);
						}
					}
				}
			}
		}
	}
}
float CalMinDistBetween2PtsSet(vector<CvPoint> & PtsSet1, vector<CvPoint> & PtsSet2)   //计算两个点集的最近距离
{
	float MinDist = 10000.0f, tempDist = 0.0f;
	for (vector<CvPoint>::size_type ix = 0; ix != PtsSet1.size(); ix++)
	{
		for (vector<CvPoint>::size_type i = 0; i != PtsSet2.size(); i++)
		{
			tempDist = sqrt((float)((PtsSet1[ix].x - PtsSet2[i].x) * (PtsSet1[ix].x - PtsSet2[i].x)
				+ (PtsSet1[ix].y - PtsSet2[i].y) * (PtsSet1[ix].y - PtsSet2[i].y)));
			if (tempDist < MinDist)
			{
				MinDist = tempDist;
			}
		}
	}
	return MinDist;
}

void MergeBlobLabel(vector<vector<CvPoint>> & BlobLabel, float DistThr, IplImage * pLabelImg)    //合并距离小的BlobLabel，并重新计算pLabelImg
{
	//float Dist = 0;
	//for (int ix = 0; ix != BlobLabel.size(); ix++)
	//{
	//	for (int i = 0; i != BlobLabel.size(); i++)
	//	{
	//		if (i == ix)
	//		{
	//			continue;
	//		}
	//		Dist = CalMinDistBetween2PtsSet(BlobLabel[ix], BlobLabel[i]);
	//		if (Dist <= DistThr)    //合并距离较小的Label
	//		{
	//			BlobLabel[ix].insert(BlobLabel[ix].end(), BlobLabel[i].begin(), BlobLabel[i].end());  //合并过程 
	//			BlobLabel[i].clear();
	//		}
	//	}
	//}
	//for (int ix = 0; ix != BlobLabel.size(); ix++)
	//{
	//	if (BlobLabel[ix].empty())
	//	{
	//		BlobLabel.erase(BlobLabel.begin() + ix);
	//		ix++;
	//	}
	//}
	//cvSetZero(pLabelImg);
	//ushort * ImgData = (ushort *)pLabelImg->imageData;
	//int step = pLabelImg->widthStep / sizeof(ushort);
	//int x = 0, y =0; 
	//for (int ix = 0; ix != BlobLabel.size(); ix++)
	//{
	//	for (int i = 0; i != BlobLabel[ix].size(); i++)
	//	{
	//		x = BlobLabel[ix][i].x;
	//		y = BlobLabel[ix][i].y;
	//		ImgData[y * step + x] = ix + 1;
	//	}
	//}
}
int  LuxFusion(SObstacle & OriginalObs, SObstacle & SendObs, vector<vector<int>> & BlobLuxID)
{
	CvBox2D MLeanBox;
	SObstacleInfo tempMergeBox;
	vector <SObstacleInfo> tempVecBox;
	vector <SObstacleInfo> vecMergeBox;
	SObstacleInfo tempLuxBox;
	int LuxIdx = 0, MergeBoxCounter = 0;
	vector<int> vecMergeLuxID;
	memset(&SendObs, 0, sizeof(SObstacle));

	for (vector<vector<int>>::size_type ix = 0; ix != BlobLuxID.size(); ix++)
	{
		if (BlobLuxID[ix].size() > 1)
		{
			tempVecBox.clear();
			vector <int> vIdUsed;

			for (vector<int>::size_type i = 0; i != BlobLuxID[ix].size(); i++)
			{
				LuxIdx = BlobLuxID[ix][i];
				tempVecBox.push_back(OriginalObs.Obs[LuxIdx]);
				vIdUsed.push_back(LuxIdx);
				//vecMergeLuxID.push_back(LuxIdx);
			}
			if (tempVecBox.size() > 0)
			{
				MergeBox(tempVecBox, tempMergeBox);
				if ((DistPointXY(tempMergeBox.LeftFront, tempMergeBox.LeftBack)
					+ DistPointXY(tempMergeBox.LeftFront, tempMergeBox.RightFront)) < 8)
				{
					SendObs.Obs[SendObs.ObstacleNum] = tempMergeBox;
					SendObs.ObstacleNum++;
					MergeBoxCounter++;
					vecMergeLuxID.insert(vecMergeLuxID.end(), vIdUsed.begin(), vIdUsed.end());
				}
			}
		}
	}
	vector<int>::iterator iter;
	for (int ix = 0; ix != OriginalObs.ObstacleNum; ix++)
	{
		iter = find(vecMergeLuxID.begin(), vecMergeLuxID.end(), ix);
		if (iter == vecMergeLuxID.end())  //说明这个Box与联通区域没有对应
		{
			SendObs.Obs[SendObs.ObstacleNum] = OriginalObs.Obs[ix];
			SendObs.ObstacleNum++;
		}
	}
	SendObs.IsOK = OriginalObs.IsOK;
	//cout<<"Original Obstacle Num: "<<OriginalObs.ObstacleNum<<"   Send Obstacle Num: "<<SendObs.ObstacleNum<<endl;
	return MergeBoxCounter;
}

IplImage *  ObsMergeGridMap(SObstacle & OriginalObs, SGridMap & GridMap, SObstacle & SendObs)
{
	memset(&SendObs, 0, sizeof(SObstacle));
	IplImage * pBinaryImg = NULL;
	IplImage * pOpenImg = NULL;
	IplImage * pLabelImg = NULL;

	pBinaryImg = cvCreateImage(cvSize(T_WIDTH_GRID_MAP, T_HIGH_GRID_MAP), IPL_DEPTH_8U, 1);
	pOpenImg = cvCreateImage(cvSize(T_WIDTH_GRID_MAP, T_HIGH_GRID_MAP), IPL_DEPTH_8U, 1);

	ReadGridMap(GridMap, pBinaryImg);

	IplConvKernel * StrtElem;    //设置结构元素
	int WinLength = 5;
	StrtElem = cvCreateStructuringElementEx(WinLength, WinLength, WinLength / 2, WinLength / 2, CV_SHAPE_RECT);
	cvCopy(pBinaryImg, pOpenImg);
	cvMorphologyEx(pBinaryImg, pOpenImg, NULL, StrtElem, CV_MOP_CLOSE, 1);

	int MinPtsNum = 1;
	vector <int> Label;
	vector<vector<CvPoint>> BlobLabel;
	pLabelImg = FindConnectedComp(pOpenImg, Label, BlobLabel, MinPtsNum);   //连通区域分析

	CString sBuf;
	sBuf.Format(_T("BlobLabel= %d "), BlobLabel.size());
	g_Show.PrintText(sBuf, 4);

	//MergeBlobLabel(BlobLabel, 4, pLabelImg);

	vector<vector<int>> BlobLuxID(BlobLabel.size());   //找出每个联通区域对应的LuxID
	GetBlobLuxLabel(GridMap, OriginalObs, BlobLuxID, pLabelImg);
	int MergeBoxCounter = LuxFusion(OriginalObs, SendObs, BlobLuxID);

	cvReleaseImage(&pBinaryImg);
	cvReleaseImage(&pOpenImg);
	//cvReleaseImage(&pLabelImg);

	return pLabelImg;
}


BOOL  ObsIn3Lane(SObstacleInfo Obs, SGridMap & GridMap)
{
	if (GridMap.nNumBoundary == 2)
	{
		//
		GridMap.dB = -GridMap.dA * T_DISTANCE_FRONT_TO_CENTOR;
		double  dDistLeft = DistPointLine(GridMap.PtLeftBack, GridMap.dA, GridMap.dB);
		double  dDistRight = DistPointLine(GridMap.PtRightBack, GridMap.dA, GridMap.dB);

		double dDistLeftBack = DistPointLine(Obs.LeftBack, GridMap.dA, GridMap.dB);
		double dDistLeftFront = DistPointLine(Obs.LeftFront, GridMap.dA, GridMap.dB);
		double dDistRightBack = DistPointLine(Obs.RightBack, GridMap.dA, GridMap.dB);
		double dDistRightFront = DistPointLine(Obs.RightFront, GridMap.dA, GridMap.dB);

		double dDistMax = min(T_DISTANCE_POINT_IN_3_LANE, dDistLeft);
		double dDistMin = max(-T_DISTANCE_POINT_IN_3_LANE, dDistRight);

		if (dDistLeftBack > dDistMin
			&& dDistLeftBack < dDistMax
			&& dDistLeftFront > dDistMin
			&& dDistLeftFront < dDistMax
			&& dDistRightBack > dDistMin
			&& dDistRightBack < dDistMax
			&& dDistRightFront > dDistMin
			&& dDistRightFront < dDistMax)
		{
			return TRUE;
		}
	}

	return FALSE;

	//if (   Obs.LeftBack.y < T_MAX_Y_POINT_IN_3_LANE
	//	&& Obs.LeftBack.y > T_MIN_Y_POINT_IN_3_LANE
	//	&& Obs.LeftFront.y < T_MAX_Y_POINT_IN_3_LANE
	//	&& Obs.LeftFront.y > T_MIN_Y_POINT_IN_3_LANE
	//	&& Obs.RightBack.y < T_MAX_Y_POINT_IN_3_LANE
	//	&& Obs.RightBack.y > T_MIN_Y_POINT_IN_3_LANE
	//	&& Obs.RightFront.y < T_MAX_Y_POINT_IN_3_LANE
	//	&& Obs.RightFront.y > T_MIN_Y_POINT_IN_3_LANE)
	//{
	//	return TRUE;
	//}
	//return FALSE;
}

bool ReadGridMap2(SGridMap & GridMap, IplImage * pImg)
{
	cvSetZero(pImg);
	uchar * ImgData = (uchar *)pImg->imageData;
	int step = pImg->width / sizeof(uchar);
	for (int y = 150; y < T_HIGH_GRID_MAP / 2; y++)
	{
		for (int x = 175; x < T_WIDTH_GRID_MAP - 175; x++)
		{
			ImgData[y * step + x] = int(GridMap.bGridMap[y][x]) * 255;
		}
	}
	return true;
}



bool ReadGridMapForTree(SGridMap & GridMap, IplImage * pImg)
{
	cvSetZero(pImg);
	uchar * ImgData = (uchar *)pImg->imageData;
	int step = pImg->width / sizeof(uchar);
	for (int y = 50; y < T_HIGH_GRID_MAP / 2; y++)
	{
		for (int x = 175; x < T_WIDTH_GRID_MAP - 175; x++)
		{
			ImgData[y * step + x] = int(GridMap.bGridMap[y][x]) * 255;
		}
	}
	return true;
}



void FindCone(SGridMap & GridMap, SObstacle & SendObs)
{
	memset(&SendObs, 0, sizeof(SObstacle));
	IplImage * pBinaryImg = NULL;
	IplImage * pLabelImg = NULL;
	//IplImage * pColorLabelImg  = NULL;
	IplImage * pOpenImg = NULL;

	pBinaryImg = cvCreateImage(cvSize(T_WIDTH_GRID_MAP, T_HIGH_GRID_MAP), IPL_DEPTH_8U, 1);
	pOpenImg = cvCreateImage(cvSize(T_WIDTH_GRID_MAP, T_HIGH_GRID_MAP), IPL_DEPTH_8U, 1);

	ReadGridMap2(GridMap, pBinaryImg);

	IplConvKernel * StrtElem;    //设置结构元素
	int WinLength = 2;
	StrtElem = cvCreateStructuringElementEx(WinLength, WinLength, WinLength / 2, WinLength / 2, CV_SHAPE_RECT);
	cvCopy(pBinaryImg, pOpenImg);
	cvMorphologyEx(pBinaryImg, pOpenImg, NULL, StrtElem, CV_MOP_CLOSE, 1);


	int MinPtsNum = 1;
	vector <int> Label;
	vector<vector<CvPoint>> BlobLabel;
	pLabelImg = FindConnectedComp(pOpenImg, Label, BlobLabel, MinPtsNum);   //连通区域分析

	GetCCABox(BlobLabel, SendObs);

	cvReleaseImage(&pBinaryImg);
	cvReleaseImage(&pOpenImg);
	cvReleaseImage(&pLabelImg);
	//cvReleaseImage(&pColorLabelImg);
}

void ReMoveTree(SGridMap & GridMap, SObstacle & SendObs)
{
	memset(&SendObs, 0, sizeof(SObstacle));
	IplImage * pBinaryImg = NULL;
	IplImage * pLabelImg = NULL;
	//IplImage * pColorLabelImg  = NULL;
	IplImage * pOpenImg = NULL;

	pBinaryImg = cvCreateImage(cvSize(T_WIDTH_GRID_MAP, T_HIGH_GRID_MAP), IPL_DEPTH_8U, 1);
	pOpenImg = cvCreateImage(cvSize(T_WIDTH_GRID_MAP, T_HIGH_GRID_MAP), IPL_DEPTH_8U, 1);

	ReadGridMapForTree(GridMap, pBinaryImg);

	IplConvKernel * StrtElem;    //设置结构元素
	int WinLength = 2;
	StrtElem = cvCreateStructuringElementEx(WinLength, WinLength, WinLength / 2, WinLength / 2, CV_SHAPE_RECT);
	cvCopy(pBinaryImg, pOpenImg);
	cvMorphologyEx(pBinaryImg, pOpenImg, NULL, StrtElem, CV_MOP_CLOSE, 1);


	int MinPtsNum = 1;
	vector <int> Label;
	vector<vector<CvPoint>> BlobLabel;
	pLabelImg = FindConnectedComp(pOpenImg, Label, BlobLabel, MinPtsNum);   //连通区域分析

	GetCCABox(BlobLabel, SendObs);

	cvReleaseImage(&pBinaryImg);
	cvReleaseImage(&pOpenImg);
	cvReleaseImage(&pLabelImg);
	//cvReleaseImage(&pColorLabelImg);
}



void FitPts2Box(vector <CvPoint> & vecPts, SObstacleInfo & ResultBox)
{
	int PtsNum = vecPts.size();
	CvPoint2D32f * PointsArr = new CvPoint2D32f[PtsNum];
	for (vector<SObstacleInfo>::size_type i = 0; i != vecPts.size(); i++)
	{
		PointsArr[i] = cvPoint2D32f(vecPts[i].x, vecPts[i].y);
	}
	CvMat pointMat = cvMat(1, PtsNum, CV_32FC2, PointsArr);
	CvBox2D MLeanBox = cvMinAreaRect2(&pointMat, 0);

	ResultBox.a = 0;
	ResultBox.v = 0;
	ResultBox.Heading = 0;
	ResultBox.ObsLaneNo = 0;
	//ResultBox.ObstacleID = e_Static;
	CvPoint2D32f pt[4];
	cvBoxPoints(MLeanBox, pt);
	ResultBox.Center.x = (pt[0].x + pt[1].x + pt[2].x + pt[3].x) / 4;
	ResultBox.Center.y = (pt[0].y + pt[1].y + pt[2].y + pt[3].y) / 4;
	ResultBox.LeftFront.x = pt[0].x;
	ResultBox.LeftFront.y = pt[0].y;

	ResultBox.LeftBack.x = pt[1].x;
	ResultBox.LeftBack.y = pt[1].y;

	ResultBox.RightBack.x = pt[2].x;
	ResultBox.RightBack.y = pt[2].y;

	ResultBox.RightFront.x = pt[3].x;
	ResultBox.RightFront.y = pt[3].y;
	free(PointsArr);
}
void CvtI2V(SPointXY & PtI, SPointXY & PtV)
{
	PtV.x = (T_HIGH_GRID_MAP / 2 - PtI.y) * GridPrecision;
	PtV.y = (T_WIDTH_GRID_MAP / 2 - PtI.x) * GridPrecision;
}
void CvtImg2Obs(SObstacleInfo & BoxI, SObstacleInfo & BoxV)
{
	memset(&BoxV, 0, sizeof(SObstacleInfo));
	CvtI2V(BoxI.LeftBack, BoxV.LeftBack);
	CvtI2V(BoxI.LeftFront, BoxV.LeftFront);
	CvtI2V(BoxI.RightBack, BoxV.RightBack);
	CvtI2V(BoxI.RightFront, BoxV.RightFront);
	BoxV.Center.x = (BoxV.LeftFront.x + BoxV.LeftBack.x + BoxV.RightFront.x + BoxV.RightBack.x) / 4;
	BoxV.Center.y = (BoxV.LeftFront.y + BoxV.LeftBack.y + BoxV.RightFront.y + BoxV.RightBack.y) / 4;
	//BoxV.v = 0;
	BoxV.Heading = -180;
	BoxV.ObstacleID = e_Static;
}
void GetCCABox(vector<vector<CvPoint>> & BlobLabel, SObstacle & Obs)
{
	SObstacleInfo tempObsInfo, tempObsV;
	memset(&tempObsInfo, 0, sizeof(SObstacleInfo));
	memset(&Obs, 0, sizeof(SObstacle));
	for (int i = 0; i != BlobLabel.size(); i++)
	{
		if (BlobLabel[i].size())
		{
			FitPts2Box(BlobLabel[i], tempObsInfo);
			CvtImg2Obs(tempObsInfo, tempObsV);
			//if (MeanLengthWidthObs(tempObsV) < 1)
			//{
			//	Obs.Obs[Obs.ObstacleNum] = tempObsV;
			//	Obs.ObstacleNum++;
			//}
			Obs.Obs[Obs.ObstacleNum] = tempObsV;
			Obs.ObstacleNum++;
			if (Obs.ObstacleNum >= T_NUM_MAX_OBS)
			{
				break;
			}
		}
	}
}