#include "stdafx.h"
#include "MyHead.h"

extern CvSize Size;
extern CvScalar Black;
extern CvScalar White;
void MatchTemplate(IplImage* imgSrc,IplImage* imgTemp)  

{   

	//double a=0.;    
	CvSize sizeSrc = cvGetSize(imgSrc);   
	CvSize sizeTemp = cvGetSize(imgTemp);   
	CvSize sizeResult = cvSize(sizeSrc.width-sizeTemp.width+1,sizeSrc.height-sizeTemp.height+1);   
	IplImage* imgResult = cvCreateImage(sizeResult,IPL_DEPTH_32F,1);   
	cvMatchTemplate(imgSrc,imgTemp,imgResult,CV_TM_SQDIFF);   

	float dMax = 0.;   
	CvPoint point = cvPoint(0,0);  

	//if(!a) MessageBox("Not Successful!",MB_OK);  

	for (int cx=0 ; cx  < imgResult->width;cx++)
	{   
		for (int cy=0 ; cy  < imgResult->height;cy++)
		{   
			float fTemp = CV_IMAGE_ELEM(imgResult,float,cy,cx);   
			if (dMax < fTemp) //找到最接近的位置   
			{   
				dMax = fTemp;   
				point = cvPoint(cx,cy); //记录位置   
			}   
		}   
	}   
	CvPoint point2 = cvPoint(point.x+sizeTemp.width,point.y+sizeTemp.height); //对角位置   
	cvRectangle(imgSrc,point,point2,cvScalar(255));   
	cvNamedWindow( "Test", CV_WINDOW_AUTOSIZE );   
	cvShowImage("Test",imgSrc);   
	/*   for(;;)  
	{  
	int k = cvWaitKey(5);  
	if( k == 27 ) break;  
	}  
	*/
}  

/*
void scan_row(IplImage* img)
{
	int i;
	int j;
	PointNum = 0;
	for(i=0;i<img->width;i=i+25)
	{
		for (j=0;j<90;j=j+1)
		{
			if (cvGet2D(img,i,j).val[0]==255 )
			{
				PointNum++;
			}
		}
	}
	if (PointNum>=2)
	{
		printf("get box!\n");
	}
}



*/
int cvThresholdOtsu(IplImage* src)  
{  
	int height=src->height;  
	int width=src->width;      

	//histogram  
	float histogram[256]={0};  
	for(int i=0;i<height;i++) {  
		unsigned char* p=(unsigned char*)src->imageData+src->widthStep*i;  
		for(int j=0;j<width;j++) {  
			histogram[*p++]++;  
		}  
	}  
	//normalize histogram  
	int size=height*width;  
	for(int i=0;i<256;i++) {  
		histogram[i]=histogram[i]/size;  
	}  

	//average pixel value  
	float avgValue=0;  
	for(int i=0;i<256;i++) {  
		avgValue+=i*histogram[i];  
	}  

	int threshold;    
	float maxVariance=0;  
	float w=0,u=0;  
	for(int i=0;i<256;i++) {  
		w+=histogram[i];  
		u+=i*histogram[i];  

		float t=avgValue*w-u;  
		float variance=t*t/(w*(1-w));  
		if(variance>maxVariance) {  
			maxVariance=variance;  
			threshold=i;  
		}  
	}  

	return threshold;  
}  


int Scan_Diff(IplImage* img)
{
	int i,j,k;
	static	BOOL flag;
	CvPoint GetPonit[20];
	unsigned char* Data = (unsigned char*)img->imageData;
	
	int step = img->widthStep/sizeof(unsigned char);
	//搜索方向 box在右，左向右搜。box在左，右向左搜。flag标志为box的位置
	//左向右横向搜索
	if (flag == RIGHT)
	{
		for (i=0;i<img->width;i++)
		{
			if (Data[img->height/2*step+i]==255)
			{
				cvLine(img,cvPoint(i,0),cvPoint(i,img->height),White);
				if (i<img->width/2)
				{
					flag = LEFT;//换边
				}
				break;
			}
		}
	}
	//右向左横向搜索
	else{
		for (i=img->width;i>0;i--)
		{
			if (Data[img->height/2*step+i]==255)
			{
				cvLine(img,cvPoint(i,0),cvPoint(i,img->height),White);
				if (i>img->width/2)
				{
					flag = RIGHT;//换边
				}
				break;
			}
		}
	}
	cvShowImage("GRAY",img);
	//cvWaitKey();
	return i;
}

/*
for(i=0;i<Size.width;i=i+25)
{
for (j=0;j<Size.height/2;j++)
{
//cvCircle(gray_frame_2,cvPoint(i,j),1,CV_RGB(0,0,0),1);
//cvShowImage("vedio0",gray_frame_2);
//cvWaitKey(1);
//cvWaitKey(10);

if (DataGrayFrame2[j*step+i]==0)
{
GetPonit[PointNum].x=i;
GetPonit[PointNum].y=j;
cvCircle(gray_frame_2,cvPoint(i,j),10,CV_RGB(0,0,0),1);
cvShowImage("vedio1",gray_frame_2);
cvWaitKey(1);
//	cvLine(gray_frame_2,cvPoint(0,j),cvPoint(i,j),CV_RGB(0,0,0));
PointNum++;
break;
}

}

}
if (PointNum>=2)
{
printf("get box!\n");
//两点在正面顶部
if (abs(GetPonit[0].y - GetPonit[1].y) <= 2)
{
//找左顶点
for(i=0;i<30;i++)
{
if(DataGrayFrame2[GetPonit[0].y*step+GetPonit[0].x-i]!=0  &&
DataGrayFrame2[(GetPonit[0].y+1)*step+GetPonit[0].x-i]!=0 &&
DataGrayFrame2[(GetPonit[0].y+2)*step+GetPonit[0].x-i]!=0 )
{
//	printf("find point!\n");
BoxRect.x = GetPonit[0].x-i;
BoxRect.y = GetPonit[0].y;

//cvCircle(rgb_frame,cvPoint(BoxRect.x+Xstart,BoxRect.y+Ystart),4,CV_RGB(255,0,0),2);
//cvShowImage("RGB",rgb_frame);
//cvWaitKey(1);
break;
}				
}
//找右顶点
for (i=0;i<30;i++)
{
if (DataGrayFrame2[(GetPonit[1].y+1)*step+GetPonit[1].x+i] != 0)
{
BoxRect.width = GetPonit[1].x+i-BoxRect.x;

//cvCircle(rgb_frame,cvPoint(BoxRect.x+Xstart,BoxRect.y+Ystart),4,CV_RGB(255,0,0),2);
//cvCircle(rgb_frame,cvPoint(BoxRect.x+Xstart+BoxRect.width,BoxRect.y+Ystart),4,CV_RGB(255,0,0),2);
//cvShowImage("RGB",rgb_frame);
//cvWaitKey(1);
break;
}
}
//找左下点
for (i=0;i<100;i++)
{
if (DataGrayFrame2[(BoxRect.y+i)*step+BoxRect.x] != 0 && 
DataGrayFrame2[(BoxRect.y+i)*step+BoxRect.x+1] !=0 && 
DataGrayFrame2[(BoxRect.y+i)*step+BoxRect.x-1] !=0 && 
DataGrayFrame2[(BoxRect.y+i)*step+BoxRect.x+2] !=0&& 
DataGrayFrame2[(BoxRect.y+i)*step+BoxRect.x-2] !=0)
{
BoxRect.height = i;
printf("%d\n",BoxRect.height);
cvRectangle(rgb_frame,cvPoint(BoxRect.x+Xstart,BoxRect.y+Ystart),
cvPoint(BoxRect.x+Xstart+BoxRect.width,BoxRect.y+Ystart+BoxRect.height),
cvScalar(255,0,0),2);
cvShowImage("RGB",rgb_frame);
cvWaitKey(1);
break;
}
}
}




}
else{
printf("no box!\n");
}
*/