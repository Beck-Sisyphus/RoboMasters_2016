/**********************************
**Using ZED with OpenCV
**********************************/

#include <iostream>

#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"

using namespace cv;
using namespace std;


int main(int argc, char** argv)
{
	VideoCapture cap(0); //capture the video from web cam

	if ( !cap.isOpened() )  // if not success, exit program
	{
	 cout << "Cannot open the web cam" << endl;
	 return -1;
	}

	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"



//Blue
//int iLowH = 89;
//int iHighH = 125;

//int iLowS = 64;
//int iHighS = 255;

//int iLowV = 128;
//int iHighV = 255;

	int iLowH = 0;
	int iHighH = 8;

	int iLowS = 100;
	int iHighS = 255;

	int iLowV = 75;
	int iHighV = 255;

	int iLowR = 0;
	int iHighR = 20;

////Create trackbars in "Control" window
///
	cvCreateTrackbar("LowR", "Control", &iLowR, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighR", "Control", &iHighR, 179);
	cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &iHighH, 179);

	cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &iHighV, 255);



// Loop until 'q' is pressed
	while (true) {
		Mat imgOriginal;

		bool bSuccess = cap.read(imgOriginal); // read a new frame from video

		if (!bSuccess) //if not success, break loop
		{
		     cout << "Cannot read a frame from video stream" << endl;
		     break;
		}

		Mat imgHSV;

		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
	 
		Mat imgThresholded;

		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
	      
		//morphological opening (remove small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

	  	//morphological closing (fill small holes in the foreground)
	  	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
	  	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

		imshow("Thresholded Image", imgThresholded); //show the thresholded image
		imshow("Original", imgOriginal); //show the original image

	/*
		uint8_t* pixelPtr = (uint8_t*)imgThresholded.data;
		int x_min = INFINITY;
		int x_max = -INFINITY;
		int y_min = INFINITY;
		int y_max = -INFINITY;
		for(int x = 0; x < width; x++){
			for(int y = 0; y < height; y++){
				if(pixelPtr[x * width + y * height] == 255 && pixelPtr[x * width + y * height + 1] == 255){
					if(x < x_min){
						x_min = x;
					} 
					if(x > x_max){
						x_max = x;
					}
					if(y < y_min){
						y_min = y;
					}
					if(y > y_max){
						y_max = y;
					}
				}
			}
		} */

		// mid point color detection output
		//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		// Need to denoise if want accurate result
		//int x_center = (x_max + x_min)/2;
		//int y_center = (y_max + y_min)/2;
		// Depth, need to decode
		// 0 far away, 255 close
		// need to figure out the actual distance
		//pixelPtr = (uint8_t*) depth.data;
		//uint8_t distance = pixelPtr[x_center * width + y_center * height];

		//printf("%d, %d, %d\n", x_center, y_center, distance);
		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			    cout << "esc key is pressed by user" << endl;
			    break; 
		}

	}
		
}

