/**********************************
**Using ZED with OpenCV
**********************************/

#include <iostream>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// ZED
#include <zed/Camera.hpp>
using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	// Initialize ZED color stream in HD and depth in Performance mode
	sl::zed::Camera* zed = new sl::zed::Camera(sl::zed::HD720);
	sl::zed::ERRCODE err = zed->init(sl::zed::MODE::PERFORMANCE, 0, true);

	// Quit if an error occurred
	if (err != sl::zed::SUCCESS) {
		std::cout << "Unable to init the ZED:" << errcode2str(err) << std::endl;
		delete zed;
		return 1;
	}	

	// Initialize color image and depth
	int width = zed->getImageSize().width;
	int height = zed->getImageSize().height;
	cv::Mat image(height, width, CV_8UC4 ,1);


	// Create OpenCV windows
	//cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
	//cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

	int iLowR = 220;
	int iHighR = 255;

	int iLowG = 0;
	int iHighG = 50;

	int iLowB = 0;
	int iHighB = 50;

	//Create trackbars in "Control" window
	cvCreateTrackbar("LowR", "Control", &iLowR, 220); //Hue (0 - 179)
	cvCreateTrackbar("HighR", "Control", &iHighR, 255);

	cvCreateTrackbar("LowG", "Control", &iLowG, 0); //Saturation (0 - 255)
	cvCreateTrackbar("HighG", "Control", &iHighG, 50);

	cvCreateTrackbar("LowB", "Control", &iLowB, 0); //Value (0 - 255)
	cvCreateTrackbar("HighB", "Control", &iHighB, 50);
	// Settings for windows
	cv::Size displaySize(720, 404);
	cv::Mat imageDisplay(displaySize, CV_8UC4);

	// Loop until 'q' is pressed
	int frame = 0;
	while (frame < 10000) {

	 	// Grab frame and compute depth in FULL sensing mode
		if (!zed->grab(sl::zed::SENSING_MODE::FULL))
		{

			// Retrieve left color image
			sl::zed::Mat right = zed->retrieveImage(sl::zed::SIDE::RIGHT);
			memcpy(image.data,right.data,width*height*4*sizeof(uchar));

			// Retrieve depth map
			//sl::zed::Mat depthmap = zed->normalizeMeasure(sl::zed::MEASURE::DEPTH);
			//memcpy(depth.data,depthmap.data,width*height*4*sizeof(uchar));


			cv::Mat imgHSV;
			//Convert the captured frame from BGR to HSV
			cv::cvtColor(image, imgHSV, CV_BGRA2RGB); 


			cv::Mat imgThresholded;

			cv::inRange(imgHSV, cv::Scalar(iLowR, iLowG, iLowB), cv::Scalar(iHighR, iHighG, iHighB), imgThresholded); //Threshold the image

			/*for(int i = 0; i < width*height*3; i++){
				
				printf("%d ", imgThresholded.data[i]);
							
			}*/
			//morphological opening (remove small objects from the foreground)
			cv::erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, cv::Size(5, 5)) );
			cv::dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, cv::Size(5, 5)) );

			//morphological closing (fill small holes in the foreground)
			cv::dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, cv::Size(5, 5)) );
			cv::erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, cv::Size(5, 5)) );
			
			cv::resize(imgThresholded, imageDisplay, displaySize);
			cv::imshow("Thresholded Image", imgThresholded); //show the thresholded image
			cv::resize(image, imageDisplay, displaySize);
			cv::imshow("Original", image); //show the original image
			frame++;
		}

	}
	

	delete zed;
	return 0;

}
