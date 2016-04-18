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


// Input from keyboard
char keyboard = ' ';

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
cv::Mat image(height, width, CV_8UC4,1);
cv::Mat depth(height, width, CV_8UC4,1);

// Create OpenCV windows
cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);
namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

//Blue
int iLowH = 89;
int iHighH = 125;

int iLowS = 64;
int iHighS = 255;

int iLowV = 128;
int iHighV = 255;

////Create trackbars in "Control" window
cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
cvCreateTrackbar("HighH", "Control", &iHighH, 179);

cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
cvCreateTrackbar("HighS", "Control", &iHighS, 255);

cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
cvCreateTrackbar("HighV", "Control", &iHighV, 255);

// Settings for windows
cv::Size displaySize(720, 404);
cv::Mat imageDisplay(displaySize, CV_8UC4);
cv::Mat depthDisplay(displaySize, CV_8UC4);

// Loop until 'q' is pressed
while (keyboard != 'q') {

 // Grab frame and compute depth in FULL sensing mode
 if (!zed->grab(sl::zed::SENSING_MODE::FULL))
 {

 // Retrieve left color image
 sl::zed::Mat left = zed->retrieveImage(sl::zed::SIDE::LEFT);
 memcpy(image.data,left.data,width*height*4*sizeof(uchar));

// Retrieve depth map
 sl::zed::Mat depthmap = zed->normalizeMeasure(sl::zed::MEASURE::DEPTH);
 memcpy(depth.data,depthmap.data,width*height*4*sizeof(uchar));



 Mat imgHSV;

cvtColor(image, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

Mat imgThresholded;

inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

//morphological opening (remove small objects from the foreground)
erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

//morphological closing (fill small holes in the foreground)
dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

imshow("Thresholded Image", imgThresholded); //show the thresholded image
//imshow("Original", image); //show the original image


 // Display image in OpenCV window
 cv::resize(image, imageDisplay, displaySize);
 cv::imshow("Image", imageDisplay);

 // Display depth map in OpenCV window
 cv::resize(depth, depthDisplay, displaySize);
 cv::imshow("Depth", depthDisplay);
 }

keyboard = cv::waitKey(30);

}

delete zed;

}
