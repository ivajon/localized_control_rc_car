#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <windows.h>  
#include "DrivabilityDetector.h"
#include "Camera_Preprocessor.h"
#include <mutex>
#include <thread> 

//#define RACE

using namespace cv;
using namespace std;

std::mutex mutex_var;
Mat grayImage;
Mat hsvImage;
int frameID;
bool stop = false;



//Main loop
int main(int argc, char** argv)
{
	VideoCapture cap(0);
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 640); // valueX = your wanted width
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480); // valueY = your wanted heigth
	Point2i centerPoint(100, 100); //Intial guess for center point(in scaled coordinates)
	DrivabilityDetector test(0.8, centerPoint, 5); //Create an object of type "DrivabilityDetector", with past weight 0.8, initial guess centerPoint and a 5 point moving average filter
	Camera_Preprocessor camReader(0.5, cap);
	std::thread camThread = camReader.startThread();


	while (true) {
		Mat grayTemp;
		mutex_var.lock();
		grayImage.copyTo(grayTemp);
		mutex_var.unlock();

		int rowFromBottom = test.calculateRowFromBottom(grayImage);

		namedWindow("Test");
		#ifndef RACE
		{
			cout << "Distance from bottom : " << test.getRowFromBottom() << endl;
		
			//Display images
			Mat overlayedImage;
			Mat testImage = test.getDrivabilityMap();
			addWeighted(grayTemp, 1, test.getDrivabilityMap(), 0.5, 0, overlayedImage);
			cvtColor(overlayedImage, overlayedImage, COLOR_GRAY2RGB); //GRAYSCALE
			circle(overlayedImage, test.getCenterPoint(), 5, Scalar(0, 0, 255), 3);
			imshow("Test", overlayedImage);     // Show our image inside the created 

		}
		#endif

		if (waitKey(2) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			cout << "esc key is pressed by user" << endl;
			break;
		}
		
	}
	return 0;
}



