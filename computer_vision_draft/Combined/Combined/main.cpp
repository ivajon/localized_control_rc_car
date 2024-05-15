#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <windows.h>  
#include "DrivabilityDetector.h"
#include "Camera_Preprocessor.h"
#include "StartStop.h"
#include "TCP.h"
#include <mutex>
#include <thread> 

//#define RACE

using namespace cv;
using namespace std;

std::mutex mutex_var;
std::condition_variable cp;
Mat grayImage;
Mat hsvImage;
int frameID;
bool stop = false;
bool initialized = false;



void main_loop(Camera_Preprocessor camReader, DrivabilityDetector test) {
	cout << "MAIN LOPTILOOP" << endl;
	while (true) {
		Mat grayTemp;


		mutex_var.lock();
		if (grayImage.empty()) {
			mutex_var.unlock();

			std::this_thread::sleep_for(100ms);
			cout << "WAITING FOR IMAGE" << endl;
			continue;
		}
		grayTemp = grayImage.clone();


		mutex_var.unlock();

		int rowFromBottom = test.calculateRowFromBottom(grayTemp);

		namedWindow("Test");
#ifndef RACE
		{
			cout << "Distance from bottom : " << test.getRowFromBottom() << endl;

			//Display images
			Mat overlayedImage;
			Mat testImage = test.getDrivabilityMap();
			cout << testImage.size() << " gray: " << grayTemp.size()<<endl;
			addWeighted(grayTemp, 1, testImage, 0.5, 0, overlayedImage);
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
}

//Main loop
int main(int argc, char** argv)
{
	VideoCapture cap(0);

	if (cap.isOpened() == false)
	{
		cout << "Cannot open the video camera" << endl;
		cin.get(); //wait for any key press
		return -1;
	}

	cap.set(cv::CAP_PROP_FRAME_WIDTH, 640); // valueX = your wanted width
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480); // valueY = your wanted heigth
	Camera_Preprocessor camReader(0.5, cap, &mutex_var);
	Point2i centerPoint(100, 100); //Intial guess for center point(in scaled coordinates)
	DrivabilityDetector test(0.8, centerPoint, 5); //Create an object of type "DrivabilityDetector", with past weight 0.8, initial guess centerPoint and a 5 point moving average filter

	// SPAWN THE THREADS
	thread camera_handle = camReader.startThread();

	//StartStop start();
	//TCP tcp();
	cout << "HEYA" << endl;
	std::thread looptiloop(main_loop, camReader, test);


	looptiloop.join();
	camera_handle.join();

	return 0;
}



