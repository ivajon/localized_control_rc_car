#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <fstream>
#include <windows.h>  
#include <chrono>
#include <cmath>
#include "DrivabilityDetector.h"

//#define RACE

using namespace cv;
using namespace std;


//For measuring elapsed time
template <
	class result_t = std::chrono::milliseconds,
	class clock_t = std::chrono::steady_clock,
	class duration_t = std::chrono::milliseconds
>
auto since(std::chrono::time_point<clock_t, duration_t> const& start)
{
	return std::chrono::duration_cast<result_t>(clock_t::now() - start);
}


struct {             // Structure declaration
	int myNum;         // Member (int variable)
	string myString;   // Member (string variable)
} myStructure;

//Main loop
int main(int argc, char** argv)
{
	VideoCapture cap(0);
	Point2i centerPoint(100, 100); //Intial guess for center point(in scaled coordinates)
	DrivabilityDetector test(0.8, centerPoint, 5); //Create an object of type "DrivabilityDetector", with past weight 0.8, initial guess centerPoint and a 5 point moving average filter

	while (true) {
	Mat image;
		bool read_success = cap.read(image); // Read image
		if (!read_success) // Check for failure
		{
			cout << "Could not open or find the image" << endl;
			system("pause"); //wait for any key press
			return -1;
		}

		//Process image
		Mat grayImage;
		cvtColor(image, grayImage, COLOR_RGB2GRAY);			//GRAYSCALE
		//cv::flip(grayImage, grayImage, 0);					//Flip image if camera was upside down
		float scale = 0.5;									//Scale down image
		resize(grayImage, grayImage, Size(), scale, scale); // Resize

		int rowFromBottom = test.calculateRowFromBottom(grayImage);

		namedWindow("Test");
		#ifndef RACE
		{
			cout << "Distance from bottom : " << test.getRowFromBottom() << endl;
		
			//Display images
			Mat overlayedImage;
			Mat testImage = test.getDrivabilityMap();
			addWeighted(grayImage, 1, test.getDrivabilityMap(), 0.5, 0, overlayedImage);
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



