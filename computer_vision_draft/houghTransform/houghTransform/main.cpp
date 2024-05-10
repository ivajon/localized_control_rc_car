#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <fstream>
#include<windows.h>  
#include<chrono>
#include<cmath>
#include "DrivabilityDetector.h"

using namespace cv;
using namespace std;

//Function for rotating 
static Mat rotateImage(Mat image, double angle);

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


//Main loop
int main(int argc, char** argv)
{
	//Vector for storing filenames
	vector<cv::String> fn;
	cv::glob("capture4/*.png", fn, true);
	if (fn.size() == 0) {
		cout << "No files in folder" << endl;
		return -1;
	}

	//Open windows for showing progress
	String windowName = "Drivability map"; 
	namedWindow(windowName); 
	String windowName2 = "Lines";
	namedWindow(windowName2); 

	Point2i centerPoint(100, 100); //Intial guess for center point(in scaled coordinates)
	DrivabilityDetector test(0.8, centerPoint,5); //Create an object of type "DrivabilityDetector", with past weight 0.8, initial guess centerPoint and a 5 point moving average filter
	for (size_t i = 0;i < fn.size();i++)
	{
		auto start = std::chrono::steady_clock::now(); //Start time meas
		Mat image = imread(fn[i]); // Read image
		if (image.empty()) // Check for failure
		{
			cout << "Could not open or find the image" << endl;
			system("pause"); //wait for any key press
			return -1;
		}

		Mat grayImage;
		cvtColor(image, grayImage, COLOR_RGB2GRAY); //GRAYSCALE
		cv::flip(grayImage, grayImage, 0);
		float scale = 0.5;
		resize(grayImage, grayImage, Size(), scale, scale);
		grayImage = rotateImage(grayImage, (double)3); //Only done to correct for the tilt of the camera
		
		auto preprocess = since(start).count();
		std::cout << "Preprocessing complete(ms) =" << preprocess;
		int rowFromBottom = test.calculateRowFromBottom(grayImage);
		auto end = since(start).count() - preprocess;
		std::cout << " Calculations complete(ms) =" << end << std::endl;
		
		cout << "Distance from bottom : " << test.getRowFromBottom() << endl;

		//Display images
		Mat overlayedImage;
		Mat testImage = test.getDrivabilityMap();
		addWeighted(grayImage, 1, test.getDrivabilityMap(), 0.5, 0, overlayedImage);
		cvtColor(overlayedImage, overlayedImage, COLOR_GRAY2RGB); //GRAYSCALE
		circle(overlayedImage,test.getCenterPoint(), 5, Scalar(0, 0, 255), 3);
		imshow(windowName, overlayedImage);     // Show our image inside the created 
		//imshow(windowName, edge);
		imshow(windowName2, test.getLineImage());
		int freq = 5;				  //Not really tho
		cv::waitKey(1000/freq);
	}


	/*END OF FUNCTION*/
	cv::waitKey(10000); // Wait for any keystroke in the window
	cv::destroyWindow(windowName); //destroy the created window

	


	return 0;
}



//Rotates an image by an angle, kinda sketchy and slow, should not be used in final version(align camera properly instead)
Mat rotateImage(Mat image, double angle) {
	Point2f center(static_cast<float>(image.cols / 2), static_cast<float>(image.rows / 2)); //Find center of image
	Mat rotationMatrix = cv::getRotationMatrix2D(center, angle, 1.0 + angle / 10);						//Rotatation matrix around image, scale up slightly to fill image
	Mat rotatedImage;																		// Declare new variable
	warpAffine(image, rotatedImage, rotationMatrix, image.size());
	return rotatedImage;
}