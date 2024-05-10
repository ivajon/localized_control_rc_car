#include <opencv2/opencv.hpp>
#include <string>
#include<windows.h>  
#include <iostream>
#include <fstream>
#include <numeric>
using namespace cv;
using namespace std;

#pragma once

//Class containing parameters for function used for estimating the distance to the wall
class DrivabilityDetector
{
private:
	//Members
	int rowFromBottom;				//Maximum number of rows from bottom of image(after moving average filter)
	vector<int> rawRowFromBottom;	//Vector containing the last n values calculated
	int averageSize;				//Size of moving average filter

	Point2i centerPoint;			//Center point estimate of image
	float prevWeight;				//Weight given to previous centerpoint in calculations
	
	//Images for visualization
	Mat drivabilityMap;			
	Mat lineImage;
public:
	//Constructor
	DrivabilityDetector(float prevWeight, Point2i initialPoint, int averageSize);

	//Method(s)
	int calculateRowFromBottom(cv::Mat grayScaleImage);

	// Get/set functions
	int getRowFromBottom() const { return rowFromBottom; }

	Point2i getCenterPoint() const { return centerPoint; }

	void setPrevWeight(float prevWeight) { this->prevWeight = prevWeight; }
	float getPrevWeight() const { return prevWeight; }

	Mat getDrivabilityMap() const { return drivabilityMap; }

	Mat getLineImage() const { return lineImage; }

	void setAverageSize(int averageSize) { this->averageSize = averageSize; }
	int getAverageSize() const { return averageSize; }
};

