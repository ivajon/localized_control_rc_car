#include <opencv2/opencv.hpp>
#include <string>
#include<windows.h>  
#include <iostream>
#include <fstream>
#include <numeric>
using namespace cv;
using namespace std;


#pragma once
class DrivabilityDetector
{
private:
	//Members
	int rowFromBottom;
	vector<int> rawRowFromBottom;
	float prevWeight;
	Point2i centerPoint;
	Mat drivabilityMap;
	Mat lineImage;
public:
	//Constructor
	DrivabilityDetector(float prevWeight, Point2i initialPoint, int averageSize);

	//Methods
	int calculateRowFromBottom(cv::Mat grayScaleImage);

	// Get/set functions
	int getRowFromBottom() const { return rowFromBottom; }

	Point2i getCenterPoint() const { return centerPoint; }

	void setPrevWeight(float prevWeight) { this->prevWeight = prevWeight; }
	float getPrevWeight() const { return prevWeight; }

	Mat getDrivabilityMap() const { return drivabilityMap; }

	Mat getLineImage() const { return lineImage; }
};

