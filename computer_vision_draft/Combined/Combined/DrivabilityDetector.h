#pragma once
#include <opencv2/opencv.hpp>
#include <string>
//#include <windows.h>  
#include <iostream>
#include <fstream>
#include <numeric>
using namespace cv;
using namespace std;


/**
* Class containing parameters for function used for estimating the distance to the wall
*
*/
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
	Mat lineImage;
	Mat edgeImage;
public:
	/**
	* Constructor for drivability detector
	*
	* @param prevWeight   The weight given to the previous centerpoint when averaging
	* @param initialPoint The initial guess for the center point
	* @param averageSize  The number of measurements used for calculating the average number of rows
	*/
	DrivabilityDetector(float prevWeight, Point2i initialPoint, int averageSize);

	/**
	* Function for calculating the drivable area in a corridor.
	*
	* It calculates the furthest row from the bottom of the image that doesn't
	* contain a wall. The output is the moving average of the last 5 measurements.
	*
	* @param grayScaleImage Grayscale image to run algorithm on
	*
	* @returns rowFromBottom The furthest row from the bottom which was determined to belong to the floor
	*/
	int calculateRowFromBottom(cv::Mat grayScaleImage);

	/// Gets the current estimated number of rows from the bottom
	int getRowFromBottom() const { return rowFromBottom; }

	/// Gets the current center point estimate
	Point2i getCenterPoint() const { return centerPoint; }

	/// Sets the current weighting for the last center point estimate
	void setPrevWeight(float prevWeight) { this->prevWeight = prevWeight; }
	/// Gets the current weighting for the last center point estimate
	float getPrevWeight() const { return prevWeight; }

	/// Gets the last calculated drivability map, a mask containing the drivable parts of the image
	Mat getDrivabilityMap() const { return drivabilityMap; }

	/// Gets an image containing the lines detected in the last image passed through the algorithm
	Mat getLineImage() const { return lineImage; }

	// Gets the size of the moving average filter
	int getAverageSize() const { return averageSize; }
};

