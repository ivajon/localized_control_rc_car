#include <opencv2/opencv.hpp>
#include <mutex>

using namespace cv;
using namespace std;
#pragma once
class Camera_Preprocessor
{
private:
	float scaleFactor;
	cv::VideoCapture camera;
	int fileid;
	std::mutex *mutex_var;
public:
	/**
	* Constructor for Camera_Preprocessor
	* 
	* @param scaleFactor  scaling factor for image
	* @param camera	 	  Camera used for capturing images
	* @param mutex_var    pointer tothe mutex variable for the images
	*/
	Camera_Preprocessor(float scaleFactor, cv::VideoCapture camera);

	/**
	* Reads images from connected camera, processes them and then
	* saves them in specified locations.
	* 
	* @param hsvImage  pointer to output image for hsv image
	* @param grayImage pointer to output image for grayscale image
	*/
	void readCameraFeed();

};

