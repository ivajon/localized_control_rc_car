#include <opencv2/opencv.hpp>
#include <mutex>

using namespace cv;
using namespace std;
#pragma once

extern std::mutex mutex_var;
extern Mat grayImage;
extern Mat hsvImage;
extern int frameID;
extern atomic<bool> stop;

class Camera_Preprocessor
{
private:
	float scaleFactor;
	cv::VideoCapture camera;
	std::mutex* mutex_var;
	Mat grayTemp;


public:
	/**
	* Constructor for Camera_Preprocessor
	*
	* @param scaleFactor  scaling factor for image
	* @param camera	 	  Camera used for capturing images
	* @param mutex_var    pointer tothe mutex variable for the images
	*/
	Camera_Preprocessor(float scaleFactor, cv::VideoCapture camera, mutex* mutex_var);
	Mat getGrayTemp() { return grayTemp; }


	/**
	* Reads images from connected camera, processes them and then
	* saves them in specified locations.
	*
	* @param hsvImage  pointer to output image for hsv image
	* @param grayImage pointer to output image for grayscale image
	*/
	void readCameraFeed();

	/// Read camera feed in a new thread
	std::thread startThread() {
		return std::thread([this] { this->readCameraFeed(); });
	}
};

