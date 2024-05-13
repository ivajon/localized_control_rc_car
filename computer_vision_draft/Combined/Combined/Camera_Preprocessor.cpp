#include "Camera_Preprocessor.h"

Camera_Preprocessor::Camera_Preprocessor(float scaleFactor, VideoCapture camera) {
	this->scaleFactor = scaleFactor;
	this->camera = camera;
}

void Camera_Preprocessor::readCameraFeed() {
	Mat img;
	bool keepGoing = true;
	frameID = 0;
	while (keepGoing) {
		bool readImage = this->camera.read(img);
		if(readImage){
			continue;
		}
		resize(img, img, Size(), this->scaleFactor, this->scaleFactor);
		Mat grayTemp, hsvTemp;
		cvtColor(img, grayTemp, COLOR_RGB2GRAY);
		cvtColor(img, hsvTemp, COLOR_RGB2HSV);

		mutex_var->lock();
		keepGoing = !stop;
		grayTemp.copyTo(grayImage);
		hsvTemp.copyTo(hsvImage);
		frameID++;
		mutex_var->unlock();
	}
}

