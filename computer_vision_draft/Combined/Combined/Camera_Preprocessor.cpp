#include "Camera_Preprocessor.h"

Camera_Preprocessor::Camera_Preprocessor(float scaleFactor, VideoCapture camera, mutex* mutex_var) {
	this->scaleFactor = scaleFactor;
	this->camera = camera;
	this->mutex_var = mutex_var;

}

void Camera_Preprocessor::readCameraFeed() {
	Mat img;
	bool keepGoing = true;
	frameID = 1;

	while (keepGoing) {
		bool readImage = this->camera.read(img);


		if (!readImage) {

			cout << "Could not read image" << endl;

			continue;
		}
		resize(img, img, Size(), this->scaleFactor, this->scaleFactor);
		flip(img, img, 0);
		Mat grayTemp, hsvTemp;
		cvtColor(img, grayTemp, COLOR_BGR2GRAY);
		cvtColor(img, hsvTemp, COLOR_BGR2HSV);
		this->grayTemp = grayTemp.clone();

		mutex_var->lock();
		grayImage = grayTemp.clone();
		hsvImage = hsvTemp.clone();
		frameID++;
		mutex_var->unlock();
	}
}

