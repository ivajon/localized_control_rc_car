#include "Camera_Preprocessor.h"

Camera_Preprocessor::Camera_Preprocessor(float scaleFactor, VideoCapture camera, mutex* mutex_var) {
	this->scaleFactor = scaleFactor;
	this->camera = camera;
	this->mutex_var = mutex_var;

}

void Camera_Preprocessor::readCameraFeed() {
	Mat img;
	bool keepGoing = true;
	frameID = 0;

	while (keepGoing) {
		bool readImage = this->camera.read(img);


		if (!readImage) {

			cout << "2222222" << endl;

			continue;
		}
		resize(img, img, Size(), this->scaleFactor, this->scaleFactor);
		Mat grayTemp, hsvTemp;
		cvtColor(img, grayTemp, COLOR_RGB2GRAY);
		cvtColor(img, hsvTemp, COLOR_RGB2HSV);
		this->grayTemp = grayTemp.clone();

		mutex_var->lock();
		keepGoing = !stop;
		grayImage = grayTemp.clone();
		hsvImage = hsvTemp.clone();
		//grayTemp.copyTo(grayImage);
		//hsvTemp.copyTo(hsvImage);
		frameID++;
		mutex_var->unlock();
	}
}

