#include "Camera_Preprocessor.h"

Camera_Preprocessor::Camera_Preprocessor(float scaleFactor, VideoCapture camera) {
	this->scaleFactor = scaleFactor;
	this->camera = camera;
}

void Camera_Preprocessor::readCameraFeed() {
	Mat img;
	bool keepGoing = true;
	while (keepGoing) {
		bool readImage = this->camera.read(img);
		if(readImage){
			continue;
		}
		resize(img, img, Size(), this->scaleFactor, this->scaleFactor);
	}
}

