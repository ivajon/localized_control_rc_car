#include "DrivabilityDetector.h"


//Constructor definition
DrivabilityDetector::DrivabilityDetector(float prevWeight, Point2i initialPoint, int averageSize) {
	//Centerpoint
	this->prevWeight = prevWeight;
	this->centerPoint = initialPoint;

	//Drivable rows
	this->rowFromBottom = 0;
	this->averageSize = averageSize;
	this->rawRowFromBottom = vector<int>(averageSize, 0);

	//Initializing images
	this->lineImage = Mat(480, 640, CV_8UC1, Scalar(0));
	this->edgeImage = lineImage.clone();
}

//Function definitions


/*
Using Hough Lines transform to determine the drivable parts of the floor.
* Basic process is
* 1 - Blur image with Gaussian filter
* 2 - Find edges using Canny edge detector
* 3 - Find nearly horizontal lines using Hough line transform
* 4 - Get main lines
* 5 - Get center
* 6 - Pass last n values for number of rows from the bottom of the array through moving average filter
* 7 - Return no.rows from bottom
*/
int DrivabilityDetector::calculateRowFromBottom(cv::Mat image) {
	//1 - Blur image with Gaussian filter
	Size2i kernelSize = Size(7, 7);			    //Size of kernel for convolution, larger -> smoother output + slower
	GaussianBlur(image, image, kernelSize, 0); //Stdev calculated to fit Kernel


	//2 - Find edges using Canny edge detector
	Mat edge;
	Canny(image, edge, 25, 80);

	//3 - Find lines
	std::vector<Vec4i> lines;
	double rho = 1.0;    //Pixel resolution
	double theta = 1.0 / 180 * CV_PI; //Angle resolution
	const double maxSlope = 0.25;

	int minVotes = 30; //Mininum number of votes for a line
	HoughLinesP(edge, lines, rho, theta, minVotes, 100, 40);

	//4 - Get main lines
	int width = image.cols;
	int height = image.rows;
	Mat cimage;
	cvtColor(image, cimage, COLOR_GRAY2BGR);
	double leftLength = 0, rightLength = 0, leftdydx, rightdydx;
	int leftIndex = -1, rightIndex = -1;
	for (size_t k = 0; k < lines.size(); k++) {
		int dx = lines[k][2] - lines[k][0];
		int dy = lines[k][3] - lines[k][1];
		if (abs(dx) < 20)
			continue;
		float dydx = (float)dy / dx;
		if (dydx < 0 && dydx > -maxSlope) {
			if (rightLength < abs(dx)) {
				rightLength = abs(dx);
				rightIndex = k;
				rightdydx = dydx;
			}
		}
		else if (dydx > 0 && dydx < maxSlope) {
			if (leftLength < abs(dx)) {
				leftLength = abs(dx);
				leftIndex = k;
				leftdydx = dydx;
			}
			//cout << "Postive slope" << endl;
		}
		else {
			continue;
		}
	}

	//4 calculate center
	int centX, centY;
	if (leftIndex != -1 && rightIndex != -1) {
		int y1 = lines[leftIndex][1];
		int x1 = lines[leftIndex][0];

		int y2 = lines[rightIndex][3];
		int x2 = lines[rightIndex][2];
		#ifndef RACE
		line(cimage, Point(lines[leftIndex][0], lines[leftIndex][1]), Point(lines[leftIndex][2], lines[leftIndex][3]), Scalar(0, 0, 255), 3, LINE_AA);
		line(cimage, Point(lines[rightIndex][0], lines[rightIndex][1]), Point(lines[rightIndex][2], lines[rightIndex][3]), Scalar(0, 0, 255), 3, LINE_AA);
		#endif
		centX = ((float)y2 - (float)y1 + leftdydx * (float)x1 - rightdydx * (float)x2) / (leftdydx - rightdydx);
		centY = leftdydx * (centX - x1) + y1;
	}
	else if (rightIndex != -1) {
		centY = lines[rightIndex][3];
		centX = lines[rightIndex][2];
		#ifndef RACE
		line(cimage, Point(lines[rightIndex][0], lines[rightIndex][1]), Point(lines[rightIndex][2], lines[rightIndex][3]), Scalar(0, 0, 255), 3, LINE_AA);
		#endif
	}
	else if (rightIndex != -1) {
		centY = lines[leftIndex][1];
		centX = lines[leftIndex][0];
		#ifndef RACE
		line(cimage, Point(lines[rightIndex][0], lines[rightIndex][1]), Point(lines[rightIndex][2], lines[rightIndex][3]), Scalar(0, 0, 255), 3, LINE_AA);
		#endif		
	}

#ifndef RACE
	circle(cimage, Point(centX, centY), 5, Scalar(255, 0, 0), 5);
#endif
	


	float prevFactor = this->prevWeight;
	Point2i calcCenter(centX, centY);
	this->centerPoint = this->centerPoint * prevFactor + (1 - prevFactor) * calcCenter;


	//6 - Pass last n values for number of rows from the bottom of the array through moving average filter
	vector<int> vec = this->rawRowFromBottom;
	std::rotate(vec.rbegin(), vec.rbegin() + 1, vec.rend());
	vec[0] = height - centY;
	this->rawRowFromBottom = vec;
	this->rowFromBottom = std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();
	this->lineImage = cimage;
	this->edgeImage = edge;

	//7 - Return no.rows from bottom
	return this->rowFromBottom;
}

