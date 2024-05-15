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
	this->drivabilityMap = Mat(480, 640, CV_8UC1, Scalar(0));
	this->drivabilityMap.copyTo(this->lineImage);
}

//Function definitions


/*
Using Hough Lines transform to determine the drivable parts of the floor.
* Basic process is
* 1 - Blur image with Gaussian filter
* 2 - Find edges using Canny edge detector
* 3 - Find nearly horizontal lines using Hough line transform
* 4 - Print lines onto empty array
* 5 - Search through array until line is found, find row furthest from bottom of image
* 6 - Calculate center row, take as centerpoint(+ weighted average with last value)
* 7 - Pass last n values for number of rows from the bottom of the array through moving average filter
* 8 - Return no. rows from bottom
*/
int DrivabilityDetector::calculateRowFromBottom(cv::Mat image) {
	//1 - Blur image with Gaussian filter
	Size2i kernelSize = Size(7, 7);			    //Size of kernel for convolution, larger -> smoother output + slower
	GaussianBlur(image, image, kernelSize, 0); //Stdev calculated to fit Kernel


	//2 - Find edges using Canny edge detector
	Mat edge;
	Canny(image, edge, 25, 80);

	//3 - Find nearly horizontal lines using Hough line transform
	std::vector<Vec2f> lines;
	double rho = 1.0;    //Pixel resolution
	double theta = 1.0 / 180 * CV_PI; //Angle resolution
	double minAngle = 85.0 / 180 * CV_PI;
	double maxAngle = 105.0 / 180 * CV_PI;

	int minVotes = 36; //Mininum number of votes for a line
	HoughLines(edge, lines, rho, theta, minVotes, 0, 0, minAngle, maxAngle);

	//4 - Print lines onto empty array
	int width = image.cols;
	int height = image.rows;
	Mat emptyMatrix(height, width, CV_8UC1, Scalar(0)); //Initialize empty matrix, 8-bit, 1-color channel, 
	for (size_t k = 0; k < lines.size(); k++) {
		float rho = lines[k][0];
		float theta = lines[k][1];
		if (((theta > 89.5 / 180 * CV_PI) && (theta < 90.5 / 180 * CV_PI)))
			continue;
		double a = cos(theta), b = sin(theta);
		double x0 = a * rho, y0 = b * rho; //Find centerpoint in x,y coordinates
		int pixelWidth = 1000;
		Point pt1(cvRound(x0 + pixelWidth * (-b)), cvRound(y0 + pixelWidth * (a))); //FInd endpoiunts of line
		Point pt2(cvRound(x0 - pixelWidth * (-b)), cvRound(y0 - pixelWidth * (a)));
		line(emptyMatrix, pt1, pt2, Scalar(255, 255, 255), 1, LINE_AA);

	}

	//5 - Search through array until line is found, find row furthest from bottom of image
	//6 - Calculate center row, take as centerpoint(+weighted average with last value)
	int minRow = height - 1;
	int lastRow = height - 1;

	Mat drivabilityMap(height, width, CV_8UC1, Scalar(0));

	//cout << "driveMap " << drivabilityMap << endl;
	//Get center point
	int centCol = 0;
	int nonZero = 0;
	for (size_t col = 0; col < width;col++) {
		for (size_t row = height - 1; row > 0;row--) {
			drivabilityMap.at<uchar>(row, col) = 255;
			if (emptyMatrix.at<uchar>(row, col) != 0) {
				lastRow = row;
				break;
			}
		}
		if (minRow > lastRow) { //If found larger row, change minimum row and start finding center column of new row
			minRow = lastRow;
			centCol = col;
			nonZero = 1;
		}
		else if (minRow == lastRow) { //If still on same row, add next column for finding center
			centCol += col;
			nonZero++;
		}
	}
	centCol = centCol / nonZero; //Calculate average(integer rounding is fine)
	float prevFactor = this->prevWeight;
	Point2i calcCenter(centCol, (float)minRow);
	this->centerPoint = this->centerPoint * prevFactor + (1 - prevFactor) * calcCenter;


	//7 - Pass last n values for number of rows from the bottom of the array through moving average filter
	vector<int> vec = this->rawRowFromBottom;
	std::rotate(vec.rbegin(), vec.rbegin() + 1, vec.rend());
	vec[0] = height - minRow;
	this->rawRowFromBottom = vec;
	this->rowFromBottom = std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();

	this->lineImage = emptyMatrix;
	this->drivabilityMap = drivabilityMap;

	//8 - Return no.rows from bottom
	return this->rowFromBottom;
}

