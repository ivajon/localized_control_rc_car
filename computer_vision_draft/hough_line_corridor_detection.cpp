#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <fstream>
#include<windows.h>  

using namespace cv;
using namespace std;

static Mat rotateImage(Mat image, double angle);


int main(int argc, char** argv)
{
	// Read the image file
	vector<cv::String> fn;
	cv::glob("capture2/*.png", fn, true);

	vector<Mat> images;
	size_t count = fn.size(); //number of png files in images folder
	for (size_t i = 0; i < count; i++) {
		cv::Mat image = imread(fn[i]);
		if (image.empty()) // Check for failure
		{
			cout << "Could not open or find the image" << endl;
			system("pause"); //wait for any key press
			return -1;
		}
		images.push_back(image);
	}
	
	if (images.empty()) // Check for failure
	{
		cout << "Could not open or find the image" << endl;
		system("pause"); //wait for any key press
		return -1;
	}


	String windowName = "My HelloWorld Window"; //Name of the window
	namedWindow(windowName); // Create a window

	/*START OF FUNCTION, I'm just to lazy to rewrite it*/
	Point2i centerPoint(100, 100); //Intial guess, it is
	for (size_t i = 0;i < count;i++)
	{
		Mat image = images[i];
		cvtColor(image, image, COLOR_RGB2GRAY); //GRAYSCALE
		float scale = 0.5;
		resize(image, image, Size(), scale, scale);
		image = rotateImage(image, (double)5); //Only done to correct for the tilt of the camera
		Size2i kernelSize = Size(7, 7);			    //Size of kernel for convolution, larger -> smoother output + slower
		GaussianBlur(image, image, kernelSize, 0); //Stdev calculated to fit Kernel

		//Edge detection
		Mat edge;
		Canny(image, edge, 25, 80);

		//Detect lines
		vector<Vec2f> lines;
		double rho = 0.5;    //Pixel resolution
		double theta = 0.5 / 180 * CV_PI; //Angle resolution
		double minAngle = 85.0 / 180 * CV_PI;

		double maxAngle = 105.0 / 180 * CV_PI;

		int minVotes = 10; //Mininum number of votes for a line
		HoughLines(edge, lines, rho, theta, minVotes, 0, 0, minAngle, maxAngle);



		//Print lines in empty matrix
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

		//Get furthest row, that is the minimum row, also create drivability map
		int minRow = height - 1;
		int lastRow = height-1;
		Mat drivabilityMap(height, width, CV_8UC1, Scalar(0));

		//Get center point
		int centCol = 0;
		int nonZero = 0;

		for (size_t col = 0; col < width;col++) {
			for (size_t row = height - 1; row > 0;row--) {
				drivabilityMap.at<uchar>(row, col) = 255; //Only for displaying image, not necessary for algorithm
				if (emptyMatrix.at<uchar>(row, col) != 0){
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


		float prevFactor = 0.9;
		Point2i calcCenter(centCol, (float)minRow);
		centerPoint = centerPoint * prevFactor + (1 - prevFactor) * calcCenter;

		int debugPrint = 1;
		switch (debugPrint) {
			case 1 :
				cout << "Minimum row: " << minRow;
				cout << ", Center column: " << centCol << endl;
				cout << "Center point: " << centerPoint << endl;
				break;
			case 2:
				cout << "Rows from bottom:" << height-centerPoint.y << endl;
				break;
			default:
				break;
		}

		//Display images
		Mat overlayedImage;
		addWeighted(image, 1, drivabilityMap, 0.5, 0, overlayedImage);
		cvtColor(overlayedImage, overlayedImage, COLOR_GRAY2RGB); //GRAYSCALE
		circle(overlayedImage, centerPoint, 5, Scalar(0, 0, 255), 3);
		imshow(windowName, overlayedImage);     // Show our image inside the created 
		//imshow(windowName, edge);
		//imshow(windowName, emptyMatrix);
		int freq = 5;				  //Not really tho
		cv::waitKey(1000/freq);
	}


	/*END OF FUNCTION*/
	cv::waitKey(10000); // Wait for any keystroke in the window
	cv::destroyWindow(windowName); //destroy the created window

	


	return 0;
}

static Mat rotateImage(Mat image, double angle) {
	Point2f center(static_cast<float>(image.cols / 2), static_cast<float>(image.rows / 2)); //Find center of image
	Mat rotationMatrix = cv::getRotationMatrix2D(center, angle, 1.0+angle/10);						//Rotatation matrix around image, scale up slightly to fill image
	Mat rotatedImage;																		// Declare new variable
	warpAffine(image,rotatedImage, rotationMatrix, image.size());
	return rotatedImage;
}