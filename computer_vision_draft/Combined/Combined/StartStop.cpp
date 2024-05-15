#include "StartStop.h"

#define RACE
StartStop::StartStop(std::mutex *mutex_var)
{
	colorLimitsHSV redLimsTemp, greenLims;
	redLimsTemp.iLowH = 0;
	redLimsTemp.iHighH = 10;
	redLimsTemp.iLowS = 70;
	redLimsTemp.iHighS = 255;
	redLimsTemp.iLowV = 50;
	redLimsTemp.iHighV = 255;

	this->redLims = redLimsTemp;
	this->greenLims = redLimsTemp;

	this->mutex_var = mutex_var;
}

int StartStop::start()
{
	cout << "Circle detector started" << endl;
	while (true)
	{
		if (stop)
		{
			this->green();
		}
		else
		{
			this->red();
		}
	}
}

int StartStop::green()
{
	int iLowH = 20;
	int iHighH = 90;

	int iLowS = 30;
	int iHighS = 255;

	int iLowV = 50;
	int iHighV = 255; // Sets the hue values to only detect green

	int i = startStop(iLowH, iHighH, iLowS, iHighS, iLowV, iHighV, false);
	if (i == 1)
	{
		cout << "Found green" << endl;
		stop = false;
	}
	return -1;
}

int StartStop::red()
{
	int iLowH = 0;
	int iHighH = 5;

	int iLowS = 70;
	int iHighS = 255;

	int iLowV = 50;
	int iHighV = 255; // Sets the hue values to only detect red

	int i = startStop(iLowH, iHighH, iLowS, iHighS, iLowV, iHighV, true);
	if (i == 1)
	{
		// TCPclient(1, 0)
		cout << "Found red" << endl;
		stop = true;
	}
	return -1;
}

int StartStop::startStop(int LowH, int HighH, int LowS, int HighS, int LowV, int HighV, bool isRed)
{
	int lastFrameID = -1;
	while (true)
	{
		Mat imgHSV, imgThreshold, imgLap, abs_Lap; // array to store picture data

		mutex_var->lock();
		if (hsvImage.empty() || frameID == lastFrameID)
		{
			mutex_var->unlock();
			std::this_thread::sleep_for(100ms);
			cout << "WAITING FOR IMAGE(CIRCLE)" << endl;
			continue;
		}
		imgHSV = hsvImage.clone();
		lastFrameID = frameID;
		mutex_var->unlock();

		if (isRed == true)
		{
			Mat1b mask1, mask2;
			inRange(imgHSV, Scalar(LowH, LowS, LowV), Scalar(HighH, HighS, HighV), mask1); // Filter out colors
			inRange(imgHSV, Scalar(165, 70, 50), Scalar(180, 255, 255), mask2);
			Mat1b mask = mask1 | mask2;
			imgThreshold = mask;
		}
		else
		{
			inRange(imgHSV, Scalar(LowH, LowS, LowV), Scalar(HighH, HighS, HighV), imgThreshold); // Filter out colors
		}

		GaussianBlur(imgThreshold, imgThreshold, Size(9, 9), 2, 2); // Doesn't work without
		Laplacian(imgThreshold, imgLap, CV_16S, 3, 1, 0, BORDER_DEFAULT);
		convertScaleAbs(imgLap, abs_Lap);

		vector<Vec3f> circles;
		HoughCircles(abs_Lap, circles, HOUGH_GRADIENT_ALT, 2, abs_Lap.rows, 300, 0.95, 20, 0); // detect the circle
		if (!circles.empty())
		{
			return 1;
		}

#ifndef RACE
		imshow("image", imgThreshold);
		imshow("Lap", abs_Lap);
		waitKey(10);
#endif
	}
}
