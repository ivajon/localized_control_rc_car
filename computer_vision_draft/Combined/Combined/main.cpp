#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include "DrivabilityDetector.h"
#include "Camera_Preprocessor.h"
#include "StartStop.h"
#include <mutex>
#include <thread> 
#include <atomic>

#define RACE
// UNCOMMENT TO RUN ON LINUX, with tcp
#define TCP

static const char FAST = 100;
static const char MEDIUM = 50;
static const char SLOW = 30;
#ifdef TCP {
	#include "TCP.h"
}
#endif
using namespace cv;
using namespace std;

std::mutex mutex_var;
std::mutex stopMutex;
Mat grayImage;
Mat hsvImage;
int frameID;
bool stop = true;
bool initialized = false;


void main_loop(Camera_Preprocessor camReader, DrivabilityDetector test) {
	cout << "MAIN LOPTILOOP" << endl;
	Mat grayTemp;
	bool running = false;
	int lastFrameID = 0;
	while (true) { 
		stopMutex.lock();
		if (stop) {
			if (running) {
				// SEND STOP
				#ifdef TCP
				TCPclient(0, 0);
				#endif // 
			}
			running = false;
		}
		while (stop) {
			std::this_thread::sleep_for(50ms);
		}
		if (!running) {
			// SEND START
			#ifdef TCP
			TCPclient(1, 0);
			#endif // TCP
		}
		running = true;
		stopMutex.unlock();

		mutex_var.lock();
		if (grayImage.empty() || frameID == lastFrameID) {
			mutex_var.unlock();
			std::this_thread::sleep_for(15ms);
			continue;
		}
		lastFrameID = frameID;
		grayTemp = grayImage.clone();
		mutex_var.unlock();

		//Call function
		int rowFromBottom = test.calculateRowFromBottom(grayTemp);


		
#ifndef RACE
		{
			namedWindow("Test");
			cout << "Distance from bottom : " << test.getRowFromBottom() << endl;

		
			imshow("Centerimage", test.getLineImage());
			imshow("EdgeImages", test.getEdgeImage());

			if (waitKey(2) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
			{
				cout << "esc key is pressed by user" << endl;
				break;
			}
		}
#endif

		// SEND SET SPEED
		const int lower = 70;
		const float upper = 150;

		float speed = upper / (rowFromBottom - lower);

		if (speed > 0.9) {
#ifdef TCP
			TCPclient(2, FAST);
#endif // TCP
			//Go fast
		}
		else if (speed > 0.5) {
			//Go medium
#ifdef TCP
			TCPclient(2, MEDIUM);
#endif // TCP
		}
		else {
			//Go slow
#ifdef TCP
			TCPclient(2, SLOW);
#endif // TCP
		}

	}
}

//Main loop
int main(int argc, char** argv)
{
	VideoCapture cap(0);

	if (cap.isOpened() == false)
	{
		cout << "Cannot open the video camera" << endl;
		cin.get(); //wait for any key press
		return -1;
	}

	cap.set(cv::CAP_PROP_FRAME_WIDTH, 640); // valueX = your wanted width
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480); // valueY = your wanted heigth
	Camera_Preprocessor camReader(0.5, cap, &mutex_var);
	Point2i centerPoint(100, 100); //Intial guess for center point(in scaled coordinates)
	DrivabilityDetector test(0.8, centerPoint, 5); //Create an object of type "DrivabilityDetector", with past weight 0.8, initial guess centerPoint and a 5 point moving average filter
	StartStop startStopDetector(&mutex_var);
	// SPAWN THE THREADS
	thread camera_handle = camReader.startThread();

	thread circle_thread = startStopDetector.startThread();
	cout << "HEYA" << endl;
	std::thread looptiloop(main_loop, camReader, test);

	looptiloop.join();
	camera_handle.join();
	circle_thread.join();

	return 0;
}



