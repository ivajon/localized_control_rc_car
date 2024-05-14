
#include <opencv2/opencv.hpp>

#include <winsock2.h>
#include <sys/types.h>
//#include <netinet/in.h> 
//#include <sys/socket.h> 
//#include <unistd.h> 

using namespace cv;
using namespace std;

static int green();
static int red();
static int startStop(int LowH, int HighH, int LowS, int HighS, int LowV, int HighV, bool isRed);
int TCPclient(char first, char second);

int main(int argc, char** argv) {

	green();
}

static int green() {
	int iLowH = 56;
	int iHighH = 88;

	int iLowS = 75;
	int iHighS = 255;


	int iLowV = 78;
	int iHighV = 255;	//Sets the hue values to only detect green

	int i = startStop(iLowH, iHighH, iLowS, iHighS, iLowV, iHighV, false); //start the loop with the Hue values
	if (i == 1) {
		cout << "Start" << endl;	//replace with start engine
		TCPclient(0, 0);
		return red();	//if successfull start search for red
	}
}

static int red() {
	int iLowH = 0;
	int iHighH = 10;

	int iLowS = 70;
	int iHighS = 255;

	int iLowV = 50;
	int iHighV = 255;	//Sets the hue values to only detect red

	int i = startStop(iLowH, iHighH, iLowS, iHighS, iLowV, iHighV, true);
	if (i == 1) {
		cout << "Stop" << endl;	//replace with stop the engine
		TCPclient(1, 0);
	}
	return -1;
}

static int startStop(int LowH, int HighH, int LowS, int HighS, int LowV, int HighV, bool isRed) {
	VideoCapture cap(0);

	while (true) {
		Mat imgOriginal, imgHSV, imgThreshold, imgCanny; //array to store picture data

		bool bSuccess = cap.read(imgOriginal); // read a new frame from video

		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);	//color converter

		if (isRed == true) {
			Mat1b mask1, mask2;
			inRange(imgHSV, Scalar(LowH, LowS, LowV), Scalar(HighH, HighS, HighV), mask1); //Filter out colors
			inRange(imgHSV, Scalar(170, 70, 50), Scalar(180, 255, 255), mask2);
			Mat1b mask = mask1 | mask2;
			imgThreshold = mask;
			cout << "red" << endl;
		}
		else if (isRed == false) {
			inRange(imgHSV, Scalar(LowH, LowS, LowV), Scalar(HighH, HighS, HighV), imgThreshold); //Filter out colors
			cout << "green" << endl;
		}

		GaussianBlur(imgThreshold, imgThreshold, Size(9, 9), 2, 2);	//Doesn't work without

		Mat imgLap, abs_Lap;

		Laplacian(imgThreshold, imgLap, CV_16S, 3, 1, 0, BORDER_DEFAULT);
		convertScaleAbs(imgLap, abs_Lap);

		vector<Vec3f> circles;
		HoughCircles(abs_Lap, circles, HOUGH_GRADIENT, 2, abs_Lap.rows, 250, 350, 100, 0); //detect the circle

		if (!circles.empty()) {
			return 1;
		}

		imshow("image", imgThreshold);
		imshow("Lap", abs_Lap);
		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			cout << "esc key is pressed by user" << endl;
			break;
		}
	}

}

int TCPclient(char first, char second) {
	int clientSocket = socket(AF_INET, SOCK_STREAM, 0);

	sockaddr_in serverAddress;
	serverAddress.sin_family = AF_INET;
	serverAddress.sin_port = htons(8080);
	serverAddress.sin_addr.s_addr = INADDR_ANY;

	// sending connection request 
	connect(clientSocket, (struct sockaddr*)&serverAddress,
		sizeof(serverAddress));

	// sending data 
	const char message[2] = { first, second };
	send(clientSocket, message, sizeof(message), 0);
	cout << "sent" << endl;

	return 0;

}