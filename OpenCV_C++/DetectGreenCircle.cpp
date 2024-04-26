#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    //Open the default video camera
    VideoCapture cap(0);

    // if not success, exit program
    if (cap.isOpened() == false)
    {
        cout << "Cannot open the video camera" << endl;
        cin.get(); //wait for any key press
        return -1;
    }

    namedWindow("window_name"); //create a window called "My Camera Feed"

    int iLowH = 40;
    int iHighH = 80;

    int iLowS = 40;
    int iHighS = 255;

    int iLowV = 40;
    int iHighV = 255;


    // Inside the main loop
    while (true)
    {
        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video

        if (!bSuccess) //if not success, break loop
        {
            cout << "Cannot read a frame from video stream" << endl;
            break;
        }

        // Convert image to HSV
        Mat imgHSV;
        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

        // Threshold the image to detect red color
        Mat imgThresholded;
        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);

        // Apply morphological operations if needed

        // Detect circles using Hough Transform
        vector<Vec3f> circles;
        HoughCircles(imgThresholded, circles, HOUGH_GRADIENT, 1, imgThresholded.rows / 8, 100, 30, 0, 0);

        // If a red circle is detected, send stop signal
        if (!circles.empty())
        {
            cout << "start" << endl;
        }

        // Display the processed image
        imshow("Thresholded Image", imgThresholded);
        imshow("Original", imgOriginal);

        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            break;
        }
    }
}
