#ifndef STARTSTOP_H
#define STARTSTOP_H

#include <opencv2/opencv.hpp>
#include <mutex>

using namespace cv;
using namespace std;

extern mutex mutex_var;
extern Mat hsvImage;
extern int frameID;
extern atomic<bool> stop;

/// Struct for storing color limits for cicrle detection in HSV space
struct colorLimitsHSV {
    int iLowH;
    int iHighH;

    int iLowS;
    int iHighS;


    int iLowV;
    int iHighV;
};

class StartStop {
private:
    colorLimitsHSV redLims;
    colorLimitsHSV greenLims;
    std::mutex* mutex_var;

    /// Actual circle detection algorithm
    int startStop(int LowH, int HighH, int LowS, int HighS, int LowV, int HighV, bool isRed);
public:
    /// Constructor
    StartStop(std::mutex* mutex_var);
 
    
    ///Search for red
    int red();
    ///Search for green
    int green();
    ///Start searching
    int start();

    /// Start functions in main thread
    std::thread startThread() {
        return std::thread([this] { this->start(); });
    }

};


#endif // STARTSTOP_H
