#ifndef STARTSTOP_H
#define STARTSTOP_H

#include <opencv2/opencv.hpp>
#include <mutex>

using namespace cv;
using namespace std;

extern mutex mutex_var;
extern Mat hsvImage;
extern int frameID;
extern bool stop;

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
public:
    /// Constructor
    StartStop();
   

    int startStop(int LowH, int HighH, int LowS, int HighS, int LowV, int HighV, bool isRed);
    int red();
    int start();

    /// Start functions in main thread
    std::thread startThread() {
        return std::thread([this] { this->start(); });
    }

};


#endif // STARTSTOP_H
