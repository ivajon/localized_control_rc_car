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

class StartStop {
public:
    static int startStop(int LowH, int HighH, int LowS, int HighS, int LowV, int HighV, bool isRed);
    static int red();
    static int start();
};

#endif // STARTSTOP_H
