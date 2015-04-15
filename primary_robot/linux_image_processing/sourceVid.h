#ifndef SOURCEVID_H
#define SOURCEVID_H

#include "opencv2/highgui/highgui.hpp"
#include "iostream"
#include "params.hpp"


using namespace cv;
using namespace std;


// Path to the image sources for YOYO
extern string strFramePattern;
extern string strFrameRaw;
extern string strFrameHSV;


int initCapture(string& title, VideoCapture& srcCap, int valBegVid = 0, bool offsetVid = false);


#endif  // SOURCEVID_H
