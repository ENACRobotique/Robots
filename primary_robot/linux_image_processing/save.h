#ifndef SAVE_H
#define SAVE_H

#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace std;


int initSave(VideoCapture frameCap, VideoWriter& oVideoWriter);
int  save(VideoWriter& oVideoWriter, Mat& frame);

#endif  //SAVE_H
