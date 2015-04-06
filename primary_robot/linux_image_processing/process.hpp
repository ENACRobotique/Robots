/*
 * process.hpp
 *
 *  Created on: 26 mars 2014
 *      Author: yoyo
 */

#ifndef PROCESS_HPP_
#define PROCESS_HPP_

#include "tools.hpp"
#include "params.hpp"

using namespace cv;  // Check if declaration here is relevant


typedef enum {
	Rouge, Jaune, Autre_Coul
}Coul_Feu;

typedef struct sPosOrien{
	double x;
	double y;
	double theta;
}sPosOrien;


extern Scalar hsv_min,hsv_max;

/// Global Variables
extern const int h_slider_max, sv_slider_max;
extern int hmin_slider, hmax_slider, smin_slider, smax_slider, vmin_slider, vmax_slider;


void on_trackbar(int, void*);
int frameProcess(Mat& rawFrame, sPosOrien& posOrienRob);
int frameStraight(Mat& frameIn, Mat& frameOut);
int frameCrop2Circle(Mat& frame, Point2i& center, int radius);
int frameThresh(Mat frameIn, Mat frameOut, Scalar hsvMin, Scalar hsvMax, int sizeErode = 5, int sizeDilate = 8);





#endif /* PROCESS_HPP_ */
