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
#include "sourceVid.hpp"

using namespace cv;  // Check if declaration here is relevant


typedef enum {
	Rouge, Jaune, Autre_Coul
}Coul_Feu;

typedef struct sPosOrien{
	double x;
	double y;
	double theta;
}sPosOrien;

typedef struct sFieldCam2Draw{
	Point2i v1;  // Relative to center
	Point2i v2;	 // Relative to center
	Point2i v3;  // Relative to center
	Point2i v4;  // Relative to center
	Point2i center;
}sFieldCam2Draw;


extern Scalar hsv_min,hsv_max;

/// Global Variables
extern const int h_slider_max, sv_slider_max;
extern int hmin_slider, hmax_slider, smin_slider, smax_slider, vmin_slider, vmax_slider;
extern Scalar hsvCalib_min,hsvCalib_max;
//extern int hMinCalib_slider, hMaxCalib_slider,
//	sMinCalib_slider, sMaxCalib_slider,
//	vMinCalib_slider, vMaxCalib_slider;
extern Scalar hsvCalib_min,hsvCalib_max;

void on_trackbar(int, void*);
int frameProcess(Mat& rawFrame, Mat& framePattern, sPosOrien& posOrienRob);
int frameStraight(Mat& frameIn, Mat& frameOut);
int frameCrop2Circle(Mat& frame, Point2i& center, int radius);
int frameThresh(const Mat& frameIn, Mat& frameOut, Scalar hsvMin, Scalar hsvMax, int sizeErode = 5, int sizeDilate = 8);
int initFramePattern(string& title, VideoCapture& srcFramePattern, Mat& matFramePattern);
void drawTrapeze(Mat& frameSrc, sFieldCam2Draw& trapeze);
void setFieldCam2Draw(sFieldCam2Draw& trapeze, Point2i c, Point2i vert1, Point2i vert2, Point2i vert3, Point2i vert4);
int initCalibHSV(VideoCapture& srcHSVPattern, Mat& matHSVPattern);
void initTrackbar();
void initTrackbarCalib(Mat& frameCalib, string& title);
//void on_trackbarCalib(Mat& frameCalib);
void on_trackbarCalib(int, void* mat);
int displTwinImages(string& title, int maxSizeEdgeImgOut, const Mat& imgIn1, const Mat& imgIn2, Mat& imgOut, int blackBand);


#endif /* PROCESS_HPP_ */
