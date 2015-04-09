/*
 * process.cpp
 *
 *  Created on: 26 mars 2014
 *      Author: yoyo
 */

#include <stdio.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace std;


#include "process.hpp"


Scalar hsv_min,hsv_max;
/// Global Variables
const int h_slider_max = 180, sv_slider_max = 256;
int hmin_slider=110, hmax_slider=180, smin_slider=105, smax_slider=220, vmin_slider=100, vmax_slider=250;

void on_trackbar(int, void*){
	hsv_min = Scalar(hmin_slider, smin_slider, vmin_slider);
	hsv_max = Scalar(hmax_slider, smax_slider, vmax_slider);
}


float xCam = 2600;    // x position of the camera on the robot
float yCam = 1400;    // y position of the camera on the robot
float zCam = 0;       // z position of the camera on the robot
float elevatCam = 0;  // elevation angle of the camera on the robot

// Coefficients for the transition matrix from robot to playground references
float Pass_R_T11 = cos(elevatCam) / ( pow(cos(elevatCam),2) + pow(sin(elevatCam),2) ),
	  Pass_R_T12 = -sin(elevatCam) / ( pow(cos(elevatCam),2) + pow(sin(elevatCam),2) ),
	  Pass_R_T13 = 0,
	  Pass_R_T14 = xCam,
	  Pass_R_T21 = sin(elevatCam) / ( pow(cos(elevatCam),2) + pow(sin(elevatCam),2) ),
	  Pass_R_T22 = cos(elevatCam) / ( pow(cos(elevatCam),2) + pow(sin(elevatCam),2) ),
	  Pass_R_T23 = 0,
	  Pass_R_T24 = yCam,
	  Pass_R_T31 = 0,
	  Pass_R_T32 = 0,
	  Pass_R_T33 = 1,
	  Pass_R_T34 = zCam,
	  Pass_R_T41 = 0,
	  Pass_R_T42 = 0,
	  Pass_R_T43 = 0,
	  Pass_R_T44 = 1;

// Matrix of transition from robot to playground references
Mat Pass_R_T = (Mat_<float>(4,4) << Pass_R_T11, Pass_R_T12, Pass_R_T13, Pass_R_T14,
									Pass_R_T21, Pass_R_T22, Pass_R_T23, Pass_R_T24,
									Pass_R_T31, Pass_R_T32, Pass_R_T33, Pass_R_T34,
									Pass_R_T41, Pass_R_T42, Pass_R_T43, Pass_R_T44);


int frameProcess(Mat& rawFrame, sPosOrien& posOrienRob){
	// General matrices
	Mat frameTopViewHSV = Mat(IR_HEIGHT, IR_WIDTH, CV_8UC3);

    // Matrices for the colors of the playgrounds
    Mat frameThresGreen;
    Mat frameThresYellow;
    Mat frameThresRed;
    Mat frameThresBlue;


    // Initialize the straightened frame and put in HSV mode
	if(frameStraight(rawFrame, frameTopViewHSV) == -1){
		cout<<"process_frame(): Error during the straightening"<<endl;
		return -1;
	}


    // Threshold, erode and dilate operations
		// FIXME: Default parameter doesn't working
		//for red color
		if(frameThresh(frameTopViewHSV, frameThresRed, hsv_min, hsv_max, 5, 8) < 0){
			cout<<"process_frame(): Error during the threshold operation"<<endl;
			return -1;
		}
		// For blue color
		if(frameThresh(frameTopViewHSV, frameThresBlue, hsv_min, hsv_max, 5, 8) < 0){
			cout<<"process_frame(): Error during the threshold operation"<<endl;
			return -1;
		}


	// TODO: Make the process and the other Mat


    // Show the frame in "MyVideo" window
    imshow("rawFrame", rawFrame);
	imshow("frameTopViewHSV", frameTopViewHSV);
    imshow("frameGreen",frameThresGreen);
    imshow("frameYellow",frameThresYellow);
    imshow("frameRed",frameThresRed);
    imshow("frameBlue",frameThresBlue);


    // Wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
    if(getKey(27, 30) == 1){  // FIXME: default parameter not working
		cout << """esc"" key is pressed by user" << endl;
		return -1;
    }
    return 0;
}


/**
 * \breif Straightened the input frame "frameIn" following the matrix transformation
 * 		to provide the output frame "frameOut"
 * \param Mat frameIn
 * \param Mat frameOut
 * \return int: 0 if OK, -1 otherwise
 * TODO: Add checks
 * TODO: Update the formulas
 */
int frameStraight(Mat& frameIn, Mat& frameOut){
	int u,v;
	for(int j=0;j<IR_HEIGHT;j++){
		for(int i=0;i<IR_WIDTH;i++){
			u = int((3195.*j - 5213.*sqrt(2)*i + 2794168.*sqrt(2) - 3952854.)/(10.*j - 12372.));
			v = int(-(7045.*j - 5151.)/(25.*j - 30930.));

			if(u>=0 && u<640 && v>=0 && v<480){ // Access to a pixel
				for(int a=0;a<3 ; a++){  // Over the three channel
					frameOut.at<Vec3b>(j,i)[a] = frameIn.at<Vec3b>(v,u)[a];
				}
			}
			else{
				for(int a=0;a<3 ; a++){
					frameOut.at<Vec3b>(j,i)[a] = 0;
				}
			}
		}
	}

	// Change the colorimetry mode from RGB to HSV
	cvtColor(frameOut, frameOut, CV_RGB2HSV);
	return 0;
}

/**
 * \brief Allow to keep only a part of the frame,
 * 		the output frame is black everywhere except in the circular area
 * 		which correspond to the input frame
 * \param Mat& frame
 * \param Point2i& center
 * \param int radius
 * \return int: 0 if Ok, -1 otherwise
 */
int frameCrop2Circle(Mat& frame, Point2i& center, int radius){
	Point2i p_c, curPx;

	// Crop the frame
	for(int j=0;j<IR_HEIGHT;j++){
		for(int i=0;i<IR_WIDTH;i++){
			curPx.x = i;
			curPx.y = j;
			if(norm(center - curPx) > radius){
				for(int a=0;a<3 ; a++){ //sur les trois canaux
					frame.at<Vec3b>(j,i)[a] = 0;
				}
			}
		}
	}

	return 0;
}


/**
 * \brief Apply a threshold on the input frame following hsvMin and hsvMax
 * 		parameters and make erode and dilate operation in order to remove
 * 		some noise or irregularity in the input frame
 * \param Mat frameIn
 * \param Mat frameOut
 * \param Scalar hsvMin
 * \param Scalar hsvMax
 * \param int sizeErode
 * \param int sizeDilate
 * \return int: 0 if OK, -1 otherwise
 */
int frameThresh(Mat frameIn, Mat frameOut, Scalar hsvMin, Scalar hsvMax, int sizeErode, int sizeDilate){
	// Create elements for erode and dilate operations
	Mat elt_erode = getStructuringElement(
					MORPH_ELLIPSE,
					Size( sizeErode, sizeErode ),
					Point( 0, 0 ) );
	Mat elt_dilate = getStructuringElement(
					MORPH_ELLIPSE,
					Size( sizeDilate, sizeDilate ),
					Point( 0, 0 ) );

	// Apply a threshold
	inRange(frameIn , hsvMin, hsvMax, frameOut);

	// Erode and dilate operations
	erode( frameOut, frameOut, elt_erode );
	dilate( frameOut, frameOut, elt_dilate );

	return 0;
}
