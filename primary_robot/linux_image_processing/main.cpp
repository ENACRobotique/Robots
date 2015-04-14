/*
 * DisplayImage.cpp
 *
 *  Created on: 23 févr. 2014
 *      Author: yoyo
 */

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>


using namespace cv;
using namespace std;


#include "tools.hpp"
#include "params.hpp"
#include "process.hpp"
#include "performance.h"
#include "save.h"
#include "sourceVid.h"


//###############
//#### TODO ####
//##############
/* To create build configuration for PC and BBB (BeagleBon Black)
 *
 */


//####################
//#### Information ##
//###################
// To param cam manually:
	// In terminal -> $ locate svv;  $ cd ->svv;  $ ./svv /dev/video1



//##################################
//############## Main ##############
//##################################
int main(int argc, char* argv[]){
    sPerf sValPerf;
    sPosOrien posOriRobot;
    Mat framePattern;
    Mat frameRaw;

    // Init postion and orientation of robot
    // TODO: Later use the information sent by the AI
    posOriRobot.x = 0;
    posOriRobot.y = 0;
    posOriRobot.theta = 0;

	// Init video sources
    VideoCapture srcFramePattern;  // For the pattern of the table
	VideoCapture cap;
    initFramePattern(srcFramePattern, framePattern);  //// Initialize the pattern frame
	initCapture(cap);

	// For calibration
	Mat frameHSVPattern;
	Mat frameHSVCalib;
	VideoCapture srcHSVPattern;
	VideoCapture srcHSVCalib;

    // Initialize calibration
    initCalibHSV(srcHSVPattern, frameHSVPattern);
    initCalibHSV(srcHSVCalib, frameHSVCalib);
    initTrackbarCalib(frameHSVCalib);

    // Apply a threshold
	inRange(frameHSVCalib , hsvCalib_min, hsvCalib_max, frameHSVCalib);

    // Create  windows
	namedWindow("rawFrame",CV_WINDOW_AUTOSIZE);
	namedWindow("framePattern",CV_WINDOW_AUTOSIZE);
	namedWindow("HSVPattern", CV_WINDOW_AUTOSIZE);
	namedWindow("HSVCalib", CV_WINDOW_AUTOSIZE);
//	namedWindow("frameTopView",CV_WINDOW_AUTOSIZE);
//	namedWindow("frameGreen",CV_WINDOW_AUTOSIZE);
//	namedWindow("frameRed",CV_WINDOW_AUTOSIZE);
//	namedWindow("frameBlue",CV_WINDOW_AUTOSIZE);
//	namedWindow("frameYellow",CV_WINDOW_AUTOSIZE);

    // Settings HSV
	if(SETTINGS_HSV){

	}

	// Iinit the record of the video
	VideoWriter oVideoWriter;
	if(SAVE){
		if(initSave(cap, oVideoWriter) == -1){
			cout<<"Error: Failed to initialize the VideoWritter"<<endl;
			return -1;
		}
	}

    while(1){
    	cmptPerfFrame(StartPerf, sValPerf);

        // Read a new frame from the video source
        if (!cap.read(frameRaw)){  //if not success, break loop
			cout << "Cannot read the frame from source video file" << endl;
			break;
		}

        // Write the raw frame into the file
        if(SAVE){
        	save(oVideoWriter, frameRaw);
        }

        // Image processing
		if(frameProcess(frameRaw, framePattern, posOriRobot)){
			break;
		}

		// Show calibration
	    imshow("HSVPattern", frameHSVPattern);
	    imshow("HSVCalib", frameHSVCalib);

        // End of measurements
        cmptPerfFrame(EndPerf, sValPerf);
    }// End while

    printf("End loop\n");

    return 0;
}






