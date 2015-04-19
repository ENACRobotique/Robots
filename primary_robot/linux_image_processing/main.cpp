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

// For communications
//#include "../../network_config/messages.h"
//#include "../../network_config/messages-elements.h"
//#include "node_cfg.h"
//#include "../../static/communication/botNet/shared/botNet_core.h"
//#include "../../static/communication/botNet/shared/message_header.h"
//#include "../../network_config/roles.h"  // FIXME: ??? Is it relevant ???

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
 * Add bn communication
 */

//####################
//#### Information ##
//###################
// To param cam manually:
// In terminal -> $ locate svv;  $ cd ->svv;  $ ./svv /dev/video1
//##################################
//############## Main ##############
//##################################
int main(int argc, char* argv[]) {
	sPerf sValPerf;
	sPosOrien posOriRobot;
	Mat framePattern;
	Mat frameRaw;
//	int ret;
//	sMsg inMsg = {{0}}, outMsg = {{0}};s

    // botNet initialization
//    bn_init();

	// Init video sources
	VideoCapture srcFramePattern;  // For the pattern of the table
	VideoCapture cap;
	string titleFrameRaw("frameRaw");
	initCapture(titleFrameRaw, cap, 0, false); // FIXME: default parameter not working
	string titleFramePatt("framePattern");
	initFramePattern(titleFramePatt, srcFramePattern, framePattern); //// Initialize the pattern frame

#ifdef SETTINGS_HSV
	// For calibration
	Mat frameHSVPattern;
	Mat frameHSVCalib;
	VideoCapture srcHSVPattern;
	VideoCapture srcHSVCalib;
	// Initialize calibration
	Mat frameGlobCalib;
	initCalibHSV(srcHSVPattern, frameHSVPattern);
	initCalibHSV(srcHSVCalib, frameHSVCalib);
	string titleCalib("HSV_Calib");
	initTrackbarCalib(frameHSVCalib, titleCalib);
#endif

	// Create  windows
//	namedWindow("Anything", CV_WINDOW_AUTOSIZE);
//	namedWindow("frameGreen",CV_WINDOW_AUTOSIZE);
//	namedWindow("frameRed",CV_WINDOW_AUTOSIZE);
//	namedWindow("frameBlue",CV_WINDOW_AUTOSIZE);
//	namedWindow("frameYellow",CV_WINDOW_AUTOSIZE);

	// Iinit the record of the video
#ifdef SAVE
	VideoWriter oVideoWriter;
	if(initSave(cap, oVideoWriter) == -1) {
		cout<<"Error: Failed to initialize the VideoWritter"<<endl;
		return -1;
	}
#endif

	while (1) {
//		ret = bn_receive(&inMsg);
//
//		switch(inMsg.header.type){
//            case E_POS_CAM:
				cmptPerfFrame(StartPerf, sValPerf);
				// Read a new frame from the video source
				if (!cap.read(frameRaw)) {  //if not success, break loop
					cout << "Cannot read the frame from source video file" << endl;
					break;
				}

				// Write the raw frame into the file
				#ifdef SAVE
				save(oVideoWriter, frameRaw);
				#endif

				// Image processing
				if (frameProcess(frameRaw, framePattern, posOriRobot)) {
					break;
				}

				//// Image calibration
				#ifdef SETTINGS_HSV
				// Apply a threshold
				if(frameThresh(frameHSVCalib, frameHSVCalib, hsvCalib_min, hsvCalib_max, 5, 8) < 0) {
					cout<<"process_frame(): Error during the threshold operation"<<endl;
					return -1;
				}
				// Show calibration
				displTwinImages(titleCalib, 700, frameHSVPattern, frameHSVCalib, frameGlobCalib, 10);
				#endif

				// End of measurements
				cmptPerfFrame(EndPerf, sValPerf);
//
//                break;
//            case E_DATA:
//            case E_PING:
//                break;
//            default:
//                bn_printfDbg("got unhandled msg: type%hhu sz%hhu", inMsg.header.type, inMsg.header.size);
//                break;
//		}  // End switch

	}  // End while

	printf("End loop\n");

	return 0;
}

