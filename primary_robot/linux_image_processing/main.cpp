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

	// Init video source
		VideoCapture cap;
		initCapture(cap);

    // Create  windows
		namedWindow("MyVideo",CV_WINDOW_AUTOSIZE);
		namedWindow("Video_thresh_R",CV_WINDOW_AUTOSIZE);
		//namedWindow("Video_thresh_J",CV_WINDOW_AUTOSIZE);

    // Réglage HSV
		if(RGL_HSV){
			namedWindow("RGL_HSV");

			createTrackbar("H min", "RGL_HSV", &hmin_slider, h_slider_max, on_trackbar );
			createTrackbar("H max", "RGL_HSV", &hmax_slider, h_slider_max, on_trackbar );
			createTrackbar("S min", "RGL_HSV", &smin_slider, sv_slider_max, on_trackbar );
			createTrackbar("S max", "RGL_HSV", &smax_slider, sv_slider_max, on_trackbar );
			createTrackbar("V min", "RGL_HSV", &vmin_slider, sv_slider_max, on_trackbar );
			createTrackbar("V max", "RGL_HSV", &vmax_slider, sv_slider_max, on_trackbar );

			on_trackbar(0,NULL);
		}

	// Iinit the record of the video
		VideoWriter oVideoWriter;
		if(SAVE){
			if(initSave(cap, oVideoWriter) == -1){
				cout<<"Error: Failed to init the VideoWritter"<<endl;
				return -1;
			}
		}

	// Initialisations
		Init_Info_Feux();

    while(1){
    	cmptPerfFrame(StartPerf, sValPerf);

        Mat frameRaw;

        // Read a new frame from video
        if (!cap.read(frameRaw)){  //if not success, break loop
			cout << "Cannot read the frame from source video file" << endl;
	        cap = VideoCapture("/home/yoyo/Robots/primary_robot/linux_image_processing/Videos/Feux.mp4");
			cap.read(frameRaw);
			break;
		}

        // Write the raw frame into the file
        if(SAVE){
        	save(oVideoWriter, frameRaw);
        }

        // Image processing
        	if(process_frame(frameRaw)){
        		break;
        	}

        // End of measurements
        cmptPerfFrame(EndPerf, sValPerf);
    }// End while

    printf("End loop\n");
    affich_Infos_feux(NBR_FEUX, true, false, true);

    return 0;
}






