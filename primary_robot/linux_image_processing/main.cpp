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

//###############
//#### Infos ####
//###############
    /*
    Feu rouge: HSVmin 170,160,60
               HSVmax 180,256,256 // Hmax=180
    Feu Jaune: HSVmin 22,160,60
               HSVmax 38,256,256
    */

    // Paramétrer cam: Ds terminal -> locate svv    cd /home/yoyo/Bureau/svv      ./svv /dev/video1
	// Execution ds term: ~/Documents/depot-ENAC-Robotique/primary_robot/linux_image_processing/Debug$


//####################################################
//############### Proto fonctions ####################
//####################################################
//void Window_RGL_HSV();


//#########################################
//############## Var globales #############
//########################################
// Var RGL_HSV
    /*int lowerH=0, lowerS=0, lowerV=0;
    int upperH=180,upperS=256, upperV=256;
    int init_H = 50, init_S = 50, init_V = 50;
	*/




//#########################################
//############## Main #####################
//#########################################
int main(int argc, char* argv[]){
	struct timeval tv_1, tv_2;

	// open the video file for reading
		VideoCapture cap("fires_test.mkv");
//	    VideoCapture cap(1);

    // if not success, exit program
		if ( !cap.isOpened() ){
			 cout << "Cannot open the video file" << endl;
			 return -1;
		}

    // Start the video at 300ms
    //cap.set(CV_CAP_PROP_POS_MSEC, 300);

    //create a windows
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

    // Get the frames per seconds of the video
		double fps = cap.get(CV_CAP_PROP_FPS);
		cout << "Frame per seconds : " << fps << endl;

    // Get the width  and height of frames of the video
		double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
		double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
		cout << "Frame size : " << dWidth << " x " << dHeight << endl;

    // Write video in file
    	Size frameSize(static_cast<int>(dWidth), static_cast<int>(dHeight));
		// Initialize the VideoWriter object
		VideoWriter oVideoWriter ("MyVideo.avi", CV_FOURCC('P','I','M','1'), 20, frameSize, true);
		//if not initialize the VideoWriter successfully, exit the program
		if ( !oVideoWriter.isOpened() ) {
			cout << "ERROR: Failed to write the video" << endl;
			return -1;
		}


	// Initialisations
		Init_Info_Feux();


    while(1){
    	gettimeofday(&tv_1, NULL);

        Mat img_brut;

        // Read a new frame from video
        bool bSuccess = cap.read(img_brut);
        //if not success, break loop
        if (!bSuccess){
//			cout << "Cannot read the frame from video file" << endl;

	        cap = VideoCapture("/home/ludo6431/Documents/ENAC/ClubRobot/guvcview_video-1.mkv");

	        cap.read(img_brut);
//			break;
		}

        // Writer the frame into the file
        //if(SAVE){
        	oVideoWriter.write(img_brut);
        //}

        	if(process_frame(img_brut)){
        		break;
        	}


        gettimeofday(&tv_2, NULL);
        {
//        	int delta_us = (tv_2.tv_sec - tv_1.tv_sec)*1000000 + tv_2.tv_usec - tv_1.tv_usec;
//        	printf("%.2fFPS\n", 1000000./float(delta_us));
        }

//        printf("frame %i\n", nb_frame++);
    }// End while

    affich_Infos_feux(NBR_FEUX, true, false, true);

    return 0;
}




/*
void Window_RGL_HSV(){
    if(RGL_HSV){
        cvNamedWindow("Réglage HSV");
        // Change H, S, V
        createTrackbar("Hue", "Réglage HSV", &init_H, 256, MyCallbackForHSV,);
        createTrackbar("Saturation", "Réglage HSV", &init_S, 256, MyCallbackForHSV);
        createTrackbar("Value", "Réglage HSV", &init_V, 256, MyCallbackForHSV);

        cvCreateTrackbar("LowerV", "Réglage HSV", &lowerV, 256, NULL);
        cvCreateTrackbar("UpperV", "Réglage HSV", &upperV, 256, NULL);
    }
}
*/

//##########################################
/*
void MyCallbackForHSV(int iValueForH, void *userData)
{
	inRange(img_HSV , cv::Scalar(iValueForH,70,45), cv::Scalar(150,256,256), img_thres);
}
*/

//##########################################


//##########################################



