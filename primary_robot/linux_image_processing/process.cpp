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


/// Global Variables
const int h_slider_max = 180, sv_slider_max = 256;
// For blue threshold
Scalar hsvB_min,hsvB_max;
int hMinB_slider=110, hMaxB_slider=180,
	sMinB_slider=105, sMaxB_slider=220,
	vMinB_slider=100, vMaxB_slider=250;

// For red threshold
Scalar hsvR_min,hsvR_max;
int hMinR_slider=110, hMaxR_slider=180,
	sMinR_slider=105, sMaxR_slider=220,
	vMinR_slider=100, vMaxR_slider=250;

// For yellow threshold
Scalar hsvY_min,hsvY_max;
int hMinY_slider=110, hMaxY_slider=180,
	sMinY_slider=105, sMaxY_slider=220,
	vMinY_slider=100, vMaxY_slider=250;

// For green threshold
Scalar hsvG_min,hsvG_max;
int hMinG_slider=110, hMaxG_slider=180,
	sMinG_slider=105, sMaxG_slider=220,
	vMinG_slider=100, vMaxG_slider=250;

// For calibration
Scalar hsvCalib_min,hsvCalib_max;
int hMinCalib_slider=0, hMaxCalib_slider=180,
	sMinCalib_slider=0, sMaxCalib_slider=220,
	vMinCalib_slider=0, vMaxCalib_slider=250;


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


int frameProcess(Mat& frameRaw, Mat& framePattern, sPosOrien& posOrienRob){
	sFieldCam2Draw FieldCam;  // To draw the fiel of cam on framePattern
	Mat frameTopView = Mat(IR_HEIGHT, IR_WIDTH, CV_8UC3);

    // Matrices for the colors of the playgrounds
    Mat frameThresGreen;
    Mat frameThresYellow;
    Mat frameThresRed;
    Mat frameThresBlue;

    // Initialize the straightened frame and put in HSV mode
	if(frameStraight(frameRaw, frameTopView) == -1){
		cout<<"process_frame(): Error during the straightening"<<endl;
		return -1;
	}


	// Display the size of the frames
    cout<<"frameRaw     = "<<frameRaw.rows<<" x "<<frameRaw.cols<<endl;
	cout<<"framePattern = "<<framePattern.rows<<" x "<<frameRaw.cols<<endl;

    // Threshold, erode and dilate operations
		// FIXME: Default parameter doesn't working
		// For red color
		if(frameThresh(frameTopView, frameThresRed, hsvR_min, hsvR_max, 5, 8) < 0){
			cout<<"process_frame(): Error during the threshold operation"<<endl;
			return -1;
		}
//		// For blue color
//		if(frameThresh(frameTopViewHSV, frameThresBlue, hsvB_min, hsvB_max, 5, 8) < 0){
//			cout<<"process_frame(): Error during the threshold operation"<<endl;
//			return -1;
//		}


	// TODO: Make the process and the other Mat


    // Show the frame in "MyVideo" window
    imshow("frameRaw", frameRaw);
    imshow("framePattern", framePattern);
	imshow("frameTopViewHSV", frameTopView);
//    imshow("frameGreen",frameThresGreen);
//    imshow("frameYellow",frameThresYellow);
//    imshow("frameRed",frameThresRed);
//    imshow("frameBlue",frameThresBlue);

    // Uptade the view on the pattern frame
    setFieldCam2Draw(FieldCam, Point2i(300,200), Point2i(50,100), Point2i(200,400), Point2i(-200,400), Point2i(-50,100));
    drawTrapeze(framePattern, FieldCam);

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
			u = int((3195.*j - 5213.*sqrt(2)*i + 2794168.*sqrt(2) - 3952854.)/(10.*j - 12372.));  // FIXME: Use matrices
			v = int(-(7045.*j - 5151.)/(25.*j - 30930.));  // FIXME: Use matrices

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
int frameThresh(const Mat frameIn, Mat frameOut, Scalar hsvMin, Scalar hsvMax, int sizeErode, int sizeDilate){
//	Mat frame2Tresh(frameIn);
	frameIn.copyTo(frameOut);
	// Create elements for erode and dilate operations
	Mat elt_erode = getStructuringElement(
					MORPH_ELLIPSE,
					Size( sizeErode, sizeErode ),
					Point( 0, 0 ) );
	Mat elt_dilate = getStructuringElement(
					MORPH_ELLIPSE,
					Size( sizeDilate, sizeDilate ),
					Point( 0, 0 ) );

	//RGB to HSV
	cvtColor(frameOut, frameOut, CV_RGB2HSV);
	// Apply a threshold
	inRange(frameOut , hsvMin, hsvMax, frameOut);

	// Erode and dilate operations
	erode( frameOut, frameOut, elt_erode );
	dilate( frameOut, frameOut, elt_dilate );

	return 0;
}


int initFramePattern(string& title, VideoCapture& srcFramePattern, Mat& matFramePattern){
//	srcFramePattern.open("/home/yoyo/Robots/primary_robot/linux_image_processing/Images/Table2015.png");
	srcFramePattern.open(strFramePattern.c_str());
	if ( !srcFramePattern.isOpened() ){
		 cout << "Error: Cannot open the reference frame" << endl;
		 return -1;
	}

	if(!srcFramePattern.read(matFramePattern)){
		cout<<"Error: Cannot read the pattern frame"<<endl;
	}
	namedWindow(title.c_str(),CV_WINDOW_AUTOSIZE);
	return 0;
}

void drawTrapeze(Mat& frameSrc, sFieldCam2Draw& trapeze){
	Scalar tri_color = Scalar(0, 0, 256);

	line(frameSrc, trapeze.center + trapeze.v1, trapeze.center + trapeze.v2,
			tri_color, 4);
	line(frameSrc, trapeze.center + trapeze.v2, trapeze.center + trapeze.v3,
			tri_color, 4);
	line(frameSrc, trapeze.center + trapeze.v3, trapeze.center + trapeze.v4,
			tri_color, 4);
	line(frameSrc, trapeze.center + trapeze.v4, trapeze.center + trapeze.v1,
			tri_color, 4);

}

void setFieldCam2Draw(sFieldCam2Draw& trapeze, Point2i c, Point2i vert1, Point2i vert2, Point2i vert3, Point2i vert4){
	trapeze.center = c;
	trapeze.v1 = vert1;
	trapeze.v2 = vert2;
	trapeze.v3 = vert3;
	trapeze.v4 = vert4;
}


int initCalibHSV(VideoCapture& srcHSVPattern, Mat& matHSVPattern){
	srcHSVPattern.open(strFrameHSV.c_str());

	if ( !srcHSVPattern.isOpened() ){
		 cout << "Error: Cannot open the HSV pattern frame" << endl;
		 return -1;
	}

	if(!srcHSVPattern.read(matHSVPattern)){
		cout<<"Error: Cannot read the HSV pattern frame"<<endl;
		return -1;
	}
	return 0;
}

void initTrackbar(){
//	namedWindow("HSVSettings");

	// For blue threshold
	createTrackbar("H_B min", "HSVSettings", &hMinB_slider, h_slider_max, on_trackbar );
	createTrackbar("H_B max", "HSVSettings", &hMaxB_slider, h_slider_max, on_trackbar );
	createTrackbar("S_B min", "HSVSettings", &sMinB_slider, sv_slider_max, on_trackbar );
	createTrackbar("S_B max", "HSVSettings", &sMaxB_slider, sv_slider_max, on_trackbar );
	createTrackbar("V_B min", "HSVSettings", &vMinB_slider, sv_slider_max, on_trackbar );
	createTrackbar("V_B max", "HSVSettings", &vMaxB_slider, sv_slider_max, on_trackbar );

	// For red threshold
	createTrackbar("H_R min", "HSVSettings", &hMinR_slider, h_slider_max, on_trackbar );
	createTrackbar("H_R max", "HSVSettings", &hMaxR_slider, h_slider_max, on_trackbar );
	createTrackbar("S_R min", "HSVSettings", &sMinR_slider, sv_slider_max, on_trackbar );
	createTrackbar("S_R max", "HSVSettings", &sMaxR_slider, sv_slider_max, on_trackbar );
	createTrackbar("V_R min", "HSVSettings", &vMinR_slider, sv_slider_max, on_trackbar );
	createTrackbar("V_R max", "HSVSettings", &vMaxR_slider, sv_slider_max, on_trackbar );

	// For yellow threshold
	createTrackbar("H_Y min", "HSVSettings", &hMinY_slider, h_slider_max, on_trackbar );
	createTrackbar("H_Y max", "HSVSettings", &hMaxY_slider, h_slider_max, on_trackbar );
	createTrackbar("S_Y min", "HSVSettings", &sMinY_slider, sv_slider_max, on_trackbar );
	createTrackbar("S_Y max", "HSVSettings", &sMaxY_slider, sv_slider_max, on_trackbar );
	createTrackbar("V_Y min", "HSVSettings", &vMinY_slider, sv_slider_max, on_trackbar );
	createTrackbar("V_Y max", "HSVSettings", &vMaxY_slider, sv_slider_max, on_trackbar );

	// For green threshold
	createTrackbar("H_G min", "HSVSettings", &hMinG_slider, h_slider_max, on_trackbar );
	createTrackbar("H_G max", "HSVSettings", &hMaxG_slider, h_slider_max, on_trackbar );
	createTrackbar("S_G min", "HSVSettings", &sMinG_slider, sv_slider_max, on_trackbar );
	createTrackbar("S_G max", "HSVSettings", &sMaxG_slider, sv_slider_max, on_trackbar );
	createTrackbar("V_G min", "HSVSettings", &vMinG_slider, sv_slider_max, on_trackbar );
	createTrackbar("V_G max", "HSVSettings", &vMaxG_slider, sv_slider_max, on_trackbar );

	on_trackbar(0,NULL);
}

void initTrackbarCalib(Mat& frameCalib, string& title){
	namedWindow("HSV_Calib", CV_WINDOW_AUTOSIZE);

// For calibration threshold
	createTrackbar("H_Calib min", title.c_str(), &hMinCalib_slider, h_slider_max, on_trackbarCalib, &frameCalib );
	createTrackbar("H_Calib max", title.c_str(), &hMaxCalib_slider, h_slider_max, on_trackbarCalib, &frameCalib );
	createTrackbar("S_Calib min", title.c_str(), &sMinCalib_slider, sv_slider_max, on_trackbarCalib, &frameCalib );
	createTrackbar("S_Calib max", title.c_str(), &sMaxCalib_slider, sv_slider_max, on_trackbarCalib, &frameCalib );
	createTrackbar("V_Calib min", title.c_str(), &vMinCalib_slider, sv_slider_max, on_trackbarCalib, &frameCalib );
	createTrackbar("V_Calib max", title.c_str(), &vMaxCalib_slider, sv_slider_max, on_trackbarCalib, &frameCalib );

//	on_trackbarCalib();
}

void on_trackbar(int, void*){
	// For blue threshold
	hsvB_min = Scalar(hMinB_slider, sMinB_slider, vMinB_slider);
	hsvB_max = Scalar(hMaxB_slider, sMaxB_slider, vMaxB_slider);

	// For red threshold
	hsvR_min = Scalar(hMinR_slider, sMinR_slider, vMinR_slider);
	hsvR_max = Scalar(hMaxR_slider, sMaxR_slider, vMaxR_slider);

	// For yellow threshold
	hsvY_min = Scalar(hMinY_slider, sMinY_slider, vMinY_slider);
	hsvY_max = Scalar(hMaxY_slider, sMaxY_slider, vMaxY_slider);

	// For green threshold
	hsvG_min = Scalar(hMinG_slider, sMinG_slider, vMinG_slider);
	hsvG_max = Scalar(hMaxG_slider, sMaxG_slider, vMaxG_slider);
}

void on_trackbarCalib(int, void* mat){
	// For calibration threshold
	hsvCalib_min = Scalar(hMinCalib_slider, sMinCalib_slider, vMinCalib_slider);
	hsvCalib_max = Scalar(hMaxCalib_slider, sMaxCalib_slider, vMaxCalib_slider);

	Mat img(*((Mat*)mat));
	imshow("HSV_Calib", img);
}

int displTwinImages(string& title, int maxwidthImgOut, const Mat& imgIn1, const Mat& imgIn2, Mat& imgOut, int blackBand){
//	#define DBG_displTwinImages
	float scale;
	Mat img1(imgIn1);
	Mat img2(imgIn2);

	// TODO: Add checks
	if(img1.cols != img2.cols ||
			img1.rows != img2.rows){
		cout<<"displTwinImages(): Dimensions of input images are different"<<endl;
		return -1;
	}
	else{
		#ifdef DBG_displTwinImages
		cout<<"displTwinImages(): maxwidthImgOut = "<<maxwidthImgOut<<endl;
		#endif
		// Scale factor
		scale =  (((float)(maxwidthImgOut-blackBand)/2) / img1.cols);
		#ifdef DBG_displTwinImages
		cout<<"displTwinImages(): scale = "<<scale<<endl;
		#endif
		// Initialize the output image
		imgOut.create(img1.rows*scale+1, img1.cols*scale*2 + blackBand, CV_8UC3);
		// Resize the sub images
		#ifdef DBG_displTwinImages
		int a = (int)(scale*img1.rows);
		int b = (int)(img1.cols*scale);
		cout<<"displTwinImages(): a = "<<a<<endl;
		cout<<"displTwinImages(): b = "<<b<<endl;
		#endif
		resize(img1, img1, cvSize(0, 0), scale, scale);
		resize(img2, img2, cvSize(0, 0), scale, scale);
		// Set the ROI of the input image to display the sub image
		Rect roi2 = Rect(img1.cols+blackBand, 0, img2.cols, img2.rows);
		Rect roi1 = Rect(0, 0, img1.cols, img1.rows);
		#ifdef DBG_displTwinImages
		cout<<"displTwinImages(): img1 : cols = "<<img1.cols<<", rows = "<< img1.rows<<endl;
		cout<<"displTwinImages(): img2 : cols = "<<img2.cols<<", rows = "<< img2.rows<<endl;
		cout<<"displTwinImages(): imgOut : cols = "<<imgOut.cols<<", rows = "<< imgOut.rows<<endl;
		#endif
		// Fill the output image
		img1.copyTo(imgOut(roi1));
		img2.copyTo(imgOut(roi2));
		// Create window and show the output image
		namedWindow(title.c_str(), CV_WINDOW_AUTOSIZE);
		imshow(title.c_str(), imgOut);

		return 0;
	}
}
