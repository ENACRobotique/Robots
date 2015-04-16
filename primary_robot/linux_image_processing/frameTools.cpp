#include <stdio.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "math.h"

using namespace cv;
using namespace std;


#include "frameTools.h"


int initCam(){
	// TODO
	return 0;
}

void setCamIntrinParams(sInfCam* cam, double* f, double* xSize, double* ySize){
	// Set intrinsic parameters
	cam->focal = *f;
	cam->xSize = *xSize-1;
	cam->ySize = *ySize-1;

	// Compute the transition matrix camera to image
	cam->matTransi_C2I.create(4, 4, CV_32FC1);  // For float values
	cam->matTransi_C2I = (Mat_<float>(4,4) <<
						cam->focal,       0,      cam->xSize/2,
			                 0,       cam->focal,   cam->ySize/2,
			                 0,           0,           1);

	// Compute the transition matrix image to camera
	cam->matTransi_I2C.create(4, 4, CV_32FC1);
	cam->matTransi_I2C = cam->matTransi_C2I.inv();
}

void setCamExtrinParams(sInfCam* cam, double* rx, double* ry, double* rz,
		                double* x, double* y, double* z){
	// Set extrinsic parameters
	cam->Rx = *rx;
	cam->Ry = *ry;
	cam->Rz = *rz;
	cam->x = *x;
	cam->y = *y;
	cam->z = *z;

	// Rotation matrix Rx_R2C, Ry_R2C, Rz_R2C
	double cRx = cos(cam->Rx);
	double sRx = sin(cam->Rx);
	Mat matRx_R2C = (Mat_<float>(4,4) << 1,   0,    0,
										 0,  cRx,  sRx,
										 0, -sRx,  cRx);
	double cRy = cos(cam->Ry);
	double sRy = sin(cam->Ry);
	Mat matRy_R2C = (Mat_<float>(4,4) << cRy,  0, -sRy,
										  0,   1,    0,
										 sRy,  0,  cRy);
	double cRz = cos(cam->Rz);
	double sRz = sin(cam->Rz);
	Mat matRz_R2C = (Mat_<float>(4,4) << cRz,  sRz,   0,
										-sRz,  cRz,   0,
										  0,    0,    1);
	// Compute the rotation matrix robot to camera
	cam->matRot_R2C = matRx_R2C*matRy_R2C*matRz_R2C;

	// Compute the rotation matrix camera to robot
	cam->matRot_C2R = cam->matRot_R2C.inv();

	// Compute the matrix transition
	cam->matTransi_I2C.create(3, 1, CV_32FC1);
	cam->matTransi_I2C = (Mat_<float>(3,1) << cam->x,
			                             cam->y,
			                             cam->z);
	// Compute the transition matrix camera to robot (homogeneous matrix)
	Rect roiRot = Rect(0, 0, cam->matRot_C2R.cols, cam->matRot_C2R.rows);
	cam->matRot_C2R.copyTo(cam->matTransit_C2R(roiRot));
	Rect roiTransi = Rect(3, 0, cam->matTransla_C2R.cols, cam->matTransla_C2R.rows); // cols first
	cam->matTransla_C2R.copyTo(cam->matTransit_C2R(roiTransi));
	Mat Z = Mat::zeros(1,3, CV_32FC1);
	Rect roiZ = Rect(0, 3, Z.cols, Z.rows);
	Z.copyTo(cam->matTransit_C2R(roiZ));
	cam->matTransit_C2R.at<float>(3,3) = 1;

	// Compute the transition matrix robot to camera (homogeneous matrix)
	cam->matTransit_R2C = cam->matTransit_C2R.inv();

	// Compute the position matrix of the camera in the robot
	cam->matOrigC_R = (Mat_<float>(4,1) << cam->x,
			                               cam->y,
			                               cam->z,
			                                1);
}



