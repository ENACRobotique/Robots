#ifndef FRAME_TOOLS_H
#define FRAME_TOOLS_H

#include "tools.hpp"
#include "params.hpp"


typedef struct sInfCam{
	//// Intrinsic parameters
		double focal;
		double xSize;  // In pixels
		double ySize;  // In pixels
		// Matrices
		Mat matTransi_C2I;  // Transition matrix camera to image
		Mat matTransi_I2C;  // Transition matrix image to camera

	//// Extrinsic parameters
		// Angles for the orientation of the camera
		double Rx;
		double Ry;
		double Rz;

		// Position of the center of the camera on the robot
		double x;
		double y;
		double z;
		// Matrices  FIXME: Maybe not pertinent here
		Mat matRot_R2C;
		Mat matRot_C2R;
		Mat matTransla_C2R;
		Mat matTransit_C2R;
		Mat matTransit_R2C;
		Mat matOrigC_R;
}sInfCam;


int initCam();
void setCamIntrinParams(sInfCam* cam, double* f, double* xSize, double* ySize);
void setCamExtrinParams(sInfCam* cam, double* rx, double* ry, double* rz,
		                double* x, double* y, double* z);

#endif  // FRAME_TOOLS_H
