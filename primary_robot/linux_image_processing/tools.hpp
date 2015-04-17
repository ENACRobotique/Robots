/*
 * tools.hpp
 *
 *  Created on: 26 mars 2014
 *      Author: yoyo
 */

#ifndef TOOLS_HPP_
#define TOOLS_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include "params.hpp"
#include "process.hpp"


using namespace cv;  // Check if it is relevant to declare this here
using namespace std;  // Same remark

//##############################
//####  General conversions ####
//##############################
#define RAD2DEG 180./M_PI
#define DEG2RAD M_PI/180.


cv::Point2f I2R(Point2f pt_I);
Point2f R2I(Point2f pt_R);
Point2f Px2mm(Point2f pt_px);
Point2f Mm2px(Point2f pt_mm);
void Init_Info_Feux();
// Convert point cam to table
// Convert table to cam
// Convert pixel to mm
// Convert mm to pixel
bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2, Point2f &r);

/**
 * \brief Give information to know if a special key "key" (decimal value of the ASCII table)
 * 		has been pressed during the delay "dalay_us" in microsecond.
 * 		Return 1 if the specified key is pressed, 0 otherwise
 * \param int key
 * \param int delay_us
 * \return int
 */
int getKey(int key, int delay_us = 30);


#endif /* TOOLS_HPP_ */
