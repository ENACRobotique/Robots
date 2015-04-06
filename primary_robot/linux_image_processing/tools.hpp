/*
 * tools.hpp
 *
 *  Created on: 26 mars 2014
 *      Author: yoyo
 */

#ifndef TOOLS_HPP_
#define TOOLS_HPP_


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


//##############################
//####  General conversions ####
//##############################
#define RAD2DEG 180./M_PI
#define DEG2RAD M_PI/180.


Point2f I2R(Point2f pt_I);
Point2f R2I(Point2f pt_R);
Point2f Px2mm(Point2f pt_px);
Point2f Mm2px(Point2f pt_mm);
void Init_Info_Feux();
// Convert point cam to table
// Convert table to cam
// Convert pixel to mm
// Convert mm to pixel
bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2, Point2f &r);

#endif /* TOOLS_HPP_ */
