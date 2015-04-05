/*
 * tools.cpp
 *
 *  Created on: 26 mars 2014
 *      Author: yoyo
 */

#include <opencv2/core/core.hpp>

using namespace cv;
using namespace std;

#include "params.hpp"
#include "tools.hpp"
#include "process.hpp"

//###########################################
//############# Fonctions ###################
//###########################################

void Init_Info_Feux(){
	for(int k=0; k<NBR_FEUX; k++ ){
		Infos_feux[k].x_feu = -1.;
		Infos_feux[k].y_feu = -1.;
		Infos_feux_cap[k].x_feu = -1.;
		Infos_feux_cap[k].y_feu = -1.;
	}
}

Point2f I2R(Point2f pt_I){
	Point2f pt_R;
	pt_R.x = (1210.*pt_I.x)/1671. - 648560./1671.;
	pt_R.y = 423809./557. - (1210.*pt_I.y)/1671.;
	return pt_R;
}


Point2f R2I(Point2f pt_R){
	Point2f pt_I;
	pt_I.x = (pt_R.x + 648560./1671.) * 1671./1210.;
	pt_I.y = (pt_R.y - 423809./557.) * 1671./1210.;
	return pt_I;
}


Point2f Px2mm(Point2f pt_px){
	Point2f pt_mm;
	pt_mm.x = 1210./1671. * pt_px.x;
	pt_mm.y = 1210./1671. * pt_px.y;
	return pt_mm;
}


Point2f Mm2px(Point2f pt_mm){
	Point2f pt_px;
	pt_px.x = 1671./1210. * pt_mm.x;
	pt_px.y = 1671./1210. * pt_mm.y;
	return pt_mm;
}

// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
// from http://answers.opencv.org/question/9511/how-to-find-the-intersection-point-of-two-lines/
bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2, Point2f &r)
{
    Point2f x = o2 - o1;
    Point2f d1 = p1 - o1;
    Point2f d2 = p2 - o2;

    float cross = d1.x*d2.y - d1.y*d2.x;
    if (fabs(cross) < /*EPS*/1e-8)
        return false;

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    r = o1 + d1 * t1;
    return true;
}

