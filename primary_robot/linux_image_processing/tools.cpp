/*
 * tools.cpp
 *
 *  Created on: 26 mars 2014
 *      Author: yoyo
 */

#include "tools.hpp"

using namespace cv;
using namespace std;


//#######################
//###### Fonctions ######
//#######################
/**
 * TODO: Use macro to generalized the function
 * TODO: Update the formulas
 */
Point2f I2R(Point2f pt_I){
	Point2f pt_R;
	pt_R.x = (pt_I.x)/PX2MM - 648560./1671.;
	pt_R.y = 423809./557. - (pt_I.y)/PX2MM;
	return pt_R;
}

/**
 * TODO: Use macro to generalized the function
 * TODO: Update the formulas
 */
Point2f R2I(Point2f pt_R){
	Point2f pt_I;
	pt_I.x = (pt_R.x + 648560./1671.) * MM2PX;
	pt_I.y = (pt_R.y - 423809./557.) * MM2PX;
	return pt_I;
}

/**
 * TODO: Use macro to generalized the function
 */
Point2f Px2mm(Point2f pt_px){
	Point2f pt_mm;
	pt_mm.x = PX2MM * pt_px.x;
	pt_mm.y = PX2MM * pt_px.y;
	return pt_mm;
}

/**
 * TODO: Use macro to generalized the function
 */
Point2f Mm2px(Point2f pt_mm){
	Point2f pt_px;
	pt_px.x = 1671./1210. * pt_mm.x;
	pt_px.y = 1671./1210. * pt_mm.y;
	return pt_mm;
}

// ___________ START geometric tools___________________
/**
 * \brief Finds the intersection of two lines, otherwise returns false.
 *        The lines are defined by (o1, p1) and (o2, p2).
 * \param Point2f o1
 * \param Point2f p1
 * \param Point2f o2
 * \param Point2f p2
 * \param Point2f &r
 * \return bool
 */
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

/**
 * Return the index of the nearest point in the vector of points "pts" to the point "pt"
 */
int getNearestPtTo(const vector<cv::Mat>& pts, const Mat pt){
    float dist = 10e6;
    int index = 0;

    for(int i=0; i<(int)pts.size();i++){
        if(cv::norm(pt, pts[i], NORM_L2) < dist){
            index = i;
            dist = cv::norm(pt, pts[i], NORM_L2);
        }
    }

    return index;
}

// ___________ END geometric tools___________________


// ___________ START actions on object of C++ _______
/**
 * \brief Give information to know if a special key "key" (decimal value of the ASCII table)
 * 		has been pressed during the delay "delay_us" in microsecond.
 * 		Return 1 if the specified key is pressed, 0 otherwise
 * \param int key
 * \param int delay_us
 * \return int
 */
int getKey(int key, int delay_us){
	if(waitKey(delay_us) == key){
		cout << "Special key: "<< (char)key<<" is pressed by user" << endl;
		return 1;
	}
	return 0;
}

void translateValVector(vector<cv::Mat>& v, int o){
    cv::Mat t;
    int s = (int) v.size();
    for(int i=0; i<o; i++){
        t = v[i];
        v[i] = v[(i + o)%s];
        v[(i + o)%s] = t;
    }
}

// ___________ END actions on object of C++ _______
