/*
 * Point.h
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef LIB_MATHTOOLS_POINT_H_
#define LIB_MATHTOOLS_POINT_H_

#define SIGN(x) ((x > 0) - (x < 0))

#include "error.h"

class Point {
    public:
        Point();
        Point(float _x, float _y);
        ~Point();

        ERROR sqdistPt2Pt(const Point& p, float& d) const;
        ERROR distPt2Pt(const Point& p, float& d) const;
        ERROR equal(const Point& p, const float& tolerance, bool ret) const;
        ERROR translation(const float& _x, const float& _y);
        ERROR rotation(const float& a);

        float x;
        float y;
};

#endif /* LIB_MATHTOOLS_POINT_H_ */
