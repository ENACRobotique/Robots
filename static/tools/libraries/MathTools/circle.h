/*
 * circle.h
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef LIB_MATHTOOLS_CIRCLE_H_
#define LIB_MATHTOOLS_CIRCLE_H_

#include "error.h"

#include "line.h"
#include "point.h"

class Circle {
    public:
        Circle(Point _c, float _r);
        Circle(float _x, float _y, float _r);
        ~Circle();

        ERROR interCircle2Line(const Line& l, int& nb, Point& pt1, Point& pt2) const;
        ERROR projPtOnCircle(Point& p) const;
        ERROR checkPtOnArc(const Point& p1, const Point& p2, Point& p, bool& ret) const;

        Point c;
        float r;
};

#endif /* LIB_MATHTOOLS_CIRCLE_H_ */
