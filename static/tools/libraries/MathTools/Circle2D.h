/*
 * Circle2D.h
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef LIB_GEOMETRYTOOLS_CIRCLE2D_H_
#define LIB_GEOMETRYTOOLS_CIRCLE2D_H_

#include "error.h"
#include "Line2D.h"
#include "Point2D.h"

class Circle2D {
    public:
        Circle2D(Point2D _c, float _r);
        Circle2D(float _x, float _y, float _r);
        ~Circle2D();

        ERROR interCircle2Line(const Line2D& l, int& nb, Point2D& pt1, Point2D& pt2) const;
        ERROR projPtOnCircle(Point2D& p) const;
        ERROR checkPtOnArc(const Point2D& p1, const Point2D& p2, Point2D& p, bool& ret) const;

        Point2D c;
        float r;
};

#endif /* LIB_GEOMETRYTOOLS_CIRCLE2D_H_ */
