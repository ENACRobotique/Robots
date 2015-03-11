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

template<typename T>
class Circle2D {
    public:
        Circle2D(Point2D<T> _c, T _r);
        Circle2D(T _x, T _y, T _r);
        ~Circle2D();

        ERROR interCircle2Line(const Line2D<T>& l, int& nb, Point2D<T>& pt1, Point2D<T>& pt2) const;
        ERROR projPtOnCircle(Point2D<T>& p) const;
        ERROR checkPtOnArc(const Point2D<T>& p1, const Point2D<T>& p2, Point2D<T>& p, bool& ret) const;

        Point2D<T> c;
        T r;
};

#endif /* LIB_GEOMETRYTOOLS_CIRCLE2D_H_ */
