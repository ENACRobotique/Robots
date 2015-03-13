/*
 * Circle2D.h
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef LIB_GEOMETRYTOOLS_CIRCLE2D_H_
#define LIB_GEOMETRYTOOLS_CIRCLE2D_H_

#include <utility>

#include "error.h"
#include "Line2D.h"
#include "Point2D.h"

template<typename T>
class Circle2D {
    public:
        Circle2D(Point2D<T> _c, T _r);
        Circle2D(T _x, T _y, T _r);
        ~Circle2D();

        std::pair<Point2D<T>, Point2D<T>> interCircle2Line(const Line2D<T>& l) const;
        Point2D<T> projPtOnCircle(Point2D<T>& p) const;
        bool checkPtOnArc(const Point2D<T>& p1, const Point2D<T>& p2, Point2D<T>& p) const;

        Point2D<T> c;
        T r;
};

#endif /* LIB_GEOMETRYTOOLS_CIRCLE2D_H_ */
