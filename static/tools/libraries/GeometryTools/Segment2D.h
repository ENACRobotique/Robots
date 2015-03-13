/*
 * Segment2D.h
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef LIB_GEOMETRYTOOLS_SEGMENT2D_H_
#define LIB_GEOMETRYTOOLS_SEGMENT2D_H_

#include "error.h"
#include "Point2D.h"
#include "Vector2D.h"

template<typename T>
class Segment2D {
    public:
        Segment2D();
        Segment2D(Point2D<T> _p1, Point2D<T> _p2);
        ~Segment2D();

        void convPts2Seg(const Point2D<T>& a, const Point2D<T>& b);
        void sqdistPt2Seg(const Point2D<T>& p, float& d, Point2D<T>& h) const;

        Point2D<T> p1;
        Point2D<T> p2;
};

#endif /* LIB_GEOMETRYTOOLS_SEGMENT2D_H_ */
