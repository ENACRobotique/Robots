/*
 * Point2D.h
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef LIB_GEOMETRYTOOLS_POINT2D_H_
#define LIB_GEOMETRYTOOLS_POINT2D_H_

#define SIGN(x) ((x > 0) - (x < 0))

#include "error.h"

template<typename T>
class Point2D {
    public:
        Point2D();
        Point2D(T _x, T _y);
        ~Point2D();

        ERROR sqdistPt2Pt(const Point2D& p, T& d) const;
        ERROR distPt2Pt(const Point2D& p, T& d) const;
        ERROR equal(const Point2D& p, const T& tolerance, bool ret) const;
        ERROR translation(const T& _x, const T& _y);
        ERROR rotation(const T& a);

        T x;
        T y;
};

#endif /* LIB_GEOMETRYTOOLS_POIN2D_H_ */
