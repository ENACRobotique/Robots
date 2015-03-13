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

        T sqdistPt2Pt(const Point2D<T>& p) const;
        T distPt2Pt(const Point2D<T>& p) const;
        void translation(const T& _x, const T& _y);
        void rotation(const T& a);

        T x;
        T y;
};

#endif /* LIB_GEOMETRYTOOLS_POIN2D_H_ */
