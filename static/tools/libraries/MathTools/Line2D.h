/*
 * Line2D.h
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef LIB_GEOMETRYTOOLS_LINE2D_H_
#define LIB_GEOMETRYTOOLS_LINE2D_H_

#include "error.h"
#include "Point2D.h"
#include "Vector2D.h"

template<typename T>
class Line2D {
    public:
        Line2D(T _a, T _b, T _c);
        ~Line2D();

        ERROR normLine();
        ERROR interLine2Line(const Line2D& l, int& nb, Point2D<T>& pt) const;
        ERROR convPts2Line(const Point2D<T>& p1, const Point2D<T>& p2, bool& _norm);
        ERROR convVecPt2Line(const Vector2D<T>& v, const Point2D<T>& p, bool& _norm);
        ERROR distPt2Line(const Point2D<T>& p, T& d, Point2D<T>& h);
        ERROR symPtprLine(Point2D<T>& p);

        // 2D line (ax+by+c=0)
        T a;
        T b;
        T c;
        bool norm;
};

#endif /* LIB_GEOMETRYTOOLS_LINE2D_H_ */
