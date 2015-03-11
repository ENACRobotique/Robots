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

class Point2D {
    public:
        Point2D();
        Point2D(float _x, float _y);
        ~Point2D();

        ERROR sqdistPt2Pt(const Point2D& p, float& d) const;
        ERROR distPt2Pt(const Point2D& p, float& d) const;
        ERROR equal(const Point2D& p, const float& tolerance, bool ret) const;
        ERROR translation(const float& _x, const float& _y);
        ERROR rotation(const float& a);

        float x;
        float y;
};

#endif /* LIB_GEOMETRYTOOLS_POIN2D_H_ */
