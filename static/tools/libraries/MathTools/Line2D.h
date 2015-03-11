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

class Line2D {
    public:
        Line2D(float _a, float _b, float _c);
        ~Line2D();

        ERROR normLine();
        ERROR interLine2Line(const Line2D& l, int& nb, Point2D& pt) const;
        ERROR convPts2Line(const Point2D& p1, const Point2D& p2, bool& _norm);
        ERROR convVecPt2Line(const Vector2D& v, const Point2D& p, bool& _norm);
        ERROR distPt2Line(const Point2D& p, float& d, Point2D& h);
        ERROR symPtprLine(Point2D& p);

        // 2D line (ax+by+c=0)
        float a;
        float b;
        float c;
        bool norm;
};

#endif /* LIB_GEOMETRYTOOLS_LINE2D_H_ */
