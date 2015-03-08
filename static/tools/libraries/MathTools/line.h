/*
 * line.h
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */


#include "error.h"

#include "vector.h"
#include "point.h"

#ifndef LIB_MATHTOOLS_LINE_H_
#define LIB_MATHTOOLS_LINE_H_

class Line {
    public:
        Line(float _a, float _b, float _c);
        ~Line();

        ERROR normLine();
        ERROR interLine2Line(const Line& l, int& nb, Point& pt) const;
        ERROR convPts2Line(const Point& p1, const Point& p2, bool& _norm);
        ERROR convVecPt2Line(const Vector& v, const Point& p, bool& _norm);
        ERROR distPt2Line(const Point& p, float& d, Point& h);
        ERROR symPtprLine(Point& p);

        // 2D line (ax+by+c=0)
        float a;
        float b;
        float c;
        bool norm;
};

#endif /* LIB_MATHTOOLS_LINE_H_ */
