/*
 * segment.h
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef LIB_MATHTOOLS_SEGMENT_H_
#define LIB_MATHTOOLS_SEGMENT_H_

#include "error.h"

#include "point.h"
#include "vector.h"

class Segment {
    public:
        Segment();
        Segment(Point _p1, Point _p2);
        ~Segment();

        ERROR convPts2Seg(const Point& a, const Point& b);
        ERROR sqdistPt2Seg(const Point& p, float& d, Point& h) const;

        Point p1;
        Point p2;
};

#endif /* LIB_MATHTOOLS_SEGMENT_H_ */
