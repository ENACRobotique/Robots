/*
 * Segment2D.cpp
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#include "Segment2D.h"

Segment2D::Segment2D() : p1(0,0), p2(0,0){
}

Segment2D::Segment2D(Point2D _p1, Point2D _p2) : p1(_p1), p2(_p2){
}

Segment2D::~Segment2D() {
}

ERROR Segment2D::convPts2Seg(const Point2D& a, const Point2D& b) {

    p1 = a;
    p2 = b;

    return 0;
}

ERROR Segment2D::sqdistPt2Seg(const Point2D& p, float& d, Point2D& h) const{
    float l2, t;
    Vector2D p1p(p1, p), p1p2(p1, p2);

    p1.sqdistPt2Pt(p2, l2);
    if (l2 == 0.){
        return p.sqdistPt2Pt(p1, d);
    }

    p1p.dotVecs(p1p2, t);

    if (t < 0.){
        h = p1;
        return p.sqdistPt2Pt(p1, d);
    }

    if (t > l2) {
        h = p2;
        return p.sqdistPt2Pt(p2, d);
    }

    t = t / l2;

    h.x = p1.x + t * p1p2.x;
    h.y = p1.y + t * p1p2.y;

    return p.sqdistPt2Pt(h, d);
}
