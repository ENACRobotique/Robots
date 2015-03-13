/*
 * Segment2D.cpp
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#include "Segment2D.h"

template<typename T>
Segment2D<T>::Segment2D() : p1(0,0), p2(0,0){
}

template<typename T>
Segment2D<T>::Segment2D(Point2D<T> _p1, Point2D<T> _p2) : p1(_p1), p2(_p2){
}

template<typename T>
Segment2D<T>::~Segment2D() {
}

template<typename T>
void Segment2D<T>::convPts2Seg(const Point2D<T>& a, const Point2D<T>& b) {

    p1 = a;
    p2 = b;

    return 0;
}

template<typename T>
void Segment2D<T>::sqdistPt2Seg(const Point2D<T>& p, float& d, Point2D<T>& h) const{
    float l2, t;
    Vector2D<T> p1p(p1, p), p1p2(p1, p2);

    p1.sqdistPt2Pt(p2, l2);
    if (l2 == 0.){
        return p.sqdistPt2Pt(p1, d);
    }

    p1p.dotVecs(p1p2, t);

    if (t < 0.){
        h = p1;
        p.sqdistPt2Pt(p1, d);
        return;
    }

    if (t > l2) {
        h = p2;
        p.sqdistPt2Pt(p2, d);
        return;
    }

    t = t / l2;

    h.x = p1.x + t * p1p2.x;
    h.y = p1.y + t * p1p2.y;
}
