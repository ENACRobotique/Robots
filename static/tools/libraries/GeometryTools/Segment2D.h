/*
 * Segment2D.h
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef LIB_GEOMETRYTOOLS_SEGMENT2D_H_
#define LIB_GEOMETRYTOOLS_SEGMENT2D_H_

#include <Line2D.h>
#include <Point2D.h>
#include <Vector2D.h>
#include <cmath>
#include <iostream>

template<typename T>
class Segment2D {
public:

    Point2D<T> p1;
    Point2D<T> p2;

    Segment2D() :
            p1(0, 0), p2(0, 0) {
    }
    Segment2D(Point2D<T> _p1, Point2D<T> _p2) :
            p1(_p1), p2(_p2) {
    }
    ~Segment2D() {
    }

    Segment2D& operator=(const Segment2D& s) {
        p1 = s.p1;
        p2 = s.p2;
        return *this;
    }

    bool operator==(Segment2D& s) {
        return s.p1 == p1 && s.p2 == p2;
    }
    bool operator!=(Segment2D& l) {
        return !(*this == l);
    }

    Point2D<T> projecte(const Point2D<T>& p) const { //projection on the segment or shortest extremity point
        Vector2D<T> p1p(p1, p), p1p2(p1, p2);
        T t = (p1p * p1p2) / length();

        if (t < 0. || length() == 0)
            return p1;
        else if (t > length())
            return p2;

        Line2D<T> l(p1, p2);
        return l.project(p);
    }
    T squareDistance(const Point2D<T>& p) const {
        return p.distanceSqTo(projecte(p));
    }
    T distance(const Point2D<T>& p) const {
        return sqrt(squareDistance(p));
    }
    T length() const {
        return p1.distanceTo(p2);
    }
};

template<typename T>
std::ostream& operator<<(std::ostream& out, Segment2D<T>& s) {
    out << "[" << s.p1 << ";" << s.p2 << "]";
    return out;
}

#endif /* LIB_GEOMETRYTOOLS_SEGMENT2D_H_ */
