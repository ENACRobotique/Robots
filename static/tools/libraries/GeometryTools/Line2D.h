/*
 * Line2D.h
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef LIB_GEOMETRYTOOLS_LINE2D_H_
#define LIB_GEOMETRYTOOLS_LINE2D_H_

#include <Point2D.h>
#include <Vector2D.h>
#include <iostream>

template<typename T>
class Line2D {
protected:
    // 2D line (ax+by+c=0)
    T _a;
    T _b;
    T _c;
    bool norm;

public:
    Line2D() :
            _a(0), _b(0), _c(0), norm(false) {
    }
    Line2D(const Line2D& l) :
            _a(l._a), _b(l._b), _c(l._c), norm(l.norm) {
    }
    Line2D(const T _a, const T _b, const T _c) :
            _a(_a), _b(_b), _c(_c), norm(false) {
    }
    Line2D(const Vector2D<T>& v) :
            _a(v.y), _b(-v.x), _c(0), norm(false) {
    }
    Line2D(const Point2D<T>& p, const Vector2D<T>& v) :
            _a(v.y), _b(-v.x), _c(-_a * p.x - _b * p.y), norm(false) {
    }
    Line2D(const Point2D<T>& p1, const Point2D<T>& p2) :
            _a(p2.y - p1.y), _b(p1.x - p2.x), _c(-_a * p1.x - _b * p1.y), norm(false) {
    }
    ~Line2D() {
    }

    const T& a() const {
        return _a;
    }
    const T& b() const {
        return _b;
    }
    const T& c() const {
        return _c;
    }

    Line2D& operator=(const Line2D& l) {
        _a = l._a;
        _b = l._b;
        _c = l._c;
        norm = l.norm;
        return *this;
    }

    bool operator==(Line2D& l) {
        l.normalize();
        normalize();
        return l._a == _a && l._b == _b && l._c == _c;
    }
    bool operator!=(Line2D& l) {
        return !(*this == l);
    }

    Line2D& normalize() {
        if (!norm) {
            Vector2D<T> nv(-_b, _a);
            T n = nv.norm();
            _a = _a / n;
            _b = _b / n;
            _c = _c / n;
            norm = true;
        }
        return *this;
    }
    Point2D<T> intersection(const Line2D& l) const {
        T det;

        if (!(det = _a * l._b - _b * l._a)) { //parallel
            return 0; // FIXME
        }
        Point2D<T> p;
        p.x = (1 / det) * (_b * l._c - _c * l._b);
        p.y = (1 / det) * (_c * l._a - _a * l._c);

        return p;
    }
    T distanceTo(const Point2D<T>& p) {
        normalize();
        return _a * p.x + _b * p.y + _c;
    }
    Point2D<T> project(const Point2D<T>& p) {
        Point2D<T> pr;

        normalize();
        pr.x = _b * (_b * p.x - _a * p.y) - _a * _c;
        pr.y = -_a * (_b * p.x - _a * p.y) - _b * _c;

        return pr;
    }
    Point2D<T> symetry(const Point2D<T>& p) {
        return p + T(2) * (project(p) - p);
    }
};

template<typename T>
std::ostream& operator<<(std::ostream& out, Line2D<T>& l) {
    out << l._a << "*x + " << l._b << "*y + " << l._c << " = 0";
    return out;
}

#endif /* LIB_GEOMETRYTOOLS_LINE2D_H_ */
