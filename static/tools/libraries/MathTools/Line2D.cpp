/*
 * Line2D.cpp
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#include "Line2D.h"

template<typename T>
Line2D<T>::Line2D(T _a, T _b, T _c) : a(_a), b(_b), c(_c), norm(false) {
}

template<typename T>
Line2D<T>::~Line2D() {
}

template<typename T>
ERROR Line2D<T>::normLine() {

    if (!norm) {
        Vector2D<T> nv(-b, a);
        T n;

        nv.normVec(n);

        RET_IF_(n == 0, ERR_DIV0);

        a = a / n;
        b = b / n;
        c = c / n;

        norm = true;
    }

    return 0;
}

template<typename T>
ERROR Line2D<T>::interLine2Line(const Line2D& l, int& nb, Point2D<T>& pt) const{
    T det;

    if (!(det = a * l.b - b * l.a)) { //parallel
        nb = 0;
    }
    else {
        pt.x = (1 / det) * (b * l.c - c * l.b);
        pt.y = (1 / det) * (c * l.a - a * l.c);
        nb = 1;
    }

    return 0;
}

template<typename T>
ERROR Line2D<T>::convPts2Line(const Point2D<T>& p1, const Point2D<T>& p2, bool& _norm){

    norm = _norm;

    if (!norm) {
        a = p2.y - p1.y;
        b = p1.x - p2.x;
    }
    else {
        T n;
        p1.distPt2Pt(p2, n);

        a = (p2.y - p1.y) / n;
        b = (p1.x - p2.x) / n;
    }

    c = -a * p1.x - b * p1.y;

    return 0;
}

template<typename T>
ERROR Line2D<T>::convVecPt2Line(const Vector2D<T>& v, const Point2D<T>& p, bool& _norm){

    norm = _norm;

    if (!norm) {
        a = v.y;
        b = -v.x;
    }
    else {
        T n;
        v.normVec(n);

        a = v.y / n;
        b = -v.x / n;
    }

    c = -a * p.x - b * p.y;

    return 0;
}

template<typename T>
ERROR Line2D<T>::distPt2Line(const Point2D<T>& p, T& d, Point2D<T>& h){

    this->normLine();

    d = a * p.x + b * p.y + c;

    h.x =  b * (b * p.x - a * p.y) - a * c;
    h.y = -a * (b * p.x - a * p.y) - b * c;


    return 0;
}

template<typename T>
ERROR Line2D<T>::symPtprLine(Point2D<T>& p){
    Point2D<T> pp(0,0), pc(p);
    T d;

    this->distPt2Line(p, d, pp);

    p.x = pc.x - 2 * (pc.x - pp.x);
    p.y = pc.y - 2 * (pc.y - pp.y);

    return 0;
}
