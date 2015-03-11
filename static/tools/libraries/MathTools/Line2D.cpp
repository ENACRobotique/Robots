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
void Line2D<T>::normLine() {

    if (!norm) {
        Vector2D<T> nv(-b, a);
        T n;

        nv.normVec(n);

        //RET_IF_(n == 0, ERR_DIV0); FIXME

        a = a / n;
        b = b / n;
        c = c / n;

        norm = true;
    }
}

template<typename T>
Point2D<T> Line2D<T>::interLine2Line(const Line2D& l) const{
    T det;
    Point2D<T> p;

    if (!(det = a * l.b - b * l.a)) { //parallel
        //nb = 0; FIXME
    }
    else {
        p.x = (1 / det) * (b * l.c - c * l.b);
        p.y = (1 / det) * (c * l.a - a * l.c);
        //nb = 1; FIXME
    }

    return p;
}

template<typename T>
void Line2D<T>::convPts2Line(const Point2D<T>& p1, const Point2D<T>& p2, bool& _norm){

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
}

template<typename T>
void Line2D<T>::convVecPt2Line(const Vector2D<T>& v, const Point2D<T>& p, bool& _norm){

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
}

template<typename T>
T Line2D<T>::distPt2Line(const Point2D<T>& p, Point2D<T>& h){
    T d;

    this->normLine();

    d = a * p.x + b * p.y + c;

    h.x =  b * (b * p.x - a * p.y) - a * c;
    h.y = -a * (b * p.x - a * p.y) - b * c;


    return d;
}

template<typename T>
Point2D<T> Line2D<T>::symPtprLine(Point2D<T>& p){
    Point2D<T> pp(0,0), pc(p);
    Point2D<T> pt;
    T d;

    this->distPt2Line(p, pp);

    pt.x = pc.x - 2 * (pc.x - pp.x);
    pt.y = pc.y - 2 * (pc.y - pp.y);

    return pt;
}
